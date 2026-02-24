#undef UNUSED
#include "FujiNet.hh"
#include "FujiBusPacket.h"
#include "Timer.hh"

#include "GlobalSettings.hh"
#include "fujiDeviceID.h"
#include "serialize.hh"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <stdio.h>

#define FUJINET_DEFAULT_PORT     1985

namespace openmsx {

static constexpr size_t MAX_BUF_LEN     = 2 * 1024;
static constexpr size_t IO_GETC_ADDR    = 0xBFFC;
static constexpr size_t IO_STATUS_ADDR  = 0xBFFD;
static constexpr size_t IO_PUTC_ADDR    = 0xBFFE;
static constexpr size_t IO_CONTROL_ADDR = 0xBFFF;

FujiNet::FujiNet(DeviceConfig& config)
    : MSXDevice(config)
    , rom(getName() + " ROM", "rom", config)
    , userRom(0)
    , debugMode(
        getCommandController(), "fujinet_debug",
        "Enable FujiNet debug logging", false)
{
    thread = std::thread(&FujiNet::readSocket, this);
    stopReading = false;
    userRomEnabled = false;
    userRomLoaded = false;
}

FujiNet::~FujiNet()
{
    stopReading = true;
    close();

    if (thread.joinable()) {
        poller.abort();
        thread.join();
    }
}

void FujiNet::close()
{
	auto oldSock = sock.exchange(OPENMSX_INVALID_SOCKET);
	if (oldSock != OPENMSX_INVALID_SOCKET) {
		sock_close(oldSock);
	}
}

void FujiNet::readSocket()
{
    if (debugMode.getBoolean()) {
        getCliComm().printInfo("FujiNet: Start read loop");
    }
    char buf[MAX_BUF_LEN];

    while (!stopReading) {
        if (sock == OPENMSX_INVALID_SOCKET) {
			sock = socket(AF_INET, SOCK_STREAM, 0);
			if (sock == OPENMSX_INVALID_SOCKET) {
				Timer::sleep(1'000'000); // retry once per second
				continue;
			}

			sockaddr_in addr{};
			addr.sin_family = AF_INET;
			addr.sin_port = htons(FUJINET_DEFAULT_PORT);
			addr.sin_addr.s_addr =
			        htonl(INADDR_LOOPBACK); // 127.0.0.1
			if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
				close();
				Timer::sleep(1'000'000); // retry once per second
				continue;
			}
		}
#ifndef _WIN32
		if (poller.poll(sock)) {
			continue; // error or abort
		}
#endif

		auto n = sock_recv(sock, &buf[0], MAX_BUF_LEN);
		if (n < 0) { // error
			close();
			continue;
		}
		else if (n > 0) {
            if (debugMode.getBoolean()) {
                getCliComm().printInfo("FujiNet: Read ", n, " bytes from pty");
                std::string str(buf, n);
                getCliComm().printInfo(str);
            }
            std::lock_guard lock(mtx);
            for (auto i : xrange(std::min<size_t>(n, MAX_BUF_LEN - rxBuffer.size()))) {
                rxBuffer.push_back(buf[i]);
            }

            auto packet = readBusPacket();
            if (!packet) {
                continue;
            }

            if (packet->device() == FUJI_DEVICEID_DBC) {
                handleDBCCommand(std::move(packet));
            }
        }
    }
}

void FujiNet::handleDBCCommand(std::unique_ptr<FujiBusPacket> packet)
{
    // Don't pass DCB commands to MSX
    rxBuffer.clear();

    switch (packet->command()) {
        case FUJICMD_OPEN:
            if (debugMode.getBoolean()) {
                getCliComm().printInfo("FUJICMD_OPEN");
            }
            clearUserROM();
            fujiBusAck();
            break;
        case FUJICMD_WRITE:
            if (debugMode.getBoolean()) {
                getCliComm().printInfo("FUJICMD_WRITE");
            }
            if (packet->data())
                writeUserROM(*(packet->data()));
            fujiBusAck();
            break;
        case FUJICMD_CLOSE:
            if (debugMode.getBoolean()) {
                getCliComm().printInfo("FUJICMD_CLOSE");
            }
            if (userRom.size())
                readyUserROM();
            fujiBusAck();
            break;
        default:
            return;
    }
}

std::unique_ptr<FujiBusPacket> FujiNet::readBusPacket()
{
    ByteBuffer packet;
    for (auto c : rxBuffer) {
        packet.push_back(c);
    }
    return FujiBusPacket::fromSerialized(packet);
}

void FujiNet::fujiBusAck()
{
    if (sock != OPENMSX_INVALID_SOCKET) {
        FujiBusPacket packet(FUJI_DEVICEID_DBC, FUJICMD_ACK);
        auto buf = packet.serialize();
        auto res = sock_send(sock, (const char*)buf.data(), buf.size());
        (void)res; // ignore error
    }
}

void FujiNet::clearUserROM()
{
    userRomLoaded = false;
    userRom.clear();
}

void FujiNet::writeUserROM(std::span<unsigned const char> data)
{
    for (auto c : data) {
        userRom.push_back(c);
    }
}

void FujiNet::readyUserROM()
{
    userRomLoaded = true;
}

void FujiNet::enableUserROM()
{
    userRomEnabled = true;
}

void FujiNet::disableUserROM()
{
    userRomEnabled = false;
}

void FujiNet::reset(EmuTime /*time*/)
{
    disableUserROM();
}

uint8_t FujiNet::readMem(uint16_t address, EmuTime time)
{
    if (debugMode.getBoolean()) {
        getCliComm().printInfo("FujiNet: readMem() ", address);
    }
	auto value = peekMem(address, time);
	switch (address) {
		case IO_GETC_ADDR:
			if (!rxBuffer.empty()) {
				std::lock_guard lock(mtx);
				rxBuffer.pop_front();
                if (debugMode.getBoolean()) {
                    char formatted[16];
                    if (value > 31 && value < 127) {
                        sprintf(formatted, "$%02X %c", value, value);
                    } else {
                        sprintf(formatted, "$%02X", value);
                    }
                    getCliComm().printInfo("FujiNet: GETC -> ", formatted);
                }
			} else {
                if (debugMode.getBoolean()) {
                    getCliComm().printInfo("FujiNet: GETC -> empty!");
                }
			}
			break;
		case IO_STATUS_ADDR:
            if (debugMode.getBoolean()) {
                if (value == 0b10000000) {
                    getCliComm().printInfo("FujiNet: STAT -> data available");
                } else {
                    getCliComm().printInfo("FujiNet: STAT -> no data");
                }
            }
            ;
	}
	return value;
}

uint8_t FujiNet::peekMem(uint16_t address, EmuTime /*time*/) const
{
    if (debugMode.getBoolean()) {
        getCliComm().printInfo("FujiNet: peekMem() ", address);
    }
	switch (address) {
		case IO_GETC_ADDR: {
			std::lock_guard lock(mtx);
			if (!rxBuffer.empty()) {
				return rxBuffer.front();
			}
			return 0x00;
		}
		case IO_STATUS_ADDR: {
            // bit 7 when 1 means data available
            // bit 6 when 1 means user rom is ready
            // bit 6 when 0 means user rom is not ready
			std::lock_guard lock(mtx);
			uint8_t status = (userRomLoaded ? 0x40 : 0x00);
			if (!rxBuffer.empty()) {
				status |= 0x80;
			}
			return status;
		}
		default:
			if (address < 0x4000 || 0xC000 <= address) return 0xFF;
			return userRomEnabled ? userRom[address - 0x4000] : rom[address - 0x4000];
	}
}

void FujiNet::writeMem(uint16_t address, uint8_t value, EmuTime /*time*/)
{
    std::lock_guard lock(mtx);
    // getCliComm().printInfo("FujiNet: writeMem() ", address, " ", value);
    switch (address) {
        case IO_PUTC_ADDR: // IO_PUTC
            if (sock != OPENMSX_INVALID_SOCKET) {
                if (debugMode.getBoolean()) {
                    char formatted[16];
                    if (value > 31 && value < 127)
                        sprintf(formatted, "$%02X %c", value, value);
                    else
                        sprintf(formatted, "$%02X", value);
                    getCliComm().printInfo("FujiNet: PUTC ", formatted);
                }

                // txBuffer.push_back(value);
                // if (value == SLIP_END) {
                //     if (inSLIPPacket) {
                //         inSLIPPacket = false;

                //         std::string packet = "";
                //         for (auto c : txBuffer) {
                //             packet.push_back(c);
                //         }
                //         auto tempFrame = FujiBusPacket::fromSerialized(packet);
                //         if (tempFrame) {
                //             char formatted[32];
                //             sprintf(formatted, "\nCF: dev:%02x cmd:%02x dlen:%d\n",
                //                         tempFrame->device(), tempFrame->command(),
                //                         tempFrame->data() ? tempFrame->data()->size() : -1);
                //             getCliComm().printInfo(formatted);
                //         }

                //         txBuffer.clear();
                //     }
                //     else {
                //         inSLIPPacket = true;
                //     }
                // }

                auto res = sock_send(sock, reinterpret_cast<const char*>(&value), 1);
                (void)res; // ignore error
            }
            return;
        case IO_CONTROL_ADDR:
            // when writing if bit 7 is high that signals "switch rom to bank"
            // bits 6~1 is reserved and must be 0
            // when bit 0 is 1 that means switching to user rom 
            // when bit 0 is 0 that means switching to config rom
            if (value == 0x81) {
                enableUserROM();
            } else if (value == 0x80){
                disableUserROM();
            }
            return;
        default:
            break;
    }
}

template<typename Archive>
void FujiNet::serialize(Archive& ar, unsigned /*version*/)
{
	ar.template serializeBase<MSXDevice>(*this);
}

INSTANTIATE_SERIALIZE_METHODS(FujiNet);
REGISTER_MSXDEVICE(FujiNet, "FujiNet");

} // namespace openmsx
