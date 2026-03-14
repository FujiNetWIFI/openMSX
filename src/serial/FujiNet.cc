#undef UNUSED
#include "FujiNet.hh"
#include "FujiBusPacket.h"
#include "Timer.hh"

#include "GlobalSettings.hh"
#include "fujiDeviceID.h"
#include "fujiRomType.h"
#include "serialize.hh"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#ifdef _WIN32
#include <fcntl.h>
#else
#include <sys/fcntl.h>
#endif
#include <stdio.h>

#define FUJINET_DEFAULT_PORT     1985

namespace openmsx {

// static constexpr size_t MAX_BUF_LEN     = 2 * 1024;
static constexpr size_t MAX_BUF_LEN     = 0x40000;
static constexpr size_t IO_GETC_ADDR    = 0xBFFC;
static constexpr size_t IO_STATUS_ADDR  = 0xBFFD;
static constexpr size_t IO_PUTC_ADDR    = 0xBFFE;
static constexpr size_t IO_CONTROL_ADDR = 0xBFFF;
static constexpr size_t IO_FLAG_USERROM_READY   = 0x40;
static constexpr size_t IO_FLAG_ROM_MODE_CMD    = 0b00000100;
static constexpr size_t IO_FLAG_USERROM_ENABLE  = 0b00000001;

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
    userRomType = FUJI_ROM_MSX_PLAIN;
    userRomBankSize = 0x4000;
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

template <typename... Args>
void FujiNet::fnDebugLog(Args&&... args) {
    if (debugMode.getBoolean()) {
        getCliComm().printInfo(std::forward<Args>(args)...);
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
    fnDebugLog("FujiNet: Start read loop");
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
                fnDebugLog("FujiNet: Read ", n, " bytes from pty");
                std::string str(buf, n);
                fnDebugLog(str);
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
            fnDebugLog("FUJICMD_OPEN");
            clearUserROM();
            // TODO: Also set offset from first param
            fnDebugLog("fujiROMType: ", packet->param(1));
            setUserROMType((fujiROMType_t)packet->param(1));
            fujiBusAck();
            break;
        case FUJICMD_WRITE:
            fnDebugLog("FUJICMD_WRITE");
            if (packet->data())
                writeUserROM(*(packet->data()));
            fujiBusAck();
            break;
        case FUJICMD_CLOSE:
            fnDebugLog("FUJICMD_CLOSE");
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
    fnDebugLog("FujiNet: readyUserROM");
    userRomLoaded = true;
}

void FujiNet::enableUserROM()
{
    fnDebugLog("FujiNet: enabledUserROM");
    userRomEnabled = true;
}

void FujiNet::disableUserROM()
{
    fnDebugLog("FujiNet: disableUserROM");
    userRomEnabled = false;
}

void FujiNet::setUserROMType(fujiROMType_t t)
{
    userRomType = t;

    switch (userRomType) {
        case FUJI_ROM_MSX_ASCII8:
        case FUJI_ROM_MSX_KONAMI:
        case FUJI_ROM_MSX_KONAMI_SCC:
            userRomBankSize = 0x2000;
            break;
        default:
            userRomBankSize = 0x4000;
            break;
    }

    // Reset all banks to sequential
    for (uint8_t n = 0; n < MAX_BANKS; n++) {
        setUserROMBank(n, n);
    }
}

void FujiNet::setUserROMBank(uint8_t n, uint8_t block)
{
    uint32_t offset = block * userRomBankSize;
    userRomMap[n] = offset;
    char offset_str[8];
    snprintf(offset_str, 8, "%04X", offset);
    fnDebugLog("FujiNet: setUserROMBank n:", n, " block:", block, " offset:", offset_str);
}

void FujiNet::handleBankSwitch(uint16_t address, uint8_t value)
{
    char addr_str[8];
    snprintf(addr_str, 8, "%04X", address);
    fnDebugLog("FujiNet: handleBankSwitch addr:", addr_str, " val:", value, " userRomType:", (uint8_t)userRomType);

    if (address < 0x4000 || address >= 0xC000)
        return;

    switch (userRomType) {
        case FUJI_ROM_MSX_ASCII8:
            if ((0x6000 <= address) && (address < 0x8000)) {
                uint8_t bank = ((address >> 11) & 3);
                setUserROMBank(bank, value);
            }
            break;

        case FUJI_ROM_MSX_ASCII16:
            if ((0x6000 <= address) && (address < 0x7800) && !(address & 0x0800)) {
          		uint8_t bank = ((address >> 12) & 1);
                setUserROMBank(bank, value);
           	}
            break;

        case FUJI_ROM_MSX_KONAMI:
            // Note: [0x4000..0x6000) is fixed at segment 0.
           	if (0x6000 <= address && address < 0xC000) {
                uint8_t bank = (address >> 13) - 2;
          		setUserROMBank(bank, value);
           	}
            break;

        case FUJI_ROM_MSX_KONAMI_SCC: // Konami+SCC; TODO: use real enum
           	if (0x5000 <= address && address < 0xC000 && (address & 0x1800) == 0x1000) {
          		uint8_t bank = (address >> 13) - 2;
          		setUserROMBank(bank, value);
          		// TODO: if bank = 4 clear SCC cache
           	}
            break;

        default:
            break;
    }
}

void FujiNet::reset(EmuTime /*time*/)
{
    disableUserROM();
}

uint8_t FujiNet::readMem(uint16_t address, EmuTime time)
{
    if (debugMode.getBoolean()) {
        // getCliComm().printInfo("FujiNet: readMem() ", address);
    }

    if (userRomEnabled) {
        return peekUserROM(address);
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
                        snprintf(formatted, 16, "$%02X %c", value, value);
                    } else {
                        snprintf(formatted, 16, "$%02X", value);
                    }
                    fnDebugLog("FujiNet: GETC -> ", formatted);
                }
			} else {
    			fnDebugLog("FujiNet: GETC -> empty!");
			}
			break;
		case IO_STATUS_ADDR:
            if (value == 0b10000000) {
                fnDebugLog("FujiNet: STAT -> data available");
            } else {
                fnDebugLog("FujiNet: STAT -> no data");
            }
            break;
	}
	return value;
}

uint8_t FujiNet::peekMem(uint16_t address, EmuTime /*time*/) const
{
    // fnDebugLog("FujiNet: peekMem() ", address);

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
			if (address < 0x4000 || 0xC000 <= address)
                return 0xFF;
		    return rom[address - 0x4000];

	}
}

uint8_t FujiNet::peekUserROM(uint16_t address)
{
    switch (userRomType) {
        case FUJI_ROM_MSX_KONAMI:
            // [0x0000, 0x4000) mirrors [0x4000, 0x8000)
            if (address < 0x4000) address += 0x4000;
            // [0xC000, 0x10000) mirrors [0x8000, 0xC000)
            else if (address >= 0xC000) address -= 0x4000;
            break;

        case FUJI_ROM_MSX_KONAMI_SCC:
            // [0x0000, 0x4000) mirrors [0xC000, 0x10000)
            if (address < 0x4000) address += 0x8000;
            // [0xC000, 0x10000) mirrors [0x4000, 0x8000)
            else if (address >= 0xC000) address -= 0x8000;
            break;

        default:
            break;
    }

    uint16_t bank = (address - 0x4000) / userRomBankSize; // TODO: validate bank
    uint32_t offset = userRomMap[bank];

    return userRom[offset + address - 0x4000 - bank * userRomBankSize];
}

void FujiNet::writeMem(uint16_t address, uint8_t value, EmuTime /*time*/)
{
    std::lock_guard lock(mtx);
    switch (address) {
        case IO_PUTC_ADDR: // IO_PUTC
            if (sock != OPENMSX_INVALID_SOCKET) {
                if (debugMode.getBoolean()) {
                    char formatted[16];
                    if (value > 31 && value < 127)
                        snprintf(formatted, 16, "$%02X %c", value, value);
                    else
                        snprintf(formatted, 16, "$%02X", value);
                    fnDebugLog("FujiNet: PUTC ", formatted);
                }

                auto res = sock_send(sock, reinterpret_cast<const char*>(&value), 1);
                (void)res; // ignore error
            }
            return;
        case IO_CONTROL_ADDR:
            if (value & IO_FLAG_ROM_MODE_CMD) {
                if (value & IO_FLAG_USERROM_ENABLE)
                    enableUserROM();
                else
                    disableUserROM();
            }
            return;
        default:
            handleBankSwitch(address, value);
            return;
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
