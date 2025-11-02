#include "FujiNet.hh"
#include "Timer.hh"

#include "GlobalSettings.hh"
#include "serialize.hh"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <stdio.h>

#define FUJINET_DEFAULT_PORT     65504

namespace openmsx {

static constexpr size_t MAX_BUF_LEN     = 2 * 1024;
static constexpr size_t IO_GETC_ADDR    = 0xBFFC;
static constexpr size_t IO_STATUS_ADDR  = 0xBFFD;
static constexpr size_t IO_PUTC_ADDR    = 0xBFFE;
static constexpr size_t IO_CONTROL_ADDR = 0xBFFF;

FujiNet::FujiNet(DeviceConfig& config)
    : MSXDevice(config)
    , rom(getName() + " ROM", "rom", config)
{
    thread = std::thread(&FujiNet::readSocket, this);
    stopReading = false;
}

FujiNet::~FujiNet()
{
    stopReading = true;
    close();

    if (thread.joinable()) {
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
    getCliComm().printInfo("FujiNet: Start read loop");
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

		auto n = sock_recv(sock, &buf[0], MAX_BUF_LEN);
		if (n < 0) { // error
			close();
			continue;
		}
		else if (n > 0) {
            getCliComm().printInfo("FujiNet: Read ", n, " bytes from pty");
            std::string str(buf, n);
            getCliComm().printInfo(str);
            std::lock_guard lock(mtx);
            for (auto i : xrange(std::min<size_t>(n, MAX_BUF_LEN - rxBuffer.size()))) {
                rxBuffer.push_back(buf[i]);
            }
        }
    }
}

void FujiNet::reset(EmuTime time)
{
}

uint8_t FujiNet::readMem(uint16_t address, EmuTime time)
{
    std::lock_guard lock(mtx);
    // getCliComm().printInfo("FujiNet: readMem() ", address);
    switch (address) {
        case IO_GETC_ADDR:
            if (!rxBuffer.empty()) {
                uint8_t value = rxBuffer.pop_front();

                char formatted[16];
                if (value > 31 && value < 127)
                    sprintf(formatted, "$%02X %c", value, value);
                else
                    sprintf(formatted, "$%02X", value);
                getCliComm().printInfo("FujiNet: GETC -> ", formatted);

				return value;
			}
			else {
			    getCliComm().printInfo("FujiNet: GETC -> empty!");
			}
            return 0x00;
        case IO_STATUS_ADDR:
            if (!rxBuffer.empty()) {
                // getCliComm().printInfo("FujiNet: STAT -> data available");
                return 0b10000000; // data available
			}
			// else {
    			// getCliComm().printInfo("FujiNet: STAT -> no data");
			// }
            return 0x00;
        default:
            if (0x4000 <= address && address < 0xC000) {
          		return rom[address - 0x4000];
           	}
           	return 0xFF;
    }
}

uint8_t FujiNet::peekMem(uint16_t address, EmuTime time) const
{
    std::lock_guard lock(mtx);
    // getCliComm().printInfo("FujiNet: peekMem() ", address);
    switch (address) {
        case IO_GETC_ADDR:
        case IO_STATUS_ADDR:
            if (!rxBuffer.empty()) {
                return 0b10000000; // data available
			}
            return 0x00;
        default:
            break;
    }
    return 0x00;
}

void FujiNet::writeMem(uint16_t address, uint8_t value, EmuTime time)
{
    std::lock_guard lock(mtx);
    // getCliComm().printInfo("FujiNet: writeMem() ", address, " ", value);
    switch (address) {
        case IO_PUTC_ADDR: // IO_PUTC
            if (sock != OPENMSX_INVALID_SOCKET) {
                char formatted[16];
                if (value > 31 && value < 127)
                    sprintf(formatted, "$%02X %c", value, value);
                else
                    sprintf(formatted, "$%02X", value);
                getCliComm().printInfo("FujiNet: PUTC ", formatted);

                auto res = sock_send(sock, reinterpret_cast<const char*>(&value), 1);
                (void)res; // ignore error
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
