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
#include <netinet/tcp.h>
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
static constexpr size_t IO_FLAG_DATA_AVAIL          = 0x80;
static constexpr size_t IO_FLAG_USERROM_READY       = 0x40;
static constexpr size_t IO_FLAG_ROM_MODE_CMD        = 0b00000100;
static constexpr size_t IO_FLAG_USERROM_ENABLE      = 0b00000001;
static constexpr size_t IO_FLAG_UNAPI_ENABLE        = 0b00010000;
static constexpr size_t IO_FLAG_EXPAND_SLOT         = 0b00100000;

// Rom's constructor throws when the id is missing; the UNAPI image is optional.
static bool hasRomWithId(DeviceConfig& config, std::string_view id)
{
    for (const auto* c : config.getXML()->getChildren("rom")) {
        if (c->getAttributeValue("id", {}) == id) {
            return true;
        }
    }
    return false;
}

FujiNet::FujiNet(DeviceConfig& config)
    : MSXDevice(config)
    , rom(getName() + " ROM", "rom", config)
    , userRom(0)
    , unapiRom(hasRomWithId(config, "unapirom")
        ? std::optional<Rom>(std::in_place, getName() + " UNAPI ROM", "rom", config, "unapirom")
        : std::nullopt)
    , debugMode(
        getCommandController(), "fujinet_debug",
        "Enable FujiNet debug logging", false)
{
    thread = std::thread(&FujiNet::readSocket, this);
    stopReading = false;

    romEnabled = true;
    userRomEnabled = false;
    userRomLoaded = false;
    userRomOpen = false;
    userRomExpected = 0;
    unapiEnabled = true;
    slotExpanded = true;
    subSlotReg = 0;
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
			// One send() per PUTC byte; Nagle would stall each request.
			int one = 1;
			setsockopt(sock, IPPROTO_TCP, TCP_NODELAY,
			           reinterpret_cast<char*>(&one), sizeof(one));
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

            size_t frameEnd = 0;
            auto packet = readBusPacket(frameEnd);
            if (!packet) {
                continue;
            }

            // Only DBC frames are consumed; the rest stay for the MSX's GETC.
            if (packet->device() == FUJI_DEVICEID_DBC) {
                for (size_t i = 0; i < frameEnd; ++i) {
                    rxBuffer.pop_front();
                }
                handleDBCCommand(std::move(packet));
            }
        }
    }
}

void FujiNet::handleDBCCommand(std::unique_ptr<FujiBusPacket> packet)
{
    switch (packet->command()) {
        case FUJICMD_OPEN: {
            fnDebugLog("FUJICMD_OPEN");

            uint8_t stream_id;
            uint32_t declared;
            fujiROMType_t rt;

            const auto& data = packet->data();
            if (data && data->size() >= 5) {
                stream_id = (*data)[0];
                declared = (static_cast<uint32_t>((*data)[1])      ) |
                           (static_cast<uint32_t>((*data)[2]) <<  8) |
                           (static_cast<uint32_t>((*data)[3]) << 16) |
                           (static_cast<uint32_t>((*data)[4]) << 24);
                rt = (data->size() >= 6) ? static_cast<fujiROMType_t>((*data)[5])
                                         : ROM_TYPE_UNKNOWN;
            } else if (packet->paramCount() >= 1) {
                // Legacy OPEN: params instead of a payload header.
                stream_id = 0;
                declared = 0;
                rt = (packet->paramCount() >= 2)
                         ? static_cast<fujiROMType_t>(packet->param(1))
                         : ROM_TYPE_UNKNOWN;
            } else {
                fujiBusReply(false);
                break;
            }

            if (rt == ROM_TYPE_UNKNOWN) {
                rt = FUJI_ROM_MSX_PLAIN; // 0 would select 8K banks
            }

            if (stream_id != 0) {
                fujiBusReply(false); // no .cfg concept on MSX
                break;
            }

            clearUserROM();
            userRomOpen = true;
            userRomExpected = declared;
            if (declared != 0 && declared <= (4u * 1024 * 1024)) {
                userRom.reserve(declared);
            }

            fnDebugLog("fujiROMType: ", rt);
            setUserROMType(rt);
            fujiBusReply(true);
            break;
        }
        case FUJICMD_WRITE:
            fnDebugLog("FUJICMD_WRITE");
            if (!userRomOpen || !packet->data()) {
                fujiBusReply(false);
                break;
            }
            writeUserROM(*(packet->data()));
            fujiBusReply(true);
            break;
        case FUJICMD_CLOSE: {
            fnDebugLog("FUJICMD_CLOSE");
            bool aborted = packet->data() && !packet->data()->empty() &&
                           (*packet->data())[0] == 0x01;
            if (!userRomOpen) {
                fujiBusReply(false);
                break;
            }
            userRomOpen = false;
            userRomExpected = 0;
            if (aborted) {
                clearUserROM(); // must not leave userRomLoaded set
                fujiBusReply(true);
                break;
            }
            if (userRom.size())
                readyUserROM();
            fujiBusReply(true);
            break;
        }
        default:
            fujiBusReply(false);
            break;
    }
}

// Isolates the first complete SLIP frame without touching rxBuffer;
// frameEnd is the count of bytes through its closing END.
std::unique_ptr<FujiBusPacket> FujiNet::readBusPacket(size_t& frameEnd)
{
    frameEnd = 0;

    size_t i0 = 0;
    bool found0 = false;
    for (size_t i = 0; i < rxBuffer.size(); ++i) {
        if (static_cast<uint8_t>(rxBuffer[i]) == SLIP_END) {
            i0 = i;
            found0 = true;
            break;
        }
    }
    if (!found0) {
        return nullptr;
    }

    size_t i1 = 0;
    bool found1 = false;
    for (size_t i = i0 + 1; i < rxBuffer.size(); ++i) {
        if (static_cast<uint8_t>(rxBuffer[i]) == SLIP_END) {
            i1 = i;
            found1 = true;
            break;
        }
    }
    if (!found1) {
        return nullptr; // incomplete
    }

    ByteBuffer packet;
    packet.reserve(i1 - i0 + 1);
    for (size_t i = i0; i <= i1; ++i) {
        packet.push_back(static_cast<uint8_t>(rxBuffer[i]));
    }

    auto result = FujiBusPacket::fromSerialized(packet);
    if (result) {
        frameEnd = i1 + 1;
    }
    return result;
}

void FujiNet::fujiBusReply(bool ack)
{
    if (sock != OPENMSX_INVALID_SOCKET) {
        FujiBusPacket packet(FUJI_DEVICEID_DBC, ack ? FUJICMD_ACK : FUJICMD_NAK);
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

void FujiNet::enableROM()
{
    fnDebugLog("FujiNet: enabledROM");
    romEnabled = true;
}

void FujiNet::disableROM()
{
    fnDebugLog("FujiNet: disableROM");
    romEnabled = false;
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

// Resets don't reach the adapter; it keeps whatever mode was last set.
void FujiNet::reset(EmuTime /*time*/)
{
}

void FujiNet::powerUp(EmuTime /*time*/)
{
    disableUserROM();
    enableROM();
    unapiEnabled = true;
    slotExpanded = true;
    subSlotReg = 0;
}

uint8_t FujiNet::readMem(uint16_t address, EmuTime time)
{
	auto value = peekMem(address, time);
	if (!windowLiveAt(address)) return value;
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
            if (value & IO_FLAG_DATA_AVAIL) {
                fnDebugLog("FujiNet: STAT -> data available");
            } else {
                fnDebugLog("FujiNet: STAT -> no data");
            }
            break;
	}
	return value;
}

// Subslot selected for the page containing address (while expanded).
int FujiNet::expandedSubSlot(uint16_t address) const
{
	return (subSlotReg >> ((address >> 14) * 2)) & 3;
}

// The IO window sits in subslot 1 when expanded, and is hidden by the user
// ROM when not.
bool FujiNet::windowLiveAt(uint16_t address) const
{
	if ((address < IO_GETC_ADDR) || (address > IO_CONTROL_ADDR)) return false;
	if (slotExpanded) return expandedSubSlot(address) == 1;
	return !userRomEnabled;
}

// Does a write at this address land in the user ROM (for bank switching)?
bool FujiNet::userRomShownAt(uint16_t address) const
{
	if (!userRomEnabled) return false;
	if (slotExpanded) return expandedSubSlot(address) == 0;
	return !unapiEnabled;
}

uint8_t FujiNet::peekUnapiROM(uint16_t address) const
{
	if (!unapiRom) return 0xFF;
	auto offset = size_t(address - 0x4000);
	if (offset >= unapiRom->size()) return 0xFF;
	return (*unapiRom)[offset];
}

uint8_t FujiNet::peekMem(uint16_t address, EmuTime /*time*/) const
{
	// Page 3 only answers FFFFh while expanded; an unexpanded slot reads FFh,
	// which is how the BIOS probe sees there is no expander.
	if (address >= 0xC000) {
		if ((address == 0xFFFF) && slotExpanded) {
			return uint8_t(0xFF ^ subSlotReg);
		}
		return 0xFF;
	}
	if (address < 0x4000) return 0xFF;

	if (windowLiveAt(address)) {
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
				break;
		}
	}

	if (slotExpanded) {
		switch (expandedSubSlot(address)) {
			case 0:
				return userRomEnabled ? peekUserROM(address) : 0xFF;
			case 1:
				return (address < 0x8000) ? peekUnapiROM(address) : 0xFF;
			case 2:
				return userRomEnabled ? 0xFF : rom[address - 0x4000];
			default:
				return 0xFF;
		}
	}

	if (unapiEnabled)
		return (address < 0x8000) ? peekUnapiROM(address) : 0xFF;
	if (userRomEnabled)
		return peekUserROM(address);
	return rom[address - 0x4000];
}

uint8_t FujiNet::peekUserROM(uint16_t address) const
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

    uint16_t bank = (address - 0x4000) / userRomBankSize;
    if (bank >= MAX_BANKS) return 0xFF;
    uint32_t offset = userRomMap[bank];

    // A user ROM smaller than the mapped window (e.g. a 16K image seen at
    // 0x8000..0xBFFF) would otherwise index past the end of the vector.
    size_t idx = offset + address - 0x4000 - bank * userRomBankSize;
    if (idx >= userRom.size()) return 0xFF;

    return userRom[idx];
}

void FujiNet::writeMem(uint16_t address, uint8_t value, EmuTime /*time*/)
{
    std::lock_guard lock(mtx);

    if (address >= 0xC000) {
        if ((address == 0xFFFF) && slotExpanded) {
            subSlotReg = value;
            fnDebugLog("FujiNet: subslot register <- ", int(value));
        }
        return;
    }

    if (windowLiveAt(address)) {
        switch (address) {
            case IO_PUTC_ADDR:
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
                // Only CONFIG sets the mode: once it has handed off, a stray
                // write (a game's stack, a RAM probe) must not.
                if (userRomEnabled || (unapiEnabled && !slotExpanded)) return;
                if (value & IO_FLAG_ROM_MODE_CMD) {
                    unapiEnabled = (value & IO_FLAG_UNAPI_ENABLE) != 0;
                    bool wantExpanded = (value & IO_FLAG_EXPAND_SLOT) != 0;
                    if (wantExpanded != slotExpanded) {
                        subSlotReg = 0;
                    }
                    slotExpanded = wantExpanded;
                    fnDebugLog("FujiNet: mode ", int(value),
                               unapiEnabled ? " unapi" : "",
                               slotExpanded ? " expanded" : "");

                    if (value & IO_FLAG_USERROM_ENABLE) {
                        enableUserROM();
                        disableROM();
                    }
                    else {
                        disableUserROM();
                        enableROM();
                    }
                }
                return;
            default:
                // other window addresses (GETC/STATUS) are not writable
                return;
        }
    }

    if (userRomShownAt(address)) {
        handleBankSwitch(address, value);
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
