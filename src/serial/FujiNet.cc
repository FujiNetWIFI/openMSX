#undef UNUSED
#include "FujiNet.hh"
#include "FujiBusPacket.h"
#include "Timer.hh"

#include "GlobalSettings.hh"
#include "fujiDeviceID.h"
#include "fujiRomType.h"
#include "serialize.hh"
#include <algorithm>
#include <bit>
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
static constexpr size_t IO_FLAG_USERROM_B_ENABLE    = 0b01000000;
static constexpr size_t IO_FLAG_UNAPI_ENABLE        = 0b00010000;
static constexpr size_t IO_FLAG_EXPAND_SLOT         = 0b00100000;

// DBC mapper commands (0x41 and 0x4D have other names in fujiCommandID.h).
static constexpr uint8_t DBC_MAPPER_SET_INFO   = 0x49; // 'I'
static constexpr uint8_t DBC_MAPPER_ADD_WINDOW = 0x41; // 'A'
static constexpr uint8_t DBC_MAPPER_ADD_MIRROR = 0x4D; // 'M'

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
    , unapiRom(hasRomWithId(config, "unapirom")
        ? std::optional<Rom>(std::in_place, getName() + " UNAPI ROM", "rom", config, "unapirom")
        : std::nullopt)
    , debugMode(
        getCommandController(), "fujinet_debug",
        "Enable FujiNet debug logging", false)
{
    thread = std::thread(&FujiNet::readSocket, this);
    stopReading = false;

    userRomExpected = 0;
    unapiEnabled = true;
    slotExpanded = true;
    subSlotReg = 0;
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
    switch (static_cast<uint8_t>(packet->command())) {
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

            if (stream_id != 0 && stream_id != 2) {
                fujiBusReply(false); // no .cfg concept on MSX
                break;
            }

            auto& img = userRoms[stream_id == 2 ? 1 : 0];
            clearUserROM(img);
            openImage = (stream_id == 2) ? 1 : 0;
            layoutImage = openImage;
            userRomExpected = declared;
            if (declared != 0 && declared <= (4u * 1024 * 1024)) {
                img.data.reserve(declared);
            }

            fnDebugLog("fujiROMType: ", rt, " stream: ", int(stream_id));
            // No type: no windows, which already means flat. The firmware
            // sends the layout with the DBC mapper commands instead.
            resetWindows(img);
            if (rt != ROM_TYPE_UNKNOWN)
                setLayoutType(img, rt);
            fujiBusReply(true);
            break;
        }
        case DBC_MAPPER_SET_INFO: {
            auto& img = userRoms[openImage >= 0 ? openImage : layoutImage];
            const auto& data = packet->data();
            resetWindows(img);
            if (data && !data->empty()) {
                setLayoutType(img, static_cast<fujiROMType_t>((*data)[0]));
            }
            fnDebugLog("DBC_MAPPER_SET_INFO");
            fujiBusReply(true);
            break;
        }
        case DBC_MAPPER_ADD_WINDOW: {
            auto& img = userRoms[openImage >= 0 ? openImage : layoutImage];
            if (packet->paramCount() < 4) {
                fujiBusReply(false);
                break;
            }
            MapperWindow w;
            w.regBase = packet->param(0);
            w.regTop = packet->param(1);
            w.windowBase = packet->param(2);
            w.windowTop = packet->param(3);
            fujiBusReply(addWindow(img, w));
            break;
        }
        case DBC_MAPPER_ADD_MIRROR: {
            auto& img = userRoms[openImage >= 0 ? openImage : layoutImage];
            if (packet->paramCount() < 3) {
                fujiBusReply(false);
                break;
            }
            MapperMirror m;
            m.mirrorBase = packet->param(0);
            m.mirrorTop = packet->param(1);
            m.source = packet->param(2);
            fujiBusReply(addMirror(img, m));
            break;
        }
        case FUJICMD_WRITE:
            fnDebugLog("FUJICMD_WRITE");
            if (openImage < 0 || !packet->data()) {
                fujiBusReply(false);
                break;
            }
            writeUserROM(userRoms[openImage], *(packet->data()));
            fujiBusReply(true);
            break;
        case FUJICMD_CLOSE: {
            fnDebugLog("FUJICMD_CLOSE");
            bool aborted = packet->data() && !packet->data()->empty() &&
                           (*packet->data())[0] == 0x01;
            if (openImage < 0) {
                fujiBusReply(false);
                break;
            }
            auto& img = userRoms[openImage];
            openImage = -1;
            userRomExpected = 0;
            if (aborted) {
                clearUserROM(img); // must not leave loaded set
                fujiBusReply(true);
                break;
            }
            if (img.data.size())
                readyUserROM(img);
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

void FujiNet::clearUserROM(UserRomImage& img)
{
    img.loaded = false;
    img.data.clear();
}

void FujiNet::writeUserROM(UserRomImage& img, std::span<unsigned const char> data)
{
    for (auto c : data) {
        img.data.push_back(c);
    }
}

void FujiNet::readyUserROM(UserRomImage& img)
{
    fnDebugLog("FujiNet: readyUserROM");
    setROMSize(img, img.data.size());
    activateUserROM(img);
    img.loaded = true;
}

void FujiNet::resetWindows(UserRomImage& img)
{
    img.windows.fill({});
    img.windowCount = 0;
    img.mirrorCount = 0;
    img.windowBase = UINT_MAX;
    img.windowTop = 0;
    img.registerBase = UINT_MAX;
    img.registerTop = 0;
    img.mappedBase = UINT_MAX;
    img.mappedTop = 0;
}

static void extend(unsigned& base, unsigned& top, unsigned newBase, unsigned newTop)
{
    if (newBase < base) base = newBase;
    if (newTop > top) top = newTop;
}

bool FujiNet::addWindow(UserRomImage& img, const MapperWindow& w)
{
    if (img.windowCount >= MAX_WINDOWS) return false;
    img.windows[img.windowCount] = w;

    // Always sized by window 0, as the bus controller does.
    auto newWS = uint16_t(img.windows[0].windowTop - img.windows[0].windowBase);
    if (newWS < 0x10000 / 64) return false;

    extend(img.windowBase, img.windowTop, w.windowBase, w.windowTop);
    extend(img.mappedBase, img.mappedTop, img.windowBase, img.windowTop);
    if (w.regBase != w.regTop) {
        extend(img.registerBase, img.registerTop, w.regBase, w.regTop);
    }
    img.windowSize = newWS;
    img.windowShift = uint16_t(std::countr_zero(unsigned(newWS)));
    img.windowCount++;
    fnDebugLog("FujiNet: window ", img.windowCount - 1,
               " reg:", w.regBase, "-", w.regTop,
               " win:", w.windowBase, "-", w.windowTop);
    return true;
}

bool FujiNet::addMirror(UserRomImage& img, const MapperMirror& m)
{
    if (img.mirrorCount >= MAX_MIRRORS) return false;
    img.mirrors[img.mirrorCount] = m;
    extend(img.mappedBase, img.mappedTop, m.mirrorBase, m.mirrorTop);
    img.mirrorCount++;
    fnDebugLog("FujiNet: mirror ", m.mirrorBase, "-", m.mirrorTop,
               " source:", m.source);
    return true;
}

// The built-in MSX layouts (the bus controller's Mapper::setROMType).
void FujiNet::setLayoutType(UserRomImage& img, fujiROMType_t t)
{
    resetWindows(img);
    auto win = [&](uint32_t rb, uint32_t rt, uint32_t wb, uint32_t wt) {
        MapperWindow w;
        w.regBase = rb; w.regTop = rt; w.windowBase = wb; w.windowTop = wt;
        (void)addWindow(img, w);
    };
    auto mir = [&](uint32_t mb, uint32_t mt, uint32_t src) {
        MapperMirror m;
        m.mirrorBase = mb; m.mirrorTop = mt; m.source = src;
        (void)addMirror(img, m);
    };
    switch (t) {
        case FUJI_ROM_MSX_ASCII8:
            win(0x6000, 0x6800, 0x4000, 0x6000);
            win(0x6800, 0x7000, 0x6000, 0x8000);
            win(0x7000, 0x7800, 0x8000, 0xA000);
            win(0x7800, 0x8000, 0xA000, 0xC000);
            break;
        case FUJI_ROM_MSX_ASCII16:
            win(0x6000, 0x6800, 0x4000, 0x8000);
            win(0x7000, 0x7800, 0x8000, 0xC000);
            break;
        case FUJI_ROM_MSX_KONAMI:
            win(0x0000, 0x0000, 0x4000, 0x6000);
            win(0x6000, 0x8000, 0x6000, 0x8000);
            win(0x8000, 0xA000, 0x8000, 0xA000);
            win(0xA000, 0xC000, 0xA000, 0xC000);
            mir(0x0000, 0x4000, 0x4000);
            mir(0xC000, 0x10000, 0x8000);
            break;
        case FUJI_ROM_MSX_KONAMI_SCC:
            win(0x5000, 0x5800, 0x4000, 0x6000);
            win(0x7000, 0x7800, 0x6000, 0x8000);
            win(0x9000, 0x9800, 0x8000, 0xA000);
            win(0xB000, 0xB800, 0xA000, 0xC000);
            mir(0x0000, 0x4000, 0x8000);
            mir(0xC000, 0x10000, 0x4000);
            break;
        default:
            break;
    }
}

void FujiNet::setROMSize(UserRomImage& img, size_t size)
{
    img.bankCount = uint16_t(std::max<size_t>(1, (size + img.windowSize - 1) / img.windowSize));
}

void FujiNet::activateUserROM(UserRomImage& img)
{
    for (unsigned i = 0; i < img.windowCount; ++i) {
        img.windows[i].bank = uint8_t(i);
    }
}

void FujiNet::handleBankSwitch(UserRomImage& img, uint16_t address, uint8_t value)
{
    if (address < img.registerBase || address >= img.registerTop) return;
    for (unsigned i = 0; i < img.windowCount; ++i) {
        auto& w = img.windows[i];
        if (w.regBase <= address && address < w.regTop) {
            w.bank = uint8_t(value & (img.bankCount - 1));
            char addr_str[8];
            snprintf(addr_str, 8, "%04X", address);
            fnDebugLog("FujiNet: bank switch addr:", addr_str, " val:", value,
                       " window:", i, " bank:", int(w.bank));
            return;
        }
    }
}

// Resets don't reach the adapter; it keeps whatever mode was last set.
void FujiNet::reset(EmuTime /*time*/)
{
}

void FujiNet::powerUp(EmuTime /*time*/)
{
    for (auto& img : userRoms) img.enabled = false;
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
	return !userRoms[0].enabled;
}

// CONFIG is visible only while nothing else is enabled.
bool FujiNet::configMode() const
{
	return !userRoms[0].enabled && !userRoms[1].enabled;
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
				bool ready = openImage < 0 && (userRoms[0].loaded || userRoms[1].loaded);
				uint8_t status = (ready ? 0x40 : 0x00);
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
				if (userRoms[0].enabled) return peekUserROM(userRoms[0], address);
				return configMode() ? rom[address - 0x4000] : 0xFF;
			case 1:
				return (address < 0x8000) ? peekUnapiROM(address) : 0xFF;
			case 2:
				return 0xFF; // reserved for Nextor
			default:
				return userRoms[1].enabled ? peekUserROM(userRoms[1], address) : 0xFF;
		}
	}

	if (unapiEnabled)
		return (address < 0x8000) ? peekUnapiROM(address) : 0xFF;
	if (userRoms[0].enabled)
		return peekUserROM(userRoms[0], address);
	return rom[address - 0x4000];
}

uint8_t FujiNet::peekUserROM(const UserRomImage& img, uint16_t address) const
{
    int64_t offset;
    if (img.windowCount == 0) {
        if (address < 0x4000 || address >= 0xC000) return 0xFF;
        offset = address - 0x4000;
    } else {
        if (address < img.mappedBase || address >= img.mappedTop) return 0xFF;
        uint32_t romOffset = address;
        for (unsigned i = 0; i < img.mirrorCount; ++i) {
            const auto& m = img.mirrors[i];
            if (m.mirrorBase <= address && address < m.mirrorTop) {
                romOffset = address + m.source - m.mirrorBase;
                break;
            }
        }
        unsigned w = ((romOffset - img.windowBase) >> img.windowShift) &
                     (img.windowCount - 1);
        int64_t adj = int64_t(romOffset) +
                      (int64_t(img.windows[w].bank) - int64_t(w)) * img.windowSize;
        offset = adj - int64_t(img.windowBase);
    }
    if (offset < 0 || size_t(offset) >= img.data.size()) return 0xFF;
    return img.data[size_t(offset)];
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
                if (userRoms[0].enabled || userRoms[1].enabled ||
                    (unapiEnabled && !slotExpanded)) return;
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

                    bool wasA = userRoms[0].enabled, wasB = userRoms[1].enabled;
                    userRoms[0].enabled = (value & IO_FLAG_USERROM_ENABLE) != 0;
                    userRoms[1].enabled = slotExpanded && (value & IO_FLAG_USERROM_B_ENABLE) != 0;
                    if (userRoms[0].enabled && !wasA) activateUserROM(userRoms[0]);
                    if (userRoms[1].enabled && !wasB) activateUserROM(userRoms[1]);
                    fnDebugLog("FujiNet: ROM A ", userRoms[0].enabled,
                               " B ", userRoms[1].enabled);
                }
                return;
            default:
                // other window addresses (GETC/STATUS) are not writable
                return;
        }
    }

    if (slotExpanded) {
        switch (expandedSubSlot(address)) {
            case 0:
                if (userRoms[0].enabled) handleBankSwitch(userRoms[0], address, value);
                break;
            case 3:
                if (userRoms[1].enabled) handleBankSwitch(userRoms[1], address, value);
                break;
            default:
                break;
        }
    } else if (userRoms[0].enabled && !unapiEnabled) {
        handleBankSwitch(userRoms[0], address, value);
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
