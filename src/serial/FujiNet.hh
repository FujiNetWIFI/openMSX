#ifndef FUJINET_HH
#define FUJINET_HH

#include "FujiBusPacket.h"
#include "MSXCPUInterface.hh"
#include "MSXCliComm.hh"
#include "MSXDevice.hh"
#include "Socket.hh"
#include "Rom.hh"
#include "Poller.hh"
#include "BooleanSetting.hh"

#include "circular_buffer.hh"
#include "fujiRomType.h"

#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

namespace openmsx {

class DeviceConfig;

class FujiNet final
    : public MSXDevice
{
public:
    static constexpr unsigned MAX_BANKS = 4;

	explicit FujiNet(DeviceConfig& config);
	~FujiNet() override;

	void reset(EmuTime time) override;
	void powerUp(EmuTime time) override;
	[[nodiscard]] uint8_t readMem(uint16_t address, EmuTime time) override;
	[[nodiscard]] uint8_t peekMem(uint16_t address, EmuTime time) const override;
	void writeMem(uint16_t address, uint8_t value, EmuTime time) override;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	void close();
	void readSocket(); // loop of helper thread that reads from socket

	std::unique_ptr<FujiBusPacket> readBusPacket(size_t& frameEnd);
	void handleDBCCommand(std::unique_ptr<FujiBusPacket> packet);
	void fujiBusReply(bool ack);

	void enableROM();
	void disableROM();
	void clearUserROM();
	void writeUserROM(std::span<unsigned const char> data);
	void readyUserROM();
	void enableUserROM();
	void disableUserROM();
	void setUserROMType(fujiROMType_t t);
	[[nodiscard]] bool windowLiveAt(uint16_t address) const;
	[[nodiscard]] bool userRomShownAt(uint16_t address) const;
	[[nodiscard]] int expandedSubSlot(uint16_t address) const;
	[[nodiscard]] uint8_t peekUnapiROM(uint16_t address) const;
	void setUserROMBank(uint8_t n, uint8_t block);
	uint8_t peekUserROM(uint16_t address) const;
	void handleBankSwitch(uint16_t address, uint8_t value);

	template <typename... Args>
	void fnDebugLog(Args&&... args);

	Rom rom;
	bool romEnabled;
	std::vector<std::uint8_t> userRom;
	std::array<std::uint32_t, MAX_BANKS> userRomMap;
	std::optional<Rom> unapiRom;
	fujiROMType_t userRomType;
	bool userRomEnabled;
	bool userRomLoaded;
	bool userRomOpen;
	uint32_t userRomExpected;
	bool unapiEnabled;
	bool slotExpanded;
	uint8_t subSlotReg; // written via FFFFh while expanded
	uint16_t userRomBankSize;
	BooleanSetting debugMode;
	std::thread thread; // receiving thread (reads from pty)
	Poller poller; // to abort read-thread in a portable way
	mutable std::mutex mtx; // to protect shared data between emulation and receiving thread
	cb_queue<char> rxBuffer; // read/written by both the main and the receiver thread. Must hold 'mutex' while doing so.
	// cb_queue<char> txBuffer;
	// bool inSLIPPacket;
	std::atomic<SOCKET> sock = OPENMSX_INVALID_SOCKET;
	bool stopReading;
};

} // namespace openmsx

#endif
