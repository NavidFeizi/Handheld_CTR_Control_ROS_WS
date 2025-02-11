#ifndef CONTROLWORD_HPP
#define CONTROLWORD_HPP

#include <cstdint> // For int16_t
#include <string>
#include <unordered_map>
#include <bitset>

#include "CanEssentials.hpp"

// Define CANopen Object Dictionary indices
constexpr uint16_t STATUS_WORD_IDX = 0x6041;
constexpr uint16_t CONTROL_WORD_IDX = 0x6040;
constexpr uint16_t ACTUAL_POSITION_IDX = 0x6064;
constexpr uint16_t ACTUAL_VELOCITY_FAULHABER_IDX = 0x606c;
constexpr uint16_t ACTUAL_TORQUE_FAULHABER_IDX = 0x6077;
constexpr uint16_t ACTUAL_VELOCITY_MAXON_IDX = 0x2028;
constexpr uint16_t ACTUAL_CURRENT_MAXON_IDX = 0x2027;
constexpr uint16_t TARGET_POSITION_IDX = 0x607A;
constexpr uint16_t TARGET_VELOCITY_IDX = 0x60FF;
constexpr uint16_t MODE_OF_OPERATION_IDX = 0x6060;
constexpr uint16_t MODE_OF_OPERATION_DISP_IDX = 0x6061;


struct ControlWord
{
public:
  ControlWord();
  int16_t get();
  void set(int16_t ctrlword);

  int16_t controlword;
  bool switch_ON : 1;        // bit 0
  bool enable_voltage : 1;   // bit 1
  bool quick_stop : 1;       // bit 2
  bool enable_operation : 1; // bit 3
  bool bit4 : 1;             // bit 4     Operation Mode Specific
  bool bit5 : 1;             // bit 5     Operation Mode Specific
  bool bit6 : 1;             // bit 6     Operation Mode Specific
  bool fault_reset : 1;      // bit 7
  bool halt : 1;             // bit 8
  bool bit9 : 1;             // bit 9     Change on set-point (only in Profile position mode)
  bool bit10 : 1;            // bit 10
  bool bit11 : 1;            // bit 11
  bool bit12 : 1;            // bit 12
  bool bit13 : 1;            // bit 13
  bool bit14 : 1;            // bit 14
  bool bit15 : 1;            // bit 15
};

struct StatusWord
{
public:
  StatusWord();
  // get status word on SDO and translate bits
  void update(uint16_t newStatusWord);
  std::string getCiA402StatusMessage();

  uint16_t statusword;
  bool ready_to_switch_ON : 1; // bit 0
  bool switched_ON : 1;        // bit 1
  bool operation_enabled : 1;  // bit 2
  bool fault : 1;              // bit 3
  bool Voltage_Enabled : 1;    // bit 4
  bool Quick_Stop : 1;         // bit 5
  bool Switch_On_Disabled : 1; // bit 6
  bool Warning : 1;            // bit 7
  // bool bit8 : 1;                  // bit 8
  // bool bit9 : 1;                  // bit 9
  bool bit10 : 1;                 // bit 10   Target Reached(PP)
  bool Internal_Limit_Active : 1; // bit 11
  bool bit12 : 1;                 // bit 12   Set-Point Acknowledge(PP)
  bool bit13 : 1;                 // bit 13
  // bool bit14 : 1;                 // bit 14
  // bool bit15 : 1;                 // bit 15
};

class Flags {
public:
    enum class FlagIndex {
        BOOT_SUCCESS,
        TASKS_POSTED,
        ENCODER_SET,
        NEW_TARG_READY,
        ENCODER_MEM_READY,
        FLAG_COUNT
    };

private:
    std::bitset<static_cast<size_t>(FlagIndex::FLAG_COUNT)> flags;

public:
    void set(FlagIndex index, bool value) {
        flags.set(static_cast<size_t>(index), value); // Cast to size_t
    }

    bool get(FlagIndex index) const {
        return flags.test(static_cast<size_t>(index)); // Cast to size_t
    }
};


std::string GetCommandFromHex(uint8_t hexValue);
uint16_t bin2Dec(const std::string &binaryString);
std::string dec2Bin(uint16_t decimalValue);

/* convert binary to string*/
template <typename T>
std::string ToBinaryString(T value)
{
  size_t numBits = sizeof(T) * 8;
  std::string binaryString = std::bitset<32>(value).to_string().substr(32 - numBits);
  // Insert spaces every 4 bits
  for (size_t i = binaryString.size() - 4; i > 0; i -= 4)
  {
    binaryString.insert(i, " ");
  }
  return binaryString;
}

// Declare your enum class
enum class OpMode : int8_t {
    Disabled = 0x00,
    PositionProfile = 0x01,
    VelocityProfile = 0x03,
    // Velocity = 0xFE,
    Homing = 0x06,
    FaulhaberCommand = -1
};

#endif // CONTROLWORD_HPP