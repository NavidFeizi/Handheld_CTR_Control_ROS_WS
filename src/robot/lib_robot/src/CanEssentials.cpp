#include "CanEssentials.hpp"


std::unordered_map<std::string, uint8_t> faulhaberComCodeDictionary = {
    {"LR", 0xB6},
    {"LA", 0xB4},
    {"M", 0x3C},
    {"AC", 0x65},
    {"ADL", 0x00},
    {"ADR", 0x01},
    {"APCMOD", 0x02},
    {"APL", 0x03},
    {"CI", 0xA2},
    {"CO", 0x05},
    {"CONTMOD", 0x06},
    {"CORRIDOR", 0x9D},
    {"CST", 0x58},
    {"DCE", 0x6B},
    {"DEC", 0x6D},
    {"DEV", 0x6F},
    {"DI", 0x08},
    {"DIGOUT", 0x0A},
    {"DIRIN", 0x0C},
    {"EN", 0x0F},
    {"ENCMOD", 0x10},
    {"ENCOUT", 0x11},
    {"ENCRES", 0x70},
    {"ENCSPEED", 0x12},
    {"ERROUT", 0x14},
    {"FAULT STATUS", 0xDF},
    {"FCONFIG", 0xD0},
    {"FHIX", 0x35},
    {"GAC", 0x15},
    {"GADC", 0xB3},
    {"GADV", 0xB2},
    {"GCC", 0x18},
    {"GCI", 0x63},
    {"GCL", 0x19},
    {"GCORRIDOR", 0x62},
    {"GDCE", 0x1A},
    {"GDEC", 0x1B},
    {"GDEV", 0x1C},
    {"GEARMOD", 0x1D},
    {"GENCRES", 0x1E},
    {"GHOSP", 0x24},
    {"GI", 0x26},
    {"GKN", 0x4D},
    {"GMAV", 0x27},
    {"GMOD", 0x28},
    {"GMOTTYP", 0x29},
    {"GMV", 0x2A},
    {"GN", 0x2B},
    {"GNL", 0x2C},
    {"GOHIX", 0x2E},
    {"GOHOSEQ", 0x2F},
    {"GOIX", 0xA3},
    {"GOPMOD", 0xFE},
    {"GPC", 0x30},
    {"GPD", 0x5E},
    {"GPL", 0x31},
    {"GPN", 0x32},
    {"GPOR", 0x33},
    {"GPP", 0x5D},
    {"GRC", 0x34},
    {"GRM", 0x4E},
    {"GRPC", 0x61},
    {"GRU", 0x60},
    {"GSP", 0x36},
    {"GSR", 0x56},
    {"GSTN", 0x38},
    {"GSTW", 0x39},
    {"GTM", 0x50},
    {"GU", 0x5F},
    {"GV", 0x3A},
    {"HA", 0x72},
    {"HALLSPEED", 0x3B},
    {"HB", 0x73},
    {"HD", 0x74},
    {"HL", 0x75},
    {"HN", 0x76},
    {"HO", 0xB8},
    {"HOC", 0x5B},
    {"HOSP", 0x78},
    {"HP", 0x79},
    {"I", 0x7B},
    {"IOC", 0x5C},
    {"KN", 0x9E},
    {"LCC", 0x80},
    {"LL", 0xB5},
    {"LPC", 0x81},
    {"LPN", 0x82},
    {"LR", 0xB6},
    {"M", 0x3C},
    {"MAV", 0x83},
    {"MV", 0x85},
    {"OPMOD", 0xFD},
    {"OST", 0x57},
    {"PD", 0x9C},
    {"POR", 0x89},
    {"POS", 0x40},
    {"POSOUT", 0x4C},
    {"PP", 0x9B},
    {"REFIN", 0x41},
    {"RESET", 0x59},
    {"RM", 0x9F},
    {"RN", 0x44},
    {"SAVE", 0x53},
    {"SETPLC", 0x51},
    {"SETTTL", 0x52},
    {"SHA", 0x8A},
    {"SHL", 0x90},
    {"SHN", 0x9A},
    {"SIN", 0xA0},
    {"SO", 0x45},
    {"SOR", 0x8E},
    {"SP", 0x8F},
    {"SR", 0xA4},
    {"STEPMOD", 0x46},
    {"STN", 0x64},
    {"STW", 0x77},
    {"SWS", 0x5A},
    {"TEM", 0x47},
    {"TM", 0xAF},
    {"TO", 0x55},
    {"TPOS", 0x4B},
    {"U", 0x92},
    {"V", 0x93},
    {"VER-Get Firmware Version", 0x100A},
    {"VOLTMOD", 0x49}};

/**/
ControlWord::ControlWord()
{
    this->controlword = 0x0000;
}

/* converts bits to int16_t*/
int16_t ControlWord::get()
{
    controlword = (switch_ON << 0) |
                  (enable_voltage << 1) |
                  (quick_stop << 2) |
                  (enable_operation << 3) |
                  (bit4 << 4) |
                  (bit5 << 5) |
                  (bit6 << 6) |
                  (fault_reset << 7) |
                  (halt << 8) |
                  (bit9 << 9) |
                  (bit10 << 10) |
                  (bit11 << 11) |
                  (bit12 << 12) |
                  (bit13 << 13) |
                  (bit14 << 14) |
                  (bit15 << 15);
    return controlword;
}

void ControlWord::set(int16_t input)
{
    switch_ON = (input & 0x0001) != 0;
    enable_voltage = (input & 0x0002) != 0;
    quick_stop = (input & 0x0004) != 0;
    enable_operation = (input & 0x0008) != 0;
    bit4 = (input & 0x0010) != 0;
    bit5 = (input & 0x0020) != 0;
    bit6 = (input & 0x0040) != 0;
    fault_reset = (input & 0x0080) != 0;
    halt = (input & 0x0100) != 0;
    bit9 = (input & 0x0200) != 0;
    bit10 = (input & 0x0400) != 0;
    bit11 = (input & 0x0800) != 0;
    bit12 = (input & 0x1000) != 0;
    bit13 = (input & 0x2000) != 0;
    bit14 = (input & 0x4000) != 0;
    bit15 = (input & 0x8000) != 0;
}

/**/
StatusWord::StatusWord()
{
    this->statusword = 0x0000;
    StatusWord::update(0x0000);
}

/* updates all bits */
void StatusWord::update(uint16_t newStatusWord)
{
    this->statusword = newStatusWord;
    ready_to_switch_ON = (statusword & 0x0001) != 0;
    switched_ON = (statusword & 0x0002) != 0;
    operation_enabled = (statusword & 0x0004) != 0;
    fault = (statusword & 0x0008) != 0;
    Voltage_Enabled = (statusword & 0x0010) != 0;
    Quick_Stop = (statusword & 0x0020) != 0;
    Switch_On_Disabled = (statusword & 0x0040) != 0;
    Warning = (statusword & 0x0080) != 0;
    // bit8 = (statusword & 0x0100) != 0;
    // bit9 = (statusword & 0x0200) != 0;
    bit10 = (statusword & 0x0400) != 0;
    Internal_Limit_Active = (statusword & 0x0800) != 0;
    bit12 = (statusword & 0x1000) != 0;
    bit13 = (statusword & 0x2000) != 0;
    // bit14 = (statusword & 0x4000) != 0;
    // bit15 = (statusword & 0x8000) != 0;
}

/* Member function that converts CiA402 code number to string message*/
std::string StatusWord::getCiA402StatusMessage()
{
    std::string message;
    if (operation_enabled)
    {
        message = "Operation enabled";
    }
    else if (switched_ON)
    {
        message = "Switched On";
    }
    else if (ready_to_switch_ON)
    {
        message = "Ready to Switch On";
    }
    else if (Switch_On_Disabled)
    {
        message = "Switch on disabled";
    }
    else
    {
        message = "Unkonwn CiA 402 status";
    }

    if (fault)
    {
        message = "Fault | " + message;
    }

    if (Warning)
    {
        message = "Warning | " + message;
    }

    return message;
}

std::string GetCommandFromHex(uint8_t hexValue)
{
    for (const auto &entry : faulhaberComCodeDictionary)
    {
        if (entry.second == hexValue)
        {
            return entry.first;
        }
    }
    return "";
}

/* convert binary to decimal*/
uint16_t bin2Dec(const std::string &binaryString)
{
    return std::bitset<16>(binaryString).to_ulong();
}

