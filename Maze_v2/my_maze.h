#ifndef MY_MAZE_H
#define MY_MAZE_H

#include <EEPROM.h>

class My_Maze {
    const int baseAddress = 10;  // Offset

public:
    // Save a single byte value at index (0-based)
    void save(uint8_t index, uint8_t value) {
        EEPROM.update(baseAddress + index, value);
        EEPROM.update(baseAddress + index+1, 255);
    }

    // Load a value from EEPROM by index
    uint8_t load(uint8_t index) {
        return EEPROM.read(baseAddress + index);
    }
};

#endif