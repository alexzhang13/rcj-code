#ifndef _TEMP_DATA_H_
#define _TEMP_DATA_H_

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>


class TempData {
public:

    typedef struct {
        uint32_t tstamp; //timestamp
        char id; //always comes out as t for temp
        int tmpR[9];
        int tmpL[9];
    }TMP_DataType;

    TempData();
    ~TempData();

    void storeCommand(char* buf);
    int parseData();
    int getRightTemp(int index);
    int getLeftTemp(int index);

private:
    TMP_DataType data;
    char m_command[128];
};

#endif // !_TEMP_DATA_H_
