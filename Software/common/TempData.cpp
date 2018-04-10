#include "TempData.h"

#include <stdio.h>
#include <string.h>

TempData::TempData()
{
    memset(m_command,'\0', 128);
}

TempData::~TempData()
{

}

void TempData::storeCommand(char* buf) {
    memcpy(m_command, buf, 64);
}

int TempData::parseData()
{
    sscanf(m_command, "%d %c %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f"
           , &data.tstamp, &data.id, &data.tmpR[0], &data.tmpR[1], &data.tmpR[2], &data.tmpR[3], &data.tmpR[4], &data.tmpR[5], &data.tmpR[6], &data.tmpR[7], &data.tmpR[8]
            , &data.tmpL[0], &data.tmpL[1], &data.tmpL[2], &data.tmpL[3], &data.tmpL[4], &data.tmpL[5], &data.tmpL[6], &data.tmpL[7], &data.tmpL[8]);
    return 0;
}

int* TempData::getRightTemp()
{
    return data.tmpR;
}

int* TempData::getLeftTemp()
{
    return data.tmpL;
}
