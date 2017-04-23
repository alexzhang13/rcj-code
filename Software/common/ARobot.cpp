#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "ARobot.h"

using namespace std;

ARobot::ARobot(SerialPort *port) :mPort(port)
{
    initialYaw = 0;
    toTurn = 0;
    backingBlack = false;
    currTileLight = WHITE;
    currDir = FRONT; 
    currState = IDLE;
}

ARobot::~ARobot() 
{

}


void ARobot::WriteCommand(char* command, int size)
{
    mPort->write(command, size);
}

void ARobot::checkRamp()
{
    //check if pitch has exceeded certain threshold
}

void ARobot::checkVictimTemp()
{

}

void ARobot::setTempThresh(float left, float right)
{
    tempLeft = left;
    tempRight = right;
}

float ARobot::getLeftVictimTemp()
{
    return tempLeft;
}

float ARobot::getRightVictimTemp()
{
    return tempRight;
}

void ARobot::setLightThresh(int black, int silver)
{
    black_thresh = black;
    silver_thresh = silver;
}

int ARobot::getBlackThresh() 
{
    return black_thresh;
}

int ARobot::getSilverThresh()
{
    return silver_thresh;
}

void ARobot::checkLightTile()
{
    mlen_light = lightDataList.size();
    if(mlen_light < 3)
        return;

    if(lightDataList[mlen_light-1].checkLight() == 2 && lightDataList[mlen_light-2].checkLight() == 2 && lightDataList[mlen_light-3].checkLight() == 2) {
        currTileLight = SILVER;
    } else if (lightDataList[mlen_light-1].checkLight() == 1 && lightDataList[mlen_light-2].checkLight() == 1 && lightDataList[mlen_light-3].checkLight() == 1) {
        currTileLight = BLACK;
        if(backingBlack == false) {
            backingBlack = true;
            MoveDistance(160, false); //move back 16 cm
            //update map
        }
    } else {
        currTileLight = WHITE;
        backingBlack = false; //reset
    }
    if(mlen_light > 200) 
        lightDataList.erase(lightDataList.begin(), lightDataList.begin() + mlen_light - 200);
    return;
}

void ARobot::LEDLight(int time)
{
    char* command;
    sprintf(command, "%c %c %d", 'd', 'b', time);
    WriteCommand(command, sizeof(command) / sizeof(command[0]));
    currState = LED;
}

void ARobot::Drop()
{
    char* command;
    sprintf(command, "%c %c", 'd', 'a');
    WriteCommand(command, sizeof(command) / sizeof(command[0]));
    currState = DROP;
}

void ARobot::SetSpeed(int left_speed, int right_speed) {
    char* command;
    sprintf(command, "%c %c %d %d", 'm', 'f', left_speed, right_speed);
    WriteCommand(command, sizeof(command) / sizeof(command[0]));
}

void ARobot::MoveDistance(int distance_mm, BotDir dir) //forward = true
{
    char* command;
    if(dir == FRONT) {
        sprintf(command, "%c %c %d", 'm', 'a', distance_mm);
    } else {
        sprintf(command, "%c %c %d", 'm', 'b', distance_mm);
    }
    currState = MOVE;
    WriteCommand(command, sizeof(command) / sizeof(command[0]));
}
void ARobot::TurnDistance(int degrees, BotDir dir)
{
    initialYaw = imuDataList.end()->m_yaw;
    char* command;
    if(dir == RIGHT) {
        //sprintf(command, "%c %c %d", 'm', 'd', distance_mm);
        toTurn = initialYaw + degrees;
        currDir = RIGHT;
    } else {
        //sprintf(command, "%c %c %d", 'm', 'e', distance_mm);
        toTurn = initialYaw - degrees;
        currDir = LEFT;
    }
    currState = TURN;
    WriteCommand(command, sizeof(command) / sizeof(command[0]));
}

void ARobot::StopTurn(BotDir dir)
{
    IMUData currYaw = imuDataList.end();
    if(dir == RIGHT) {
        if(initialYaw >= 0.0f && currYaw.m_yaw < 0.0f) { //if robot crosses over from 180 to -180, direction switches
            currYaw.m_yaw += 360; //range fixing
        }
        if(currYaw.m_yaw >= toTurn) {
            char* command;
            sprintf(command, "%c %c", 'm', 'c');
            currState = TURN;
            WriteCommand(command, sizeof(command) / sizeof(command[0]));
        }
    } else if(dir == LEFT) {
        if(initialYaw <= 0.0f && currYaw.m_yaw > 0.0f) { //if robot crosses over from -180 to 180, direction switches
            currYaw.m_yaw -= 360; //range fixing
        }
        if(currYaw.m_yaw <= toTurn) {
            char* command;
            sprintf(command, "%c %c", 'm', 'c');
            currState = TURN;
            WriteCommand(command, sizeof(command) / sizeof(command[0]));
        }
    }
    
}

void ARobot::ParseIMU()
{
    for(int i = 0; i < imuParseList.size(); i++)
    {
        imuParseList.front().parseData();
        imuParseList.front().runFilter();
        imuDataList.push_back(imuParseList.front());
        imuParseList.pop();
    }
}
void ARobot::ParseRange() {
    rangeParseList.front().parseData();
    rangeParseList.front().getPosition();
    rangeDataList.push_back(rangeParseList.front());
    rangeParseList.pop();
}

void ARobot::ParseTemp() {
    tempParseList.front().parseData();
    tempParseList.front()->checkTemp();
    tempDataList.push_back(tempParseList.front());
    tempParseList.pop();
}

void ARobot::ParseLight() {
    lightParseList.front().parseData();
    lightParseList.front().checkLight();
    lightDataList.push_back(lightParseList.front());
    lightParseList.pop();
}

void ARobot::ClearIMU()
{
    mlen_imu = imuDataList.size();
    while(mlen_imu > 200) {
        imuDataList.erase(imuDataList.begin(), imuDataList.begin() + mlen_imu - 200);
    }
}

void ARobot::ClearRange()
{
    mlen_range = rangeDataList.size();
    while(mlen_range > 200) {
        rangeDataList.erase(rangeDataList.begin(), rangeDataList.begin() + mlen_range - 200);
    }
}

void ARobot::ClearTemp()
{
    mlen_temp = tempDataList.size();
    while(mlen_temp > 200) {
        tempDataList.erase(tempDataList.begin(), tempDataList.begin() + mlen_temp - 200);
    }
}


