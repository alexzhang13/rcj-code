#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "ARobot.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"
#include <vector>

using namespace std;

ARobot::~ARobot() 
{

}


void ARobot::WriteCommand(char* command, int size)
{
    port->write(command, size);
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

void ARobot::checkLight()
{
    if(lightDataList.end().checkLight() == 2) {
        if(lightDataList.end().checkLight() == 2) {
           ++lightCounter;
        } else {
            lightCounter = 0;
        }
        if(lightCounter >= 3) {
            currTileLight = SILVER;
        }
    } else if (lightDataList.end().checkLight() == 1) {
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
}

void ARobot::LEDLight(int time)
{
    char* command;
    sprintf(command, "%c %c %d", 'd', 'b', time);
    WriteCommand(command, command.length());
    currState = LED;
}

void ARobot::Drop()
{
    char* command;
    sprintf(command, "%c %c", 'd', 'a');
    WriteCommand(command, command.length());
    currState = DROP;
}

void ARobot::SetSpeed(int left_speed, int right_speed) {
    char* command;
    sprintf(command, "%c %c %d %d", 'm', 'f', left_speed, right_speed);
    WriteCommand(command, command.length());
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
    WriteCommand(command, command.length());
}
void ARobot::TurnDistance(int degrees, BotDir dir)
{
    initialYaw = imuDataList.end().m_yaw;
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
    WriteCommand(command, command.length());
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
            WriteCommand(command, command.length());
        }
    } else if(dir == LEFT) {
        if(initialYaw <= 0.0f && currYaw.m_yaw > 0.0f) { //if robot crosses over from -180 to 180, direction switches
            currYaw.m_yaw -= 360; //range fixing
        }
        if(currYaw.m_yaw <= toTurn) {
            char* command;
            sprintf(command, "%c %c", 'm', 'c');
            currState = TURN;
            WriteCommand(command, command.length());
        }
    }
    
}

void ARobot::ParseIMU()
{
    imuParseList.front().parseData();
    imuParseList.front().runFilter();
    imuDataList.push_back(imuParseList.front());
    imuParseList.pop();
}
void ARobot::ParseRange() {
    rangeParseList.front().parseData();
    rangeParseList.front().getPosition();
    rangeDataList.push_back(rangeParseList.front());
    rangeParseList.pop();
}

void ARobot::ParseTemp() {
    tempParseList.front().parseData();
    tempDataList.push_back(tempParseList.front());
    //tempDataList.front().checkTemp();
    tempParseList.pop();
}

void ARobot::ParseLight() {
    lightParseList.front().parseData();
    lightDataList.push_back(lightParseList.front());
    //lightDataList.front().checkLight();
    lightParseList.pop();
}

void ARobot::ClearVectors()
{
    while(imuDataList.size() > 200) {
        pop_front(imuDataList);
    }
    while(rangeDataList.size() > 200) {
        pop_front(rangeDataList);
    }
    while(tempDataList.size() > 200) {
        pop_front(tempDataList);
    }
    while(lightDataList.size() > 200) {
        pop_front(lightDataList);
    }
}
//function for popping the front of a vector
template<typename T>
void ARobot::pop_front(std::vector<T>& vec)
{
    assert(!vec.empty());
    vec.erase(vec.begin());
}