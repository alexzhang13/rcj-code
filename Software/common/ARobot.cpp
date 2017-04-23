#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "ARobot.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "UartRx.h"
#include "UartTx.h"

using namespace std;

ARobot::~ARobot() 
{

}

void ARobot::WriteCommand(char* command, int size)
{
    port->write(command, size);
}

void ARobot::SetSpeed(int left_speed, int right_speed) {
    char* command;
    sprintf(command, "%c %c %d %d", 'm', 'f', left_speed, right_speed);
    WriteCommand(command, command.length());
}

void ARobot::MoveDistance(int distance_mm, bool forward) //forward = true
{
    char* command;
    if(forward == true) {
        sprintf(command, "%c %c %d", 'm', 'a', distance_mm);
    } else {
        sprintf(command, "%c %c %d", 'm', 'b', distance_mm);
    }
    isMoving = true;
    WriteCommand(command, command.length());
}
void ARobot::TurnDistance(int degrees, bool right)
{
    currentYaw = imuDataList.end().m_yaw;
    char* command;
    if(right == true) {
        //sprintf(command, "%c %c %d", 'm', 'd', distance_mm);
        toTurn = currentYaw + degrees;
    } else {
        //sprintf(command, "%c %c %d", 'm', 'e', distance_mm);
        toTurn = currentYaw - degrees;
    }
    isMoving = true;
    isTurning = true;
    WriteCommand(command, command.length());
}

void ARobot::StopTurn(bool right)
{
    if(imuDataList.end().m_yaw >= toTurn) {
        char* command;
        sprintf(command, "%c %c", 'm', 'c');
        isMoving = false;
        isTurning = false;
        WriteCommand(command, command.length());
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
    tempParseList.front().getPosition();
    tempDataList.push_back(tempParseList.front());
    tempParseList.pop();
}

void ARobot::ParseLight() {
    lightParseList.front().parseData();
    lightParseList.front().checkLight();
    lightDataList.push_back(lightParseList.front());
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
