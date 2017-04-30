#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "ARobot.h"
#include <math.h>

using namespace std;

ARobot::ARobot(SerialPort *port) :mPort(port)
{
    initialYaw = 0;
    toTurn = 0;
    backingBlack = false;
    currTileLight = WHITE;
    currDir = FRONT; 
    victimDir = MazeCell::NotDecided;
    currState = PLANNING;
    currTile.x = 0;
    currTile.y = 0;
}

ARobot::~ARobot() 
{

}

void ARobot::WriteCommand(char* command, int size)
{
    mPort->write(command, size);
}

void ARobot::UpdateCellMap(MazeCell *sensor_info)
{
    if(currTileLight == SILVER) {
        sensor_info->setCheckPt(true);
        sensor_info->setNonMovable(false);
    } else { //black is a different case *WHITE
        sensor_info->setCheckPt(false);
        sensor_info->setNonMovable(false);
    }
    if(checkRamp() == true) {
        sensor_info->setStairCell(true);
    } else {sensor_info->setStairCell(false);}
    if(checkVictimTemp() == true) {
        //sensor_info->setVictim(true);
        //sensor_info->setVictimDirection(victimDir);
    } else {
        sensor_info->setVictim(false);
    }

    /*WALL DATA*/
    if(rangeDataList.end()->walls.wallN > 0) {
        sensor_info->setWallNorth(MazeCell::MOpen);
    } else if(rangeDataList.end()->walls.wallN == 0) {
        sensor_info->setWallNorth(MazeCell::MWall); 
    } else {sensor_info->setWallNorth(MazeCell::MUnknown);} //rare and not usual case

    if(rangeDataList.end()->walls.wallE > 0) {
        sensor_info->setWallEast(MazeCell::MOpen);
    } else if(rangeDataList.end()->walls.wallE == 0) {
        sensor_info->setWallEast(MazeCell::MWall); 
    } else {sensor_info->setWallEast(MazeCell::MUnknown);} //rare and not usual case

    if(rangeDataList.end()->walls.wallS > 0) {
        sensor_info->setWallSouth(MazeCell::MOpen);
    } else if(rangeDataList.end()->walls.wallS == 0) {
        sensor_info->setWallSouth(MazeCell::MWall); 
    } else {sensor_info->setWallSouth(MazeCell::MUnknown);} //rare and not usual case

    if(rangeDataList.end()->walls.wallW > 0) {
        sensor_info->setWallWest(MazeCell::MOpen);
    } else if(rangeDataList.end()->walls.wallW == 0) {
        sensor_info->setWallWest(MazeCell::MWall); 
    } else {sensor_info->setWallWest(MazeCell::MUnknown);} //rare and not usual case

}

void ARobot::UpdateNeighborCells()
{
    MazeCell temp_cell;
    size_t sizeRange = rangeDataList.size();
    bool wallsN = true;
    bool wallsE = true;
    bool wallsS = true;
    bool wallsW = true; //if valid/usable data
    for(int i = 1; i < 6; i++) {
        if(rangeDataList[sizeRange-i].walls.wallN != rangeDataList[sizeRange-1-i].walls.wallN)
            wallsN = false;
            break;
    }
    for(int i = 1; i < 6; i++) {
        if(rangeDataList[sizeRange-i].walls.wallE != rangeDataList[sizeRange-1-i].walls.wallE)
            wallsE = false;
            break;
    }
    for(int i = 1; i < 6; i++) {
        if(rangeDataList[sizeRange-i].walls.wallS != rangeDataList[sizeRange-1-i].walls.wallS)
            wallsS = false;
            break;
    }
    for(int i = 1; i < 6; i++) {
        if(rangeDataList[sizeRange-i].walls.wallW != rangeDataList[sizeRange-1-i].walls.wallW)
            wallsW = false;
            break;
    }
    if(wallsN == true) {
        if(rangeDataList[sizeRange-1].walls.wallN != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallN; i++) {
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                if(i == rangeDataList[sizeRange-i].walls.wallN) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallNorth(MazeCell::MWall);
                } else {
                    temp_cell.setWallNorth(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            for(int i = 1; i <= 2; i++) { //open???
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                temp_cell.setWallNorth(MazeCell::MOpen);
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        }
    }

    if(wallsE == true) {
        if(rangeDataList[sizeRange-1].walls.wallE != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallE; i++) {
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                if(i == rangeDataList[sizeRange-i].walls.wallE) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallEast(MazeCell::MWall);
                } else {
                    temp_cell.setWallEast(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            for(int i = 1; i <= 2; i++) { //open???
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                temp_cell.setWallEast(MazeCell::MOpen);
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        }
    }

    if(wallsS == true) {
        if(rangeDataList[sizeRange-1].walls.wallS != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallS; i++) {
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                if(i == rangeDataList[sizeRange-i].walls.wallS) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallSouth(MazeCell::MWall);
                } else {
                    temp_cell.setWallSouth(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            for(int i = 1; i <= 2; i++) { //open???
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                temp_cell.setWallSouth(MazeCell::MOpen);
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        }
    }

    if(wallsW == true) {
        if(rangeDataList[sizeRange-1].walls.wallW != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallW; i++) {
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                if(i == rangeDataList[sizeRange-i].walls.wallW) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallWest(MazeCell::MWall);
                } else {
                    temp_cell.setWallWest(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            for(int i = 1; i <= 2; i++) { //open???
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                temp_cell.setWallWest(MazeCell::MOpen);
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        }
    }
}

void ARobot::CalcNextTile()
{
    BotOrientation nextDir;
    int next_x = currTile.x_tovisit*300 - currTile.x_map; //next tile coords
    int next_y = currTile.y_tovisit*300 - currTile.y_map; //next tile coords
    int32_t dist = (int32_t)sqrt(next_x*next_x + next_y*next_y); //pythagorean
    float angle; //offset angle
    if(currTile.x_tovisit - currTile.x > 0) { //east
        nextDir = EAST;
        angle = -atan(next_y/next_x)*180.0f/3.1415926535; //angle to right, should be pos
    } else if (currTile.x_tovisit - currTile.x < 0) { //west
        nextDir = WEST;
        angle = -atan(next_y/next_x)*180.0f/3.1415926535; //angle to left, should be neg
    } else if (currTile.y_tovisit - currTile.y > 0) { //north
        nextDir = NORTH;
        angle = atan(next_x/next_y)*180.0f/3.1415926535; //angle to left, should be neg
    } else if (currTile.y_tovisit - currTile.y < 0) { //south
        nextDir = SOUTH;
        angle = -atan(next_x/next_y)*180.0f/3.1415926535; //angle to left, should be neg
    }
    TileTransition(nextDir, angle, dist);

}

void ARobot::TileTransition(BotOrientation direction, float angle, int32_t dist)
{
    
}

bool ARobot::checkRamp()
{
    size_t pitch_vals = imuDataList.size();
    for(int i = 1; i < 5; i++) {
        if(!(abs(imuDataList[pitch_vals-i].m_pitch) >= 15)) {
            return false;
        }
        
    }
    return true;
}

bool ARobot::checkVictimTemp()
{
    size_t temp_vals = tempDataList.size();
    //if(tempDataList[temp_vals-1].checkTemp() == 1) { //left sensor activated

    //}
    return true;
}

void ARobot::setTempThresh(float left, float right)
{
    threshLeft = left;
    threshRight = right;
}

float ARobot::getLeftVictimTemp()
{
    return threshLeft;
}

float ARobot::getRightVictimTemp()
{
    return threshRight;
}

void ARobot::setLightThresh(uint16_t black, uint16_t silver)
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
            MoveDistance(160, BACK); //move back 16 cm
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
    float currYaw = imuDataList.end()->m_yaw;
    if(dir == RIGHT) {
        if(initialYaw >= 0.0f && currYaw < 0.0f) { //if robot crosses over from 180 to -180, direction switches
            currYaw += 360; //range fixing
        }
        if(currYaw >= toTurn) {
            char* command;
            sprintf(command, "%c %c", 'm', 'c');
            currState = TURN;
            WriteCommand(command, sizeof(command) / sizeof(command[0]));
        }
    } else if(dir == LEFT) {
        if(initialYaw <= 0.0f && currYaw > 0.0f) { //if robot crosses over from -180 to 180, direction switches
            currYaw -= 360; //range fixing
        }
        if(currYaw <= toTurn) {
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
    for(int i = 0; i < rangeParseList.size(); i++)
    {
        rangeParseList.front().parseData();
        rangeParseList.front().getPosition();
        if(rangeParseList.front()->coord.x_flag == true) {
            currTile.x_map = (currTile.x*300) + rangeParseList.front()->x_glob;
        }
        if(rangeParseList.front()->y_flag == true) {
            currTile.y_map = (currTile.y*300) + rangeParseList.front()->y_glob;
        }
        rangeDataList.push_back(rangeParseList.front());
        rangeParseList.pop();
    }
}

void ARobot::ParseTemp() {
    for(int i = 0; i < tempParseList.size(); i++)
    {
        tempParseList.front().parseData();
        tempParseList.front().checkTemp();
        tempDataList.push_back(tempParseList.front());
        tempParseList.pop();
    }
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