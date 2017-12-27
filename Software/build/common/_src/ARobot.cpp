#include "../_headers/ARobot.h"

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

using namespace std;

ARobot::ARobot(SerialPort *port) :mPort(port)
{
    this->backingBlack = false;
	this->black_thresh=0;
	this->cross_over = false;
    this->currTileLight = WHITE;
    this->currDir = FRONT;
    this->currState = PLANNING; //>9 = testing purposes
    this->currOrientation = NORTH;
    this->currTile.x = 0;
    this->currTile.y = 0;
    this->dist_temp=0;
    this->dropCnt = 0; //dropping counter
	this->initialYaw = 0;
    this->isDropped = false;
    this->isVictim = false;
    this->mlen_imu = 0;
    this->mlen_light = 0;
    this->mlen_range = 0;
    this->mlen_temp = 0;
    this->m_letter = ' ';
    this->off_left = 5;
    this->off_right = 25;
    this->silver_thresh = 0;
    this->speed_left = 100;
    this->speed_right = 120;
    this->threshLeft = 0;
    this->threshRight = 0;
    this->toMove = false;
    this->toTurn = 0;
    this->victim.letter = '0';
    this->victim.m_isVictim = false;
    this->victimDir = MazeCell::NotDecided;
    this->victimFront = false;
    this->victimLeft = false;
    this->victimRight = false; //true if is dropping to the right
    this->yaw_drift = 0;
}

ARobot::~ARobot() 
{

}

void ARobot::WriteCommand(char* i_command, int size)
{
    mPort->write(i_command, size);
}

void ARobot::UpdateCellMap(MazeCell *sensor_info, bool black_flag)
{
    size_t range_size = rangeDataList.size();
    if(!black_flag) {
        if(currTileLight == SILVER) {
            sensor_info->setCheckPt(true);
            sensor_info->setNonMovable(false);
        } else { //black is a different case *WHITE
            sensor_info->setCheckPt(false);
            sensor_info->setNonMovable(false);
        }
        if(CheckRamp()) {
            sensor_info->setStairCell(true);
        } else {sensor_info->setStairCell(false);}
        if(victimRight) {
            sensor_info->setVictim(true);
            sensor_info->setVictimDirection(MazeCell::NavDir(((int)currOrientation + 1)%4));
            victimRight = false;
        } else if (victimFront) {
            sensor_info->setVictim(true);
            sensor_info->setVictimDirection(MazeCell::NavDir((int)currOrientation));
            victimFront = false;
        } else if (victimLeft){
            sensor_info->setVictim(true);
            sensor_info->setVictimDirection(MazeCell::NavDir(((int)currOrientation + 3)%4));
            victimLeft = false;
        } else {
            sensor_info->setVictim(false);
        }

        /*WALL DATA*/
        printf("Walls: N: %d, E: %d, S: %d, W: %d\n", rangeDataList[range_size-1].walls.wallN, rangeDataList[range_size-1].walls.wallE, rangeDataList[range_size-1].walls.wallS, rangeDataList[range_size-1].walls.wallW);
        if(rangeDataList[range_size-1].walls.wallN == 0) {
            sensor_info->setWallNorth(MazeCell::MWall);
        } else {
             sensor_info->setWallNorth(MazeCell::MOpen);
        }

        if(rangeDataList[range_size-1].walls.wallE == 0) {
            sensor_info->setWallEast(MazeCell::MWall); 
        } else {
            sensor_info->setWallEast(MazeCell::MOpen);
        }

        if(rangeDataList[range_size-1].walls.wallS == 0) {
            sensor_info->setWallSouth(MazeCell::MWall);
        } else {
            sensor_info->setWallSouth(MazeCell::MOpen);
        } 

        if(rangeDataList[range_size-1].walls.wallW == 0) {
            sensor_info->setWallWest(MazeCell::MWall);
        } else {
            sensor_info->setWallWest(MazeCell::MOpen);
        } 
        sensor_info->setVisitStatus(MazeCell::Visited);
    } else {
        sensor_info->setNonMovable(true);
        sensor_info->setCheckPt(false);
        sensor_info->setVictim(false);
        sensor_info->setStairCell(false);
        sensor_info->setVisitStatus(MazeCell::Visited);
        printf("black!");
    }

}

void ARobot::UpdateNeighborCells()
{
    MazeCell temp_cell;
    size_t sizeRange = rangeDataList.size();
    if(sizeRange < 4)
	return;

    bool wallsN = true;
    bool wallsE = true;
    bool wallsS = true;
    bool wallsW = true; //if valid/usable data

    for(int i = 1; i < 4; i++) {
        if(rangeDataList[sizeRange-i].walls.wallN != rangeDataList[sizeRange-1-i].walls.wallN)
            wallsN = false;
            break;
    }
    for(int i = 1; i < 4; i++) {
        if(rangeDataList[sizeRange-i].walls.wallE != rangeDataList[sizeRange-1-i].walls.wallE)
            wallsE = false;
            break;
    }
    for(int i = 1; i < 4; i++) {
        if(rangeDataList[sizeRange-i].walls.wallS != rangeDataList[sizeRange-1-i].walls.wallS)
            wallsS = false;
            break;
    }
    for(int i = 1; i < 4; i++) {
        if(rangeDataList[sizeRange-i].walls.wallW != rangeDataList[sizeRange-1-i].walls.wallW)
            wallsW = false;
            break;
    }
 
    if(wallsN == true) {
        if(rangeDataList[sizeRange-1].walls.wallN != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallN; i++) {
                temp_cell.setCellGrid(currTile.x, currTile.y+i);
                if(i == rangeDataList[sizeRange-1].walls.wallN) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallNorth(MazeCell::MWall);
                } else {
                    temp_cell.setWallNorth(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            temp_cell.setCellGrid(currTile.x, currTile.y+1);
            temp_cell.setWallSouth(MazeCell::MOpen);
            temp_cell_list.push_back(temp_cell);
            temp_cell.reset();
        }
    }

    if(wallsE == true) {
        if(rangeDataList[sizeRange-1].walls.wallE != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallE; i++) {
                temp_cell.setCellGrid(currTile.x+i, currTile.y);
                if(i == rangeDataList[sizeRange-1].walls.wallE) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallEast(MazeCell::MWall);
                } else {
                    temp_cell.setWallEast(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            temp_cell.setCellGrid(currTile.x+1, currTile.y);
            temp_cell.setWallWest(MazeCell::MOpen);
            temp_cell_list.push_back(temp_cell);
            temp_cell.reset();
        }
    }

    if(wallsS == true) {
        if(rangeDataList[sizeRange-1].walls.wallS != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallS; i++) {
                temp_cell.setCellGrid(currTile.x, currTile.y-i);
                if(i == rangeDataList[sizeRange-1].walls.wallS) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallSouth(MazeCell::MWall);
                } else {
                    temp_cell.setWallSouth(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            temp_cell.setCellGrid(currTile.x, currTile.y-1);
            temp_cell.setWallNorth(MazeCell::MOpen);
            temp_cell_list.push_back(temp_cell);
            temp_cell.reset();
        }
    }


    if(wallsW == true) {
        if(rangeDataList[sizeRange-1].walls.wallW != -1) {
            for(int i = 1; i <= rangeDataList[sizeRange-i].walls.wallW; i++) {
                temp_cell.setCellGrid(currTile.x-i, currTile.y);
                if(i == rangeDataList[sizeRange-1].walls.wallW) { //There is a wall at the reading point/furthest reading
                    temp_cell.setWallWest(MazeCell::MWall);
                } else {
                    temp_cell.setWallWest(MazeCell::MOpen);
                }
                temp_cell_list.push_back(temp_cell);
                temp_cell.reset();
            }
        } else {
            temp_cell.setCellGrid(currTile.x-1, currTile.y);
            temp_cell.setWallEast(MazeCell::MOpen);
            temp_cell_list.push_back(temp_cell);
            temp_cell.reset();
        }
    }
}

void ARobot::CalcNextTile()
{
    BotOrientation nextDir;
    int next_x = (currTile.x_tovisit*300+150) - (currTile.x*300) - (int)rangeDataList[rangeDataList.size()-1].coord.x_glob; //next tile coords
    int next_y = (currTile.y_tovisit*300+150) - (currTile.y*300) - (int)rangeDataList[rangeDataList.size()-1].coord.y_glob; //next tile coords
    printf("Next_x: %d, Next_y, %d, ToVisit_X: %d, ToVisit_Y: %d, X: %f, Y: %f\n", next_x, next_y, currTile.x_tovisit, currTile.y_tovisit, currTile.x_map, currTile.y_map);
    int32_t dist = (int32_t)sqrt(next_x*next_x + next_y*next_y); //pythagorean
    float angle; //offset angle degrees
    if(currTile.x_tovisit - currTile.x > 0) { //east
    	angle = atan((float)next_y/(float)next_x)*180.0f/3.1415926535f; //angle to left, should be pos
        nextDir = EAST;
    } else if (currTile.x_tovisit - currTile.x < 0) { //west
    	angle = atan((float)next_y/(float)next_x)*180.0f/3.1415926535f; //angle to left, should be pos
        nextDir = WEST;
    } else if (currTile.y_tovisit - currTile.y > 0) { //north
    	angle = -atan((float)next_x/(float)next_y)*180.0f/3.1415926535f; //angle to left, should be pos
        nextDir = NORTH;
    } else if (currTile.y_tovisit - currTile.y < 0) { //south
    	angle = -atan((float)next_x/(float)next_y)*180.0f/3.1415926535f; //angle to left, should be pos
        nextDir = SOUTH;
    }

    int turnNext = (int)currOrientation - (int)nextDir; //left is pos, right is neg
    /*Turning first*/
    if(turnNext == 3) turnNext = -1; //west -> north = turn right 1
    else if (turnNext == -3) turnNext = 1; //north -> west = turn left 1

    angle *= (turnNext>=0) ? 1 : -1; //determine direction of offset (left >= 0)

    toTurn = turnNext*90+(int)angle; //turning distance


    //printf("Next_X: %d, Next_Y: %d, Dist: %d, Angle Dif: %f\n", next_x, next_y, dist, angle);
    //printf("To_X: %d, To_Y: %d, Curr_X: %f, Curr_Y: %f\n", currTile.x_tovisit, currTile.y_tovisit, currTile.x_map, currTile.y_map);
    TileTransition(dist);

}

void ARobot::TileTransition(int32_t dist)
{
    if(abs(toTurn) > 3) { //ignore smaller angles
        TurnDistance(abs(toTurn), (toTurn > 0) ? LEFT : RIGHT); //left is positive for IMU
        dist_temp = dist;
        toMove = true;
        return;
    }

    MoveDistance(dist, FRONT);
    return;
}

void ARobot::SpinLaser() {
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c", 'r', 'a') + 1;
    i_command = (char*)malloc(i_length);

    snprintf(i_command, i_length, "%c %c", 'r', 'a');
    WriteCommand(i_command, i_length);
}
bool ARobot::CheckRamp()
{
    size_t pitch_vals = imuDataList.size();
    for(int i = 1; i < 5; i++) {
        if(!(abs(imuDataList[pitch_vals-i].m_pitch) <= 15)) { //if not ramp, break (return false)
            return false;
        } else if (!(abs(imuDataList[pitch_vals-i].m_pitch) >= 345)) {
            return false;
        }
        
    }
    return true; //ramp is true if past 5 pitches match > 15 degrees
}

int ARobot::CheckVictimTemp()
{
    if(isVictim) //if a victim has already been there
        return 0;

    size_t temp_vals = tempDataList.size(); //get average values
    float temp_avg = 0;
    for(int i = 1; i < 5; i++) { //left threshold
        temp_avg += tempDataList[temp_vals-i].getLeftTemp();
    }
    if(temp_avg/4.0f > threshLeft) {
        return 2;
    }
    temp_avg = 0; //reset
    for(int i = 1; i < 5; i++) { //right threshold
        temp_avg += tempDataList[temp_vals-i].getRightTemp();
    }
    if(temp_avg/4.0f > threshRight) {
        return 1;
    }
    return 0;
}

void ARobot::CheckVictimVisual() {
    for(int i = 0; i < picam.getImageList()->size(); i++) {
        imgList.push_back(picam.getImageList()->at(i));
    }
    printf("List Size: %d\n", imgList.size());
}

int ARobot::ProcessImage_Victim() {
    victim.letter = '0'; //reset
    victim.m_isVictim = false;
    for(int i = 0; i < imgList.size(); i++) {
        m_letter = knn.detectVictim(imgList[i]);
        if(m_letter != '0' && victim.m_isVictim == true) { //error, not supposed to happen, means there is a mistake
            victim.m_isVictim = false;
            isVictim = false;
            break;
        }
        if(m_letter != '0') {
            victim.letter = m_letter;
            if(i == 0) {
                victim.dir_victim = LEFT;
            } else if(i == 1) {
                victim.dir_victim = FRONT;
            } else {
                victim.dir_victim = RIGHT;
            }
            victim.m_isVictim = true;
            isVictim = true;
        }
    }
    ClearImgList();
    picam.resetFrameBuffers();
    if(victim.m_isVictim == true) {
        if(victim.dir_victim == RIGHT) {
            return 2;
        } else if (victim.dir_victim == LEFT) {
            return 0;
        } else if (victim.dir_victim == FRONT) {
            return 1;
        }
    }
    return -1;
}

void ARobot::ClearImgList() {
    imgList.clear();
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

void ARobot::CheckLightTile()
{
    if(backingBlack == true) {return;}
    mlen_light = lightDataList.size();
    if(mlen_light < 3)
        return;

    if(lightDataList[mlen_light-1].checkLight() == 2 && lightDataList[mlen_light-2].checkLight() == 2 && lightDataList[mlen_light-3].checkLight() == 2) {
        currTileLight = SILVER;
    } else if (lightDataList[mlen_light-1].checkLight() == 1 && lightDataList[mlen_light-2].checkLight() == 1 && lightDataList[mlen_light-3].checkLight() == 1) {
        currTileLight = BLACK;
        if(backingBlack == false) {
            backingBlack = true;
            ResetEncoder();
            sleep(1);
            MoveDistance(155, BACK);
        }
    } else {
        currTileLight = WHITE;
    }
    if(mlen_light > 200) 
        lightDataList.erase(lightDataList.begin(), lightDataList.begin() + mlen_light - 200);
    return;
}

void ARobot::LEDLight(int time)
{
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c %d", 'd', 'b', time) + 1;
    i_command = (char*)malloc(i_length);

    snprintf(i_command, i_length, "%c %c %d", 'd', 'b', time);
    WriteCommand(i_command, i_length);
}

void ARobot::Drop()
{
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c", 'd', 'a') + 1;
    i_command = (char*)malloc(i_length);

    snprintf(i_command, i_length, "%c %c", 'd', 'a');
    WriteCommand(i_command, i_length);
}

void ARobot::setSpeed(int left_speed, int right_speed) {
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c %d %d", 'm', 'f', left_speed, right_speed) + 1;
    i_command = (char*)malloc(i_length);

    snprintf(i_command, i_length, "%c %c %d %d", 'm', 'f', left_speed, right_speed);
    WriteCommand(i_command, i_length);
}

void ARobot::setOffsetSpeed(int offset_l, int offset_r) {
	char* i_command;
	int i_length = snprintf(NULL, 0, "%c %c %d %d", 'm', 'i', offset_l, offset_r) + 1;
	i_command = (char*)malloc(i_length);

	snprintf(i_command, i_length, "%c %c %d %d", 'm', 'i', offset_l, offset_r);
	WriteCommand(i_command, i_length);
}

void ARobot::MoveDistance(int distance_mm, BotDir dir) //forward = true
{
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c %d", 'm', 'a', distance_mm) + 1;
    i_command = (char*)malloc(i_length);

    if(dir == FRONT) {
        snprintf(i_command, i_length, "%c %c %d", 'm', 'a', distance_mm);
    } else {
        snprintf(i_command, i_length, "%c %c %d", 'm', 'b', distance_mm);
    }
    if(!(currState == RAMP)) {
        currState = MOVE;
    }
    printf("Distance: %d\n", distance_mm);
    WriteCommand(i_command, i_length);
}

void ARobot::ResetEncoder() {
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c", 'm', 'g') + 1;
    i_command = (char*)malloc(i_length);

    snprintf(i_command, i_length, "%c %c", 'm', 'g');
    WriteCommand(i_command, i_length);
}
void ARobot::TurnDistance(int degrees, BotDir dir)
{
    size_t imu_list = imuDataList.size();
    int i_length = snprintf(NULL, 0, "%c %c", 'm', 'd') + 1;
    char* i_command = (char*)malloc(i_length);
    initialYaw = imuDataList[imu_list-1].m_yaw;
    
    if(dir == RIGHT) {
        snprintf(i_command, i_length, "%c %c", 'm', 'e');
        toTurn = initialYaw - degrees;
        currDir = RIGHT;
    } else {
        snprintf(i_command, i_length, "%c %c", 'm', 'd');
        toTurn = initialYaw + degrees;
        currDir = LEFT;
    }
    currState = TURN;
    printf("Initial Yaw: %f Degrees: %d ToTurn: %f\n", initialYaw, degrees, toTurn);
    WriteCommand(i_command, i_length);
}

void ARobot::StopMove() {
    int i_length = snprintf(NULL, 0, "%c %c", 'm', 'c') + 1;
    char* i_command = (char*)malloc(i_length);
    snprintf(i_command, i_length, "%c %c", 'm', 'c');
    WriteCommand(i_command, i_length);
}

void ARobot::StopTurn(BotDir dir)
{
    size_t imu_list = imuDataList.size();
    float currYaw = imuDataList[imu_list-1].m_yaw;
    if(dir == RIGHT) {
        if(initialYaw <= 180.0f && currYaw > 180.0f) //if robot crosses over from 180 to -180, direction switches
            cross_over = true;
        if(cross_over) //condition holds even if prev doesn't when cross_over is already true
        	currYaw -= 360; //range fixing

        if(currYaw <= toTurn) {
            char* i_command;
            cross_over = false; //default cross bool now off
            int i_length = snprintf(NULL, 0, "%c %c", 'm', 'c') + 1;
            i_command = (char*)malloc(i_length);
            snprintf(i_command, i_length, "%c %c", 'm', 'c');
            WriteCommand(i_command, i_length);
            if(isVictim && isDropped == false) {
                currState = DROP;
            } else {
                currState = IDLE; 
                //CheckVictimTemp();
            }          
            return;
        }
    } else if(dir == LEFT) {
    	if(initialYaw > 180.0f && currYaw <= 180.0f) //if robot crosses over from -180 to 180, direction switches
    		cross_over = true;
        if(cross_over)
        	currYaw += 360; //range fixing

        if(currYaw >= toTurn) {
            char* i_command;
            cross_over = false; //default cross bool now off
            int i_length = snprintf(NULL, 0, "%c %c", 'm', 'c') + 1;
            i_command = (char*)malloc(i_length);
            snprintf(i_command, i_length, "%c %c", 'm', 'c');
            WriteCommand(i_command, i_length);
            if(isVictim && isDropped == false) {
                currState = DROP;
            } else {
                currState = IDLE;
                //CheckVictimTemp();
            }
            return;
        }
    }
    
}

void ARobot::CalibrateIMU()
{
    int i_length = snprintf(NULL, 0, "%c %c", 'i', 'b') + 1;
    char* i_command = (char*)malloc(i_length);
    snprintf(i_command, i_length, "%c %c", 'i', 'b');
    WriteCommand(i_command, i_length);
}

void ARobot::ParseIMU()
{
    /*if(imuDataList.size() <= 0) {
        imuParseList.front().parseData(-1); //special initial case
    } else {
        imuParseList.front().parseData(imuDataList[imuDataList.size()-1].getTStamp());
    }*/
    imuParseList.front().parseData();
    imuParseList.front().runFilter();
    imuDataList.push_back(imuParseList.front());
    imuParseList.pop();
}
void ARobot::ParseRange() {
    rangeParseList.front().parseData();
    rangeParseList.front().getPosition();
    if(rangeParseList.front().coord.x_flag == true) {
        currTile.x_map = (currTile.x*300.0f) + rangeParseList.front().coord.x_glob;
        //printf("X Glob: %f\n", currTile.x_map);
    }
    if(rangeParseList.front().coord.y_flag == true) {
        currTile.y_map = (currTile.y*300.0f) + rangeParseList.front().coord.y_glob;
        //printf("Y Glob: %f\n", currTile.y_map);
    }

    //printf("x: %d y: %d x_map: %d y_map: %d\n", currTile.x, currTile.y, currTile.x_map, currTile.y_map);
    rangeDataList.push_back(rangeParseList.front());
    rangeParseList.pop();
}

void ARobot::ParseTemp() {
    tempParseList.front().parseData();
    tempDataList.push_back(tempParseList.front());
    tempParseList.pop();

}

void ARobot::ParseLight() {
    lightParseList.front().parseData();
    lightDataList.push_back(lightParseList.front());
    lightParseList.pop();
}

void ARobot::ParseScan() {
    scanParseList.front().getScan();
    scanDataList.push_back(scanParseList.front());
    scanParseList.pop();
}

void ARobot::ClearIMU()
{
    mlen_imu = imuDataList.size();
    while(mlen_imu > 200) {
        imuDataList.erase(imuDataList.begin());
        mlen_imu--;
    }
}

void ARobot::ClearRange()
{
    mlen_range = rangeDataList.size();
    while(mlen_range > 200) {
        rangeDataList.erase(rangeDataList.begin());
        mlen_range--;
    }
}

void ARobot::ClearTemp()
{
    mlen_temp = tempDataList.size();
    while(mlen_temp > 200) {
        tempDataList.erase(tempDataList.begin());
        mlen_temp--;
    }
}

void ARobot::ClearLight()
{
    mlen_light = lightDataList.size();
    while(mlen_light > 200) {
        lightDataList.erase(lightDataList.begin());
        mlen_light--;
    }
}

void ARobot::ClearScan()
{
    scanDataList.clear();
}
void ARobot::TestSignal() {
	char* i_command;
	int i_length = snprintf(NULL, 0, "%c %c", 'z', 'a') + 1;
	i_command = (char*)malloc(i_length);
	snprintf(i_command, i_length, "%c %c", 'z', 'a');
	WriteCommand(i_command, i_length);
}
