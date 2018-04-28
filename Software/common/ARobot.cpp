#include "ARobot.h"

#include <string.h>
#include <string>
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
    this->imuCalibrated = false;
    this->initialYaw = 0;
    this->initTurnRec = false;
    this->isDropped = false;
    this->isVictim = false;
    this->mlen_imu = 0;
    this->mlen_light = 0;
    this->mlen_range = 0;
    this->mlen_temp = 0;
    this->m_letter = '0';
    this->off_left = 5;
    this->off_right = 25;
    this->offsetdir = 1;
    this->prevYaw = 0;
    this->silver_thresh = 0;
    this->sLock = 0;
    this->speed_left = 100;
    this->speed_right = 115;
    this->threshLeft = 0;
    this->threshRight = 0;
    this->toMove = false;
    this->toTurn = 0;
    this->turnOffset = 0;
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

void ARobot::UpdateCellMap(MazeCell *sensor_info, bool black_flag, bool CheckRamp)
{
    const size_t range_size = rangeDataList.size();
    int orientflag = (int)currOrientation % 2;
    if(!black_flag) {
        if(currTileLight == SILVER) {
            sensor_info->setCheckPt(true);
            sensor_info->setNonMovable(false);
        } else { //black is a different case *WHITE
            sensor_info->setCheckPt(false);
            sensor_info->setNonMovable(false);
        }
        if(CheckRamp) {
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

        if(rangeDataList[range_size-1].walls.wallN == 0) {
            sensor_info->setWallNorth(MazeCell::MWall);
            cout << "North Wall State: " <<  sensor_info->getWallNorth() << endl;
        } else if(rangeDataList[range_size-1].walls.wallN == -1 && sensor_info->getWallNorth() != MazeCell::MOpen) {
            sensor_info->setWallNorth(MazeCell::MUnknown);
        } else {
            sensor_info->setWallNorth(MazeCell::MOpen);
        }

        if(rangeDataList[range_size-1].walls.wallE == 0) {
            printf("EAST IS A WALL\n");
            sensor_info->setWallEast(MazeCell::MWall);
        } else if (rangeDataList[range_size-1].walls.wallE == -1 && sensor_info->getWallEast() != MazeCell::MOpen) {
            sensor_info->setWallEast(MazeCell::MUnknown);
        } else {
            sensor_info->setWallEast(MazeCell::MOpen);
        }

        if(rangeDataList[range_size-1].walls.wallS == 0) {
            sensor_info->setWallSouth(MazeCell::MWall);
        } else if (rangeDataList[range_size-1].walls.wallS == -1 && sensor_info->getWallSouth() != MazeCell::MOpen) {
            sensor_info->setWallSouth(MazeCell::MUnknown);
        } else {
            sensor_info->setWallSouth(MazeCell::MOpen);
        }

        if(rangeDataList[range_size-1].walls.wallW == 0) {
            sensor_info->setWallWest(MazeCell::MWall);
        } else if (rangeDataList[range_size-1].walls.wallW == -1 && sensor_info->getWallWest() != MazeCell::MOpen) {
            sensor_info->setWallWest(MazeCell::MUnknown);
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
    /*WALL DATA*/
    printf("Walls: N: %d, E: %d, S: %d, W: %d\n", rangeDataList[range_size-1].walls.wallN, rangeDataList[range_size-1].walls.wallE, rangeDataList[range_size-1].walls.wallS, rangeDataList[range_size-1].walls.wallW);

}

void ARobot::UpdateNeighborCells()
{
    MazeCell temp_cell;
    const size_t sizeRange = rangeDataList.size();
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

void ARobot::CalcNextTile(bool first)
{
    BotOrientation nextDir;
    float calculatedNextGlobX=0;
    float calculatedNextGlobY=0;

    //Average the Previous Readings[Assuming Still], TODO: Change Later, Messy Algorithm
    for(int i=2;i<10;i++) {
        calculatedNextGlobX += rangeDataList[rangeDataList.size()-i].coord.x_glob;
        calculatedNextGlobY += rangeDataList[rangeDataList.size()-i].coord.y_glob;
    }
    calculatedNextGlobX /= 8.0f;
    calculatedNextGlobY /= 8.0f;

    //Find Next Coordinates for the Bot
    int next_x = (currTile.x_tovisit*300+150) - (currTile.x*300) - (int)calculatedNextGlobX; //next tile coords
    int next_y = (currTile.y_tovisit*300+150) - (currTile.y*300) - (int)calculatedNextGlobY; //next tile coords
    int32_t dist = (int32_t)sqrt(next_x*next_x + next_y*next_y); //pythagorean
    float angle; //offset angle degrees
    if(currTile.x_tovisit - currTile.x > 0) { //east
        angle = atan((float)next_y/(float)next_x)*180.0f/3.1415926535f; //angle to left, should be neg
        nextDir = EAST;
    } else if (currTile.x_tovisit - currTile.x < 0) { //west
        angle = atan((float)next_y/(float)next_x)*180.0f/3.1415926535f; //angle to left, should be neg
        nextDir = WEST;
    } else if (currTile.y_tovisit - currTile.y > 0) { //north
        angle = -atan((float)next_x/(float)next_y)*180.0f/3.1415926535f; //angle to left, should be pos
        nextDir = NORTH;
    } else if (currTile.y_tovisit - currTile.y < 0) { //south
        angle = -atan((float)next_x/(float)next_y)*180.0f/3.1415926535f; //angle to left, should be pos
        nextDir = SOUTH;
    }

    //Calculate the Bot's turning, including any offsets
    int turnNext = (int)currOrientation - (int)nextDir; //left is pos, right is neg
    if(turnNext == 3) turnNext = -1; //west -> north = turn right 1
    else if (turnNext == -3) turnNext = 1; //north -> west = turn left 1

    angle = turnNext==0 ? 0 : angle/2.0; //if the bot is turning don't correct
    toTurn = turnNext*90 + (int)angle; //turning distance

    //Debugging Stuff
    printf("Calculated X: %f\nCalculated Y: %f\n", calculatedNextGlobX, calculatedNextGlobY);
    printf("Current Orientation: %d\nNext Direction: %d\nTurn Angle: %f", (int)currOrientation, (int)nextDir, toTurn);
    printf("To Travel[X]: %d, To Travel[Y]: %d, Next X-Cell: %d, Next Y-Cell: %d, X-Absolute: %f, Y-Absolute: %f\n", next_x, next_y, currTile.x_tovisit, currTile.y_tovisit, currTile.x_map, currTile.y_map);
    PrintXYCoords((int)calculatedNextGlobX/10, (int)calculatedNextGlobY/10);

    //Move on to actual movement
    currOrientation = nextDir;
    if(!first)
        toTurn = 0;
    TileTransition(dist);

}

void ARobot::TileTransition(int32_t dist)
{
    if(abs(toTurn) > 3.0f) { //ignore smaller angles
        TurnDistance((int)abs(toTurn), (toTurn > 0) ? LEFT : RIGHT); //left is positive for IMU
        dist_temp = dist;
        toMove = true;
        return;
    }
    printf("Calculated Distance: %d\t", dist);
    MoveDistance(dist, FRONT);
    return;
}

/*called only when robot is moving*/
void ARobot::setOffsetDir() {
    const size_t range_vals = rangeDataList.size()-2;
    //uint32_t tstamp = rangeDataList[range_vals].data.tstamp;
    //printf("SetOffsetDir() X: %f\t", rangeDataList[range_vals].coord.x_glob);
    //printf("SetOffsetDir() Y: %f\n", rangeDataList[range_vals].coord.y_glob);
    x_vals.push_back(rangeDataList[range_vals].coord.x);
    y_vals.push_back(rangeDataList[range_vals].coord.y);
    sLock++;
}
int ARobot::getOffsetDir() {
    return offsetdir;
}

void ARobot::CorrectYaw() {
    const size_t range_vals = rangeDataList.size()-1; //size may change, set constant size
    const size_t imu_vals = imuDataList.size()-1;
    float angley;
    float newyaw=0.0;
    this->isCorrecting = true;

    //average of previous vals - Very very messy. The <= 30 is supposed to be a case where it's 360 degrees or 0, and it flucuates. I will fix this later
    for(int i = 0; i < 5; i++) {
        angley = (rangeDataList[range_vals-i].getAngle()*offsetdir + ((4-(int)currOrientation)%4)*90.0);
        if(angley <= 30.0) angley += 360;
        newyaw += rangeDataList[range_vals-i].getAlpha() * angley + (1.0 - rangeDataList[range_vals-i].getAlpha()) * (imuDataList[imu_vals-i].m_yaw > 10 ? imuDataList[imu_vals-i].m_yaw : imuDataList[imu_vals-i].m_yaw + 360.0);
        printf("Angley: %f\tCurrent Alpha: %f\tCurrent New: %f\n", angley, rangeDataList[range_vals-i].getAlpha(), newyaw);
    }
    newyaw /= 5.0;
    if(newyaw>=360)
        newyaw-=360;
    printf("Correction-New Yaw: %f\n", newyaw);
    this->turnOffset = newyaw - (imuDataList[imu_vals].m_yaw - this->turnOffset);
    imuDataList[imu_vals].setYawOffset(this->turnOffset);
    imuDataList[imu_vals].m_yaw = newyaw; //after change

    this->correctionError = rangeDataList[range_vals-1].getRangeOffset();
    Correction();
}

void ARobot::Correction() {
    const size_t yaw_vals = imuDataList.size()-1; //size may change, set constant size
    float currYaw = imuDataList[yaw_vals].m_yaw;

    //Assume correction is only necessary in the range -90 degrees -> +90 degrees
    printf("Current Orientation: %d\nCurr Yaw: %f\n", (int)currOrientation, currYaw);
    switch((int)currOrientation) {
    case 0: //Bot facing North
        if(currYaw >= 180) currYaw -= 360; //negative range
        if(abs(0.0f-currYaw) >= 2.0f) {
            this->correctionDir = (0.0f-currYaw > 0.0f) ? LEFT : RIGHT;
            TurnDistance((int)abs(0.0f-currYaw), this->correctionDir); //If yaw is negative, robot is on right side, so turn left, and vice versa
            return;
        }
        break;
    case 1: //Bot facing East
        if(abs(270.0f-currYaw) >= 2.0f) {
            this->correctionDir = (270.0f-currYaw > 0.0f) ? LEFT : RIGHT;
            TurnDistance((int)abs(270.0f-currYaw), this->correctionDir); //If 270-yaw is positive, robot is on right side, so turn left, and vice versa
            return;
        }
        break;
    case 2: //Bot facing South
        if(abs(180.0f-currYaw) >= 2.0f) {
            this->correctionDir = (180.0f-currYaw > 0.0f) ? LEFT : RIGHT;
            TurnDistance((int)abs(180.0f-currYaw), this->correctionDir); //If 180-yaw is positive, robot is on right side, so turn left, and vice versa
            return;
        }
        break;
    case 3: //Bot facing West
        if(abs(90.0f-currYaw) >= 2.0f) {
            this->correctionDir = (90.0f-currYaw > 0.0f) ? LEFT : RIGHT;
            TurnDistance((int)abs(90.0f-currYaw), this->correctionDir); //If 90-yaw is positive, robot is on right side, so turn left, and vice versa
            return;
        }
        break;
    default:
        return;
    }
    this->currState = ARobot::WAYPTNAV; //if fails
    return;

}

void ARobot::CheckCorrection() {
    int currentError = rangeDataList[rangeDataList.size()-1].getRangeOffset();
    if(correctionError > currentError && currentError < 5) {
        printf("PrevError: %d\tCurrError: %d\n", correctionError, currentError);
        this->correctionFailed = false; //correction finished
        this->isCorrecting = false;
        return;
    } else {
        this->correctionFailed = true; //automatically set this parameter for correction
        return;
    }
}

void ARobot::CorrectionFailed() { //if correction was faulty try to change
    //check if everything is good
    int currentError = rangeDataList[rangeDataList.size()-1].getRangeOffset();
    printf("Current Error: %d\tPrevious Error: %d\n", currentError, this->correctionError);

    if(this->correctionDir == RIGHT) {
        if(currentError < this->correctionError) { //correct direction
            this->FixYaw(2.0);
            TurnDistance(2.0, RIGHT); //correct correction
        } else if(currentError == this->correctionError) {
            this->correctionFailed = false; //correction finished
            this->isCorrecting = false;
        } else { //bad case, means something went wrong
            this->FixYaw(-2.0);
            this->correctionFailed = false; //correction finished
            this->isCorrecting = false;
            TurnDistance(2.0, LEFT); //overshot correction
        }
    } else { //right
        if(currentError < this->correctionError) { //good case, 3.0 margin
            this->FixYaw(-2.0);
            TurnDistance(2.0, LEFT); //correct correction
        } else if(currentError == this->correctionError) {
            this->correctionFailed = false; //correction finished
            this->isCorrecting = false;
        } else { //bad case, means something went wrong
            this->FixYaw(2.0);
            this->correctionFailed = false; //correction finished
            this->isCorrecting = false;
            TurnDistance(2.0, RIGHT); //correct correction
        }
    }
    this->correctionError = rangeDataList[rangeDataList.size()-1].getRangeOffset();
    return;
}

//https://www.easycalculation.com/statistics/learn-regression.php
int ARobot::SlopeDir() {
    const int n = this->x_vals.size();
    if(n<=0) {
        printf("Size is 0\n");
        return 1;
    }

    const auto s_x = std::accumulate(this->x_vals.begin(), this->x_vals.end(), 0.0);
    const auto s_y = std::accumulate(this->y_vals.begin(), this->y_vals.end(), 0.0);
    const auto s_xx = std::inner_product(this->x_vals.begin(), this->x_vals.end(), this->x_vals.begin(), 0.0);
    const auto s_xy = std::inner_product(this->x_vals.begin(), this->x_vals.end(), this->y_vals.begin(), 0.0);
    const auto numer = n * s_xy - s_x * s_y;
    const auto denom = n * s_xx - s_x * s_x;
    auto a = numer/denom;

    //printf("n: %d\tSummed X: %f\tSummed Y: %f\tSummed X^2: %f\tSummed XY: %f\tNumerator: %lf\tDenominator: %lf\tA: %lf\n", n, s_x, s_y, s_xx, s_xy, numer, denom, a);
    if(a == 0) return 1;
    return a / abs(a); //1 or -1
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
    int check = 0;
    size_t pitch_vals = imuDataList.size()-1;
    for(int i = 1; i < 5; i++) {
        if(imuDataList[pitch_vals-i].m_pitch >= 10 && imuDataList[pitch_vals-i].m_pitch <= 350) { //if not ramp, break (return false)
            ++check;
        }
    }
    if(check > 3)
        return true; //ramp is true if past 5 pitches match > 15 degrees
    return false;
}

int ARobot::CheckVictimTemp()
{
    if(isVictim) //if a victim has already been there
        return 0;
    size_t temp_vals = tempDataList.size(); //get average values
    int numAboveThreshR = 0; //multiple values above threshold [at least 1/2]
    int numAboveThreshL = 0;
    for(int i = 2; i < 6; i++) {
        for(int k = 1; k < 9; k++) { //left threshold
            cout << "i: " << i << " k: " << k << "Left Temp: " << tempDataList[temp_vals-i].getLeftTemp(k) << endl;
            if(tempDataList[temp_vals-i].getLeftTemp(k) > this->threshLeft) {
                ++numAboveThreshL;
            }
            if(tempDataList[temp_vals-i].getRightTemp(k) > this->threshRight) {
                ++numAboveThreshR;
            }
        }
        if(numAboveThreshL < 3 && numAboveThreshR < 3) {
            return 0;
        }
        if(i < 4) {
            numAboveThreshL=0; //reset
            numAboveThreshR=0; //reset
        }
    }
    if(numAboveThreshL >= 3) { //after test (4*8)
        return 2;
    } else if (numAboveThreshR >= 3) {
        return 1;
    } else {
        return 0;
    }
}

void ARobot::CheckVictimVisual() {
}

/**
 * Tester Function
 */
void ARobot::DisplayVictimVisual() {

}

int ARobot::ProcessImage_Victim() {
    this->victim.m_isVictim = false;
    this->victim.letter = '0';
    this->dropCnt = 0;

    //Check LEFT Visual Victim First
    this->pystream = popen(pyLeft, "r" );
    fgets(leftVVictim, 10, pystream);
    fprintf(stdout, "%c", this->leftVVictim[0]);
    pclose(pystream);

    if(leftVVictim[0] == 'H' || leftVVictim[0] == 'S' || leftVVictim[0] == 'U') {
        this->victim.letter = leftVVictim[0];
        printf("Left Victim Detected with Letter: %c\n", leftVVictim[0]);
        this->victim.dir_victim = LEFT;
        this->victim.m_isVictim = true;
        if(leftVVictim[0] == 'U') {
            return 3;
        } else if(leftVVictim[0] == 'S') {
            this->dropCnt = 1;
        } else {
            this->dropCnt = 2;
        }
        return 0;
    }
#if 0
    //Check RIGHT Visual Victim First
    this->pystream = popen(pyRight, "r" );
    fgets(rightVVictim, 10, pystream);
    fprintf(stdout, "%c", this->rightVVictim[0]);
    pclose(pystream);

    if(rightVVictim[0] == 'H' || rightVVictim[0] == 'S' || rightVVictim[0] == 'U') {
        this->victim.letter = rightVVictim[0];
        printf("Right Victim Detected with Letter: %c\n", rightVVictim[0]);
        this->victim.dir_victim = RIGHT;
        this->victim.m_isVictim = true;
        if(rightVVictim[0] == 'U') {
            return 3;
        } else if(rightVVictim[0] == 'S') {
            this->dropCnt = 1;
        } else {
            this->dropCnt = 2;
        }
        return 2;
    }
#endif
    return 3;
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
    if(backingBlack)
        return;
    mlen_light = lightDataList.size();
    if(mlen_light < 3)
        return;

    if (lightDataList[mlen_light-1].CheckLight() == 1 && lightDataList[mlen_light-2].CheckLight() == 1 && lightDataList[mlen_light-3].CheckLight() == 1) {
        currTileLight = BLACK;
        if(backingBlack == false) {
            backingBlack = true;
            sleep(0.2);
            StopMove();
            sleep(0.2);
            ResetEncoder();
            sleep(1.0);
            MoveDistance(175, BACK);
        }
    } else {
        //Calculate Previous
        int prevVals[5];
        int avgVal;
        for(int i=1;i<5;i++) {
            prevVals[i] = lightDataList[mlen_light-i-1].ReturnLight();
            avgVal += prevVals[i];
        }
        avgVal /= (sizeof(prevVals)/sizeof(prevVals[0]));

        //Standard Deviation
        //float std = this->getSTD(prevVals, avgVal);
        if(lightDataList[mlen_light-2].CheckLight(avgVal)==2) //10.0 calculated from recorded values
            currTileLight = SILVER;
        else
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
        printf("Forward: Distance: %d\n", distance_mm);
    } else {
        snprintf(i_command, i_length, "%c %c %d", 'm', 'b', distance_mm);
        printf("Backward: Distance: %d\n", distance_mm);
    }
    if(!(currState == RAMP)) {
        currState = MOVE;
    }
    WriteCommand(i_command, i_length);
}

void ARobot::ResetEncoder() {
    char* i_command;
    int i_length = snprintf(NULL, 0, "%c %c", 'm', 'g') + 1;
    i_command = (char*)malloc(i_length);

    snprintf(i_command, i_length, "%c %c", 'm', 'g');
    WriteCommand(i_command, i_length);
}
void ARobot::TurnDistance(float degrees, BotDir dir)
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
    this->currState = TURN;
    this->initTurnRec = true;
    printf("Initial Yaw: %f Degrees: %f ToTurn: %f\n", initialYaw, degrees, toTurn);
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
    if(initTurnRec) {
        initTurnRec = false;
        prevYaw = currYaw;
    }
    if(dir == RIGHT) {
        if(prevYaw < currYaw && abs(prevYaw-currYaw) >= 0.5) {//if robot crosses over from 180 to -180, direction switches
            //printf("Curr Yaw: %f\tPrev Yaw: %f\n", currYaw, prevYaw);
            cross_over = true;
        }
        if(cross_over) //condition holds even if prev doesn't when cross_over is already true
            currYaw -= 360.0f; //range fixing
        if(currYaw <= toTurn) { //error fixing
            printf("Turn is Finished?\n");
            char* i_command;
            cross_over = false; //default cross bool now off
            int i_length = snprintf(NULL, 0, "%c %c", 'm', 'c') + 1;
            i_command = (char*)malloc(i_length);
            snprintf(i_command, i_length, "%c %c", 'm', 'c');
            WriteCommand(i_command, i_length);
            if(isVictim && isDropped == false) {
                printf("Drop\n");
                currState = DROP;
            } else if (isVictim && isDropped) {
                currState = PLANNING;
            } else {
                currState = IDLE;
            }
            return;
        }
    } else if(dir == LEFT) {
        if(prevYaw > currYaw && abs(prevYaw-currYaw) >= 0.5) //if robot crosses over from -180 to 180, direction switches
            cross_over = true;
        if(cross_over)
            currYaw += 360.0f; //range fixing

        if(currYaw >= toTurn-1.0) {
            char* i_command;
            cross_over = false; //default cross bool now off
            int i_length = snprintf(NULL, 0, "%c %c", 'm', 'c') + 1;
            i_command = (char*)malloc(i_length);
            snprintf(i_command, i_length, "%c %c", 'm', 'c');
            WriteCommand(i_command, i_length);
            if(isVictim && isDropped == false) {
                printf("Drop\n");
                currState = DROP;
            } else {
                currState = IDLE;
            }
            return;
        }
    }
    prevYaw = imuDataList[imu_list-1].m_yaw;
    //printf("To_X: %d, To_Y: %d, Curr_X: %f, Curr_Y: %f\n", currTile.x_tovisit, currTile.y_tovisit, currTile.x_map, currTile.y_map);
}

void ARobot::CalibrateIMU()
{
    int i_length = snprintf(NULL, 0, "%c %c", 'i', 'b') + 1;
    char* i_command = (char*)malloc(i_length);
    snprintf(i_command, i_length, "%c %c", 'i', 'b');
    WriteCommand(i_command, i_length);
}

void ARobot::FixYaw(int degrees) {\
    const size_t imu_vals = imuDataList.size();
    float newyaw = imuDataList[imu_vals-1].m_yaw;
    newyaw += degrees;
    this->turnOffset += degrees;

    if(newyaw>=360) newyaw-=360;
    else if(newyaw<0) newyaw+=360;
    imuDataList[imu_vals-1].setYawOffset(this->turnOffset);
    imuDataList[imu_vals-1].m_yaw = newyaw; //after change
    printf("Fixed Yaw: %f\n", newyaw);
}

void ARobot::TestRangeSensors() {
    const size_t range_vals = rangeDataList.size()-1;
    printf("Short Dist: %d\tLong Dist: %d\n", rangeDataList[range_vals].getRangeShort(), rangeDataList[range_vals].getRangeLong());
    printf("Error: %d\t", rangeDataList[range_vals].getRangeOffset());
    printf("Alpha Value: %f\n", rangeDataList[range_vals].getAlpha());
    return;
}

void ARobot::ParseIMU()
{
    /*if(imuDataList.size() <= 0) {
        imuParseList.front().parseData(-1); //special initial case
    } else {
        imuParseList.front().parseData(imuDataList[imuDataList.size()-1].getTStamp());
    }*/
    imuParseList.front().parseData();
    if(imuCalibrated)
        imuParseList.front().runFilter();
    imuDataList.push_back(imuParseList.front());
    imuParseList.pop();
}
void ARobot::ParseRange() {
    rangeParseList.front().parseData();
    rangeParseList.front().getPosition();
    rangeParseList.front().setAngle();
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

//testing method
void ARobot::PrintXYCoords(int x, int y) {
    for(int i = 29; i > 0; i--) {
        for(int j = 0; j < 30; j++) {
            if((y >= i-2 && y <= i+2) && (x <= j+2 && x >= j-2)) {
                printf("X");
            } else {
                printf(".");
            }
        }
        printf("\n");
    }
}

void ARobot::Reset() {
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
    this->imuCalibrated = false;
    this->initialYaw = 0;
    this->initTurnRec = false;
    this->isDropped = false;
    this->isVictim = false;
    this->mlen_imu = 0;
    this->mlen_light = 0;
    this->mlen_range = 0;
    this->mlen_temp = 0;
    this->m_letter = '0';
    this->off_left = 5;
    this->off_right = 25;
    this->offsetdir = 1;
    this->prevYaw = 0;
    this->silver_thresh = 0;
    this->sLock = 0;
    this->speed_left = 100;
    this->speed_right = 115;
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

