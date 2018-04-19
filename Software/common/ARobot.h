#ifndef _ROBOT_h_
#define _ROBOT_h_

#include <vector>
#include <queue>

#include "IMUData.h"
#include "LightData.h"
#include "picamera.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "TempData.h"
#include "Thread.h"
#include "UartRx.h"
#include "UartTx.h"
#include "cell.h"
#include "navigate2D.h"
#include "kNNFilter.h"
#include <algorithm>
#include <numeric>

class LightData;
class RangeData;
class IMUData;
class TempData;
class _PiCamera_;

class ARobot {
public:
    ARobot(SerialPort *port);
    ~ARobot();

    /*Enums*/
    enum LightVal {WHITE=0, BLACK=1, SILVER=2};
    enum BotDir {FRONT=0, RIGHT=1, BACK=2, LEFT=3};
    enum BotOrientation {NORTH=0, EAST=1, SOUTH=2, WEST=3};
    enum CurrentState {PLANNING=0, WAYPTNAV=1, TURN=2, IDLE=3, RAMP=4, MOVE=5, DROP=6, BLACKBACK=7, DONE=8, DATA=9, STOP=10};

    /*Structs*/
    typedef struct {
        int32_t x; //cell coord
        int32_t y; //cell coord
        int32_t x_tovisit; //x cell about to go to
        int32_t y_tovisit; //y cell about to go to
        float x_map; //map coord in mm
        float y_map; //map coord in mm
    } Map_Coord;

    typedef struct {
        char letter;
        BotDir dir_victim;
        bool m_isVictim;
    } Visual_Victim;

    /*Writing to Arduino*/
    void WriteCommand(char* command, int size);

    /*Algorithm <-> Control*/
    void UpdateCellMap(MazeCell *sensor_info, bool backing_black, bool CheckRamp);
    void UpdateNeighborCells();
    void TileTransition(int32_t dist);
    void CalcNextTile();
    void CorrectionFailed();
    void CheckCorrection();
    void Correction();
    void CorrectYaw();
    int SlopeDir(); //const std::vector<double>& x, const std::vector<double>& y

    /*IMU*/
    void CalibrateIMU();
    void setOffsetDir();
    void FixYaw(int degrees); //add certain number of degrees to fix yaw
    int getOffsetDir();

    /*Ramp*/
    bool CheckRamp();

    /*Laser*/
    void SpinLaser();
    void TestRangeSensors();

    /*Temperature Sensor -> Victim Control*/
    int CheckVictimTemp();
    void setTempThresh(float left, float right);
    float getLeftVictimTemp();
    float getRightVictimTemp();
    void CheckVictimVisual();
    int ProcessImage_Victim();

    /*Light Sensor -> Tile Control*/
    void setLightThresh(int black, int silver);
    void CheckLightTile();
    int getBlackThresh();
    int getSilverThresh();

    /*Dropper Components*/
    void LEDLight(int time);
    void Drop();

    /*DC Motor Control*/
    void setSpeed(int left_speed, int right_speed);
    void setOffsetSpeed(int offset_l, int offset_r);
    void MoveDistance(int distance_mm, BotDir forward);
    void StopMove();
    void TurnDistance(int degrees, BotDir right);
    void StopTurn(BotDir right);
    void ResetEncoder();

    /*Parsing*/
    void ParseIMU();
    void ParseRange();
    void ParseTemp();
    void ParseLight();
    void ParseScan();
    void ClearIMU();
    void ClearRange();
    void ClearTemp();
    void ClearLight();
    void ClearScan();
    void ClearImgList();

    /*Testing Signal*/
    void TestSignal();
    void PrintXYCoords(int x, int y);
    void DisplayVictimVisual();

    /*Inline Functions*/
    float getSTD(int* arr, int avg) {
        float total = 0;
        //no break condition for size of arr = 0
        for(int i=0; i<sizeof(arr)/sizeof(arr[0]); i++)
            total += std::sqrt((float)std::pow(arr[0]-avg, 2)/(sizeof(arr)/sizeof(arr[0])));
        return total;
    }

    std::vector<MazeCell> temp_cell_list;
    std::vector<int32_t> waypts; //current waypoint list
    std::vector<double> x_vals; //x accumulate
    std::vector<double> y_vals; //y accumulate

    std::vector<IMUData> imuDataList;
    std::vector<RangeData> rangeDataList;
    std::vector<TempData> tempDataList;
    std::vector<LightData> lightDataList;
    std::vector<RangeData> scanDataList;
    std::vector<cv::Mat> imgList;

    std::queue<IMUData> imuParseList;
    std::queue<RangeData> rangeParseList;
    std::queue<TempData> tempParseList;
    std::queue<LightData> lightParseList;
    std::queue<RangeData> scanParseList;

    LightVal currTileLight; //Current Tile's Light Status
    BotDir currDir; //Turning and Moving directions, local to current pos and next direction
    CurrentState currState; //Current state of the Robot
    BotOrientation currOrientation; //Current Direction through Compass System, Global
    MazeCell::NavDir victimDir; //Direction of Victim in local coord
    Map_Coord currTile;
    kNNFilter knn;

    MazeCell sensor_info; //sensor info for current cell
    _PiCamera_ picam; //pi camera object
    Visual_Victim victim; //visual victim param object

    uint8_t dropCnt; //dropping counter
    uint8_t sLock; //lock slope coords when > 5
    bool isVictim; //if a victim has been detected in this cell already
    bool isDropped;
    bool toMove; //true means the robot still has to move after turning
    bool victimRight; //true if is dropping to the right
    bool victimLeft;
    bool victimFront;
    bool backingBlack; //if the robot is backing up on a black tile
    bool imuCalibrated; //flag if IMU is calibrated
    bool correctionFailed; // if correction fails
    bool isCorrecting; //is the robot in a state of correcting
    int dist_temp; //store temporary distance to travel
    float correctionErrorChange; //delta error

protected:
    SerialPort *mPort;

private:
    char m_letter;
    float threshLeft; //Left Temperature Threshold
    float threshRight; //Right Temperature Threshold
    int silver_thresh; //Silver Tile Threshold
    int black_thresh; //Black Tile Threshold
    int speed_left; //robot left motor power
    int speed_right; //robot right motor power
    int off_left; //robot left motor offset
    int off_right; //robot right motor offset
    int yaw_drift; //Add to counter yaw drift...
    float initialYaw; //initial yaw in a turn
    float prevYaw; //previous yaw reading
    float toTurn; //distance to turns
    float correctionError; //if correction fails, find distance to ensure something went wrong
    BotDir correctionDir; //direction the robot just tried to fix itself
    bool initTurnRec; //specific special case (glitch) -- cleaner way will remove this sooner or later
    bool cross_over; //[check StopTurn() function] --> determines if the yaw has turned over 360 degs
    int8_t offsetdir; //what direction is the robot offset from the center right=1, left=3
    size_t mlen_light;
    size_t mlen_imu;
    size_t mlen_range;
    size_t mlen_temp;
}; // class Thread
#endif // _Thread_h_
