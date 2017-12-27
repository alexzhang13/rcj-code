#ifndef _ROBOT_h_
#define _ROBOT_h_

#include <vector>
#include <queue>

#include "../_headers/IMUData.h"
#include "../_headers/LightData.h"
#include "../_headers/picamera.h"
#include "../_headers/RangeData.h"
#include "../_headers/SerialPort.h"
#include "../_headers/TempData.h"
#include "../_headers/Thread.h"
#include "../_headers/UartRx.h"
#include "../_headers/UartTx.h"
#include "cell.h"
#include "navigate2D.h"
#include "kNNFilter.h"

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
	enum CurrentState {PLANNING=0, WAYPTNAV=1, TURN=2, IDLE=3, RAMP=4, MOVE=5, DROP=6, BLACKBACK=7, DONE=8, DATA=9};

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
 	void UpdateCellMap(MazeCell *sensor_info, bool backing_black);
 	void UpdateNeighborCells();
 	void TileTransition(int32_t dist);
 	void CalcNextTile();

 	/*IMU*/
 	void CalibrateIMU();

 	/*Ramp*/
 	bool CheckRamp();

 	/*Laser*/
 	void SpinLaser();

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

	std::vector<MazeCell> temp_cell_list;
	std::vector<int32_t> waypts; //current waypoint list

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
	bool isVictim; //if a victim has been detected in this cell already
	bool isDropped;
	bool toMove; //true means the robot still has to move after turning
	bool victimRight; //true if is dropping to the right
	bool victimLeft;
	bool victimFront;
	bool backingBlack; //if the robot is backing up on a black tile
	int dist_temp; //store temporary distance to travel

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
 	float toTurn; //distance to turns
 	bool cross_over; //[check StopTurn() function] --> determines if the yaw has turned over 360 degs
 	size_t mlen_light;
 	size_t mlen_imu;
 	size_t mlen_range;
 	size_t mlen_temp;
}; // class Thread
#endif // _Thread_h_
