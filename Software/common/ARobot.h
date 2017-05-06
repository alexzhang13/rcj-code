#ifndef _ROBOT_h_
#define _ROBOT_h_

#include <vector>
#include <queue>
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "TempData.h"
#include "LightData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"
#include "cell.h"
#include "navigate2D.h"

class LightData;
class RangeData;
class IMUData;
class TempData;

class ARobot {
 public:
 	ARobot(SerialPort *port);
 	~ARobot();

 	/*Structs*/
 	typedef struct {
 		int32_t x; //cell coord
 		int32_t y; //cell coord
 		int32_t x_tovisit; //x cell about to go to
 		int32_t y_tovisit; //y cell about to go to
 		int32_t x_map; //map coord in mm
 		int32_t y_map; //map coord in mm
 	} Map_Coord;

 	/*Enums*/
 	enum LightVal {WHITE=0, BLACK=1, SILVER=2};
	enum BotDir {RIGHT=0, LEFT=1, FRONT=2, BACK=3};
	enum BotOrientation {NORTH=0, EAST=1, SOUTH=2, WEST=3};
	enum CurrentState {DROP=0, MOVE=1, TURN=2, LED=3, IDLE=4, RAMP=5, PLANNING=6, WAYPTNAV=7, DONE=8};

 	/*Writing to Arduino*/
 	void WriteCommand(char* command, int size);

 	/*Algorithm <-> Control*/
 	void UpdateCellMap(MazeCell *sensor_info);
 	void UpdateNeighborCells();
 	void TileTransition(BotOrientation direction, float angle, int32_t dist);
 	void CalcNextTile();

 	/*Ramp*/
 	bool checkRamp();

 	/*Temperature Sensor -> Victim Control*/
 	bool checkVictimTemp();
 	void setTempThresh(float left, float right);
 	float getLeftVictimTemp();
 	float getRightVictimTemp();

 	/*Light Sensor -> Tile Control*/
 	void setLightThresh(uint16_t black, uint16_t silver);
 	void checkLightTile();
 	int getBlackThresh();
 	int getSilverThresh(); 	

 	/*Dropper Components*/
 	void LEDLight(int time);
 	void Drop();

 	/*DC Motor Control*/
 	void SetSpeed(int left_speed, int right_speed);
 	void MoveDistance(int distance_mm, BotDir forward);
 	void TurnDistance(int degrees, BotDir right);
 	void StopTurn(BotDir right);

 	/*Parsing*/
 	void ParseIMU();
 	void ParseRange();
 	void ParseTemp();
 	void ParseLight();
	void ClearIMU();
	void ClearRange();
	void ClearTemp();
	void ClearLight();

	std::vector<MazeCell> temp_cell_list;
	std::vector<int32_t> waypts; //current waypoint list

 	std::vector<IMUData> imuDataList;
	std::vector<RangeData> rangeDataList;
	std::vector<TempData> tempDataList;
	std::vector<LightData> lightDataList;

	std::queue<IMUData> imuParseList;
	std::queue<RangeData> rangeParseList;
	std::queue<TempData> tempParseList;
	std::queue<LightData> lightParseList;

	LightVal currTileLight; //Current Tile's Light Status
	BotDir currDir; //Turning and Moving directions, local to current pos and next direction
	CurrentState currState; //Current state of the Robot
	BotOrientation currOrientation; //Current Direction through Compass System, Global
	MazeCell::NavDir victimDir; //Direction of Victim in local coord
	Map_Coord currTile;

	MazeCell sensor_info; //sensor info for current cell

	bool backingBlack; //if the robot is backing up on a black tile
 	int silver_thresh; //Silver Tile Threshold
 	int black_thresh; //Black Tile Threshold
 	int white_thresh; //White Tile Threshold
 	float threshLeft; //Left Temperature Threshold
 	float threshRight; //Right Temperature Threshold

 protected:
 	SerialPort *mPort;

 private:
 	float initialYaw;
 	float toTurn;
 	size_t mlen_light;
 	size_t mlen_imu;
 	size_t mlen_range;
 	size_t mlen_temp;
}; // class Thread
#endif // _Thread_h_
