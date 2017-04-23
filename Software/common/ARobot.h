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

class LightData;
class RangeData;
class IMUData;
class TempData;

class ARobot {
 public:
 	ARobot(SerialPort *port);
 	~ARobot();
 	/*Writing to Arduino*/
 	void WriteCommand(char* command, int size);

 	/*Ramp*/
 	void checkRamp();

 	/*Temperature Sensor -> Victim Control*/
 	void checkVictimTemp();
 	void setTempThresh(float left, float right);
 	float getLeftVictimTemp();
 	float getRightVictimTemp();

 	/*Light Sensor -> Tile Control*/
 	void setLightThresh(int black, int silver);
 	void checkLightTile();
 	int getBlackThresh();
 	int getSilverThresh(); 	

 	/*Dropper Components*/
 	void LEDLight(int time);
 	void Drop();

 	/*DC Motor Control*/
 	void SetSpeed(int left_speed, int right_speed);
 	void MoveDistance(int distance_mm, botDir forward);
 	void TurnDistance(int degrees, botDir right);
 	void StopTurn(bool right);

 	/*Parsing*/
 	void ParseIMU();
 	void ParseRange();
 	void ParseTemp();
 	void ParseLight();
	void ClearIMU();
	void ClearRange();
	void ClearTemp();

	enum LightVal {WHITE=0, BLACK=1, SILVER=2};
	enum BotDir {RIGHT, LEFT, FRONT, BACK};
	enum CurrentState {DROP, MOVE, TURN, LED, IDLE};


 	std::vector<IMUData> imuDataList;
	std::vector<RangeData> rangeDataList;
	std::vector<TempData> tempDataList;
	std::vector<LightData> lightDataList;

	std::queue<IMUData> imuParseList;
	std::queue<RangeData> rangeParseList;
	std::queue<TempData> tempParseList;
	std::queue<LightData> lightParseList;

	LightVal currTileLight;
	BotDir currDir; 
	CurrentState currState;

 protected:
 	SerialPort *mPort;

 private:
 	float initialYaw;
 	float toTurn;
 	bool backingBlack; //if the robot is backing up on a black tile
 	uint16_t silver_thresh;
 	uint16_t black_thresh;
 	uint16_t white_thresh;
 	float threshLeft;
 	float threshight;
 	size_t mlen_light;
 	size_t mlen_imu;
 	size_t mlen_range;
 	size_t mlen_temp;
}; // class Thread
#endif // _Thread_h_
