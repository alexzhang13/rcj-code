#ifndef _ROBOT_h_
#define _ROBOT_h_

#include <vector>
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "TempData.h"
#include "LightData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"

class ARobot {
 public:
 	ARobot();
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
 	void checkLight();
 	int getBlackThresh();
 	int getSilverThresh(); 	

 	/*Dropper Components*/
 	void LEDLight(int time);
 	void Drop();

 	/*DC Motor Control*/
 	void SetSpeed(int left_speed, int right_speed);
 	void MoveDistance(int distance_mm, bool forward);
 	void TurnDistance(int degrees, bool right);
 	void StopTurn(bool right);

 	/*Parsing*/
 	void ParseIMU();
 	void ParseRange();
 	void ParseTemp();
 	void ParseLight();
	void ClearVectors();
	template<typename T> void pop_front(std::vector<T>& vec);

	enum LightVal {WHITE, BLACK, SILVER};
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
 	uint8_t lightCounter; //counter for determining 
 	bool backingBlack; //if the robot is backing up on a black tile
 	uint16_t silver_thresh;
 	uint16_t black_thresh;
 	float tempLeft;
 	float tempRight;
}; // class Thread
#endif // _Thread_h_
