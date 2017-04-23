#ifndef _ROBOT_h_
#define _ROBOT_h_

#include "SerialPort.h"
#include <vector>
#include <queue>

 
class ARobot {
 public:
 	ARobot(SerialPort *port)
 		:mPort(port)
 	{}

 	~ARobot();
 	void WriteCommand(char* command, int size);
 	void SetSpeed(int left_speed, int right_speed);
 	void MoveDistance(int distance_mm, bool forward);
 	void TurnDistance(int degrees, bool right);
 	void StopTurn(bool right);
 	void ParseIMU();
 	void ParseRange();
 	void ParseTemp();
 	void ParseLight();
	void ClearVectors();
	template<typename T> void pop_front(std::vector<T>& vec);


 	std::vector<IMUData> imuDataList;
	std::vector<RangeData> rangeDataList;
	std::vector<TempData> tempDataList;
	std::vector<LightData> lightDataList;

	std::queue<IMUData> imuParseList;
	std::queue<RangeData> rangeParseList;
	std::queue<TempData> tempParseList;
	std::queue<LightData> lightParseList;

	bool dropperFlag = false;
	bool ledFlag = false;
	bool isMoving = false;
	bool isTurning = false;
	bool turnDir = true; //true is right, false is left


 protected:
 	SerialPort *mPort;

 private:
 	float currentYaw;
 	float toTurn;
}; // class Thread
#endif // _Thread_h_
