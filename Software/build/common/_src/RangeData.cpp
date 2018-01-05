#include "../_headers/RangeData.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../_headers/IMUData.h"

#define OFFSET 17.5
#define PI 3.1415926535
#define SHORTOFFSET 30.0
#define LONGOFFSET 55.0

RangeData::RangeData(ARobot *robot) :myRobot(robot)
{
	memset(m_command,'\0', 128);
	alpha = 0.0f;
	angled = -1.0f; //undeterminable when negative
	avalid_long = true;
	avalid_short = true;
	coord.x = 0; 
	coord.y = 0; //default center (0,0)
	coord.x_flag = true; 
	coord.y_flag = true; //assume readings are true at first
	curr_yaw=0;
	temp_dist=0;
	walls.wallN = 0;
	walls.wallE = 0;
	walls.wallS = 0;
	walls.wallW = 0;
	x_count = 0;
	y_count = 0;

}

RangeData::~RangeData()
{
	
}

void RangeData::storeCommand(char* buf) {
	memcpy(m_command, buf, 64);
}

int RangeData::parseData()
{
	sscanf(m_command, "%d %c %d %f %f %f %f", &data.tstamp, &data.id, &data.angle, &data.laserL_a, &data.laserS_a, &data.laserL_b, &data.laserS_b);
	if(data.laserL_a >= 400) {
		data.laserL_a -= LONGOFFSET;
	} else if (data.laserL_b >= 300) {
		data.laserL_a -= LONGOFFSET;
	}
	if(data.laserL_b >= 400) {
		data.laserL_b -= LONGOFFSET;
	} else if (data.laserL_b >= 300) {
		data.laserL_b -= SHORTOFFSET;
	}
	return 0;
}

int RangeData::getScan() {
	sscanf(m_command, "%d %c %d %f %f %f %f", &scan.tstamp, &scan.id, &scan.angle, &scan.readingN, &scan.readingE, &scan.readingS, &scan.readingW);
	return 0;
}

float RangeData::getAlpha() {
	return this->alpha;
}

float RangeData::getAngle() {
	return this->angled;
}

int RangeData::setAngle() {
	//drift is seemingly always negative
	//keep threshold at 30 degrees (0 - 30)
	if(avalid_short) {
		angled = acos(min(1.0, 300.0/(data.laserS_a + data.laserS_b + OFFSET))) * 180 / PI;
		alpha = max(0.0, 7.62*(300.0/(data.laserS_a + data.laserS_b + OFFSET)-0.866)); //30 degree turn range 0-1
		if(alpha >= 1.3) alpha = 0;
		else if (alpha >= 1) alpha = 1;
		//printf("Distance: %f\tAlpha: %f\n", data.laserS_a + data.laserS_b + OFFSET, alpha);
		// 7.62 = (1 - 0) / (1 - 0.866) --> 0.886 = root (3) / 2 which is cos(30 deg)
	} else if(avalid_long) {
		int far = ((int)temp_range[0]/300 + (int)temp_range[2]/300 + 1);
		angled = acos(min(1.0, (double)(far*300)/(data.laserL_a + data.laserL_b + OFFSET))) * 180 / PI;
		alpha = max(0.0, (7.62/pow(far-1,2))*((far*300)/(data.laserL_a + data.laserL_b + OFFSET)-0.786)); //30 degree turn range 0-1
		if(alpha >= 1.3) alpha = 0;
		else if (alpha >= 1) alpha = 1;
		//printf("Angle: %f\tAlpha: %f\tValue: %i Distance: %f\n", angled, alpha, far*300, data.laserL_a + data.laserL_b + OFFSET);
	}
	return 0;
}

int RangeData::getPosition()
{	
	size_t imuSize = myRobot->imuDataList.size();
	curr_yaw = myRobot->imuDataList[imuSize-1].m_yaw;

	if(curr_yaw <= 45.0 || curr_yaw >= 315.0) { //north
		data.dir = 0;
		myRobot->currOrientation = ARobot::NORTH;
		//printf("north\n");
	} else if (curr_yaw <= 135.0 && curr_yaw >= 45.0) { //west
		data.dir = 1;
		myRobot->currOrientation = ARobot::WEST;
		curr_yaw -= 90;
		//printf("west\n");
	} else if (curr_yaw <= 225.0 && curr_yaw >= 135.0) { //south
		data.dir = 2;
		myRobot->currOrientation = ARobot::SOUTH;
		curr_yaw -= 180;
		//printf("south\n");
	} else { //east
		data.dir = 3;
		myRobot->currOrientation = ARobot::EAST;
		curr_yaw -= 270;
		//printf("east\n");
	}
	//3.1415926535 * 90 = 282.743338815
	if(data.laserL_a <= 1200) { //check if reading is valid LONG FRONT
		temp_range[0] = (data.laserL_a-30.0) * std::abs(cos((3.1415926535*curr_yaw)/180.0)); //-18.0
		temp_dist = (int)temp_range[0]/300;
		distance[0] = temp_range[0] - temp_dist*300;
		if(data.laserL_a >= 900) avalid_long = false;
	} else {
		distance[0] = 451.0f; //impossible number for distance[0], as it's %300
		temp_range[0] = -300;
		avalid_long = false;
	} 
	if(data.laserL_b <= 1200) { //check if reading is valid LONG BACK
		temp_range[2] = (data.laserL_b+59.25) * std::abs(cos((3.1415926535*curr_yaw)/180.0));
		temp_dist = (int)temp_range[2]/300;
		distance[2] = temp_range[2] - temp_dist*300;
		if(data.laserL_a >= 900) avalid_long = false;
	} else {
		distance[2] = 451.0f; //impossible number for distance[2], as it's %300
		temp_range[2] = -300;
		avalid_long = false;
	}
	if(data.laserS_a <= 250) { //check if reading is valid SHORT RIGHT
		temp_range[1] = (data.laserS_a+13.75) * std::abs(cos((3.1415926535*curr_yaw)/180.0)); //30 - x - 15 = 15 - x 
		temp_dist = (int)temp_range[1]/300;
		distance[1] = temp_range[1] - temp_dist*300;
	} else {
		distance[1] = 451.0f;
		temp_range[1] = -300;
		avalid_short = false;
	}
	if(data.laserS_b <= 250) { //check if reading is valid SHORT LEFT
		temp_range[3] = (data.laserS_b+13.75) * std::abs(cos((3.1415926535*curr_yaw)/180.0)); //x - 15
		temp_dist = (int)temp_range[3]/300;
		distance[3] = temp_range[3] - temp_dist*300;
	} else {
		distance[3] = 451.0f;
		temp_range[3] = -300;
		avalid_short = false;
	}

	/*Convert Readings into Cartesian Coordinates where (0,0) is the center of the cell*/
	distance[(0+data.dir)%4] = 150.0f - distance[(0+data.dir)%4]; //depending on the rotational orientation, to get the center as (15, 15)
	distance[(1+data.dir)%4] = 150.0f - distance[(1+data.dir)%4]; //we have to use the same reference, thus making north and east (15 - distance) for the coordinate, it's normally 30 - distance, but because of -15, it becomes 15 - distance 
	distance[(2+data.dir)%4] -= 150.0f; //for south and west it is regularly just subtracting 15 
	distance[(3+data.dir)%4] -= 150.0f;

	//printf("Lasers: %f %f %f %f\n", data.laserL_a, data.laserS_a, data.laserL_b, data.laserS_b);
	//printf("Temps: %f %f %f %f\n", temp_range[(0+data.dir)%4], temp_range[(1+data.dir)%4], temp_range[(2+data.dir)%4], temp_range[(3+data.dir)%4]);

	walls.wallN = (int)temp_range[(0+data.dir)%4]/300;
	walls.wallE = (int)temp_range[(1+data.dir)%4]/300;
	walls.wallS = (int)temp_range[(2+data.dir)%4]/300;
	walls.wallW = (int)temp_range[(3+data.dir)%4]/300;
	//printf("Walls: %d %d %d %d\n", walls.wallN, walls.wallE, walls.wallS, walls.wallW);

	for(int i = 0; i < 4; i++) {
		if((data.dir + i) % 2 == 1) { //x coords
			if(abs(distance[(i+data.dir)%4]) != 301) {
				coord.x += distance[(i+data.dir)%4];
				if(distance[(i+data.dir)%4] != coord.x){ //If you already went through more than one iteration, now you want to divide by 2
					coord.x /= 2;
				}
				coord.x_glob = coord.x + 150.0f; //real coord output
			} else {
				++x_count;
				if(x_count >= 2) {
					coord.x_flag = false;
				}
			}
		} else { //y coords
			if(abs(distance[(i+data.dir)%4]) != 301) {
				coord.y += distance[(i+data.dir)%4];
				if(distance[(i+data.dir)%4] != coord.y){ //If you already went through more than one iteration, now you want to divide by 2
					coord.y /= 2;
				}
				coord.y_glob = coord.y + 150.0f; //real coord output
			} else {
				++y_count;
				if(y_count >= 2) {
					coord.x_flag = false;
				}
			}
		}
	}

	return 0;
}
