#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>

#include "processThread.h"
#include "ARobot.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"
#include <vector>

using namespace std;

static void readConfig(const char* filename, ARobot *robot);
static void readCurrentMap(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
static void Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
static int WayPointNav(ARobot *robot, Navigate2D &nav_rt);
size_t bot_waypts = 0;

int main(int argc,char **argv){
    Navigate2D nav; //main map class obj
#ifdef WIN32
    const char* fileConfig = "C:/projects/StormingRobots2017/Data/Mem/config.txt";
    const char* in_dir = "C:/projects/StormingRobots2017/Data/map_data";
    const char* xml_name = "mazemap";
#else
    const char* fileConfig = "/home/alex/projects/rcj-code/Software/common/Mem/config.txt";
    const char* in_dir = "/home/alex/projects/rcj-code/Data/map_data";
    const char* xml_name = "mazemap";
#endif

    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
	if(port == NULL)
		printf(" Serial port open failed\n");
	printf(".Start robot navigation\n");
    ARobot *myRobot = new ARobot(port);
    printf("Fault 1 Passed\n");
    UartRx *uartrx = new UartRx(port, myRobot);
    printf("Fault 2 Passed\n");
    Process_T *process_thread = new Process_T(port, myRobot);
    printf("Fault 3 Passed\n");
    readConfig(fileConfig, myRobot); //read config file about threshold calibrations
    /*
    readCurrentMap(in_dir, xml_name, myRobot, nav); //check for previous map from mem
    printf("step3");
    sleep(8000); //8 second delay
    while(1) {
        if(myRobot->currState == ARobot::PLANNING) {
            Navigate(in_dir, xml_name, myRobot, nav);
        }
        if(myRobot->currState == ARobot::WAYPTNAV) {
            WayPointNav(myRobot, nav);
        }
        if(myRobot->currState == ARobot::TURN) {
            StopTurn(currDir);
        }
    }*/
    sleep(1);
    myRobot->TurnDistance(90, ARobot::RIGHT);
	int32_t c = 0;
    while(1) {
		printf("test %d \n", c);
		c++;	
        if(myRobot->currState == ARobot::TURN) {
            Robot->StopTurn(myRobot->currDir);
        }
		sleep(1);
	}

    return 0;
}

void readConfig(const char* filename, ARobot *robot)
{
    FILE *datafile;
    if (filename == NULL)
        return;

    datafile = fopen(filename, "r");
    if(datafile == NULL) {
		printf("%s is not available\n", filename);
      	return;
	}
    int ret = fscanf(datafile, "%d %d %d %f %f", &robot->black_thresh, &robot->silver_thresh, &robot->white_thresh, &robot->threshLeft, &robot->threshRight);
}

void readCurrentMap(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt)
{
    MazeCell::NavDir heading = MazeCell::navNorth;

    int32_t home_floor_num = 0;
    nav_rt.getCurTime();
    if(nav_rt.readChkPtMaps(filename, xmlname)!= 0) {
        nav_rt.setHomeCell(home_floor_num, heading);    
        //set current robot coords to x, y
    }
}

void Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt) 
{
    /*Navigational functions*/
    robot->UpdateCellMap(&robot->sensor_info);
    robot->UpdateNeighborCells();
    nav_rt.configureCurCell(&robot->sensor_info);
    nav_rt.detectLocalCells(robot->temp_cell_list);
    nav_rt.updateLocalMap();
    nav_rt.getNavigateMaps()->writeXmlMap(filename, xmlname);

    robot->temp_cell_list.clear();

    //nav_rt.slam2d(); // will move to another thread
    // what to do next
    nav_rt.navigatePlanning();
    // move on to the next cell
    nav_rt.navigation2D();
    if(nav_rt.getNextCell()->waypts.size() >= 2) {
        robot->waypts = nav_rt.getNextCell()->waypts; //waypts
    } else {
        robot->currState = ARobot::DONE;
    }
    
    //nav_rt->getCellbyIndex(nav_rt.m_next_cell.waypts.begin()).getCellGrid(&robot.currTile.x_tovisit, &robot.currTile.y_tovisit);
}

int WayPointNav(ARobot *robot, Navigate2D &nav_rt)
{
    bot_waypts = robot->waypts.size();
    if(bot_waypts < 2) {
        robot->waypts.pop_back();
        robot->currState = ARobot::PLANNING;
        return -1;
    } 
    nav_rt.getCellbyIndex(robot->waypts[bot_waypts-1])->getCellGrid(robot->currTile.x_tovisit, robot->currTile.y_tovisit);
    robot->CalcNextTile();
}
