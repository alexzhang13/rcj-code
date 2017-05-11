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
static void readCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
static void writeCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
static void Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
static int WayPointNav(ARobot *robot, Navigate2D &nav_rt);
size_t bot_waypts = 0;

int main(int argc,char **argv){
    Navigate2D nav; //main map class obj
#ifdef WIN32
    const char* fileConfig = "C:/projects/StormingRobots2017/Data/Mem/config.txt";
    const char* in_dir = "C:/projects/StormingRobots2017/Data";
	const char* rt_logname = "realtime/rcj_log";
    const char* xml_name = "map_data/mazemap";
#else
    const char* fileConfig = "/home/alex/projects/rcj-code/Software/common/Mem/config.txt";
    const char* in_dir = "/home/alex/projects/rcj-code/Data";
	const char* rt_logname = "realtime/rcj_log";
    const char* xml_name = "map_data/mazemap";
#endif

    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
	if(port == NULL)
		printf(" Serial port open failed\n");
	printf(".Start robot navigation\n");
    ARobot *myRobot = new ARobot(port);
    printf("Fault 1 Passed\n");
    UartRx *uartrx = new UartRx(port, myRobot);
	uartrx->setLogFile(in_dir, rt_logname);
    printf("Fault 2 Passed\n");
    Process_T *process_thread = new Process_T(port, myRobot);
    printf("Fault 3 Passed\n");
    readConfig(fileConfig, myRobot); //read config file about threshold calibrations
    
    //readCurrentMap(in_dir, xml_name, myRobot, nav); //check for previous map from mem

    sleep(3); //gather data in 3 secs
    //myRobot->TurnDistance(90, ARobot::RIGHT);
    while(1) {
        switch(myRobot->currState) {
            printf("1");
            case 0: //Planning
                Navigate(in_dir, xml_name, myRobot, nav);
                break;
            case 1: //WayPtNav
                WayPointNav(myRobot, nav);
                break;
            case 2: //Turn
                myRobot->StopTurn(myRobot->currDir);
                break;
            case 3: //Idle
                if(myRobot->toMove){
                    myRobot->MoveDistance(myRobot->dist_temp, ARobot::FRONT);
                    myRobot->toMove = false;
                }
                break;
            case 4: //Ramp
                /*Put stuff here*/
                break;
            case 5: //Move
                /*Put stuff here*/
                break;
            case 6: //Drop
                /*Put stuff here*/
                break;
            case 7: //LED
                /*Put stuff here*/
                break;
            case 8: //DONE
                /*Put stuff here*/
                break;
            case 9: //Data collection
                //spin laser
                sleep(3);
                myRobot->currState = ARobot::PLANNING;
                break;
            default:
                /*Put stuff here*/
                break;
        }
        sleep(0.01);
    }
    /*sleep(3);
    myRobot->TurnDistance(90, ARobot::RIGHT);
	int32_t c = 0;
    int cnt = 0;
    while(1) {
		//printf("test %d \n", c);
		//c++;	
        if(myRobot->currState == ARobot::TURN) {
            myRobot->StopTurn(myRobot->currDir);
        } else if(myRobot->currState == ARobot::IDLE && cnt == 0) {
            sleep(3);
            myRobot->TurnDistance(90, ARobot::LEFT);
            cnt++;
        }
        sleep(0.1);
	}*/

    return 0;
}

void readConfig(const char* filename, ARobot *robot)
{
    int black_thresh, silver_thresh;
    float threshLeft, threshRight;
    FILE *datafile;
    if (filename == NULL)
        return;

    datafile = fopen(filename, "r");
    if(datafile == NULL) {
		printf("%s is not available\n", filename);
      	return;
	}
    int ret = fscanf(datafile, "%d %d %f %f", &black_thresh, &silver_thresh, &threshLeft, &threshRight);
    robot->setTempThresh(threshLeft, threshRight);
    robot->setLightThresh(black_thresh, silver_thresh);
	fclose(datafile);
}

void readCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt)
{
    MazeCell::NavDir heading = MazeCell::navNorth;

    int32_t home_floor_num = 0;
    std::string t = nav_rt.getCurTime();
	std::string newfile = std::string(xmlname) + "_" + t;
    if(nav_rt.readChkPtMaps(filedir, xmlname)!= 0) {
        nav_rt.setHomeCell(home_floor_num, heading);    
        //set current robot coords to x, y
    }
	return;
}

void writeCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt)
{
    std::string t = nav_rt.getCurTime();
	std::string newfile = std::string(xmlname) + "_" + t;
    nav_rt.writeMapFile(filedir, xmlname);
	return;
}

void Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt) 
{
    /*Navigational functions*/
    printf("1");
    robot->UpdateCellMap(&robot->sensor_info, false); //false = not black
    printf("2");
    robot->UpdateNeighborCells();
    printf("3");
    nav_rt.configureCurCell(&robot->sensor_info);
    printf("4");
    nav_rt.detectLocalCells(robot->temp_cell_list);
    printf("5");
    nav_rt.updateLocalMap();
    printf("6");
    nav_rt.getNavigateMaps()->writeXmlMap(filename, xmlname);
    printf("7");

    robot->temp_cell_list.clear();
    printf("8");

    //nav_rt.slam2d(); // will move to another thread
    // what to do next
    nav_rt.navigatePlanning();
    // move on to the next cell
    //nav_rt.navigation2D();
    if(nav_rt.getNextCell()->waypts.size() >= 2) {
        robot->waypts = nav_rt.getNextCell()->waypts; //waypts
    } else {
        robot->currState = ARobot::DONE;
        return;
    }
    robot->currState = ARobot::WAYPTNAV;
    return;
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
