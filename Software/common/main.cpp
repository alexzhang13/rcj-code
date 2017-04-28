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

void readConfig(const char* filename, ARobot *robot);
void readCurrentMap(const char* filename, const char* xmlname, ARobot *robot);
void Navigate(const char* filename, const char* xmlname, ARobot *robot);

int main(int argc,char **argv){
    Navigate2D nav_rt; //main map class obj

    const char* fileConfig = "./Mem/config.txt";
    const char* in_dir = "C:/projects/StormingRobots2017/Data/map_data";
    const char* xml_name = "mazemap_04272017";

    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
    ARobot *myRobot = new ARobot(port);
    UartRx *uartrx = new UartRx(port, myRobot);
    //UartTx *uarttx = new UartTx(port);
    Process_T *process_thread = new Process_T(port, myRobot);

    readConfig(fileConfig, xml_name, myRobot); //read config file about threshold calibrations

    readCurrentMap(in_dir); //check for previous map from mem

    while(1) {
        if(myRobot.currState == ARobot::IDLE) {
            Nagivate(myRobot);
        }
        sleep(1); //small gap
    }

    return 0;
}

void readConfig(const char* filename, ARobot *robot)
{
    FILE *datafile;
    if (filename == NULL)
        return;

    datafile = fopen(filename, "r");
    int ret = fscanf(datafile, "%d %d %d %f %f", &robot->black_thresh, &robot->silver_thresh, &robot->white_thresh, &robot->threshLeft, &robot->threshRight);
}

void readCurrentMap(const char* filename, const char* xmlname, ARobot *robot);
{
    MazeCell::NavDir heading = MazeCell::navNorth;

    int32_t home_floor_num = 0;
    nav_rt.getCurTime();
    if(nav_rt.readChkPtMaps(filename, xmlname)!= 0) {
        nav_rt.setHomeCell(home_floor_num, heading);    
        nav_rt.setCellGrid(robot->currTile.x, robot->currTile.y);
    }
}

void Navigate(const char* filename, const char* xmlname, ARobot *robot) {
    /*Navigational functions*/
    robot->UpdateCellMap(&robot->sensor_info);
    robot->UpdateNeighborCells();
    nav_rt.configureCurCell(&robot->sensor_info);
    nav_rt.detectLocalCells(robot->temp_cell_list);
    nav_rt.updateLocalMap();
    nav_rt.m_navigateMaps.writeXMLMap(filename, xmlname);
    temp_cell_list.clear();

    //nav_rt.slam2d(); // will move to another thread
    // what to do next
    nav_rt.navigatePlanning();
    // move on to the next cell
    nav_rt.navigation2D();

}
