#include "navThread.h"

using namespace std;

void NavThread::run(void){
    sleep(1);
    readConfig(fileConfig, myRobot); //read config file about threshold calibrations
    printf("Config File Read...\n");
    if(this->readMap)
        readCurrentMap(in_dir, xml_name, myRobot, nav); //check for previous map from mem
    else
        startNewMap(myRobot, nav);
    printf("Map Generation Started...\n");
    //myRobot->picam.cameraOpen(720, 480);
    sleep(1);
    myRobot->imuCalibrated = true; //turn on IMU flag

    while(!this->isExit()) {
        if(this->toDestroy) {
            this->DestroyThread();
        }
        switch(myRobot->currState) {
        case 0: //Planning
            Navigate(in_dir, xml_name, myRobot, nav);
            sleep(0.5);
            break;
        case 1: //WayPtNav
            sleep(1);
            WayPointNav(myRobot, nav);
            break;
        case 2: //Turn
            myRobot->StopTurn(myRobot->currDir);
            sleep(0.1);
            break;
        case 3: //Idle
            if(myRobot->toMove){
                sleep(1.0);
                myRobot->CalcNextTile(false);
                myRobot->toMove = false;
            }  else if(myRobot->correctionFailed) {
                sleep(1.0);
                myRobot->CorrectionFailed();
            } else if(myRobot->isCorrecting) {
                sleep(0.5);
                myRobot->CheckCorrection();
            } else {
                sleep(0.5);
                myRobot->currState = ARobot::WAYPTNAV;
            }
            break;
        case 4: //Ramp
            while(myRobot->CheckRamp()) {
                sleep(0.1);
            }
            sleep(2);
            myRobot->StopMove();
            //do something with myRobot-sensor_info to update the cell info
            //myRobot->UpdateCellMap(&myRobot->sensor_info, false, true); //update curr cell location ??
            myRobot->currState = ARobot::WAYPTNAV;
            break;
        case 5: //Move
            myRobot->CheckLightTile(); //check if anything happens during this time
            //find the offset direction of the robot
            sleep(0.2);
            break;
        case 6: //Drop
            myRobot->LEDLight(4500);
            for(int i = 0; i < myRobot->dropCnt; i++) {
                myRobot->Drop();
                sleep(2);
            }
            myRobot->isDropped = true;
            nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->setVictim(true);
            myRobot->currState = ARobot::PLANNING;
            break;
        case 7: //BLACKBACK
            sleep(1);
            myRobot->backingBlack = false;
            nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-2]); //update "temp curr_cell"
            myRobot->UpdateCellMap(&myRobot->sensor_info, myRobot->backingBlack, false); //sensor_info auto resets in this function call
            nav.configureCurCell(&myRobot->sensor_info);
            nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-1]); //reupdate curr_cell
            myRobot->currState = ARobot::PLANNING;
            break;
        case 8: //DONE
            writeCurrentMap(this->map_dir, this->map_name, this->myRobot, this->nav);
            sleep(1);
            printf("DONE!");
            myRobot->currState = ARobot::STOP;
        case 9: //Data collection
            //myRobot->UpdateCellMap(&myRobot->sensor_info, false, false); //false = not black
            myRobot->isVictim = nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVictim();
            if(!myRobot->isVictim) {myRobot->isDropped = false;}
            sleep(0.1);
            bot_waypts = myRobot->waypts.size();
            myRobot->currTile.x = myRobot->currTile.x_tovisit;
            myRobot->currTile.y = myRobot->currTile.y_tovisit;

            nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-2]);
            printf("x: %d, y: %d\n", myRobot->currTile.x, myRobot->currTile.y);
            if(nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVisitStatus() != MazeCell::Visited) {
                myRobot->CheckLightTile();
                if(myRobot->CheckRamp()) { //is ramp
                    myRobot->MoveDistance(10000, ARobot::FRONT); //keep moving up ramp unless stopped otherwise
                    break;
                }
                if(myRobot->currTileLight == ARobot::SILVER) {
                    writeCurrentMap(this->map_dir, this->map_name, this->myRobot, this->nav);
                    myRobot->LEDLight(1000);
                    sleep(1.5);
                    //save state
                }
            }
            if(!nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVictim()) {
                //myRobot->picam.frameCapture(leftcapture_file);
                sleep(2);
                //int i = myRobot->ProcessImage_Victim();
                int i = 3;
                sleep(2);
                cout << "Victim Status: " << i << endl;
                switch(i) {
                case 0: //drop left
                    myRobot->victimLeft = true;
                    myRobot->isVictim = true;
                    myRobot->TurnDistance(90, ARobot::RIGHT); //turn left to drop from back onto right side
                    break;
                case 1: //drop front
                    myRobot->victimFront = true;
                    myRobot->isVictim = true;
                    myRobot->TurnDistance(180, ARobot::RIGHT); //turn left to drop from back onto right side
                    break;
                case 2: //drop right
                    myRobot->victimRight = true;
                    myRobot->isVictim = true;
                    myRobot->TurnDistance(90, ARobot::LEFT); //turn left to drop from back onto right side
                    break;
                case 3:
                    if(myRobot->victim.letter == 'U') { //U or nothing
                        myRobot->dropCnt = 0;
                        myRobot->LEDLight(3500);
                        sleep(4);
                        myRobot->CorrectYaw();
                        sleep(0.2);
                        break;
                    }
                    printf("Temperature Victim Results: %d\n", myRobot->CheckVictimTemp());
                    switch(myRobot->CheckVictimTemp()) {
                    case 0:
                        sleep(0.5);
                        myRobot->CorrectYaw();
                        sleep(0.2);
                        break;
                    case 1: //drop or go back to calculating
                        myRobot->victimRight = true;
                        myRobot->dropCnt = 1;
                        myRobot->isVictim = true;
                        myRobot->TurnDistance(90, ARobot::LEFT); //turn left to drop from back onto right side
                        break;
                    case 2:
                        myRobot->victimLeft = true;
                        myRobot->dropCnt = 1;
                        myRobot->isVictim = true;
                        myRobot->TurnDistance(90, ARobot::RIGHT); //turn right to drop from back onto left side
                        break;
                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
            } else {
                sleep(0.5);
                myRobot->CorrectYaw();
                sleep(0.2);
            }

            break;
        case 10:
            printf("stop\n");
            //kill thread here
            break;
        default:
            /*Testing Purposes Only*/
            sleep(1);
            break;
        }
        sleep(0.1);
    }
    pthread_cancel(pthread_self());
    sleep(0.1);
    myReadyExitFlag = true;
    return;
}

void NavThread::readConfig(const char* filename, ARobot *robot)
{
    int black_thresh, silver_thresh, speed_left, speed_right, off_left, off_right;
    float threshLeft, threshRight;
    FILE *datafile;
    if (filename == NULL)
        return;

    datafile = fopen(filename, "r");
    if(datafile == NULL) {
        printf("%s is not available\n", filename);
        return;
    }
    int ret = fscanf(datafile, "%d %d %f %f %d %d %d %d", &black_thresh, &silver_thresh, &threshLeft, &threshRight, &speed_left, &speed_right, &off_left, &off_right);
    robot->setTempThresh(threshLeft, threshRight);
    robot->setLightThresh(black_thresh, silver_thresh);
    sleep(0.5);
    robot->setSpeed(speed_left, speed_right);
    sleep(1);
    robot->setOffsetSpeed(off_left, off_right);
    sleep(1);
    fclose(datafile);
}

void NavThread::startNewMap(ARobot *robot, Navigate2D &nav_rt) {
    MazeCell::NavDir heading = MazeCell::navNorth;
    int32_t home_floor_num = 0;
    nav_rt.setHomeCell(home_floor_num, heading);
    return;
}

void NavThread::readCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt)
{
    MazeCell::NavDir heading = MazeCell::navNorth;
    int32_t home_floor_num = 0;
    std::string t = nav_rt.getCurTime();
    std::string newfile = std::string(xmlname) + "_" + t;
    if(nav_rt.readChkPtMaps(filedir, newfile.c_str())!= 0) {
        printf("call_home\n");
        nav_rt.setHomeCell(home_floor_num, heading);
        //set current robot coords to x, y
    }
    return;
}

void NavThread::writeCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt)
{
    nav_rt.writeMapFile(filedir, xmlname);
    return;
}

void NavThread::Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt) 
{
    /*Navigational functions*/
    robot->sensor_info.reset(); //reset temp object
    robot->UpdateCellMap(&robot->sensor_info, false, false); //false = not black
    robot->UpdateNeighborCells();
    cout << "Cells Updated..." << endl;

    nav_rt.configureCurCell(&robot->sensor_info);
    cout << "Configuring Current Cell with Sensor Info..." << endl;

    nav_rt.detectLocalCells(robot->temp_cell_list);
    cout << "Detecting Local Cells..." << endl;

    nav_rt.updateLocalMap();
    cout << "Local Map Updating..." << endl;

    nav_rt.getNavigateMaps()->writeXmlMap(filename, xmlname);
    cout << "Map File Written..." << endl;

    robot->temp_cell_list.clear();

    //nav_rt.slam2d(); // will move to another thread
    // what to do next
    nav_rt.navigatePlanning();
    // move on to the next cell
    //nav_rt.navigation2D();
    if(nav_rt.getNextCell()->waypts.size() >= 2) {
        robot->waypts = nav_rt.getNextCell()->waypts; //waypts
    } else {
        writeCurrentMap(this->map_dir, this->map_name, this->myRobot, this->nav);
        robot->currState = ARobot::DONE;
        robot->picam.close();
        return;
    }
    first_iter = true;
    robot->currState = ARobot::WAYPTNAV;
    return;
    //nav_rt->getCellbyIndex(nav_rt.m_next_cell.waypts.begin()).getCellGrid(&robot.currTile.x_tovisit, &robot.currTile.y_tovisit);
}

int NavThread::WayPointNav(ARobot *robot, Navigate2D &nav_rt)
{
    int x = 0; int y = 0;
    bot_waypts = robot->waypts.size();
    if(bot_waypts > 1 && first_iter == false) {//remove where u went
        robot->waypts.pop_back();
        --bot_waypts;
    }
    if(bot_waypts < 2) {
        robot->waypts.pop_back();
        --bot_waypts;
        robot->currState = ARobot::PLANNING;
        return -1;
    }
    nav_rt.getCellbyIndex(robot->waypts[bot_waypts-1])->getCellGrid(robot->currTile.x, robot->currTile.y);
    nav_rt.getCellbyIndex(robot->waypts[bot_waypts-2])->getCellGrid(robot->currTile.x_tovisit, robot->currTile.y_tovisit);
    printf("X_Tovisit: %d, Y_Tovisit: %d\n", robot->currTile.x_tovisit, robot->currTile.y_tovisit);
    for(int i = 1; i <= bot_waypts; i++) {
        nav_rt.getCellbyIndex(robot->waypts[bot_waypts-i])->getCellGrid(x, y);
        printf("Coords -> coord: %d x: %d, y: %d\n", robot->waypts[bot_waypts-i], x, y);
    }
    first_iter = false;
    robot->CalcNextTile(true);
    return 0;
}

void NavThread::DestroyThread()
{
    printf("1");
    myRobot->currState = ARobot::STOP;
    printf("2");
    myRobot->StopMove();
    //myRobot->picam.close();
    printf("3");
    this->mExitFlag = true;
    printf("4\n");
}

