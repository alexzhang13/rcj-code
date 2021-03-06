#include "navThread.h"

using namespace std;

void NavThread::run(void){
    //clock_gettime(CLOCK_REALTIME, &gettime_now);
    //start_time = gettime_now.tv_sec;
    sleep(0.2);
    myRobot->slamOut.open(slampath);
    sleep(0.2);
    readConfig(fileConfig, myRobot); //read config file about threshold calibrations
    if(this->readMap)
        readCurrentMap(map_dir, map_name, myRobot, nav); //check for previous map from mem
    else
        startNewMap(myRobot, nav);
    //myRobot->picam.cameraOpen(720, 480);
    sleep(3);

    UpdatePositionSLAM();
    myRobot->SpinLaser();
    sleep(8.5);
    myRobot->imuCalibrated = true; //turn on IMU flag

    while(!this->isExit()) {
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
            if(myRobot->toMove) {
                sleep(0.5);
                myRobot->currState = ARobot::WAYPTNAV;
            } else if(myRobot->correctionFailed) {
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
            sleep(1.25);
            myRobot->StopMove();
            //do something with myRobot-sensor_info to update the cell info
            sleep(2.5);
            myRobot->currState = ARobot::PLANNING;
            break;
        case 5: //Move
            //myRobot->CheckLightTile(); //check if anything happens during this time
            sleep(0.2);
            break;
        case 6: //Drop
            sleep(1);
            for(int i = 0; i < myRobot->dropCnt; i++) {
                myRobot->Drop();
                sleep(2.5);
            }
            myRobot->LEDLight(3000);
            sleep(3.25);
            myRobot->isDropped = true;
            nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->setVictim(true);

            //turn back
            if(myRobot->victimLeft) {
                myRobot->TurnDistance(90, ARobot::LEFT); //turn back left after right turn
            } else if(myRobot->victimFront) {
                myRobot->TurnDistance(180, ARobot::LEFT); //turn left to drop from back onto right side
            } else if(myRobot->victimRight) {
                myRobot->TurnDistance(90, ARobot::RIGHT); //turn back right after left turn
            } else { //this shouldn't happen
                myRobot->currState = ARobot::WAYPTNAV;
            }
            break;
        case 7: //BLACKBACK
            sleep(1);
            bot_waypts = myRobot->waypts.size();
            myRobot->backingBlack = false;
            nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-2]); //update "temp curr_cell"
            myRobot->UpdateCellMap(&myRobot->sensor_info, true, false); //sensor_info auto resets in this function call
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
            sleep(0.5);
            bot_waypts = myRobot->waypts.size();
            myRobot->currTile.x = myRobot->currTile.x_tovisit;
            myRobot->currTile.y = myRobot->currTile.y_tovisit;
            nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-2]);

	    UpdatePositionSLAM();
            sleep(0.5);
            myRobot->SpinLaser();
            sleep(8.5);

            printf("x: %d, y: %d\n", myRobot->currTile.x, myRobot->currTile.y);
#if 0
            if(nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVisitStatus() != MazeCell::Visited) {
                //myRobot->CheckLightTile();
                sleep(0.1);
                if(myRobot->CheckRamp()) { //is ramp
                    myRobot->currState = ARobot::RAMP;
                    myRobot->UpdateCellMap(&myRobot->sensor_info, false, true);
                    nav.configureCurCell(&myRobot->sensor_info);
                    myRobot->MoveDistance(10000, ARobot::FRONT); //keep moving up ramp unless stopped otherwise
                    break;
                }
                // cout << "Current Light Tile: " << myRobot->currTileLight << endl;
                /*if(myRobot->currTileLight == ARobot::SILVER) {
                    writeCurrentMap(this->map_dir, this->map_name, this->myRobot, this->nav);
                    myRobot->LEDLight(3000);
                    sleep(3.25);
                    //save state
                }*/
            }
            if(!nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVictim()) {
                myRobot->picam.frameCapture(leftcapture_file);
                sleep(2);
                int i = myRobot->ProcessImage_Victim();
                sleep(1);
                // cout << "Victim Status: " << i << endl;
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
                        myRobot->LEDLight(3000);
                        sleep(3.5);
                        myRobot->CorrectYaw();
                        sleep(0.2);
                        break;
                    }
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
#endif
            myRobot->currState = ARobot::WAYPTNAV;
            break;
        case 10: //STOP
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
    printf("Thread Destroyed: Signal\n");
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
    sleep(0.2);
    robot->setTempThresh(threshLeft, threshRight);
    sleep(0.2);
    robot->setLightThresh(black_thresh, silver_thresh);
    sleep(0.2);
    robot->setSpeed(speed_left, speed_right);
    sleep(0.2);
    robot->setOffsetSpeed(off_left, off_right);
    sleep(0.2);
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
    if(nav_rt.readChkPtMaps(filedir, xmlname) != 0) {
        nav_rt.setHomeCell(home_floor_num, heading);
        //set current robot coords to x, y
    }
    printf("Silver Map Loaded...\n");
    return;
}

void NavThread::writeCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt)
{
    nav_rt.writeMapFile(filedir, xmlname);
    return;
}

void NavThread::Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt) 
{
    //clock_gettime(CLOCK_REALTIME, &gettime_now);
    //time_difference = gettime_now.tv_sec - start_time;
    // cout << "\n\nNEW NAV HAS BEEN ENTERED with TIME = " << time_difference << endl << endl;

    /*Navigational functions*/
    robot->sensor_info.reset(); //reset temp object
    robot->UpdateCellMap(&robot->sensor_info, false, false); //false = not black
    robot->UpdateNeighborCells();
    // cout << "Floor Number: " << nav_rt.getCurrentFloorIndex() << endl;
    // cout << "Cell: " << nav_rt.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->getCurrentCell() << endl;

    nav_rt.configureCurCell(&robot->sensor_info);

    nav_rt.detectLocalCells(robot->temp_cell_list);

    nav_rt.updateLocalMap();

    // cout << "North Wall State: " <<  nav_rt.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->getCurrentCell()->getWallNorth() << endl;
    // cout << "South Wall State: " <<  nav_rt.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->getCurrentCell()->getWallSouth() << endl;
    // cout << "East Wall State: " <<  nav_rt.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->getCurrentCell()->getWallEast() << endl;
    // cout << "West Wall State: " <<  nav_rt.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->getCurrentCell()->getWallWest() << endl;


    robot->temp_cell_list.clear();

    //nav_rt.slam2d(); // will move to another thread
    // what to do next
    nav_rt.navigatePlanning(time_difference > 360);
    // move on to the next cell
    //nav_rt.navigation2D();
    if(nav_rt.getNextCell()->waypts.size() >= 2 && nav_rt.getNextCell()->waypts[0] != 0) {
        robot->waypts = nav_rt.getNextCell()->waypts; //waypts
    } else {
        writeCurrentMap(this->map_dir, this->map_name, this->myRobot, this->nav);
        robot->currState = ARobot::DONE;
        return;
    }
    first_iter = true;
    robot->currState = ARobot::WAYPTNAV;
    return;
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
        //printf("Coords -> coord: %d x: %d, y: %d\n", robot->waypts[bot_waypts-i], x, y);
    }
    first_iter = false;
    sleep(0.1);
    if(myRobot->toMove){
        myRobot->CalcNextTile(false);
        myRobot->toMove = false;
    } else {
        robot->CalcNextTile(true);
    }
    return 0;
}

void NavThread::DestroyThread()
{
    myRobot->StopMove();
    myRobot->currState = ARobot::STOP;
    //myRobot->picam.close();
    myRobot->slamOut.close();
    this->mExitFlag = true;
}

