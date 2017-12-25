#include "../_headers/navThread.h"

using namespace std;

void NavThread::run(void){

    sleep(2);
    readConfig(fileConfig, myRobot); //read config file about threshold calibrations
    printf("Fault 4 Passed\n");
    readCurrentMap(in_dir, xml_name, myRobot, nav); //check for previous map from mem
    printf("Fault 5 Passed\n");
    myRobot->picam.cameraOpen(320, 240); //start up camera

    myRobot->CalibrateIMU();
    myRobot->ProcessImage_Victim();
    sleep(1.5);


    while(1) {
        switch(myRobot->currState) {
            //if(cnt%100==0) {printf("State: %d\n", cnt);}
            case 0: //Planning
                Navigate(in_dir, xml_name, myRobot, nav);
                printf("navigating...\n");
                break;
            case 1: //WayPtNav
                WayPointNav(myRobot, nav);
                break;
            case 2: //Turn
                myRobot->StopTurn(myRobot->currDir);
                break;
            case 3: //Idle
                if(myRobot->toMove){
		            sleep(1.5);
		            printf("%s, %d\n", "Distance: ", myRobot->dist_temp);
                    myRobot->MoveDistance(myRobot->dist_temp, ARobot::FRONT);
                    myRobot->toMove = false;
                } else {
                    myRobot->currState = ARobot::WAYPTNAV;
                }
                break;
            case 4: //Ramp
                while(myRobot->CheckRamp()) {    
                    sleep(0.1);
                }
                myRobot->StopMove();
                break;
            case 5: //Move
                myRobot->CheckLightTile(); //check if anything happens during this time
                break;
            case 6: //Drop
                myRobot->LEDLight(4500);
                sleep(5);
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
                myRobot->UpdateCellMap(&myRobot->sensor_info, myRobot->backingBlack); //sensor_info auto resets in this function call
                nav.configureCurCell(&myRobot->sensor_info);
                nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-1]); //reupdate curr_cell
                myRobot->currState = ARobot::PLANNING;
                break;
            case 8: //DONE
                sleep(1);
		        printf("DONE!");
                break;
            case 9: //Data collection
                myRobot->isVictim = nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVictim();
                if(!myRobot->isVictim) {myRobot->isDropped = false;}
                sleep(2);
                myRobot->CalibrateIMU();
                bot_waypts = myRobot->waypts.size();
                myRobot->currTile.x = myRobot->currTile.x_tovisit;
                myRobot->currTile.y = myRobot->currTile.y_tovisit;
                //nav.getCellbyIndex(myRobot->waypts[bot_waypts-1])->getCellGrid(myRobot->currTile.x, myRobot->currTile.y);
                nav.getNavigateMaps()->getFloorMap(nav.getCurrentFloorIndex())->setCurCellIndex(myRobot->waypts[bot_waypts-2]);
                printf("x: %d, y: %d\n", myRobot->currTile.x, myRobot->currTile.y);
                sleep(0.1);
                if(nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVisitStatus() != MazeCell::Visited) {
                    //myRobot->SpinLaser();
                    //sleep(2.5); //time for laser
                    myRobot->CheckVictimVisual();

                    if(myRobot->CheckRamp()) { //is ramp
                        myRobot->MoveDistance(10000, ARobot::FRONT); //keep moving up ramp unless stopped otherwise
                        break;
                    }
                    myRobot->CheckLightTile();
                    if(myRobot->currTileLight == ARobot::SILVER) {
                        myRobot->LEDLight(5000);
                        sleep(5);
                        //save state

                    }
                    //check visual victim
                    if(!nav.getCellbyIndex(myRobot->waypts[bot_waypts-2])->getVictim()) { //get currCell
                        switch(myRobot->ProcessImage_Victim()) {
                            if(myRobot->victim.letter == 'H') { //H
                                myRobot->dropCnt = 2;
                            } else if (myRobot->victim.letter == 'S') { //S
                                myRobot->dropCnt = 1;
                            } else { //U or nothing
                                myRobot->dropCnt = 0;
                            }
                            case 0: //drop left
                                myRobot->victimLeft = true;
                                myRobot->TurnDistance(90, ARobot::RIGHT); //turn left to drop from back onto right side
                                myRobot->isVictim = true;
                                break; 
                            case 1: //drop front
                                myRobot->victimFront = true;
                                myRobot->TurnDistance(180, ARobot::RIGHT); //turn left to drop from back onto right side
                                myRobot->isVictim = true;
                                break;
                            case 2: //drop right
                                myRobot->victimRight = true;
                                myRobot->TurnDistance(90, ARobot::LEFT); //turn left to drop from back onto right side
                                myRobot->isVictim = true;
                                break;                               
                            default:
                                switch(myRobot->CheckVictimTemp()) {
                                    printf("Victim Results: %d\n", myRobot->CheckVictimTemp());
                                    case 0:
                                        myRobot->currState = ARobot::WAYPTNAV;
                                        break;
                                    case 1: //drop or go back to calculating
                                        myRobot->victimRight = true;
                                        myRobot->TurnDistance(90, ARobot::LEFT); //turn left to drop from back onto right side
                                        myRobot->dropCnt = 1;
                                        myRobot->isVictim = true;
                                        //myRobot->currState = ARobot::Drop; --> Done in StopTurn();
                                        break;
                                    case 2:
                                        myRobot->victimLeft = true;
                                        myRobot->TurnDistance(90, ARobot::RIGHT); //turn right to drop from back onto left side
                                        myRobot->dropCnt = 1;
                                        myRobot->isVictim = true;
                                        //myRobot->currState = ARobot::Drop;
                                        break;
                                    default:
                                    myRobot->currState = ARobot::WAYPTNAV;
                                        break;
                                }
                                break;
                        }
                        break;
                    } else {
                        myRobot->currState = ARobot::WAYPTNAV;
                    }
                }
                myRobot->currState = ARobot::WAYPTNAV;
                break;
            default:
                /*Testing Purposes Only*/
            	sleep(1);
                break;
        }
        //cnt++;
    }

    return;
}

void NavThread::readConfig(const char* filename, ARobot *robot)
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
    std::string t = nav_rt.getCurTime();
	std::string newfile = std::string(xmlname) + "_" + t;
    nav_rt.writeMapFile(filedir, xmlname);
	return;
}

void NavThread::Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt) 
{
    /*Navigational functions*/
    robot->sensor_info.reset(); //reset temp object
    robot->UpdateCellMap(&robot->sensor_info, false); //false = not black
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
    //nav_rt.navigation2D();
    if(nav_rt.getNextCell()->waypts.size() >= 2) {
        robot->waypts = nav_rt.getNextCell()->waypts; //waypts
    } else {
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
    robot->CalcNextTile();
    return 0;
}


