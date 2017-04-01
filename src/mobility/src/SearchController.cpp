#include "SearchController.h"
#include <math.h> /*fabs*/

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    this->init = false;
    this->prelim = true;
    this->numOfItr = 0;
    this->ratio = 0;
    this->started = false;
    this->startingLocation = false;
    this->checkpoint = 0;
    this->shiftX = 0;
    this->shiftY=0;
    this->rover;
    this->offShift = 1.5;
    this->movementTracker=1;

}

    static int DECREASER = 0;
    static int COUNTER = 0;

geometry_msgs::Pose2D SearchController::search(string robotName, geometry_msgs::Pose2D centerLocation,
                                               geometry_msgs::Pose2D currentLocation, double oTheta) {
    geometry_msgs::Pose2D goalLocation;

    ROS_INFO("runningSearchPattern");

    if(!init){
        //make robots move to their starting position
        //pulling information (starting width) from the server
        ros::NodeHandle n;
        ros::ServiceClient setArenaClient = n.serviceClient<hive_srv::setArena>("set_arena");
        hive_srv::setArena srv;
        srv.request.robotName = robotName;
        // this wont twerk :(
        // prelim = srv.response.prelim; // figuring out if we're in prelim or not


        if(setArenaClient.call(srv)){
            startSearchWidth = srv.response.searchStartWidth;
            endSearchWidth = srv.response.searchEndWidth;

            // if the startSearchWidth is 0, that means we're in a non prelim round
            if (startSearchWidth != 0)
                prelim = true;

            ROS_INFO("Values are set");
        }
        else {
            ROS_INFO("Could not call set arena client");
        }

        //set the adjusts
        ros::ServiceClient getPosAdjust = n.serviceClient<hive_srv::getPosAdjust>("get_pos_adjust");
        hive_srv::getPosAdjust psrv;
        psrv.request.robotName = robotName;

        if(getPosAdjust.call(psrv)){
            this->posAdjustX = psrv.response.posAdjustX;
            this->posAdjustY = psrv.response.posAdjustY;

            ROS_INFO("Adjusts set for: %s, X: %f, Y: %f", robotName.c_str(), posAdjustX, posAdjustY);
        } else {
            ROS_INFO("Could not call position adjust client");
        }


        // this is assigning IDs to the rovers for non prelim rounds
        if((10* oTheta) >27)
            rover=3; // pi
        else if((10* oTheta) >11)
            rover=4; // pi/2 NOT 3*pi/4 ***show amit
        else if((10 * oTheta) > 4)
            rover=5; // pi/4
        else if((10 * oTheta) >-5)
            rover=6; // 0
        else if((10 * oTheta) >-20)
            rover=1; // -pi/2
        else if((10 * oTheta) >-27)
            rover=2; // -3*pi/4
        else
            rover=3; // -pi

        ROS_INFO("*********just set rover number to %d, ***** looking at %f, *****",rover,oTheta);

        init = true; // set initialization to true because we don't need to go through it again
    } // end of init

    // This block of code executes when there are only three rovers
    // this needs to be set back to true after testing ***
    if (prelim) {

        //check i = falsef we are at the starting location
        switch (movementTracker)
            {
            case 1:
                goalLocation.theta = goalLocation.theta + 3*M_PI/4;        //Go to Upper-Left Corner

                goalLocation.x = currentLocation.x + ((7.5) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((7.5) * sin(goalLocation.theta));

                movementTracker = 2;

                COUNTER++;
                break;
            case 2:
                goalLocation.theta = goalLocation.theta + 0;               //Turn Right

                goalLocation.x = currentLocation.x + ((9.8-DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8-DECREASER) * sin(goalLocation.theta));

                movementTracker = 3;

                COUNTER++;
                break;
            case 3:
                goalLocation.theta = goalLocation.theta + 3*M_PI/2;        //Turn Down

                if(COUNTER > 4){
                    DECREASER = DECREASER - 2;
                }

                goalLocation.x = currentLocation.x + ((9.8 - DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8 - DECREASER) * sin(goalLocation.theta));

                movementTracker = 4;

                COUNTER++;
                break;
            case 4:
                goalLocation.theta = goalLocation.theta + M_PI;            //Turn Left

                goalLocation.x = currentLocation.x + ((9.8-DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8-DECREASER) * sin(goalLocation.theta));

                movementTracker = 5;

                COUNTER++;
                break;
            case 5:
                goalLocation.theta = goalLocation.theta + M_PI/2;          //Turn Up

                DECREASER = DECREASER - 2;

                goalLocation.x = currentLocation.x + ((9.8 - DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8 - DECREASER) * sin(goalLocation.theta));

                movementTracker = 2;

                COUNTER++;
                break;
            }

        }
    // end of prelim search

    // for testing, i'm making this execute if prelim is TRUE cause my laptop cant handle 6 rovers ***
    // this algorithm executes when there are 6 rovers
    if (!prelim) {


        //check i = falsef we are at the starting location
        switch (movementTracker) {
            case 1:
                goalLocation.theta = goalLocation.theta + 3 * M_PI / 4;        //Go to Upper-Left Corner

                goalLocation.x = currentLocation.x + ((7.5) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((7.5) * sin(goalLocation.theta));

                movementTracker = 2;

                COUNTER++;
                break;
            case 2:
                goalLocation.theta = goalLocation.theta + 0;               //Turn Right

                goalLocation.x = currentLocation.x + ((9.8 - DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8 - DECREASER) * sin(goalLocation.theta));

                movementTracker = 3;

                COUNTER++;
                break;
            case 3:
                goalLocation.theta = goalLocation.theta + 3 * M_PI / 2;        //Turn Down

                if (COUNTER > 4) {
                    DECREASER = DECREASER - 2;
                }

                goalLocation.x = currentLocation.x + ((9.8 - DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8 - DECREASER) * sin(goalLocation.theta));

                movementTracker = 4;

                COUNTER++;
                break;
            case 4:
                goalLocation.theta = goalLocation.theta + M_PI;            //Turn Left

                goalLocation.x = currentLocation.x + ((9.8 - DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8 - DECREASER) * sin(goalLocation.theta));

                movementTracker = 5;

                COUNTER++;
                break;
            case 5:
                goalLocation.theta = goalLocation.theta + M_PI / 2;          //Turn Up

                DECREASER = DECREASER - 2;

                goalLocation.x = currentLocation.x + ((9.8 - DECREASER) * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + ((9.8 - DECREASER) * sin(goalLocation.theta));

                movementTracker = 2;

                COUNTER++;
                break;
        }

        //ROS_INFO("Returning to transform from searchController");
    }
    return goalLocation;
}


/* Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
    geometry_msgs::Pose2D newGoalLocation;

    //remainingGoalDist avoids magic numbers by calculating the dist
    double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.8 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.8 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));


    return newGoalLocation;
}
