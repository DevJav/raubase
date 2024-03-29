/*
 *
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
 *
 * The MIT License (MIT)  https://mit-license.org/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE. */

#include <string>
#include <string.h>
#include <math.h>
#include <map>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"
#include "sdist.h"

#include "astatemachine.h"

#define FOLLOW_RIGHT true
#define FOLLOW_LEFT false
#define ONE_SECOND 1000000

// create class object
AStateMachine state_machine;

void AStateMachine::setup()
{ // ensure there is default values in ini-file
    if (not ini["state_machine"].has("log"))
    { // no data yet, so generate some default values
        ini["state_machine"]["log"] = "true";
        ini["state_machine"]["run"] = "false";
        ini["state_machine"]["print"] = "true";
    }
    // get values from ini-file
    toConsole = ini["state_machine"]["print"] == "true";
    //
    if (ini["state_machine"]["log"] == "true")
    { // open logfile
        std::string fn = service.logPath + "log_state_machine.txt";
        logfile = fopen(fn.c_str(), "w");
        fprintf(logfile, "%% Mission state_machine logfile\n");
        fprintf(logfile, "%% 1 \tTime (sec)\n");
        fprintf(logfile, "%% 2 \tMission state\n");
        fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
    }

    // read parameters
    minimum_line_width = strtof(ini["state_machine"]["minimum_line_width"].c_str(), nullptr);
    default_follow_line_margin = strtof(ini["state_machine"]["default_follow_line_margin"].c_str(), nullptr);
    follow_line_speed = strtof(ini["state_machine"]["follow_line_speed"].c_str(), nullptr);
    turn_speed = strtof(ini["state_machine"]["turn_speed"].c_str(), nullptr);
    threshold_distance_to_start_detection = strtof(ini["state_machine"]["threshold_distance_to_start_detection"].c_str(), nullptr);

    // read roundabout parameters
    avoid_regbot_margin = strtof(ini["state_machine"]["avoid_regbot_margin"].c_str(), nullptr);
    minimum_distance_to_regbot = strtof(ini["state_machine"]["minimum_distance_to_regbot"].c_str(), nullptr);
    distance_to_roundabout = strtof(ini["state_machine"]["distance_to_roundabout"].c_str(), nullptr);
    seconds_for_regbot_to_leave = strtof(ini["state_machine"]["seconds_for_regbot_to_leave"].c_str(), nullptr);

    // read axe parameters
    minimum_distance_to_axe = strtof(ini["state_machine"]["minimum_distance_to_axe"].c_str(), nullptr);
    free_distance_to_axe = strtof(ini["state_machine"]["free_distance_to_axe"].c_str(), nullptr);
    axe_cross_speed = strtof(ini["state_machine"]["axe_cross_speed"].c_str(), nullptr);
    approximation_distance_to_axe = strtof(ini["state_machine"]["approximation_distance_to_axe"].c_str(), nullptr);
    distance_to_cross_axe = strtof(ini["state_machine"]["distance_to_cross_axe"].c_str(), nullptr);

    // to chrono parameters
    chrono_distance_1 = strtof(ini["state_machine"]["chrono_distance_1"].c_str(), nullptr);
    chrono_distance_2 = strtof(ini["state_machine"]["chrono_distance_2"].c_str(), nullptr);
    chrono_distance_3 = strtof(ini["state_machine"]["chrono_distance_3"].c_str(), nullptr);

    // read door parameters

    minimum_distance_to_wall = strtof(ini["state_machine"]["minimum_distance_to_wall"].c_str(), nullptr);
    second_door_distance = strtof(ini["state_machine"]["second_door_distance"].c_str(), nullptr);
    dist_threshold = strtof(ini["state_machine"]["dist_threshold"].c_str(), nullptr);

    const char *calib_wood = ini["edge"]["calibwood"].c_str();
    for (int i = 0; i < 8; i++)
    {
        calibWood[i] = strtol(calib_wood, (char **)&calib_wood, 10);
    }

    setupDone = true;
}

AStateMachine::~AStateMachine()
{
    terminate();
}

enum states
{
    START_TO_FIRST_INTERSECTION,
    TO_ROUNDABOUT,
    ROUNDABOUT,
    TO_AXE,
    AXE,
    DOORS,
    TO_CHRONO,
    FIND_LINE,
    UP_RAMP,
};

enum roundabout_states
{
    ROUNDABOUT_TURN_TO_WAIT,
    ROUNDABOUT_WAIT_FOR_REGBOT_TO_ARRIVE,
    ROUNDABOUT_WAIT_FOR_REGBOT_TO_GO,
    ROUNDABOUT_ENTER_ROUNDABOUT,
    ROUNDABOUT_FOLLOW_LINE,
    ROUNDABOUT_EXIT_ROUNDABOUT,
};

enum axe_states
{
    AXE_GET_NEAR_AXE,
    AXE_WAIT_FOR_AXE,
    AXE_WAIT_FOR_FREE,
    AXE_CROSS,
    AXE_TO_INTERSECTION,
};

enum to_chrono_states
{
    TO_CHRONO_1,
    TO_CHRONO_2,
    TO_CHRONO_3,
};

enum door_states
{
    DOOR_TRAVEL_DISTANCE,
    DOOR_TURN_TO_WALL,
    DOOR_TO_WALL,
    DOOR_PERPENDICULAR_TO_WALL,
    DOOR_GET_CLOSE_TO_WALL,
    DOOR_FIRST_DOOR,
    DOOR_GET_TO_INTERSECTION,
    DOOR_SECOND_DOOR,
};

bool AStateMachine::isLineDetected(){
    return medge.edgeValid;
}

bool AStateMachine::isLineLost()
{
    if(!medge.edgeValid) 
        edge_counter++;
    else 
        edge_counter = 0;

    if (edge_counter > 20){
        edge_counter = 0;
        return true;
    }
    else if (!medge.edgeValid)
    {
        return false;
    }
    
    else{
        edge_counter = 0;
        return false;
    }
}

bool AStateMachine::detectIntersection()
{
    if (pose.dist <= threshold_distance_to_start_detection)
        return false;

    if (medge.width > minimum_line_width)
        intersection_detection_counter++;
    else
        intersection_detection_counter = 0;

    if (intersection_detection_counter > intersection_detection_threshold)
    {
        std::cout << "Intersection found!" << std::endl;
        return true;
    }
    return false;
}

void AStateMachine::followLine(bool move_right, float margin = 0.0, float speed = 0.0)
{
    if (margin == 0.0)
        margin = default_follow_line_margin;
    std::cout << "Following line to the " << (move_right ? "right" : "left") << " with margin " << margin << std::endl;
    mixer.setVelocity(speed == 0.0 ? follow_line_speed : speed);
    mixer.setEdgeMode(!move_right, move_right ? -margin : margin);
}

void AStateMachine::turnOnItself(float target_angle)
{
    std::cout << "Turning to " << target_angle * 180 / M_PI << " degrees with turn rate " << turn_speed << std::endl;
    mixer.setVelocity(0.0);
    usleep(5000);
    resetPose();
    if (target_angle < 0.0)
    {
        mixer.setTurnrate(-turn_speed);

        while (pose.h > target_angle)
        {
            // std::cout << pose.h << std::endl;
            usleep(2000);
        }
    }
    else
    {
        mixer.setTurnrate(turn_speed);

        while (pose.h < target_angle)
        {
            // std::cout << pose.h << std::endl;
            usleep(2000);
        }
    }
    // mixer.setManualControl(false, 0.0, 0.0);
    mixer.setTurnrate(0.0);
    usleep(20000);
    std::cout << "Finished turning" << std::endl;
    usleep(ONE_SECOND);
}

void AStateMachine::turnHeading(float target_angle)
{
    std::cout << "Turning to " << target_angle * 180 / M_PI << " degrees with turn rate " << turn_speed << std::endl;
    pose.turned = 0;
    pose.h = 0;
    usleep(20000);
    mixer.setVelocity(0.1);
    mixer.setDesiredHeading(target_angle);
    while (pose.h < target_angle)
    {
        // mixer.setTurnrate(turn_speed);
        // std::cout << pose.h << std::endl;
        usleep(400);
    }

    mixer.setVelocity(0.0);
    std::cout << "Finished turning" << std::endl;
    usleep(ONE_SECOND);
}

void AStateMachine::stopMovement(int wait_time = ONE_SECOND)
{
    mixer.setVelocity(0.0);
    usleep(20000);
    mixer.setTurnrate(0.0);
    usleep(20000);
    usleep(wait_time);
}

void AStateMachine::resetPose()
{
    pose.resetPose();
    usleep(50000);
}

void AStateMachine::run()
{
    if (not setupDone)
        setup();
    if (ini["state_machine"]["run"] == "false")
        return;
    UTime t("now");
    //
    toLog("State Machine started");
    //

    states state = START_TO_FIRST_INTERSECTION;
    // states state = AXE;
    roundabout_states enter_roundabout_state = ROUNDABOUT_TURN_TO_WAIT;
    axe_states axe_state = AXE_GET_NEAR_AXE;
    door_states door_state = DOOR_SECOND_DOOR;
    // to_chrono_states to_chrono_state = TO_CHRONO_1;

    bool finished = false;
    bool lost = false;
    bool just_entered_new_state = true;
    bool intersection_detected = false;

    toLog("Starting loop");

    while (not finished and not lost and not service.stop)
    {
        intersection_detected = detectIntersection();

        switch (state)
        {
        case START_TO_FIRST_INTERSECTION:
            // Seguir la linea hasta la primera interseccion
            if (just_entered_new_state)
            {
                std::cout << "Start to first intersection" << std::endl;
                followLine(FOLLOW_LEFT);
                just_entered_new_state = false;
            }
            if (intersection_detected)
            {
                state = TO_ROUNDABOUT;
                resetPose();
                intersection_detected = false;
                just_entered_new_state = true;
            }
            break;

        case TO_ROUNDABOUT:
            // Seguir la linea hasta la interseccion que lleva a la rotonda
            if (just_entered_new_state)
            {
                std::cout << "To roundabout" << std::endl;
                followLine(FOLLOW_LEFT, avoid_regbot_margin);
                just_entered_new_state = false;
            }
            if (pose.dist > distance_to_roundabout)
            {
                state = ROUNDABOUT;
                resetPose();
                intersection_detected = false;
                just_entered_new_state = true;

                stopMovement();
            }
            break;

        case ROUNDABOUT:

            // Girar hacia la derecha y avanzar para entrar al pasillito contrario a la rotonda
            switch (enter_roundabout_state)
            {
            case ROUNDABOUT_TURN_TO_WAIT:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] turn to wait" << std::endl;
                    turnOnItself(M_PI / 2);
                    mixer.setVelocity(-0.1);
                    t.now();
                    while(t.getTimePassed() < 0.8) usleep(2000);
                    // turnHeading(M_PI / 2 + 0.3);
                    // just_entered_new_state = false;

                    just_entered_new_state = true;
                    enter_roundabout_state = ROUNDABOUT_WAIT_FOR_REGBOT_TO_ARRIVE;
                }
                // if (pose.h > M_PI / 2){

                //     mixer.setTurnrate(0.0);
                //     pose.resetPose();
                //     usleep(20000);
                //     just_entered_new_state = true;
                //     enter_roundabout_state = ROUNDABOUT_WAIT_FOR_REGBOT_TO_ARRIVE;
                // }
                break;

            case ROUNDABOUT_WAIT_FOR_REGBOT_TO_ARRIVE:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] wait for regbot to arrive" << std::endl;
                    stopMovement(2000);
                    just_entered_new_state = false;
                }
                if (dist.dist[0] < minimum_distance_to_regbot)
                {
                    std::cout << "Regbot detected!" << std::endl;
                    enter_roundabout_state = ROUNDABOUT_WAIT_FOR_REGBOT_TO_GO;
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                    t.now();
                }
                break;

            case ROUNDABOUT_WAIT_FOR_REGBOT_TO_GO:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] wait for regbot to go" << std::endl;
                    std::cout << "Waiting for " << seconds_for_regbot_to_leave << " seconds" << std::endl;
                    just_entered_new_state = false;
                }
                if (t.getTimePassed() > seconds_for_regbot_to_leave)
                {
                    std::cout << "Regbot left! Entering roundabout" << std::endl;
                    enter_roundabout_state = ROUNDABOUT_ENTER_ROUNDABOUT;
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                }
                break;

            case ROUNDABOUT_ENTER_ROUNDABOUT:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] enter roundabout" << std::endl;
                    resetPose();
                    mixer.setVelocity(follow_line_speed);
                    just_entered_new_state = false;
                }
                if (isLineDetected() && pose.dist > 0.7)
                // if (pose.dist > 1.0)
                {
                    enter_roundabout_state = ROUNDABOUT_FOLLOW_LINE;
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                    stopMovement(ONE_SECOND / 2);
                }
                break;

            case ROUNDABOUT_FOLLOW_LINE:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] follow line" << std::endl;
                    mixer.setInModeTurnrate(0.075);
                    followLine(FOLLOW_RIGHT);
                    just_entered_new_state = false;
                }
                if (intersection_detected)
                {
                    enter_roundabout_state = ROUNDABOUT_EXIT_ROUNDABOUT;
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                }
                break;

            case ROUNDABOUT_EXIT_ROUNDABOUT:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] exit roundabout" << std::endl;
                    stopMovement(1000);
                    turnOnItself(M_PI - M_PI / 6); // it goes to -3.13 before reading 3.14
                    // turnHeading(M_PI);
                    mixer.setVelocity(follow_line_speed);
                    just_entered_new_state = false;
                    t.now();
                }
                if (isLineDetected() && (t.getTimePassed() > 0.5))
                {
                    stopMovement(ONE_SECOND / 2);
                    state = TO_AXE;
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                }
                break;
            }

            break;

        case TO_AXE:
            // Seguir la linea hasta la interseccion que lleva al hacha
            if (just_entered_new_state)
            {
                std::cout << "To axe" << std::endl;
                followLine(FOLLOW_RIGHT);
                just_entered_new_state = false;
            }
            if (intersection_detected)
            {
                turnOnItself(M_PI / 4 - M_PI / 12);
                // turnHeading((M_PI / 4 - M_PI / 6)*1.19);
                state = AXE;
                resetPose();
                intersection_detected = false;
                just_entered_new_state = true;
            }
            break;

        case AXE:
            // Seguir la linea hasta el hacha
            if (just_entered_new_state)
            {
                std::cout << "Axe" << std::endl;
                followLine(FOLLOW_LEFT, 0.0, 0.15);
                just_entered_new_state = false;
            }

            // TODO add axe detection logic
            switch (axe_state)
            {
            case AXE_GET_NEAR_AXE:
                if (pose.dist > approximation_distance_to_axe)
                {
                    std::cout << "[GET_NEAR_AXE] Changing to WAIT_FOR_AXE" << std::endl;
                    axe_state = AXE_WAIT_FOR_AXE;
                    stopMovement(ONE_SECOND);
                }
                break;
            case AXE_WAIT_FOR_AXE:
                if (dist.dist[0] < minimum_distance_to_axe)
                {
                    std::cout << "[WAIT_FOR_AXE] Measured distance: " << dist.dist[0] << std::endl;
                    std::cout << "[WAIT_FOR_AXE] Changing to WAIT_FOR_FREE" << std::endl;
                    axe_state = AXE_WAIT_FOR_FREE;
                }
                break;

            case AXE_WAIT_FOR_FREE:
                if (dist.dist[0] > free_distance_to_axe)
                {
                    std::cout << "[WAIT_FOR_FREE] Changing to CROSS" << std::endl;
                    axe_state = AXE_CROSS;
                    pose.resetPose();
                    usleep(2000);
                    mixer.setVelocity(axe_cross_speed);
                }

                break;

            case AXE_CROSS:
                if (pose.dist > distance_to_cross_axe)
                {
                    std::cout << "[CROSS] Changing to FOLLOW_LINE" << std::endl;
                    axe_state = AXE_TO_INTERSECTION;
                    // medge.updateCalibrationBlack(calibWood);

                    resetPose();
                    followLine(FOLLOW_RIGHT);
                    // stopMovement(4000);
                    state = DOORS;
                }
                break;

            case AXE_TO_INTERSECTION:

                if (intersection_detected)
                {
                    std::cout << "[TO_INTERSECTION] Turning" << std::endl;
                    usleep(ONE_SECOND / 3);
                    stopMovement();
                    turnOnItself(-M_PI / 2 + M_PI / 8);
                    mixer.setVelocity(follow_line_speed);
                    usleep(ONE_SECOND * 2);
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                }
            }

            break;


        case DOORS:
            switch (door_state)
            {
            case DOOR_TRAVEL_DISTANCE:
                if (just_entered_new_state)
                {
                    std::cout << "[DOORS] wait to travel" << std::endl;
                    // turnOnItself(M_PI / 2);

                    just_entered_new_state = false;
                    // door_state = TO_WALL;
                }
                if(pose.dist > 0.7){
                    stopMovement(4000);
                    door_state = DOOR_TURN_TO_WALL;
                    just_entered_new_state = true;
                    resetPose();
                }
                // }
                break;

            case DOOR_TURN_TO_WALL:
                if (just_entered_new_state)
                {
                    std::cout << "[DOORS] turn to wall" << std::endl;
                    turnOnItself(M_PI / 3);

                    just_entered_new_state = true;
                    door_state = DOOR_TO_WALL;
                }
                // }
                break;

            case DOOR_TO_WALL:
                if (just_entered_new_state)
                {
                    std::cout << "[DOORS] approach wall" << std::endl;
                    stopMovement(2000);
                    mixer.setVelocity(0.15);
                    just_entered_new_state = false;
                }
                if (dist.dist[0] < minimum_distance_to_wall)
                {
                    std::cout << "[DOORS] wall detected!" << std::endl;
                    door_state = DOOR_PERPENDICULAR_TO_WALL;
                    resetPose();
                    intersection_detected = false;
                    just_entered_new_state = true;
                    t.now();
                }
                break;

            case DOOR_PERPENDICULAR_TO_WALL:

                // std::cout << dist.dist[0] << " " << dist.dist[1] << std::endl;
                if (just_entered_new_state)
                {
                    std::cout << "[DOORS] Starting perpendicular" << std::endl;
                    stopMovement(2000);
                    mixer.setTurnrate(-0.07);
                    just_entered_new_state = false;
                }
                if (abs(dist.dist[0] - dist.dist[1]) < dist_threshold)
                {
                    std::cout << "[DOORS] Perpendicular detected!" << std::endl;
                    // door_state = DOOR_PERPENDICULAR_TO_WALL;
                    stopMovement(2000);
                    resetPose();

                    turnOnItself(-M_PI/2);
                    intersection_detected = false;
                    just_entered_new_state = true;
                    door_state = DOOR_GET_CLOSE_TO_WALL;
                }
                break;
            case DOOR_GET_CLOSE_TO_WALL:
                if (just_entered_new_state)
                    {
                        std::cout << "[DOORS] Getting close to wall" << std::endl;
                        stopMovement(2000);
                        turnOnItself(-M_PI/8);
                        resetPose();
                        mixer.setVelocity(-0.1);
                        just_entered_new_state = false;
                        t.now();
                    }
                if (t.getTimePassed() > 2.5)
                {   

                    stopMovement(2000);
                    turnOnItself(M_PI/8);
                    just_entered_new_state = true;
                    door_state = DOOR_FIRST_DOOR;
                }
                break;



            case DOOR_FIRST_DOOR:
                if (just_entered_new_state)
                {
                    std::cout << "[DOORS] Opening DOOR #1" << std::endl;
                    stopMovement(2000);
                    resetPose();
                    mixer.setVelocity(0.9);
                    just_entered_new_state = false;
                }
                if (pose.dist > 0.7)
                {
                    std::cout << "[DOORS] Door #1 Openend!" << std::endl;
                    stopMovement(2000);
                    resetPose();
                    turnOnItself(M_PI/2);
                    intersection_detected = false;
                    just_entered_new_state = true;
                    door_state = DOOR_GET_TO_INTERSECTION;
                    t.now();
                }
                break;

            case DOOR_GET_TO_INTERSECTION:
                // stopMovement(10000000);

                if (just_entered_new_state){
                    std::cout << "[DOOR] Get to intersection" << std::endl;
                    resetPose();
                    mixer.setVelocity(follow_line_speed);
                    just_entered_new_state = false;
                }

                if (intersection_detected)
                {
                    usleep(ONE_SECOND / 4);
                    stopMovement(2000);
                    turnOnItself(M_PI / 2);
                    door_state = DOOR_SECOND_DOOR;
                    just_entered_new_state = true;
                    // usleep(20000000);
                }


                break;

            case DOOR_SECOND_DOOR:
                // stopMovement(10000000);
                // std::cout << isLineDetected() << std::endl;

                if (just_entered_new_state){
                    std::cout << "[DOOR] Second door!" << std::endl;
                    resetPose();
                    followLine(FOLLOW_RIGHT);
                    just_entered_new_state = false;
                }


                if(isLineLost()){
                    stopMovement(2000);
                    turnOnItself(M_PI - 0.02);
                    // mixer.setVelocity(0.1);
                    // usleep(ONE_SECOND); 
                    just_entered_new_state = true;
                    state = TO_CHRONO;
                }
                // if (dist.dist[0] < second_door_distance)
                // {
                //     resetPose();
                //     mixer.setVelocity(0.7);
                //     t.now();
                //     if (t.getTimePassed() > 0.3)
                //         stopMovement();
                //     usleep(20000000);
                // }


                break;
            

            default:
                break;
            }



            break;









        case TO_CHRONO:


            if (just_entered_new_state)
            {
                std::cout << "Follow line to chrono" << std::endl;
                followLine(FOLLOW_RIGHT, 0.0, 0.8);
                just_entered_new_state = false;
            }
            if (isLineLost())
            {
                std::cout << "LIne lost. Arrived to chrono" << std::endl;
                stopMovement();
                state = FIND_LINE;
                just_entered_new_state = true;
            }

            break;
        case FIND_LINE:
            // Seguir recto hasta encontrar la linea
            if (just_entered_new_state)
            {
                std::cout << "Find line" << std::endl;
                mixer.setVelocity(follow_line_speed);
                just_entered_new_state = false;
            }
            if (isLineDetected())
            {
                state = UP_RAMP;
                usleep(ONE_SECOND / 4);
                stopMovement();
                turnOnItself(M_PI / 2);
                // turnHeading((M_PI/2)*1.19);
                resetPose();
                intersection_detected = false;
                just_entered_new_state = true;
            }
            break;

        case UP_RAMP:
            // Subir la rampa
            if (just_entered_new_state)
            {
                std::cout << "Up ramp (last_state)" << std::endl;
                followLine(FOLLOW_LEFT);
                just_entered_new_state = false;
            }
            break;

        default:
            break;
        }
        usleep(2000);
    }
    mixer.setVelocity(0.0);
}

void AStateMachine::terminate()
{ //
    mixer.setVelocity(0.0);
    // servo.setServo(1, false, 0.0, 0.0);
    if (logfile != nullptr)
        fclose(logfile);
    logfile = nullptr;
}

void AStateMachine::toLog(const char *message)
{
    UTime t("now");
    if (logfile != nullptr)
    {
        fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
                oldstate,
                message);
    }
    if (toConsole)
    {
        printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
               oldstate,
               message);
    }
}
