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

bool AStateMachine::isLineDetected()
{
    return medge.edgeValid;
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

void AStateMachine::followLine(bool move_right, float margin = 0.0)
{
    if (margin == 0.0)
        margin = default_follow_line_margin;
    std::cout << "Following line to the " << (move_right ? "right" : "left") << " with margin " << margin << std::endl;
    mixer.setVelocity(follow_line_speed);
    mixer.setEdgeMode(!move_right, move_right ? -margin : margin);
}

void AStateMachine::turnOnItself(float target_angle)
{
    std::cout << "Turning to " << target_angle * 180 / M_PI << " degrees" << std::endl;
    pose.resetPose();
    mixer.setVelocity(0.0);
    while (pose.h < target_angle)
    {
        mixer.setTurnrate(turn_speed);
        usleep(2000);
    }
    mixer.setTurnrate(0.0);
    std::cout << "Finished turning" << std::endl;
    usleep(ONE_SECOND);
}

void AStateMachine::stopMovement(int wait_time = ONE_SECOND)
{
    mixer.setVelocity(0.0);
    usleep(wait_time);
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
    roundabout_states enter_roundabout_state = ROUNDABOUT_TURN_TO_WAIT;

    bool finished = false;
    bool lost = false;
    bool just_entered_new_state = true;
    bool intersection_detected = false;

    bool second_time_line_detected = false; // to ignore the first line detection when entering the roundabout

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
                pose.dist = 0.0;
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
                pose.dist = 0.0;
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
                    just_entered_new_state = true;
                    enter_roundabout_state = ROUNDABOUT_WAIT_FOR_REGBOT_TO_ARRIVE;
                }
                break;

            case ROUNDABOUT_WAIT_FOR_REGBOT_TO_ARRIVE:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] wait for regbot to arrive" << std::endl;
                    just_entered_new_state = false;
                }
                if (dist.dist[0] < minimum_distance_to_regbot)
                {
                    std::cout << "Regbot detected!" << std::endl;
                    enter_roundabout_state = ROUNDABOUT_WAIT_FOR_REGBOT_TO_GO;
                    pose.dist = 0.0;
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
                    pose.dist = 0.0;
                    intersection_detected = false;
                    just_entered_new_state = true;
                }
                break;

            case ROUNDABOUT_ENTER_ROUNDABOUT:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] enter roundabout" << std::endl;
                    mixer.setVelocity(follow_line_speed);
                    just_entered_new_state = false;
                }
                if (isLineDetected())
                {
                    enter_roundabout_state = ROUNDABOUT_FOLLOW_LINE;
                    pose.dist = 0.0;
                    intersection_detected = false;
                    just_entered_new_state = true;
                    stopMovement(ONE_SECOND / 2);
                }
                break;

            case ROUNDABOUT_FOLLOW_LINE:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] follow line" << std::endl;
                    followLine(FOLLOW_RIGHT);
                    just_entered_new_state = false;
                }
                if (intersection_detected)
                {
                    enter_roundabout_state = ROUNDABOUT_EXIT_ROUNDABOUT;
                    pose.dist = 0.0;
                    intersection_detected = false;
                    just_entered_new_state = true;
                }
                break;

            case ROUNDABOUT_EXIT_ROUNDABOUT:
                if (just_entered_new_state)
                {
                    std::cout << "[Enter roundabout] exit roundabout" << std::endl;
                    stopMovement();
                    turnOnItself(M_PI);
                    mixer.setVelocity(follow_line_speed);
                    usleep(ONE_SECOND * 2);
                    just_entered_new_state = false;
                }
                if (intersection_detected)
                {
                    stopMovement();
                    state = TO_AXE;
                    pose.dist = 0.0;
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
                followLine(FOLLOW_LEFT, avoid_regbot_margin);
                just_entered_new_state = false;
            }
            if (intersection_detected)
            {
                state = AXE;
                pose.dist = 0.0;
                intersection_detected = false;
                just_entered_new_state = true;

                stopMovement(); // TODO: maybe let the robot go a bit further?
            }
            break;

        case AXE:
            // Seguir la linea hasta el hacha
            if (just_entered_new_state)
            {
                std::cout << "Axe" << std::endl;
                followLine(FOLLOW_LEFT);
                just_entered_new_state = false;
            }

            // TODO add axe detection logic

            if (intersection_detected)
            {
                state = TO_CHRONO;
                pose.dist = 0.0;
                intersection_detected = false;
                just_entered_new_state = true;
            }
            break;

        case TO_CHRONO:
            // Seguir la linea hasta la interseccion que lleva al cronometro
            if (just_entered_new_state)
            {
                std::cout << "To chrono" << std::endl;
                followLine(FOLLOW_LEFT, avoid_regbot_margin);
                just_entered_new_state = false;
            }
            if (!isLineDetected())
            {
                std::cout << "Lost line" << std::endl;
                stopMovement();
                state = FIND_LINE;
                pose.dist = 0.0;
                intersection_detected = false;
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
                stopMovement();
                turnOnItself(M_PI / 2);
                pose.dist = 0.0;
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
