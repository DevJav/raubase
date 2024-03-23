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

#include "bplan41.h"

// create class object
BPlan41 plan41;

void BPlan41::setup()
{ // ensure there is default values in ini-file
    if (not ini["plan41"].has("log"))
    { // no data yet, so generate some default values
        ini["plan41"]["log"] = "true";
        ini["plan41"]["run"] = "false";
        ini["plan41"]["print"] = "true";
    }
    // get values from ini-file
    toConsole = ini["plan41"]["print"] == "true";
    //
    if (ini["plan41"]["log"] == "true")
    { // open logfile
        std::string fn = service.logPath + "log_plan41.txt";
        logfile = fopen(fn.c_str(), "w");
        fprintf(logfile, "%% Mission plan41 logfile\n");
        fprintf(logfile, "%% 1 \tTime (sec)\n");
        fprintf(logfile, "%% 2 \tMission state\n");
        fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
    }
    setupDone = true;
}

BPlan41::~BPlan41()
{
    terminate();
}

enum intersection_state
{
    INIT,
    FIRST,
    SECOND,
    THIRD,
    FOURTH,
    FIFTH,
    SIXTH,
    SEVENTH,
    EIGHTH,
    NINTH,
    TENTH
};

void BPlan41::movement(bool right, float margin = 0.03, float speed = 0.3)
{
    if (right)
    {

        mixer.setVelocity(0.3);
        usleep(200);
        mixer.setEdgeMode(false /* right */, -abs(margin) /* offset */);
        usleep(200);
    }
    else
    {

        mixer.setVelocity(0.3);
        usleep(200);
        mixer.setEdgeMode(true /* left */, abs(margin) /* offset */);
        usleep(200);
    }
}

void BPlan41::turn(float angle, bool first_time)
{
    if (first_time == true)
    {
        pose.resetPose();
        mixer.setVelocity(0.0);
    }
    mixer.setTurnrate(0.2);
    // std::cout << "first_time" << first_time << " pose.h " << pose.h << std::endl;
    if (pose.h > angle)
        mixer.setTurnrate(0.0);
}

void BPlan41::run()
{
    if (not setupDone)
        setup();
    if (ini["Plan41"]["run"] == "false")
        return;
    UTime t("now");
    //
    toLog("Plan41 started");
    //

    int cont_int = 0;
    int cont_int2 = 0;
    bool right = true;

    intersection_state state = INIT;
    std::map<intersection_state, int> visited_counter = {
        {INIT, 0},
        {FIRST, 0},
        {SECOND, 0},
        {THIRD, 0},
        {FOURTH, 0},
        {FIFTH, 0},
        {SIXTH, 0},
        {SEVENTH, 0},
        {EIGHTH, 0},
        {NINTH, 0},
        {TENTH, 0}};

    pose.resetPose();
    bool finished = false;
    bool lost = false;

    bool first_time = true;

    //
    toLog("Plan41 started");
    //

    bool servo_enabled = (ini["servo_control"]["enabled"] == "true");
    int servo_position = strtof(ini["servo_control"]["position"].c_str(), nullptr);
    int servo_velocity = strtof(ini["servo_control"]["velocity"].c_str(), nullptr);

    // std::cout << servo_enabled << servo_position << servo_velocity << std::endl;

    servo.setServo(1, servo_enabled, servo_position, servo_velocity);

    int max_cont = 5;

    bool new_inters = false;

    float travel_dist = 0.3;

    while (not finished and not lost and not service.stop)
    {
        if (pose.dist > travel_dist)
        {
            if (medge.width > 0.05)
            {
                cont_int++;
            }
            else
            {
                cont_int = 0;
            }

            // If 5 positives are found, then we are in an intersection
            if (cont_int > max_cont)
            {
                std::cout << "Intersection found" << std::endl;
                new_inters = true;
            }
        }
        switch (state)
        {
        case INIT: // Follow line either right edge or left edge
            // std::cout << "Enabled:" << servo_enabled << "Position" << servo_position << "vel" << servo_velocity << std::endl;
            // std::cout << t.getTimePassed() << std::endl;
            // if (first_time)
            // {
            //     mixer.setVelocity(0.3);
            //     t.now();
            //     first_time = false;
            // }
            // else if (t.getTimePassed() > 3)
            // {
            //     std::cout << "Turning" << std::endl;
            //     mixer.setVelocity(0.0);
            //     mixer.setDesiredHeading(M_PI / 2);
            // }

            movement(true, 0.04);
            usleep(2000);

            if (new_inters)
            {
                state = FIRST;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }

            break;

        case FIRST:
            // Girar a la izquierda en la primera interseccion despues de la guillotina
            movement(false, 0.00);
            if (first_time)
            {
                std::cout << "First" << std::endl;

                usleep(2000);
                first_time = false;
                pose.resetPose();
                servo_position = -200;
                // servo.setServo(1, servo_enabled, servo_position, servo_velocity);
            }
            if (pose.dist > 1.05)
            {
                state = SECOND;
                pose.resetPose();
                new_inters = false;
                first_time = true;
                t.now();
            }
            servo_position = 200;
            // servo.setServo(1, servo_enabled, servo_position, servo_velocity);

            break;

        case SECOND:
            // Llegar a interseccion para entrar en la rotonda. Girar 90 grados y esperar a poder pasar sin chocar con el robot.
            turn(M_PI / 2, first_time);

            usleep(2000);
            if (first_time)
            {
                std::cout << "Second" << std::endl;
                first_time = false;
            }

            // Esperara a que pase el regbot por delante. Avanzar.
            if (dist.dist[0] < 0.3 && pose.h > M_PI / 2 && t.getTimePassed() > 2.0)
            {
                state = THIRD;
                pose.resetPose();
                new_inters = false;
                first_time = true;
                // t.now();
            }

            break;

        case THIRD:

            // Girar de nuevo a izquierda para entrar en la rotonda y luego seguir la linea hasta llegar al punto inicial (nueva interseccion)
            // turn(-M_PI / 2, first_time);
            mixer.setVelocity(0.2);
            if (first_time)
            {
                std::cout << "Third" << std::endl;

                usleep(2000);
                first_time = false;
            }
            if (medge.edgeValid)
            {
                state = FOURTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }

            break;

        case FOURTH:

            // Girar de nuevo a izquierda para salir de la rotonda
            // turn(-M_PI / 2, first_time);
            movement(true, 0.03, 0.2);
            if (first_time)
            {
                std::cout << "Fourth" << std::endl;
                first_time = false;
            }
            if (new_inters)
            {
                state = FIFTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }

            break;

        case FIFTH:
            // avanzar por la union entre la rotonda y el circuito hasta la interseccion
            turn(M_PI / 2, first_time);
            if (first_time)
            {
                std::cout << "FIFHT" << std::endl;
                // movement(true);
                first_time = false;
            }
            if (new_inters)
            {
                state = SIXTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }

            break;

        case SIXTH:

            // Girar izquierda y volver al trazo principal
            // Continuar linea y esperar a siguiente interseccion (hacha)
            if (first_time)
            {
                std::cout << "SIXTH" << std::endl;
                // movement(true);
                mixer.setVelocity(0.4);
                travel_dist = 0.1;
                first_time = false;
            }
            if (new_inters)
            {
                state = SEVENTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
                travel_dist = 0.3;
            }

            break;

        case SEVENTH:

            turn(M_PI / 2, first_time);
            if (first_time)
            {
                std::cout << "SEVENTH" << std::endl;
                // turn(-M_PI / 2);
                // movement(false);
                first_time = false;
            }
            if (new_inters)
            {
                state = NINTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }
            break;

        case EIGHTH:
            // Implementar logica para pasar el hacha, continuar hasta siguiente interseccion y seguir hasta el fin de la linea
            if (first_time)
            {
                movement(false);
                first_time = false;
                pose.resetPose();
            }
            if (new_inters)
            {
                state = EIGHTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }
            break;

        case NINTH:
            if (first_time)
            {
                // turn(-M_PI / 2);
                movement(false);
                first_time = false;
                pose.resetPose();
            }
            if (new_inters)
            {
                state = EIGHTH;
                pose.resetPose();
                new_inters = false;
                first_time = true;
            }
            break;

        default:
            break;
        }
        usleep(2000);
    }
    mixer.setVelocity(0.0);
}

void BPlan41::terminate()
{ //
    mixer.setVelocity(0.0);
    // servo.setServo(1, false, 0.0, 0.0);
    if (logfile != nullptr)
        fclose(logfile);
    logfile = nullptr;
}

void BPlan41::toLog(const char *message)
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
