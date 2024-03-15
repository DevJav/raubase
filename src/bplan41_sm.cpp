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
#include <unistd.h>
#include <map>
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
// #include "line_follower.h"
#include "bplan41.h"

// create class object
BPlan41 plan41;

void BPlan41::setup()
{ // ensure there is default values in ini-file
  if (not ini["Plan41"].has("log"))
  { // no data yet, so generate some default values
    ini["Plan41"]["log"] = "true";
    ini["Plan41"]["run"] = "false";
    ini["Plan41"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["Plan41"]["print"] == "true";
  //
  if (ini["Plan41"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan41.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission Plan41 logfile\n");
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

bool BPlan41::followLineLeft(float offset, float velocity){
    mixer.setEdgeMode(true /* left */, abs(offset) /* offset */);
    mixer.setVelocity(velocity);
}

bool BPlan41::followLineRight(float offset, float velocity){
    mixer.setEdgeMode(false /* right */, -abs(offset) /* offset */);
    mixer.setVelocity(velocity);
}

bool BPlan41::detectIntersection(int & intersection_detection_count){
    if (medge.width > 0.05){
        intersection_detection_count++;
    }
    else{
        intersection_detection_count = 0;
    }
    
    if (intersection_detection_count > 5){
        return true;
    }
    else{
        return false;
    }
}

enum intersection_state {
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
    TENTH,
    STAIRS
};

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

    int intersection_detection_count = 0;
    int intersection_count = 0;
    bool right = true;
    bool first_time_in_state = true;

    intersection_state state = INIT;
    std::map <intersection_state, int> visited_counter = {
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
        {TENTH, 0}
    };

    pose.dist = 0.0;

    switch (state)
    {
    case INIT:  // Follow line either right edge or left edge
        
        /* code */
        if(right && first_time_in_state == true){
            toLog("Following line right");
            followLineRight(0.03, 0.4);
            first_time_in_state = false;
        }
        else{
            toLog("Following line left");
            followLineLeft(0.03, 0.4);
            first_time_in_state = false;
        }

        if (detectIntersection(intersection_detection_count) && intersection_count == 0){
            state = FIRST;
            first_time_in_state = true;
            intersection_count++;
            intersection_detection_count = 0;
            pose.resetPose();
        }
        else if (detectIntersection(intersection_detection_count) && intersection_count == 1){
            state = SECOND;
            first_time_in_state = true;
            intersection_count++;
            intersection_detection_count = 0;
            pose.resetPose();
        }
        else if (detectIntersection(intersection_detection_count) && intersection_count == 2){
            state = THIRD;
            first_time_in_state = true;
            intersection_count++;
            intersection_detection_count = 0;
            pose.resetPose();
        }


        
        break;

    case FIRST:
        
        
        if (first_time_in_state == true){
            toLog("First intersection");
            visited_counter[state]++;
            first_time_in_state = false;
            followLineRight(0.03, 0.4);
        }

        if (pose.dist > 0.2){
            state = INIT;
            first_time_in_state = true;
            pose.resetPose();
        }

        break;

    case SECOND:

        if (first_time_in_state == true){
            toLog("Second intersection");
            visited_counter[state]++;
            first_time_in_state = false;
            followLineRight(0.03, 0.4);
        }

        if (pose.dist > 0.2){
            state = INIT;
            first_time_in_state = true;
            pose.resetPose();
        }
        break;

    case THIRD:
        if (first_time_in_state == true){
            toLog("Second intersection");
            visited_counter[state]++;
            first_time_in_state = false;
            followLineLeft(0.03, 0.1);
        }

        if (pose.dist > 0.2){
            state = STAIRS;
            first_time_in_state = true;
            pose.resetPose();
        }
                
        break;

    case STAIRS:
        if (first_time_in_state == true){
            toLog("STAIRS");
            visited_counter[state]++;
            first_time_in_state = false;
            followLineLeft(0.03, 0.1);
        }

        if (detectIntersection(intersection_detection_count) && intersection_count == 3){
            state = FOURTH;
            first_time_in_state = true;
            intersection_count++;
            intersection_detection_count = 0;
            pose.resetPose();
        }
        break;

    case FOURTH:
        if (first_time_in_state == true){
            toLog("Fourth intersection");
            visited_counter[state]++;
            first_time_in_state = false;
            followLineRight(0.03, 0.4);
        }

        if (pose.dist > 0.2){
            state = INIT;
            first_time_in_state = true;
            pose.resetPose();
        }


        break;

    case FIFTH:
        visited_counter[state]++;
        
        right = false;
        pose.dist = 0.0;
        state = INIT;
        break;

    case SIXTH: 
        
        
        if (pose.dist > 0.1){
            visited_counter[state]++;

            pose.dist = 0.0;
            state = INIT;
        }
        break;

    case SEVENTH:
        visited_counter[state]++;

        
        
        right = true;
        pose.dist = 0.0;
        state = INIT;
        break;
    
    default:
        break;
    }
    usleep(2000);
    
}

void BPlan41::terminate()
{ //
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
