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

void BPlan41::run()
{
  if (not setupDone)
    setup();
  if (ini["plan41"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  float base_velocity = 0.25;
  float turn_velocity = 0.6;
  int last_state = 4;
  int lost_counter = 0;
  bool right = true;
  int cont_int = 0;   // Contador para saber si estamos en una interseccion
  int cont_int2 = 0; // Contador para numero de intersecciones encontradas
  //
  toLog("Plan41 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 1:
        if (medge.edgeValid)
        {
          float dist_right_edge = medge.rightEdge;
          float dist_left_edge = medge.leftEdge;

          // std::cout << "Right edge: " << dist_right_edge << std::endl;
          // std::cout << "Left edge: " << dist_left_edge << std::endl;
          if (medge.width > 0.05){
            cont_int++;
          }
          else {
            cont_int = 0;
          }
          if (cont_int > 5){
            cont_int2++;
            if (cont_int2 == 1){
              state = 21;
              cont_int = 0;
            }
            else if (cont_int2 == 2){
              state = 22;
              cont_int = 0;
            }
            else if (cont_int2 == 3){
              state = 23;
              cont_int = 0;
            }
            break;
          }
          else if ((dist_right_edge < 0.0 && dist_left_edge > 0.0) && last_state != 0)
          {
            std::cout << "Both edges are found" << std::endl;
            mixer.setManualControl(true, base_velocity, 0.0);
            last_state = 0;
          }
          else if ((dist_right_edge > 0.0) && last_state != 1)
          {
            std::cout << "Correcting right edge" << std::endl;
            mixer.setManualControl(true, base_velocity, turn_velocity);
            last_state = 1;
          }
          else if ((dist_left_edge < 0.0) && last_state != 2)
          {
            std::cout << "Correcting left edge" << std::endl;
            mixer.setManualControl(true, base_velocity, -turn_velocity);
            last_state = 2;
          }
        }
        else
        {
          if (last_state != 4 && lost_counter > 10)
          {
            std::cout << "Lost" << std::endl;
            mixer.setManualControl(true, 0.0, 0.0);
            last_state = 4;
            lost_counter = 0;
          }
          else
          {
            lost_counter++;
          }
        }
        break;
      
      case 20: // forward looking for line, then turn

        mixer.setManualControl(true, 0.0, 0.0);
        std::cout << "Intersection!" << medge.width << std::endl;
        if (right){
          toLog("found intersection, turn right");
          // set to edge control, left side and 0 offset
          mixer.setManualControl(true, base_velocity, -turn_velocity);
          state = 30;
          pose.dist = 0.0;
        }
        else {
          toLog("found intersection, turn right");
          // set to edge control, left side and 0 offset
          mixer.setManualControl(true, base_velocity, turn_velocity);
          state = 30;
          pose.dist = 0.0;
        }
        right = !right;
        break;

      
      case 21: // First intersection, take right turn

        mixer.setManualControl(true, 0.0, 0.0);
        std::cout << "Intersection!" << medge.width << std::endl;
        toLog("found intersection, turn right");
        // set to edge control, left side and 0 offset
        mixer.setManualControl(true, base_velocity, -turn_velocity);
        state = 30;
        pose.dist = 0.0;
        break;
      
      case 22: // Second intersection (seesaw), keep straight

        state = 30;

        std::cout << "Intersection!" << medge.width << std::endl;
        toLog("found intersection, keep straight");
        break;

      
      case 23: // Third intersection, turn left to the stairs

        mixer.setManualControl(true, 0.0, 0.0);
        std::cout << "Intersection!" << medge.width << std::endl;
        toLog("found intersection, turn left");
        // set to edge control, left side and 0 offset
        mixer.setManualControl(true, base_velocity, turn_velocity);
        state = 30;
        pose.dist = 0.0;
        break;
        
      case 30: // Continue turn until right edge is almost reached, then follow right edge
      if (right){
        mixer.setEdgeMode(false /* right */, -0.03 /* offset */);
      }
      else {
        mixer.setEdgeMode(true /* left */, 0.03 /* offset */);
      }
      if (pose.dist > 0.15 && medge.edgeValid )
      {
        toLog("Line detected, that is OK to follow");
        
        mixer.setVelocity(base_velocity);
        state = 1;
        pose.dist = 0;
      }
      else if (t.getTimePassed() > 10)
      {
        toLog("Time passed, no crossing line");
        lost = true;
      }
      else if (pose.dist > 1.0)
      {
        toLog("Driven too long");
        state = 90;
      }
        break;
      // case 40: // follow edge until crossing line, the go straight
      //   if (medge.width > 0.075 and pose.dist > 0.2)
      //   { // go straight
      //     mixer.setTurnrate(0);
      //     pose.dist = 0;
      //     state = 50;
      //   }
      //   else if (t.getTimePassed() > 10)
      //   {
      //     toLog("too long time");
      //     finished = true;
      //   }
      //   else if (not medge.edgeValid)
      //   {
      //     toLog("Lost line");
      //     state = 80;
      //   }
      //   break;
      // case 50: // continue straight until wall is close
      //   if (dist.dist[0] < 0.15)
      //   { // wall found
      //     toLog("wall found");
      //     mixer.setVelocity(0);
      //     finished = true;
      //   }
      //   else if (t.getTimePassed() > 10)
      //   {
      //     toLog("too long time");
      //     lost = true;
      //   }
      //   else if (pose.dist > 1.5)
      //   {
      //     toLog("too far");
      //     lost = true;
      //   }
      //   break;
      default:
        lost = true;
        break;
    }
    if (state != oldstate)
    { // C-type string print
      snprintf(s, MSL, "State change from %d to %d", oldstate, state);
      toLog(s);
      oldstate = state;
      t.now();
    }
    // wait a bit to offload CPU (4000 = 4ms)
    usleep(4000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan41 got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan41 finished");
}


void BPlan41::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan41::toLog(const char* message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
           oldstate,
           message);
  }
}
