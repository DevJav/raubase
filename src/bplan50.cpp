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

#include "bplan50.h"

// create class object
BPlan50 plan50;

// enum for states
enum
{
  APROXIMATION,
  WAIT_FOR_AXE,
  WAIT_FOR_FREE,
  CROSS,
  FOLLOW_LINE,
};

void BPlan50::setup()
{ // ensure there is default values in ini-file
  if (not ini["Plan50"].has("log"))
  { // no data yet, so generate some default values
    ini["Plan50"]["log"] = "true";
    ini["Plan50"]["run"] = "false";
    ini["Plan50"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["Plan50"]["print"] == "true";
  //
  if (ini["Plan50"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan50.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission Plan50 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan50::~BPlan50()
{
  terminate();
}

void BPlan50::follow_line()
{
  if (medge.edgeValid)
  {
    float dist_right_edge = medge.rightEdge;
    float dist_left_edge = medge.leftEdge;

    if ((dist_right_edge < 0.0 && dist_left_edge > 0.0) && last_state != 0)
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
}

void BPlan50::run()
{
  if (not setupDone)
    setup();
  if (ini["Plan50"]["run"] == "false")
    return;
  UTime t("now");
  //
  toLog("Plan50 started");
  //

  int state = APROXIMATION;

  float init_distance = 0.0;

  pose.resetPose();

  while (not service.stop)
  {
    switch (state)
    {
    case APROXIMATION:
      if (pose.dist > 0.5)
      {
        init_distance = dist.dist[0];
        mixer.setManualControl(true, 0.0, 0.0);

        if (init_distance < 0.3)
        {
          std::cout << "Changing to WAIT_FOR_FREE" << std::endl;
          std::cout << init_distance << std::endl;
          state = WAIT_FOR_FREE;
        }
        else
        {
          std::cout << "Changing to WAIT_FOR_AXE" << std::endl;
          std::cout << init_distance << std::endl;
          state = WAIT_FOR_AXE;
        }
        break;
      }
      follow_line();
      break;

    case WAIT_FOR_AXE:
      if (dist.dist[0] < 0.3)
      {
        std::cout << "Changing to WAIT_FOR_FREE" << std::endl;
        state = WAIT_FOR_FREE;
        init_distance = dist.dist[0];
        std::cout << init_distance << std::endl;
      }
      break;

    case WAIT_FOR_FREE:
      if (dist.dist[0] > 0.8)
      {
        std::cout << "Changing to CROSS" << std::endl;
        state = CROSS;
        pose.resetPose();
        mixer.setManualControl(true, 0.4, 0.0);
      }
      std::cout << dist.dist[0] << std::endl;

      break;

    case CROSS:
      if (pose.dist > 1.0)
      {
        std::cout << "Changing to FOLLOW_LINE" << std::endl;
        state = FOLLOW_LINE;
      }
      break;

    case FOLLOW_LINE:
      return;
      break;

    default:
      break;
    }
    usleep(4000);
  }
}

void BPlan50::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan50::toLog(const char *message)
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
