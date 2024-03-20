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

  pose.dist = 0.0;
  bool finished = false;
  bool lost = false;

  //
  toLog("Plan41 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
    case INIT: // Follow line either right edge or left edge
      visited_counter[state]++;
      /* code */
      if (right)
      {

        mixer.setVelocity(0.3);
        mixer.setEdgeMode(false /* right */, -abs(0.03) /* offset */);
      }
      else
      {

        mixer.setVelocity(0.3);
        mixer.setEdgeMode(true /* left */, abs(0.03) /* offset */);
      }

      // Wait 10 cm to start checking again for intersections (prevents false positives)
      if (pose.dist > 0.1)
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
        if (cont_int > 5)
        {
          cont_int2++;
          if (cont_int2 == 1)
          {
            state = FIRST; // First intersection, take right turn
            cont_int = 0;
            std::cout << "First intersection" << std::endl;
          }
          if (cont_int2 == 2)
          {
            state = SECOND; // First intersection, take right turn
            cont_int = 0;
            std::cout << "Second intersection" << std::endl;
          }
          if (cont_int2 == 3)
          {
            state = THIRD; // First intersection, take right turn
            cont_int = 0;
            std::cout << "Third intersection" << std::endl;
          }
          if (cont_int2 == 4)
          {
            state = FOURTH; // First intersection, take right turn
            cont_int = 0;
            std::cout << "Fourth intersection" << std::endl;
          }
          if (cont_int2 == 5)
          {
            state = FIFTH; // First intersection, take right turn
            cont_int = 0;
            std::cout << "Fifth intersection" << std::endl;
          }
          if (cont_int2 == 6)
          {
            state = SIXTH; // First intersection, take right turn
            cont_int = 0;
            std::cout << "Sixth intersection" << std::endl;

            pose.resetPose();
            mixer.setVelocity(0.3);
            // mixer.setDesiredHeading(M_PI/2);
          }
          if (cont_int2 == 7)
          {
            state = FIRST; // First intersection, take right turn
            cont_int = 0;
            std::cout << "Seventh intersection" << std::endl;
          }
        }
      }
      break;

    case FIRST:

      if (visited_counter[state] == 1)
      {
        right = true;
        visited_counter[state]++;
      }
      else if (visited_counter[state] == 2 && pose.dist > 0.10)
      { // TODO: aun estar por ver que le mandamos
        right = true;

        visited_counter[state]++;

        pose.resetPose();
        mixer.setVelocity(0.0);
        mixer.setDesiredHeading(M_PI);
      }

      pose.dist = 0.0;
      state = INIT;
      break;

    case SECOND:
      visited_counter[state]++;

      if (visited_counter[state] == 1)
      {
        right = true;
      }
      else if (visited_counter[state] == 2)
      { // TODO: aun estar por ver que le mandamos
        right = false;
      }
      right = true;
      pose.dist = 0.0;
      state = INIT;
      break;

    case THIRD:
      visited_counter[state]++;

      right = false;
      pose.dist = 0.0;
      state = INIT;

      break;

    case FOURTH:
      visited_counter[state]++;

      right = true;
      pose.dist = 0.0;
      state = INIT;
      break;

    case FIFTH:
      visited_counter[state]++;

      right = false;
      pose.dist = 0.0;
      state = INIT;
      break;

    case SIXTH:

      if (pose.dist > 0.1)
      {
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
