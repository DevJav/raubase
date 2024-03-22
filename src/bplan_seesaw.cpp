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

#include "bplan_seesaw.h"

// create class object
BPlan_seesaw plan_seesaw;


void BPlan_seesaw::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan_seesaw"].has("log"))
  { // no data yet, so generate some default values
    ini["plan_seesaw"]["log"] = "true";
    ini["plan_seesaw"]["run"] = "false";
    ini["plan_seesaw"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan_seesaw"]["print"] == "true";
  //
  if (ini["plan_seesaw"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan_seesaw.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan_seesaw logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan_seesaw::~BPlan_seesaw()
{
  terminate();
}

void BPlan_seesaw::run()
{
  if (not setupDone)
    setup();
  if (ini["plan_seesaw"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 5;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  //
  toLog("Plan_seesaw started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 5: // start
        mixer.setVelocity(0.2);
        cservo.setServo(1, true, 0, 500);
        state = 10;
        break;
      case 10: // continue straight until the distance is 1.5m to be near the ball then stop and put the cservo at 0 position
        if (pose.dist > 1.5)
        { // ball found
          toLog("ball found");
          mixer.setVelocity(0);
          cservo.setServo(1, true, 0, 500);
          cservo.setPosition(0);
          state = 15;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        break;



      case 15: // continue straight for 0.5m and then set the wheels to go backwards to not fall
        if (pose.dist > 0.5)
        { // going down
          toLog("going down");
          mixer.setVelocity(-0.2);
          state = 20;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        break;
      case 20: // continue straight for 0.5m and stop
        if (pose.dist > 3)
        { // going down
          toLog("going down");
          mixer.setVelocity(0);
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        break;

        


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
    toLog("Plan_seesaw got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan_seesaw finished");
}


void BPlan_seesaw::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan_seesaw::toLog(const char* message)
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
