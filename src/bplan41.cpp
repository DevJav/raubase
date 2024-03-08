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
#include "line_follower.h"
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

void BPlan41::run()
{
  if (not setupDone)
    setup();
  if (ini["Plan41"]["run"] == "false")
    return;
  UTime t("now");
  const int MSL = 100;
  char s[MSL];
  //
  toLog("Plan41 started");
  //

  // float base_velocity = 0.25;
  // float turn_velocity = 0.6;
  // int last_state = 4;
  // int lost_counter = 0;

  while (not service.stop)
  {
    bool line_found = line_follower.followLine();
    usleep(4000);
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
