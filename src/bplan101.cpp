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
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
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
#include "maruco.h"
#include "scam.h"

#include "bplan101.h"

// create class object
BPlan101 plan101;

void BPlan101::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan101"].has("log"))
  { // no data yet, so generate some default values
    ini["plan101"]["log"] = "true";
    ini["plan101"]["run"] = "false";
    ini["plan101"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan101"]["print"] == "true";
  //
  if (ini["plan101"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan101.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan101 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan101::~BPlan101()
{
  terminate();
}

void BPlan101::run()
{
  if (not setupDone)
    setup();
  if (ini["plan101"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 11;
  oldstate = state;
  //
  toLog("Plan101 started");
  int count = 0;
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // Test ArUco plan
    case 10:
    { // brackets to allow local variables
      toLog("get ArUco");
      UTime t("now");
      int n = aruco.findAruco(0.1);
      printf("# plan101: find ArUco took %g sec\n", t.getTimePassed());
      for (int i = 0; i < n; i++)
      { // convert to robot coordinates
        cv::Vec3d pos = cam.getPositionInRobotCoordinates(aruco.arTranslate[i]);
        // rotation
        cv::Vec3f re = cam.getOrientationInRobotEulerAngles(aruco.arRotate[i], true);
        if (logfile != nullptr or toConsole)
        {
          const int MSL = 200;
          char s[MSL];
          snprintf(s, MSL, "# ArUco (%d, %d) in robot coordinates (x,y,z) = (%g %g %g)", i, aruco.arCode[i], pos[0], pos[1], pos[2]);
          toLog(s);
          snprintf(s, MSL, "# Aruco angles in robot coordinates (roll = %.1f deg, pitch = %.1f deg, yaw = %.1f deg)", re[0], re[1], re[2]);
          toLog(s);
        }
      }
      count++;
      // repeat 4 times (to get some statistics)
      if (count > 30)
        finished = true;
      break;
    }
    case 11:
    {
      cv::Mat image = cam.getFrame();
      // Convert to HSV color space (adjust values as needed)
      cv::Mat hsv_image;
      cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

      cv::Scalar lower_green(0, 100, 0);
      cv::Scalar upper_green(50, 255, 50);

      // Create mask for green pixels
      cv::Mat mask;
      cv::inRange(hsv_image, lower_green, upper_green, mask);

      // Find contours of green object
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // Calculate image center
      cv::Point image_center(image.cols / 2, image.rows / 2);

      // Iterate through green object contours
      double biggest_area = 0;
      std::vector<cv::Point> biggest_contour;
      for (const auto &contour : contours)
      {
        double area = cv::contourArea(contour);
        if (area > biggest_area)
        {
          biggest_area = area;
          biggest_contour = contour;
        }
      }

      // Find moments (weighted mean of pixels)
      cv::Moments moments = cv::moments(biggest_contour);

      // Calculate object centroid (center of mass)
      cv::Point object_centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

      // Calculate distance from image center to object centroid
      int distance_x = std::abs(object_centroid.x - image_center.x);
      int distance_y = std::abs(object_centroid.y - image_center.y);

      // Output distance information
      std::cout << "Distance from image center to object: "
                << distance_x << " x pixels" << std::endl;
      std::cout << "Object area: " << biggest_area << std::endl;

      break;
    }
    default:
      toLog("Unknown state");
      lost = true;
      break;
    }
    if (state != oldstate)
    {
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan101 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan101 finished");
}

void BPlan101::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan101::toLog(const char *message)
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
