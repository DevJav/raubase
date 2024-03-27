/*
 *
 * Copyright © 2023 DTU, Christian Andersen jcan@dtu.dk
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

#pragma once

using namespace std;

/**
 * Class intended to follow the defined route and accomplish the selected challenges
 * */
class AStateMachine
{
public:
    /**
     * destructor */
    ~AStateMachine();
    /** setup and request data */
    void setup();
    /**
     * run this mission */
    void run();
    /**
     * terminate */
    void terminate();

    bool isLineDetected();
    bool detectIntersection();
    void followLine(bool move_right, float margin);
    void turnOnItself(float target_angle);
    void turnHeading(float target_angle);
    void stopMovement(int wait_time);
    void resetPose();

private:
    /**
     * Write a timestamped message to log */
    void toLog(const char *message);
    /// added to log
    int state, oldstate;
    /// private stuff
    // debug print to console
    bool toConsole = true;
    // logfile
    FILE *logfile = nullptr;
    bool setupDone = false;

    // general parameters
    float minimum_line_width;
    float default_follow_line_margin{0.0};
    float follow_line_speed;
    float turn_speed;
    float threshold_distance_to_start_detection;

    // roundabout parameters
    float avoid_regbot_margin;
    float minimum_distance_to_regbot;
    float distance_to_roundabout;
    float seconds_for_regbot_to_leave;

    // axes parameters
    float minimum_distance_to_axe = 0.3;
    float free_distance_to_axe = 0.6;
    float axe_cross_speed;
    float approximation_distance_to_axe;
    float distance_to_cross_axe;

    // to chrono parameters
    float chrono_distance_1;
    float chrono_distance_2;
    float chrono_distance_3;

    // internal variables
    int intersection_detection_counter{0};
    int intersection_detection_threshold{5};
};

/**
 * Make this visible to the rest of the software */
extern AStateMachine state_machine;
