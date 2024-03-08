
#include <line_follower.h>
#include <iostream>
#include "medge.h"
#include "cmixer.h"

// create class object
LineFollower line_follower;

// enum for the last state
enum State
{
    BOTH_EDGES = 0,
    CORRECTING_RIGHT_EDGE = 1,
    CORRECTING_LEFT_EDGE = 2,
    LOST = 4
};

void LineFollower::setBaseVelocity(float base_velocity)
{
    std::cout << "Setting base velocity to " << base_velocity << std::endl;
    __base_velocity = base_velocity;
}

void LineFollower::setTurnVelocity(float turn_velocity)
{
    std::cout << "Setting turn velocity to " << turn_velocity << std::endl;
    __turn_velocity = turn_velocity;
}

bool LineFollower::followLine()
{
    if (medge.edgeValid)
    {
        float dist_right_edge = medge.rightEdge;
        float dist_left_edge = medge.leftEdge;

        if ((dist_right_edge < 0.0 && dist_left_edge > 0.0) && __last_state != State::BOTH_EDGES)
        {
            std::cout << "Both edges are found" << std::endl;
            mixer.setManualControl(true, __base_velocity, 0.0);
            __last_state = State::BOTH_EDGES;
        }
        else if ((dist_right_edge > 0.0) && __last_state != State::CORRECTING_RIGHT_EDGE)
        {
            std::cout << "Correcting right edge" << std::endl;
            mixer.setManualControl(true, __base_velocity, __turn_velocity);
            __last_state = State::CORRECTING_RIGHT_EDGE;
        }
        else if ((dist_left_edge < 0.0) && __last_state != State::CORRECTING_LEFT_EDGE)
        {
            std::cout << "Correcting left edge" << std::endl;
            mixer.setManualControl(true, __base_velocity, -__turn_velocity);
            __last_state = State::CORRECTING_LEFT_EDGE;
        }
    }
    else
    {
        if (__last_state != State::LOST && __lost_counter > 10)
        {
            std::cout << "Lost" << std::endl;
            mixer.setManualControl(true, 0.0, 0.0);
            __last_state = State::LOST;
            __lost_counter = 0;
            return false;
        }
        else
        {
            __lost_counter++;
        }
    }
    return true;
}