#pragma once

class LineFollower
{
public:
    ~LineFollower();
    bool followLine();

    void setBaseVelocity(float base_velocity);
    void setTurnVelocity(float turn_velocity);

private:
    float __base_velocity = 0.25;
    float __turn_velocity = 0.6;
    int __last_state = 4;
    int __lost_counter = 0;
};