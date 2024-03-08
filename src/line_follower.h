#ifndef LINE_FOLLOWER
#define LINE_FOLLOWER

#include <thread>

using namespace std;

class LineFollower
{
public:
    /** setup and request data */
    void setup();
    /**
     * thread to do updates, when new data is available */
    void run();
    /**
     * terminate */
    void terminate();

    bool followLine();

    void setBaseVelocity(float base_velocity);
    void setTurnVelocity(float turn_velocity);

private:
    static void runObj(LineFollower *obj)
    { // called, when thread is started
        // transfer to the class run() function.
        obj->run();
    }

    std::thread *th1;

    float __base_velocity = 0.25;
    float __turn_velocity = 0.6;
    int __last_state = 4;
    int __lost_counter = 0;
};

extern LineFollower line_follower;

#endif