#include <Krembot/controller/krembot_controller.h>
#include "../foraging/krembot.ino.h"


class foraging_controller_2_controller : public foraging_controller {
private:
    enum State{
        turn,
        move,
        stop,
        block
    };

    State state = move;
    SandTimer sandTimer;
    int turning_speed = 100;
    int direction = 1;
    bool flag;
    Colors ourColor, opponentColor;
public:
    void setup() override;
    void loop() override;
    /*
     * This function calculates the degree in which the robot should move towards the nest.
     */
    CDegrees calculateDegreeHome();
    /*
     * This function is taken from class and checks if we got to the right orientation.
     */
    bool got_to_orientation(CDegrees degree);

    Colors convertStringToColor(std::string col);

};


REGISTER_CONTROLLER(foraging_controller_2_controller, "foraging_controller_2_controller")