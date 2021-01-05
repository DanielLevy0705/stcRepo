#include <Krembot/controller/krembot_controller.h>
#include "../foraging/krembot.ino.h"


class foraging_controller_2 : public foraging_controller {
private:
    enum State{
        turn,
        move,
        stop
    };

    State state = move;
    SandTimer sandTimer;
    int turning_speed = 100;
    int direction = 1;
    Colors ourColor, opponentColor;
public:
    void setup() override;
    void loop() override;
};


REGISTER_CONTROLLER(foraging_controller_2, "foraging_controller_2")