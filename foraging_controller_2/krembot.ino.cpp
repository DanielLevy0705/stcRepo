#include "krembot.ino.h"
#include "math.h"

CVector2 pos;
CDegrees degreeX;
CVector2 home_position;
void foraging_controller_2_controller::setup() {
    krembot.setup();
    writeTeamColor();
    teamName = "foraging_controller_2_controller";
    LOGERR << foragingMsg.ourColor << std::endl;
    // init variables
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    home_position = foragingMsg.homeLocation;
}

void foraging_controller_2_controller::loop() {
    krembot.loop();
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    home_position = foragingMsg.homeLocation;
    switch (state) {
        case State::move: {
            Colors colorFront = krembot.RgbaFront.readColor();
            Colors colorFrontLeft = krembot.RgbaFrontLeft.readColor();
            Colors colorFrontRight = krembot.RgbaFrontRight.readColor();
            float distance = krembot.RgbaFront.readRGBA().Distance;
            if ((distance < 25) || (colorFront != Colors::None)  ||
                 (colorFrontLeft !=  Colors::None) || (colorFrontRight !=  Colors::None)) {
                sandTimer.start(200);
                state = State::turn;
            } else {
                krembot.Base.drive(100, 0);
            }
            if (hasFood){
                state = State::turn;
                LOGERR << "Has food" << std::endl;
            }
            break;
        }

        case State::turn: {
            //WE NEED TO CALCULATE THE DEGREE IN WHICH THE ROBOT WILL MOVE TO HOME
            if (hasFood){
                CDegrees angle = calculateDegreeHome();
                if (got_to_orientation(angle)) {
                    krembot.Base.stop();
                    state = State::move;
                } else {
                    int res = (angle - degreeX).UnsignedNormalize().GetValue();
                    if (res>50) { krembot.Base.drive(0, 50);
                        krembot.Base.drive(0, 50);
                    } else if (res>25) {
                        krembot.Base.drive(0, 25);
                    } else if (res>5) {
                        krembot.Base.drive(0, 5);
                    } else {
                        krembot.Base.drive(0, 1);
                    }
                }
            } else {
                // if dont have food, keep looking for it
                if (sandTimer.finished()) {
                    state = State::move;
                } else {
                    direction = rand() % 2;
                    if (direction == 0) {
                        direction = -1;
                    }
                    krembot.Base.drive(0, direction * turning_speed);
                }
            }
            break;
        }

    }

}

CDegrees foraging_controller_2_controller::calculateDegreeHome() {
    Real y_distance = pos.GetY() - home_position.GetY();
    Real x_distance = pos.GetX() - home_position.GetX();
    CDegrees angle = CDegrees(atan2(-y_distance,-x_distance)*180/M_PI);
    return angle;
}

bool foraging_controller_2_controller::got_to_orientation(CDegrees degree) {
    if (((degreeX - degree).UnsignedNormalize().GetValue() > 0.50) &&
        ((degreeX - degree).UnsignedNormalize().GetValue() < 359.50)) {
        return false;
    } else {
        return true;
    }
}

