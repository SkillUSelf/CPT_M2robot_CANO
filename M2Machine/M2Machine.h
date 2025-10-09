
#ifndef M2_MACHINE_H
#define M2_MACHINE_H

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2States.h"

class M2Machine : public StateMachine {
public:
    M2Machine();
    ~M2Machine();

    void init();
    void end();

    void hwStateUpdate();

    RobotM2* robot() { return static_cast<RobotM2*>(_robot.get()); } // typed getter

    std::shared_ptr<FLNLHelper> UIserver = nullptr; // UI/command server
    // Session/subject identifier provided by Unity
    std::string sessionId = "UNSET";
};

#endif /* M2_MACHINE_H */
