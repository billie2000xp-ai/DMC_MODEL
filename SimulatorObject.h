#ifndef _SIMULATOROBJ_H
#define _SIMULATOROBJ_H

#include <stdint.h>

namespace DRAMSim {

class SimulatorObject {
    private:
    uint64_t currentClockCycle;

    public:
    SimulatorObject() { currentClockCycle = 0; }
    void step() { currentClockCycle ++; }
    inline uint64_t now() { return currentClockCycle; }
    virtual void update() = 0;
};
}
#endif