#ifndef _MRAS_H_
#define _MRAS_H_

#include "SimulatorObject.h"
#include "MemorySystemTop.h"
#include "Transaction.h"
#include <vector>
#include <memory>
#include <iostream>

namespace DRAMSim {

using std::ostream;

class Inline_ECC;
class MPFQ;
class MemorySystemTop;

class MRAS : public SimulatorObject {

public:
    MRAS(MemorySystemTop* top, vector<ofstream> &DDRSim_log_, vector<ofstream> &trace_log_, vector<ofstream> &cmdnum_log_);
    virtual ~MRAS();
    void setMPFQ(MPFQ* mpfq);
    void initializeIECCs();
    Inline_ECC* getIECC(unsigned index) const;
    MemorySystemTop* getMemorySystemTop() const {return memorySystemTop_;}

    void update();
private:
    MPFQ* mpfq_;
    std::vector<std::unique_ptr<Inline_ECC>> ieccs_;
    MemorySystemTop* memorySystemTop_;
    string log_path;
    vector<ofstream> &DDRSim_log;
    vector<ofstream> &trace_log;
    vector<ofstream> &cmdnum_log;
    ofstream check_log;
    uint32_t channel;
    uint32_t sub_cha;
    uint64_t channel_ohot;
};
}
#endif // _MRAS_H_