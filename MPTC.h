#ifndef _MPTC_H_
#define _MPTC_H_

#include "SimulatorObject.h"
#include "MemorySystemTop.h"
#include <vector>
#include <memory>

namespace DRAMSim {
// using std::ostream;
class MemorySystemTop;
class PTC;
class Rank;
class MPFQ;

class MPTC : public SimulatorObject {

public:
    MPTC(MemorySystemTop* top, vector<ofstream> &DDRSim_log_, vector<ofstream> &trace_log_, vector<ofstream> &cmdnum_log_, vector<vector<ofstream>> &dram_log_);
    virtual ~MPTC();

    void associateWithPFQs(MPFQ* mpfq);
    PTC* getPTC(unsigned index) const;
    Rank* getRank(unsigned ptc_index, unsigned rank_index) const;
    MemorySystemTop* getMemorySystemTop() const { return memorySystemTop_; }

    void update();

private:
    std::vector<std::unique_ptr<PTC>> ptcs_;
    std::vector<std::unique_ptr<Rank>> ranks_;
    MemorySystemTop* memorySystemTop_;
    void initializeComponents();

    string log_path;
    vector<ofstream> &DDRSim_log;
    vector<ofstream> &trace_log;
    vector<ofstream> &cmdnum_log;
    vector<vector<ofstream>> &dram_log;

    ofstream check_log;
    uint32_t channel;
    uint32_t sub_cha;
    uint64_t channel_ohot;
};
}

#endif