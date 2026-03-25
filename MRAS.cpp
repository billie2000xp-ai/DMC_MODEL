#include "MRAS.h"
#include "MPFQ.h"
#include "Inline_ECC.h"
using namespace DRAMSim;
//==============================================================================
MRAS::MRAS(MemorySystemTop* top, vector<ofstream> &DDRSim_log_, vector<ofstream> &trace_log_, vector<ofstream> &cmdnum_log_): 
    memorySystemTop_(top),
    DDRSim_log(DDRSim_log_),
    trace_log(trace_log_), 
    cmdnum_log(cmdnum_log_) {

    log_path = memorySystemTop_->log_path;
    channel = memorySystemTop_->systemID * NUM_CHANS + memorySystemTop_->dmc_id;
    sub_cha = memorySystemTop_->dmc_id; 
    channel_ohot = 0;

}

void MRAS::setMPFQ(MPFQ* mpfq) {
    mpfq_ = mpfq;
}

void MRAS::initializeIECCs() {
    ieccs_.clear();
    for (size_t i = 0; i < NUM_PFQS; i++) {
        auto iecc = std::make_unique<Inline_ECC>(i, memorySystemTop_, channel, DDRSim_log[i]);
        if(mpfq_) {
            iecc->setPFQ(mpfq_->getPFQ(i));
        }
        ieccs_.push_back(std::move(iecc));
    }
}

Inline_ECC* MRAS::getIECC(unsigned index) const {
    if (index < ieccs_.size()) {
        return ieccs_[index].get();
    } 
    return nullptr;   
}

void MRAS::update() {
}

MRAS::~MRAS() {
}