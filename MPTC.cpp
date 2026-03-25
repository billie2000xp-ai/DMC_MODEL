#include "MPTC.h"
#include "PTC.h"
#include "Rank.h"
#include "MemorySystemTop.h"
#include "MPFQ.h"

using namespace DRAMSim;
//==============================================================================
MPTC::MPTC(MemorySystemTop *top, vector<ofstream> &DDRSim_log_,vector<ofstream> &trace_log_,
        vector<ofstream> &cmdnum_log_, vector<vector<ofstream>> &dram_log_): 
        memorySystemTop_(top),
        DDRSim_log(DDRSim_log_),
        trace_log(trace_log_), 
        cmdnum_log(cmdnum_log_),
        dram_log(dram_log_) {
    log_path = memorySystemTop_->log_path;
    channel = memorySystemTop_->systemID * NUM_CHANS + memorySystemTop_->dmc_id;
    sub_cha = memorySystemTop_->dmc_id; 
    channel_ohot = 1ull << channel;
    initializeComponents();     
}

void MPTC::initializeComponents() {
    for (size_t i = 0; i < NUM_CHANS; ++i) {
        ptcs_.push_back(std::make_unique<PTC>(i, memorySystemTop_, DDRSim_log[i], trace_log[i], cmdnum_log[i]));
    }
    
    for (size_t ptc_index = 0; ptc_index < NUM_CHANS; ++ptc_index) {
        for (size_t rank_index = 0; rank_index < NUM_RANKS; ++rank_index) {
            unsigned rank_id = ptc_index * NUM_RANKS + rank_index;
            PTC* ptc = getPTC(ptc_index);
            auto rank = std::make_unique<Rank>(rank_id, rank_index, ptc, DDRSim_log[ptc_index], dram_log[ptc_index][rank_index]);
            rank->setPTC(ptc);
            if (ptc) {
                ptc->addRank(rank.get());
            }
            ranks_.push_back(std::move(rank));
        }
    }
}

void MPTC::associateWithPFQs(MPFQ* mpfq) {
    if (!mpfq) return;

    for (size_t i = 0; i < NUM_CHANS; ++i) {
        PTC* ptc = getPTC(i);
        if (ptc) {
            unsigned pfq_index = i / (NUM_CHANS/NUM_PFQS);
            PFQ* pfq = mpfq->getPFQ(pfq_index);           
            if (pfq) {
                ptc->setAssociatedPFQ(pfq);
            }
        }
    }
}

PTC* MPTC::getPTC(unsigned index) const {
    if (index < ptcs_.size()) {
        return ptcs_[index].get();
    }
    return nullptr;
}

Rank* MPTC::getRank(unsigned ptc_index, unsigned rank_index) const {
    unsigned global_index = ptc_index * NUM_RANKS + rank_index;
    if (global_index < ranks_.size()) {
        return ranks_[global_index].get();
    }
    return nullptr;
}

void MPTC::update() {

}

MPTC::~MPTC() {
}