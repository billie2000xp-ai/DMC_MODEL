#include "MPFQ.h"
#include "PFQ.h"
#include "MPTC.h"
#include "PTC.h"
#include "MemorySystemTop.h"
#include <assert.h>
#include <iomanip>

using namespace DRAMSim;

// ============================
// CONSTRUCTOR AND DESTRUCTOR
// ============================
MPFQ::MPFQ(MemorySystemTop* top, vector<ofstream> &DDRSim_log_, vector<ofstream> &trace_log_) :
    memorySystemTop_(top),
    DDRSim_log(DDRSim_log_),
    trace_log(trace_log_),
    ReturnReadData(NULL),
    WriteResp(NULL),
    ReadResp(NULL),
    CmdResp(NULL) {
    
    channel_ohot = 0;

    for (size_t i = 0; i < NUM_PFQS; i++) {
        pfqs_.push_back(std::make_unique<PFQ>(i, top, this, DDRSim_log[i], trace_log[i]));
    }
    
    // ============ LOG INITIALIZATION ============
    if (DEBUG_PERF) {
        perf_log.open("./perf.log");
        if (!perf_log.is_open()) {
            ERROR("open perf.log error!!!!");
            exit(0);
        }
    }
       
}

MPFQ::~MPFQ() {
    // ============ CLEAN UP PFQ INSTANCES ============
    // for (auto pfq : pfqs_) {       
    //     delete pfq;       
    // }
    
    // ============ CLOSE LOG FILES ============
    if (perf_log.is_open()) {
        perf_log.close();
    }
}

void MPFQ::setMPTC(MPTC* mptc) {
    mptc_ = mptc;
}

void MPFQ::associateWithPTCs() {
    if (!mptc_) return;
    
    for (size_t pfq_index = 0; pfq_index < NUM_PFQS; ++pfq_index) {
        PFQ* pfq = getPFQ(pfq_index);
        if (!pfq) continue;
        
        for (size_t ptc_offset = 0; ptc_offset < (NUM_CHANS/NUM_PFQS); ++ptc_offset) {
            unsigned ptc_index = pfq_index * (NUM_CHANS/NUM_PFQS) + ptc_offset;
            PTC* ptc = mptc_->getPTC(ptc_index);
            if (ptc) {
                pfq->addAssociatedPTC(ptc);
            }
        }
    }
}

PFQ* MPFQ::getPFQ(unsigned index) const {
    if (index < pfqs_.size()) {
        return pfqs_[index].get();
    }
    return nullptr;
}

// ============================
// FAST READ FUNCTIONS
// ============================
void MPFQ::addFastRead(unsigned pfq_idx) {
    uint32_t chl = 0; //modify
    if (!FAST_READ_EN) return;
    bool all_empty = true;
    for (size_t ch = pfq_idx*(NUM_CHANS/NUM_PFQS); ch < ((pfq_idx+1)*(NUM_CHANS/NUM_PFQS)); ch++) {
        if (!mptc_->getPTC(ch)->fastrd_fifo.empty()) {
            chl = ch;
            all_empty = false;
            break;
        }
    }
    if (all_empty) return;

    // fastrd_fifo size check
    if (mptc_->getPTC(chl)->fastrd_fifo.size() > 1) {
        ERROR(setw(10)<<now()<<" Fast Read Cmd More Than One, size="<<mptc_->getPTC(chl)->fastrd_fifo.size());
        assert(0);
    }

    auto fastrd_trans = mptc_->getPTC(chl)->fastrd_fifo[0];
    auto state = mptc_->getPTC(chl)->bankStates[fastrd_trans->bankIndex];
    unsigned sub_channel = (fastrd_trans->bankIndex % NUM_BANKS) / (mptc_->getPTC(chl)->sc_bank_num);
    bool refresh_waiting = mptc_->getPTC(chl)->refreshPerBank[fastrd_trans->bankIndex].refreshWaiting;
    bool refresh_all_waiting = mptc_->getPTC(chl)->refreshALL[fastrd_trans->rank][sub_channel].refreshWaiting; 
    bool refreshing = mptc_->getPTC(chl)->refreshPerBank[fastrd_trans->bankIndex].refreshing;
    bool refreshing_all = mptc_->getPTC(chl)->refreshALL[fastrd_trans->rank][sub_channel].refreshing;
    bool perf_rnkgrp = PERF_RANK_RMERGE_EN && (pfqs_[pfq_idx]->wbuff_rnkgrp_state == WBUFF_IN_RNK_GROUP) 
                                           && (fastrd_trans->rank != pfqs_[pfq_idx]->perf_sch_rrank);
    bool ptc_full = mptc_->getPTC(chl)->full();
    bool ptc_bs_full = mptc_->getPTC(chl)->get_bs_full(fastrd_trans);

    //delete stored fast read cmd
    if (!mptc_->getPTC(chl)->fastrd_fifo.empty()) {
        bool fastrd_del = (pfqs_[pfq_idx]->wbuff_state == WBUFF_WRITE) || (mptc_->getPTC(chl)->rank_cnt[fastrd_trans->rank] >= PTC_MAX_CNT_PER_RANK
                            || refresh_waiting || refresh_all_waiting || refreshing || refreshing_all) || perf_rnkgrp || ptc_full || ptc_bs_full;
        if (fastrd_del) {
            if (DEBUG_BUS) {
                PRINTN_M(pfq_idx, setw(10)<<now()<<" -- Fast Read Cmd Unvalid :: task="<<fastrd_trans->task<<", rank="<<fastrd_trans->rank
                        <<", bank="<<fastrd_trans->bankIndex<<", perf_state="<<pfqs_[pfq_idx]->wbuff_state
                        <<", ptc_rank_cnt="<<mptc_->getPTC(chl)->rank_cnt[fastrd_trans->rank]
                        <<", refreshWaiting="<<refresh_waiting<<", refreshALLWaiting="<<refresh_all_waiting
                        <<", refreshing="<<refreshing<<", refreshing_all="<<refreshing_all<<endl);
            }
            mptc_->getPTC(chl)->fastrd_fifo.erase(mptc_->getPTC(chl)->fastrd_fifo.begin());
            return;
        }
    }

    //ptc add fast rd cmd at odd cycle or no perf_cmd_vld at even cycle    
    if (!mptc_->getPTC(chl)->fastrd_fifo.empty() && ((((now()%CLKH2CLKL==1) || (now()%CLKH2CLKL==0 && !pfqs_[pfq_idx]->perf_cmd_vld)) && PERF_CLK_MODE==1) || PERF_CLK_MODE==0)) { 
        if (DEBUG_BUS) {
            PRINTN_M(pfq_idx, setw(10)<<now()<<" -- pfq["<<pfq_idx<<"]->perf_cmd_vld="<<pfqs_[pfq_idx]->perf_cmd_vld<<", now()%CLKH2CLKL="<<now()%CLKH2CLKL<<endl);
        }
        if (!fastrd_trans->fast_rd) {
            ERROR(setw(10)<<now()<<" Fast Read Label Failed, task="<<fastrd_trans->task);
            assert(0);
        }
        if (state.ptc_samebank_rd > 0 || mptc_->getPTC(chl)->r_bank_cnt[fastrd_trans->bankIndex] > 0
                || state.ptc_samebank_wr > 0 || mptc_->getPTC(chl)->w_bank_cnt[fastrd_trans->bankIndex] > 0) {
            ERROR(setw(10)<<now()<<" Fast Read Conflict with Ptc Cmd, task="<<fastrd_trans->task<<", ptc_samebank_rd="<<state.ptc_samebank_rd
                    <<", r_bank_cnt"<<mptc_->getPTC(chl)->r_bank_cnt[fastrd_trans->bankIndex]<<", perf_rd_conf="<<state.perf_bankrd_conflict
                    <<", ptc_samebank_rd="<<state.ptc_samebank_wr<<", w_bank_cnt"<<mptc_->getPTC(chl)->w_bank_cnt[fastrd_trans->bankIndex]
                    <<", perf_wr_conf="<<state.perf_bankwr_conflict);
            assert(0);
        }
        bool fastrd_pos = mptc_->getPTC(chl)->addTransaction(fastrd_trans, false);
        if (fastrd_pos) {
            if (DEBUG_BUS) {
                PRINTN_M(pfq_idx, setw(10)<<now()<<" -- ADD_DMC (FAST_RD) :: [R]B["<<fastrd_trans->burst_length<<"]QOS["<<fastrd_trans->qos
                        <<"]MID["<<fastrd_trans->mid<<"] addr=0x"<<hex<<fastrd_trans->address<<dec<<" task="<<fastrd_trans->task
                        <<" rank="<<fastrd_trans->rank<<" group="<<fastrd_trans->group<<" bank="<<fastrd_trans->bank<<" bankIndex="
                        <<fastrd_trans->bankIndex<<" row="<<fastrd_trans->row<<" col="<<fastrd_trans->col<<" addr_col="<<fastrd_trans->addr_col
                        <<" data_size="<<fastrd_trans->data_size<<" Q="<<mptc_->getPTC(chl)->GetDmcQsize()
                        <<" ptc_timeAdded="<<fastrd_trans->ptc_timeAdded<<" timeout_th="<<fastrd_trans->timeout_th
                        <<" timeAdded="<<fastrd_trans->timeAdded<<" mask_wcmd="<<fastrd_trans->mask_wcmd<<" perf_rd_rowhit="
                        <<fastrd_trans->perf_rd_rowhit<<" perf_rd_rowmiss="<<fastrd_trans->perf_rd_rowmiss<<" perf_maxbg_hit="<<fastrd_trans->perf_maxbg_hit
                        <<" perf_qos="<<fastrd_trans->perf_qos<<" pri="<<fastrd_trans->pri<<" byp_act="<<fastrd_trans->byp_act
                        <<" fast_rd="<<fastrd_trans->fast_rd<<" act_only="<<fastrd_trans->act_only<<endl);
            }
            // set fast_rd flag in perf queue
            bool exist_fastrd = false;
            for (auto &rtrans : pfqs_[pfq_idx]->PerfQue) {
                if (rtrans == nullptr) continue;
                if (rtrans->transactionType != DATA_READ) continue;
                if (rtrans->task != fastrd_trans->task) continue;
                exist_fastrd = true;
                if (FAST_READ_MODE == 1) {  // full command mode
                    rtrans->fast_rd = fastrd_trans->fast_rd;
                }
                if (DEBUG_BUS) {
                    PRINTN_M(pfq_idx, setw(10)<<now()<<" -- SET FAST_RD IN PERF, task="<<rtrans->task<<" fast_rd="<<rtrans->fast_rd
                            <<" act_only="<<rtrans->act_only<<endl);
                }
            }

            if (!exist_fastrd) {
                ERROR(setw(10)<<now()<<" Fast Rd Not Found in Perf :: task="<<fastrd_trans->task<<" fast_rd="<<fastrd_trans->fast_rd
                        <<" act_only="<<fastrd_trans->act_only);
                assert(0);
            }

            mptc_->getPTC(chl)->fastrd_fifo.erase(mptc_->getPTC(chl)->fastrd_fifo.begin());
        } else {
            ERROR(setw(10)<<now()<<" Fast Read Cmd Met, but PTC reject :: task="<<fastrd_trans->task<<", ptc_size="<<mptc_->getPTC(chl)->transactionQueue.size()
                    <<", channel="<<chl<<", ptc_pre_req_time="<<mptc_->getPTC(chl)->pre_req_time);
            assert(0);
        }
    }
}

// ============================
// READ DATA RETURN FUNCTIONS
// ============================
bool MPFQ::returnReadData(data_packet& packet) {
    return true;
}

// ============================
// CALLBACK REGISTRATION
// ============================
void MPFQ::RegisterCallbacks(Callback_t* readData, Callback_t* writeCB, 
                            Callback_t* readCB, Callback_t* cmdCB) {
    ReturnReadData = readData;
    WriteResp = writeCB;
    ReadResp = readCB;
    CmdResp = cmdCB;
}

// ============================
// STATUS QUERY FUNCTIONS
// ============================
uint8_t MPFQ::get_occ(uint32_t chl) const {
    if (EM_ENABLE && EM_MODE == 0) chl = 0;  
    return mptc_->getPTC(chl)->occ;
}

uint8_t MPFQ::get_bandwidth(uint32_t chl) const {
    if (EM_ENABLE && EM_MODE == 0) chl = 0;  
    return uint8_t(mptc_->getPTC(chl)->availability);
}

void MPFQ::update() {
    
}