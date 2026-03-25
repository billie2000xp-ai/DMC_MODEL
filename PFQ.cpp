#include "PFQ.h"
#include "PTC.h"
#include "MPFQ.h"
#include "MemorySystemTop.h"
#include <assert.h>
#include <iomanip>
using namespace DRAMSim;

#define PROTECT_SUB(a) a = (a > 0) ? (a - 1) : 0;

// ===============================
// CONSTRUCTOR AND INITIALIZATION
// ===============================

PFQ::PFQ(unsigned index, MemorySystemTop* top, MPFQ* mpfq, ofstream &DDRSim_log_, ofstream &trace_log_) :
    id_(index),
    memorySystemTop_(top),
    mpfq_(mpfq),
    DDRSim_log(DDRSim_log_),
    trace_log(trace_log_) {
    
    // ============ LOG INITIALIZATION ============
    if (DEBUG_PERF) {
        perf_log.open("./perf.log");
        if (!perf_log.is_open()) {
            ERROR("Open perf.log error!!!!");
            exit(0);
        }
    }
    
    // ============ CHANNEL ID INITIALIZATION ============
    channel = 0;
    channel_ohot = 1ull << channel;

    // ============ LRU_MATRIX INITIALIZATION ============
    subq_lru_matrix.resize(PERF_SUBQ_NUM, vector<uint32_t>(PERF_SUBQ_NUM, 0));
    for (size_t i = 0; i < PERF_SUBQ_NUM; i ++) {
        for (size_t j = 0; j < PERF_SUBQ_NUM; j ++) {
            if (i < j) subq_lru_matrix[i][j] = 1;
            else subq_lru_matrix[i][j] = 0;
        }

    }

    // cmd1_lru_matrix.resize((PERF_DEPTH/PERF_SUBQ_NUM), vector<uint32_t>((PERF_DEPTH/PERF_SUBQ_NUM), 0));
    // for (size_t i = 0; i < (PERF_DEPTH/PERF_SUBQ_NUM); i ++) {
    //     for (size_t j = 0; j < (PERF_DEPTH/PERF_SUBQ_NUM); j ++) {
    //         if (i < j) cmd1_lru_matrix[i][j] = 1;
    //         else cmd1_lru_matrix[i][j] = 0;
    //     }
    // }

    // 前提PERF_DEPTH 能被 PERF_SUBQ_NUM 整除
    const size_t subq_size = PERF_DEPTH / PERF_SUBQ_NUM;

    // 调整 cmd1_lru_matrix 为三维：第一维子队列，后两维矩阵
    cmd1_lru_matrix.resize(PERF_SUBQ_NUM);
    for (size_t sq = 0; sq < PERF_SUBQ_NUM; ++sq) {
        cmd1_lru_matrix[sq].resize(subq_size, std::vector<uint32_t>(subq_size, 0));
        for (size_t i = 0; i < subq_size; ++i) {
            for (size_t j = 0; j < subq_size; ++j) {
                cmd1_lru_matrix[sq][i][j] = (i < j) ? 1 : 0;
            }
        }
    }

    cmd2_lru_matrix.resize(PERF_SUBQ_NUM, vector<uint32_t>(PERF_SUBQ_NUM, 0));
    for (size_t i = 0; i < PERF_SUBQ_NUM; i ++) {
        for (size_t j = 0; j < PERF_SUBQ_NUM; j ++) {
            if (i < j) cmd2_lru_matrix[i][j] = 1;
            else cmd2_lru_matrix[i][j] = 0;
        }
    }
    
    // ============ QUEUE INITIALIZATION ============
    PerfQue.resize(PERF_DEPTH);    
    ArbCmd.resize(PERF_SUBQ_NUM);
    
    // ============ BANK-RELATED DATA STRUCTURE INITIALIZATION ============
    // Bank count initialization
    wb_bank_cnt.resize(NUM_RANKS);
    rb_bank_cnt.resize(NUM_RANKS);
    wrel_bank_cnt.resize(NUM_RANKS);
    rrel_bank_cnt.resize(NUM_RANKS);
    
    for (auto& wb_bank : wb_bank_cnt) wb_bank.resize(NUM_BANKS);
    for (auto& rb_bank : rb_bank_cnt) rb_bank.resize(NUM_BANKS);
    for (auto& wrel_bank : wrel_bank_cnt) wrel_bank.resize(NUM_BANKS);
    for (auto& rrel_bank : rrel_bank_cnt) rrel_bank.resize(NUM_BANKS);
    
    // Bank group count initialization
    rb_bg_cnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    wb_bg_cnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    rrel_bg_cnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    wrel_bg_cnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    
    // Performance count initialization
    perf_bg_rcnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    perf_bg_wcnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    perf2ptc_bg_rcnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    perf2ptc_bg_wcnt.resize(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0));
    
    // ============ RANK-RELATED DATA STRUCTURE INITIALIZATION ============
    // Transaction interleaving initialization
    trans_baintlv.resize(NUM_RANKS, vector<unsigned>(2, 0));
    
    // Rank count initialization
    wb_rank_cnt.resize(NUM_RANKS);
    rb_rank_cnt.resize(NUM_RANKS);
    wrel_rank_cnt.resize(NUM_RANKS);
    rrel_rank_cnt.resize(NUM_RANKS);
    rel_rank_cnt.resize(NUM_RANKS);
    rank_cnt.resize(NUM_RANKS);
    
    // Initialize rank counts
    for (size_t i = 0; i < NUM_RANKS; i++) {
        wb_rank_cnt[i] = 0;
        rb_rank_cnt[i] = 0;
        wrel_rank_cnt[i] = 0;
        rrel_rank_cnt[i] = 0;
        rel_rank_cnt[i] = 0;
        rank_cnt[i] = 0;
    }
    
    // ============ QoS-RELATED DATA STRUCTURE INITIALIZATION ============
    // QoS count initialization
    wb_qos_cnt.resize(QOS_MAX + 1);
    rb_qos_cnt.resize(QOS_MAX + 1);
    for (size_t i = 0; i < QOS_MAX + 1; i++) {
        wb_qos_cnt[i] = 0;
        rb_qos_cnt[i] = 0;
    }
    
    // High QoS management initialization
    que_read_highqos_cnt.resize(NUM_RANKS);
    rank_cmd_high_qos.resize(NUM_RANKS);
    que_read_highqos_vld_cnt.resize(NUM_RANKS);
    perf_has_hqos.resize(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i++) {
        que_read_highqos_cnt[i] = 0;
        rank_cmd_high_qos[i] = false;
        que_read_highqos_vld_cnt[i] = 0;
        perf_has_hqos[i] = false;
    }
    
    // ============ STATE MACHINE INITIALIZATION ============  
    // Write buffer state initialization
    if (PERF_RWGRP_MODE == 0) {
        wbuff_state = WBUFF_IDLE;
        wbuff_state_pre = WBUFF_IDLE;
    } else if (PERF_RWGRP_MODE == 1) {
        wbuff_state = WBUFF_NO_GROUP;
        wbuff_state_pre = WBUFF_NO_GROUP;
        wbuff_rnkgrp_state = WBUFF_NO_RNK_GROUP;
    }
    
    // ============ WSRAM-RELATED INITIALIZATION ============ 
    // WSRAM slot initialization
    wsram_slt.clear();
    wsram_slt.resize(WSRAM_QUEUE_DEPTH);
    
    // Size bitmap initialization
    msize_bmp.clear();
    msize_bmp.resize(WSRAM_QUEUE_DEPTH / 2, true);
    
    msize_single.clear();
    msize_single.resize(WSRAM_QUEUE_DEPTH / 4, true);
    
    lsize_bmp.clear();
    lsize_bmp.resize(WSRAM_QUEUE_DEPTH / 4, true);

    WdataToSend.clear();
    
    // ============ PERF COUNT VECTOR INITIALIZATION ============   
    // Perf to PTC count initialization
    perf2ptc_bank_rcnt.clear();
    perf2ptc_bank_wcnt.clear();
    perf_bank_rcnt.clear();
    perf_bank_wcnt.clear();
    rb_highqos_bank_cnt.clear();
    perf_has_highqos_cmd_rowhit.clear();
    
    // Pre-allocate space
    perf2ptc_bank_rcnt.reserve(NUM_RANKS * NUM_BANKS);
    perf2ptc_bank_wcnt.reserve(NUM_RANKS * NUM_BANKS);
    perf_bank_rcnt.reserve(NUM_RANKS * NUM_BANKS);
    perf_bank_wcnt.reserve(NUM_RANKS * NUM_BANKS);
    rb_highqos_bank_cnt.reserve(NUM_RANKS * NUM_BANKS);
    perf_has_highqos_cmd_rowhit.reserve(NUM_RANKS * NUM_BANKS);
    
    // Fill default values
    for (size_t i = 0; i < NUM_RANKS; i++) {
        for (size_t j = 0; j < NUM_BANKS; j++) {
            perf2ptc_bank_rcnt.push_back(0);
            perf2ptc_bank_wcnt.push_back(0);
            perf_bank_rcnt.push_back(0);
            perf_bank_wcnt.push_back(0);
            rb_highqos_bank_cnt.push_back(0);
            perf_has_highqos_cmd_rowhit.push_back(false);
        }
    }
    
    // ============ SCHEDULING-RELATED STATE INITIALIZATION ============   
    perf_sch_rrank = 0;            
    perf_sch_wrank = 0;            
    perf_rd_in_rnkgrp = false;      
    perf_wr_in_rnkgrp = false;      
    
    // ============ COUNTER INITIALIZATION ============   
    ws_bp_cnt = 0;                  
    perf_bp_cnt = 0;                
    no_cmd_sch_cnt = 0;             
    no_rcmd_sch_cnt = 0;            
    no_cmd_sch_th = 0;              
    used_wdata_que_cnt = 0;         
    wbuff_state_gap = 0;            
    perf_highqos_trig_grpsw_cnt = 0;
    ser_write_cnt = 0;              
    ser_read_cnt = 0;               
    serial_cmd_cnt = 0;             
    wcmd_cnt = 0;                   
    rcmd_cnt = 0;                   
    max_rank = 0;                   
    read_cnt = 0;                   
    write_cnt = 0;                  
    forward_cnt = 0;                
    merge_cnt = 0;                  
    push_cnt = 0;                   
    same_bank_cnt = 0;              
    ptc_empty_cnt = 0;              
    
    // ============ OTHER PARAMETER INITIALIZATION ============    
    bytes_per_col = JEDEC_DATA_BUS_BITS / 8;  
    lastselcmd_subq_idx = static_cast<size_t>(-1);
    lastcmd_seltime = 0;          
    
    // ============ PATH AND LOG ============   
    // log_path = linked_ptcs_[0]->log_path;           
    
    // ============ STATE VECTOR INITIALIZATION ============   
    rcmd_bank_state.resize(NUM_RANKS * NUM_BANKS);
    
    // ============ TIME PARAMETER INITIALIZATION ============  
    m_pre_req_time = 0xFFFFFFFFFFFFFFFF;    
    m_pre_data_time = 0xFFFFFFFFFFFFFFFF;   
    perf_pre_req_time = 0xFFFFFFFFFFFFFFFF; 
    perf_pre_data_time = 0xFFFFFFFFFFFFFFFF;
    
    // ============ PERFORMANCE SIGNAL INITIALIZATION ============  
    perf_cmd_vld = false;                       
    perf_addrconf_cnt = 0;                      
    
    // ============ TIMEOUT FLAG INITIALIZATION ============   
    has_rd_tout = false;        
    has_wr_tout = false;        
    has_rd_dummy_tout = false;  
    has_wr_dummy_tout = false;  
    has_hqos_dummy_tout = false;
    hqos_rank = 0xFFFFFFFF;     
    
    // ============ SCHEDULE LEVEL COUNT INITIALIZATION ============
    sch_level_cnt.clear();
    sch_level_cnt.resize(7);
    for (size_t i = 0; i < 7; i++) {
        sch_level_cnt[i] = 0;
    }
    
    // ============ CONFIGURATION MAP INITIALIZATION ============   
    // Level configuration initialization
    wr_level.clear(); wr_level.resize(4);
    wr_most_level.clear(); wr_most_level.resize(4);
    rd_level.clear(); rd_level.resize(4);
    grp_mode.clear(); grp_mode.resize(4);
    
    for (size_t i = 0; i < 4; i++) {
        wr_level[i].clear(); wr_level[i].resize(5);
        wr_most_level[i].clear(); wr_most_level[i].resize(5);
        rd_level[i].clear(); rd_level[i].resize(5);
        grp_mode[i].clear(); grp_mode[i].resize(5);
        
        for (size_t j = 0; j < 5; j++) {
            wr_level[i].push_back(NULL);
            wr_most_level[i].push_back(NULL);
            rd_level[i].push_back(NULL);
            grp_mode[i].push_back(NULL);
        }
    }
    
    // Load level configuration from MAP_CONFIG
    for (size_t i = 0; i < 5; i++) {
        wr_level[0][i] = &MAP_CONFIG["WR_LEVEL0"][i];
        wr_level[1][i] = &MAP_CONFIG["WR_LEVEL1"][i];
        wr_level[2][i] = &MAP_CONFIG["WR_LEVEL2"][i];
        wr_level[3][i] = &MAP_CONFIG["WR_LEVEL3"][i];
        
        wr_most_level[0][i] = &MAP_CONFIG["WR_MOST_LEVEL0"][i];
        wr_most_level[1][i] = &MAP_CONFIG["WR_MOST_LEVEL1"][i];
        wr_most_level[2][i] = &MAP_CONFIG["WR_MOST_LEVEL2"][i];
        wr_most_level[3][i] = &MAP_CONFIG["WR_MOST_LEVEL3"][i];
        
        rd_level[0][i] = &MAP_CONFIG["RD_LEVEL0"][i];
        rd_level[1][i] = &MAP_CONFIG["RD_LEVEL1"][i];
        rd_level[2][i] = &MAP_CONFIG["RD_LEVEL2"][i];
        rd_level[3][i] = &MAP_CONFIG["RD_LEVEL3"][i];
        
        grp_mode[0][i] = &MAP_CONFIG["GRP_MODE_LEVEL0"][i];
        grp_mode[1][i] = &MAP_CONFIG["GRP_MODE_LEVEL1"][i];
        grp_mode[2][i] = &MAP_CONFIG["GRP_MODE_LEVEL2"][i];
        grp_mode[3][i] = &MAP_CONFIG["GRP_MODE_LEVEL3"][i];
    }

    // m_bank_rowhit_infos.resize(NUM_BANKS);
    
    // Previous N command history initialization
    pre_n_cmd_each_rank.resize(NUM_RANKS, list<arb_cmd>());

    pre_wresp_time = 0xFFFFFFFFFFFFFFFF;
    pre_rresp_time = 0xFFFFFFFFFFFFFFFF;

    rcmd_bp_byrp = false;
    rmw_bp_rdata_path = false;

    // for sid group
    sid_group_state.reserve(SID_GRP_PIPE);
    for (size_t i = 0; i < SID_GRP_PIPE; i ++) sid_group_state.push_back(NO_LR_GROUP);
    quc_slt_grp_lr.reserve(SID_GRP_PIPE);
    pre_quc_slt_grp_lr.reserve(SID_GRP_PIPE);
    for (size_t i = 0; i < SID_GRP_PIPE; i ++) {
        quc_slt_grp_lr.push_back(4);
        pre_quc_slt_grp_lr.push_back(4);
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        serial_sid_cnt.push_back(0);
        sidgrp_ch_cmd_cnt.push_back(0);
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        sid_timeout.push_back(vector<unsigned>());
        sid_issue_state.push_back(vector<bool>());
        for (size_t j = 0; j < NUM_SIDS; j ++) {
            sid_timeout[i].push_back(0);
            sid_issue_state[i].push_back(false);
        }
    }
    in_sid_group = 4;

    // Constructor initialization completed
}

PFQ::~PFQ() {
}

void PFQ::addAssociatedPTC(PTC* ptc) {
    if (linked_ptcs_.size() < (NUM_CHANS/NUM_PFQS)) {
        linked_ptcs_.push_back(ptc);
        
        ptc->setAssociatedPFQ(this);
    }
}

unsigned PFQ::ptcID(unsigned channel) const {
    unsigned ret = channel-getID()*(NUM_CHANS/NUM_PFQS);
    return ret;
}

PTC* PFQ::getPTC(unsigned channel) const {
    if (ptcID(channel) < linked_ptcs_.size()) {
        return linked_ptcs_[ptcID(channel)];
    }
    return nullptr;
}

/*==================================================================================================
Descriptor: ADD TRANSACTION FIELD
===================================================================================================*/

/***************************************************************************************************
Descriptor: main addTransaction
****************************************************************************************************/
bool PFQ::addTransaction(Transaction* trans) {
    if (WSRAM_MAP_EN) {
        bool slot_idle = false;
        bool wsram_ready = (trans->transactionType == DATA_WRITE) && (trans->pre_act || enoughWSRAM(trans,trans->burst_length));
        for (unsigned i = 0; i <= (WSRAM_QUEUE_DEPTH - 1); i += 4) {
            if (wsram_slt[i].state == SLT_IDLE && wsram_slt[i+1].state == SLT_IDLE
                && wsram_slt[i+2].state == SLT_IDLE && wsram_slt[i+3].state == SLT_IDLE) slot_idle = true;
        }
        if ((!wsram_ready) && trans->transactionType == DATA_WRITE) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- WSRAM_FULL_BP :: task="<<trans->task<<" type="<<trans->transactionType
                        <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                        <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt
                        <<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<endl);
            }
            ws_bp_cnt++;
            return false;
        }
        if ((!slot_idle) && trans->transactionType == DATA_WRITE) {
            if (DEBUG_BUS) { 
                PRINTN(setw(10)<<now()<<" -- CAN'T ACCEPT 128B BP :: task="<<trans->task<<" type="<<trans->transactionType
                        <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                        <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt
                        <<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<endl);
            }
            return false;
        }
    }
    
    if (perf_full()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF_FULL_DROP :: task="<<trans->task<<" type="<<trans->transactionType
                    <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt
                    <<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<endl);
        }
        perf_bp_cnt++;
        return false;
    }

    trans->timeAdded = now();
    if (!curClkCanAddTrans()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- CLK1X_DROP :: task="<<trans->task<<" type="<<trans->transactionType
                    <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt
                    <<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<endl);
        }
        return false;
    }

    setPerfTimeout(trans);
    genCresp(trans);

    if (trans->transactionType == DATA_READ) {
        // mpfq_->addFastRead(trans, getID());
        if (FORWARD_ENABLE && trans->data_size == 64 && read_forward(trans)) return true;
        if (RQ_ADCONF_PUSH_EN) rcmd_push_wcmd(trans);
        rcmd_set_conflict(trans);
        trans->arb_time = now() + tPERF2PTC;

        read_cnt ++;
        rcmd_cnt ++;
        if (PerfQue_pushCmd(trans)) {
            perf_bank_rcnt[trans->bankIndex] ++;
            perf_bg_rcnt[trans->rank][trans->group] ++;
            rb_bank_cnt[trans->rank][trans->bankIndex % NUM_BANKS] ++;
            rb_bg_cnt[trans->rank][trans->group] ++;
            rb_rank_cnt[trans->rank] ++;
            rb_qos_cnt[trans->qos] ++;
            if (trans->qos >= PERF_SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[trans->rank] ++;
                rb_highqos_bank_cnt[trans->bankIndex] ++;
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF_ADD :: [R]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                        <<trans->address<<dec<<" task="<<trans->task<<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                        <<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col
                        <<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded<<" timeout_th="<<trans->timeout_th
                        <<" inject_time="<<trans->inject_time<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<endl);
            }
        } else {
            ERROR(setw(10)<<now()<<" -- CAN'T push cmd to PerfQue !");
        }
    } else {
        if (MERGE_ENABLE && trans->data_size == 64 && write_merge(trans)) return true;
        wcmd_set_conflict(trans);
        write_cnt ++;
        wcmd_cnt ++;
        trans->arb_time = now() + tPERF2PTC;
        if (WSRAM_MAP_EN) {
            unsigned need_size = wdataQueNeedSize(trans,trans->data_size);
            bool success = findFreeWsram(trans,need_size);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- WSRAM :: before success="<<success <<" need_size="<<need_size<<" wsram_idx="<<trans->wsram_idx<<" need_size="<<need_size<<" used_wdata_que_cnt="<<used_wdata_que_cnt<<endl);
            }
            assert(success && "Has No Enough WSRAM !!\n");
            used_wdata_que_cnt += need_size;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- WSRAM :: after success="<<success <<" need_size="<<need_size<<" wsram_idx="<<trans->wsram_idx<<" need_size="<<need_size<<" used_wdata_que_cnt="<<used_wdata_que_cnt<<endl);
            }
            if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- WSRAM :: msize_bmp=""\n");
                    for (unsigned i=0;i<msize_bmp.size();i++) {
                        if (i!=0 && i%4 == 0) {
                            PRINTN("    ");
                        }
                        PRINTN(msize_bmp[i]<<" ");
                    }
                    PRINTN(endl);
            }
            if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- WSRAM :: msize_single=""\n");
                    for (unsigned i=0;i<msize_single.size();i++) {
                        if (i!=0 && i%4 == 0) {
                            PRINTN("    ");
                        }
                        PRINTN(msize_single[i]<<" ");
                    }
                    PRINTN(endl);
            }
            for (unsigned i = 0; i < need_size; i++) {
                unsigned loc = trans->wsram_idx + i;
                assert(wsram_slt[loc].state == SLT_IDLE);
                wsram_slt[loc].state = PENDING;
                wsram_slt[loc].channel = trans->channel;
                wsram_slt[loc].task = trans->task;
            }
            
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- WSRAM :: OCCUPY, TRAN ACCEPT task="<<trans->task <<" data_size="<<trans->data_size
                      <<" occupy_size="<<wdataQueNeedSize(trans,trans->data_size)<<" wsram_idx="<<trans->wsram_idx
                       <<" used_size_after_occupy="<<used_wdata_que_cnt<<endl);
                PRINTN(setw(10)<<now()<<" -- WSRAM :: ");
                for (unsigned i = 0; i < wsram_slt.size(); i++) {
                    if (i!=0 && i%4 == 0) {
                        PRINTN("    ");
                    }
                    PRINTN(wsram_slt[i].state<<" ");
                }
                PRINTN(endl);
            }
        }
        
        if (PerfQue_pushCmd(trans)) {
            perf_bank_wcnt[trans->bankIndex] ++;
            perf_bg_wcnt[trans->rank][trans->group] ++;
            wb_rank_cnt[trans->rank] ++;
            wb_qos_cnt[trans->qos] ++;
            wb_bank_cnt[trans->rank][trans->bankIndex % NUM_BANKS] ++;
            wb_bg_cnt[trans->rank][trans->group] ++;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF_ADD :: [W]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                        <<trans->address<<dec<<" task="<<trans->task<<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                        <<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col
                        <<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded<<" timeout_th="<<trans->timeout_th
                        <<" wsram_idx="<<trans->wsram_idx<<" channels="<<trans->channel<<" fast_rd="<<trans->fast_rd
                        <<" inject_time="<<trans->inject_time<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<endl);
            }
        } else {
            ERROR(setw(10)<<now()<<" -- CAN'T push cmd to PerfQue !");
        }
    }
    rank_cnt[trans->rank] ++;
    perf_pre_req_time = now();

    // addFastRd
    linked_ptcs_[ptcID(trans->channel)]->addTransaction(trans, true);
    
    return true;
}

/***************************************************************************************************
Descriptor: judge enough slt in wsram&
****************************************************************************************************/
unsigned PFQ::wdataQueNeedSize(Transaction* trans, unsigned data_size) {
    if (trans->pre_act || WSRAM_QUEUE_DEPTH == 0) {
        return 0;
    }
    unsigned need_wdata_que = 0;
    static const std::set<unsigned> valid_sizes = {32,64,96,128,256,512};
    if (valid_sizes.find(data_size) == valid_sizes.end()) {
        ERROR("forbiddon data_size = "<<data_size);
        assert(0);
        return 0;
    } else {
        need_wdata_que = ceil(1.0 * data_size / WSRAM_QUEUE_WIDTH);
    }
    return need_wdata_que;
}

bool PFQ::findFreeWsram(Transaction* trans, unsigned need_size) {
    if (need_size == 0 || WSRAM_QUEUE_DEPTH == 0) {
        return true;
    }
    if (need_size == 2 || need_size == 1) {
        bool find_single = false;
        for (unsigned i = 0; i <= (WSRAM_QUEUE_DEPTH - 1); i += 2) {
            msize_bmp[i/2] = wsram_slt[i].state == SLT_IDLE || wsram_slt[i+1].state == SLT_IDLE;
        }
        for (unsigned j = 0; j <= (WSRAM_QUEUE_DEPTH/2 - 1); j += 2) {
            msize_single[j/2] = (msize_bmp[j] != msize_bmp[j+1]);
        }
        for (unsigned k = 0; k <= msize_single.size() - 1; k++) {
            if (msize_single[k] == 1) {
                if (msize_bmp[2*k] == 1 && msize_bmp[2*k + 1] == 0) {
                    if (wsram_slt[4*k].state == SLT_IDLE && wsram_slt[4*k+1].state == SLT_IDLE) {
                        trans->wsram_idx = k*4;
                        find_single = true;
                        return true;
                    }
                    
                } else if (msize_bmp[2*k] == 0 && msize_bmp[2*k + 1] == 1) {
                    if (wsram_slt[(2*k+1)*2].state == SLT_IDLE && wsram_slt[(2*k+1)*2+1].state == SLT_IDLE) {
                        trans->wsram_idx = (2*k+1)*2; 
                        find_single = true;
                        return true;
                    }                   
                }
            }
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WSRAM :: msize_bmp=""\n");
            for (unsigned i = 0; i < msize_bmp.size(); i++) {
                if (i !=0 && i%4 == 0) {
                    PRINTN("    ");
                }
                PRINTN(msize_bmp[i]<<" ");
            }
            PRINTN(endl);
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WSRAM :: msize_single=""\n");
            for (unsigned i=0;i<msize_single.size();i++) {
                if (i != 0 && i%4 == 0) {
                    PRINTN("    ");
                }
                PRINTN(msize_single[i]<<" ");
            }
            PRINTN(endl);
        }
        if (!find_single) {
            for (unsigned loc = 0; loc <= (WSRAM_QUEUE_DEPTH - 1); loc += 2) {
                bool find_loc = true;
                for (unsigned i = 0; i < 2; i++) {
                    if (wsram_slt[loc+i].state != SLT_IDLE) {
                        find_loc = false;
                        break;
                    }
                }
                if (find_loc) {
                    trans->wsram_idx = loc;
                    return true;
                }
            } 
        }      
    } else if (need_size == 4 || need_size == 3) {
        for (unsigned loc = 0; loc <= (WSRAM_QUEUE_DEPTH - 1); loc += 4) {
            bool find_loc = true;
            for (unsigned i = 0; i < 4; i++) {
                if (wsram_slt[loc+i].state != SLT_IDLE) {
                    find_loc = false;
                    break;
                }
            }
            if (find_loc) {
                trans->wsram_idx = loc;
                return true;
            }
        }
    } else {
        assert(0 && "ERROR Data size!!");
    }
    return false;
}

bool PFQ::enoughWSRAM(Transaction* trans, uint32_t burst_length) {
    // burst_length = 7;//max burst length(256B)
    bool ret;
    Transaction tmp_trans;
    tmp_trans.task = static_cast<uint64_t>(-1);
    tmp_trans.data_size = (burst_length + 1) * DMC_DATA_BUS_BITS / 8;
    unsigned need_wdata_que = wdataQueNeedSize(trans,tmp_trans.data_size);
    if (need_wdata_que + used_wdata_que_cnt > WSRAM_QUEUE_DEPTH) {
        ret = false;
    } else {
        ret = findFreeWsram(trans,need_wdata_que);
    }
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- WSRAM :: enough func need_wdata_que="<<need_wdata_que<<" used_wdata_que_cnt="<<used_wdata_que_cnt
                <<" WSRAM_QUEUE_DEPTH="<<WSRAM_QUEUE_DEPTH<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<"\n");
        PRINTN(setw(10)<<now()<<" -- WSRAM :: enough= "<<ret<<"\n");
        for (unsigned i = 0; i < wsram_slt.size(); i++) {
            if (i != 0 && i%4 == 0) {
                PRINTN("    ");
            }
            PRINTN(wsram_slt[i].state<<" ");
        }
        PRINTN(endl);
    }
    return ret;
}

bool PFQ::perf_full() {
    return (rcmd_cnt + wcmd_cnt + rel_rcmd_cnt + rel_wcmd_cnt) >= PERF_DEPTH;
}

bool PFQ::curClkCanAddTrans() {
    unsigned add_time = (now()/CLKH2CLKL+1)*CLKH2CLKL -1;//clk syn
    if (m_pre_req_time == add_time) {
        return false;
    } else {
        m_pre_req_time = add_time;
        return true;
    }
}

void PFQ::setPerfTimeout(Transaction* trans) {
    string mpam_timeout, mpam_adapt;
    unsigned timeout = 0, pri_adapt = 0;
    trans->perf_qos = trans->qos;
    trans->perf_pri = trans->pri;
    trans->improve_cnt = 0;
    trans->adj_cmd_cnt = 0;
    if (trans->qos < QOS_MAX) { // qos mapping timeout
        if (trans->transactionType == DATA_READ) {
            mpam_timeout = "PERF_TIMEOUT_PRI_RD";
            mpam_adapt = "PERF_ADAPT_PRI_RD";
        } else {
            mpam_timeout = "PERF_TIMEOUT_PRI_WR";
            mpam_adapt = "PERF_ADAPT_PRI_WR";
        }
        if (PERF_TIMEOUT_EN) timeout = MAP_CONFIG[mpam_timeout][trans->qos];
        if (PERF_PRI_ADAPT_ENABLE) pri_adapt = MAP_CONFIG[mpam_adapt][trans->qos];
    }
    if (timeout == 0) {
        trans->timeout_th = 0;
    } else {
        trans->timeout_th = timeout;
    }
    trans->pri_adapt_th = pri_adapt;
}

void PFQ::genCresp(Transaction* trans) {
    //lp6: full write and merge read under fast command mode
    if (((trans->transactionType == DATA_WRITE && !trans->mask_wcmd) || (trans->transactionType == DATA_READ && trans->mask_wcmd))
            && RMW_CMD_MODE == 0 && IS_LP6) {
        resp cmdresp;
        cmdresp.channel = trans->channel;
        cmdresp.task = trans->task;
        cmdResp.push_back(cmdresp);
    }

    //lp5: full write and mask write
    if ((trans->transactionType == DATA_WRITE) && IS_LP5) {
        resp cmdresp;
        cmdresp.channel = trans->channel;
        cmdresp.task = trans->task;
        cmdResp.push_back(cmdresp);
    }
}

bool PFQ::read_forward(Transaction* trans) {
    for (auto& w : PerfQue) {
        if (w == nullptr) continue;
        if (w->transactionType != DATA_WRITE) continue;
        if (w->in_ptc) continue;
        if (trans->rank != w->rank) continue;
        if (trans->bankIndex != w->bankIndex) continue;
        if (trans->row != w->row) continue;
        if (w->data_ready_cnt <= w->burst_length) continue;
        if (trans->col >= w->col && (trans->col + 64/bytes_per_col) <= (w->col + 64/bytes_per_col)) {
            for (size_t i = 0; i <= trans->burst_length; i ++) {
                unsigned cnt = i % 2 + 1;
                linked_ptcs_[ptcID(trans->channel)]->gen_rdata(trans->task, cnt, 0, trans->mask_wcmd);
            }
            if (trans->data_size == 64) forward_cnt ++;
            gen_wresp(trans->task, trans->channel);

            if (FASTWAKEUP_EN && !PREDICT_FASTWAKEUP) {
                if (linked_ptcs_[ptcID(trans->channel)]->fast_wakeup_cnt[trans->rank] == 0 && now() >= 100) {
                    ERROR(setw(10)<<now()<<" -- GBUF["<<channel<<"] Error fast wakeup count!");
                    assert(0);
                }
                linked_ptcs_[ptcID(trans->channel)]->fast_wakeup_cnt[trans->rank] -= 1;
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- GBUF_FORWARD :: task="<<trans->task<<hex<<" address="
                        <<trans->address<<dec<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<")"<<endl);
            }
            return true;
        }
    }
    return false;
}

void PFQ::rcmd_push_wcmd(Transaction* t) {
    for (auto& w : PerfQue) {
        if (w == nullptr) continue;
        if (w->in_ptc) continue;
        if (w->transactionType == DATA_READ) continue;
        if (t->bankIndex == w->bankIndex && t->row == w->row 
                && (t->addr_col < (w->addr_col + w->data_size))
                && ((t->addr_col + t->data_size) > w->addr_col)) {
            push_cnt ++;
            if (GrpMode.grp_mode6) { // priority is 1
                w->pri = 1;
            } else { // high priority
                if (!w->adtimeout) {                
                    wr_adtimeout_cnt ++;
                }
                w->adtimeout = true;
                w->pri = QOS_MAX - 1;
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD_PUSH :: task="<<t->task<<hex<<" address="<<t->address<<dec
                        <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" w_task="<<w->task
                        <<" w_adtimeout="<<w->adtimeout<<" w_tout_cnt="<<wr_adtimeout_cnt<<endl);
            }
        }
    }
}

void PFQ::rcmd_set_conflict(Transaction* t) {
    conf_state *c = new conf_state;
    for (auto& w : PerfQue) {
        if (w == nullptr) continue;
        if (w->transactionType == DATA_READ) continue;
        if (t->group == w->group) c->perf.bg_conf_cnt ++;
        if (t->bankIndex == w->bankIndex) c->perf.ba_conf_cnt ++;
        if (t->bankIndex == w->bankIndex && t->row == w->row 
                && (t->addr_col < (w->addr_col + w->data_size))
                && ((t->addr_col + t->data_size) > w->addr_col)) {
            c->perf.ad_conf_cnt ++;
            perf_addrconf_cnt ++;
            t->perf_addrconf = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RCMD_SET_CONF :: task="<<t->task<<hex<<" address="<<t->address<<dec
                        <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" col="<<t->col
                        <<" g_bg_cc="<<c->perf.bg_conf_cnt<<" g_ba_cc="<<c->perf.ba_conf_cnt<<" g_ad_cc="<<c->perf.ad_conf_cnt
                        <<" w_task="<<w->task<<" rank="<<w->rank<<" bank="<<w->bankIndex<<" row="<<w->row<<" col="<<w->col<<endl);
            }
        }
    }
    c->conf_time = now() + tPERF_CONF;
    t->conflict_state = c;
}

void PFQ::wcmd_set_conflict(Transaction* t) {
    conf_state *c = new conf_state;
    for (auto& cmd : PerfQue) {
        if (cmd == nullptr) continue;
        if (t->group == cmd->group) c->perf.bg_conf_cnt ++;
        if (t->bankIndex == cmd->bankIndex) c->perf.ba_conf_cnt ++;
        if (t->bankIndex == cmd->bankIndex && t->row == cmd->row 
                && (t->addr_col < (cmd->addr_col + cmd->data_size))
                && ((t->addr_col + t->data_size) > cmd->addr_col)) {
            c->perf.ad_conf_cnt ++;
            perf_addrconf_cnt ++;
            t->perf_addrconf = true;
            if (DEBUG_BUS) {
                if (cmd->transactionType == DATA_READ) {
                    PRINTN(setw(10)<<now()<<" -- WCMD_SET_CONF_R :: task="<<t->task<<hex<<" address="<<t->address<<dec
                            <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" col="<<t->col
                            <<" g_bg_cc="<<c->perf.bg_conf_cnt<<" g_ba_cc="<<c->perf.ba_conf_cnt<<" g_ad_cc="<<c->perf.ad_conf_cnt
                            <<" r_task="<<cmd->task<<" rank="<<cmd->rank<<" bank="<<cmd->bankIndex<<" row="<<cmd->row<<" col="<<cmd->col<<endl);
                } else if (cmd->transactionType == DATA_WRITE) {
                    PRINTN(setw(10)<<now()<<" -- WCMD_SET_CONF_W :: task="<<t->task<<hex<<" address="<<t->address<<dec
                            <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" col="<<t->col
                            <<" g_bg_cc="<<c->perf.bg_conf_cnt<<" g_ba_cc="<<c->perf.ba_conf_cnt<<" g_ad_cc="<<c->perf.ad_conf_cnt
                            <<" w_task="<<cmd->task<<" rank="<<cmd->rank<<" bank="<<cmd->bankIndex<<" row="<<cmd->row<<" col="<<cmd->col<<endl);
                }
            }
        }
    }
    t->conflict_state = c;
}

/***************************************************************************************************
Descriptor: 2 modes for Cmd push into PerfQue
****************************************************************************************************/
// Lru_cbb
unsigned PFQ::lru_arb(uint64_t index1, uint64_t index2, uint64_t sel) {
    if (sel == 0) return subq_lru_matrix[index1][index2];
    // else if (sel == 1) return cmd1_lru_matrix[index1][index2];
    else if (sel == 1) {
        // 命令级 LRU，每个子队列独立
        const size_t L = PERF_DEPTH / PERF_SUBQ_NUM; // 每个子队列的命令数
        size_t subq1 = index1 / L;
        size_t subq2 = index2 / L;
        size_t local1 = index1 % L;
        size_t local2 = index2 % L;
        // 如果两个命令属于不同子队列，按原逻辑可能不应该比较，这里返回0（可根据需要调整）
        if (subq1 != subq2) {
            ERROR(setw(10)<<now()<<"Can not compare lru pri withing diff subq_index in Lv1 arb!");
            assert(0);
        }
        return cmd1_lru_matrix[subq1][local1][local2];
    }
    else if (sel == 2) return cmd2_lru_matrix[index1][index2];
    else return -1;
}

void PFQ::update_lru (uint64_t index, uint64_t sel, arb_cmd* cmd) {
    if(sel == 0){
        for(size_t i=0;i<PERF_SUBQ_NUM;i++){
                if(i==index) continue;
                subq_lru_matrix[i][index] = 1;
                subq_lru_matrix[index][i] = 0;
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LRU :: update subq lru matrix="<<sel<<", index="<<index<<", task="<<cmd->task<<endl);
        }
    }
    // if(sel == 1){
    //     for(size_t i=0;i<(PERF_DEPTH/PERF_SUBQ_NUM);i++){
    //             if(i==index) continue;
    //             cmd1_lru_matrix[i][index] = 1;
    //             cmd1_lru_matrix[index][i] = 0;
    //     }
    //     if (DEBUG_BUS) {
    //         PRINTN(setw(10)<<now()<<" -- LRU :: update cmd lv1 sel lru matrix="<<sel<<", index="<<index<<", task="<<cmd->task<<endl);
    //     }
    // }
    if (sel == 1) {
        // 根据全局 index 计算子队列 ID 和局部索引
        size_t subq = index / (PERF_DEPTH / PERF_SUBQ_NUM);
        size_t local_idx = index % (PERF_DEPTH / PERF_SUBQ_NUM);
        // 更新对应子队列的矩阵
        for (size_t i = 0; i < PERF_DEPTH / PERF_SUBQ_NUM; i++) {
            if (i == local_idx) continue;
            cmd1_lru_matrix[subq][i][local_idx] = 1;
            cmd1_lru_matrix[subq][local_idx][i] = 0;
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10) << now() << " -- LRU :: update cmd lv1 sel lru matrix=" << sel << ", subq=" << subq << ", local_idx=" << local_idx << ", task=" << cmd->task << endl);
        }
    }
    if(sel == 2){
        for(size_t i=0;i<PERF_SUBQ_NUM;i++){
                if(i==index) continue;
                cmd2_lru_matrix[i][index] = 1;
                cmd2_lru_matrix[index][i] = 0;
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LRU :: update cmd lv2 sel lru matrix="<<sel<<", index="<<index<<", task="<<cmd->task<<endl);
        }
    }
}

// Get the start and end indices of a SUBQ
std::pair<size_t,size_t> PFQ::getSubqRange(size_t subq_idx) {
    if (subq_idx >= PERF_SUBQ_NUM) {
        // ERROR()
        return {0, 0};
    }
    size_t subq_size = PERF_DEPTH / PERF_SUBQ_NUM;
    size_t start = subq_idx * subq_size;
    size_t end = start + subq_size - 1;
    return {start, end};
}

size_t PFQ::findTargetSubq(size_t bg, size_t bank) {
    switch (NUM_GROUPS) {
        case 4:
            switch (PERF_SUBQ_NUM) {
                case 4:
                    return bg & 0x3;
                case 8:
                    return ((bank & 0x1) << 2) | (bg & 0x3);
                case 16:
                    return ((bank & 0x3) << 2) | (bg & 0x3);
                default:
                    return bg & 0x3;
            }
        case 8:
            switch (PERF_SUBQ_NUM) {
                case 4:
                    return bg & 0x3;
                case 8:
                    return bg & 0x7;
                case 16:
                    return ((bank & 0x1) << 3) | (bg & 0x7);
                default:
                    return bg & 0x7;
            }
        default:
            return bg & 0x3;
    }
}

// check if a SUBQ has an empty slot
size_t PFQ::findEmptySlotInSubq(size_t subq_idx) {
    auto range = getSubqRange(subq_idx);
    size_t start = range.first;
    size_t end = range.second;
    for (size_t i = start; i <= end; i++) {
        if (PerfQue[i] == nullptr) {
            return i;
            break;
        }
    }
    return static_cast<size_t>(-1);
}

// find all SUBQs with empty slots
size_t PFQ::findBestSubqByLRU(size_t avoid_subq) {
    size_t best_subq = static_cast<size_t>(-1);
    for (size_t subq_idx = 0; subq_idx < PERF_SUBQ_NUM; subq_idx++) {
        if (subq_idx == avoid_subq) continue;
        // check if this SUBQ has an empty slot
        if (findEmptySlotInSubq(subq_idx) != static_cast<size_t>(-1)) {
            if (best_subq == static_cast<size_t>(-1)) {
                best_subq = subq_idx;
            } else {
                if (lru_arb(subq_idx, best_subq, 0)) best_subq = subq_idx;
            }
        }
    }
    return best_subq;
}

// push a Cmd into PerfQue
bool PFQ::PerfQue_pushCmd(Transaction* trans) {
    size_t target_subq = static_cast<size_t>(-1);
    size_t insert_pos = static_cast<size_t>(-1);
    if (SUBQ_SEL_MODE == 0) { //rename
        // mode 1: pri SUBQ with the smallest num that has empty slts
        for (size_t subq_idx = 0; subq_idx < PERF_SUBQ_NUM; subq_idx++) {
            insert_pos = findEmptySlotInSubq(subq_idx);
            if (insert_pos != static_cast<size_t>(-1)) {
                target_subq = subq_idx;
                break;
            }
        }
    } else if (SUBQ_SEL_MODE == 1) {
        // mode 2: first try to store in the co SUBQ based on BG
        size_t bg = trans->group;
        size_t bank = trans->bankIndex;
        target_subq = findTargetSubq(bg, bank);
        insert_pos = findEmptySlotInSubq(target_subq);
        // if the target SUBQ is full, use LRU to select another SUBQ
        if (insert_pos == static_cast<size_t>(-1)) {
            target_subq = findBestSubqByLRU(target_subq);
            if (target_subq != static_cast<size_t>(-1)) {
                insert_pos = findEmptySlotInSubq(target_subq);
            }
        }
    }
    // if an empty slot is found, store the cmd
    arb_cmd *cmd = new arb_cmd;
    cmd->creat(trans);
    if (insert_pos != static_cast<size_t>(-1)) {
        PerfQue[insert_pos] = trans;
        update_lru(target_subq, 0, cmd);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- CMD slot_index ="<<insert_pos<<", subq="<<target_subq<<", task="<<cmd->task<<endl);
        }
        return true;
    } else {
        return false;
    }

}

bool PFQ::write_merge(Transaction* trans) {
    for (auto& w : PerfQue) {
        if (w->in_ptc) continue;
        if (w->transactionType == DATA_READ) continue;
        if (w->data_size != 64) continue;
        if (trans->rank != w->rank) continue;
        if (trans->bankIndex != w->bankIndex) continue;
        if (w->data_ready_cnt <= w->burst_length) continue;
        if ((trans->address & ALIGNED_DATA_64B) == (w->address & ALIGNED_DATA_64B)
                && (trans->address & ALIGNED_NUMB_64B) != (w->address & ALIGNED_NUMB_64B)) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF_MERGE :: Send task="<<trans->task<<" address="<<hex<<trans->address
                        <<dec<<", Merge task="<<w->task<<" address="<<hex<<w->address<<dec<<endl);
            }
            merge_cnt ++;
            w->burst_length = trans->burst_length * 2 + 1;
            w->data_ready_cnt = w->burst_length + 1;
            w->data_size = 128;
            w->address = w->address & ~ALIGNED_NUMB_64B;
            w->col = w->col & trans->col;
            w->addr_col = w->addr_col & trans->addr_col;
            return true;
        }
    }
    return false;
}
bool PFQ::addData(uint32_t *data, uint64_t task) {
    (void)data;
    if (!curClkCanAddData() && PERF_CLK_MODE == 1) {
        if (DEBUG_BUS) {
             PRINTN(setw(10)<<now()<<" -- ADD DATA BP :: task="<<task<<endl);
        }
        return false;
    }

    bool wbuf_exist = false;
    for (auto& trans : PerfQue) {
        if (trans == nullptr) continue;
        if (task != trans->task) continue;
        if (trans->transactionType != DATA_WRITE) continue;  
        if (trans->data_ready_cnt >= (trans->burst_length + 1)) {
            ERROR(setw(10)<<now()<<"Too Much Data, task="<<trans->task<<", data_ready_cnt="<<trans->data_size);
            assert(0);
        }
        if (WSRAM_MAP_EN) {
            unsigned slot_idx = trans->wsram_idx + trans->data_ready_cnt;
            if (wsram_slt[slot_idx].state != PENDING) {
                ERROR(setw(10)<<now()<<"Invalid wsram state transition at slot"<<slot_idx<<", expected PENDING but got "
                        <<wsram_slt[slot_idx].state<<" for task="<<trans->task<<endl);
                assert(0);
            }
            if (wsram_slt[slot_idx].task != task) {
                ERROR(setw(10)<<now()<<"Invalid wdata task at slot"<<slot_idx<<", expected task="<<task<<"but got "
                        <<wsram_slt[slot_idx].task<<endl);
                assert(0);
            }
            wsram_slt[slot_idx].state = OCCUPIED;  
            if (IS_HBM3) {
                wsram_slt[slot_idx+1].state = OCCUPIED; 
            }
        }
        trans->data_ready_cnt ++;
        if (IS_HBM3) {
            trans->data_ready_cnt ++;
        }
        if (trans->in_ptc) {
            ERROR(setw(10)<<now()<<" Wcmd Already in PTC, Wdata Rejected, task="<<trans->task);
            assert(0);
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF_MATCH :: data_ready_cnt:"<<trans->data_ready_cnt
                    <<", "<<trans->data_size<<", task="<<trans->task<<endl);
        }
        if ((trans->transactionType == DATA_WRITE && trans->mask_wcmd && RMW_ENABLE_PERF && IS_LP6)
                && trans->data_ready_cnt == (trans->burst_length + 1)) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMW_WDATA_COLLECTED :: task="<<trans->task<<" type="<<trans->transactionType
                        <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                        <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt
                        <<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt<<")"<<" timeadded="<<trans->timeAdded<<endl);
            }
            trans->transactionType = DATA_READ;
            rcmd_set_conflict(trans);
            rcmd_cnt ++;
            perf_bank_rcnt[trans->bankIndex] ++;
            perf_bg_rcnt[trans->rank][trans->group] ++;
            rb_bank_cnt[trans->rank][trans->bankIndex % NUM_BANKS] ++;
            rb_bg_cnt[trans->rank][trans->group] ++;
            rb_rank_cnt[trans->rank] ++;
            rb_qos_cnt[trans->qos] ++;
            if (trans->qos >= PERF_SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[trans->rank] ++;
                rb_highqos_bank_cnt[trans->bankIndex] ++;
            }
            
            getPTC(trans->channel)->bankStates[trans->bankIndex].perf_bankrd_conflict = true;//modify
            getPTC(trans->channel)->bankStates[trans->bankIndex].samebank_rcnt ++;
            getPTC(trans->channel)->push_pending_TransactionQue(trans);
        
            wcmd_cnt --;
            wcmd_release_conflict(trans);
            perf_bank_wcnt[trans->bankIndex] --;
            perf_bg_wcnt[trans->rank][trans->group] --;
            wb_rank_cnt[trans->rank] --;
            wb_qos_cnt[trans->qos] --;
            wb_bank_cnt[trans->rank][trans->bankIndex % NUM_BANKS] --;
            wb_bg_cnt[trans->rank][trans->group] --;
            linked_ptcs_[ptcID(trans->channel)]->bankStates[trans->bankIndex].samebank_wcnt --;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMW_CMD W2R_conversion completed for task"<<trans->task<<endl);
            }
        } 
        wbuf_exist = true;
        break;
    }

    if (!wbuf_exist) {
        ERROR("Can't Find Task, task="<<task);
        assert(0);
    }
    perf_pre_data_time = now();
    return true;
}

bool PFQ::curClkCanAddData() {
    unsigned add_time = (now()/CLKH2CLKL+1) * CLKH2CLKL - 1; //clk syn
    if (add_time == m_pre_data_time) {
        return false;
    } else {
        m_pre_data_time = add_time;
        return true;
    }
}

void PFQ::rcmd_release_conflict(Transaction* trans) {
    for (auto& w : PerfQue) {
        if (w == nullptr) continue;
        if (w->in_ptc) continue;
        if (w->transactionType == DATA_READ) continue;
        if (w->conflict_state == nullptr) continue;
        if (trans->group == w->group) PROTECT_SUB(w->conflict_state->perf.bg_conf_cnt);
        if (trans->bankIndex == w->bankIndex) PROTECT_SUB(w->conflict_state->perf.ba_conf_cnt);
        if (trans->bankIndex == w->bankIndex && trans->row == w->row 
                && (trans->addr_col < (w->addr_col + w->data_size))
                && ((trans->addr_col + trans->data_size) > w->addr_col)) {
            PROTECT_SUB(w->conflict_state->perf.ad_conf_cnt);
            if (w->conflict_state->perf.ad_conf_cnt == 0) w->perf_addrconf = false;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RCMD_REL_CONF :: task="<<w->task<<hex<<" address="<<w->address<<dec
                        <<" rank="<<w->rank<<" bank="<<w->bankIndex<<" row="<<w->row
                        <<" g_bg_cc="<<w->conflict_state->perf.bg_conf_cnt<<" g_ba_cc="<<w->conflict_state->perf.ba_conf_cnt 
                        <<" g_ad_cc="<<w->conflict_state->perf.ad_conf_cnt<<" r_task="<<trans->task<<endl);
            }
        }
    }
}

void PFQ::wcmd_release_conflict(Transaction* trans) {
    for (auto& cmd : PerfQue) {
        if (cmd == nullptr) continue;
        if (cmd->in_ptc) continue;
        if (cmd->transactionType == DATA_WRITE && trans->task == cmd->task) continue;
        if (cmd->transactionType == DATA_READ && trans->task == cmd->task && trans->mask_wcmd) continue;
        if (cmd->conflict_state == nullptr) continue;
        if (trans->group == cmd->group) PROTECT_SUB(cmd->conflict_state->perf.bg_conf_cnt);
        if (trans->bankIndex == cmd->bankIndex) PROTECT_SUB(cmd->conflict_state->perf.ba_conf_cnt);
        if (trans->bankIndex == cmd->bankIndex && trans->row == cmd->row 
                && (trans->addr_col < (cmd->addr_col + cmd->data_size))
                && ((trans->addr_col + trans->data_size) > cmd->addr_col)) {
            PROTECT_SUB(cmd->conflict_state->perf.ad_conf_cnt);
            if (cmd->conflict_state->perf.ad_conf_cnt == 0) cmd->perf_addrconf = false;
            if (DEBUG_BUS) {
                if (cmd->transactionType == DATA_READ) {
                    PRINTN(setw(10)<<now()<<" -- WCMD_REL_CONF_R :: task="<<cmd->task<<hex<<" address="<<cmd->address<<dec
                            <<" rank="<<cmd->rank<<" bank="<<cmd->bankIndex<<" row="<<cmd->row
                            <<" g_bg_cc="<<cmd->conflict_state->perf.bg_conf_cnt<<" g_ba_cc="<<cmd->conflict_state->perf.ba_conf_cnt 
                            <<" g_ad_cc="<<cmd->conflict_state->perf.ad_conf_cnt<<" w_task="<<trans->task<<endl);
                } else if (cmd->transactionType == DATA_WRITE) {
                    PRINTN(setw(10)<<now()<<" -- WCMD_REL_CONF_W :: task="<<cmd->task<<hex<<" address="<<cmd->address<<dec
                            <<" rank="<<cmd->rank<<" bank="<<cmd->bankIndex<<" row="<<cmd->row
                            <<" g_bg_cc="<<cmd->conflict_state->perf.bg_conf_cnt<<" g_ba_cc="<<cmd->conflict_state->perf.ba_conf_cnt 
                            <<" g_ad_cc="<<cmd->conflict_state->perf.ad_conf_cnt<<" w_task="<<trans->task<<endl);
                }    
            }
        }
    }
}

void PFQ::dmc_release_conflict(Transaction *trans) {
    for (auto& cmd : PerfQue) {
        if (cmd == nullptr) continue;
        auto dmc_conf_state = cmd->conflict_state;
        if (dmc_conf_state == nullptr) continue;
        if (dmc_conf_state->dmc.bg_conf_cnt > 0 || dmc_conf_state->dmc.ba_conf_cnt >0 || dmc_conf_state->dmc.ad_conf_cnt > 0) {
            ERROR("PTC Conf Value Wrong, task="<<cmd->task<<", bankIndex="<<cmd->bankIndex<<", dmc_bgcc="<<dmc_conf_state->dmc.bg_conf_cnt
                    <<", dmc_bacc="<<dmc_conf_state->dmc.ba_conf_cnt<<", dmc_adcc="<<dmc_conf_state->dmc.ad_conf_cnt);
            assert(0);
        }

        if (cmd->in_ptc) continue;
        if (trans->group == cmd->group) PROTECT_SUB(dmc_conf_state->dmc.bg_conf_cnt);
        if (trans->bankIndex == cmd->bankIndex) PROTECT_SUB(dmc_conf_state->dmc.ba_conf_cnt);
        if (trans->bankIndex == cmd->bankIndex && trans->row == cmd->row && trans->col == cmd->col) {
            PROTECT_SUB(dmc_conf_state->dmc.ad_conf_cnt);
        }
    }
}

/*==================================================================================================
Descriptor: UPDATE FIELD
===================================================================================================*/

/***************************************************************************************************
Descriptor: main update
****************************************************************************************************/
void PFQ::update() {
    // resp
    updateWRresp();
    if (!clkSyn() && PERF_CLK_MODE==1) {
        return;
    }   
    updateCresp();
    update_state();
    update_state_trig();
    if (PERF_TIMEOUT_MODE == 1) {
        perf_check_timeout_and_aging();
    }
    sch_subque();
    arb_node();
    stateTransition();
    for (size_t rank = 0; rank < NUM_RANKS; rank++) {
        update_sidgroup_state(rank);
    }
    update_rnkgroup_state();
    update_rpsram_state();
    if (WSRAM_MAP_EN) {
        wsram_release_wdata();
    } else {
        send_wdata();
    }
}

void PFQ::updateWRresp() {
    if (!writeResp.empty()) {
        if (pre_wresp_time != now()) {
            if ((*mpfq_->WriteResp)(writeResp[0].channel, writeResp[0].task, 0, 0, 0)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Wresp Received :: task="<<writeResp[0].task<<endl);
                }
                pre_wresp_time = now();
                unsigned channel = writeResp[0].channel;
                writeResp.erase(writeResp.begin());
                linked_ptcs_[ptcID(channel)]->dresp_cnt--;
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Wresp Back Pressure :: task="<<writeResp[0].task<<endl);
                }
            }
        }
    }

    if (!readResp.empty()) {
        if (pre_rresp_time != now()) {
            if ((*mpfq_->ReadResp)(readResp[0].channel, readResp[0].task, 0, 0, 0)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rresp Received :: task="<<readResp[0].task<<endl);
                }
                pre_rresp_time = now();
                unsigned channel = readResp[0].channel;
                readResp.erase(readResp.begin());
                linked_ptcs_[ptcID(channel)]->dresp_cnt--;
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rresp Back Pressure :: task="<<readResp[0].task<<endl);
                }
            }
        }
    }
}

bool PFQ::clkSyn() {
    if (now()%CLKH2CLKL != 0) {
        return false;
    }

    m_pre_update_time = now();
    return true;
}

/***************************************************************************************************
Descriptor: update Cresp
****************************************************************************************************/
void PFQ::updateCresp() {
    if (!cmdResp.empty()) {
        if (pre_cresp_time != now()) {
            // auto mpfq_ = ;
            if ((*mpfq_->CmdResp)(cmdResp[0].channel, cmdResp[0].task, 0, 0, 0)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Cresp Received :: task="<<cmdResp[0].task<<endl);
                }
                pre_cresp_time = now();
                cmdResp.erase(cmdResp.begin());
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Cresp Back Pressure :: task="<<cmdResp[0].task<<endl);
                }
            }
        }
    }
}

/***************************************************************************************************
Descriptor: update state:PRINTN & DEBUG
****************************************************************************************************/
void PFQ::update_state() {
    if (!DEBUG_GBUF_STATE) return;
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Perf Total Status: R:"<<rcmd_cnt<<" W:"<<wcmd_cnt<<" wbuff_state"<<wbuff_state<<" occ="<<linked_ptcs_[0]->occ
            <<" state_trig="<<state_trig<<endl);
    for (auto& t : PerfQue) {
        if (t == nullptr) continue;
        if (t->transactionType == DATA_WRITE) continue;
        if (t->in_ptc) continue;
        auto r = t->conflict_state;
        auto rg = r->perf;
        auto rd = r->dmc;
        PRINTN("Rbuf time: "<<now()<<" | task="<<t->task<<" | mask="<<t->mask_wcmd<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | len="<<t->burst_length<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" rd_byp="<<t->has_active<<" | rhit="<<r->rowhit<<" | g_bg_cc="<<rg.bg_conf_cnt<<" | g_ba_cc="
                <<rg.ba_conf_cnt<<" | g_ad_cc="<<rg.ad_conf_cnt<<" | d_bg_cc="<<rd.bg_conf_cnt<<" | d_ba_cc="
                <<rd.ba_conf_cnt<<" | d_ad_cc="<<rd.ad_conf_cnt<<" | r_bank_cnt="<<linked_ptcs_[0]->r_bank_cnt[t->bankIndex]
                <<" | w_bank_cnt="<<linked_ptcs_[0]->w_bank_cnt[t->bankIndex]<<" | in_ptc="<<t->in_ptc<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    for (auto& t : PerfQue) {
        if (t == nullptr) continue;
        if (t->transactionType == DATA_READ) continue;
        if (t->in_ptc) continue;
        auto w = t->conflict_state;
        auto wg = w->perf;
        auto wd = w->dmc;
        PRINTN("Wbuf time: "<<now()<<" | task="<<t->task<<" | mask="<<t->mask_wcmd<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | len="<<t->burst_length<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" rd_byp="<<t->has_active<<" | rhit="<<w->rowhit<<" | g_bg_cc="<<wg.bg_conf_cnt<<" | g_ba_cc="
                <<wg.ba_conf_cnt<<" | g_ad_cc="<<wg.ad_conf_cnt<<" | d_bg_cc="<<wd.bg_conf_cnt<<" | d_ba_cc="
                <<wd.ba_conf_cnt<<" | d_ad_cc="<<wd.ad_conf_cnt<<" | r_bank_cnt="<<linked_ptcs_[0]->r_bank_cnt[t->bankIndex]
                <<" | w_bank_cnt="<<linked_ptcs_[0]->w_bank_cnt[t->bankIndex]<<" | in_ptc"<<t->in_ptc<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Perf Total Status(in Ptc): RR:"<<rel_rcmd_cnt<<" RW:"<<rel_wcmd_cnt<<endl);
    for (auto& t : PerfQue) {
        if (t == nullptr) continue;
        if (t->transactionType == DATA_WRITE) continue;
        if (!t->in_ptc) continue;
        auto r = t->conflict_state;
        auto rg = r->perf;
        auto rd = r->dmc;
        PRINTN("Rbuf time: "<<now()<<" | task="<<t->task<<" | mask="<<t->mask_wcmd<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | len="<<t->burst_length<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" rd_byp="<<t->has_active<<" | rhit="<<r->rowhit<<" | g_bg_cc="<<rg.bg_conf_cnt<<" | g_ba_cc="
                <<rg.ba_conf_cnt<<" | g_ad_cc="<<rg.ad_conf_cnt<<" | d_bg_cc="<<rd.bg_conf_cnt<<" | d_ba_cc="
                <<rd.ba_conf_cnt<<" | d_ad_cc="<<rd.ad_conf_cnt<<" | r_bank_cnt="<<linked_ptcs_[0]->r_bank_cnt[t->bankIndex]
                <<" | w_bank_cnt="<<linked_ptcs_[0]->w_bank_cnt[t->bankIndex]<<" | in_ptc="<<t->in_ptc<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    for (auto& t : PerfQue) {
        if (t == nullptr) continue;
        if (t->transactionType == DATA_READ) continue;
        if (t->in_ptc) continue;
        auto w = t->conflict_state;
        auto wg = w->perf;
        auto wd = w->dmc;
        PRINTN("Wbuf time: "<<now()<<" | task="<<t->task<<" | mask="<<t->mask_wcmd<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | len="<<t->burst_length<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" rd_byp="<<t->has_active<<" | rhit="<<w->rowhit<<" | g_bg_cc="<<wg.bg_conf_cnt<<" | g_ba_cc="
                <<wg.ba_conf_cnt<<" | g_ad_cc="<<wg.ad_conf_cnt<<" | d_bg_cc="<<wd.bg_conf_cnt<<" | d_ba_cc="
                <<wd.ba_conf_cnt<<" | d_ad_cc="<<wd.ad_conf_cnt<<" | r_bank_cnt="<<linked_ptcs_[0]->r_bank_cnt[t->bankIndex]
                <<" | w_bank_cnt="<<linked_ptcs_[0]->w_bank_cnt[t->bankIndex]<<" | in_ptc"<<t->in_ptc<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
}

/***************************************************************************************************
Descriptor: update_state_trig&
****************************************************************************************************/
void PFQ::update_state_trig() {
    if (PERF_RWGRP_MODE != 0) return;
    if (wbuff_state == WBUFF_IDLE) {
        state_trig = 0;
        state_trig = check_wr_level();
        if (state_trig == 0 && (rel_wcmd_cnt + wcmd_cnt) > 0 && (rel_rcmd_cnt + rcmd_cnt) == 0) {
            if (idle_trig_time == IDLE_TRIG_TIME) state_trig = 1; // idle trig
            else idle_trig_time ++;
        } else {
            idle_trig_time = 0;
        }
    }

    if (state_trig != 0) {
        GrpMode.grp_mode0 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 0, 1);
        GrpMode.grp_mode1 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 1, 1);
        GrpMode.grp_mode23 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 2, 2);
        GrpMode.grp_mode4 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 4, 1);
        GrpMode.grp_mode5 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 5, 1);
        GrpMode.grp_mode6 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 6, 1);
        GrpMode.grp_mode7 = BIT_GET(*grp_mode[linked_ptcs_[0]->occ][state_trig - 1], 7, 1);
    }

    if (wbuff_state_gap > 0) wbuff_state_gap --;

    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- State Trig Update: state_trig="<<state_trig<<", availability="<<linked_ptcs_[0]->occ
                <<", wcmd_cnt="<<wcmd_cnt<<", rcmd_cnt="<<rcmd_cnt<<", dmc_rd_cnt="<<linked_ptcs_[0]->Read_Cnt()
                <<", rel_wcmd_cnt="<<rel_wcmd_cnt<<", rel_rcmd_cnt="<<rel_rcmd_cnt
                <<", idle_trig_time="<<idle_trig_time<<", wbuff_state_gap="<<wbuff_state_gap<<endl)
    }
}

unsigned PFQ::check_wr_level() {
    unsigned rd_cnt = 0;
    unsigned wr_cnt = 0;
    if (TOTAL_RCMD_MODE == 0) rd_cnt = rel_rcmd_cnt + rcmd_cnt;
    else if (TOTAL_RCMD_MODE == 1) rd_cnt = rcmd_cnt;
    else {
        for (uint32_t i=0; i < (NUM_CHANS/NUM_PFQS); i++) {
            rd_cnt = rd_cnt + linked_ptcs_[i]->Read_Cnt();
        }
    }
    wr_cnt = rel_wcmd_cnt + wcmd_cnt; 

    unsigned state_trig_tmp = 0;
    if (wr_cnt > *wr_level[linked_ptcs_[0]->occ][4] && rd_cnt < *rd_level[linked_ptcs_[0]->occ][4]) state_trig_tmp = 5;
    else if (wr_cnt <= *wr_level[linked_ptcs_[0]->occ][4] && wr_cnt > *wr_level[linked_ptcs_[0]->occ][3]
            && rd_cnt < *rd_level[linked_ptcs_[0]->occ][3]) state_trig_tmp = 4;
    else if (wr_cnt <= *wr_level[linked_ptcs_[0]->occ][3] && wr_cnt > *wr_level[linked_ptcs_[0]->occ][2]
            && rd_cnt < *rd_level[linked_ptcs_[0]->occ][2]) state_trig_tmp = 3;
    else if (wr_cnt <= *wr_level[linked_ptcs_[0]->occ][2] && wr_cnt > *wr_level[linked_ptcs_[0]->occ][1]
            && rd_cnt < *rd_level[linked_ptcs_[0]->occ][1]) state_trig_tmp = 2;
    else if (wr_cnt <= *wr_level[linked_ptcs_[0]->occ][1] && wr_cnt > *wr_level[linked_ptcs_[0]->occ][0]
            && rd_cnt < *rd_level[linked_ptcs_[0]->occ][0]) state_trig_tmp = 1;

    return state_trig_tmp;

}

/***************************************************************************************************
Descriptor: perf_check_timeout_and_aging&
****************************************************************************************************/
void PFQ::perf_check_timeout_and_aging() {
    // adapt cmd qos not related to tout
    if (PERF_PRI_ADAPT_ENABLE) {
        for (auto& trans : PerfQue) {
            if (trans == nullptr) continue;
            if (trans->in_ptc) continue;
            if (now() < trans->arb_time) continue;
            if (trans->pri_adapt_th == 0 || trans->qos == QOS_MAX) continue;
            if (now() - trans->timeAdded >= ((trans->improve_cnt + 1) * trans->pri_adapt_th)) {
                trans->improve_cnt ++;
                if (trans->pri < PERF_ADAPT_PRI_MAX) {   // adapt low qos to pri_max && noadapt for hqos 
                    trans->pri ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PERF PRI_ADAPT :: task="<<trans->task<<" pri="<<trans->pri
                                <<" qos="<<trans->qos<<endl);
                    }
                }
            }
        }
    }

    //timeout check
    if (!PERF_TIMEOUT_EN) return;

    // calculate the highest pri for all commands in DMC Queue
    unsigned highest_pri = 0x0;
    if (PERF_QOS_POLICY == 2) {        
        for (auto& trans : PerfQue) {
            if (trans == nullptr) continue;
            if (trans->in_ptc) continue;
            if (trans->fast_rd && !trans->act_only) continue;
            if (now() < trans->arb_time) continue;
            if (trans->pri > highest_pri) highest_pri = trans->pri;
        }    
    }

    has_rd_tout = false;
    has_wr_tout = false;

    // label timeout and adapt pri to QOS_MAX  
    for (auto& trans : PerfQue) {
        if (trans == nullptr) continue;
        if (trans->in_ptc) continue;
        if (now() < trans->arb_time) continue;
        if (trans->perf_addrconf) continue;
        if (trans->fast_rd && !trans->act_only) continue;
        if ((trans->transactionType == DATA_WRITE) && (trans->data_ready_cnt < (trans->burst_length+1))) continue;
        if (!trans->timeout && ((now() - trans->timeAdded >= trans->timeout_th && trans->timeout_th != 0))) {
            if (trans->qos == QOS_MAX || (PERF_QOS_POLICY == 1) || (PERF_QOS_POLICY == 2 && trans->pri >= highest_pri)) {
                trans->timeout = true;
                trans->timeout_type = 0;
                trans->time_timeout = now();
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                            <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<endl);
                }
                if (PRINT_TIMEOUT) {
                    PRINTN(setw(10)<<now()<<" -- PERF TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                            <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<endl);
                }
                if (trans->pri != QOS_MAX) {
                    trans->pri = QOS_MAX;
                }
            }
        }
        if (trans->timeout && !has_rd_tout && trans->transactionType == DATA_READ) has_rd_tout = true;
        if (trans->timeout && !has_wr_tout && trans->transactionType == DATA_WRITE) has_wr_tout = true;
    }
    
    // generate dummy timeout
    has_rd_dummy_tout = false;
    has_wr_dummy_tout = false;
    if (PERF_DUMMY_TOUT_EN) {
        for (auto& trans : PerfQue) {
            if (trans == nullptr) continue;
            auto bank_state = linked_ptcs_[ptcID(trans->channel)]->get_bank_state(trans->bankIndex);
            auto cur_bank_state = bank_state.state->currentBankState;
            auto open_row = bank_state.state->openRowAddress;
            bool rowhit = (cur_bank_state == CurrentBankState::RowActive) && (open_row == trans->row); 

            trans->dummy_timeout = false;
            // if (trans == nullptr) continue;
            if (!trans->timeout) continue;
            if (trans->dummy_ever) continue;
            if (rowhit) continue;  // rowhit -> not dummy
            if (bankConfPolicyPass(trans) && cur_bank_state != CurrentBankState::RowActive) continue; // rowmiss && bank credit enough && not RowActive -> not dummy
            trans->dummy_timeout = true;
            if (trans->transactionType == DATA_READ) has_rd_dummy_tout = true;
            else has_wr_dummy_tout = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF DUMMY TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<endl);
            }
            if (PRINT_TIMEOUT) {
                PRINTN(setw(10)<<now()<<" -- PERF DUMMY TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<endl);
            }
        }        
    }
}

/***************************************************************************************************
Descriptor: sch_subque& lv1 arb
****************************************************************************************************/
void PFQ::sch_subque() {

    bool schedule_read = true;
    bool schedule_write = true;
    if (TOUT_FORCE_RWGRP_EN) {
        if (wbuff_state == WBUFF_WRITE) {
            schedule_read = false;
        } else if (wbuff_state == WBUFF_IDLE) {
            schedule_write = false;
        }
    } else {
        if (wbuff_state == WBUFF_WRITE && !has_rd_tout) {
            schedule_read = false;
        } else if (wbuff_state == WBUFF_IDLE && !has_wr_tout) {
            schedule_write = false;
        }
    }
    if (!schedule_read && !schedule_write) return;

    std::map<string,unsigned> forbidden_cnt;
    for (size_t subq_idx = 0; subq_idx < PERF_SUBQ_NUM; subq_idx++) {
        auto range = getSubqRange(subq_idx);
        size_t start_idx = range.first;
        size_t end_idx = range.second;
        size_t lastselcmd_idx = static_cast<size_t>(-1);
        arb_cmd* cur_cmd = nullptr;
        for (size_t i = start_idx; i <= end_idx; i++) {
            Transaction* trans = PerfQue[i];
            if (trans == nullptr) continue;
            if (trans->in_ptc) continue;
            auto& conf = trans->conflict_state;
            unsigned bankIndex = trans->bankIndex;
            unsigned rank = trans->rank;
            unsigned sub_channel = (bankIndex % NUM_BANKS) / (linked_ptcs_[ptcID(trans->channel)]->sc_bank_num);
            if (!schedule_read && trans->transactionType == DATA_READ) {
                forbidden_cnt["read_disabled"]++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF rcmd blocked by WRITE_GROUP :: "<< "task="
                            <<trans->task<<endl);
                } 
                continue;
            }
            if (!schedule_write && trans->transactionType == DATA_WRITE) {
                forbidden_cnt["write_disabled"]++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF wcmd blocked by READ_GROUP :: "<< "task="
                            <<trans->task<<endl);
                }
                continue;
            }
            // check for high qos label
            if (trans->hqos_timeout && trans->dummy_hqos && trans->transactionType==DATA_READ) {
                ERROR(setw(10)<<now()<<" Hqos Timeout and Dummy Not Coexist, task="<<trans->task<<" hqos_timeout="<<trans->hqos_timeout
                        <<" dummy_hqos="<<trans->dummy_hqos<<" dummy_hqos_ever="<<trans->dummy_hqos_ever);
                assert(0);
            }

            if ((trans->hqos_timeout || trans->dummy_hqos || trans->dummy_hqos_ever) && trans->transactionType==DATA_WRITE) {
                ERROR(setw(10)<<now()<<" High Qos Not for Wcmd, task="<<trans->task<<" hqos_timeout="<<trans->hqos_timeout
                        <<" dummy_hqos="<<trans->dummy_hqos<<" dummy_hqos_ever="<<trans->dummy_hqos_ever);
                assert(0);
            }

            if (now() < trans->arb_time) {
                forbidden_cnt["arb_time"]++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by arb_time :: "<< "task="
                        <<trans->task<<" TimeAdded="<<trans->timeAdded<<" ArbTime="<<trans->arb_time<<endl);
                }
                continue;
            }

            if (trans->transactionType==DATA_WRITE && trans->data_ready_cnt < (trans->burst_length + 1)) {
                forbidden_cnt["data_ready"]++;
                if(DEBUG_BUS){
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by data_ready :: "<< "task="
                        <<trans->task<<" DataReadyCnt="<<trans->data_ready_cnt << " NeedSize="<<trans->burst_length+1<<endl);
                }
                continue;
		    }

            if ((conf->dmc.ad_conf_cnt + conf->perf.ad_conf_cnt) > 0) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by addr_conf :: "<< "task="
                        <<trans->task<<" Addr="<<trans->address<<" perf_ad_cc="<<conf->perf.ad_conf_cnt<<endl;)
                }
                if (conf->dmc.ad_conf_cnt > 0) {
                    ERROR(setw(10)<<now()<<" -- r_dmc_ad_cc larger than 0, task="<<trans->task);
                    assert(0);
                }
                continue;
            }
            if (trans->fast_rd) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by fast_rd :: "<< "task="
                        <<trans->task<<" fast_rd="<<trans->fast_rd<<" act_only="<<trans->act_only<<endl);
                }
                continue;
            }
            if (trans->bp_by_tout) {
                forbidden_cnt["timeout_bp"]++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by timeout, task="<<trans->task<<", qos="<<trans->qos<<", timeAdded="<<trans->timeAdded<<endl);
                }
                continue;
            }
            if (linked_ptcs_[ptcID(trans->channel)]->get_refresh_state(trans->bankIndex)) {//rename
                forbidden_cnt["refresh_cnf"]++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by refresh, task="<<trans->task<<", qos="
                        <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                        <<", row="<<trans->row<<", ch="<<trans->channel
                            <<", pbrw="<<linked_ptcs_[ptcID(trans->channel)]->refreshPerBank[bankIndex].refreshWaiting<<", pbr="<<linked_ptcs_[ptcID(trans->channel)]->refreshPerBank[bankIndex].refreshing
                            <<", pbr_pre="<<linked_ptcs_[ptcID(trans->channel)]->refreshPerBank[bankIndex].refreshWaitingPre<<", abrW="<<linked_ptcs_[ptcID(trans->channel)]->refreshALL[rank][sub_channel].refreshWaiting
                            <<", abr="<<linked_ptcs_[ptcID(trans->channel)]->refreshALL[rank][sub_channel].refreshing<<endl);
                }
                continue;
            }

            if (!sidGroupPass(trans)) {
                continue;
            }
            if (!rankGroupPass(trans)) {
                continue;
            }
            if (!rowhitPolicyPass(trans)) {
                continue;
            }
            if (!ptcDummyToutPolicyPass(trans)) {
                continue;
            }
            if (!bankGroupPolicyPass(trans)) {
                continue;
            }      
    
            arb_cmd *cmd = new arb_cmd;
            cmd->creat(trans);

            auto& pri_info = cmd->pri_info;
            if (PERF_RANK_PRI_EN) pri_info.m_rank_pri = getRankGroupPri(trans);
            pri_info.m_bg_pri = getBankGroupPri(trans);
            if (PERF_STATE_PRI_EN) pri_info.m_state_pri = trans->state_pri; 
            pri_info.m_ori_pri = trans->pri;
            if (PERF_RHIT_PRI_EN) pri_info.m_rowhit_pri = realRowhit(trans);

            if (cur_cmd == nullptr) {
                cur_cmd = cmd; 
                lastselcmd_idx = i;
 
            } else if (cmd->pri_info > cur_cmd->pri_info) {
                cur_cmd = cmd;
                lastselcmd_idx = i;
            } else if (cmd->pri_info == cur_cmd->pri_info) {
                if (lru_arb(i, lastselcmd_idx, 1)) {
                    cur_cmd = cmd;
                    lastselcmd_idx = i;
                }
            }
        }
        ArbCmd[subq_idx] = cur_cmd;
        
        if (ArbCmd[subq_idx] != nullptr) {
            update_lru(lastselcmd_idx, 1, cur_cmd);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF cmd enter ArbCmd Lv1 arb, task="<<ArbCmd[subq_idx]->task<<", subq="<<subq_idx<<", qos="<<ArbCmd[subq_idx]->qos<<", type="<<ArbCmd[subq_idx]->type
                        <<", timeAdded="<<ArbCmd[subq_idx]->timeAdded<<", rank="<<ArbCmd[subq_idx]->rank<<", group="<<ArbCmd[subq_idx]->group<<", bank="<<ArbCmd[subq_idx]->bank
                        <<", bankIndex="<<ArbCmd[subq_idx]->bankIndex<< " Pri_info="<<ArbCmd[subq_idx]->pri_info.m_bg_pri<<","<<ArbCmd[subq_idx]->pri_info.m_bg_num<<","<<ArbCmd[subq_idx]->pri_info.m_bank_num
                        <<","<<ArbCmd[subq_idx]->pri_info.m_ori_pri<<","<<ArbCmd[subq_idx]->pri_info.m_rowhit_pri<<","<<ArbCmd[subq_idx]->pri_info.m_rank_pri<<","<<ArbCmd[subq_idx]->pri_info.m_state_pri<<endl);
            }
        }
    }
}

// SID分组通过检查（类似rankGroupPass，用于调度时过滤事务）
bool PFQ::sidGroupPass(Transaction* trans) {
    if (!SIMPLE_GRP_SID_EN) return true;
    unsigned rank = trans->rank;
    unsigned sid = trans->sid;

    // 紧急事务直接放行
    if (trans->timeout) return true;
    if (trans->dummy_timeout) return true;
    if (trans->hqos_timeout) return true;
    if (trans->dummy_hqos) return true;

    // 获取当前分组状态（流水线头部）
    int cur_state = sid_group_state.empty() ? NO_LR_GROUP : sid_group_state[0];
    unsigned cur_target_sid = quc_slt_grp_lr.empty() ? 0 : quc_slt_grp_lr[0];

    // 如果不在LR分组，或者实际生效组为4（过渡态），则允许所有SID通过
    if (cur_state != LR_GROUP || in_sid_group == 4) return true;

    // 以下逻辑对应于原scheduler中的两个if块，用于实现SID组内的背压
    // 条件：处于LR分组，且实际生效组已确定，且事务类型为读，且满足一系列条件
    if (cur_state == LR_GROUP && in_sid_group != 4 &&
        trans->transactionType == DATA_READ && !trans->timeout) {

        // 第一个if块：当当前事务的SID不等于目标SID时，背压
        if (trans->sid != cur_target_sid) {
            // 检查是否有其他条件（与原代码一致）
            // 这里直接阻塞，但原代码还有更深的条件，可简化
            if (DEBUG_BUS) {
                PRINTN(setw(10) << now() << " -- PERF cmd blocked by Sid group. task="
                       << trans->task << ", cur_sid=" << in_sid_group
                       << ", cmd_sid=" << trans->sid << endl);
            }
            return false;
        }

        // // 第二个if块：当当前事务的SID等于目标SID时，检查是否有其他SID的未完成命令
        // if (trans->sid == in_sid_group) {
        //     // 检查是否有其他SID的请求未完成（sid_issue_state需要定义）
        //     // 这里借用PFQ中可能存在的变量，例如issue状态表
        //     bool other_sid_issued = false;
        //     for (unsigned s = 0; s < NUM_SIDS; ++s) {
        //         if (s == trans->sid) continue;
        //         if (sid_issue_state[s]) { // sid_issue_state需在PFQ中定义
        //             other_sid_issued = true;
        //             break;
        //         }
        //     }
        //     if (other_sid_issued) {
        //         if (DEBUG_BUS) {
        //             PRINTN(setw(10) << now() << " -- LC :: Incomplete READ command backpress a READ command. task="
        //                    << trans->task << ", nextCmd=" << trans->nextCmd << ", cur_sid=" << in_sid_group[rank]
        //                    << ", cmd_sid=" << trans->sid << endl);
        //         }
        //         return false;
        //     }
        // }
    }

    return true;
}

std::set<unsigned> PFQ::getPtcExistRank() {
    std::set<unsigned> ptc_exist_rank;
    for (size_t i = 0; i < (NUM_CHANS/NUM_PFQS); i++) {
        for (unsigned rank_id = 0; rank_id < NUM_RANKS; rank_id++) {
            unsigned rd_cnt = linked_ptcs_[i]->get_rank_cnt(rank_id,true);
            unsigned wr_cnt = linked_ptcs_[i]->get_rank_cnt(rank_id,false);
            if ((rd_cnt+wr_cnt) != 0) {
                ptc_exist_rank.insert(rank_id);
            }
        }
    }
    return ptc_exist_rank;
}

bool PFQ::rankGroupPass(Transaction* trans) {
    // timeout/dummy timeout/hqos_timeout/hqos_dummy ignore rank policy
    if (trans->timeout) return true;
    if (trans->timeout && trans->dummy_timeout) return true;
    if (trans->hqos_timeout) return true;
    if (trans->dummy_hqos) return true;
    
    unsigned perf_timeout_rank_cnt = linked_ptcs_[ptcID(trans->channel)]->get_perf_timeout_rank_cnt(trans->rank);
    
        if (PTC_MAX_RANK > 0) {
        std::set<unsigned> ptc_exist_rank = getPtcExistRank();
        if (ptc_exist_rank.size() > PTC_MAX_RANK) {
            ERROR("Too Much Rank in PTC");
            assert(0);
        }else if (ptc_exist_rank.size() == PTC_MAX_RANK) {
            if (ptc_exist_rank.count(trans->rank) == 0) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by rankpolicy1, task="<<trans->task<<", qos="
                           <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                           <<", row="<<trans->row<<endl);
                }
                return false;
            }
        }
    }
    
    if (PTC_MAX_CNT_PER_RANK > 0) {
        unsigned rank_cnt_rw = linked_ptcs_[ptcID(trans->channel)]->get_rank_cnt(trans->rank,0) + linked_ptcs_[ptcID(trans->channel)]->get_rank_cnt(trans->rank,1);
        if ((rank_cnt_rw - perf_timeout_rank_cnt) > PTC_MAX_CNT_PER_RANK) {
            ERROR("Too Much Cmd Per Rank in PTC");
            assert(0);
        }else if (rank_cnt_rw >= PTC_MAX_CNT_PER_RANK) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by rankpolicy2, task="<<trans->task<<", qos="
                       <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                       <<", row="<<trans->row<<", ptc_rank_cmd="<<rank_cnt_rw<<", max_per_rank="<<PTC_MAX_CNT_PER_RANK<<endl);
            }
            return false;
        }
    }
    if (PERF_RANK_WMERGE_EN || PERF_RANK_RMERGE_EN) {
        if (((trans->rank != perf_sch_wrank && PERF_RANK_WMERGE_EN && wbuff_state==WBUFF_WRITE) || 
                (trans->rank != perf_sch_rrank && PERF_RANK_RMERGE_EN && wbuff_state==WBUFF_IDLE)) && (wbuff_rnkgrp_state == WBUFF_IN_RNK_GROUP))  {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF rcmd blocked by rankpolicy3, task="<<trans->task<<", qos="
                       <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                       <<", row="<<trans->row<<", rw_state="<<wbuff_state<<", rnk_state="<<wbuff_rnkgrp_state
                       <<", rd_rnkgrp="<<perf_rd_in_rnkgrp<<", wr_rnkgrp="<<perf_wr_in_rnkgrp<<endl);
            }
            return false;
        }
    }
    return true;
}

bool PFQ::bankStatePolicyPass(Transaction* trans) {
    auto bank_state = linked_ptcs_[ptcID(trans->channel)]->get_bank_state(trans->bankIndex);
    bool act_timing_met = bank_state.act_timing_met;
    auto cur_bank_state = bank_state.state->currentBankState;
    auto open_row = bank_state.state->openRowAddress;

    bool state_timing_met = false;
    if (!((trans->timeout && trans->dummy_timeout) || trans->dummy_hqos)) {  // non-dummy_timeout/non-dummy_hqos
        if ((cur_bank_state == CurrentBankState::Refreshing) || (cur_bank_state == CurrentBankState::PowerDown)) {
            state_timing_met = false;
        }else if (cur_bank_state == CurrentBankState::Idle) {  
            if (act_timing_met) {
                state_timing_met = true;
                trans->state_pri = 1;
            } else {
                state_timing_met = false;
            }
        }else if (cur_bank_state == CurrentBankState::Precharging) {
            if (act_timing_met) {
                state_timing_met = true;
                trans->state_pri = 0;
            } else {
                state_timing_met = false;
            }
        }else if (cur_bank_state == CurrentBankState::RowActive) {    // todo: problem when PTC /send Ap cmd ???
            if (open_row == trans->row) {
                state_timing_met = true;
                trans->state_pri = 2;
            } else {
                state_timing_met = false;
            }
        } else {
            ERROR("ERROR BANK STATE!!");
            assert(0);
        }
    } else {   // dummy_timeout/dummy_hqos
        if (cur_bank_state == CurrentBankState::Refreshing 
                || cur_bank_state == CurrentBankState::Idle || cur_bank_state == CurrentBankState::RowActive) {
            state_timing_met = true; 
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Dummy Timeout/Hqos Pass act_timing, task="<<trans->task<<", qos="
                    <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                    <<", row="<<trans->row<<", bank_state="<<linked_ptcs_[ptcID(trans->channel)]->bank_state_opcode(cur_bank_state)<<", state_met="<<state_timing_met
                    <<", act_met="<<act_timing_met<<" dummy_timeout="<<trans->dummy_timeout<<" dummy_hqos="<<trans->dummy_hqos<<endl);
        }
    
    }


    if (DEBUG_BUS && !state_timing_met) {
        PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by act_timing, task="<<trans->task<<", qos="
                <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                <<", row="<<trans->row<<", bank_state="<<linked_ptcs_[ptcID(trans->channel)]->bank_state_opcode(cur_bank_state)<<", state_met="<<state_timing_met
			    <<", act_met="<<act_timing_met<<endl);
    }

    return state_timing_met;
}

bool PFQ::bankConfPolicyPass(Transaction* trans) {
    // dummy timeout/dummy hqos igore baconf policy
    if (trans->timeout && trans->dummy_timeout) return true;
    if (trans->dummy_hqos) return true;

    auto bank_state = linked_ptcs_[ptcID(trans->channel)]->get_bank_state(trans->bankIndex);
    auto cur_bank_state = bank_state.state->currentBankState;
    auto open_row = bank_state.state->openRowAddress;
    unsigned same_bank_cnt = linked_ptcs_[ptcID(trans->channel)]->get_bank_cnt(trans->bankIndex,true)
            + linked_ptcs_[ptcID(trans->channel)]->get_bank_cnt(trans->bankIndex,false);
    bool row_hit = (cur_bank_state == CurrentBankState::RowActive && open_row == trans->row);

    unsigned max_same_bank_cnt = 0;
    if (row_hit) {
        if (trans->timeout) return true;   //perf timeout && rowhit exclude
        if (trans->hqos_timeout) return true;  //perf hqos && rowhit exclude
        if (trans->transactionType == DATA_READ) {
            max_same_bank_cnt = SAME_BANK_CNT_RD;
        } else {
            max_same_bank_cnt = SAME_BANK_CNT_WR;
        }
    } else {
        max_same_bank_cnt = SAME_BANK_CNT_ROWMISS;
    }

    if (max_same_bank_cnt == 0) return true; 

    if (same_bank_cnt < max_same_bank_cnt) {
        return true;
    } else {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by bankpolicy, task="<<trans->task<<", qos="
                   <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                   <<", row="<<trans->row<<", same_bank_cnt="<<same_bank_cnt<<", max_same_bank_cnt="<<max_same_bank_cnt<<endl);
        }
        return false;
    }
}

bool PFQ::ptcRowhitBreakPolicyPass(Transaction* trans) {
    if (!PTC_RHIT_BREAK_EN) return true;

    //dummy timeout/dummy hqos ignore rhit break policy
    if (trans->timeout && trans->dummy_timeout) return true;
    if (trans->dummy_hqos) return true;

    auto bank_state = linked_ptcs_[ptcID(trans->channel)]->get_bank_state(trans->bankIndex);
    bool rhit_break_bankbp = bank_state.has_rhit_break;

    if (!rhit_break_bankbp) {
        return true;
    } else {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by rhitbreak policy, task="<<trans->task<<", qos="
                   <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                   <<", row="<<trans->row<<", openrow="<<bank_state.state->openRowAddress<<endl);
        }
        return false;
    }
}

bool PFQ::rowhitPolicyPass(Transaction* trans) {
    if (!bankStatePolicyPass(trans) && PERF_ACT_TCHK_EN) {
        return false;
    }
    if (!bankConfPolicyPass(trans)) {
        return false;
    }
    if (!ptcRowhitBreakPolicyPass(trans)) {
        return false;
    }
    return true;
}

bool PFQ::ptcDummyToutPolicyPass(Transaction* trans) {
    if (!PERF_DUMMY_TOUT_EN) return true;

    auto bank_state = linked_ptcs_[ptcID(trans->channel)]->get_bank_state(trans->bankIndex);
    bool dummy_tout_bankbp = bank_state.has_dummy_tout;

    // dummy label check

    if (!dummy_tout_bankbp) {
        return true;
    } else {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by dummy_tout policy, task="<<trans->task<<", qos="
                   <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                   <<", row="<<trans->row<<", openrow="<<bank_state.state->openRowAddress<<endl);
        }
        return false;
    }
}

bool PFQ::bankGroupPolicyPass(Transaction* trans) {
    // dummy tiemout igore bg policy
    if (trans->timeout && trans->dummy_timeout) return true;
    if (trans->timeout) return true;

    unsigned bg_cnt = linked_ptcs_[ptcID(trans->channel)]->get_bg_cnt(trans->rank,trans->group,true)
            + linked_ptcs_[ptcID(trans->channel)]->get_bg_cnt(trans->rank,trans->group,false);
    unsigned max_bg_cnt = 0;
    if (trans->transactionType == DATA_READ) {
        max_bg_cnt = SAME_BG_CNT_RD;
    } else {
        max_bg_cnt = SAME_BG_CNT_WR;
    }
    
    if (max_bg_cnt == 0) return true;

    if (bg_cnt >= max_bg_cnt) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF cmd blocked by bgpolicy, task="<<trans->task<<", qos="
                   <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
                   <<", row="<<trans->row<<", bg_cnt="<<bg_cnt<<", max_bg="<<max_bg_cnt<<endl);
        }
        return false;
    } else {
        return true;
    }
}

unsigned PFQ::getRankGroupPri(Transaction* trans) {
    if (NUM_RANKS == 0) return 0;

    unsigned ret = 0;
    if (RANK_GROUP_MODE == 0) {
        ret = 0;
    }else if (RANK_GROUP_MODE == 1) {
        if (trans->rank != pre_cmd.rank) {
            ret = 0;
        } else {
            ret = 1;
        }
    }else if (RANK_GROUP_MODE == 2) {
        if (trans->rank != max_rank) {
            ret = 0;
        } else {
            ret = 1;
        }
    }
    return ret;
}

unsigned PFQ::getBankGroupPri(Transaction* trans) {
    auto& pre_n_cmd = pre_n_cmd_each_rank[trans->rank];
    unsigned ret = 0;
    for (auto it = pre_n_cmd.begin();it != pre_n_cmd.end();it ++) {
        if (it->group != trans->group) {
            ret++;
        } else {
            break;
        }
    }
    return ret;
}

bool PFQ::realRowhit(Transaction* trans) {
    auto bank_state = linked_ptcs_[ptcID(trans->channel)]->get_bank_state(trans->bankIndex);
    auto cur_bank_state = bank_state.state->currentBankState;
    auto open_row = bank_state.state->openRowAddress;
    bool row_hit = (cur_bank_state == CurrentBankState::RowActive && open_row == trans->row);
    return row_hit;

}

/***************************************************************************************************
Descriptor: arb_node& lv2 arb
****************************************************************************************************/
bool PFQ::ArbCmd_notempty() {
    for (auto cmd : ArbCmd) {
        if (cmd != nullptr) {
            return true;
        }
    }
    return false;
}

void PFQ::arb_node() {
    perf_cmd_vld = false;

    if (DEBUG_BUS) {
        auto ptc_cmd_str = getPTCCmdSituation();
        PRINTN(setw(10)<<now()<<" -- PTC CMD :: "<<ptc_cmd_str<<endl);
    }
    if (wbuff_state == WBUFF_WRITE) {
        if (!ArbCmd_notempty()) no_cmd_sch_cnt ++;
        else no_cmd_sch_cnt = 0;
    }

    if (wbuff_state == WBUFF_IDLE) {
        if (!ArbCmd_notempty()) no_rcmd_sch_cnt ++;
        else no_rcmd_sch_cnt = 0;
    }
    if (!ArbCmd_notempty()) return;

    for (unsigned i = 0; i < (NUM_CHANS / NUM_PFQS); i++) {
        if (!linked_ptcs_[i]->fastrd_fifo.empty() && PERF_CLK_MODE==0) return;//deal with fastrd & pfq arb cmd in same cycle
    }

    size_t vld_cmd_cnt = 0;
    size_t first_vld_idx = static_cast<size_t>(-1);
    for (size_t i = 0; i < PERF_SUBQ_NUM; i++) {
        if (ArbCmd[i] != nullptr) {
            vld_cmd_cnt++;
            if (first_vld_idx == static_cast<size_t>(-1)) {
                first_vld_idx = i;
            }
        }
    }
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- vld cmd cnt="<<vld_cmd_cnt<<", first_vld_idx="<<first_vld_idx<<endl);
    }

    arb_cmd* selected_cmd = nullptr;
    if (vld_cmd_cnt == 1) {
        if (first_vld_idx == lastselcmd_subq_idx) {
            if (now() >= (lastcmd_seltime + CLKH2CLKL*2)) {
                selected_cmd = ArbCmd[first_vld_idx];
                lastcmd_seltime = now();
                // update_lru(first_vld_idx, 2, selected_cmd);
            }
        }
        else {
            selected_cmd = ArbCmd[first_vld_idx];
            lastselcmd_subq_idx = first_vld_idx;
            lastcmd_seltime = now();
            // update_lru(first_vld_idx, 2, selected_cmd);
        }
    } else if (vld_cmd_cnt > 1) {
        for (size_t i = 0; i < PERF_SUBQ_NUM; i++) {
            if (ArbCmd[i] == nullptr) continue;
            if (i == lastselcmd_subq_idx) continue;
            if (selected_cmd == nullptr) {
                selected_cmd = ArbCmd[i];
                lastselcmd_subq_idx = i;
                lastcmd_seltime = now();
                // update_lru(i, 2, selected_cmd);
                continue;
            }
            if (ArbCmd[i]->pri_info > selected_cmd->pri_info) {
                selected_cmd = ArbCmd[i];
                lastselcmd_subq_idx = i;
                lastcmd_seltime = now();
                // update_lru(i, 2, selected_cmd);
            } else if (ArbCmd[i]->pri_info == selected_cmd->pri_info) {
                if (lru_arb(i, lastselcmd_subq_idx, 2)) {
                    selected_cmd = ArbCmd[i];
                    lastselcmd_subq_idx = i;
                    lastcmd_seltime = now();
                    // update_lru(i, 2, selected_cmd);
                }
            }
        }
    }

    if (selected_cmd != nullptr) {
        update_lru(lastselcmd_subq_idx, 2, selected_cmd);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF cmd selected Lv2 arb, task="<<selected_cmd->task<<", qos="<<selected_cmd->qos<<", type="<<selected_cmd->type
                    <<", timeAdded="<<selected_cmd->timeAdded<<", rank="<<selected_cmd->rank<<", group="<<selected_cmd->group<<", bank="<<selected_cmd->bank
                    <<", bankIndex="<<selected_cmd->bankIndex<< " Pri_info="<<selected_cmd->pri_info.m_bg_pri<<","<<selected_cmd->pri_info.m_bg_num<<","<<selected_cmd->pri_info.m_bank_num
                    <<","<<selected_cmd->pri_info.m_ori_pri<<","<<selected_cmd->pri_info.m_rowhit_pri<<","<<selected_cmd->pri_info.m_rank_pri<<","<<selected_cmd->pri_info.m_state_pri<<endl);
        }
    } else {
        return;
    }

    for (auto& cmd : PerfQue) {
        if (cmd == nullptr) continue;
        if (selected_cmd->type == DATA_READ && cmd->transactionType == DATA_WRITE) continue;
        if (selected_cmd->type == DATA_WRITE && cmd->transactionType == DATA_READ) continue;
        if (cmd->task != selected_cmd->task) continue;
        setRowhitMiss(cmd);
        Transaction* ptc_trans = new Transaction(cmd);
        if (!linked_ptcs_[ptcID(ptc_trans->channel)]->addTransaction(ptc_trans, false)) {
            if (cmd->timeout && cmd->dummy_timeout) {  // sending dummy timeout successfully
                perf_cmd_vld = true;
                cmd->dummy_timeout = false;
                cmd->dummy_ever = true;
            } else if (cmd->dummy_hqos && cmd->transactionType == DATA_READ) {  // sending dummy hqos successfully
                perf_cmd_vld = true;
                cmd->dummy_hqos = false;
                cmd->dummy_hqos_ever = true;
            }
            delete ptc_trans;
        } else { // ptc accept the cmd from pfq
            pre_cmd = *selected_cmd;
            if (selected_cmd->type == DATA_READ) {
                if (rd_sch_rhit_cnt >= 1) rd_sch_rhit_cnt = 0;
                else if (cmd->conflict_state->rowhit && LastArbCmd.bankIndex == cmd->bankIndex) rd_sch_rhit_cnt ++; // rd_sch_rhit_cnt useless?
            }
            if ((selected_cmd->type == DATA_READ && CMD_ROW_ORDER == 2) || (selected_cmd->type == DATA_WRITE && RO_HIT_EN)) {
                for (auto& other_cmd : PerfQue) {
                    if (other_cmd == nullptr) continue;
                    if (other_cmd->transactionType == DATA_WRITE && selected_cmd->type == DATA_READ) continue;
                    if (other_cmd->transactionType == DATA_READ && selected_cmd->type == DATA_WRITE) continue;
                    if (other_cmd->conflict_state->rowhit) continue;
                    if (other_cmd->task == cmd->task) continue;
                    if (other_cmd->bankIndex != cmd->bankIndex) continue;
                    if (other_cmd->row != cmd->row) continue;
                    other_cmd->conflict_state->rowhit = true;
                } 
            }
            LastArbCmd.creat(cmd);
            if (selected_cmd->type == DATA_READ && rcmd_cnt == 0) {
                ERROR(setw(10)<<now()<<" -- rcmd_cnt is 0.");
                assert(0);
            }
            if (selected_cmd->type == DATA_WRITE) {
                if (!WSRAM_MAP_EN) {
                    for (size_t j = 0; j <= cmd->burst_length; j ++) {
                        WdataToSend.push_back(cmd->task);
                    }
                }
                if (cmd->adtimeout) wr_adtimeout_cnt --;
                if (cmd->timeout) wr_timeout_cnt ++;
                if (wcmd_cnt == 0) {
                    ERROR(setw(10)<<now()<<" -- wcmd_cnt is 0.");
                    assert(0);
                }
            }
            perf_cmd_vld = true;
            serial_cmd_cnt ++;
            if (selected_cmd->type == DATA_READ) {
                ser_read_cnt ++;
                rcmd_cnt --;
                rel_rcmd_cnt ++;
                rrel_bank_cnt[selected_cmd->rank][selected_cmd->bankIndex % NUM_BANKS] ++;
                rrel_bg_cnt[selected_cmd->rank][selected_cmd->group] ++;
                rrel_rank_cnt[selected_cmd->rank] ++;
                rel_rank_cnt[selected_cmd->rank] ++;
                rb_bank_cnt[selected_cmd->rank][selected_cmd->bankIndex % NUM_BANKS] --;
                rb_bg_cnt[selected_cmd->rank][selected_cmd->group] --;
                rb_rank_cnt[selected_cmd->rank] --;
                rank_cnt[selected_cmd->rank] --;
                rb_qos_cnt[cmd->qos] --;
                perf2ptc_bank_rcnt[cmd->bankIndex] ++;
                perf2ptc_bg_rcnt[cmd->rank][cmd->group] ++;
                if (selected_cmd->qos >= PERF_SWITCH_HQOS_LEVEL) {
                    que_read_highqos_cnt[selected_cmd->rank] --; 
                }
                if (cmd->timeout) rd_timeout_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_SCH :: READ (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt
                            <<"|QR:"<<linked_ptcs_[ptcID(cmd->channel)]->Read_Cnt()<<"|QW:"<<linked_ptcs_[ptcID(cmd->channel)]->Write_Cnt()<<") task=" <<cmd->task<<" addr="<<hex<<cmd->address<<dec
                            <<" rank="<<cmd->rank<<" group="<<cmd->group<<" bank="<<cmd->bankIndex<<" row="<<cmd->row<<" col="<<cmd->col
                            <<" addr_col="<<cmd->addr_col<<" data_size="<<cmd->data_size<<" qos="<<cmd->qos<<" pri="<<cmd->pri
                            <<" blen="<<cmd->burst_length<<" rhit="<<cmd->conflict_state->rowhit<<" rbc="<<linked_ptcs_[ptcID(cmd->channel)]->r_bank_cnt[cmd->bankIndex]
                            <<" wbc="<<linked_ptcs_[ptcID(cmd->channel)]->w_bank_cnt[cmd->bankIndex]<<" timeout="<<cmd->timeout<<" perf_rd_rowhit="<<cmd->perf_rd_rowhit
                            <<" perf_rd_rowmiss="<<cmd->perf_rd_rowmiss<<" perf_wr_rowhit="<<cmd->perf_wr_rowhit<<" perf_wr_rowmiss="<<cmd->perf_wr_rowmiss 
                            <<" perf_maxbg_hit="<<cmd->perf_maxbg_hit<<" perf_maxbank_hit="<<cmd->perf_maxbank_hit<<endl);
                }
                cmd->has_active = false; // trans_state_clr(r);
                cmd->in_ptc = true;                
                updatePreNCmd(*selected_cmd);
            } else if (selected_cmd->type == DATA_WRITE) {
                wcmd_cnt --;
                rel_wcmd_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_SCH :: WRITE (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|RR:"<<rel_rcmd_cnt<<"|RW:"<<rel_wcmd_cnt
                            <<"|QR:"<<linked_ptcs_[ptcID(cmd->channel)]->Read_Cnt()<<"|QW:"<<linked_ptcs_[ptcID(cmd->channel)]->Write_Cnt()<<") task="<<cmd->task<<" addr="<<hex<<cmd->address<<dec
                            <<" rank="<<cmd->rank<<" group="<<cmd->group<<" bank="<<cmd->bankIndex<<" row="<<cmd->row<<" col="<<cmd->col
                            <<" addr_col="<<cmd->addr_col<<" data_size="<<cmd->data_size<<" qos="<<cmd->qos<<" pri="<<cmd->pri<<" blen="<<cmd->burst_length
                            <<" data_ready_cnt="<<cmd->data_ready_cnt<<" rhit="<<cmd->conflict_state->rowhit<<" rbc="<<linked_ptcs_[ptcID(cmd->channel)]->r_bank_cnt[cmd->bankIndex]
                            <<" wbc="<<linked_ptcs_[ptcID(cmd->channel)]->w_bank_cnt[cmd->bankIndex]<<" timeout="<<cmd->timeout<<" perf_rd_rowhit="<<cmd->perf_rd_rowhit
                            <<" perf_rd_rowmiss="<<cmd->perf_rd_rowmiss<<" perf_wr_rowhit="<<cmd->perf_wr_rowhit<<" perf_wr_rowmiss="<<cmd->perf_wr_rowmiss 
                            <<" perf_maxbg_hit="<<cmd->perf_maxbg_hit<<" perf_maxbank_hit="<<cmd->perf_maxbank_hit<<endl);
                }
                ser_write_cnt ++;
                wrel_bank_cnt[selected_cmd->rank][selected_cmd->bankIndex % NUM_BANKS] ++;
                wrel_bg_cnt[selected_cmd->rank][selected_cmd->group] ++;
                wrel_rank_cnt[selected_cmd->rank] ++;
                rel_rank_cnt[selected_cmd->rank] ++;
                wb_rank_cnt[selected_cmd->rank] --;
                rank_cnt[selected_cmd->rank] --;
                wb_qos_cnt[cmd->qos] --;
                wb_bank_cnt[cmd->rank][cmd->bankIndex % NUM_BANKS] --;
                wb_bg_cnt[cmd->rank][cmd->group] --;
                perf2ptc_bank_wcnt[cmd->bankIndex] ++;
                perf2ptc_bg_wcnt[cmd->rank][cmd->group] ++;
                cmd->has_active = false; // trans_state_clr(w);
                cmd->in_ptc = true;                
                updatePreNCmd(*selected_cmd);
            }
        }
        break;
    }
    
    for (auto& trans : PerfQue) {
        if (trans == nullptr) continue;
        if (trans->in_ptc) continue;
        if ((now() - trans->timeAdded) > 100000) {
            ERROR(setw(10)<<now()<<" -- PERF task="<<trans->task<<" mask="<<trans->mask_wcmd<<" address="<<hex<<trans->address<<dec
                    <<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" matgrp="
                    <<(trans->row&(NUM_MATGRPS-1)));
            ERROR(setw(10)<<now()<<" -- error, qos="<<trans->qos<<", pri="<<trans->pri);
            ERROR(setw(10)<<now()<<" -- FATAL ERROR == big latency"<<", chnl:"<<channel);
            assert(0);
        }
        if (now() - trans->timeAdded > 100000 && trans->transactionType == DATA_WRITE && trans->data_ready_cnt <= trans->burst_length) {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] task="<<trans->task<<" Wdata number miss match, EXP="
                    <<trans->burst_length<<", ACT="<<trans->data_ready_cnt);
            assert(0);
        }
    }   

    for (auto& cmd : ArbCmd) {
        delete cmd;
        cmd = nullptr;
    }
    
}

string PFQ::getPTCCmdSituation() {
    stringstream ret;
    for (unsigned rank_id=0;rank_id<NUM_RANKS;rank_id++) {
        ret<<"Rank"<<rank_id<<" BG0~"<<NUM_GROUPS-1<<":";
        for (unsigned bg_id=0;bg_id<NUM_GROUPS;bg_id++) {
            ret<<linked_ptcs_[0]->get_bg_cnt(rank_id,bg_id,true)+linked_ptcs_[0]->get_bg_cnt(rank_id,bg_id,false)<<" ";
        }
    }
    return ret.str();
}

void PFQ::setRowhitMiss(Transaction* trans) {
    bool exist_task = false;
    trans->perf_rd_rowhit = false;
    trans->perf_wr_rowhit = false;
    trans->perf_rd_rowmiss = false;
    trans->perf_wr_rowmiss = false;

    for (auto& t : PerfQue) {
        if (t == nullptr) continue;
        if (trans->transactionType != t->transactionType) continue;
        if (t->in_ptc) continue;
        if (t->fast_rd) continue;
        if (t->perf_addrconf) continue;       
        if (linked_ptcs_[ptcID(trans->channel)]->fastrd_fifo.size() > 0) {
            if (t->task == linked_ptcs_[ptcID(trans->channel)]->fastrd_fifo[0]->task) {
                continue;
            }
        }        
        if (trans->task == t->task) {
            exist_task = true;
            continue;
        }
        if (trans->bankIndex == t->bankIndex) {
            if (trans->row == t->row) {
                if (trans->transactionType == DATA_READ) {
                    trans->perf_rd_rowhit = true;
                } else {
                    trans->perf_wr_rowhit = true;
                }
            } else {
                if (trans->transactionType == DATA_READ) {
                    trans->perf_rd_rowmiss = true;
                } else {
                    trans->perf_wr_rowmiss = true;
                }
            }
        }
    }
    assert(exist_task);       
    for (auto& t : PerfQue) {
        if (t == nullptr) continue;
        if (trans->transactionType == t->transactionType) continue;
        if (t->in_ptc) continue;
        if (t->fast_rd) continue;
        if (t->perf_addrconf) continue;       
        if (linked_ptcs_[ptcID(trans->channel)]->fastrd_fifo.size() > 0) {
            if (t->task == linked_ptcs_[ptcID(trans->channel)]->fastrd_fifo[0]->task) {
                continue;
            }
        }        
        if (trans->task == t->task) continue;
        if (trans->bankIndex == t->bankIndex) continue;
        if (trans->row == t->row) {
            if (trans->transactionType == DATA_READ) {
                trans->perf_wr_rowhit = true;
            } else {
                trans->perf_rd_rowhit = true;
            }
        } else {
            if (trans->transactionType == DATA_READ) {
                trans->perf_wr_rowmiss = true;
            } else {
                trans->perf_rd_rowmiss = true;
            }
        }       
    }
    return;
}

void PFQ::updatePreNCmd(arb_cmd cmd) {
    if (pre_n_cmd_each_rank[cmd.rank].size() > BG_PRE_N_CMD) {
        assert(0);
    }
    auto& pre_n_cmd = pre_n_cmd_each_rank[cmd.rank];

    for (auto it=pre_n_cmd.begin();it!=pre_n_cmd.end();it++) {
        if (it->group == cmd.group) {
            pre_n_cmd.erase(it);
            break;
        }
    }
    if (pre_n_cmd.size() == BG_PRE_N_CMD) {
        pre_n_cmd.pop_back();
    }
    pre_n_cmd.push_front(cmd);
}

/***************************************************************************************************
Descriptor: stateTransition& --update_rwgroup_state
****************************************************************************************************/
void PFQ::stateTransition() {
    if (PERF_RWGRP_MODE != 0) return;
    unsigned que_read_cnt = rcmd_cnt + rel_rcmd_cnt;  
    unsigned que_write_cnt = wcmd_cnt + rel_wcmd_cnt;
    // count ptc empty state
    bool allptcs_empty = true;
    for (size_t i = 0; i < (NUM_CHANS / NUM_PFQS); i++) {
        if (linked_ptcs_[i]->GetDmcQsize() != 0) {
            allptcs_empty = false;
            break;
        }
    }
    if (allptcs_empty) {
        ptc_empty_cnt++;
    } else {
        ptc_empty_cnt = 0;
    }
    // initialize
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        perf_has_hqos[i] = false;
        que_read_highqos_vld_cnt[i] = 0;
    }
    
    //check if valid read/write cmd
    bool has_read_cmd = false;
    bool has_write_cmd = false;
    for (auto& trans : PerfQue) {
        if (trans == nullptr) continue;
        if (trans->conflict_state == nullptr) continue;
        bool is_read_cmd = (trans->transactionType == DATA_READ);
        bool is_write_cmd = (trans->transactionType == DATA_WRITE);
        auto& conf = trans->conflict_state;
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / (linked_ptcs_[ptcID(trans->channel)]->sc_bank_num);
        
        // check if any high qos cmd in perf queue
        if (is_read_cmd && trans->qos >= PERF_SWITCH_HQOS_LEVEL) { 
            if (!trans->in_ptc && !trans->fast_rd && !trans->perf_addrconf) {
                perf_has_hqos[trans->rank] = true;
                que_read_highqos_vld_cnt[trans->rank] ++;
            }
        }

        if (conf->perf.ad_conf_cnt > 0) continue;
        if (linked_ptcs_[ptcID(trans->channel)]->refreshALL[trans->rank][sub_channel].refreshing) continue;
        if (linked_ptcs_[ptcID(trans->channel)]->refreshPerBank[trans->bankIndex].refreshing) continue;
        if (trans->bp_by_tout) continue;
        if (is_read_cmd) {
            has_read_cmd = true;
        } else if (is_write_cmd && trans->data_ready_cnt == (trans->burst_length + 1)) {
            has_write_cmd = true;
        }
    }
    
    unsigned perf_read_hqos_cnt = (unsigned)accumulate(que_read_highqos_cnt.begin(), que_read_highqos_cnt.end(), 0); 
    unsigned perf_has_hqos_vld = (unsigned)accumulate(perf_has_hqos.begin(), perf_has_hqos.end(), 0); 
    //highqos w2r trigger
    bool high_qos_trig = PERF_RCMD_HQOS_W2R_SWITCH_EN && (perf_read_hqos_cnt >= PERF_RCMD_HQOS_W2R_RLEVELH) && (perf_has_hqos_vld >= 1);
    
    //ptc highqos rank trigger
    if (PTC_HQOS_RANK_SWITCH_EN) {
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (rank_cmd_high_qos[i]) rank_cmd_high_qos[i] = (que_read_highqos_vld_cnt[i] >= PERF_RCMD_HQOS_RANK_SWITCH_LEVELL);
            else rank_cmd_high_qos[i] = (que_read_highqos_vld_cnt[i] >= PERF_RCMD_HQOS_RANK_SWITCH_LEVELH);
        }
    }

    switch (wbuff_state) {
        case WBUFF_NO_GROUP : {
            wbuff_state_pre = WBUFF_NO_GROUP;
            if (PERF_NG_HOLD_WR_EN) {
                if (PERF_NG_HOLD_WR_MODE == 0) {
                    nogrp_cnt ++; 
                
                } else if (PERF_NG_HOLD_WR_MODE == 1) {
                    if (que_read_cnt == 0 && que_write_cnt > 0) {
                        nogrp_cnt ++; 
                    } else if (que_read_cnt > 0) {
                        nogrp_cnt = 0;
                    }
                }
            }
            if (GetPerfQsize() >= PERF_ENGRP_LEVEL) {
                if (que_write_cnt >= PERF_CMD_WLEVELH || !has_read_cmd) {
                    state_trig = 1;
                    ser_sch_write = *wr_most_level[linked_ptcs_[0]->occ][state_trig - 1];
                    same_bank_cnt = MAP_CONFIG["BANK_CMD_TH"][state_trig - 1];
                    no_cmd_sch_th = MAP_CONFIG["NO_CMD_SCH_TH"][state_trig - 1];
                    max_rank = get_max_rank(false);
                    no_cmd_sch_cnt = 0;
                    no_rcmd_sch_cnt = 0;
                    serial_cmd_cnt = 0;
                    ser_read_cnt = 0;
                    ptc_empty_cnt = 0;
                    nogrp_cnt = 0;
                    wbuff_state = WBUFF_WRITE;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PERF_ST NG to WRITE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                                <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                                <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                                <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
                                <<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                                <<" wr_tout_cnt="<<wr_adtimeout_cnt<<" ser_read="<<ser_read_cnt<<" has_read_cmd="<<has_read_cmd
                                <<" has_write_cmd="<<has_write_cmd<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
                    }
                } else {
                    max_rank = get_max_rank(true);
                    no_cmd_sch_cnt = 0;
                    no_rcmd_sch_cnt = 0;
                    serial_cmd_cnt = 0;
                    ser_write_cnt = 0;
                    ptc_empty_cnt = 0;
                    nogrp_cnt = 0;
                    wbuff_state = WBUFF_IDLE;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PERF_ST NG to IDLE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                                <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                                <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                                <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
                                <<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                                <<" wr_tout_cnt="<<wr_adtimeout_cnt<<" ser_read="<<ser_read_cnt<<" has_read_cmd="<<has_read_cmd
                                <<" has_write_cmd="<<has_write_cmd<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
                    }
                }
            }
            break;
        }
        case WBUFF_IDLE : {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF_ST STATE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                        <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                        <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                        <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
                        <<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                        <<" wr_tout_cnt="<<wr_adtimeout_cnt<<" ser_read="<<ser_read_cnt
                        <<" has_read_cmd="<<has_read_cmd<<" has_write_cmd="<<has_write_cmd
                        <<" high_qos_trig="<<high_qos_trig<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
            }
            wbuff_state_pre = WBUFF_IDLE;
            if (GetPerfQsize() < PERF_EXGRP_LEVEL) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_ST IDLE to NG, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                            <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                            <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
							<<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                            <<" wr_tout_cnt="<<wr_adtimeout_cnt<<" ser_read="<<ser_read_cnt<<" has_read_cmd="<<has_read_cmd
                            <<" has_write_cmd="<<has_write_cmd<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
                }
                max_rank = get_max_rank(false);
                no_cmd_sch_cnt = 0;
                no_rcmd_sch_cnt = 0;
                serial_cmd_cnt = 0;
                ser_read_cnt = 0;
                ptc_empty_cnt = 0;
                nogrp_cnt = 0;
                wbuff_state = WBUFF_NO_GROUP;
            } else if (has_wr_tout && !has_rd_tout && TOUT_FORCE_RWGRP_EN) {
                if (state_trig == 0) {
                    state_trig = 1;
                }
                ser_sch_write = *wr_most_level[linked_ptcs_[0]->occ][state_trig - 1];
                same_bank_cnt = MAP_CONFIG["BANK_CMD_TH"][state_trig - 1];
                no_cmd_sch_th = MAP_CONFIG["NO_CMD_SCH_TH"][state_trig - 1];
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_ST IDLE to WRITE (TOUT), WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                            <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                            <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
							<<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                            <<" has_wr_tout="<<has_wr_tout<<" has_rd_tout="<<has_rd_tout<<endl);
                }
                max_rank = get_max_rank(false);
                no_cmd_sch_cnt = 0;
                no_rcmd_sch_cnt = 0;
                serial_cmd_cnt = 0;
                ser_read_cnt = 0;
                ptc_empty_cnt = 0;
                nogrp_cnt = 0;
                sch_level_cnt[state_trig] ++;
                wbuff_state = WBUFF_WRITE;
            } else if (((wbuff_state_gap == 0 && state_trig != 0) || (!has_read_cmd && PERF_FAST_R2W_EN) || (ptc_empty_cnt > PTC_EMPTY_RD_TH)) && has_write_cmd && !high_qos_trig) {
                if (state_trig == 0) {
                    state_trig = 1;
                }
                ser_sch_write = *wr_most_level[linked_ptcs_[0]->occ][state_trig - 1];
                same_bank_cnt = MAP_CONFIG["BANK_CMD_TH"][state_trig - 1];
                no_cmd_sch_th = MAP_CONFIG["NO_CMD_SCH_TH"][state_trig - 1];
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_ST IDLE to WRITE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                            <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                            <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
							<<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                            <<" wr_tout_cnt="<<wr_adtimeout_cnt<<" ser_read="<<ser_read_cnt
                            <<" has_read_cmd="<<has_read_cmd<<" has_write_cmd="<<has_write_cmd
                            <<" high_qos_trig="<<high_qos_trig<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
                }
                max_rank = get_max_rank(false);
                no_cmd_sch_cnt = 0;
                no_rcmd_sch_cnt = 0;
                serial_cmd_cnt = 0;
                ser_read_cnt = 0;
                ptc_empty_cnt = 0;
                nogrp_cnt = 0;
                sch_level_cnt[state_trig] ++;
                wbuff_state = WBUFF_WRITE;
            }
            break;
        }
        case WBUFF_WRITE : {
            wbuff_state_pre = WBUFF_WRITE;
            if (GetPerfQsize() < PERF_EXGRP_LEVEL) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_ST WRITE to NG, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                            <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                            <<"] availability="<<linked_ptcs_[0]->availability<<" state_trig="<<state_trig
							<<" ser_sch_write="<<ser_sch_write<<" no_cmd_sch_th="<<no_cmd_sch_th
                            <<" wr_tout_cnt="<<wr_adtimeout_cnt<<" ser_read="<<ser_read_cnt<<" has_read_cmd="<<has_read_cmd
                            <<" has_write_cmd="<<has_write_cmd<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
                }
                max_rank = get_max_rank(false);
                no_cmd_sch_cnt = 0;
                no_rcmd_sch_cnt = 0;
                serial_cmd_cnt = 0;
                ser_read_cnt = 0;
                ptc_empty_cnt = 0;
                nogrp_cnt = 0;
                wbuff_state = WBUFF_NO_GROUP;
            } else if (has_rd_tout && !has_wr_tout && TOUT_FORCE_RWGRP_EN) {
                if (state_trig == 5) wbuff_state_gap = 4; // 4 is rtl gap
                else wbuff_state_gap = 4 * GAP_CNT_BASE + ser_write_cnt * pow(2,GAP_CNT_TIMES);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_ST WRITE to IDLE (TOUT), WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                            <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                            <<"] availability="<<linked_ptcs_[0]->availability<<" ser_write="<<ser_write_cnt
                            <<" no_cmd_sch="<<no_cmd_sch_cnt<<" state_trig="<<state_trig
                            <<" wbuff_state_gap="<<wbuff_state_gap<<" has_rd_tout="<<has_rd_tout
                            <<" has_wr_tout="<<has_wr_tout<<endl);
                }
                max_rank = get_max_rank(true);
                no_cmd_sch_cnt = 0;
                no_rcmd_sch_cnt = 0;
                serial_cmd_cnt = 0;
                ser_write_cnt = 0;
                ptc_empty_cnt = 0;
                nogrp_cnt = 0;
                wbuff_state = WBUFF_IDLE;
            } else if (((((ser_write_cnt >= ser_sch_write)||(no_cmd_sch_cnt > no_cmd_sch_th))&&(check_wr_level()==0)) 
                        || (!has_write_cmd) || (ptc_empty_cnt > PTC_EMPTY_WR_TH) || high_qos_trig) && has_read_cmd) {
                if (high_qos_trig && has_read_cmd) perf_highqos_trig_grpsw_cnt ++;
                if (state_trig == 5) wbuff_state_gap = 4; // 4 is rtl gap
                else wbuff_state_gap = 4 * GAP_CNT_BASE + ser_write_cnt * pow(2,GAP_CNT_TIMES);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PERF_ST WRITE to IDLE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<" rel_wcmd_cnt="<<rel_wcmd_cnt<<" rel_rcmd_cnt="<<rel_rcmd_cnt
                            <<"] DMC[R:"<<linked_ptcs_[0]->Read_Cnt()<<" W:"<<linked_ptcs_[0]->Write_Cnt()
                            <<"] availability="<<linked_ptcs_[0]->availability<<" ser_write="<<ser_write_cnt
                            <<" no_cmd_sch="<<no_cmd_sch_cnt<<" state_trig="<<state_trig
                            <<" wbuff_state_gap="<<wbuff_state_gap<<" wr_tout_cnt="<<wr_adtimeout_cnt
                            <<" has_read_cmd="<<has_read_cmd<<" has_write_cmd="<<has_write_cmd
                            <<" high_qos_trig="<<high_qos_trig<<" ptc_emp_cnt="<<ptc_empty_cnt<<endl);
                }
                max_rank = get_max_rank(true);
                no_cmd_sch_cnt = 0;
                no_rcmd_sch_cnt = 0;
                serial_cmd_cnt = 0;
                ser_write_cnt = 0;
                ptc_empty_cnt = 0;
                nogrp_cnt = 0;
                wbuff_state = WBUFF_IDLE;
            }
            break;
        }
        default:{
            break;
        }
    }
}

unsigned PFQ::get_max_rank(bool isRd) {
    unsigned maxrnk = 0;
    unsigned maxrnk_cnt = 0;
    maxrnk_cnt = isRd ? rb_rank_cnt[0] : wb_rank_cnt[0];
    for (size_t i = 0; i < NUM_RANKS; i++) {
        unsigned rnk_cnt = isRd ? rb_rank_cnt[i] : wb_rank_cnt[i];
        if (rnk_cnt > maxrnk_cnt) {
            maxrnk_cnt = rnk_cnt;
            maxrnk = i;
        }
    }
    return maxrnk;
}

/***************************************************************************************************
Descriptor: update_rwgroup_state& --old verision(deleted)
****************************************************************************************************/

/***************************************************************************************************
Descriptor: update_rwgroup_state& --old verision(deleted)
****************************************************************************************************/
// 更新SID分组状态（应在每次调度周期或队列变化时调用）
void PFQ::update_sidgroup_state(unsigned rank) {
    if (!SIMPLE_GRP_SID_EN) return;

    // 当前流水线深度取第一个元素（假设流水线从索引0开始）
    int cur_state = sid_group_state.empty() ? NO_LR_GROUP : sid_group_state[0];
    unsigned cur_target_sid = quc_slt_grp_lr.empty() ? 0 : quc_slt_grp_lr[0];

    // 如果当前处于写分组（借用wbuff_state判断，根据实际调整）
    if (wbuff_state == WBUFF_WRITE) { // in_write_group需在PFQ中定义或根据wbuff_state判断
        // 写操作时清空SID分组状态
        sid_group_state.push_back(NO_LR_GROUP);
        pre_quc_slt_grp_lr.push_back(quc_slt_grp_lr.back());
        quc_slt_grp_lr.push_back(4);          // 4表示无特定SID
        pre_quc_slt_grp_lr.push_back(4);
        serial_sid_cnt[rank] = 0;
        sidgrp_ch_cmd_cnt[rank] = 0;
        return;
    }

    // 检查其他SID是否有命令（用于切换条件）
    bool has_cmd = false;
    for (unsigned sid = 0; sid < NUM_SIDS; ++sid) {
        if (cur_state == LR_GROUP && in_sid_group == sid && cur_target_sid == sid) continue;
        if (linked_ptcs_[0]->r_sid_cnt[rank][sid] > 0) {
            has_cmd = true;
            break;
        }
    }

    // 检查是否有SID超时
    bool has_sid_timeout = false;
    unsigned sid_tout = 4;
    for (unsigned sid = 0; sid < NUM_SIDS; ++sid) {
        if (cur_state == LR_GROUP && in_sid_group == sid && cur_target_sid == sid) continue;
        if (linked_ptcs_[0]->r_sid_cnt[rank][sid] == 0) continue;
        if (sid_timeout[rank][sid] >= TIMEOUT_SID) {
            has_sid_timeout = true;
            sid_tout = sid;
            break;
        }
    }

    // 检查地址冲突（遍历PerfQue）
    bool has_no_addr_conf = false; // true表示无冲突，false表示有冲突
    for (auto &trans : PerfQue) {
        if (trans == nullptr) continue;
        if (cur_state == NO_LR_GROUP) continue;
        if (cur_target_sid != trans->sid) continue;
        if (trans->transactionType != DATA_READ) continue;
        if (trans->conflict_state->perf.ad_conf_cnt > 0) continue;
        has_no_addr_conf = true;
        break;
    }

    // 检查是否有有效读命令（用于切换条件）
    bool has_valid_cmd = false;
    for (auto &trans : PerfQue) {
        if (trans == nullptr) continue;
        if (cur_state == NO_LR_GROUP) continue;
        if (cur_target_sid != trans->sid) continue;
        if (trans->conflict_state->perf.ad_conf_cnt > 0) continue;
        if (trans->pre_act) continue;           // 预激活未完成
        if (now() < trans->arb_time) continue;   // 未到仲裁时间
        if (linked_ptcs_[0]->refreshALL[trans->rank][0].refreshing) continue;
        if (trans->bp_by_tout) continue;
        if (trans->transactionType == DATA_READ) {
            has_valid_cmd = true;
            break;
        }
    }

    // 检查其他SID组是否有大量请求（用于切换条件）
    bool has_other_sidgroup = false;
    for (unsigned sid = 0; sid < NUM_SIDS; ++sid) {
        if (cur_state == NO_LR_GROUP) continue;
        if (cur_target_sid == sid) continue;
        if (linked_ptcs_[0]->r_sid_cnt[rank][sid] >= LR_LEVELH) {
            has_other_sidgroup = true;
            break;
        }
    }

    // 计算普通切换请求
    bool lr_switch_req = ((serial_sid_cnt[rank] >= SERIAL_LR_LEVELL) &&
                          ((linked_ptcs_[0]->r_sid_cnt[rank][cur_target_sid] <= LR_LEVELL) || has_other_sidgroup))
                          || (serial_sid_cnt[rank] >= SERIAL_LR_LEVELH)
                          || (!has_valid_cmd);

    unsigned que_read_cnt = rcmd_cnt + rel_rcmd_cnt; // 读队列总长度（需根据PFQ现有变量定义）

    // 状态机处理
    switch (cur_state) {
        case NO_LR_GROUP: {
            if (que_read_cnt >= ENGRP_LR_LEVEL) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10) << now() << " -- GRP_CHG :: NG to LRG, que_read_cnt=" << que_read_cnt
                           << ", max_sid=" << get_max_sid(rank) << ", in_sid_group=" << in_sid_group
                           << ", quc_slt_grp_lr=" << cur_target_sid
                           << ", serial_sid_cnt=" << serial_sid_cnt[rank]
                           << ", sidgrp_ch_cmd_cnt=" << sidgrp_ch_cmd_cnt[rank] << endl);
                }
                sid_group_state.push_back(LR_GROUP);
                pre_quc_slt_grp_lr.push_back(cur_target_sid);
                quc_slt_grp_lr.push_back(get_max_sid(rank));
                in_sid_group = 4; // 过渡态
            }
            break;
        }
        case LR_GROUP: {
            if (que_read_cnt < EX_LR_LEVEL) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10) << now() << " -- GRP_CHG :: LRG to NG (LEVELL), que_read_cnt=" << que_read_cnt
                           << ", max_sid=" << get_max_sid(rank) << ", in_sid_group=" << in_sid_group
                           << ", quc_slt_grp_lr=" << cur_target_sid
                           << ", serial_sid_cnt=" << serial_sid_cnt[rank]
                           << ", sidgrp_ch_cmd_cnt=" << sidgrp_ch_cmd_cnt[rank] << endl);
                }
                sid_group_state.push_back(NO_LR_GROUP);
                pre_quc_slt_grp_lr.push_back(cur_target_sid);
                quc_slt_grp_lr.push_back(get_max_sid(rank));
                in_sid_group = 4;
                serial_sid_cnt[rank] = 0;
                sidgrp_ch_cmd_cnt[rank] = 0;
            } else if (lr_switch_req && has_cmd) {
                // 普通切换
                if (DEBUG_BUS) {
                    PRINTN(setw(10) << now() << " -- GRP_CHG :: NORMAL LRG, que_read_cnt=" << que_read_cnt
                           << ", max_sid=" << get_max_sid(rank) << ", in_sid_group=" << in_sid_group
                           << ", quc_slt_grp_lr=" << cur_target_sid
                           << ", serial_sid_cnt=" << serial_sid_cnt[rank]
                           << ", sidgrp_ch_cmd_cnt=" << sidgrp_ch_cmd_cnt[rank]
                           << ", has_valid_cmd=" << has_valid_cmd
                           << ", has_other_sidgroup=" << has_other_sidgroup << endl);
                }
                sid_group_state.push_back(LR_GROUP);
                pre_quc_slt_grp_lr.push_back(cur_target_sid);
                quc_slt_grp_lr.push_back(get_max_sid(rank));
                in_sid_group = 4;
                serial_sid_cnt[rank] = 0;
                sidgrp_ch_cmd_cnt[rank] = 0;
            } else if (has_sid_timeout && TIMEOUT_SID_ENABLE && has_cmd) {
                // 超时切换
                if (DEBUG_BUS) {
                    PRINTN(setw(10) << now() << " -- GRP_CHG :: TIMEOUT LRG, que_read_cnt=" << que_read_cnt
                           << ", max_sid=" << get_max_sid(rank) << ", in_sid_group=" << in_sid_group
                           << ", quc_slt_grp_lr=" << cur_target_sid
                           << ", serial_sid_cnt=" << serial_sid_cnt[rank]
                           << ", sidgrp_ch_cmd_cnt=" << sidgrp_ch_cmd_cnt[rank]
                           << ", sid_timeout=" << sid_tout
                           << ", sid_tout_prd=" << sid_timeout[rank][sid_tout] << endl);
                }
                sid_group_state.push_back(LR_GROUP);
                pre_quc_slt_grp_lr.push_back(cur_target_sid);
                quc_slt_grp_lr.push_back(sid_tout);
                in_sid_group = 4;
                serial_sid_cnt[rank] = 0;
                sidgrp_ch_cmd_cnt[rank] = 0;
            } else if (!has_no_addr_conf) {
                // 地址冲突，退出分组
                if (DEBUG_BUS) {
                    PRINTN(setw(10) << now() << " -- GRP_CHG :: LRG to NG (ADDR), que_read_cnt=" << que_read_cnt
                           << ", max_sid=" << get_max_sid(rank) << ", in_sid_group=" << in_sid_group
                           << ", quc_slt_grp_lr=" << cur_target_sid
                           << ", serial_sid_cnt=" << serial_sid_cnt[rank]
                           << ", sidgrp_ch_cmd_cnt=" << sidgrp_ch_cmd_cnt[rank] << endl);
                }
                sid_group_state.push_back(NO_LR_GROUP);
                pre_quc_slt_grp_lr.push_back(cur_target_sid);
                quc_slt_grp_lr.push_back(get_max_sid(rank));
                in_sid_group = 4;
                serial_sid_cnt[rank] = 0;
                sidgrp_ch_cmd_cnt[rank] = 0;
            } else if ((in_sid_group == 4) && (sidgrp_ch_cmd_cnt[rank] >= SERIAL_PRE_SIDGRP)) {
                // 实际生效切换
                if (DEBUG_BUS) {
                    PRINTN(setw(10) << now() << " -- REAL_SID_CHG :: que_read_cnt=" << que_read_cnt
                           << ", max_sid=" << get_max_sid(rank) << ", in_sid_group=" << in_sid_group
                           << ", quc_slt_grp_lr=" << cur_target_sid
                           << ", serial_sid_cnt=" << serial_sid_cnt[rank]
                           << ", sidgrp_ch_cmd_cnt=" << sidgrp_ch_cmd_cnt[rank] << endl);
                }
                in_sid_group = cur_target_sid;
                sidgrp_ch_cmd_cnt[rank] = 0;
            }
            break;
        }
        default:
            ERROR("Unknown SID group state");
            assert(0);
    }

    // 维护流水线：确保长度不超过SID_GRP_PIPE
    if (sid_group_state.size() > SID_GRP_PIPE) {
        sid_group_state.erase(sid_group_state.begin());
        quc_slt_grp_lr.erase(quc_slt_grp_lr.begin());
        pre_quc_slt_grp_lr.erase(pre_quc_slt_grp_lr.begin());
    } else {
        sid_group_state.push_back(sid_group_state.back());
        sid_group_state.erase(sid_group_state.begin());
        quc_slt_grp_lr.push_back(quc_slt_grp_lr.back());
        quc_slt_grp_lr.erase(quc_slt_grp_lr.begin());
        pre_quc_slt_grp_lr.push_back(pre_quc_slt_grp_lr.back());
        pre_quc_slt_grp_lr.erase(pre_quc_slt_grp_lr.begin());
    }
}

unsigned PFQ::get_max_sid(unsigned rank) {
    unsigned max_cnt = 0, max_sid = 0;
    for (unsigned sid = 0; sid < NUM_SIDS; ++sid) {
        if (linked_ptcs_[0]->r_sid_cnt[rank][sid] >= max_cnt) {
            max_cnt = linked_ptcs_[0]->r_sid_cnt[rank][sid];
            max_sid = sid;
        }
    }
    return max_sid;
}
/***************************************************************************************************
Descriptor: update_rnkgroup_state&
****************************************************************************************************/
void PFQ::update_rnkgroup_state() {
    if (!PERF_RANK_WMERGE_EN && !PERF_RANK_RMERGE_EN) return;
    bool has_rank_rcmd = false;
    bool has_rank_wcmd = false;
    bool has_other_rank_rcmd = false;
    bool has_other_rank_wcmd = false;
    unsigned nxt_rrank = 0;
    unsigned nxt_wrank = 0;
    for (auto& trans : PerfQue) {
        if (trans == nullptr) continue;
        bool is_cur_rank_read = (trans->transactionType == DATA_READ) && (trans->rank == perf_sch_rrank);
        bool is_cur_rank_write = (trans->transactionType == DATA_WRITE) && (trans->rank == perf_sch_wrank);
        bool is_other_rank_read = (trans->transactionType == DATA_READ) && (trans->rank != perf_sch_rrank);
        bool is_other_rank_write = (trans->transactionType == DATA_WRITE) && (trans->rank != perf_sch_wrank);
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / (linked_ptcs_[ptcID(trans->channel)]->sc_bank_num);
        if (!is_cur_rank_read && !is_cur_rank_write && !is_other_rank_read && !is_other_rank_write) continue;
        if (trans->perf_addrconf) continue;
        if (now() < trans->arb_time) continue;
        if (linked_ptcs_[ptcID(trans->channel)]->refreshALL[trans->rank][sub_channel].refreshing) continue;
        if (linked_ptcs_[ptcID(trans->channel)]->refreshPerBank[trans->bankIndex].refreshing) continue;
        if (trans->bp_by_tout) continue;
        if (is_cur_rank_read) {
            has_rank_rcmd = true;
        } else if (is_cur_rank_write) {
            if (trans->data_ready_cnt == (trans->burst_length + 1)) {
                has_rank_wcmd = true;
            }
        }
        if (is_other_rank_read) {
            has_other_rank_rcmd = true;
            nxt_rrank = trans->rank;
        } else if (is_other_rank_write) {
            if (trans->data_ready_cnt == (trans->burst_length + 1)) {
                has_other_rank_wcmd = true;
                nxt_wrank = trans->rank;
            }
        }     
        if (has_rank_rcmd && has_rank_wcmd && has_other_rank_rcmd && has_other_rank_wcmd) break;
    }

    unsigned que_read_cnt = rcmd_cnt + rel_rcmd_cnt;
    unsigned que_write_cnt = wcmd_cnt + rel_wcmd_cnt; 
    switch (wbuff_rnkgrp_state) {
        case WBUFF_NO_RNK_GROUP: {
            perf_rd_in_rnkgrp = false;
            perf_wr_in_rnkgrp = false;
            if (que_read_cnt >= PERF_RRNK_ENGRP_LEVEL && PERF_RANK_RMERGE_EN && wbuff_state==WBUFF_IDLE) {
                perf_rd_in_rnkgrp = true;
                perf_sch_rrank = get_max_rank(true);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- R_RNKGRP_ENTER :: que_read_cnt="<<que_read_cnt<<", rnk_state="<<wbuff_rnkgrp_state
                            <<" perf_sch_rrank="<<perf_sch_rrank<<endl);
                }
            }
            if (que_write_cnt >= PERF_WRNK_ENGRP_LEVEL && PERF_RANK_WMERGE_EN && wbuff_state==WBUFF_WRITE) {
                perf_wr_in_rnkgrp = true;           
                perf_sch_wrank = get_max_rank(false);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- W_RNKGRP_ENTER :: que_write_cnt="<<que_write_cnt<<", rnk_state="<<wbuff_rnkgrp_state
                            <<" perf_sch_wrank="<<perf_sch_wrank<<endl);
                }
            }
            if (perf_rd_in_rnkgrp || perf_wr_in_rnkgrp) {
                wbuff_rnkgrp_state = WBUFF_IN_RNK_GROUP;
            }
            break;
        }
        case WBUFF_IN_RNK_GROUP: {
            switch (wbuff_state) {
                case WBUFF_IDLE: {
                    if (!perf_rd_in_rnkgrp) {
                        wbuff_rnkgrp_state = WBUFF_NO_RNK_GROUP;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- R_RNKGRP_EXIT0 :: que_read_cnt="<<que_read_cnt<<", wbuff_state="<<wbuff_state
                                    <<", wbuff_state_pre="<<wbuff_state_pre<<endl);
                        }
                        break;
                    }
                    if (wbuff_state_pre == WBUFF_NO_GROUP || wbuff_state_pre == WBUFF_WRITE) {
                        if (((unsigned)accumulate(rank_cmd_high_qos.begin(), rank_cmd_high_qos.end(), 0) == 0)
                                || ((unsigned)accumulate(rank_cmd_high_qos.begin(), rank_cmd_high_qos.end(), 0) == NUM_RANKS)) {  // no rank/all ranks meet hqos rank conditions
                            perf_sch_rrank = get_max_rank(true);
                        } else {  // select rank which meet hqos rank conditions
                            for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                                if (rank_cmd_high_qos[rank]) {
                                    perf_sch_rrank = rank;
                                    break;
                                }
                            }
                        }
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- R_RNKGRP_START :: perf_sch_rrank="<<perf_sch_rrank<<" hqos_state="<<rank_cmd_high_qos[perf_sch_rrank]<<endl);
                        }
                    } else if (wbuff_state_pre == WBUFF_IDLE) {
                        if (que_read_cnt < PERF_RRNK_EXGRP_LEVEL) {
                            perf_rd_in_rnkgrp = false;
                            wbuff_rnkgrp_state = WBUFF_NO_RNK_GROUP;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- R_RNKGRP_EXIT1 :: que_read_cnt="<<que_read_cnt<<", wbuff_state="<<wbuff_state
                                        <<", wbuff_state_pre="<<wbuff_state_pre<<endl);
                            }
                        } else if (!rank_cmd_high_qos[perf_sch_rrank] && rank_cmd_high_qos[nxt_rrank]) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- R_RNKGRP_CHG_HQOS :: rank"<<perf_sch_rrank<<" to rank"<<nxt_rrank<<endl);
                            }
                            perf_sch_rrank = nxt_rrank;
                        } else if (!has_rank_rcmd && has_other_rank_rcmd) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- R_RNKGRP_CHG_CMD :: rank"<<perf_sch_rrank<<" to rank"<<nxt_rrank<<endl);
                            }
                            perf_sch_rrank = nxt_rrank;
                        } else if ((no_rcmd_sch_cnt>PERF_READ_SCH_THH) && rrel_rank_cnt[perf_sch_rrank] < PERF_SCH_RRANK_LEVELH && has_other_rank_rcmd && rb_rank_cnt[nxt_rrank]>PERF_HOLD_RRANK_LEVELH) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- R_RNKGRP_CHG_H :: rank"<<perf_sch_rrank<<" to rank"<<nxt_rrank<<endl);
                            }
                            perf_sch_rrank = nxt_rrank;
                        } else if ((no_rcmd_sch_cnt>PERF_READ_SCH_THL) && rrel_rank_cnt[perf_sch_rrank] < PERF_SCH_RRANK_LEVELL && has_other_rank_rcmd && rb_rank_cnt[nxt_rrank]>PERF_HOLD_RRANK_LEVELL) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- R_RNKGRP_CHG_L :: rank"<<perf_sch_rrank<<" to rank"<<nxt_rrank<<endl);
                            }
                            perf_sch_rrank = nxt_rrank;
                        }
                    }                  
                    perf_wr_in_rnkgrp = que_write_cnt >= PERF_WRNK_ENGRP_LEVEL && PERF_RANK_WMERGE_EN;           
                    break;
                }
                case WBUFF_WRITE: {
                    if (!perf_wr_in_rnkgrp) {
                        wbuff_rnkgrp_state = WBUFF_NO_RNK_GROUP;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- W_RNKGRP_EXIT0 :: que_write_cnt="<<que_write_cnt<<", wbuff_state="<<wbuff_state
                                    <<", wbuff_state_pre="<<wbuff_state_pre<<endl);
                        }
                        break;
                    }
                    if (wbuff_state_pre == WBUFF_NO_GROUP || wbuff_state_pre == WBUFF_IDLE) {
                        perf_sch_wrank = get_max_rank(false);
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- WGRP_START :: perf_sch_wrank="<<perf_sch_wrank<<endl);
                        }
                    } else if (wbuff_state_pre == WBUFF_WRITE) {
                        if (que_write_cnt < PERF_WRNK_EXGRP_LEVEL) {
                            perf_wr_in_rnkgrp = false;
                            wbuff_rnkgrp_state = WBUFF_NO_RNK_GROUP;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- W_RNKGRP_EXIT1 :: que_write_cnt="<<que_write_cnt<<", wbuff_state="<<wbuff_state
                                        <<", wbuff_state_pre="<<wbuff_state_pre<<endl);
                            }
                        } else if (!has_rank_wcmd && has_other_rank_wcmd) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- W_RNKGRP_CHG_CMD :: rank"<<perf_sch_wrank<<" to rank"<<nxt_wrank<<endl);
                            }
                            perf_sch_wrank = nxt_wrank;
                        } else if ((no_cmd_sch_cnt>PERF_WRITE_SCH_THH) && wrel_rank_cnt[perf_sch_wrank] < PERF_SCH_WRANK_LEVELH && has_other_rank_wcmd && wb_rank_cnt[nxt_wrank]>PERF_HOLD_WRANK_LEVELH) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- W_RNKGRP_CHG_H :: rank"<<perf_sch_wrank<<" to rank"<<nxt_wrank<<endl);
                            }
                            perf_sch_wrank = nxt_wrank;
                        } else if ((no_cmd_sch_cnt>PERF_WRITE_SCH_THL) && wrel_rank_cnt[perf_sch_wrank] < PERF_SCH_WRANK_LEVELL && has_other_rank_wcmd && wb_rank_cnt[nxt_wrank]>PERF_HOLD_WRANK_LEVELL) {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- W_RNKGRP_CHG_L :: rank"<<perf_sch_wrank<<" to rank"<<nxt_wrank<<endl);
                            }
                            perf_sch_wrank = nxt_wrank;
                        }
                    }
                    perf_rd_in_rnkgrp = que_read_cnt >= PERF_RRNK_ENGRP_LEVEL && PERF_RANK_RMERGE_EN;           
                    break;
                }
                case WBUFF_NO_GROUP: {
                    break;                     
                }
            }
            break;
        }
    }
}

/***************************************************************************************************
Descriptor: update_data_path&
****************************************************************************************************/
void PFQ::update_rpsram_state() {

    // 1.Update backpressure signal status
    rmw_write_bp_this_cycle = rmw_write_bp_next_cycle;
    if (rmw_write_bp_next_cycle) {
        rmw_write_bp_next_cycle = false;
    }
    // 2.Process RMW data flow
    process_rmw_data_flow();
    // 3.Process read data from PTC with delay==0
    bool sys_bp = false;
    bool rpfifo_write_this_cycle = false;
    if (rp_fifo.empty()) {
        bool exit = false;
        for (unsigned i = 0; i < (NUM_CHANS / NUM_PFQS) && !exit; i++) {
            auto& read_data_buffer = linked_ptcs_[i]->read_data_buffer;
            for (auto it = read_data_buffer.begin(); it != read_data_buffer.end();) {
                if (it->delay == 0) {
                    uint64_t task = it->task;
                    auto pending_it = linked_ptcs_[i]->pending_TransactionQue.find(task);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- T_SYSC :: Issuing to SYSC, task="<<task<<endl);
                    }
                    if (pending_it == linked_ptcs_[i]->pending_TransactionQue.end()) {
                        uint32_t ch = getID() * (NUM_CHANS / NUM_PFQS) + i;
                        ERROR(setw(10)<<now()<<" -- [DMC channel="<<ch<<"] mismatch read data, task="<<task);
                        assert(0);
                    }

                    TRANS_MSG& msg = pending_it->second;
                    msg.burst_cnt++;
                    if (IS_HBM3) {
                        msg.burst_cnt++;
                    }
                    it->channel = msg.channel;
                    it->cnt = msg.burst_cnt;
                    it->is_last = (msg.burst_cnt == (msg.burst_length + 1));
#ifdef SYSARCH_PLATFORM
                    unsigned rdata_type = 0;
                    if (msg.burst_cnt == (msg.burst_length + 1)) rdata_type |= 1; // bit[0] is rdata_end
                    if (msg.burst_cnt == 0) rdata_type |= (1 << 1); // bit[1] is rdata_start
                    rdata_type |= (msg.qos << 2); // bit[5:2] is qos
                    rdata_type |= (msg.pf_type << 6); // bit[7:6] is pf_type
                    rdata_type |= (msg.sub_pftype << 8); // bit[11:8] is pf_type
                    rdata_type |= (msg.sub_src << 12); // bit[13:12] is pf_type
                    msg.reqAddToDmcTime = double(rdata_type);
#endif
                    if (it->mask_wcmd && RMW_ENABLE_PERF && IS_LP6) {
                        if (rp_fifo.empty() && !rmw_rdata_fifo_full) {
                            rmw_rdata_fifo.push_back(*it);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- RMW_Data Bybass RPFIFO to RMWFIFO, task="<<task<<" burst_cnt="<<it->cnt<<endl);
                            }
                        } else {
                            rp_fifo.push_back(*it);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- RMW_Data To RPFIFO, task="<<task<<" burst_cnt="<<it->cnt<<endl);
                            }
                        }
                    } else {
                        if ((!IECC_ENABLE || !linked_ptcs_[ptcID(it->channel)]->tasks_info[task].rd_ecc) && (!RMW_ENABLE || (!it->mask_wcmd && RMW_ENABLE))) {
                            it->readDataEnterDmcTime = now() * tDFI;
                            sys_bp = !returnReadDataResp(it->channel, task, it->readDataEnterDmcTime, msg.reqAddToDmcTime, msg.reqEnterDmcBufTime);
                            if (sys_bp) {                              
                                if (DEBUG_BUS) {
                                    PRINTN(setw(10)<<now()<<" -- Rdata Back Pressure :: task="<<task<<" ch="<<it->channel<<endl);
                                }
                                if (!RPFIFO_EN) {
                                    return;
                                } else {
                                    rp_fifo.push_back(*it);
                                    rpfifo_write_this_cycle = true;
                                    if (DEBUG_BUS) {
                                        PRINTN(setw(10)<<now()<<" -- Rdata to rp_fifo, task="<<task<<", rp_fifo_size="<<rp_fifo.size()<<endl);
                                    }
                                }
                            } else {
                                linked_ptcs_[i]->pre_rdata_time = now();
                                linked_ptcs_[i]->rdata_cnt ++;                              
                                if (IS_HBM3) {
                                    linked_ptcs_[i]->rdata_cnt ++;
                                }
                                if (PRINT_IDLE_LAT) {
                                    DEBUG(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
                                            <<ceil(((it->readDataEnterDmcTime - msg.reqEnterDmcBufTime) / tDFI)));
                                }
                                if (DEBUG_BUS) {
                                    PRINTN(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
                                            <<ceil(((it->readDataEnterDmcTime - msg.reqEnterDmcBufTime) / tDFI))<<endl);
                                }
                            }
                        }
                        if (!sys_bp) {
                            if (it->is_last) {
                                linked_ptcs_[i]->ReturnData_statistics(task, msg.time, msg.qos, msg.mid, msg.pf_type, msg.rank);
                                // return latency
                                linked_ptcs_[i]->pending_TransactionQue.erase(task);
                                if (IECC_ENABLE) linked_ptcs_[ptcID(it->channel)]->tasks_info[task].rd_finish = true;
                                if (DEBUG_BUS) {
                                    double latency = ceil(((it->readDataEnterDmcTime / tDFI) - msg.reqEnterDmcBufTime));
                                    PRINTN(setw(10)<<now()<<" -- READ_COMPLETE :: task="<<task<<" latency="<<latency<<" cycles"<<endl);
                                }
                            } else {
                                pending_it->second.burst_cnt = msg.burst_cnt;
                            }
                        }
                    }
                    it = read_data_buffer.erase(it);
                    exit = true;
                    break;
                } else {
                    ++it;
                }                
            }
        }
    } else { // 4.Send main path data to upstream
        if (!rpfifo_write_this_cycle) {
            bool exit = false;
            for (unsigned i = 0; i < (NUM_CHANS / NUM_PFQS) && !exit; i++) {
                auto& read_data_buffer = linked_ptcs_[i]->read_data_buffer;
                for (auto it = read_data_buffer.begin(); it != read_data_buffer.end();) {
                    if (it->delay == 0) {
                        auto pending_it = linked_ptcs_[i]->pending_TransactionQue.find(it->task);
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- T_RDS :: Issuing to RDS, task="<<it->task<<endl);
                        }
                        if (pending_it == linked_ptcs_[i]->pending_TransactionQue.end()) {
                            uint32_t ch = getID() * (NUM_CHANS / NUM_PFQS) + i;
                            ERROR(setw(10)<<now()<<" -- [DMC channel="<<ch<<"] mismatch read data, task="<<it->task);
                            assert(0);
                        }
                        TRANS_MSG& msg = pending_it->second;
                        msg.burst_cnt++;
                        if (IS_HBM3) {
                            msg.burst_cnt++;
                        }
                        it->channel = msg.channel;
                        it->cnt = msg.burst_cnt;
                        it->is_last = (msg.burst_cnt == (msg.burst_length + 1));
                        if (rp_fifo.size() < RPFIFO_AMFULL_TH) {
                            rp_fifo.push_back(*it);
                            rpfifo_write_this_cycle = true;
                            it = read_data_buffer.erase(it);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- Rdata to rp_fifo, task="<<it->task<<", rpfifosize="<<rp_fifo.size()<<endl);
                            }
                        } else if (rp_fifo.size() < RPFIFO_DEPTH) {
                            rp_fifo.push_back(*it);
                            rpfifo_write_this_cycle = true;
                            rcmd_bp_byrp = true;
                            it = read_data_buffer.erase(it);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- Rcmd bp, Rdata to rp_fifo, task="<<it->task<<", rpfifosize="<<rp_fifo.size()<<endl);
                            }
                        }
                        exit = true;
                        break;
                    } else {
                        ++it;
                    }    
                }
            }
        }
        if (!rpfifo_write_this_cycle) {
            auto return_rdata = rp_fifo.front();
            uint64_t task = return_rdata.task;
            auto pending_it = linked_ptcs_[ptcID(return_rdata.channel)]->pending_TransactionQue.find(task);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- T_MSYSC :: Issuing to MSYSC, from rp_fifo, task="<<task<<endl);
            }
            if (pending_it == linked_ptcs_[ptcID(return_rdata.channel)]->pending_TransactionQue.end()) {
                ERROR(setw(10)<<now()<<" -- [DMC"<<return_rdata.channel<<"]"<<" mismatch data, task="<<task);
                assert(0);
            }

            TRANS_MSG msg = pending_it->second;
            msg.burst_cnt ++;
            if (IS_HBM3) {
                msg.burst_cnt ++;
            } 

#ifdef SYSARCH_PLATFORM
            unsigned rdata_type = 0;
            if (msg.burst_cnt == (msg.burst_length + 1)) rdata_type |= 1; // bit[0] is rdata_end
            if (msg.burst_cnt == 0) rdata_type |= (1 << 1); // bit[1] is rdata_start
            rdata_type |= (msg.qos << 2); // bit[5:2] is qos
            rdata_type |= (msg.pf_type << 6); // bit[7:6] is pf_type
            rdata_type |= (msg.sub_pftype << 8); // bit[11:8] is pf_type
            rdata_type |= (msg.sub_src << 12); // bit[13:12] is pf_type
            msg.reqAddToDmcTime = double(rdata_type);
#endif
            if (return_rdata.mask_wcmd && RMW_ENABLE_PERF && IS_LP6) {
                if (!rmw_rdata_fifo_full) {
                    rmw_rdata_fifo.push_back(return_rdata);
                    rp_fifo.pop_front();
                    rmw_bp_rdata_path = false;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- RMW_DATA from RPFIFO TO RMWFIFO, task="<<task<<" burst_cnt="<<return_rdata.cnt<<endl);
                    }
                } else {
                    rmw_bp_rdata_path = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- RMW_DATA backpress RPFIFO, task="<<task<<" burst_cnt="<<return_rdata.cnt<<endl);
                    }
                }
            } else {
                if ((!IECC_ENABLE || !linked_ptcs_[ptcID(return_rdata.channel)]->tasks_info[task].rd_ecc) && (!RMW_ENABLE || (!return_rdata.mask_wcmd && RMW_ENABLE))) {
                    return_rdata.readDataEnterDmcTime = now() * tDFI;
                    sys_bp = !returnReadDataResp(return_rdata.channel, task, return_rdata.readDataEnterDmcTime,
                            msg.reqAddToDmcTime, msg.reqEnterDmcBufTime);
                    if (sys_bp) {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- Rdata from rpfifo Back Pressure :: task="<<task<<" ch="<<return_rdata.channel<<endl);
                        }
                        return;
                    } else {
                        linked_ptcs_[ptcID(return_rdata.channel)]->pre_rdata_time = now();
                        linked_ptcs_[ptcID(return_rdata.channel)]->rdata_cnt ++;
                        if (IS_HBM3) {
                            linked_ptcs_[ptcID(return_rdata.channel)]->rdata_cnt ++;
                        }
                        if (PRINT_IDLE_LAT) {
                            DEBUG(setw(10)<<now()<<" -- Rdata Received from rpfifo :: task="<<task<<", latency="
                                    <<ceil(((return_rdata.readDataEnterDmcTime
                                    - msg.reqEnterDmcBufTime) / tDFI)));
                        }
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- Rdata Received from rpfifo:: task="<<task<<", latency="
                                    <<ceil(((return_rdata.readDataEnterDmcTime
                                    - msg.reqEnterDmcBufTime) / tDFI))<<endl);
                        }
                    }
                }
                if (!sys_bp) {
                    if (return_rdata.is_last) {
                        linked_ptcs_[ptcID(return_rdata.channel)]->ReturnData_statistics(task, msg.time, msg.qos, msg.mid, msg.pf_type, msg.rank);
                        // return latency
                        linked_ptcs_[ptcID(return_rdata.channel)]->pending_TransactionQue.erase(task);
                        if (IECC_ENABLE) linked_ptcs_[ptcID(return_rdata.channel)]->tasks_info[task].rd_finish = true;
                        if (DEBUG_BUS) {
                            double latency = ceil(((return_rdata.readDataEnterDmcTime / tDFI) - msg.reqEnterDmcBufTime));
                            PRINTN(setw(10)<<now()<<" -- READ_COMPLETE :: task="<<task<<" latency="<<latency<<" cycles"<<endl);
                        }
                    } else {
                        pending_it->second.burst_cnt = msg.burst_cnt;
                    }
                    
                    rp_fifo.pop_front();
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- Rdata from rp_fifo to upstream, task="<<task<<", rpfifosize="<<rp_fifo.size()<<endl);
                    }
                }
            }
        }
    }

    if (rp_fifo.empty()) {
        rcmd_bp_byrp = false;
    }
    
    // 5.Update FIFO status
    rmw_rdata_fifo_full = (rmw_rdata_fifo.size() >= RMW_RDATA_FIFO_DEPTH);
}

void PFQ::process_rmw_data_flow() {
    // 1.Process RMW data already in progress in rmw_rdata_fifo
    for (auto& rmw_data : rmw_rdata_fifo) {
        if (rmw_data.rmw_processing_started) {
            uint64_t cycles_passed = now() - rmw_data.rmw_processing_cycle;
            if (cycles_passed == CLKH2CLKL) {//modify
                //Stage 1:Read in progress
                rmw_data.rmw_stage = 1;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW_READ_STAGE1, task="<<rmw_data.task<< " stage=1 (rmw read wsram in progress)" <<endl);
                }
            } else if (cycles_passed == CLKH2CLKL*2) {
                //Stage 2:Data returned,merge and write back
                rmw_data.rmw_stage = 2;
                rmw_data.rmw_ready_for_wds = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW_READ_STAGE2, task="<<rmw_data.task<<" stage=2 (data returned, merge and write)"<<endl);
                }
            }
        }
    }
    // 2.Send ready RMW data to WDS (from front of queue)
    if (!rmw_rdata_fifo.empty()) {
        auto& front_rmw = rmw_rdata_fifo.front();
        if (front_rmw.rmw_ready_for_wds) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMW_SEND_TO_WDS, task="<<front_rmw.task<<" burst_cnt="<<front_rmw.cnt<<endl);
            }
            rmw_rdata_fifo.pop_front();
        }
    }
    // 3.Initiate wsram read for new RMW data in rmw_rdata_fifo
    if (!rmw_rdata_fifo.empty()) {
        for (auto& rmw_data : rmw_rdata_fifo) {
            if (!rmw_data.rmw_processing_started) {
                //Step 1:Check if wsram is being read by ptc this cycle
                bool wsram_busy_this_cycle = false;
                if (WSRAM_MAP_EN) {
                    for (unsigned i = 0; i < WSRAM_QUEUE_DEPTH; i++) {
                        if (wsram_slt[i].state == READING && now() >= wsram_slt[i].wdda) {
                            wsram_busy_this_cycle = true;
                            break;
                        }
                    }
                }
                //Step 2:If wsram not busy this cycle,initiate wsram read
                if (!wsram_busy_this_cycle) {
                    rmw_data.rmw_processing_started = true;
                    rmw_data.rmw_stage = 0;
                    rmw_data.rmw_processing_cycle = now();
                    //Step 3:Raise wsram_write_bp in next cycle
                    rmw_write_bp_next_cycle = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- RMW_READ_START, task="<<rmw_data.task<<" stage=0 (initiate read)"<<" bp_next_cycle="<<rmw_write_bp_next_cycle<<endl);
                    }
                    break;
                }
            }
        }
    }
}
 
void PFQ::wsram_release_wdata() {
    for (unsigned slot_idx = 0; slot_idx < WSRAM_QUEUE_DEPTH; slot_idx++) {
        if (wsram_slt[slot_idx].state != READING) continue;
        if (now() >= wsram_slt[slot_idx].wdda) {
            // if (wsram_slt[slot_idx].channel) ERROR
            linked_ptcs_[ptcID(wsram_slt[slot_idx].channel)]->receive_wdata(0, wsram_slt[slot_idx].task);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- Wsram released slt, slt_idx="<<slot_idx<<", task="<<wsram_slt[slot_idx].task
                <<", channel="<<wsram_slt[slot_idx].channel<<endl);
            }
            wsram_slt[slot_idx].state = SLT_IDLE;
            wsram_slt[slot_idx].channel = ~0ULL;
            wsram_slt[slot_idx].wdda = 0;
            wsram_slt[slot_idx].task = 0;
            used_wdata_que_cnt -= 1;
        }
    }
}

void PFQ::send_wdata() {
    if (!WdataToSend.empty() && !Perf_WdataSram.empty()) {
        
        if (!Perf_WdataSram.empty() && WdataToSend.empty()) {
            ERROR(setw(10)<<now()<<" No Wdata in Perf");
            assert(0);
        }

        bool find_wdata = false;
        unsigned task = Perf_WdataSram[0].task; 
        for (size_t i = 0; i < WdataToSend.size(); i++) {
            if (WdataToSend[i] != Perf_WdataSram[0].task) continue;
            linked_ptcs_[ptcID(Perf_WdataSram[0].channel)]->receive_wdata(0, task);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PERF_SEND_WDATA :: task="<<WdataToSend[i]<<" data_cnt="<<Perf_WdataSram[0].issue_length<<endl);
            }
            WdataToSend.erase(WdataToSend.begin() + i);
            Perf_WdataSram[0].issue_length ++;
            if (Perf_WdataSram[0].issue_length == (Perf_WdataSram[0].burst_length + 1)){
                Perf_WdataSram.erase(Perf_WdataSram.begin());
            }
            find_wdata = true;
            break;
        }

        if (!find_wdata) {
            ERROR(setw(10)<<now()<<" No Wdata in Perf, task="<<task);
            assert(0);
        }
    }
}

/*==================================================================================================
Descriptor: INTF FIELD 
===================================================================================================*/

/***************************************************************************************************
Descriptor: update cmd state in PFQ&
****************************************************************************************************/
void PFQ::perf_release_rd(uint64_t task, bool release_state) {
    assert(release_state);
    bool exist_task = false;
    for (auto it = PerfQue.begin(); it != PerfQue.end(); it++) {
        Transaction* r = *it;
        if (r == nullptr) continue;
        if (r->task != task) continue;
        if (r->transactionType != DATA_READ) {
            ERROR("Release Type Error!! task="<<task<<" type="<<r->transactionType);
            assert(0);
        }
        unsigned rank = r->rank;
        unsigned group = r->group;
        unsigned bankIndex = r->bankIndex;

        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PERF RELEASE R :: task="<<task<<" in_ptc="<<r->in_ptc
                    <<" fast_rd="<<r->fast_rd<<endl);
        }
        exist_task = true;
        if (!r->fast_rd) {
            rel_rcmd_cnt --;
            rrel_bank_cnt[rank][bankIndex % NUM_BANKS] --;
            rrel_bg_cnt[rank][group] --;
            rrel_rank_cnt[rank] --;
            rel_rank_cnt[rank] --;
        } else {
            rcmd_cnt --;
            rb_bank_cnt[rank][bankIndex % NUM_BANKS] --;
            rb_bg_cnt[rank][group] --;
            rb_rank_cnt[rank] --;
            rank_cnt[rank] --;
            rb_qos_cnt[r->qos] --;
            if (r->qos >= PERF_SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[rank] --;
            }
        }

        if (r->mask_wcmd && RMW_ENABLE_PERF && IS_LP6) {
            r->transactionType = DATA_WRITE;
            r->in_ptc = false;
            wcmd_set_conflict(r);
            wcmd_cnt ++;
            perf_bank_wcnt[r->bankIndex] ++;
            perf_bg_wcnt[r->rank][r->group] ++;
            wb_rank_cnt[r->rank] ++;
            wb_qos_cnt[r->qos] ++;
            wb_bank_cnt[r->rank][r->bankIndex % NUM_BANKS] ++;
            wb_bg_cnt[r->rank][r->group] ++;
            linked_ptcs_[ptcID(r->channel)]->bankStates[r->bankIndex].perf_bankwr_conflict = true;
            linked_ptcs_[ptcID(r->channel)]->bankStates[r->bankIndex].samebank_wcnt ++;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMW_CMD R2W_conversion completed for task="<<r->task<<endl);
            }
            rcmd_release_conflict(r);
        } else {
            rcmd_release_conflict(r);
            delete r->conflict_state;
            delete r;
            *it = nullptr;
        }
        break;
    }
    if (!exist_task) {
        ERROR("Release Task"<<task<<" But Not Exist!");
        assert(0);
    }
    return;
}

void PFQ::perf_release_wr(uint64_t task, bool release_state) {
    if (PERF_RELEASE_REC_WIN != 0) {
        ERROR("NOT SUPPORT WREC");
        assert(0);
        return;
    }
    assert(release_state);
    bool exist_task = false;
    for (auto it = PerfQue.begin(); it != PerfQue.end(); it++) {
        Transaction* w = *it;
        if (w == nullptr) continue;
        if (!w->in_ptc) continue;
        if (w->task != task) continue;

        unsigned channel = w->channel;
        unsigned rank = w->rank;
        unsigned group = w->group;
        unsigned bankIndex = w->bankIndex;

        if (w->transactionType != DATA_WRITE) {
            ERROR("Release Type Error!! task="<<task);
            assert(0);
        }
        exist_task = true;
        rel_wcmd_cnt --;
        wrel_bank_cnt[rank][bankIndex % NUM_BANKS] --;
        wrel_bg_cnt[rank][group] --;
        wrel_rank_cnt[rank] --;
        rel_rank_cnt[rank] --;
        if (w->transactionType == DATA_WRITE && WSRAM_MAP_EN) {
            unsigned data_size = (w->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
            unsigned need_size = wdataQueNeedSize(w,data_size);
            for (unsigned i = 0; i < need_size; i++) {
                unsigned slot_idx = w->wsram_idx + i;
                if (wsram_slt[slot_idx].state != OCCUPIED) {
                    ERROR("Invalid wsram sate at slot "<<slot_idx<<" for task="<<task
                            <<", expected OCCUPIED but got "<<wsram_slt[slot_idx].state);
                    assert(0);
                }
                wsram_slt[slot_idx].state = READING;
                wsram_slt[slot_idx].wdda = now() + WDDA + i*CLKH2CLKL;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Wcmd released, wsram_slt set, slt_idx="<<slot_idx<<", task="<<wsram_slt[slot_idx].task
                    <<", channel="<<wsram_slt[slot_idx].channel<<", wdda="<<wsram_slt[slot_idx].wdda<<endl);
                }

            }
        }

        if (!WSRAM_MAP_EN) {
            perf_wdata_sram wdata_sram;
            wdata_sram.task = task;
            wdata_sram.channel = channel;
            wdata_sram.burst_length = w->burst_length;
            wdata_sram.wsram_idx = w->wsram_idx;
            wdata_sram.wdda = now() + WDDA;//TBD parameter to adjust
            wdata_sram.issue_length = 0;
            Perf_WdataSram.push_back(wdata_sram);
        }

        wcmd_release_conflict(w);
        delete w->conflict_state;
        delete w;
        *it = nullptr;
        break;
    }
    if (!exist_task) {
        ERROR("Release Task"<<task<<" But Not Exist!");
        assert(0);
    }
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- PERF RELEASE W :: task="<<task<<" rel_wcnt="<<rel_wcmd_cnt<<endl);
    }
    return;
}

/***************************************************************************************************
Descriptor: update conflict interface with ptc
****************************************************************************************************/
bool PFQ::perf_conflict_intf(uint64_t task) {
    bool exist_task = false;
    bool ptc_rd_cancel = false;
    Transaction* fr_cancel_cmd = nullptr;
    for (auto& r : PerfQue) {
        if (r == nullptr) continue;
        if (r->transactionType != DATA_READ) continue;
        if (r->task != task) continue;
        if (r->in_ptc) {
            ERROR(setw(10)<<now()<<" Rcmd can not be in ptc, task="<<task);
            assert(0);
        }
        exist_task = true;
        if (now() < r->conflict_state->conf_time) continue;
        if (r->conflict_state->perf.ad_conf_cnt == 0) continue;
        
        //flag check
        if (!r->fast_rd || r->act_only) {
            ERROR(setw(10)<<now()<<" Wrong Fast_rd/Act_only Label, task="<<task<<" fast_rd="<<r->fast_rd
                    <<" act_only="<<r->act_only);
            assert(0);
        }

        ptc_rd_cancel = true;
        r->fast_rd = false;
        fr_cancel_cmd = r;
        break;
    }

    if (!exist_task) {
        ERROR(setw(10)<<now()<<" Get Conf Info, task="<<task<<", But Not Exist!");
        assert(0);
    }
    if (ptc_rd_cancel) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Rcmd Ready For Cancel :: task="<<task<<" conf_time="<<fr_cancel_cmd->conflict_state->conf_time
                    <<" ad_cc="<<fr_cancel_cmd->conflict_state->perf.ad_conf_cnt<<endl);
        }
    }
    
    return ptc_rd_cancel;
}

bool PFQ::returnReadDataResp(unsigned int channel_num, unsigned long long task,
        double readDataEnterDmcTime, double reqAddToDmcTime, double reqEnterDmcBufTime) {
    if (mpfq_->ReturnReadData!=NULL) {
        return (*mpfq_->ReturnReadData)(channel_num, task,
                readDataEnterDmcTime, reqAddToDmcTime, reqEnterDmcBufTime);
    } else {
        return false;
    }
}

void PFQ::gen_wresp(uint64_t task, uint32_t channel) {
    resp wresp;
    wresp.channel = channel;
    wresp.task = task;
    writeResp.push_back(wresp);
    linked_ptcs_[ptcID(channel)]->dresp_cnt++;
}

void PFQ::gen_rresp(uint64_t task, uint32_t channel) {
    resp rresp;
    rresp.channel = channel;
    rresp.task = task;
    readResp.push_back(rresp);
    linked_ptcs_[ptcID(channel)]->dresp_cnt++; 
}
