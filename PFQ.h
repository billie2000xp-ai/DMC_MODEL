#ifndef _PFQ_H
#define _PFQ_H

#include "Transaction.h"
#include "SimulatorObject.h"
#include "SystemConfiguration.h"

#include <memory>
#include <stdint.h>
#include <ostream>
#include <fstream>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <set>
#include <list>
#include <sstream>

using namespace std;

namespace DRAMSim {

// Forward declarations
typedef Transaction& t_ptr;

// ============================
// DATA STRUCTURES
// ============================

// WSRAM slot structure
struct perf_wsram_slt {
    WSram_State state = SLT_IDLE;
    uint64_t channel = ~0ULL;
    uint64_t task = 0;
    uint64_t wdda = 0;
};

struct perf_wdata_sram {
    uint64_t task = 0;
    uint64_t channel      = 0;
    uint64_t burst_length = 0;
    uint64_t issue_length = 0;
    uint64_t wsram_idx    = 0;
    uint64_t wdda         = 0;
};

// WSRAM slot structure
struct resp {
    uint64_t channel;
    uint64_t task;
};

// Performance priority information
struct PerfPriInfo {
    unsigned m_ori_pri;      // Original priority
    unsigned m_rank_pri;     // Rank priority
    unsigned m_bg_pri;       // Bank group priority
    unsigned m_bg_num;       // Bank group number
    unsigned m_bank_num;     // Bank number
    unsigned m_rowhit_pri;   // Row hit priority
    unsigned m_state_pri;    // State priority
    
    PerfPriInfo() {
        m_bg_pri = 0;
        m_rank_pri = 0;
        m_rowhit_pri = 0;
        m_ori_pri = 0;
        m_bg_num = 0;
        m_bank_num = 0;
        m_state_pri = 0;
    }
    
    PerfPriInfo(unsigned bg_pri, unsigned rowhit_pri, unsigned ori_pri, 
                unsigned bg_num, unsigned bank_num, unsigned rank_pri, 
                unsigned state_pri) {
        m_bg_pri = bg_pri;
        m_rowhit_pri = rowhit_pri;
        m_ori_pri = ori_pri;
        m_bg_num = bg_num;
        m_bank_num = bank_num;
        m_rank_pri = rank_pri;
        m_state_pri = state_pri;
    }
    
    // Comparison operators for priority ordering
    bool operator<(PerfPriInfo other) {
        if (other.m_ori_pri == QOS_MAX && m_ori_pri != QOS_MAX) {
            return true;
        } else if (other.m_ori_pri != QOS_MAX && m_ori_pri == QOS_MAX) {
            return false;
        } else if (m_rank_pri != other.m_rank_pri) {
            return m_rank_pri < other.m_rank_pri;
        } else if (m_bg_pri != other.m_bg_pri) {
            return m_bg_pri < other.m_bg_pri;
        } else if (m_bank_num != other.m_bank_num) {
            return m_bank_num < other.m_bank_num;
        } else if (m_bg_num != other.m_bg_num) {
            return m_bg_num < other.m_bg_num;
        } else if (m_rowhit_pri != other.m_rowhit_pri) {
            return m_rowhit_pri < other.m_rowhit_pri;
        } else if (m_state_pri != other.m_state_pri) {
            return m_state_pri < other.m_state_pri;
        } else if (m_ori_pri != other.m_ori_pri) {
            return m_ori_pri < other.m_ori_pri;
        }
        return false;
    }
    
    bool operator==(PerfPriInfo other) {
        return (m_ori_pri == other.m_ori_pri &&
                m_rowhit_pri == other.m_rowhit_pri &&
                m_bg_pri == other.m_bg_pri &&
                m_bank_num == other.m_bank_num &&
                m_bg_num == other.m_bg_num &&
                m_rank_pri == other.m_rank_pri &&
                m_state_pri == other.m_state_pri);
    }
    
    bool operator>(PerfPriInfo other) {
        return (!this->operator<(other)) && !this->operator==(other);
    }
};

// Arbitration command structure
struct arb_cmd {
    uint64_t task;
    TransactionType type;
    PerfPriInfo pri_info;
    unsigned timeAdded;
    unsigned rank;
    unsigned row;
    unsigned bank;
    unsigned group;
    unsigned bankIndex;
    unsigned pri;
    unsigned qos;
    
    arb_cmd() {
        task = 0;
        type = DATA_READ;
        timeAdded = 0;
        rank = 0;
        row = 0;
        bank = 0;
        group = 0;
        bankIndex = 0;
        pri = 0;
        qos = 0;
    }
    
    void creat(Transaction *t) {
        task = t->task;
        type = t->transactionType;
        timeAdded = t->timeAdded;
        rank = t->rank;
        row = t->row;
        bank = t->bank;
        group = t->group;
        bankIndex = t->bankIndex;
        pri = t->pri;
        qos = t->qos;
    }
};

// Group mode structure
struct Grp_Mode {
    uint8_t grp_mode0;
    uint8_t grp_mode1;
    uint8_t grp_mode23;
    uint8_t grp_mode4;
    uint8_t grp_mode5;
    uint8_t grp_mode6;
    uint8_t grp_mode7;
    
    Grp_Mode() {
        grp_mode0 = 0;
        grp_mode1 = 0;
        grp_mode23 = 0;
        grp_mode4 = 0;
        grp_mode5 = 0;
        grp_mode6 = 0;
        grp_mode7 = 0;
    }
};

// ============================
// PFQ CLASS DEFINITION
// ============================

class PTC;
class MPFQ;
class MemorySystemTop;

class PFQ : public SimulatorObject {
public:
    // ============ CONSTRUCTOR ============
    PFQ(unsigned index, MemorySystemTop* top, MPFQ* mpfq, ofstream &DDRSim_log_, ofstream &trace_log_);
    virtual ~PFQ();

    // ============ PUBLIC INTERFACE METHODS ============
    // Transaction & Data interface
    bool addTransaction(Transaction *trans);
    bool addData(uint32_t *data, uint64_t task);

    void addAssociatedPTC(PTC* ptc);
    // void setMPFQ(MPFQ* mpfq) { mpfq_ = mpfq; }
    unsigned getID() const { return id_; }
    unsigned ptcID(unsigned channel) const;
    PTC* getPTC(unsigned channel) const;
    MemorySystemTop* getMemorySystemTop() const { return memorySystemTop_; }
    MPFQ* getMPFQ() const { return mpfq_; }
    
    // WSRAM management
    bool enoughWSRAM(Transaction *trans, uint32_t burst_length);
    bool findFreeWsram(Transaction *trans, unsigned need_size);
    unsigned wdataQueNeedSize(Transaction *trans, unsigned data_size);
    
    // Perf_Queue management
    bool perf_full();
    
    // Subqueue management
    bool PerfQue_pushCmd(Transaction *trans);
    std::pair<size_t,size_t> getSubqRange(size_t subq_idx);
    unsigned lru_arb(uint64_t index1, uint64_t index2, uint64_t sel);
    void update_lru(uint64_t index, uint64_t sel, arb_cmd* cmd);
    size_t findTargetSubq(size_t bg, size_t bank);
    size_t findEmptySlotInSubq(size_t subq_idx);
    size_t findBestSubqByLRU(size_t avoid_subq);
    
    // Conflict management
    void rcmd_set_conflict(Transaction *trans);
    void wcmd_set_conflict(Transaction *trans);
    void rcmd_release_conflict(Transaction *trans);
    void wcmd_release_conflict(Transaction *trans);
    void dmc_release_conflict(Transaction *trans);
    
    // Read/write operations
    bool read_forward(Transaction *trans);
    bool write_merge(Transaction *trans);
    void rcmd_push_wcmd(Transaction *trans);
    
    // Scheduler interface
    void update();
    void sch_subque();
    void arb_node();
    unsigned priority(arb_cmd *cmd);
    
    // State management
    void update_state();
    void update_state_trig();
    unsigned check_wr_level();
    void stateTransition();
    
    // Timeout handling
    void perf_check_timeout_and_aging();

    void process_rmw_data_flow();
    void update_rpsram_state();
    
    // WSRAM release
    void wsram_release_wdata();
    void send_wdata();
    
    // Statistics and monitoring
    inline unsigned rcmd_num() { return rcmd_cnt; }
    inline unsigned wcmd_num() { return wcmd_cnt; }
    unsigned get_max_rank(bool isRd);
    unsigned GetPerfQsize() {return (rcmd_cnt + wcmd_cnt + rel_wcmd_cnt + rel_rcmd_cnt);};
    
    // Interface methods for external modules
    void perf_release_rd(uint64_t task, bool release_state);
    void perf_release_wr(uint64_t task, bool release_state);
    bool perf_conflict_intf(uint64_t task);
    
    // ============ PUBLIC MEMBER VARIABLES ============
    
    // Timing information
    uint64_t m_pre_req_time;
    uint64_t perf_pre_req_time;
    uint64_t m_pre_data_time;
    uint64_t perf_pre_data_time;
    
    // WSRAM storage
    std::vector<perf_wsram_slt> wsram_slt;
    std::vector<uint64_t> WdataToSend;
    vector <perf_wdata_sram> Perf_WdataSram;
    // Bitmaps and state vectors
    std::vector<bool> msize_bmp;
    std::vector<bool> msize_single;
    std::vector<bool> lsize_bmp;
    std::vector<bool> rcmd_bank_state;

    vector<vector<uint32_t>> subq_lru_matrix;
    // vector<vector<uint32_t>> cmd1_lru_matrix;
    vector<vector<vector<uint32_t>>> cmd1_lru_matrix;
    vector<vector<uint32_t>> cmd2_lru_matrix;
    
    // Performance queue
    std::vector<Transaction*> PerfQue;
    
    // Counters for commands
    unsigned wcmd_cnt;
    unsigned rcmd_cnt;
    unsigned rel_wcmd_cnt;
    unsigned rel_rcmd_cnt;
    
    // Write buffer states
    WB_State wbuff_state;
    WB_State wbuff_state_pre;
    WB_RankState wbuff_rnkgrp_state;
    
    // Bank and rank counters
    std::vector<std::vector<unsigned>> wb_bg_cnt;
    std::vector<std::vector<unsigned>> rb_bg_cnt;
    std::vector<std::vector<unsigned>> wb_bank_cnt;
    std::vector<std::vector<unsigned>> rb_bank_cnt;
    std::vector<unsigned> wb_rank_cnt;
    std::vector<unsigned> rb_rank_cnt;
    
    // Release counters
    std::vector<std::vector<unsigned>> rrel_bg_cnt;
    std::vector<std::vector<unsigned>> wrel_bg_cnt;
    std::vector<std::vector<unsigned>> rrel_bank_cnt;
    std::vector<std::vector<unsigned>> wrel_bank_cnt;
    std::vector<unsigned> rrel_rank_cnt;
    std::vector<unsigned> wrel_rank_cnt;
    std::vector<unsigned> rel_rank_cnt;
    std::vector<unsigned> rank_cnt;
    
    // Performance counters
    std::vector<unsigned> perf2ptc_bank_rcnt;
    std::vector<unsigned> perf2ptc_bank_wcnt;
    std::vector<unsigned> perf_bank_rcnt;
    std::vector<unsigned> perf_bank_wcnt;
    std::vector<std::vector<unsigned>> perf_bg_rcnt;
    std::vector<std::vector<unsigned>> perf_bg_wcnt;
    std::vector<std::vector<unsigned>> perf2ptc_bg_rcnt;
    std::vector<std::vector<unsigned>> perf2ptc_bg_wcnt;
    
    // Read data field
    bool rmw_write_bp_this_cycle;
    bool rmw_write_bp_next_cycle;
    bool rp_fifo_almost_full;
    bool rmw_rdata_fifo_full;
    bool rmw_bp_rdata_path;
    bool rcmd_bp_byrp;
    // bool upstream_bp;
    std::deque<data_packet> rp_fifo;
    std::deque<data_packet> rmw_rdata_fifo;
    
    // Rank group information
    unsigned perf_sch_rrank;
    unsigned perf_sch_wrank;
    bool perf_rd_in_rnkgrp;
    bool perf_wr_in_rnkgrp;
    
    // Performance signals
    bool perf_cmd_vld;
    unsigned perf_addrconf_cnt;
    
    // Timeout flags
    bool has_rd_tout;
    bool has_wr_tout;
    
    // Dummy timeout flags
    bool has_rd_dummy_tout;
    bool has_wr_dummy_tout;
    bool has_hqos_dummy_tout;
    unsigned hqos_rank;
    
    // High QoS management
    std::vector<unsigned> que_read_highqos_cnt;
    std::vector<unsigned> que_read_highqos_vld_cnt;
    std::vector<bool> rank_cmd_high_qos;
    std::vector<bool> perf_has_hqos;
    std::vector<unsigned> rb_highqos_bank_cnt;
    std::vector<bool> perf_has_highqos_cmd_rowhit;
    unsigned perf_highqos_trig_grpsw_cnt;
    
    // Transaction interleaving
    vector<vector<unsigned>> trans_baintlv; // Related to MemorySystem
    
    // Statistics and debug
    size_t lastselcmd_subq_idx;
    uint64_t lastcmd_seltime;
    unsigned ws_bp_cnt;
    unsigned perf_bp_cnt;
    unsigned forward_cnt;
    unsigned merge_cnt;
    unsigned push_cnt;
    vector<uint8_t> rb_qos_cnt;
    vector<uint8_t> wb_qos_cnt;
    vector<unsigned> sch_level_cnt;
    
    vector<vector<unsigned *>> wr_level;
    vector<vector<unsigned *>> wr_most_level;
    vector<vector<unsigned *>> rd_level;
    vector<vector<unsigned *>> grp_mode;
    
    unsigned state_trig = 0;
    unsigned idle_trig_time = 0;
    arb_cmd LastArbCmd;
    std::vector<arb_cmd *> ArbCmd;
    Grp_Mode GrpMode;
    
    unsigned rd_sch_rhit_cnt = 0;
    unsigned rd_timeout_cnt = 0;
    unsigned wr_timeout_cnt = 0;
    unsigned wr_adtimeout_cnt = 0;
    unsigned ptc_empty_cnt;
    unsigned nogrp_cnt = 0;
    
    // Counters
    unsigned read_cnt;
    unsigned write_cnt;
    unsigned used_wdata_que_cnt;
    
    // Performance log
    ofstream perf_log;
    string pfq_log;

    uint64_t pre_wresp_time;
    uint64_t pre_rresp_time;
    void gen_wresp(uint64_t task, uint32_t channel);
    void gen_rresp(uint64_t task, uint32_t channel);
    bool ArbCmd_notempty();

    //for sid group
    bool sidGroupPass(Transaction* trans);
    void update_sidgroup_state(unsigned rank);
    unsigned get_max_sid(unsigned rank);
    vector <unsigned> sid_group_state;
    vector <unsigned> quc_slt_grp_lr; //sid group state, delay array
    vector <unsigned> pre_quc_slt_grp_lr;
    vector <unsigned> serial_sid_cnt;
    vector <unsigned> sidgrp_ch_cmd_cnt;
    unsigned in_sid_group; // cur sid grp, 0-3 is illegal val, 4 means not real in sid group
    vector<vector<unsigned>> sid_timeout;
    vector<vector<bool>> sid_issue_state;
    
private:
    // ============ PRIVATE MEMBER VARIABLES ============
    unsigned id_;
    std::vector<PTC*> linked_ptcs_;
    MemorySystemTop* memorySystemTop_;
    MPFQ* mpfq_;
    // Channel identification
    unsigned channel;
    uint64_t channel_ohot;
    
    // Logging
    ofstream &DDRSim_log;
    ofstream &trace_log;
    // string log_path;
    
    // Internal counters
    unsigned ser_write_cnt;
    unsigned ser_read_cnt;
    unsigned serial_cmd_cnt;
    unsigned wbuff_state_gap;
    unsigned max_rank;
    unsigned ser_sch_write;
    unsigned same_bank_cnt;
    unsigned no_cmd_sch_cnt;
    unsigned no_rcmd_sch_cnt;
    unsigned no_cmd_sch_th;
    unsigned bytes_per_col;
    uint64_t ecc_wb_cnt;
    uint64_t ecc_wb_task_next;
    struct ecc_wb_wdata {
        uint32_t channel;
        uint64_t task;
        uint32_t remaining;
    };
    std::deque<hha_command> ecc_wb_cmd_queue;
    std::deque<ecc_wb_wdata> ecc_wb_wdata_queue;
    
    // Timing and response
    unsigned m_pre_update_time = 0;
    vector<resp> writeResp;
    vector<resp> readResp;
    vector<resp> cmdResp;
    unsigned pre_cresp_time = 0;
    
    // Command history
    vector<std::list<arb_cmd>> pre_n_cmd_each_rank;
    arb_cmd pre_cmd;
    
    // ============ PRIVATE HELPER METHODS ============
    
    // Clock synchronization
    bool clkSyn();
    bool curClkCanAddTrans();
    bool curClkCanAddData();
    
    // Response generation
    void updateWRresp();
    void updateCresp();
    void genCresp(Transaction *trans);

    bool returnReadDataResp(unsigned int channel_num, unsigned long long task,
    double readDataEnterDmcTime, double reqAddToDmcTime, double reqEnterDmcBufTime);
    
    // Timeout handling
    void setPerfTimeout(Transaction *trans);
    
    // State transition helpers
    void update_rwgroup_state();
    void update_rnkgroup_state();
    
    // Policy checks
    bool rankGroupPass(Transaction *trans);
    bool rowhitPolicyPass(Transaction *trans);
    bool bankStatePolicyPass(Transaction *trans);
    bool bankConfPolicyPass(Transaction *trans);
    bool rowhitBreakPolicyPass(Transaction *trans);
    bool ptcRowhitBreakPolicyPass(Transaction *trans);
    bool ptcDummyToutPolicyPass(Transaction *trans);
    bool bankGroupPolicyPass(Transaction *trans);
    
    // Priority calculation helpers
    unsigned getBankGroupPri(Transaction *trans);
    unsigned getRankGroupPri(Transaction *trans);
    bool realRowhit(Transaction *trans);
    
    // Command history management
    void updatePreNCmd(arb_cmd cmd);
    
    // Debug and monitoring
    void setRowhitMiss(Transaction *trans);
    string getPTCCmdSituation();
    
    // PTC interface
    std::set<unsigned> getPtcExistRank();

    // void PFQInitOutputFiles();
};

} // namespace DRAMSim

#endif // _PFQ_H
