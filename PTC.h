#ifndef _PTC_H_
#define _PTC_H_

#include "SimulatorObject.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "BankState.h"
#include "Inline_ECC.h"
#include "AddressMapping.h"
#include <map>
#include <queue>
#include <iomanip>
#include <cmath>
#include <assert.h>
#include <numeric>
#include <algorithm>

using namespace std;

namespace DRAMSim {

class Inline_ECC;
typedef Transaction& t_ptr;

struct task_info {
    bool rd_finish = false;
    bool wr_finish = false;
    bool rd_ecc    = false;
    bool wr_ecc    = false;
    uint32_t slt_cnt = 0;
};

struct wdata_pipe {
    uint64_t task = 0;
    uint64_t delay = 0;
};

class PFQ;
class Rank;
class MemorySystemTop;

class PTC : public SimulatorObject {

public:
    //functions
    PTC(unsigned index, MemorySystemTop* top, ofstream &DDRSim_log_, ofstream &trace_log_, ofstream &cmdnum_log_);
    virtual ~PTC();

    void addRank(Rank* rank);
    void setAssociatedPFQ(PFQ* pfq) { pfq_ = pfq; }
    unsigned getID() const { return id_; }
    PFQ* getAssociatedPFQ() const { return pfq_; }
    MemorySystemTop* getMemorySystemTop() const { return memorySystemTop_; }
    Rank* getRank(unsigned index) const;

    bool addTransaction(Transaction *trans, bool fastread);
    void update_even_cycle();
    void noc_read_inform(bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty);
    bool WillAcceptTransaction();
    bool returnReadData(unsigned int channel_num, unsigned long long task,
    double readDataEnterDmcTime, double reqAddToDmcTime, double reqEnterDmcBufTime);
    void ReturnData_statistics(uint64_t task, uint64_t timeAdded, unsigned qos, unsigned mid, unsigned pf_type, unsigned rank);
    void Cmd2Dfi_statistics(uint64_t task, uint64_t timeAdded, unsigned qos, unsigned mid, unsigned pf_type, unsigned rank);
    void receive_rdata(unsigned long long task, bool mask_wcmd);
    void receive_wdata(unsigned *data ,uint64_t task);
    // void attachRanks(vector<Rank *> *rank);
    unsigned CalcCasTiming(unsigned bl, unsigned sync, unsigned wck_pst);
    unsigned CalcTiming(bool is_trtp, unsigned cmd_bl, unsigned timing);
    unsigned CalcTccd(bool is_samebg, unsigned cmd_bl, unsigned tccd);
    unsigned CalcWrite2Mwrite(bool is_samebg, bool is_sameba, unsigned cmd_bl);
    unsigned CalcMwrite2Write(bool is_samebg, bool is_sameba, unsigned cmd_bl);
    unsigned CalcMwrite2Mwrite(bool is_samebg, bool is_sameba, unsigned cmd_bl);
    void CalcBl(Transaction *trans);
    void update();
    void ent_pop();
    void rw_pop();
    void resetStats();
    void cmd_select(Transaction *transaction);
    void power_event_stat();
    vector<uint64_t> que_cmd_time;
    void update_lp_state();
    void update_wdata();
    void update_grt_fifo();
    void dfi_pipe();
    void refresh(unsigned sc);
    void all_bank_refresh(unsigned rank, unsigned sc);
    void per_bank_refresh(unsigned rank, unsigned sc);
    void send_pb_refresh(unsigned rank, unsigned sc);
    void send_pb_precharge(unsigned rank, unsigned sc);
    void enh_per_bank_refresh(unsigned rank, unsigned sc);
    void enh_send_pb_refresh(unsigned rank, unsigned sc);
    void enh_send_pb_precharge(unsigned rank, unsigned sc);
    void gd2_dist_refresh();
    enum DIST_STATE {DIST_IDLE, SEND_ACT1, SEND_ACT2, SEND_PRE};
    struct GD2_DIST_STATE {
        DIST_STATE state;
        unsigned bank;
        vector <uint32_t> pre_cmd_cnt;
        vector <uint32_t> dist_pstpnd_num;
        vector<uint32_t> pre_cmd_cnt_fg;
        bool force_dist_refresh;
        GD2_DIST_STATE (size_t index) {
            state = DIST_IDLE;
            bank = index;
            for (size_t i = 0; i < 4; i ++) {
                pre_cmd_cnt.push_back(0);
                dist_pstpnd_num.push_back(0);
                pre_cmd_cnt_fg.push_back(0);
            }
            force_dist_refresh = false;
        }
    };
    vector <GD2_DIST_STATE> DistRefState;
    struct pre_cmd {
        TransactionType trans_type;
        TransactionCmd type;
        unsigned rank;
        unsigned bankIndex;
        unsigned sid;
        unsigned group;
        pre_cmd () {
            type = READ_CMD;
            trans_type = DATA_READ;
            rank = 0;
            bankIndex = 0;
            sid = 0;
            group = 0;
        }
    };
    pre_cmd PreCmd;
    unsigned ser_rw_cnt = 0;
    unsigned ser_rank_cnt = 0;
    unsigned ser_sid_cnt = 0;
    unsigned neg_r2w_rank_sw = 0;
    void scheduler();
    void state_fresh();
    void data_fresh();
    void need_issue(Transaction *transaction);
    void check_timeout_and_aging();
    void ptc_check_timeout_and_aging();
    void generate_packet(Cmd *c);
    void tsc_update(const BusPacket &bus_packet,bool hit);
    void check_conflict(Transaction *trans);
    void sch_pipe();
    bool apply_bank(Transaction *transaction);
    void release_bank(unsigned BankIndex);
    unsigned priority(Cmd *cmd);
    unsigned priority_pri(Cmd *cmd);
    void queue_update();
    void bsc_update();
    bool check_fresh(uint32_t &frerank);
    bool check_rankStates(uint8_t rank);
    bool check_samebank(unsigned int bank);
    void faw_update();
    void page_timeout_policy();
    void update_rwgroup_state();
    void update_group_state();
    void page_adapt_policy(Transaction *trans);
    void page_adpt_policy(Transaction *trans);
    unsigned opc_cnt;
    unsigned ppc_cnt;
    unsigned page_adpt_win_cnt;
    string bank_state_opcode(CurrentBankState state);
    string trans_cmd_opcode(TransactionCmd state);
    string trans_type_opcode(TransactionType state);
    void update_state();
    void update_state_pre();
    void update_state_post();
    void update_statistics();
    void ehs_page_adapt_policy();
    void update_deque_fifo();
    // void attachPFQ(PFQ *pfq);
    inline unsigned data_cnt_perburst(unsigned length) {
            return ((DmcLog2(length, JEDEC_DATA_BUS_BITS)) / DMC_DATA_BUS_BITS);}
    unsigned Read_Cnt() {return que_read_cnt;};
    unsigned Write_Cnt() {return que_write_cnt;};
    void PostCalcTiming();
    void calc_occ();
    unsigned get_occ() {return occ;}
    double calc_sqrt(double sum, double sum_pwr2, unsigned cnt);
    void trans_state_init(Transaction *trans);
    uint32_t occ;
    uint32_t availability;
    uint64_t sum_avai;
    uint64_t sum_pwr_avai;
    double avai_sqrt;
    uint32_t row_hit_ratio;
    unsigned occ_1_cnt;
    unsigned occ_2_cnt;
    unsigned occ_3_cnt;
    unsigned occ_4_cnt;
    unsigned bw_totalcmds;
    unsigned bw_totalwrites;
    unsigned bw_totalreads;
    unsigned ecc_total_bytes;
    unsigned ecc_total_reads;
    unsigned ecc_total_writes;
    unsigned rmw_total_bytes;
    unsigned rmw_total_reads;
    uint64_t TotalDmcBytes;
    uint64_t TotalDmcRdBytes;
    uint64_t TotalDmcWrBytes;
    unsigned TotalDmcRd32B;
    unsigned TotalDmcRd64B;
    unsigned TotalDmcRd128B;
    unsigned TotalDmcRd256B;
    unsigned TotalDmcWr32B;
    unsigned TotalDmcWr64B;
    unsigned TotalDmcWr128B;
    unsigned TotalDmcWr256B;
    float CalcBwByByte(uint64_t byte, unsigned cycle);

    //fields
    vector<Transaction *> transactionQueue;
    vector<Transaction *> WtransQueue;
    vector <bool> slt_valid;
    std::map< uint64_t,TRANS_MSG > pending_TransactionQue;
    BusPacket command;
    unsigned  command_pend;
    
    //even/odd cycle flag
    bool even_cycle;
    bool odd_cycle;

    //statistics for DMC
    unsigned totalTransactions;
    unsigned RtCmdCnt;
    unsigned totalReads;
    unsigned totalWrites;
    unsigned addrconf_cnt;
    unsigned idconf_cnt;
    unsigned baconf_cnt;
    unsigned totalconf_cnt;
    unsigned active_cnt;
    unsigned active_dst_cnt;
    unsigned bypass_active_cnt;
    unsigned precharge_sb_cnt;
    unsigned precharge_pb_cnt;
    unsigned precharge_ab_cnt;
    unsigned precharge_pb_dst_cnt;
    unsigned read_p_cnt;
    unsigned write_p_cnt;
    unsigned read_cnt;
    unsigned write_cnt;
    unsigned ecc_read_cnt;
    unsigned ecc_write_cnt;
    unsigned merge_read_cnt;
    unsigned mwrite_cnt;
    unsigned mwrite_p_cnt;
    unsigned refresh_ab_cnt;
    unsigned refresh_pb_cnt;
    unsigned pbr_overall_cnt;
    unsigned page_exceed_cnt;
    unsigned dresp_cnt;
    vector<vector<unsigned>> forceRankBankIndex;
    vector<vector<unsigned>> refresh_cnt_pb;
    vector<vector<unsigned>> rank_refresh_cnt;
    vector<unsigned> perbank_refresh_cnt;
    vector<bool> has_timeout_rank;
    uint32_t dmc_timeout_cnt;
    uint32_t dmc_fast_timeout_cnt;
    uint32_t dmc_slow_timeout_cnt;
    //uint32_t timeout_cnt;
    unsigned com_read_cnt;
    unsigned rw_switch_cnt;
    unsigned r2w_switch_cnt;
    unsigned w2r_switch_cnt;
    unsigned rank_switch_cnt;
    unsigned r_rank_switch_cnt;
    unsigned w_rank_switch_cnt;
    unsigned sid_switch_cnt;
    unsigned r_lgap_cnt;
    unsigned w_lgap_cnt;
    unsigned r2w_lgap_cnt;
    unsigned w2r_lgap_cnt;
    unsigned dly_ex2000_cnt;
    uint64_t total_latency;
    vector<uint64_t> total_latency_rank;
    vector<uint32_t> com_read_cnt_rank;
    float ddrc_av_lat;
    vector<float> ddrc_av_lat_rank;
    vector<unsigned> bank_slot;
    vector<unsigned> ptc_que_slot;
    vector<uint32_t> r_bank_cnt;
    vector<uint32_t> w_bank_cnt;
    vector<uint32_t> bank_cnt;
    vector<vector<uint32_t>> ptc_lru_matrix;
    vector<vector<uint32_t>> bank_lru_matrix;
    vector<vector<uint32_t>> r_bg_cnt;
    vector<vector<uint32_t>> w_bg_cnt;
    vector<vector<uint32_t>> bg_cnt;
    vector<uint32_t> ptc_r_bank_cnt;
    vector<uint32_t> ptc_w_bank_cnt;
    vector<vector<uint32_t>> ptc_r_bg_cnt;
    vector<vector<uint32_t>> ptc_w_bg_cnt;
    vector<uint32_t> r_rank_cnt;
    vector<uint32_t> w_rank_cnt;
    vector<uint32_t> rank_cnt;
    vector<vector<uint32_t>> sc_cnt;
    vector<vector<uint32_t>> r_sid_cnt;
    vector<vector<uint32_t>> w_sid_cnt;
    vector<vector<uint32_t>> sid_cnt;
    vector<deque<unsigned>> deqCmdWakeupLp;
    vector<uint32_t> r_qos_cnt;
    vector<uint32_t> w_qos_cnt;
    vector<COUNTER> access_bank_delay;
    vector<uint32_t> bank_cas_delay;
    vector<uint32_t> acc_rank_cnt;
    vector<uint32_t> acc_bank_cnt;
    vector<uint32_t> racc_rank_cnt;
    vector<uint32_t> racc_bank_cnt;
    vector<uint32_t> wacc_rank_cnt;
    vector<uint32_t> wacc_bank_cnt;
    vector<uint32_t> active_cmd_cnt;
    vector<uint32_t> page_cmd_cnt;
    vector<uint32_t> page_timeout_rd;
    vector<uint32_t> page_timeout_wr;
    vector<vector<int32_t>> page_row_hit;
    vector<vector<int32_t>> page_row_miss;
    vector<vector<int32_t>> page_row_conflict;
    uint32_t page_timeout_window[34];
    vector<uint32_t> BankRowActCnt;
    unsigned adpt_openpage_time;
    vector<uint32_t> r_rank_bst;
    vector<uint32_t> w_rank_bst;
    vector<uint32_t *> r_rank_mux;
    vector<uint32_t *> w_rank_mux;
    vector<uint32_t> bank_cnt_ehs;
    vector<vector<uint64_t>> ehs_page_adapt_cnt;

    vector<vector<unsigned>> Rcmd_Dist;
    vector<vector<unsigned>> Wcmd_Dist;

//    vector<vector<unsigned>> trans_baintlv;
    

    vector<unsigned> qos_delay_cnt;
    vector<unsigned> qos_cnt;
    vector<unsigned> qos_timeout_cnt;
    vector<unsigned> qos_fast_timeout_cnt;
    vector<unsigned> qos_slow_timeout_cnt;
    vector<vector<unsigned>> qos_level_cnt;
    vector<unsigned> mid_delay_cnt;
    vector<unsigned> mid_cnt;
    vector<unsigned> pf_delay_cnt;
    vector<unsigned> pf_cnt;

    //grt fifo realted
    unsigned grt_fifo_wcmd_cnt;
    bool grt_fifo_bp;
    
    //PTC constraint check
    void ptc_constraint_check();

    //seperate lc for col/row op
    void Col_Row_lc(vector<unsigned> &arb_group_pri, unsigned arb_type);
    //Round Robin Priority for arb groups for col/row op
    void update_arb_group_pri(Cmd *c, vector<unsigned> &arb_group_pri, unsigned arb_type);
    vector<unsigned> arb_group_pri_col;
    vector<unsigned> arb_group_pri_row;
    // number of tran for arb groups
    vector<unsigned> arb_group_cnt;

    //interface for perf queue
    unsigned getPtcQueSize(bool isRd);
    BankTableState get_bank_state(unsigned bankIndex);
    unsigned get_rank_cnt(unsigned rank, bool isRd);
    bool get_bs_full(Transaction *trans);
    unsigned get_bg_cnt(unsigned rank, unsigned group, bool isRd);
    unsigned get_bank_cnt(unsigned bankIndex, bool isRd);
    bool get_refresh_state(unsigned bankIndex);
    unsigned get_perf_timeout_rank_cnt(unsigned rank);

//    bool force_rank_switch;
    unsigned PreCmdTime;
    bool has_other_rank_cmd;
    bool has_hqos_rank_rcmd;
    unsigned hqos_rank;
    bool all_rank_has_cmd;

    uint64_t flowStatisTotalBytes;

    uint64_t TotalBytes;
    uint64_t TotalReadBytes;
    uint64_t TotalWriteBytes;
    vector<uint64_t> TotalBytesRank;
    uint64_t DmcTotalBytes;
    uint64_t DmcTotalReadBytes;
    uint64_t DmcTotalWriteBytes;

    vector<unsigned> lat_dly_cnt;
    vector<unsigned> lat_dly_step;
    unsigned min_delay;
    unsigned max_delay;
    uint64_t min_delay_id;
    uint64_t max_delay_id;

    unsigned que_read_cnt;
    unsigned que_write_cnt;
    unsigned dmc_cmd_cnt;
    unsigned pre_act_cnt;
    unsigned total_pre_act_cnt;
    unsigned fast_rd_cnt;
    unsigned fast_act_cnt;
    unsigned total_fast_rd_cnt;
    unsigned total_fast_act_cnt;
    unsigned total_fastrd_cancel_cnt;
    unsigned total_iecc_cnt;
    unsigned total_noiecc_cnt;
    std::vector<std::vector<BankState>> banktable;
    bool full() {return ((que_read_cnt + que_write_cnt) >= TRANS_QUEUE_DEPTH);};
    void dfs_backpress(bool backpress);
    bool dfs_backpress_en;
    uint64_t total_dfs_bp_cnt;
    unsigned GetDmcQsize() {return (que_read_cnt + que_write_cnt);}
    std::vector<vector<FORALLREFRESH>> refreshALL;
    std::vector<PERBANKREFRESH> refreshPerBank;
    vector <BankTableState> bankStates;
    vector <RankStatus> RankState;
    vector<data_packet>read_data_buffer;
    PhyLpStatus PhyLpState;
    vector <bool> fast_wakeup;
    vector <int> fast_wakeup_cnt;
    vector <FuncState> funcState;
    vector <uint32_t> PdTime;
    vector <uint32_t> AsrefTime;
    vector <uint32_t> SrpdTime;
    vector <uint32_t> WakeUpTime;
    vector <uint32_t> PdEnterCnt;
    vector <uint32_t> PdExitCnt;
    vector <uint32_t> AsrefEnterCnt;
    vector <uint32_t> AsrefExitCnt;
    vector <uint32_t> SrpdEnterCnt;
    vector <uint32_t> SrpdExitCnt;
    map <unsigned, unsigned> RdCntBl;
    map <unsigned, unsigned> WrCntBl;
    vector <wdata_pipe> WdataPipe;
    uint64_t rd_inc_cnt;
    uint64_t bs_max;
    uint64_t bs_cnt;
    uint64_t bs_total;
    uint64_t rd_wrap_cnt;
    uint64_t wr_inc_cnt;
    uint64_t wr_wrap_cnt;
    uint64_t rdata_cnt;
    uint64_t wdata_cnt;
    unsigned phy_lp_cnt;
    unsigned phy_notlp_cnt;
    vector <uint8_t> rw_group_state;
    bool in_write_group;
    uint8_t rk_grp_state;
    uint8_t real_rk_grp_state;
    map <unsigned, unsigned> BL_n;
    map <unsigned, unsigned> BL_n_min;
    map <unsigned, unsigned> BL_n_max;
    uint64_t cmd_in2dfi_lat;
    uint64_t cmd_in2dfi_cnt;
    uint64_t cmd_rdmet_cnt;
    unsigned forward_64B_cnt;
    unsigned forward_128B_cnt;
    // MPTC *parentMPTC;
    vector <uint64_t> bp_cycle;
    vector <uint32_t> bp_step;
    bool has_cmd_bp();
    bool has_bypact_exec;
    vector <bool> act_executing;
    vector<unsigned> pre_enh_pbr_bagroup;
    vector<unsigned> pre_sch_bankIndex;
    vector<bool> issue_state;
    vector<Transaction *> fastrd_fifo;

    // ptc rwgrp control
    unsigned perf_grpst_switch_cnt;
    WB_State perf_state_pre;

    //dummy timeout/hqos cnt
    unsigned dummy_timeout_cnt;
    unsigned dummy_hqos_cnt;

    //long-short-long
    unsigned rw_idle_cnt;

private:
    unsigned id_;
    PFQ* pfq_;
    std::vector<Rank*> linked_ranks_;
    MemorySystemTop* memorySystemTop_;

    ofstream &DDRSim_log;
    ofstream &trace_log;
    ofstream &cmdnum_log;
    ofstream check_log;
    uint32_t channel;
    uint32_t sub_cha;
    uint64_t channel_ohot;
    deque <Cmd *> rw_delay_buf;
    deque <Transaction *> ent_delay_buf;
    vector <Cmd *> CmdQueue;
    vector <Cmd *> rw_cmdqueue;
    vector <Cmd *> act_cmdqueue;
    vector <Cmd *> pre_cmdqueue;
    vector<DataPacket> writeDataToSend;
    uint32_t data_delay;
    vector< vector<unsigned> > tFAWCountdown;
    vector< vector<unsigned> > tFAWCountdown_sc1;
    vector< vector<unsigned> > tFPWCountdown;
    unsigned cmdCyclesLeft;
    // vector<Rank *> *ranks;
    bool refreshing;
    bool exec_valid;
    vector <vector<bool>> refresh_pbr_has_finish;
    vector <vector<bool>> force_pbr_refresh;
    vector<phy>packet;
    uint64_t pre_dat_time;
    //for adapt page policy
    bool arb_enable;
    vector <std::vector<pbr_weight>> PbrWeight;
    vector <std::vector<pbr_weight>> SbGroupWeight;       // used for different groups, in which banks have same ba1ba0
    vector <std::vector<pbr_weight>> SbWeight;            // used for banks with same ba1ba0
    vector <std::vector<unsigned>>   bank_pair_cmd_cnt;   // added for SBR_WEIGHT_ENH_MODE=3/4
//    vector <pbr_weight> SbWeightOrder;
    uint8_t serial_cmd_cnt;
    uint8_t rwgrp_ch_cmd_cnt;
    uint8_t rankgrp_ch_cmd_cnt;
    vector <uint64_t> WriteResp;
    vector <uint64_t> ReadResp;
    vector <uint64_t> CmdResp;
//    vector<data_packet>read_data_buffer;
    TransactionCmd activate_cmd;
    bool core_concurr_en;
    uint8_t cmd_cycle;
    uint8_t pre_cycle;
    uint8_t rw_cycle;
    int CalcCmdCycle(uint8_t pre_cmd, uint8_t next_cmd);
    unsigned lru_arb(uint64_t index1, uint64_t index2, uint64_t sel);
    void update_lru (uint64_t index, uint64_t sel, Cmd *cmd);
    void LoadTfpw(uint8_t rank, unsigned tfpw);
    void LoadTfaw(uint8_t rank, unsigned tfaw, unsigned sc);
    void CheckFgRef(Cmd *c, unsigned bank, unsigned matgrp);

public:
    Cmd pre_rwcmd;
    Cmd *pregrt_cmd = NULL;
    Cmd prerw;
    uint64_t pre_req_time;          // time point for last cmd
    uint64_t pre_req_data_time;    // time point for last wdata 
    uint64_t rd_met_abr_cnt;
    uint64_t rd_met_pbr_cnt;
    unsigned pbr_bank_num;
    unsigned pbr_bg_num;
    unsigned pbr_sb_group_num;      //added for enhanced dbr
    unsigned pbr_sb_num;            //added for enhanced dbr
    unsigned sc_num;
    unsigned sc_bank_num;

    //V750
    unsigned arb_per_group;
    
    uint64_t pbr_block_allcmd_cycle;        // added for statistics of influence of enhanced dbr
    uint64_t pbr_cycle;                     // added for statistics of influence of enhanced dbr
    
    uint64_t pre_cmd_time;

    map<uint64_t, bool> rmw_rd_finish;
    std::map< uint64_t, task_info > tasks_info;
    std::map< uint64_t, uint32_t > wdata_info;
    void pushQosForSameMpamTrans(Transaction *trans);
    void pushQosForSameMidTrans(Transaction *trans);
    string log_path;
    void gen_rdata(uint64_t task, unsigned cnt, unsigned delay, bool mask_wcmd);
    void push_pending_TransactionQue(Transaction *trans);
    void rank_group_weight(unsigned * rank, unsigned * type);
    unsigned max_bl_data_size;
    unsigned min_bl_data_size;
    map <unsigned, unsigned> bl_data_size;
    vector<unsigned> rowconf_pre_cnt;
    vector<unsigned> pageto_pre_cnt;
    vector<unsigned> func_pre_cnt;
    vector<unsigned> rhit_break_pre_cnt;
    vector<unsigned> dummy_tout_pre_cnt;
    float calc_power();
    int bg_intlv_cnt;
    uint64_t pre_rdata_time;
    uint64_t pre_wresp_time;
    uint64_t pre_rresp_time;
    uint64_t pre_cresp_time;
    unsigned rw_cmd_num;
    unsigned act_cmd_num;
    unsigned tout_high_pri;
    unsigned GetRwRank(unsigned rank) {return (r_rank_cnt[rank] + w_rank_cnt[rank]);};
    vector <bool> has_wakeup;
    vector <bool> rank_has_cmd;
    vector <uint64_t> bank_idle_cnt;
    vector <uint64_t> bank_act_cnt;
    uint64_t rd_met_pd_cnt;
    uint64_t rd_met_asref_cnt;
    uint64_t cmd_met_pd_cnt;
    uint64_t cmd_met_asref_cnt;
    vector <uint32_t> rank_cnt_asref;
//    vector <uint32_t> rank_cnt_sbridle;
//    vector <bool> rank_send_pbr;
    vector < vector<uint32_t> > rank_cnt_sbridle;
    vector < vector<bool> > rank_send_pbr;
    uint32_t page_act_cnt;
    uint32_t page_rw_cnt;
    unsigned samerow_mask_rdcnt;
    unsigned samerow_mask_wrcnt;
    vector <unsigned> samerow_bit_rdcnt;
    vector <unsigned> samerow_bit_wrcnt;
    uint64_t pbr_cc_cnt;
    uint64_t abr_cc_cnt;
    bool sch_tout_cmd;
    TransactionType sch_tout_type;
    unsigned sch_tout_rank;
    unsigned trfcpb;
    unsigned trfcab;
    unsigned no_sch_cmd_en;
    unsigned no_sch_cmd_cnt;
    vector <bool> send_wckfs; // for CAS_FS
    unsigned PCFG_RANKTRTR_CASFS; // for CAS_FS
    unsigned PCFG_RANKTRTW_CASFS; // for CAS_FS
    unsigned PCFG_RANKTWTR_CASFS; // for CAS_FS
    unsigned PCFG_RANKTWTW_CASFS; // for CAS_FS
    unsigned casfs_cnt = 0; // for CAS_FS
    uint64_t casfs_time = 0; // for CAS_FS
    unsigned casfs_pre_rank = 0; // for CAS_FS
    vector <bool> pbr_hold_pre;
    vector <uint64_t> pbr_hold_pre_time;
    unsigned rw_exec_cnt;
    unsigned table_use_cnt;

    vector<bool> rank_cmd_high_qos;
    vector<unsigned> rank_rhit_num;
    unsigned com_highqos_read_cnt;
    uint64_t total_highqos_latency;
    vector<uint64_t> rank_total_highqos_latency;
    vector<uint32_t> rank_com_highqos_read_cnt;
    float ddrc_av_highqos_lat;
    vector<float> rank_ddrc_av_highqos_lat;
    vector<uint32_t> highqos_r_bank_cnt;
    unsigned highqos_max_delay;
    unsigned highqos_max_delay_id;
    vector <uint32_t> que_read_highqos_cnt;
    unsigned highqos_trig_grpsw_cnt;
    vector <uint64_t> sbr_gap_cnt;
    vector <unsigned> ref_offset;
    vector <unsigned> grp_sid_level;
    vector <unsigned> max_rcmd_bank;
    vector <unsigned> max_wcmd_bank;

    bool dropPreAct(Transaction *trans);
    map <uint64_t, bool> rdata_toggle_map;  // 用于merge rdata
};
}
#endif 
