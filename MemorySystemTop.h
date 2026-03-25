#ifndef _MEMORYSYSTEMTOP_H_
#define _MEMORYSYSTEMTOP_H_

#include "SimulatorObject.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "IniReader.h"
#include "assert.h"
#include "Callback.h"
#include <memory>
#include <deque>
#ifdef SYSARCH_PLATFORM
#include "HALib/event.h"
#endif

namespace DRAMSim {

struct cmd_message {
    Transaction *trans;
    size_t delay;
    bool can_send;

    cmd_message() {
        trans = NULL;
        delay = 0;
        can_send = false;
    }
};

struct data_message {
    uint32_t *trans;
    size_t delay;
};

class MRAS;
class MPFQ;
class MPTC;

class MemorySystemTop : public SimulatorObject {
    vector<ofstream> DDRSim_log;
    vector<ofstream> DDRSim_iecc_log;
    vector<ofstream> DDRSim_pfq_log;
    vector<ofstream> DDRSim_ptc_log;
    vector<ofstream> state_log;
    vector<ofstream> trace_log;
    vector<ofstream> cmdnum_log;
    vector<vector<ofstream>> dram_log;
public:
#ifdef SYSARCH_PLATFORM
    MemorySystemTop(unsigned id, string IniFilePath = "./parameter", string LogPath = "./log",
            HALib::Configurable* cfg = NULL);
#else
    MemorySystemTop(unsigned id, string IniFilePath = "./parameter", string LogPath = "./log",
            int argc = 0, char *argv[] = NULL);
#endif
    virtual ~MemorySystemTop();
    bool addTransaction(const hha_command &command);
    void trans_init(Transaction *trans, uint64_t inject_time);
    void trans_check(Transaction *trans);
    void noc_read_inform(uint32_t channel, bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty);
    void update();
    void RegisterCallbacks(TransactionCompleteCB *readData,
            TransactionCompleteCB *writeDone,TransactionCompleteCB *readDone, TransactionCompleteCB *cmdDone);
    void InitOutputFiles(unsigned channel);
    bool addData(uint32_t *data, uint32_t channel, uint64_t id);

    uint32_t getDmcPressureLevel();
    uint32_t getTransQueSize(uint32_t dmc_id, bool isRd);
    uint32_t getRmwQueueCmdNum() const;

    void initialize();

    MRAS* getMRAS() const { return mras_.get(); }
    MPFQ* getMPFQ() const { return mpfq_.get(); }
    MPTC* getMPTC() const { return mptc_.get(); }

    //output file
    uint32_t curr_index;//
    uint32_t pre_index;//
    vector<uint32_t> clock_que;//
    vector<uint32_t> over_clock_que;//
    vector<uint32_t> start_clock_que;//
    vector<uint32_t> data_clk_que;//
    getfile param;//
    vector<cmd_message> transaction;//
    vector<data_message> data_que;//
    void dfs_backpress(uint32_t ch, bool backpress);
    void GetQueueCmdNum(uint32_t channel, unsigned *dmc_rd_num, unsigned *dmc_wr_num,
            unsigned *gbuf_rd_num, unsigned *gbuf_wr_num);
    void GetDmcBusyStatus(uint32_t channel, bool *dmc_busy);

    unsigned hhaId;
    string log_path;
    unsigned dmc_id;
    unsigned systemID;

    std::map<uint64_t,write_msg>write_map;
    vector<vector<Transaction *>> PreDmcPipeQueue;

    string dmc_log;
    // void update_print();

    ofstream inputs;
    ofstream ms_sim;
    uint64_t pre_rdata_receive_time;//
    uint64_t pre_rdata_receive_channel;//
    uint64_t start_cycle;
    uint64_t end_cycle;
    uint64_t flow_statis_start_cycle;
    uint64_t flow_statis_end_cycle;

    vector<uint32_t> curFlowPressureLevel;

    uint64_t totalBytes;//
    uint64_t totalWriteBytes;//
    uint64_t totalReadBytes;//
    vector<uint64_t> task_cnt;
    vector<uint64_t> total_task_cnt;
    bool enable_statistics;//
    vector<uint64_t> access_cnt;
    vector<uint64_t> bp_cnt;
    vector<uint64_t> total_access_cnt;
    vector<uint64_t> total_bp_cnt;
    
    //added for trans_fifo
    unsigned trans_fifo_data_cnt;//
    bool trans_fifo_full;//

    void statistics(uint32_t ch);
    float flowStatistic(uint32_t ch);
    uint32_t updateFlowState(float total_bw);

    // void addFastRead();
    // uint32_t getFlowPressureLevel();
    void GetQueueCmdNum(unsigned *ptc_rd_num, unsigned *ptc_wr_num,
            unsigned *pfq_rd_num, unsigned *pfq_wr_num);
    void GetDmcBusyStatus(bool *dmc_busy);
    void UnitConvert(double *oenergy, string *ouint, double ienergy);

    vector<vector<uint32_t>> que_cnt;
    vector<vector<uint32_t>> perf_que_cnt;
    vector<vector<uint32_t>> que_rd_cnt;
    vector<vector<uint32_t>> que_wr_cnt;
    vector<vector<uint32_t>> pre_acc_rank_cnt;
    vector<vector<uint32_t>> pre_acc_bank_cnt;
    vector<vector<uint32_t>> pre_racc_rank_cnt;
    vector<vector<uint32_t>> pre_racc_bank_cnt;
    vector<vector<uint32_t>> pre_wacc_rank_cnt;
    vector<vector<uint32_t>> pre_wacc_bank_cnt;

    uint32_t occ_1_cnt;
    uint32_t occ_2_cnt;
    uint32_t occ_3_cnt;
    uint32_t occ_4_cnt;
    vector<vector<unsigned>> pre_sch_level_cnt;

    void check_cnt(uint32_t ch);
    void perf_check_cnt(uint32_t ch);
    void register_read(uint64_t address, uint32_t &data);
    void register_write(uint64_t address, uint32_t data, uint32_t ch);

    vector<unsigned> pre_reads;
    vector<unsigned> pre_writes;
    vector<unsigned> pre_totals;
    vector<unsigned> pre_gbuf_reads;
    vector<unsigned> pre_gbuf_writes;
    vector<unsigned> pre_address_conf_cnt;
    vector<unsigned> pre_perf_address_conf_cnt;
    vector<unsigned> pre_id_conf_cnt;
    vector<unsigned> pre_ba_conf_cnt;
    vector<unsigned> pre_total_conf;
    vector<unsigned> pre_act_cnt;
    vector<unsigned> pre_act_dst_cnt;
    vector<unsigned> pre_pre_sb_cnt;
    vector<unsigned> pre_pre_pb_cnt;
    vector<unsigned> pre_pre_pb_dst_cnt;
    vector<unsigned> pre_pre_ab_cnt;
    vector<unsigned> pre_read_cnt;
    vector<unsigned> pre_write_cnt;
    vector<unsigned> pre_pde_cnt;
    vector<unsigned> pre_asrefe_cnt;
    vector<unsigned> pre_srpde_cnt;
    vector<unsigned> pre_pdx_cnt;
    vector<unsigned> pre_asrefx_cnt;
    vector<unsigned> pre_srpdx_cnt;
    vector<float> pre_power;
    vector<unsigned> PreBankRowActCnt;// Toconfirm

    vector<unsigned> pre_merge_read_cnt;
    vector<unsigned> pre_fast_read_cnt;
    vector<unsigned> pre_fast_act_cnt;
    vector<unsigned> pre_ecc_read_cnt;
    vector<unsigned> pre_ecc_write_cnt;

    vector<unsigned> Total_func_pre_cnt;
    vector<unsigned> Total_rhit_break_pre_cnt;
    vector<unsigned> Total_dummy_tout_pre_cnt;

    vector<unsigned> pre_read_p_cnt;
    vector<unsigned> pre_write_p_cnt;
    vector<unsigned> pre_mwrite_cnt;
    vector<unsigned> pre_mwrite_p_cnt;
    vector<unsigned> pre_timeout_cnt;
    vector<unsigned> pre_fast_timeout_cnt;
    vector<unsigned> pre_slow_timeout_cnt;
    vector<unsigned> pre_rt_timeout_cnt;
    vector<unsigned> pre_perf_rd_timeout_cnt;
    vector<unsigned> pre_perf_wr_timeout_cnt;
    vector<unsigned> pre_dummy_timeout_cnt;
    vector<unsigned> pre_dummy_hqos_cnt;
    vector<unsigned> pre_row_hit_cnt;
    vector<unsigned> pre_row_miss_cnt;
    vector<unsigned> pre_rw_switch_cnt;
    vector<unsigned> pre_rank_switch_cnt;
    vector<unsigned> pre_r_rank_switch_cnt;
    vector<unsigned> pre_w_rank_switch_cnt;
    vector<unsigned> pre_sid_switch_cnt;
    vector<unsigned> pre_rw_idle_cnt;
    vector<unsigned> pre_refresh_pb_cnt;
    vector<unsigned> pre_refresh_ab_cnt;
    vector<unsigned> pre_r2w_switch_cnt;
    vector<unsigned> pre_w2r_switch_cnt;
    vector<unsigned> pre_phy_notlp_cnt;
    vector<unsigned> pre_phy_lp_cnt;
    map <unsigned, unsigned> PreRdCntBl;
    map <unsigned, unsigned> PreWrCntBl;

    vector<vector<unsigned>> pre_perf2ptc_bank_rcnt;
    vector<vector<unsigned>> pre_perf2ptc_bank_wcnt;
    vector<vector<unsigned>> pre_perf2ptc_bank_cnt;
    vector<vector<unsigned>> pre_perf_bank_rcnt;
    vector<vector<unsigned>> pre_perf_bank_wcnt;
    vector<vector<unsigned>> pre_ptc_r_bank_cnt;
    vector<vector<unsigned>> pre_ptc_w_bank_cnt;

    vector<vector<vector<unsigned>>> pre_perf_bg_rcnt;
    vector<vector<vector<unsigned>>> pre_perf_bg_wcnt;
    vector<vector<vector<unsigned>>> pre_perf2ptc_bg_rcnt;
    vector<vector<vector<unsigned>>> pre_perf2ptc_bg_wcnt;
    vector<vector<vector<unsigned>>> pre_ptc_r_bg_cnt;
    vector<vector<vector<unsigned>>> pre_ptc_w_bg_cnt;

    vector<vector<unsigned>> trans_baintlv;

    vector<uint64_t> pre_cmd_in2dfi_lat;
    vector<uint64_t> pre_cmd_in2dfi_cnt;

    bool rd_one = true;//
    bool rd_two = false;//
    bool rd_three = false;//

    bool wr_one = true;//
    bool wr_two = false;//
    bool wr_three = false;//

    vector<uint64_t> pre_total_latency;
    vector<unsigned> pre_com_read_cnt;
    // unsigned channel;
    uint64_t channel_ohot;
    // void trans_init(Transaction *trans, uint64_t inject_time);
    // void trans_check(Transaction *trans);
    
    bool cmdlat_offset_en;

private:
    std::unique_ptr<MRAS> mras_;
    std::unique_ptr<MPFQ> mpfq_;
    std::unique_ptr<MPTC> mptc_;

    vector<hha_command> CommandDelay;
    string IniFilename;

    void command_check(const hha_command &c);
    uint8_t addr_map_ch(const hha_command &c);

    vector<vector<uint32_t>> prePdTime;
    vector<vector<uint32_t>> preAsrefTime;
    vector<vector<uint32_t>> preSrpdTime;
    vector<vector<uint32_t>> preWakeUpTime;
    vector<vector<bus_state>> BusStateAsync;
    vector<vector<vector<unsigned>>> pre_abr_cnt;
    vector<vector<unsigned>> pre_pbr_cnt;

#ifdef DDRC_NEED_DEBUG
    uint32_t test_mode;
    uint32_t cin_num;
    uint32_t send_cnt;
    bool fifo_not_empty;
#define FIFO_DEPTH (3)
#define EEROR_VALUE (3)
    vector<Transaction*>  TransactionFifo;
#endif
    };
}
#endif