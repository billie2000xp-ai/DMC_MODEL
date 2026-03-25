#include "Transaction.h"
#include "Macros.h"
#include <iomanip>
#include <math.h>
#include <assert.h>

using std::endl;
using std::hex;
using std::dec;
using namespace std;
namespace DRAMSim {

Transaction::Transaction() {
    reset();
}

void Transaction::reset() {
    transactionType = DATA_READ;
    nextCmd = INVALID;
    conflict_state = nullptr;
    wsram_idx = 0;
//    rmwState = QUE_WAITING;
    arb_time = 0;
    enter_que_time = 0;
    address = 0;
    timeAdded = 0;
    ptc_timeAdded = 0;
    time_timeout = 0;
    reqAddToDmcTime = 0.0;
    reqEnterDmcBufTime = 0.0;
    async_delay_time = 0.0;
    improve_cnt = 0;
    pri = 0;
    qos = 0;
    perf_qos = 0;
    perf_pri = 0;
    sid = 0;
    group = 0;
    rank = 0;
    sc = 0;
    bank = 0;
    row = 0;
    col = 0;
    group_ini = 0;
    bank_ini = 0;
    row_ini = 0;
    col_ini = 0;
    channel = 0;
    burst_length = 0;
    issue_size = 0;
    trans_size = 0;
    addr_col = 0;
    addr_col_ini = 0;
    bankIndex = 0;
    addrconf = false;
    perf_addrconf = true;
    task = 0x0;
    data_ready_cnt = 0;
    addr_block_source_id = 0;
    has_active = false;
    timeout_th = 0;
    pri_adapt_th = 0;
    timeout = false;
    timeout_type = 0xffffffff;
    adtimeout = false;
    dummy_timeout = false;
    dummy_type = 0xffffffff;
    dummy_ever = false;
    dummy_push = false;
    hqos_timeout = false;
    dummy_hqos = false;
    dummy_hqos_ever = false;
    hqos_push = false;
    mid = 0;
    mpam_id = 0;
    cmd_rt_type = false;
    cmd_hqos_type = false;
    pre_act = false;
    act_executing = false;
    has_send_act = false;
    ecc_flag = false;
    ap_cmd = false;
    mask_wcmd = false;
    wrap_cmd = false;
    pf_type = 0;
    sub_pftype = 0;
    sub_src = 0;
    bg_rotate_pri = 0;
    trcd_met = 0;
    bp_by_tout = false;
    bl = 0;
    arb_group = 0;
    fast_rd = false;
    act_only = false;
    byp_act = false;
    perf_rd_rowhit = false;
    perf_wr_rowhit = false;
    perf_rd_rowmiss = false;
    perf_wr_rowmiss = false;
    perf_rowhit_break = false;
    perf_maxbg_hit = false;
    perf_maxbank_hit = false;
    in_ptc = false;
    perf_slot_id = 0;
    state_pri = 0;
    adj_cmd_cnt = 0;
}

Transaction::Transaction(const Transaction &t) {
    transactionType = t.transactionType;
    nextCmd = t.nextCmd;
    conflict_state = t.conflict_state;
    arb_time = t.arb_time;
    enter_que_time = t.enter_que_time;
    address = t.address;
    timeAdded = t.timeAdded;
    ptc_timeAdded = t.ptc_timeAdded;
    time_timeout = t.time_timeout;
    inject_time = t.inject_time;
    reqAddToDmcTime = t.reqAddToDmcTime;
    reqEnterDmcBufTime = t.reqEnterDmcBufTime;
    async_delay_time = t.async_delay_time;
    improve_cnt = t.improve_cnt;
    pri = t.pri;
    qos = t.qos;
    perf_qos = t.perf_qos;
    perf_pri = t.perf_pri;
    sid = t.sid;
    group = t.group;
    rank = t.rank;
    sc = t.sc;
    bank = t.bank;
    row = t.row;
    col = t.col;
    group_ini = t.group_ini;
    bank_ini = t.bank_ini;
    row_ini = t.row_ini;
    col_ini = t.col_ini;
    channel = t.channel;
    data_size = t.data_size;
    burst_length = t.burst_length;
    issue_size = t.issue_size;
    trans_size = t.trans_size;
    addr_col = t.addr_col;
    addr_col_ini = t.addr_col_ini;
    bankIndex = t.bankIndex;
    addrconf = t.addrconf;
    perf_addrconf = t.perf_addrconf;
    task = t.task;
    data_ready_cnt = t.data_ready_cnt;
    addr_block_source_id = t.addr_block_source_id;
    has_active = t.has_active;
    timeout_th = t.timeout_th;
    pri_adapt_th = t.pri_adapt_th;
    timeout = t.timeout;
    timeout_type = t.timeout_type;
    adtimeout = t.adtimeout;
    dummy_timeout = t.dummy_timeout;
    dummy_type = t.dummy_type;
    dummy_ever = t.dummy_ever;
    dummy_push = t.dummy_push;
    hqos_timeout = t.hqos_timeout;
    dummy_hqos = t.dummy_hqos;
    dummy_hqos_ever = t.dummy_hqos_ever;
    hqos_push = t.hqos_push;
    mid = t.mid;
    mpam_id = t.mpam_id;
    cmd_rt_type = t.cmd_rt_type;
    cmd_hqos_type = t.cmd_hqos_type;
    pre_act = t.pre_act;
    act_executing = t.act_executing;
    has_send_act = t.has_send_act;
    ecc_flag = t.ecc_flag;
    ap_cmd = t.ap_cmd;
    mask_wcmd = t.mask_wcmd;
    wrap_cmd = t.wrap_cmd;
    pf_type = t.pf_type;
    sub_pftype = t.sub_pftype;
    sub_src = t.sub_src;
    bg_rotate_pri = t.bg_rotate_pri;
    trcd_met = t.trcd_met;
    bp_by_tout = t.bp_by_tout;
    bl = t.bl;
    arb_group = t.arb_group;
    fast_rd = t.fast_rd;
    act_only = t.act_only;
    byp_act = t.byp_act;
    perf_rd_rowhit = t.perf_rd_rowhit;
    perf_wr_rowhit = t.perf_wr_rowhit;
    perf_rd_rowmiss = t.perf_rd_rowmiss;
    perf_wr_rowmiss = t.perf_wr_rowmiss;
    perf_rowhit_break = t.perf_rowhit_break;
    perf_maxbg_hit = t.perf_maxbg_hit;
    perf_maxbank_hit = t.perf_maxbank_hit;
    in_ptc = t.in_ptc;
    perf_slot_id = t.perf_slot_id;
    state_pri = t.state_pri;
    adj_cmd_cnt = t.adj_cmd_cnt;
}
//add for rmw
Transaction::Transaction(const Transaction *t) {
    transactionType = t->transactionType;
    nextCmd = t->nextCmd;
    arb_time = t->arb_time;
    enter_que_time = t->enter_que_time;
    address = t->address;
    timeAdded = t->timeAdded;
    ptc_timeAdded = t->ptc_timeAdded;
    time_timeout = t->time_timeout;
    inject_time = t->inject_time;
    reqAddToDmcTime = t->reqAddToDmcTime;
    reqEnterDmcBufTime = t->reqEnterDmcBufTime;
    async_delay_time = t->async_delay_time;
    improve_cnt = t->improve_cnt;
    pri = t->pri;
    qos = t->qos;
    perf_qos = t->perf_qos;
    perf_pri = t->perf_pri;
    sid = t->sid;
    group = t->group;
    rank = t->rank;
    sc = t->sc;
    bank = t->bank;
    row = t->row;
    col = t->col;
    group_ini = t->group_ini;
    bank_ini = t->bank_ini;
    row_ini = t->row_ini;
    col_ini = t->col_ini;
    channel = t->channel;
    data_size = t->data_size;
    burst_length = t->burst_length;
    issue_size = t->issue_size;
    trans_size = t->trans_size;
    addr_col = t->addr_col;
    addr_col_ini = t->addr_col_ini;
    bankIndex = t->bankIndex;
    addrconf = t->addrconf;
    perf_addrconf = t->perf_addrconf;
    task = t->task;
    data_ready_cnt = t->data_ready_cnt;
    addr_block_source_id = t->addr_block_source_id;
    has_active = t->has_active;
    timeout_th = t->timeout_th;
    pri_adapt_th = t->pri_adapt_th;
    timeout = t->timeout;
    timeout_type = t->timeout_type;
    adtimeout = t->adtimeout;
    dummy_timeout = t->dummy_timeout;
    dummy_type = t->dummy_type;
    dummy_ever = t->dummy_ever;
    dummy_push = t->dummy_push;
    hqos_timeout = t->hqos_timeout;
    dummy_hqos = t->dummy_hqos;
    dummy_hqos_ever = t->dummy_hqos_ever;
    hqos_push = t->hqos_push;
    mid = t->mid;
    mpam_id = t->mpam_id;
    cmd_rt_type = t->cmd_rt_type;
    cmd_hqos_type = t->cmd_hqos_type;
    pre_act = t->pre_act;
    act_executing = t->act_executing;
    has_send_act = t->has_send_act;
    ecc_flag = t->ecc_flag;
    ap_cmd = t->ap_cmd;
    mask_wcmd = t->mask_wcmd;
    wrap_cmd = t->wrap_cmd;
    pf_type = t->pf_type;
    sub_pftype = t->sub_pftype;
    sub_src = t->sub_src;
    bg_rotate_pri = t->bg_rotate_pri;
    trcd_met = t->trcd_met;
    bp_by_tout = t->bp_by_tout;
    bl = t->bl;
    arb_group = t->arb_group;
    fast_rd = t->fast_rd;
    act_only = t->act_only;
    byp_act = t->byp_act;
    perf_rd_rowmiss = t->perf_rd_rowmiss;
    perf_wr_rowmiss = t->perf_wr_rowmiss;
    perf_rd_rowhit = t->perf_rd_rowhit;
    perf_wr_rowhit = t->perf_wr_rowhit;
    perf_rowhit_break = t->perf_rowhit_break;
    perf_maxbg_hit = t->perf_maxbg_hit;
    perf_maxbank_hit = t->perf_maxbank_hit;
    in_ptc = t->in_ptc;
    perf_slot_id = t->perf_slot_id;
    state_pri = t->state_pri;
    adj_cmd_cnt = t->adj_cmd_cnt;
}

Transaction::Transaction(const hha_command &c) {
    transactionType = (TransactionType)c.type;
    nextCmd = INVALID;
    conflict_state = nullptr;
    arb_time = 0;
    wsram_idx = 0;
    enter_que_time = 0;
    address = c.address;
    timeAdded = 0;
    ptc_timeAdded = 0;
    time_timeout = 0;
    reqAddToDmcTime = 0.0;
    reqEnterDmcBufTime = c.reqEnterDmcBufTime;
    async_delay_time = 0;
    improve_cnt = 0;
    pri = c.pri;
    qos = c.qos;
    perf_qos = c.perf_qos;
    perf_pri = c.perf_pri;
    sid = 0;
    group = 0;
    rank = 0;
    sc = 0;
    bank = 0;
    row = 0;
    col = 0;
    group_ini = 0;
    bank_ini = 0;
    row_ini = 0;
    col_ini = 0;
    channel = c.channel;
    burst_length = c.burst_length;
    issue_size = 0;
    trans_size = 0;
    addr_col = 0;
    addr_col_ini = 0;
    bankIndex = 0;
    addrconf = false;
    perf_addrconf = false;
    task = c.task;
    data_ready_cnt = 0;
    addr_block_source_id = 0;
    has_active = false;
    timeout = false;
    timeout_type = 0xffffffff;
    adtimeout = false;
    dummy_timeout = false;
    dummy_type = 0xffffffff;
    dummy_ever = false;
    dummy_push = false;
    hqos_timeout = false;
    dummy_hqos = false;
    dummy_hqos_ever = false;
    hqos_push =  false;
    timeout_th = 0;
    pri_adapt_th = 0;
    mid = c.mid;
    mpam_id = c.mpam_id;
    cmd_rt_type = c.cmd_rt_type;
    cmd_hqos_type = false;
    pre_act = c.pre_act;
    act_executing = false;
    has_send_act = false;
    ecc_flag = c.ecc_flag;
    ap_cmd = c.ap_cmd;
    mask_wcmd = c.mask_wcmd;
    wrap_cmd = c.wrap_cmd;
    pf_type = c.pf_type;
    sub_pftype = c.sub_pftype;
    sub_src = c.sub_src;
    bg_rotate_pri = 0;
    trcd_met = 0;
    bp_by_tout = false;
    bl = 0;
    arb_group = 0;
    fast_rd = false;
    act_only = false;
    byp_act = false;
    perf_rd_rowhit = false;
    perf_wr_rowhit = false;
    perf_rd_rowmiss = false;
    perf_wr_rowmiss = false;
    perf_rowhit_break = false;
    perf_maxbg_hit = false;
    perf_maxbank_hit = false;
    in_ptc = false;
    perf_slot_id = 0;
    state_pri = 0;
    adj_cmd_cnt = 0;
}

ostream &operator<<(ostream &os, const Transaction *trans) {
    if (trans->transactionType == DATA_READ)
        os<<"[Read] ";
    else if (trans->transactionType == DATA_WRITE)
        os<<"[Write] ";
    os << "task="<<trans->task<<", qos="
            <<trans->qos<<", rank="<<trans->rank<<", group="<<trans->group<<" ,bank="<<trans->bankIndex
            <<", row="<<trans->row;
    return os;
}

void Transaction::print_inputs(ostream &os,uint64_t clk) {
    if (transactionType == DATA_READ)
        os<<"[ R ] T["<<setw(8)<<dec<<clk<<"] " <<" A[" <<hex<<setw(10)<<address<<"] D["<<data_size
            << "] p["<<pri<<"] g["<<group<<"] r["<<rank<<"] b["<<bank<<"] row["<<hex<<setw(4)<<row<<"]"<<endl;
    else if (transactionType == DATA_WRITE)
        os<<"[ W ] T["<<setw(8)<<dec<<clk<<"] " <<" A[" <<hex<<setw(10)<<address<< "] D["<<data_size
            <<"] p["<<pri<< "] g["<<group<<"] r["<<rank<<"] b["<<bank<<"] row["<<hex<<setw(4)<<row<<"]"<<endl;
    os.flush();
}

void Transaction::print(ostream &os) {
    switch (transactionType) {
        case DATA_READ:
            os<<"BP [READ] pa[0x"<<hex<<address<<dec<<"] g["<<group<<"] r["<<rank
                <<"] b["<<bank<<"] row["<<row<<"] pri["<<pri<<"]"<<endl;
            break;
        case DATA_WRITE:
            os<<"BP [WRITE] pa[0x"<<hex<<address<<dec<<"] g["<<group<<"] r["<<rank
                <<"] b["<<bank<<"] row["<<row<<"] pri["<<pri<<"]"<<endl;
            break;
        default:
            ERROR("Trying to print unknown kind of bus packet");
            assert(0);
    }
    os.flush();
}
}