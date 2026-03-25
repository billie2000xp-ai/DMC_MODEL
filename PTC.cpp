#include "PTC.h"
#include "PTC.h"
#include "Rank.h"
#include "PFQ.h"
#include "MPFQ.h"
#include "MemorySystemTop.h"


using namespace DRAMSim;
//==============================================================================
PTC::PTC(unsigned index, MemorySystemTop* top, ofstream &DDRSim_log_, ofstream &trace_log_, ofstream &cmdnum_log_): 
        id_(index), 
        memorySystemTop_(top),
        DDRSim_log(DDRSim_log_),
        trace_log(trace_log_), 
        cmdnum_log(cmdnum_log_) {
    //get handle on parent
    // parentMPTC = parent;
    log_path = memorySystemTop_->log_path;
    channel = memorySystemTop_->systemID * NUM_CHANS + memorySystemTop_->dmc_id;
    sub_cha = memorySystemTop_->dmc_id;
    channel_ohot = 1ull << channel;
    PostCalcTiming();
    //bus related fields
    cmdCyclesLeft = 0;
    //set here to avoid compile errors
    //reserve memory for vectors
    transactionQueue.reserve(TRANS_QUEUE_DEPTH);
    slt_valid.clear();
    for (size_t i = 0; i < TRANS_QUEUE_DEPTH; i ++) {
        slt_valid.push_back(false);
    }
    CmdQueue.reserve(TRANS_QUEUE_DEPTH);
    rw_cmdqueue.reserve(TRANS_QUEUE_DEPTH);
    act_cmdqueue.reserve(TRANS_QUEUE_DEPTH);
    pre_cmdqueue.reserve(TRANS_QUEUE_DEPTH);
    pending_TransactionQue.clear();
    WtransQueue.clear();
    totalTransactions = 0;
    RtCmdCnt = 0;
    totalReads = 0;
    totalWrites = 0;
    addrconf_cnt = 0;
    idconf_cnt = 0;
    baconf_cnt = 0;
    totalconf_cnt = 0;
    active_cnt = 0;
    bypass_active_cnt = 0;
    precharge_sb_cnt = 0;
    precharge_pb_cnt = 0;
    precharge_ab_cnt = 0;
    bs_max = 0;
    bs_cnt = 0;
    bs_total = 0;
    read_p_cnt = 0;
    write_p_cnt = 0;
    read_cnt = 0;
    write_cnt = 0;
    mwrite_cnt = 0;
    mwrite_p_cnt = 0;
    refresh_ab_cnt = 0;
    refresh_pb_cnt = 0;
    dmc_timeout_cnt = 0;
    dmc_fast_timeout_cnt = 0;
    dmc_slow_timeout_cnt = 0;
    com_read_cnt = 0;
    rw_switch_cnt = 0;
    r2w_switch_cnt = 0;
    w2r_switch_cnt = 0;
    rank_switch_cnt = 0;
    r_rank_switch_cnt = 0;
    w_rank_switch_cnt = 0;
    sid_switch_cnt = 0;
    pbr_overall_cnt = 0;
    page_exceed_cnt = 0;
    dresp_cnt = 0;
    pre_act_cnt = 0;
    total_pre_act_cnt = 0;
    fast_rd_cnt = 0;
    fast_act_cnt = 0;
    total_fast_rd_cnt = 0;
    total_fast_act_cnt = 0;
    total_fastrd_cancel_cnt = 0;
    perf_grpst_switch_cnt = 0;
    perf_state_pre = WBUFF_NO_GROUP;
    fast_wakeup.reserve(NUM_RANKS);
    fast_wakeup_cnt.reserve(NUM_RANKS);
    has_timeout_rank.reserve(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        RankStatus lp = RankStatus(i);
        lp.pd_cnt = tXP;
        RankState.push_back(lp);
        PdTime.push_back(0);
        AsrefTime.push_back(0);
        SrpdTime.push_back(0);
        WakeUpTime.push_back(0);
        PdEnterCnt.push_back(0);
        PdExitCnt.push_back(0);
        AsrefEnterCnt.push_back(0);
        AsrefExitCnt.push_back(0);
        SrpdEnterCnt.push_back(0);
        SrpdExitCnt.push_back(0);
        fast_wakeup.push_back(false);
        fast_wakeup_cnt.push_back(0);
        has_timeout_rank.push_back(false);
    }
    WdataPipe.clear();
    phy_lp_cnt = 0;
    rd_inc_cnt = 0;
    rd_wrap_cnt = 0;
    wr_inc_cnt = 0;
    wr_wrap_cnt = 0;
    rdata_cnt = 0;
    wdata_cnt = 0;
    phy_notlp_cnt = 0;
    dummy_timeout_cnt = 0;
    dummy_hqos_cnt = 0;
    rw_idle_cnt = 0;
    if (PD_ENABLE) {
        for (size_t i = 0; i < NUM_RANKS; i ++) RankState[i].lp_state = IDLE;
    }
    if (ASREF_ENABLE) {
        for (size_t i = 0; i < NUM_RANKS; i ++) RankState[i].asref_cnt = ASREF_PRD;
    }
    total_latency = 0;
    dly_ex2000_cnt = 0;
    writeDataToSend.reserve(TRANS_QUEUE_DEPTH);
    
    for (size_t i = 0; i < ARB_GROUP_NUM; i ++) {
        arb_group_pri_col.push_back(i);
        arb_group_pri_row.push_back(i);
    }

    for (size_t i = 0; i < ARB_GROUP_NUM; i ++) {
        arb_group_cnt.push_back(0);
    }

    pre_cmd_time = 0xFFFFFFFFFFFFFFFF;
    packet.clear();
    pre_req_time = 0xFFFFFFFFFFFFFFFF;
    pre_req_data_time = 0xFFFFFFFFFFFFFFFF;
    pre_rresp_time = 0xFFFFFFFFFFFFFFFF;
    pre_cresp_time = 0xFFFFFFFFFFFFFFFF;
    rd_met_abr_cnt = 0;
    rd_met_pbr_cnt = 0;
    pbr_block_allcmd_cycle = 0;
    pbr_cycle = 0;
    pre_dat_time = 0xFFFFFFFFFFFFFFFF;
    ptc_que_slot.clear();
    ptc_que_slot.resize(TRANS_QUEUE_DEPTH,0x0);
    bank_slot.clear();
    bank_slot.resize(BANK_SLOT_NUM,0xFFFF);
    acc_rank_cnt.clear();
    acc_rank_cnt.reserve(NUM_RANKS);
    acc_bank_cnt.clear();
    acc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    racc_rank_cnt.clear();
    racc_rank_cnt.reserve(NUM_RANKS);
    racc_bank_cnt.clear();
    racc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    wacc_rank_cnt.clear();
    wacc_rank_cnt.reserve(NUM_RANKS);
    wacc_bank_cnt.clear();
    wacc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    r_bank_cnt.clear();
    r_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    w_bank_cnt.clear();
    w_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    rank_cnt.clear();
    rank_cnt.reserve(NUM_RANKS);
    bank_cnt.clear();
    bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    ptc_r_bank_cnt.clear();
    ptc_r_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    ptc_w_bank_cnt.clear();
    ptc_w_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        for (size_t j = 0; j < NUM_BANKS; j ++) {
            ptc_r_bank_cnt.push_back(0);
            ptc_w_bank_cnt.push_back(0);
        }
    } 

    deqCmdWakeupLp.clear();
    deqCmdWakeupLp.resize(NUM_RANKS);
    r_rank_cnt.clear();
    r_rank_cnt.reserve(NUM_RANKS);
    w_rank_cnt.clear();
    w_rank_cnt.reserve(NUM_RANKS);
    r_qos_cnt.clear();
    r_qos_cnt.reserve(8);
    w_qos_cnt.clear();
    w_qos_cnt.reserve(8);
    active_cmd_cnt.clear();
    active_cmd_cnt.reserve(NUM_BANKS * NUM_RANKS);
    BankRowActCnt.clear();
    BankRowActCnt.reserve(NUM_BANKS * NUM_RANKS);
    min_delay = 0;
    max_delay = 0;
    min_delay_id = 0;
    max_delay_id = 0;
    r_rank_bst.clear();
    r_rank_bst.reserve(NUM_RANKS);
    r_rank_mux.clear();
    r_rank_mux.reserve(NUM_RANKS);
    bank_cnt_ehs.clear();
    bank_cnt_ehs.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < 8; i ++) {
        qos_delay_cnt.push_back(0);
        qos_cnt.push_back(0);
        qos_timeout_cnt.push_back(0);
        qos_fast_timeout_cnt.push_back(0);
        qos_slow_timeout_cnt.push_back(0);
        r_qos_cnt.push_back(0);
        w_qos_cnt.push_back(0);
    }
    for (size_t i = 0; i < MidMax; i ++) {
        mid_delay_cnt.push_back(0);
        mid_cnt.push_back(0);
    }
    for (size_t i = 0; i < 4; i ++) {
        pf_delay_cnt.push_back(0);
        pf_cnt.push_back(0);
    }
    qos_level_cnt.resize(8);
    for (auto &level_cnt : qos_level_cnt) {
        level_cnt.resize(9);
    }
    lat_dly_cnt.resize(40);
    lat_dly_step.resize(40);
    lat_dly_step[0] = 10; lat_dly_step[1] = 20; lat_dly_step[2] = 30;
    lat_dly_step[3] = 40; lat_dly_step[4] = 50; lat_dly_step[5] = 60;
    lat_dly_step[6] = 70; lat_dly_step[7] = 80; lat_dly_step[8] = 90;
    lat_dly_step[9] = 100; lat_dly_step[10] = 110; lat_dly_step[11] = 120;
    lat_dly_step[12] = 130; lat_dly_step[13] = 140; lat_dly_step[14] = 150;
    lat_dly_step[15] = 160; lat_dly_step[16] = 170; lat_dly_step[17] = 180;
    lat_dly_step[18] = 190; lat_dly_step[19] = 200; lat_dly_step[20] = 210;
    lat_dly_step[21] = 220; lat_dly_step[22] = 230; lat_dly_step[23] = 240;
    lat_dly_step[24] = 250; lat_dly_step[25] = 260; lat_dly_step[26] = 270;
    lat_dly_step[27] = 280; lat_dly_step[28] = 290; lat_dly_step[29] = 300;
    lat_dly_step[30] = 400; lat_dly_step[31] = 500; lat_dly_step[32] = 600;
    lat_dly_step[33] = 700; lat_dly_step[34] = 800; lat_dly_step[35] = 900;
    lat_dly_step[36] = 1000; lat_dly_step[37] = 1500; lat_dly_step[38] = 2000;
    lat_dly_step[39] = 10000;

    ddrc_av_lat = 0;

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        rank_cnt.push_back(0);
        r_rank_cnt.push_back(0);
        w_rank_cnt.push_back(0);
        ddrc_av_lat_rank.push_back(0);
        total_latency_rank.push_back(0);
        com_read_cnt_rank.push_back(0);
        r_rank_bst.push_back(0);
        w_rank_bst.push_back(0);
        r_rank_mux.push_back(NULL);
        w_rank_mux.push_back(NULL);
        for (size_t j = 0; j < tCMD_WAKEUP; j ++) {
            deqCmdWakeupLp[i].push_back(0);
        }
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        if (GRP_RANK_MODE == 0) {
            r_rank_mux[i] = &r_rank_cnt[i];
            w_rank_mux[i] = &w_rank_cnt[i];
        } else if (GRP_RANK_MODE == 1) {
            r_rank_mux[i] = &r_rank_bst[i];
            w_rank_mux[i] = &w_rank_bst[i];
        }
    }

    issue_state.clear();
    issue_state.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i< NUM_RANKS; i ++) {
        for (size_t j = 0; j < NUM_BANKS; j ++){
            issue_state.push_back(false);
        }
    }


    arb_per_group = TRANS_QUEUE_DEPTH / ARB_GROUP_NUM;

    if (IS_LP5 || IS_LP6) {
        pbr_bg_num = 2;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE1_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_LP4) {
        pbr_bg_num = 1;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE1_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_HBM2E || IS_HBM3) {
        pbr_bg_num = 1;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE1_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_DDR5 || IS_DDR4) {
        pbr_bg_num = NUM_GROUPS; //pbr bg number of bank
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE2_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    }

    //added for SBR_WEIGHT_ENH_MODE (new stratege)   
    pre_enh_pbr_bagroup.clear();
    pre_enh_pbr_bagroup.reserve(NUM_RANKS);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++){
        pre_enh_pbr_bagroup.push_back(0xFFFFFFFF);
    }

    pre_sch_bankIndex.clear();
    pre_sch_bankIndex.reserve(NUM_RANKS);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++){
        pre_sch_bankIndex.push_back(0xFFFFFFFF);
    }

    refreshALL.clear();
    refreshALL.reserve(NUM_RANKS);
    rank_refresh_cnt.clear();
    rank_refresh_cnt.reserve(NUM_RANKS);
    refresh_pbr_has_finish.clear();
    refresh_pbr_has_finish.reserve(NUM_RANKS);
    force_pbr_refresh.clear();
    force_pbr_refresh.reserve(NUM_RANKS);
    forceRankBankIndex.clear();
    forceRankBankIndex.reserve(NUM_RANKS);
    refresh_cnt_pb.clear();
    refresh_cnt_pb.reserve(NUM_RANKS);
    sc_cnt.clear();
    sc_cnt.reserve(NUM_RANKS);
    rank_cnt_sbridle.clear();
    rank_cnt_sbridle.reserve(NUM_RANKS);
    rank_send_pbr.clear();
    rank_send_pbr.reserve(NUM_RANKS);
    bank_pair_cmd_cnt.clear();
    bank_pair_cmd_cnt.reserve(NUM_RANKS);
    Wcmd_Dist.resize(NUM_RANKS,vector<unsigned>(NUM_BANKS,0));
    Rcmd_Dist.resize(NUM_RANKS,vector<unsigned>(NUM_BANKS,0));

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        bank_pair_cmd_cnt.push_back(vector<unsigned>());
        if (ENH_PBR_EN) {
            for (size_t bank = 0; bank < pbr_sb_group_num; bank ++){
                for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++){
                    for (size_t sb_bank_tmp = (sb_bank+1); sb_bank_tmp < pbr_sb_num; sb_bank_tmp ++){
                        bank_pair_cmd_cnt[i].push_back(0);
                    }
                }
            }
        } else {
            for (size_t j = 0; j < sc_num * pbr_bank_num ; j ++) {
                bank_pair_cmd_cnt[i].push_back(0);
            }
        }
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank++) {
        refreshALL.push_back(vector<FORALLREFRESH>());
        for (size_t sub_ch = 0; sub_ch < sc_num; sub_ch ++) {
            FORALLREFRESH far = FORALLREFRESH(rank);
            refreshALL[rank].push_back(far);
        }
    }

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        rank_refresh_cnt.push_back(vector<unsigned>());
        refresh_pbr_has_finish.push_back(vector<bool>());
        force_pbr_refresh.push_back(vector<bool>());
        forceRankBankIndex.push_back(vector<unsigned>());
        refresh_cnt_pb.push_back(vector<unsigned>());
        sc_cnt.push_back(vector<uint32_t>());
        rank_cnt_sbridle.push_back(vector<uint32_t>());
        rank_send_pbr.push_back(vector<bool>());
        for (size_t j = 0; j < sc_num ; j ++) {
//            refreshALL[i].push_back(i);        // todo, revise initialization
            rank_refresh_cnt[i].push_back(0);
            refresh_pbr_has_finish[i].push_back(false);
            force_pbr_refresh[i].push_back(false);
            forceRankBankIndex[i].push_back(0);
            refresh_cnt_pb[i].push_back(0);
            sc_cnt[i].push_back(0);
            rank_cnt_sbridle[i].push_back(0);
            rank_send_pbr[i].push_back(SBR_IDLE_ADAPT_EN);
        }
    }

    NUM_MATGRPS = 1;

    refreshPerBank.clear();
    refreshPerBank.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
        refreshPerBank.push_back(i);
        bank_cnt.push_back(0);
    }

    perbank_refresh_cnt.clear();
    if (ENH_PBR_EN) {
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            perbank_refresh_cnt.push_back(0);
        }
    } else {
        for (size_t i = 0; i < NUM_RANKS * pbr_bank_num * sc_num; i ++) {   // todo: revise for e-mode
            perbank_refresh_cnt.push_back(0);
        }
    }

    PbrWeight.resize(NUM_RANKS);
    SbGroupWeight.resize(NUM_RANKS);
    SbWeight.resize(NUM_RANKS);
    
    if(ENH_PBR_EN){
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                pbr_weight pbr = pbr_weight(bank, bank, bank);
                PbrWeight[rank].push_back(pbr);
            }
        }
    } else {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            for (size_t bank = 0; bank < pbr_bank_num * sc_num * NUM_MATGRPS; bank ++) {
                pbr_weight pbr = pbr_weight(bank, bank, bank);
                PbrWeight[rank].push_back(pbr);
            }
        }
    }

    // sgw : samebank group weight
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++) {
            pbr_weight sgw = pbr_weight(bank, bank, bank);
            SbGroupWeight[rank].push_back(sgw);
        }
    }
    
    // initialization for weight of 24 bank bank pairs: 
    // (0,4),(0,8),(0,12),(4,8),(4,12),(8,12),(1,5),(1,9),(1,13),(5,9),(5,13),(9,13)
    // (2,6),(2,10),(2,14),(6,10),(6,14),(10,14),(3,7),(3,11),(3,15),(7,11),(7,15),(11,15)
    // sw: samebank weight
//    unsigned sb_cnt = 0;
    ptc_lru_matrix.resize(TRANS_QUEUE_DEPTH, vector<uint32_t>(TRANS_QUEUE_DEPTH, 0));
    for (size_t i = 0; i < TRANS_QUEUE_DEPTH; i ++) {
        for (size_t j = 0; j < TRANS_QUEUE_DEPTH; j ++) {
            if (i < j) ptc_lru_matrix[i][j] = 1;
            else ptc_lru_matrix[i][j] = 0;
        }

    }

    bank_lru_matrix.resize((NUM_BANKS*NUM_RANKS), vector<uint32_t>((NUM_BANKS*NUM_RANKS), 0));
    for (size_t i = 0; i < (NUM_BANKS*NUM_RANKS); i ++) {
        for (size_t j = 0; j < (NUM_BANKS*NUM_RANKS); j ++) {
            if (i < j) bank_lru_matrix[i][j] = 1;
            else bank_lru_matrix[i][j] = 0;
        }
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++){
            for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++){
                unsigned sb_bank_idx = rank * NUM_BANKS + bank + sb_bank*pbr_sb_num; 
                for (size_t sb_bank_tmp = (sb_bank+1); sb_bank_tmp < pbr_sb_num; sb_bank_tmp ++){
                    unsigned sb_bank_tmp_idx = rank * NUM_BANKS + bank + sb_bank_tmp*pbr_sb_num; 
                    pbr_weight sw = pbr_weight(sb_bank_idx, sb_bank_idx, sb_bank_tmp_idx);
                    SbWeight[rank].push_back(sw);
                }
            }
        }
    }

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        r_bg_cnt.push_back(vector<unsigned>());
        w_bg_cnt.push_back(vector<unsigned>());
        bg_cnt.push_back(vector<unsigned>());
        ptc_r_bg_cnt.push_back(vector<unsigned>());
        ptc_w_bg_cnt.push_back(vector<unsigned>());
        r_sid_cnt.push_back(vector<unsigned>());
        w_sid_cnt.push_back(vector<unsigned>());
        sid_cnt.push_back(vector<unsigned>());
        for (size_t j = 0; j < NUM_GROUPS; j ++) {
            r_bg_cnt[i].push_back(0);
            w_bg_cnt[i].push_back(0);
            bg_cnt[i].push_back(0);
            ptc_r_bg_cnt[i].push_back(0);
            ptc_w_bg_cnt[i].push_back(0);
        }
        for (size_t j = 0; j < NUM_SIDS; j ++) {
            r_sid_cnt[i].push_back(0);
            w_sid_cnt[i].push_back(0);
            sid_cnt[i].push_back(0);
        }
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        acc_rank_cnt.push_back(0);
        racc_rank_cnt.push_back(0);
        wacc_rank_cnt.push_back(0);
        for (size_t j = 0; j < NUM_BANKS; j ++) {
            r_bank_cnt.push_back(0);
            w_bank_cnt.push_back(0);
            acc_bank_cnt.push_back(0);
            racc_bank_cnt.push_back(0);
            wacc_bank_cnt.push_back(0);
            active_cmd_cnt.push_back(0);
            BankRowActCnt.push_back(0);
            bank_cnt_ehs.push_back(0);
        }
    }
    ehs_page_adapt_cnt.resize(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        ehs_page_adapt_cnt[i].resize(MAP_CONFIG["ENH_PAGE_ADPT_TIME"].size());
    }
    access_bank_delay.clear();
    access_bank_delay.reserve(NUM_BANKS * NUM_RANKS);
    bank_cas_delay.clear();
    bank_cas_delay.reserve(NUM_BANKS * NUM_RANKS);
    page_timeout_rd.clear();
    page_timeout_rd.reserve(NUM_BANKS * NUM_RANKS);
    page_timeout_wr.clear();
    page_timeout_wr.reserve(NUM_BANKS * NUM_RANKS);
    page_cmd_cnt.clear();
    page_cmd_cnt.reserve(NUM_BANKS * NUM_RANKS);
    COUNTER cnt;
    cnt.enable = false;
    cnt.cnt = 0;
    que_read_cnt = 0;
    que_write_cnt = 0;
    dmc_cmd_cnt = 0;
    arb_enable = true;
    total_iecc_cnt = 0;
    total_noiecc_cnt = 0;
    even_cycle = false;
    odd_cycle = false;
    PreCmdTime = 0xffffffff;
    has_other_rank_cmd = false;
    has_hqos_rank_rcmd = false;
    hqos_rank = 0xffffffff;
    all_rank_has_cmd = true;

    rowconf_pre_cnt.clear();
    pageto_pre_cnt.clear();
    func_pre_cnt.clear();
    rhit_break_pre_cnt.clear();
    dummy_tout_pre_cnt.clear();
    act_executing.clear();
    rowconf_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    pageto_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    func_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    rhit_break_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    dummy_tout_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    act_executing.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        access_bank_delay.push_back(cnt);
        bank_cas_delay.push_back(0);
        page_timeout_rd.push_back(OPENPAGE_TIME_RD);
        page_timeout_wr.push_back(OPENPAGE_TIME_WR);
        page_cmd_cnt.push_back(0);
        rowconf_pre_cnt.push_back(0);
        pageto_pre_cnt.push_back(0);
        func_pre_cnt.push_back(0);
        rhit_break_pre_cnt.push_back(0);
        dummy_tout_pre_cnt.push_back(0);
        act_executing.push_back(false);
    }
    adpt_openpage_time = OPENPAGE_TIME_RD;

    page_timeout_window[0] = 1; page_timeout_window[1] = 5; page_timeout_window[2] = 10;
    page_timeout_window[3] = 20; page_timeout_window[4] = 30; page_timeout_window[5] = 40;
    page_timeout_window[6] = 50; page_timeout_window[7] = 60; page_timeout_window[8] = 70;
    page_timeout_window[9] = 80; page_timeout_window[10] = 90; page_timeout_window[11] = 100;
    page_timeout_window[12] = 110; page_timeout_window[13] = 120; page_timeout_window[14] = 130;
    page_timeout_window[15] = 140; page_timeout_window[16] = 150; page_timeout_window[17] = 160;
    page_timeout_window[18] = 170; page_timeout_window[19] = 180; page_timeout_window[20] = 190;
    page_timeout_window[21] = 200; page_timeout_window[22] = 210; page_timeout_window[23] = 220;
    page_timeout_window[24] = 230; page_timeout_window[25] = 240; page_timeout_window[26] = 250;
    page_timeout_window[27] = 260; page_timeout_window[28] = 270; page_timeout_window[29] = 280;
    page_timeout_window[30] = 290; page_timeout_window[31] = 300; page_timeout_window[32] = 400;
    page_timeout_window[33] = 500;
    page_row_hit.resize(NUM_BANKS*NUM_RANKS);
    for (auto &ele : page_row_hit)
        ele.resize(sizeof(page_timeout_window)/sizeof(page_timeout_window[0]));

    page_row_miss.resize(NUM_BANKS * NUM_RANKS);
    for (auto &ele : page_row_miss)
        ele.resize(sizeof(page_timeout_window)/sizeof(page_timeout_window[0]));

    page_row_conflict.resize(NUM_BANKS * NUM_RANKS);
    for (auto &ele : page_row_conflict)
        ele.resize(sizeof(page_timeout_window)/sizeof(page_timeout_window[0]));

    cmd_in2dfi_lat = 0;
    cmd_in2dfi_cnt = 0;
    cmd_rdmet_cnt = 0;
    forward_64B_cnt = 0;
    forward_128B_cnt = 0;
    //staggers when each rank is due for a refresh
    TotalBytes = 0;
    TotalReadBytes = 0;
    TotalWriteBytes = 0;
    DmcTotalBytes = 0;
    DmcTotalReadBytes = 0;
    DmcTotalWriteBytes = 0;
    flowStatisTotalBytes = 0;

    tFPWCountdown.reserve(NUM_RANKS);
    tFAWCountdown.reserve(NUM_RANKS);
    tFAWCountdown_sc1.reserve(NUM_RANKS);
    has_wakeup.reserve(NUM_RANKS);
    rank_has_cmd.reserve(NUM_RANKS);
    bank_idle_cnt.reserve(NUM_RANKS);
    bank_act_cnt.reserve(NUM_RANKS);
    rank_cnt_asref.reserve(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        //init the empty vectors here so we don't seg fault later
        tFAWCountdown.push_back(vector<unsigned>());
        tFAWCountdown_sc1.push_back(vector<unsigned>());
        tFPWCountdown.push_back(vector<unsigned>());
        FuncState state;
        funcState.push_back(state);
        TotalBytesRank.push_back(0);
        has_wakeup.push_back(false);
        rank_has_cmd.push_back(false);
        bank_idle_cnt.push_back(0);
        bank_act_cnt.push_back(0);
        rank_cnt_asref.push_back(0);
    }
    exec_valid = false;

    for (size_t r = 0; r < NUM_RANKS; r++) {
        for (size_t s = 0; s < NUM_SIDS; s++) {
            for (size_t g = 0; g < NUM_GROUPS; g++) {
                for (size_t b = 0; b < NUM_BANKS / NUM_SIDS / NUM_GROUPS; b++) {
                    BankTableState state = BankTableState(DDRSim_log,r,s,g,b);
                    bankStates.push_back(state);
                }
            }
        }
    }

    tasks_info.clear();
    wdata_info.clear();
    rmw_rd_finish.clear();

    occ = 0;
    availability = 0;
    sum_avai = 0;
    sum_pwr_avai = 0;
    row_hit_ratio = 0;
    occ_1_cnt = 0;
    occ_2_cnt = 0;
    occ_3_cnt = 0;
    occ_4_cnt = 0;
    bw_totalcmds = 0;
    bw_totalwrites = 0;
    bw_totalreads = 0;
    ecc_total_bytes = 0;
    ecc_total_reads = 0;
    ecc_total_writes = 0;
    rmw_total_bytes = 0;
    rmw_total_reads = 0;
    TotalDmcBytes = 0;
    TotalDmcRdBytes = 0;
    TotalDmcWrBytes = 0;
    TotalDmcRd32B = 0;
    TotalDmcRd64B = 0;
    TotalDmcRd128B = 0;
    TotalDmcRd256B = 0;
    TotalDmcWr32B = 0;
    TotalDmcWr64B = 0;
    TotalDmcWr128B = 0;
    TotalDmcWr256B = 0;
    // pfq = NULL;
    // iecc = new Inline_ECC(parentMPTC,channel,DDRSim_log);
    avai_sqrt = 0;
    rw_group_state.reserve(8);
    for (size_t i = 0; i < 8; i ++) rw_group_state.push_back(NO_GROUP);
    in_write_group = false; // true is write group, false is read group
    rk_grp_state = NO_RGRP;
    real_rk_grp_state = NO_RGRP;
    serial_cmd_cnt = 0x0;
    rwgrp_ch_cmd_cnt = 0x0;
    rankgrp_ch_cmd_cnt = 0x0;
    no_sch_cmd_en = false;
    no_sch_cmd_cnt = 0x0;
    if (IS_LP4) {
        cmd_cycle = ceil(2 / WCK2DFI_RATIO);
        pre_cycle = ceil(2 / WCK2DFI_RATIO);
        rw_cycle = ceil(4 / WCK2DFI_RATIO);
    } else if (IS_LP6) {
        cmd_cycle = ceil(2 * OFREQ_RATIO);
        pre_cycle = ceil(2 * OFREQ_RATIO);
        rw_cycle = ceil(2 * OFREQ_RATIO);
    } else if (IS_LP5 || IS_DDR5 || IS_DDR4) {
        cmd_cycle = ceil(OFREQ_RATIO);
        pre_cycle = ceil(OFREQ_RATIO);
        rw_cycle = ceil(OFREQ_RATIO);
    } else if (IS_HBM2E || IS_HBM3) {
        cmd_cycle = ceil(4 / WCK2DFI_RATIO);
        pre_cycle = ceil(1 / WCK2DFI_RATIO);
        rw_cycle = ceil(1 / WCK2DFI_RATIO);
    }

    BLEN = MAP_CONFIG["BL"][0];
    bl_data_size.clear();
    uint8_t bl_size = MAP_CONFIG["BL"].size();
    for (size_t i = 0; i < bl_size; i ++) {
        unsigned bl = MAP_CONFIG["BL"][i];
        if (IS_LP6){
            bl_data_size[bl] = bl * JEDEC_DATA_BUS_BITS * PAM_RATIO / 9;
        } else {
            bl_data_size[bl] = bl * JEDEC_DATA_BUS_BITS * PAM_RATIO / 8;
        }
    }

    if (IS_LP6) {
        max_bl_data_size = BLEN * JEDEC_DATA_BUS_BITS * PAM_RATIO / 9;
        min_bl_data_size = MAP_CONFIG["BL"][bl_size - 1] * JEDEC_DATA_BUS_BITS * PAM_RATIO / 9;
    } else {
        max_bl_data_size = BLEN * JEDEC_DATA_BUS_BITS * PAM_RATIO / 8;
        min_bl_data_size = MAP_CONFIG["BL"][bl_size - 1] * JEDEC_DATA_BUS_BITS * PAM_RATIO / 8;
    }

    bp_cycle.clear();
    bp_step.clear();
    if (NUM_GROUPS > 1) {
        if (IS_LP5) {
            if (WCK2DFI_RATIO == 4) {
                for (size_t i = 3; i <= 5; i ++) bp_step.push_back(i);
            } else if (WCK2DFI_RATIO == 2) {
                for (size_t i = 5; i <= 11; i ++) bp_step.push_back(i);
            } else {
                ERROR(setw(10)<<now()<<" -- Error WCK2DFI_RATIO: "<<WCK2DFI_RATIO);
                assert(0);
            }
        } else if (IS_LP6) {
            if (WCK2DFI_RATIO == 4) {
                for (size_t i = 4; i <= 8; i ++) bp_step.push_back(i);
            } else if (WCK2DFI_RATIO == 2) {
                for (size_t i = 7; i <= 17; i ++) bp_step.push_back(i);
            } else {
                ERROR(setw(10)<<now()<<" -- Error WCK2DFI_RATIO: "<<WCK2DFI_RATIO);
                assert(0);
            }
        }
    }

    RdCntBl.clear();   WrCntBl.clear();
    RdCntBl[BL8] = 0;  WrCntBl[BL8] = 0;
    RdCntBl[BL16] = 0; WrCntBl[BL16] = 0;
    RdCntBl[BL24] = 0; WrCntBl[BL24] = 0;
    RdCntBl[BL32] = 0; WrCntBl[BL32] = 0;
    RdCntBl[BL48] = 0; WrCntBl[BL48] = 0;
    RdCntBl[BL64] = 0; WrCntBl[BL64] = 0;

    dfs_backpress_en = false;
    total_dfs_bp_cnt = 0;

    if (NUM_GROUPS > 1) {
        if (IS_LP5) {
            BL_n_min[BL16] = unsigned(ceil(float(BL16) / WCK2DFI_RATIO / 2));
            BL_n_max[BL16] = unsigned(ceil(float(BL16) / WCK2DFI_RATIO));
            BL_n_min[BL32] = unsigned(ceil(1.5 * float(BL32) / WCK2DFI_RATIO / 2));
            BL_n_max[BL32] = unsigned(ceil(float(BL32) / WCK2DFI_RATIO));
        } else if (IS_LP6) {
            BL_n_min[BL24] = unsigned(ceil(float(BL24) / WCK2DFI_RATIO / 2));
            BL_n_max[BL24] = unsigned(ceil(float(BL24) / WCK2DFI_RATIO));
            BL_n_min[BL48] = unsigned(ceil(1.5 * float(BL48) / WCK2DFI_RATIO / 2));
            BL_n_max[BL48] = unsigned(ceil(float(BL48) / WCK2DFI_RATIO));
        }
    } else {
        for (auto &it : MAP_CONFIG["BL"]) {
            BL_n_min[it] = ceil(float(it) / WCK2DFI_RATIO / 2);
            BL_n_max[it] = ceil(float(it) / WCK2DFI_RATIO / 2);
        }
    }

    if (PRINT_CMD_NUM) {
        CMDNUM_PRINT("DFI Cycle | ");
        CMDNUM_PRINT("            DMC Read           ||");
        CMDNUM_PRINT("            DMC Write          ||");
        CMDNUM_PRINT("           Gbuf Read           ||");
        CMDNUM_PRINT("           Gbuf Write          ||");
        CMDNUM_PRINT(endl);
        CMDNUM_PRINT("Qos       | ");
        for (size_t i = 0; i < 4; i ++) {
            for (size_t j = 0; j < 8; j ++) {
                CMDNUM_PRINT(setw(3)<<j<<"|");
            }
            CMDNUM_PRINT("|");
        }
        CMDNUM_PRINT(endl);
    }

    for (size_t i = 0; i <= TRANS_QUEUE_DEPTH; i ++) {
        que_cmd_time.push_back(0);
    }
    bg_intlv_cnt = 0;
    pre_rdata_time = 0xFFFFFFFFFFFFFFFF;
    pre_wresp_time = 0xFFFFFFFFFFFFFFFF;
    core_concurr_en = false;
    rw_cmd_num = 0;
    act_cmd_num = 0;
    tout_high_pri = 0xFFFF;
    rd_met_pd_cnt = 0;
    rd_met_asref_cnt = 0;
    cmd_met_pd_cnt = 0;
    cmd_met_asref_cnt = 0;
    page_act_cnt = 0;
    page_rw_cnt = 0;
    samerow_mask_rdcnt = 0;
    samerow_mask_wrcnt = 0;
    for (size_t i = 0; i < 32; i ++) {
        samerow_bit_rdcnt.push_back(0);
        samerow_bit_wrcnt.push_back(0);
    }
    pbr_cc_cnt = 0;
    abr_cc_cnt = 0;
    has_bypact_exec = false;
    sch_tout_cmd = false;
    sch_tout_type = DATA_READ;
    sch_tout_rank = 0;
    opc_cnt = 0;
    ppc_cnt = 0;
    rw_exec_cnt = 0;
    table_use_cnt = 0;
    page_adpt_win_cnt = 0;
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        send_wckfs.push_back(false);
        pbr_hold_pre.push_back(false);
        pbr_hold_pre_time.push_back(0);
        max_rcmd_bank.push_back(0);
        max_wcmd_bank.push_back(0);
    }

    com_highqos_read_cnt = 0;
    total_highqos_latency = 0;
    highqos_max_delay = 0;
    highqos_max_delay_id = 0;
    ddrc_av_highqos_lat = 0;
    highqos_trig_grpsw_cnt = 0;
    que_read_highqos_cnt.reserve(NUM_RANKS);
    rank_cmd_high_qos.reserve(NUM_RANKS);
    rank_rhit_num.reserve(NUM_RANKS);
    rank_ddrc_av_highqos_lat.reserve(NUM_RANKS);
    rank_total_highqos_latency.reserve(NUM_RANKS);
    rank_com_highqos_read_cnt.reserve(NUM_RANKS);
    highqos_r_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    sbr_gap_cnt.reserve(NUM_RANKS);
    ref_offset.reserve(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        que_read_highqos_cnt.push_back(0);
        rank_cmd_high_qos.push_back(false);
        rank_rhit_num.push_back(0);
        rank_ddrc_av_highqos_lat.push_back(0);
        rank_total_highqos_latency.push_back(0);
        rank_com_highqos_read_cnt.push_back(0);
        sbr_gap_cnt.push_back(0);
        if (AREF_OFFSET_EN) ref_offset.push_back((tREFI / NUM_RANKS) * i);
        else ref_offset.push_back(0);
        for (size_t j = 0; j < NUM_BANKS; j ++) {
            highqos_r_bank_cnt.push_back(0);
        }
    }
}
void PTC::PostCalcTiming() {
    // -- (4{pipe} + 5{bp_pipe} + 1{pipe} + 3{Match UT}) * func_clk_ratio
    if (sub_cha == 0) {
        PD_PRD = ceil(float(PD_PRD) / tDFI);
        ASREF_PRD = ceil(float(ASREF_PRD) / tDFI);
        ASREF_ADAPT_WIN = ceil(float(ASREF_ADAPT_WIN) / tDFI);
        SBR_IDLE_ADAPT_WIN = ceil(float(SBR_IDLE_ADAPT_WIN) / tDFI);
        tREFI = ceil((tREFI / DERATING_RATIO));
        
        PRINT_BW_WIN = ceil(float(PRINT_BW_WIN) / tDFI);
    }

    if (channel == 0) {    // MAP_CONFIG obly read once
        for (size_t i = 0; i < MAP_CONFIG["ASREF_ADAPT_PRD"].size(); i ++) {
            MAP_CONFIG["ASREF_ADAPT_PRD"][i] = ceil(float(MAP_CONFIG["ASREF_ADAPT_PRD"][i]) / tDFI);
        }
    }

    if (DMC_RATE > 3200) {
        trfcpb = tRFCpb + (OFREQ_EN ? 6 : 13); // add state machine pipe cycle for pbr
        trfcab = tRFCab + (OFREQ_EN ? 6 : 13); // add state machine pipe cycle for abr
    } else {
        trfcpb = tRFCpb + 6; // add state machine pipe cycle for pbr
        trfcab = tRFCab + 6; // add state machine pipe cycle for abr
    }

    if (DERATING_EN && sub_cha == 0) {
        tRCD    = tRCD + ceil(1.875 / tDFI);
        tRCD_WR = tRCD_WR + ceil(1.875 / tDFI);
        tRAS    = tRAS + ceil(1.875 / tDFI);
        tRPpb   = tRPpb + ceil(1.875 / tDFI);
        tRPab   = tRPab + ceil(1.875 / tDFI);
    }

    if (OFREQ_EN && sub_cha == 0) {
        OPENPAGE_TIME_RD = OPENPAGE_TIME_RD >> 1;
        OPENPAGE_TIME_WR = OPENPAGE_TIME_WR >> 1;
        ENH_PAGE_ADPT_WIN = ENH_PAGE_ADPT_WIN >> 1;
        tREFI = tREFI & 0xFFFFFFFE;
    }

    if (OFREQ_EN && channel == 0) {      //MAP_CONFIG only read once
        unsigned size = MAP_CONFIG["TIMEOUT_PRI_RD"].size();
        for (size_t i = 0; i < size; i ++) {
            MAP_CONFIG["TIMEOUT_PRI_RD"][i] = MAP_CONFIG["TIMEOUT_PRI_RD"][i] >> 1;
        }
        size = MAP_CONFIG["TIMEOUT_PRI_WR"].size();
        for (size_t i = 0; i < size; i ++) {
            MAP_CONFIG["TIMEOUT_PRI_WR"][i] = MAP_CONFIG["TIMEOUT_PRI_WR"][i] >> 1;
        }
    }

    if (OFREQ_EN && channel == 0) {      //MAP_CONFIG only read once
        unsigned size = MAP_CONFIG["PERF_TIMEOUT_PRI_RD"].size();
        for (size_t i = 0; i < size; i ++) {
            MAP_CONFIG["PERF_TIMEOUT_PRI_RD"][i] = MAP_CONFIG["PERF_TIMEOUT_PRI_RD"][i] >> 1;
            MAP_CONFIG["PERF_ADAPT_PRI_RD"][i] = MAP_CONFIG["PERF_ADAPT_PRI_RD"][i] >> 1;
        }
        size = MAP_CONFIG["PERF_TIMEOUT_PRI_WR"].size();
        for (size_t i = 0; i < size; i ++) {
            MAP_CONFIG["PERF_TIMEOUT_PRI_WR"][i] = MAP_CONFIG["PERF_TIMEOUT_PRI_WR"][i] >> 1;
            MAP_CONFIG["PERF_ADAPT_PRI_WR"][i] = MAP_CONFIG["PERF_ADAPT_PRI_WR"][i] >> 1;
        }
    }


    if (DMC_RATE < 3000) {
        PCFG_RANKTRTR_CASFS = 15;
        PCFG_RANKTRTW_CASFS = 17;
        PCFG_RANKTWTR_CASFS = 9;
        PCFG_RANKTWTW_CASFS = 13;
    } else if (DMC_RATE == 3063) {
        PCFG_RANKTRTR_CASFS = 15;
        PCFG_RANKTRTW_CASFS = 19;
        PCFG_RANKTWTR_CASFS = 8;
        PCFG_RANKTWTW_CASFS = 14;
    } else if (DMC_RATE == 4266) {
        PCFG_RANKTRTR_CASFS = 9;
        PCFG_RANKTRTW_CASFS = 15;
        PCFG_RANKTWTR_CASFS = 7;
        PCFG_RANKTWTW_CASFS = 15;
    } else if (DMC_RATE == 5500) {
        PCFG_RANKTRTR_CASFS = 9;
        PCFG_RANKTRTW_CASFS = 17;
        PCFG_RANKTWTR_CASFS = 7;
        PCFG_RANKTWTW_CASFS = 17;
    } else {
        PCFG_RANKTRTR_CASFS = PCFG_RANKTRTR;
        PCFG_RANKTRTW_CASFS = PCFG_RANKTRTW;
        PCFG_RANKTWTR_CASFS = PCFG_RANKTWTR;
        PCFG_RANKTWTW_CASFS = PCFG_RANKTWTW;
    }
}

//get a bus packet from either data or cmd bus
void PTC::receive_rdata(unsigned long long task, bool mask_wcmd) {
    //add to return read data queue
    if (mask_wcmd) {
        data_delay = tD_D + tDAT_PHY + tDAT_RASC + 2;
    } else {
        data_delay = tD_D + tDAT_PHY + tDAT_RASC;
    }
    gen_rdata(task, 1, data_delay, mask_wcmd);
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- R DDR :: Receiving From Data Bus, task="<<task<<endl);
    }
}

void PTC::Cmd2Dfi_statistics(uint64_t task, uint64_t timeAdded, unsigned qos,
        unsigned mid, unsigned pf_type, unsigned rank) {
    uint32_t delay = (now() + 1 - timeAdded);

    if      (float(delay) * tDFI < 50  ) {qos_level_cnt[qos][0] ++;}
    else if (float(delay) * tDFI < 100 ) {qos_level_cnt[qos][1] ++;}
    else if (float(delay) * tDFI < 150 ) {qos_level_cnt[qos][2] ++;}
    else if (float(delay) * tDFI < 200 ) {qos_level_cnt[qos][3] ++;}
    else if (float(delay) * tDFI < 300 ) {qos_level_cnt[qos][4] ++;}
    else if (float(delay) * tDFI < 500 ) {qos_level_cnt[qos][5] ++;}
    else if (float(delay) * tDFI < 1000) {qos_level_cnt[qos][6] ++;}
    else if (float(delay) * tDFI < 2000) {qos_level_cnt[qos][7] ++;}
    else                                 {qos_level_cnt[qos][8] ++;}

    float delay_ns = tDFI * delay;
    uint8_t size = lat_dly_cnt.size();
    for (size_t i = 0; i < size; i ++) {
        if (delay_ns < float(lat_dly_step[i])) {
            lat_dly_cnt[i] ++;
            break;
        }
    }

    qos_cnt[qos] ++;
    qos_delay_cnt[qos] += delay;
    mid_cnt[mid] ++;
    mid_delay_cnt[mid] += delay;
    pf_cnt[pf_type] ++;
    pf_delay_cnt[pf_type] += delay;

    if (mid >= CPU_MID_START && mid <= CPU_MID_END && qos >= PERF_SWITCH_HQOS_LEVEL) {
        com_highqos_read_cnt ++;
        total_highqos_latency += delay;
        ddrc_av_highqos_lat = float(total_highqos_latency) / com_highqos_read_cnt;
        rank_com_highqos_read_cnt[rank] ++;
        rank_total_highqos_latency[rank] += delay;
        rank_ddrc_av_highqos_lat[rank] = float(rank_total_highqos_latency[rank]) / rank_com_highqos_read_cnt[rank];
    }
    if (mid >= CPU_MID_START && mid <= CPU_MID_END && qos >= PERF_SWITCH_HQOS_LEVEL) {
        if (delay > highqos_max_delay) {
            highqos_max_delay = delay;
            highqos_max_delay_id = task;
        }
    }
}

void PTC::ReturnData_statistics(uint64_t task, uint64_t timeAdded, unsigned qos,
        unsigned mid, unsigned pf_type, unsigned rank) {
    uint32_t delay = (now() - timeAdded);
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- OVER :: Read DELAY ("<<delay<<") task="<<task<<endl);
    }

    // if (PRINT_RDATA) {
    //     TRACE_PRINT(setw(10)<<now()<<" -- Rdata task="<<task<<", latency="<<delay<<endl);
    // }
    if (DEBUG_BUS) {    
        if (delay > 2500) {
            dly_ex2000_cnt ++;
            PRINTN(setw(16)<<now()<<"Read DMC["<<delay<<"], num ["<<dly_ex2000_cnt<<"]"<<endl);
        }
    }
    com_read_cnt ++;
    total_latency += delay;
    ddrc_av_lat = float(total_latency) / com_read_cnt;

    total_latency_rank[rank] += delay;
    com_read_cnt_rank[rank] ++;
    ddrc_av_lat_rank[rank] = float(total_latency_rank[rank]) / com_read_cnt_rank[rank];

    if (delay > max_delay) {
        max_delay = delay;
        max_delay_id = task;
    }
    if (delay < min_delay || min_delay == 0) {
        min_delay = delay;
        min_delay_id = task;
    }
}
//sends read data back to the CPU
// bool PTC::returnReadData(unsigned int channel_num,unsigned long long task,
//         double readDataEnterDmcTime, double reqAddToDmcTime, double reqEnterDmcBufTime) {
//     if (memorySystemTop->ReturnReadData!=NULL) {
//         return (memorySystemTop->ReturnReadData)(channel_num, task,
//                 readDataEnterDmcTime, reqAddToDmcTime, reqEnterDmcBufTime);
//     } else {
//         return false;
//     }
// }

//receive the write data from CPU
void PTC::receive_wdata(unsigned int *data, uint64_t task) {
    unsigned second_time = 0;
    unsigned third_time = 0;
    if (!IECC_ENABLE || (!tasks_info[task].wr_ecc && !tasks_info[task].rd_ecc)) {
        if (pre_dat_time == now()) {
           ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] DATA received in the same cycle");
           assert(0);
        }
        if (third_time == 1) {
            third_time = 0;
            pre_dat_time = now();
        } else if (second_time == 1) {
            second_time = 0;
            third_time = 1;
        } else {
            second_time = 1;
        }
    }
    wdata_cnt ++;
    wdata_pipe wdata;
    wdata.task = task;
    wdata.delay = now() + tWDATA_DMC;
    WdataPipe.push_back(wdata);
    pre_req_data_time= now();
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- R_CPU :: Get Data from CPU, task="<<task<<endl);
    }
}

void PTC::addRank(Rank* rank) {
    if (linked_ranks_.size() < NUM_RANKS) {
        linked_ranks_.push_back(rank);
    }
}

Rank* PTC::getRank(unsigned index) const {
    if (index < linked_ranks_.size()) {
        return linked_ranks_[index];
    }
    return nullptr;
}

unsigned PTC::CalcCasTiming(unsigned bl, unsigned sync, unsigned wck_pst) {
    unsigned ret = 0;
    if (NUM_GROUPS > 1) { // BG mode
        ret = ceil(float(BL32) / WCK2DFI_RATIO) + sync + wck_pst;
    } else { // Bank mode
        ret = ceil(float(BL32) / 2 / WCK2DFI_RATIO) + sync + wck_pst;
    }
    return ret;
}

unsigned PTC::CalcTiming(bool is_trtp, unsigned cmd_bl, unsigned timing) {
    if (IS_LP5) {
        if (cmd_bl == BL32) {
            return timing;
        } else if (cmd_bl == BL16) {
            if (timing < ceil(float(BL32) / 2 / WCK2DFI_RATIO)) {
                return 0;
            } else {
                return timing - ceil(float(BL32) / 2 / WCK2DFI_RATIO);
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_LP6) {
        if (cmd_bl == BL48) {
            return timing;
        } else if (cmd_bl == BL24) {
            if (timing < ceil(float(BL48) / 2 / WCK2DFI_RATIO)) {
                return 0;
            } else {
                return timing - ceil(float(BL48) / 2 / WCK2DFI_RATIO);
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_LP4 || IS_DDR5 || IS_DDR4 || IS_HBM2E || IS_HBM3) {
        if (is_trtp) {
            if (IS_LP4 && cmd_bl == BL32) {
                return timing + ceil(float(BL16) / 2 / WCK2DFI_RATIO);
            } else {
                if (cmd_bl == BL32) return timing;
                else return timing - (8 / unsigned(WCK2DFI_RATIO));
            }
        } else {
            if (timing < ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO)) {
                return 0;
            } else {
                return timing - ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO);
            }
        }
    } else { // IS_G3D
        if (timing < ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO)) {
            return 0;
        } else {
            return timing - ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    }
}

unsigned PTC::CalcTccd(bool is_samebg, unsigned cmd_bl, unsigned tccd) {
    if (IS_LP5) {
        if (cmd_bl == BL32) {
            return tccd;
        } else if (cmd_bl == BL16) {
            if (is_samebg) {
                if (NUM_GROUPS > 1) return (BL16 / unsigned(WCK2DFI_RATIO));
                else return (BL16 / 2 / unsigned(WCK2DFI_RATIO));
            } else {
                return tccd;
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_LP6) {
        if (cmd_bl == BL48) {
            if (is_samebg) {
                if (NUM_GROUPS > 1) return tCCD_L48;
                else return tccd;
            } else {
                return tccd;
            }
        } else if (cmd_bl == BL24) {
            if (is_samebg) {
                if (NUM_GROUPS > 1) return tCCD_L24;
                else return (BL24 / 2 / unsigned(WCK2DFI_RATIO));
            } else {
                return tccd;
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_DDR5) {
        if (is_samebg) {
            return max(ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO), float(tCCD_L));
        } else {
            return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else if (IS_HBM2E || IS_HBM3) {
        return tccd;
    } else if (IS_DDR4) {
        return tccd;
    } else if (IS_LP4) {
        return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
    } else { // IS_G3D
        if (!is_samebg) {
            if (cmd_bl == BLEN) {
                return tccd;
            } else {
                return tccd / 2;
            }
        } else {
            return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    }
}

int PTC::CalcCmdCycle(uint8_t pre_cmd, uint8_t next_cmd) {
    return (pre_cmd - next_cmd);
}

unsigned PTC::CalcWrite2Mwrite(bool is_samebg, bool is_sameba, unsigned cmd_bl) {
    if (IS_LP4) {
        if (is_sameba) {
            if (cmd_bl == BL16) return tCCDMW;
            else return tCCDMW + unsigned(float(BL16) / 2 / WCK2DFI_RATIO);
        } else {
            return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else if (IS_LP5 || IS_LP6) {
        if (NUM_GROUPS > 1) { // BG mode
            if (is_sameba) {
                if (cmd_bl == BL16 || cmd_bl == BL24) { 
                    if (DMC_RATE <= 8533){
                        return 4 * BL_n_max[cmd_bl];
                    } else { // 9600/10667
                        return 5 * BL_n_max[cmd_bl];
                    }
                } else {
                    if (DMC_RATE <= 8533) {
                        return unsigned(2.5 * float(BL_n_max[cmd_bl]));
                    } else {
                        return unsigned(3 * float(BL_n_max[cmd_bl]));
                    }
                }
            } else if (is_samebg) {
                return BL_n_max[cmd_bl];
            } else {
                if (cmd_bl == BL16 || cmd_bl == BL24) return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
                else return unsigned(float(cmd_bl) / 4 / WCK2DFI_RATIO);
            }
        } else { // Bank mode
            if (is_sameba) {
                if (cmd_bl == BL16 || cmd_bl == BL24) return unsigned(4 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
                else return unsigned(2.5 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
            } else {
                return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
            }
        }
    } else {
        return tCCDMW;
    }
}

unsigned PTC::CalcMwrite2Write(bool is_samebg, bool is_sameba, unsigned cmd_bl) {
    if (IS_LP4) {
        return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
    } else if (IS_LP5 || IS_LP6) {
        if (NUM_GROUPS > 1) { // BG mode
            if (is_samebg) return unsigned(2 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
            else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        } else { // Bank mode
            return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else {
        return tCCDMW;
    }
}

unsigned PTC::CalcMwrite2Mwrite(bool is_samebg, bool is_sameba, unsigned cmd_bl) {
    if (IS_LP4) {
        if (is_sameba) return tCCDMW;
        else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
    } else if (IS_LP5 || IS_LP6) {
        if (NUM_GROUPS > 1) { // BG mode
            if (is_sameba) { 
                if (DMC_RATE <= 8533) {
                    return 4 * BL_n_max[cmd_bl];
                } else { // 9600/10667
                    return 5 * BL_n_max[cmd_bl];
                }
                
            }else if (is_samebg) return BL_n_max[cmd_bl];
            else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        } else { // Bank mode
            if (is_sameba) return unsigned(4 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
            else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else {
        return tCCDMW;
    }
}

/***************************************************************************************************
descriptor: This function is to refresh the timing, once there is a new command to send, it need to
update all the timing,
****************************************************************************************************/
#if defined(USE_TSC_LP)
#include "TSC_LP.cpp"
#elif defined(USE_TSC_DDR)
#include "TSC_DDR.cpp"
#elif defined(USE_TSC_HBM)
#include "TSC_HBM.cpp"
#else
#error "NO TSC selected"
#endif
/***************************************************************************************************
descriptor: power event statistics
****************************************************************************************************/
void PTC::power_event_stat() {
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (RankState[rank].lp_state == PD) continue;
        if (RankState[rank].lp_state == PDLP) continue;
        if (RankState[rank].lp_state == ASREF) continue;
        if (RankState[rank].lp_state == SRPD) continue;
        if (RankState[rank].lp_state == SRPDLP) continue;
        que_cmd_time[GetDmcQsize()] ++;
        break;
    }
}

/***************************************************************************************************
descriptor: update wdata pipeline
****************************************************************************************************/
void PTC::update_wdata() {
    if (WdataPipe.empty()) return;

    uint64_t task = WdataPipe[0].task;
    uint64_t delay = WdataPipe[0].delay;
    if (now() < delay) {
        return;
    }
    for (auto &t : WtransQueue) {
        if (t->transactionType == DATA_READ) continue;
        if (t->data_ready_cnt > t->burst_length) continue;
        if (task != t->task) continue;
        t->data_ready_cnt ++;
        WdataPipe.erase(WdataPipe.begin());
        if (DEBUG_BUS) {
             PRINTN(setw(10)<<now()<<" -- MATCH :: data ready cnt="<<t->data_ready_cnt<<", burst_length="
                     <<t->burst_length<<", data_size="<<t->data_size<<", task="<<t->task<<endl);
        }
        break;
    }
}

/***************************************************************************************************
descriptor: update pd status, include pd enter and pd exit
****************************************************************************************************/
void PTC::update_lp_state() {
    bool has_rank_blocking = false;
    unsigned blocking_rank = 0;

    // PHY LP logic
    bool phylp_timing_met = false;
    if (PhyLpState.lp_cnt > 0) {
        PhyLpState.lp_cnt --;
        if (PhyLpState.lp_cnt == 0) phylp_timing_met = true;
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (deqCmdWakeupLp[rank].front() > 0) rank_has_cmd[rank] = true;
        else rank_has_cmd[rank] = false;
    }

    bool is_allrank_phylp = true;
    bool phylp_wakeup = false;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (RankState[rank].lp_state != PDLP && RankState[rank].lp_state != SRPDLP) {
            is_allrank_phylp = false;
            break;
        }
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (rank_has_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) {
            phylp_wakeup = true;
            break;
        }
    }
    switch (PhyLpState.phylp_state) {
        case PHYLP_IDLE: {
            if (is_allrank_phylp) {
                PhyLpState.lp_cnt = tPHYLPE;
                PhyLpState.phylp_state = PHYLPE;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLP_IDLE, next_state=PHYLPE"<<endl);
                }
            }
            phy_notlp_cnt ++;
            break;
        }
        case PHYLPE: {
            if (phylp_wakeup) {
                PhyLpState.lp_cnt = tPHYLPX;
                PhyLpState.phylp_state = PHYLPX;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLPE, next_state=PHYLPX"<<endl);
                }
            } else if (phylp_timing_met) {
                PhyLpState.phylp_state = PHYLP;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLPE, next_state=PHYLP"<<endl);
                }
            }
            phy_notlp_cnt ++;
            break;
        }
        case PHYLP: {
            if (phylp_wakeup) {
                PhyLpState.lp_cnt = tPHYLPX;
                PhyLpState.phylp_state = PHYLPX;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLP, next_state=PHYLPX"<<endl);
                }
            }
            phy_lp_cnt ++;
            break;
        }
        case PHYLPX: {
            if (phylp_timing_met) {
                PhyLpState.phylp_state = PHYLP_IDLE;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLPX, next_state=PHYLP_IDLE"<<endl);
                }
            }
            phy_notlp_cnt ++;
            break;
        }
        default: break;
    }

    // Func PD & Asref logic
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (IS_LP5 || IS_LP6) {
            if (RankState[rank].lp_state == PDE || RankState[rank].lp_state == PDX ||
                    RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREFX ||
                    RankState[rank].lp_state == SRPDE || RankState[rank].lp_state == SRPDX) {
                has_rank_blocking = true;
                blocking_rank = rank;
                break;
            }
        }
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (has_rank_blocking && blocking_rank != rank) continue;
        if (RankState[rank].pd_cnt > 0) RankState[rank].pd_cnt --;
        if (RankState[rank].asref_cnt > 0) RankState[rank].asref_cnt --;
        if (RankState[rank].state_cnt > 0) RankState[rank].state_cnt --;
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        bool combo_emode_unable = EM_ENABLE && EM_MODE==2 && rank==1;
        bool pd_timing_met = false;
        bool asref_timing_met = false;
        bool state_timing_met = false;
        bool has_fastwakeup = fast_wakeup_cnt[rank] > 0;
        if (RankState[rank].pd_cnt == 0 && now() >= funcState[rank].nextPde) pd_timing_met = PD_ENABLE && even_cycle;  // every other command, even;
        if (RankState[rank].asref_cnt == 0) {
            bool all_bank_idle = true;
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                unsigned sub_channel = bank / sc_bank_num;
                if (combo_emode_unable && sub_channel==1) continue;        //rank1, sc1 forbidden under combo e-mode
                if (bankStates[rank*NUM_BANKS+bank].state->currentBankState == Idle) continue;
                all_bank_idle = false;
                break;
            }
            if (all_bank_idle) asref_timing_met = ASREF_ENABLE && even_cycle;   //every other command, even;
        }
        if (RankState[rank].state_cnt == 0) {
            state_timing_met = PD_ENABLE && even_cycle;   //every other command, even;
        }
        if (DMC_V590 && ASREF_ADAPT_EN) {
            if (now() % ASREF_ADAPT_WIN == 0 && now() != 0) {
                if (rank_cnt_asref[rank] >= MAP_CONFIG["ASREF_ADAPT_LEVEL"][2]) {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][3];
                } else if (rank_cnt_asref[rank] >= MAP_CONFIG["ASREF_ADAPT_LEVEL"][1]) {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][2];
                } else if (rank_cnt_asref[rank] >= MAP_CONFIG["ASREF_ADAPT_LEVEL"][0]) {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][1];
                } else {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][0];
                }
                rank_cnt_asref[rank] = 0;
            }
        }
        // added for e-mode
        unsigned refresh_all = true;
        for (size_t i = 0; i < sc_num; i++) {
            if (combo_emode_unable && i==1) continue;    //rank1, sc1 forbidden under combo e-mode
            if (refreshALL[rank][i].refresh_cnt !=0){
                refresh_all = false;
                break;
            } 
        }
        switch (RankState[rank].lp_state) {
            case IDLE: {
                if (rank_has_cmd[rank] || fast_wakeup[rank] || has_fastwakeup) {
                    if (PD_ENABLE) RankState[rank].pd_cnt = PD_PRD;
                    if (ASREF_ENABLE) RankState[rank].asref_cnt = ASREF_PRD;
                } else if (!DMC_V580 && funcState[rank].wakeup) {
                    if (PD_ENABLE) RankState[rank].pd_cnt = PD_PRD;
                } else if (asref_timing_met && ASREF_ENABLE && !funcState[rank].wakeup) {
                    RankState[rank].asref_cnt = tASREFE;
                    RankState[rank].lp_state = pd_timing_met ? SRPDE : ASREFE;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=IDLE, next_state=ASREFE"<<endl);
                    }
                } else if (pd_timing_met && PD_ENABLE && !funcState[rank].wakeup && refresh_all) {
                    RankState[rank].pd_cnt = tPDE;
                    RankState[rank].lp_state = PDE;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=IDLE, next_state=PDE"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case PDE: {
                if (pd_timing_met) {
                    RankState[rank].pd_cnt = tPDLP;
                    RankState[rank].lp_state = PD;
                    RankState[rank].state_cnt = tCSPD;
                    phy p;
                    p.command.type = PD_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF6;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    PdEnterCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PDE, next_state=PD"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case PD: {
                if ((rank_has_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) &&
                        state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = PDX;
                    phy p;
                    p.command.type = PDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF5;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PD, next_state=PDX"<<endl);
                    }
                } else if (pd_timing_met) {
                    RankState[rank].lp_state = PDLP;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PD, next_state=PDLP"<<endl);
                    }
                }
                PdTime[rank] ++;
                break;
            }
            case PDLP: {
                if ((rank_has_cmd[rank] || fast_wakeup[rank] || asref_timing_met ||
                        funcState[rank].wakeup) && state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = PDX;
                    phy p;
                    p.command.type = PDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF5;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PDLP, next_state=PDX"<<endl);
                    }
                }
                PdTime[rank] ++;
                break;
            }
            case PDX: {
                if (pd_timing_met) {
                    if (!asref_timing_met && !funcState[rank].wakeup) {
                        RankState[rank].pd_cnt = PD_PRD;
                        if (ASREF_ENABLE) RankState[rank].asref_cnt = ASREF_PRD;
                    }
                    RankState[rank].lp_state = IDLE;
                    PdExitCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PDX, next_state=IDLE"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case ASREFE: {
                if (asref_timing_met) {
                    RankState[rank].lp_state = ASREF;
                    phy p;
                    p.command.type = ASREF_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF3;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    AsrefEnterCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREFE, next_state=ASREF"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case ASREF: {
                if (rank_has_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) {
                    RankState[rank].asref_cnt = tXSR;
                    RankState[rank].lp_state = ASREFX;
                    phy p;
                    p.command.type = ASREFX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF2;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREF, next_state=ASREFX"<<endl);
                    }
                } else if (pd_timing_met) {
                    if (DMC_V590) {
                        RankState[rank].pd_cnt = tPDLP;
                        RankState[rank].lp_state = SRPD;
                        RankState[rank].state_cnt = tCSPD;
                        phy p;
                        p.command.type = SRPD_CMD;
                        p.command.rank = rank;
                        p.command.task = 0xFFFFFFFFFFFFFF0;
                        p.delay = tCMD_PHY;
                        packet.push_back(p);
                        PdEnterCnt[rank] ++;
                        SrpdEnterCnt[rank] ++;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDE, next_state=SRPD"<<endl);
                        }
                    } else {
                        RankState[rank].pd_cnt = tPDE;
                        RankState[rank].lp_state = SRPDE;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREF, next_state=SRPDE"<<endl);
                        }
                    }
                }
                AsrefTime[rank] ++;
                break;
            }
            case ASREFX: {
                if (asref_timing_met) {
                    if (!funcState[rank].wakeup) {
                        RankState[rank].pd_cnt = PD_PRD;
                        RankState[rank].asref_cnt = ASREF_PRD;
                    }
                    RankState[rank].lp_state = IDLE;
                    AsrefExitCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREFX, next_state=IDLE"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case SRPDE: {
                if (pd_timing_met) {
                    RankState[rank].pd_cnt = tPDLP;
                    RankState[rank].lp_state = SRPD;
                    RankState[rank].state_cnt = tCSPD;
                    phy p;
                    p.command.type = SRPD_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF0;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    PdEnterCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDE, next_state=SRPD"<<endl);
                    }
                }
                AsrefTime[rank] ++;
                break;
            }
            case SRPD: {
                if ((rank_has_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) &&
                        state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = SRPDX;
                    phy p;
                    p.command.type = SRPDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFED;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPD, next_state=SRPDX"<<endl);
                    }
                } else if (pd_timing_met) {
                    RankState[rank].lp_state = SRPDLP;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPD, next_state=SRPDLP"<<endl);
                    }
                }
                SrpdTime[rank] ++;
                break;
            }
            case SRPDLP: {
                if ((rank_has_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) && state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = SRPDX;
                    phy p;
                    p.command.type = SRPDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFED;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDLP, next_state=SRPDX"<<endl);
                    }
                }
                SrpdTime[rank] ++;
                break;
            }
            case SRPDX: {
                if (pd_timing_met) {
                    RankState[rank].asref_cnt = tXSR;
                    RankState[rank].lp_state = ASREFX;
                    PdExitCnt[rank] ++;
                    SrpdExitCnt[rank] ++;
                    phy p;
                    p.command.type = ASREFX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF2;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDX, next_state=ASREFX"<<endl);
                    }
                }
                AsrefTime[rank] ++;
                break;
            }
            default: break;
        }
    }
    // Fast wakeup clear
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) fast_wakeup[rank] = false;
    // Func wakeup clear
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (now() >= funcState[rank].nextPde && funcState[rank].wakeup) {
            funcState[rank].wakeup = false;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- FUNC :: Pde timing met, rank="<<rank<<endl);
            }
        }
    }
}

/***************************************************************************************************
descriptor: execute unit, this function interacts with DDR grain
            this function will be DFI bus
****************************************************************************************************/
void PTC::dfi_pipe() {
    if (PRINT_CMD_NUM) {
        CMDNUM_PRINT(setw(10)<<now()<<": ");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+r_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+w_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+pfq_->rb_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+pfq_->wb_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|"<<endl);
    }

    if (!packet.empty()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- packet_size="<<packet.size()<<endl);
        }
        uint8_t size = packet.size();
        uint8_t erase_cnt = 0;
        for (size_t i = 0; i < size; i ++) {
            if (packet[i - erase_cnt].delay == 0) {
                linked_ranks_[packet[0].command.rank]->rdata_return_pipe(packet[i - erase_cnt].command);
                if (PRINT_IDLE_LAT) {
                    if (command.type == READ_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [READ_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row<<" wcnt="
                                <<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                    } else if (command.type == READ_P_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [READ_P_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row<<" wcnt="
                                <<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                    } else if (command.type == ACTIVATE1_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [ACTIVATE1_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row
                                <<" bl="<<command.bl);
                    } else if (command.type == ACTIVATE2_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [ACTIVATE2_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row
                                <<" bl="<<command.bl);
                    }
                }
                packet.erase(packet.begin());
                erase_cnt ++;
            }
        }
    }
    
    //check for outgoing command packets and handle countdowns
    arb_enable = true;
    if (exec_valid) {
        if (command_pend != 0 && core_concurr_en) {
            command_pend --;
            if (command_pend > 1) arb_enable = false;
            if (DEBUG_BUS && command_pend > 0) {
                PRINTN(setw(10)<<now()<<" -- CMD_PEND :: task="<<command.task<<endl);
            }
        }
        if (command_pend == 0) {
            phy p;
            p.command = command;
            p.delay = tCMD_PHY;
            packet.push_back(p);
            exec_valid = false;

            if (!odd_cycle){
                ERROR(setw(10)<<now()<<" Every other command at even cycle violated"<<" task="<<command.task<<" cmd_type="
                        <<command.type<<" rank="<<command.rank<<" group="<<command.group<<" bank="<<command.bankIndex
                        <<" row"<<command.row);
                assert(0);
            }

            if (PRINT_EXEC) {
                static uint64_t pre_time_exec = 0;
                if (command.type == READ_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [READ_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == READ_P_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [READ_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col"<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_P_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_MASK_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_MASK_P_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                }
            }
            if (PRINT_IDLE_LAT) {
                if (command.type == READ_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [READ_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                } else if (command.type == READ_P_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [READ_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                } else if (command.type == ACTIVATE1_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [ACTIVATE1_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" bl="<<command.bl);
                } else if (command.type == ACTIVATE2_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [ACTIVATE2_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" bl="<<command.bl);
                }
            }
            if (DEBUG_BUS) {
                static uint64_t pre_time = 0;
                if (command.type == READ_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [READ_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == READ_P_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [READ_P_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == ACTIVATE1_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [ACTIVATE1_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == ACTIVATE2_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [ACTIVATE2_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == WRITE_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == WRITE_P_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_P_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == WRITE_MASK_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == WRITE_MASK_P_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_P_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == PRECHARGE_SB_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_SB_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == PRECHARGE_PB_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_PB_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == PRECHARGE_AB_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_AB_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<endl);
                } else if (command.type == REFRESH_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [REFRESH_CMD] to DFI task="<<command.task<<", rank="
                            <<command.rank<<endl);
                } else if (command.type == PER_BANK_REFRESH_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PER_BANK_REFRESH_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<endl);
                }
            }
        }
    }
    if (core_concurr_en) {
        for (auto &dfi : packet) if (dfi.delay > 0) dfi.delay --;
        for (auto &state : bankStates) {
            if (state.state->currentBankState == RowActive) BankRowActCnt[state.bankIndex] ++;
        }
    }
}


/***************************************************************************************************
descriptor: state machine
****************************************************************************************************/
void PTC::state_fresh() {
    //update bank states
    for (auto &state : bankStates) {
        unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE && EM_MODE==2 && state.rank==1 && state_channel==1) continue;   // rank1, sc1 forbidden under combo e-mode                 
        if (state.state->rwIntlvCountdown > 0 && core_concurr_en) state.state->rwIntlvCountdown --;
        if (state.state->stateChangeEn) {
            //decrement counters
            if (core_concurr_en && state.state->stateChangeCountdown > 0){
                state.state->stateChangeCountdown --;
            }
            //if counter has reached 0, change state
            if (state.state->stateChangeCountdown == 0) {
                switch (state.state->lastCommand) { //only these commands have an implicit state change
                    case WRITE_P_CMD:
                    case READ_P_CMD:
                    case WRITE_MASK_P_CMD:{
                        // fix ,if the state of transaction is precharging ,it must be hold until idle
                        state.state->currentBankState = Precharging;
                        state.state->lastCommand = PRECHARGE_PB_CMD;
                        state.state->stateChangeCountdown = state.state->fg_ref ? tRPfg : tRPpb;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: READ/WRITE/WRITE_MASK_P_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case REFRESH_CMD :{
                        unsigned sub_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                        state.state->currentBankState = Idle;
                        refreshALL[state.rank][sub_channel].refreshWaiting = false;       //todo: revise for e-mode
                        refreshALL[state.rank][sub_channel].refreshing = false;           //todo: revise for e-mode 
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.hold_refresh_pb = false;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS && (state.bankIndex % NUM_BANKS == 0)) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: REFRESH_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PRECHARGE_SB_CMD :
                    case PRECHARGE_PB_CMD :{
                        state.state->currentBankState = Idle;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PRECHARGE_P/SB_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PRECHARGE_AB_CMD :{
                        state.state->currentBankState = Idle;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS && (state.bankIndex % NUM_BANKS == 0)) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PRECHARGE_AB_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PER_BANK_REFRESH_CMD :{
                        state.state->currentBankState = Idle;
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.hold_refresh_pb = false;
                        //state.finish_refresh_pb = true;
                        state.state->stateChangeEn = false;
                        sbr_gap_cnt[state.rank] = now() + SBR_GAP_CNT;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PER_BANK_REFRESH_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    // case ACTIVATE2_DST_CMD :
                    case ACTIVATE2_CMD :{
                        state.state->stateChangeEn = false;
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.hold_refresh_pb = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: ACTIVATE2/_DST_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    default: break;
                }
            }
        }
    }

    if (!WCK_ALWAYS_ON && (IS_LP5 || IS_LP6)) {
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (!RankState[i].wck_on) continue;
            if (now() < RankState[i].wck_off_time) continue;
            RankState[i].wck_on = false;
        }
    }
}

/***************************************************************************************************
descriptor: refreshing process
****************************************************************************************************/
bool PTC::check_rankStates(uint8_t rank) {
     for (size_t i = 0; i < NUM_BANKS; i ++) {
         uint32_t bank = rank*NUM_BANKS + i;
         if (bankStates[bank].state->currentBankState == RowActive) return false;
     }
     return true;
}

unsigned PTC::priority(Cmd *cmd) {
    unsigned level_ret = 0;
    switch (cmd->cmd_type) {
        case DRAMSim::INVALID :
        case DRAMSim::REFRESH_CMD :
        case DRAMSim::PRECHARGE_SB_CMD :
        case DRAMSim::PRECHARGE_PB_CMD :
        case DRAMSim::PRECHARGE_AB_CMD : {
            if (cmd->force_pbr) level_ret = 400;
            else level_ret = 0;
            break;
        }
        case DRAMSim::ACTIVATE1_CMD : {
            level_ret = 300;
            break;
        }
        case DRAMSim::PER_BANK_REFRESH_CMD : {
            level_ret = 400;
            break;
        }
        case DRAMSim::ACTIVATE2_CMD : {
            if (IS_LP5 || IS_LP6) {
                level_ret = 500;
            } else if (IS_LP4) {
                level_ret = 9999;
            } else if (IS_DDR5 || IS_DDR4) {
                level_ret = 300;
            } else if (IS_HBM2E || IS_HBM3) {
                if(cmd->sid == PreCmd.sid) {
                    if(cmd->group == PreCmd.group) level_ret = 310;
                    else level_ret = (tCCD_R > tCCD_L) ? 320 : 300;
                } else {
                    level_ret = (tCCD_R > tCCD_L) ? 300 : 320;
                }
            }
            break;
        }
        case DRAMSim::WRITE_CMD :
        case DRAMSim::WRITE_P_CMD :
        case DRAMSim::WRITE_MASK_CMD :
        case DRAMSim::WRITE_MASK_P_CMD : {
            if (cmd->issue_size == 0) level_ret = 600;
            else level_ret = 700;
            break;
        }
        case DRAMSim::READ_CMD :
        case DRAMSim::READ_P_CMD : {
            if (cmd->issue_size == 0) level_ret = 600;
            else level_ret = 700;
            break;
        }
        default:
            break;
    }
    return level_ret;
}

unsigned PTC::priority_pri(Cmd *cmd) {
    unsigned level_ret = cmd->pri;
    return level_ret;
}

/***************************************************************************************************
descriptor: main scheduler,The purpose of this function is selecting the best task to perform from the queue
****************************************************************************************************/
void PTC::scheduler() {

    if (act_cmdqueue.empty() && rw_cmdqueue.empty() && pre_cmdqueue.empty() && CmdQueue.empty()) return;

    Cmd *c = NULL;
    if (CORE_CONCURR == 1) {
        Cmd *prec = NULL;
        for (auto &cmd : pre_cmdqueue) {
            if (cmd==NULL) continue;
            if (cmd->issue_size != 0) {prec = cmd; break;}
            if (prec == NULL) {prec = cmd; continue;}
            if (priority_pri(cmd) > priority_pri(prec)) {
                prec = cmd;
            } else if (priority_pri(cmd) == priority_pri(prec)){
                if(lru_arb(cmd->bankIndex, prec->bankIndex, 1)) prec = cmd;
            }
        }
        if (DEBUG_BUS && prec != NULL) {
            PRINTN(setw(10)<<now()<<" -- precmd_task="<<prec->task<<endl);
        }
        if (prec != NULL) CmdQueue.push_back(prec);
        for (auto &cmd : CmdQueue) {
            if (cmd == NULL) continue;
            if (cmd->issue_size != 0) {c = cmd; break;}
            if (c == NULL) {c = cmd; continue;}
            //TBD ACT2_HOLD_GAP
            if (priority(cmd) > priority(c)) {
                c = cmd;
            }
            
        }
        if (c == NULL) return;
        if(c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) update_lru(c->ptc_slot, 0, c);
        if(c->cmd_type == PRECHARGE_PB_CMD || c->cmd_type == PRECHARGE_SB_CMD) update_lru(c->bankIndex, 1, c);
        pregrt_cmd = c;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- pregrt_cmd_task="<<c->task<<endl);
        }

    } else {
        vector <bool> bank_has_rw;
        vector <bool> rank_has_rw;
        bank_has_rw.clear();
        bank_has_rw.reserve(NUM_BANKS * NUM_RANKS);
        rank_has_rw.clear();
        rank_has_rw.reserve(NUM_RANKS);
        for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
            bank_has_rw.push_back(false);
        }
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            rank_has_rw.push_back(false);
        }

        for (auto &cmd : rw_cmdqueue) {
            if (cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= READ_P_CMD) {
                bank_has_rw[cmd->bankIndex] = true;
                rank_has_rw[cmd->rank] = true;
            }
        }

        uint8_t erase_cnt = 0;
        uint8_t que_size = pre_cmdqueue.size();
        for (size_t i = 0; i < que_size; i ++) {
            uint8_t ecnt = i - erase_cnt;
            if (pre_cmdqueue[ecnt]->cmd_type == PRECHARGE_PB_CMD && bank_has_rw[pre_cmdqueue[ecnt]->bankIndex]) {
                pre_cmdqueue.erase(pre_cmdqueue.begin() + ecnt);
                erase_cnt ++;
            }
        }
        
        erase_cnt = 0;
        que_size = CmdQueue.size();
        for (size_t i = 0; i < que_size; i ++) {
            uint8_t ecnt = i - erase_cnt;
            if (CmdQueue[ecnt]->cmd_type == PRECHARGE_AB_CMD && rank_has_rw[CmdQueue[ecnt]->rank]) {
                CmdQueue.erase(CmdQueue.begin() + ecnt);
                erase_cnt ++;
            }
        }

        if (now() % CORE_CONCURR_PRD != 0) {
            return;
        } else if (!core_concurr_en) { //send read/write command
            Cmd *rwc = NULL;
            for (auto &cmd : CmdQueue) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- CmdQueue entry: "<<"task="<<cmd->task<<", sid="<<cmd->sid<<", bank group="<<cmd->group<<", bank="<<cmd->bank<<", bankIndex="<<cmd->bankIndex<<", row="<<cmd->row
                    <<", issue_size="<<cmd->issue_size<<", cmd_type="<<cmd->cmd_type<<", trans_type="<<cmd->type<<", type_pri="<<priority(cmd)<<", cmd_pri="<<priority_pri(cmd)<<endl);
                }
                if (cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= READ_P_CMD) {
                    if (cmd->issue_size != 0) {rwc = cmd; break;}
                    if (rwc == NULL) {rwc = cmd; continue;}
                    if (priority_pri(cmd) > priority_pri(rwc)) {
                        rwc = cmd;
                    } else if (priority_pri(cmd) == priority_pri(rwc)) {
                        if(lru_arb(cmd->ptc_slot, rwc->ptc_slot, 0)) rwc = cmd;
                    }
                }
            }
            c = rwc;
            if (rwc == NULL) {
                for (auto &cmd : act_cmdqueue) delete cmd;
                    act_cmdqueue.clear();
                for (auto &cmd : rw_cmdqueue) delete cmd;
                    rw_cmdqueue.clear();
                for (auto &cmd : pre_cmdqueue) delete cmd;
                    pre_cmdqueue.clear();
                // for (auto &cmd : CmdQueue) delete cmd;
                CmdQueue.clear();
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- delete cmdqueue"<<endl);
                }
                return;
            }
            update_lru(c->ptc_slot, 0, c);
            pregrt_cmd = c;
        } else { //send act/pre/ref/pbr and so on
            // Cmd *actc = NULL;
            // for (auto &cmd : act_cmdqueue) {
            //     if (cmd->issue_size != 0) {actc = cmd; break;}
            //     if (actc == NULL) {actc = cmd; continue;}
            //     if (priority_pri(cmd) > priority_pri(actc)) {
            //         actc = cmd;
            //     } else if (priority_pri(cmd) == priority_pri(actc)) {
            //         if(lru_arb(cmd->ptc_slot, actc->ptc_slot, 0)) actc = cmd;
            //     }
            // }
            // if (actc != NULL) {
            //     CmdQueue.push_back(actc);
            //     if (DEBUG_BUS) {
            //         PRINTN(setw(10)<<now()<<" -- act cmd selected, task="<<actc->task<<endl);
            //     }
            // }
            Cmd *prec = NULL;
            for (auto &cmd : pre_cmdqueue) {
                if (cmd->issue_size != 0) {prec = cmd; break;}
                if (prec == NULL) {prec = cmd; continue;}
                if (priority_pri(cmd) > priority_pri(prec)) {
                    prec = cmd;
                } else if (priority_pri(cmd) == priority_pri(prec)) {
                    if(lru_arb(cmd->bankIndex, prec->bankIndex, 1)) prec = cmd;
                }
            }
            if (prec != NULL) CmdQueue.push_back(prec);
            for (auto &cmd : CmdQueue) {
                if (cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= READ_P_CMD) continue;
                // if (cmd->issue_size != 0) {c = cmd; break;}
                if (c == NULL) {c = cmd; continue;}
                //TBD ACT2_HOLD_GAP
                if (priority(cmd) > priority(c)) {
                    c = cmd;
                }
            }
            
            if (c == NULL) return;
            if (c != NULL && DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- act or pre cmd selected, task="<<c->task<<endl);
            }
            if(c->cmd_type == PRECHARGE_PB_CMD || c->cmd_type == PRECHARGE_SB_CMD) update_lru(c->bankIndex, 1, c);
            pregrt_cmd = c;
        }
    }

    arb_enable = false;
    if (c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) {
        no_sch_cmd_en = true;
        no_sch_cmd_cnt = 0x0;
        page_rw_cnt ++;
        if (RWGRP_TRANS_BY_TOUT && c->timeout) {
            sch_tout_cmd = true;
            sch_tout_type = c->type;
            sch_tout_rank = c->rank;
        }
        if (PreCmd.trans_type != c->type) {     
            rw_switch_cnt ++;
            if (PreCmd.trans_type != DATA_READ && c->type == DATA_READ) w2r_switch_cnt ++;
            else r2w_switch_cnt ++;
            if (PRINT_EXEC) {
                DEBUGN(setw(10)<<now()<<" -- EXEC :: [RW_SWITCH] type="<<c->type<<" ser_rw_cnt="<<ser_rw_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]
                            <<" R"<<i<<"PR="<<+pfq_->rb_rank_cnt[i]<<" R"<<i<<"PW="<<+pfq_->wb_rank_cnt[i]
                            <<" R"<<i<<"PRR="<<+pfq_->rrel_rank_cnt[i]<<" R"<<i<<"PRW="<<+pfq_->wrel_rank_cnt[i]);
                }
                DEBUGN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" perf_wcnt="<<pfq_->wcmd_cnt<<" perf_rcnt="<<pfq_->rcmd_cnt
                        <<" rel_wcmd_cnt="<<pfq_->rel_wcmd_cnt<<" rel_rcmd_cnt="<<pfq_->rel_rcmd_cnt<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- EXEC :: [RW_SWITCH] type="<<c->type<<" ser_rw_cnt="<<ser_rw_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]
                            <<" R"<<i<<"PR="<<+pfq_->rb_rank_cnt[i]<<" R"<<i<<"PW="<<+pfq_->wb_rank_cnt[i]
                            <<" R"<<i<<"PRR="<<+pfq_->rrel_rank_cnt[i]<<" R"<<i<<"PRW="<<+pfq_->wrel_rank_cnt[i]);
                }
                PRINTN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" perf_wcnt="<<pfq_->wcmd_cnt<<" perf_rcnt="<<pfq_->rcmd_cnt
                        <<" rel_wcmd_cnt="<<pfq_->rel_wcmd_cnt<<" rel_rcmd_cnt="<<pfq_->rel_rcmd_cnt<<endl);
            }

            if (PRINT_BANK) {
                if (c->type == DATA_READ) {
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        for (size_t j = 0; j < NUM_BANKS; j ++) {
                            Rcmd_Dist[i][j] = 0;
                            for (auto& r : pfq_->PerfQue) {
                                if (r == nullptr) continue;
                                if (r->transactionType != DATA_READ) continue;
                                if (r->rank != i) continue;
                                if ((r->bankIndex % NUM_BANKS) != j) continue;
                                Rcmd_Dist[i][j] ++;
                            }
                            for (unsigned l = 0; l <transactionQueue.size(); l++) {
                                if (transactionQueue[l]->rank != i) continue;
                                if ((transactionQueue[l]->bankIndex % NUM_BANKS) != j) continue;
                                if (transactionQueue[l]->transactionType != DATA_READ) continue;
                                Rcmd_Dist[i][j] ++;
                            }
                            PRINTN(setw(10)<<now()<<" -- W2R, R"<<i<<"_Ba"<<j<<"="<<Rcmd_Dist[i][j]<<endl);
                        }
                    }
                } else if (c->type == DATA_WRITE) {
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        for (size_t j = 0; j < NUM_BANKS; j ++) {
                            Wcmd_Dist[i][j] = 0;
                            for (auto& w : pfq_->PerfQue) {
                                if (w == nullptr) continue;
                                if (w->transactionType != DATA_WRITE) continue;
                                if (w->rank != i) continue;
                                if ((w->bankIndex % NUM_BANKS) != j) continue;
                                Wcmd_Dist[i][j] ++;
                            }
                            for (unsigned l = 0; l <transactionQueue.size(); l++) {
                                if (transactionQueue[l]->rank != i) continue;
                                if ((transactionQueue[l]->bankIndex % NUM_BANKS) != j) continue;
                                if (transactionQueue[l]->transactionType != DATA_WRITE) continue;
                                Wcmd_Dist[i][j] ++;
                            }
                            PRINTN(setw(10)<<now()<<" -- R2W, R"<<i<<"_Ba"<<j<<"="<<Wcmd_Dist[i][j]<<endl);
                        }
                    }
                
                }
            }

            ser_rw_cnt = 0;
        }
        if (PreCmd.rank != c->rank) {
            rank_switch_cnt ++;
            if (PreCmd.trans_type == DATA_READ && c->type == DATA_READ) r_rank_switch_cnt ++;
            if (PreCmd.trans_type == DATA_WRITE && c->type == DATA_WRITE) w_rank_switch_cnt ++;
            if (PRINT_EXEC) {
                DEBUGN(setw(10)<<now()<<" -- EXEC :: [RANK_SWITCH] rank="<<c->rank<<" ser_rank_cnt="<<ser_rank_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]
                            <<" R"<<i<<"PR="<<+pfq_->rb_rank_cnt[i]<<" R"<<i<<"PW="<<+pfq_->wb_rank_cnt[i]
                            <<" R"<<i<<"PRR="<<+pfq_->rrel_rank_cnt[i]<<" R"<<i<<"PRW="<<+pfq_->wrel_rank_cnt[i]);
                }
                DEBUGN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" perf_wcnt="<<pfq_->wcmd_cnt<<" perf_rcnt="<<pfq_->rcmd_cnt
                        <<" rel_wcmd_cnt="<<pfq_->rel_wcmd_cnt<<" rel_rcmd_cnt="<<pfq_->rel_rcmd_cnt<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- EXEC :: [RANK_SWITCH] rank="<<c->rank<<" ser_rank_cnt="<<ser_rank_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]
                            <<" R"<<i<<"PR="<<+pfq_->rb_rank_cnt[i]<<" R"<<i<<"PW="<<+pfq_->wb_rank_cnt[i]
                            <<" R"<<i<<"PRR="<<+pfq_->rrel_rank_cnt[i]<<" R"<<i<<"PRW="<<+pfq_->wrel_rank_cnt[i]);
                }
                PRINTN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" perf_wcnt="<<pfq_->wcmd_cnt<<" perf_rcnt="<<pfq_->rcmd_cnt
                        <<" rel_wcmd_cnt="<<pfq_->rel_wcmd_cnt<<" rel_rcmd_cnt="<<pfq_->rel_rcmd_cnt<<endl);
            }
            if (c->type==DATA_WRITE && ser_rw_cnt < 3) {
                neg_r2w_rank_sw ++;
            }

            ser_rank_cnt = 0;
        }
        if (PreCmd.sid != c->sid) {
            sid_switch_cnt ++;
            if (PRINT_EXEC) {
                DEBUGN(setw(10)<<now()<<" -- EXEC :: [SID_SWITCH] ser_sid_cnt="<<ser_sid_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- EXEC :: [SID_SWITCH] ser_sid_cnt="<<ser_sid_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            ser_sid_cnt = 0;
        }
        if (PreCmdTime > 16 && PreCmdTime!=0xffffffff && PreCmd.rank==c->rank && PreCmd.trans_type==c->type) {
            if (c->type==DATA_READ) {
                r_lgap_cnt ++;
            } else if (c->type==DATA_WRITE) {
                w_lgap_cnt ++;
            }
        }
        if (PreCmdTime > 40 && PreCmdTime!=0xffffffff && PreCmd.trans_type!=c->type) {
            if (c->type==DATA_READ) {
                w2r_lgap_cnt ++;
            } else if (c->type==DATA_WRITE) {
                r2w_lgap_cnt ++;
            }
        }
            
        PreCmdTime = 0;

        ser_rw_cnt ++;
        ser_rank_cnt ++;
        ser_sid_cnt ++;
        PreCmd.type = c->cmd_type;
        PreCmd.trans_type = c->type;
        PreCmd.rank = c->rank;
        PreCmd.sid = c->sid;
        PreCmd.group = c->group;
    }

    if (CAS_FS_EN && NUM_RANKS > 1 && DMC_RATE < 4266 && (IS_LP5 || IS_LP6)) {
        bool rw_cmd = c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD;
        bool has_wckfs = false;
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (send_wckfs[i]) {
                has_wckfs = true;
                break;
            }
        }
        if (!has_wckfs && rw_cmd) {
            bool wck_on = false;
            for (auto &state : RankState) {
                if (!state.wck_on) continue;
                wck_on = true;
                break;
            }

            if (!wck_on) {
                for (size_t i = 0; i < NUM_RANKS; i ++) { // next rank
                    if (c->rank == i) continue;
                    if (rank_cnt[c->rank] <= CAS_FS_TH && rank_cnt[i] > 0) {
                        for (size_t j = 0; j < NUM_RANKS; j ++) send_wckfs[j] = true;
                        casfs_cnt ++;
                        break;
                    }
                }
            }
        }
    }

    if (c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) {
        pre_sch_bankIndex[c->rank] = c->bankIndex;
    }

    unsigned matgrp = 0;
    unsigned sub_channel = (c->bankIndex % NUM_BANKS) / sc_bank_num;
    unsigned bank_pair_start = sub_channel * pbr_bank_num; 
    switch (c->cmd_type) {
        case READ_CMD : {
            if (RDATA_TYPE == 0) {
                for (auto &trans : transactionQueue) {
                    if (c->task != trans->task) continue;
                    if (trans->issue_size != 0) continue;
                    if (trans->mask_wcmd==true) continue;
                    pfq_->gen_rresp(c->task, c->channel);
                    break;
                }
            }
            if (IECC_ENABLE && tasks_info[c->task].rd_ecc) {
                ecc_read_cnt++;
            }
            if (RMW_ENABLE && c->mask_wcmd) {
                merge_read_cnt++;
            }
            read_cnt++;
            access_bank_delay[c->bankIndex].enable = true;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = READ_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            has_wakeup[c->rank] = false;
            RdCntBl[c->bl] ++;
            if ((IS_LP5 && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSR; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: READ_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<<" bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: READ_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<<" bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case READ_P_CMD : {
            if (RDATA_TYPE == 0) {
                for (auto &trans : transactionQueue) {
                    if (c->task != trans->task) continue;
                    if (trans->issue_size != 0) continue;
                    if (trans->mask_wcmd==true) continue;
                    pfq_->gen_rresp(c->task, c->channel);
                    break;
                }
            }
            if (IECC_ENABLE && tasks_info[c->task].rd_ecc) {
                ecc_read_cnt++;
            }
            if (RMW_ENABLE && c->mask_wcmd) {
                merge_read_cnt++;
            }
            read_p_cnt++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = READ_P_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = 1;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            bankStates[c->bankIndex].has_rhit_break = false;
            bankStates[c->bankIndex].has_dummy_tout = false;
            has_wakeup[c->rank] = false;
            RdCntBl[c->bl] ++;
            if ((IS_LP5 && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSR; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex<<" pageOpenTime="
                            <<bankStates[c->bankIndex].state->pageOpenTime
                            <<" open_time="<<(now() - bankStates[c->bankIndex].state->pageOpenTime)
                            <<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: READ_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: READ_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            write_cnt++;
            access_bank_delay[c->bankIndex].enable = true;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = WRITE_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if ((IS_LP5 && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_P_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            write_p_cnt++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = WRITE_P_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = 1;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            bankStates[c->bankIndex].has_rhit_break = false;
            bankStates[c->bankIndex].has_dummy_tout = false;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if ((IS_LP5 && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex
                            <<" pageOpenTime="<<bankStates[c->bankIndex].state->pageOpenTime
                            <<" open_time="<<(now() - bankStates[c->bankIndex].state->pageOpenTime)
                            <<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_MASK_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            mwrite_cnt ++;
            access_bank_delay[c->bankIndex].enable = true;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = WRITE_MASK_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if ((IS_LP5 && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_MASK_P_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            mwrite_p_cnt ++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = WRITE_MASK_P_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = 1;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            bankStates[c->bankIndex].has_rhit_break = false;
            bankStates[c->bankIndex].has_dummy_tout = false;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if ((IS_LP5 && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex
                            <<" pageOpenTime="<<bankStates[c->bankIndex].state->pageOpenTime
                            <<" open_time="<<(now() - bankStates[c->bankIndex].state->pageOpenTime)
                            <<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="
                        <<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row
                        <<" trans_size="<<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="
                        <<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row
                        <<" trans_size="<<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case ACTIVATE1_CMD : {
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastCmdType = c->type;
            bankStates[c->bankIndex].state->lastCmdPri = c->pri;
            bankStates[c->bankIndex].state->act_executing = true;
            act_executing[c->bankIndex] = true;
            for (auto &trans : transactionQueue) {
                if (c->task == trans->task) {
                    trans->act_executing = true;
                    break;
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: ACTIVATE1_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: ACTIVATE1_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp<<endl);
            }
            break;
        }
        case ACTIVATE2_CMD : {
            if ((bankStates[c->bankIndex].state->lastRow & 0x80000000) != 0x80000000) {
                if ((bankStates[c->bankIndex].state->lastRow & 0xFFFFFF00) == (c->row & 0xFFFFFF00)) {
                    if (c->type == DATA_READ) samerow_mask_rdcnt ++;
                    else samerow_mask_wrcnt ++;
                }
            }
            if (bankStates[c->bankIndex].state->lastPrechargeSource == 1 && bankStates[c->bankIndex].state->lastRow == c->row) {
                bank_cnt_ehs[c->bankIndex] ++;
            }

            active_cnt++;
            active_cmd_cnt[c->bankIndex] ++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bankStates[c->bankIndex].state->lastCommand = ACTIVATE2_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastCmdType = c->type;
            bankStates[c->bankIndex].state->lastCmdPri = c->pri;
            bankStates[c->bankIndex].state->lastRow = c->row;
            bankStates[c->bankIndex].state->pageOpenTime = now();
            act_executing[c->bankIndex] = false;
            act_cmd_num ++;
            for (auto &trans : transactionQueue) {
                if (trans->task != c->task) continue;
                if (trans->transactionType != DATA_READ) trans->trcd_met = now() + tRCD_WR;
                else trans->trcd_met = now() + tRCD;
                break;
            }
            page_act_cnt ++;
            bankStates[c->bankIndex].state->act_executing = false;
            for (auto &trans : transactionQueue) {
                if (c->task == trans->task) {
                    trans->act_executing = false;
                    break;
                }
            }
            if (has_bypact_exec) {
                has_bypact_exec = false;
                for (auto &trans : transactionQueue) trans->arb_time ++;
            }
            for (auto &trans : transactionQueue) {
                if (c->task == trans->task) {
                    trans->has_send_act = true;
                    break;
                }
            }

            bankStates[c->bankIndex].row_hit_cnt = 0;
            bankStates[c->bankIndex].row_miss_cnt = 0;
            for (auto &t : transactionQueue) {
                if (c->bankIndex != t->bankIndex) continue;
                if (t->row == c->row) bankStates[c->bankIndex].row_hit_cnt ++;
                else bankStates[c->bankIndex].row_miss_cnt ++;
            }

            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: ACTIVATE2_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: ACTIVATE2_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_SB_CMD : {
            if ((c->bankIndex % NUM_BANKS) >= pbr_bank_num) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] PRECHAGE_SB_CMD, rank="<<c->rank
                        <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                assert(0);
            }
            precharge_sb_cnt++;
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                unsigned bank_tmp = c->bankIndex + sbr_bank * pbr_bank_num;
                access_bank_delay[bank_tmp].enable = false;
                access_bank_delay[bank_tmp].cnt = 0;
                bankStates[bank_tmp].state->lastCommand = PRECHARGE_SB_CMD;
                bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
                bankStates[bank_tmp].ser_rhit_cnt = 0;
                bankStates[bank_tmp].has_rhit_break = false;
                bankStates[bank_tmp].has_dummy_tout = false;
            }
            for (auto &trans : transactionQueue) {
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_tmp = c->bankIndex + sbr_bank * pbr_bank_num;
                    if (bank_tmp == trans->bankIndex) trans->act_executing = false;
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_SB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_SB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_PB_CMD : {   //todo: revise for enh_send_precharge
            precharge_pb_cnt++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bankStates[c->bankIndex].state->lastCommand = PRECHARGE_PB_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = c->cmd_source;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            bankStates[c->bankIndex].has_rhit_break = false;
            bankStates[c->bankIndex].has_dummy_tout = false;
            for (auto &trans : transactionQueue) {
                if (c->bankIndex == trans->bankIndex) trans->act_executing = false;
            }
            if (c->cmd_source == 0) rowconf_pre_cnt[c->bankIndex] ++;
            else if (c->cmd_source == 1) pageto_pre_cnt[c->bankIndex] ++;
            else if (c->cmd_source == 2) func_pre_cnt[c->bankIndex] ++;
            else if (c->cmd_source == 3) rhit_break_pre_cnt[c->bankIndex] ++;
            else if (c->cmd_source == 4) dummy_tout_pre_cnt[c->bankIndex] ++;
            if (c->cmd_source == 2) {
                refreshPerBank[c->bankIndex].refreshWaitingPre = true;
            } else {
                refreshPerBank[c->bankIndex].refreshWaitingPre = false;
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex<<" pageOpenTime="
                            <<bankStates[c->bankIndex].state->pageOpenTime<<" open_time="
                            <<(now()-bankStates[c->bankIndex].state->pageOpenTime)<<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_PB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="
                        <<c->row<<" matgrp="<<matgrp<<" POSTPND_AB="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_PB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="
                        <<c->row<<" matgrp="<<matgrp<<" POSTPND_AB="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_AB_CMD : {
            if ((c->bankIndex % sc_bank_num) != 0) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] PRECHAGE_AB_CMD, rank="<<c->rank
                        <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                assert(0);
            }
            precharge_ab_cnt++;
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                unsigned bank_tmp = c->rank * NUM_BANKS + bank;
                unsigned sub_channel = bank / sc_bank_num;
                if (EM_ENABLE && EM_MODE==2 && c->rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode
                access_bank_delay[bank_tmp].enable = false;
                access_bank_delay[bank_tmp].cnt = 0;
                bankStates[bank_tmp].state->lastCommand = PRECHARGE_AB_CMD;
                bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
                bankStates[bank_tmp].ser_rhit_cnt = 0;
                bankStates[bank_tmp].has_rhit_break = false;
                bankStates[bank_tmp].has_dummy_tout = false;
            }
            for (auto &trans : transactionQueue) {
                if (c->rank == trans->rank) trans->act_executing = false;
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_AB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_AB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case REFRESH_CMD : {
            if ((c->bankIndex % sc_bank_num) != 0) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] REFRESH_CMD, rank="<<c->rank
                        <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                assert(0);
            }
            refresh_ab_cnt ++; 
            rank_refresh_cnt[c->rank][sub_channel] ++;
            refreshALL[c->rank][sub_channel].refreshing = true;
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                unsigned bank_tmp = c->rank * NUM_BANKS + i;
                unsigned sub_channel = i / sc_bank_num;
                if (EM_ENABLE && EM_MODE==2 && c->rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode
                bankStates[bank_tmp].finish_refresh_pb = false;
                access_bank_delay[bank_tmp].enable = false;
                access_bank_delay[bank_tmp].cnt = 0;
                bankStates[bank_tmp].state->lastCommand = REFRESH_CMD;
                bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
            }
            refreshALL[c->rank][sub_channel].refresh_cnt --;
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: REFRESH_CMD task="<<c->task<<" QoS="<<c->pri
                        << " rank="<<c->rank<<" sc="<<sub_channel<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    DEBUGN(" rank"<<rank<<"r="<<+r_rank_cnt[rank]<<" rank"<<rank<<"w="<<+w_rank_cnt[rank]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: REFRESH_CMD task="<<c->task<<" QoS="<<c->pri
                        << " rank="<<c->rank<<" sc="<<sub_channel<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    PRINTN(" rank"<<rank<<"r="<<+r_rank_cnt[rank]<<" rank"<<rank<<"w="<<+w_rank_cnt[rank]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PER_BANK_REFRESH_CMD : {
            if (ENH_PBR_EN) {
                if ((c->fst_bankIndex % NUM_BANKS) >= NUM_BANKS || (c->lst_bankIndex % NUM_BANKS) >= NUM_BANKS) {
                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] ENHANCED PER_BANK_REFRESH_CMD, rank="<<c->rank
                            <<" sid="<<c->sid<<", bank="<<c->bankIndex<<", fst_bank="<<c->fst_bankIndex<<", lst_bank="<<c->lst_bankIndex);
                    assert(0);
                }
                refresh_pb_cnt ++;
                perbank_refresh_cnt[c->fst_bankIndex] ++;        //todo: need change for statistics
                perbank_refresh_cnt[c->lst_bankIndex] ++;        //todo: need change for statistics
            
                access_bank_delay[c->fst_bankIndex].enable = false;
                access_bank_delay[c->fst_bankIndex].cnt = 0;
                bankStates[c->fst_bankIndex].state->lastCommand = PER_BANK_REFRESH_CMD;
                bankStates[c->fst_bankIndex].state->lastCmdSource = c->cmd_source;
                refreshPerBank[c->fst_bankIndex].refreshWaiting = false;
                refreshPerBank[c->fst_bankIndex].refreshing = true;
                refreshPerBank[c->fst_bankIndex].refreshWaitingPre = false;
                access_bank_delay[c->lst_bankIndex].enable = false;
                access_bank_delay[c->lst_bankIndex].cnt = 0;
                bankStates[c->lst_bankIndex].state->lastCommand = PER_BANK_REFRESH_CMD;
                bankStates[c->lst_bankIndex].state->lastCmdSource = c->cmd_source;
                refreshPerBank[c->lst_bankIndex].refreshWaiting = false;
                refreshPerBank[c->lst_bankIndex].refreshing = true;
                refreshPerBank[c->lst_bankIndex].refreshWaitingPre = false;
                
                //added for SBR_WEIGHT_ENH_MODE==1
                pre_enh_pbr_bagroup[c->rank] = (c->fst_bankIndex % NUM_BANKS) % pbr_sb_num;

                if (PRINT_SCH) {
                    DEBUGN(setw(10)<<now()<<" -- SCH :: ENHANCED PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" fst_bank="<<c->fst_bankIndex<<" lst_bank="<<
                            c->lst_bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(endl);
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- SCH :: ENHANCED PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" fst_bank="<<c->fst_bankIndex<<" lst_bank="<<
                            c->lst_bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                }
            
            } else {
                if ((c->bankIndex % (NUM_BANKS/sc_num)) >= pbr_bank_num) {
                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] PER_BANK_REFRESH_CMD, rank="<<c->rank
                            <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                    assert(0);
                }
                refresh_pb_cnt ++;
                perbank_refresh_cnt[c->rank * pbr_bank_num * sc_num + c->bankIndex % pbr_bank_num + sub_channel * bank_pair_start] ++;      //todo: revise for e-mode
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_tmp = c->bankIndex + sbr_bank * pbr_bank_num;
                    access_bank_delay[bank_tmp].enable = false;
                    access_bank_delay[bank_tmp].cnt = 0;
                    bankStates[bank_tmp].state->lastCommand = PER_BANK_REFRESH_CMD;
                    bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
                    refreshPerBank[bank_tmp].refreshWaiting = false;
                    refreshPerBank[bank_tmp].refreshing = true;
                    refreshPerBank[bank_tmp].refreshWaitingPre = false;
                }
                if (PRINT_SCH) {
                    DEBUGN(setw(10)<<now()<<" -- SCH :: PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(endl);
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- SCH :: PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                }
            }
            break;
        }
        default : break;
    }


    // reorder arb group proority after one Cmd scheduled
    if (c != NULL && !SLOT_FIFO) {
        if (c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) {
            update_arb_group_pri(c, arb_group_pri_col, 0);
        } else if (c->cmd_type==activate_cmd) {
            update_arb_group_pri(c, arb_group_pri_row, 1);
        }
    }


    if ((c!=NULL) && !SLOT_FIFO && c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD && c->issue_size + c->trans_size >= c->data_size) {
        arb_group_cnt[c->arb_group]--;
        if (arb_group_cnt[c->arb_group] > arb_per_group) {
            ERROR(setw(10)<<now()<<" Wrong Arb Group Cnt, arb_group="<<c->arb_group<<", cnt="<<arb_group_cnt[c->arb_group]);
            assert(0);
        }
    }

    if (c != NULL) {
        generate_packet(c);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- generate_packet(c), task="<<c->task<<endl);
        }
    }
    if (CORE_CONCURR == 1 || (CORE_CONCURR != 1 && !core_concurr_en)) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- delete cmdqueue"<<endl);
        }
        for (auto &cmd : act_cmdqueue) delete cmd;
        act_cmdqueue.clear();
        for (auto &cmd : rw_cmdqueue) delete cmd;
        rw_cmdqueue.clear();
        for (auto &cmd : pre_cmdqueue) delete cmd;
        pre_cmdqueue.clear();
        // for (auto &cmd : CmdQueue) delete cmd;
        CmdQueue.clear();

    }
}

void PTC::refresh(unsigned sc) {
    bool combo_emode_unable = EM_ENABLE && (EM_MODE==2) && (sc==1); 
    if (AREF_EN || PBR_EN) { // other spec
        if (DMC_V590 && SBR_IDLE_ADAPT_EN) {
            if (now() % SBR_IDLE_ADAPT_WIN == 0 && now() != 0) {
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
                    rank_send_pbr[rank][sc] = rank_cnt_sbridle[rank][sc] >= SBR_IDLE_ADAPT_LEVEL;      //todo: revise for e-mode
                    rank_cnt_sbridle[rank][sc] = 0;                                                    //todo: revise for e-mode
                }
            }
        }
        

        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (refreshALL[rank][sc].refresh_cnt > 8) {
                ERROR(setw(10)<<now()<<" Postponed exceed 8, rank="<<rank<<", sc="<<sc<<", cnt="<<refreshALL[rank][sc].refresh_cnt);
                assert(0);
            }
        }
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
            if ((RankState[rank].lp_state == IDLE || RankState[rank].lp_state == PDE ||
                    RankState[rank].lp_state == PD || RankState[rank].lp_state == PDLP ||
                    RankState[rank].lp_state == PDX) && 
                    ((now() + ref_offset[rank] - AsrefTime[rank] - SrpdTime[rank]) % tREFI) == 0) {    //todo: revise for e-mode
                refreshALL[rank][sc].refresh_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PSTPND_ADD :: POSTPND="<<refreshALL[rank][sc].refresh_cnt<<", rank="
                            <<rank<<", sc="<<sc<<", AsrefTime="<<AsrefTime[rank]<<", SrpdTime="<<SrpdTime[rank]<<endl);
                }
            }
            if (refresh_pbr_has_finish[rank][sc]) {
                refreshALL[rank][sc].refresh_cnt --;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PSTPND_SUB :: POSTPND="<<refreshALL[rank][sc].refresh_cnt<<", rank="
                            <<rank<<", sc="<<sc<<", AsrefTime="<<AsrefTime[rank]<<", SrpdTime="<<SrpdTime[rank]<<endl);
                }
            }
        }
    }

    // check refreshing state with ptc slot
    // for (size_t i = 0; i < NUM_RANKS; i ++) {
    //     for (size_t j = 0; j < NUM_BANKS/sc_num; j++) {
    //         unsigned bank = i * NUM_BANKS + j + sc*sc_bank_num;
    //         if (refreshPerBank[bank].refreshing && bank_cnt[bank]>0 && (refreshALL[i][sc].refresh_cnt < (PBR_PSTPND_LEVEL-1) && PBR_PSTPND_LEVEL>0)) {
    //             ERROR(setw(10)<<now()<<" -- Perf Slot Wasted When Rrefreshing, rank="<<i<<", sc="<<sc<<", bank="<<bank);
    //             assert(0);
    //         }
    //     }
    // } 


    if (AREF_EN) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (RankState[rank].lp_state == ASREFE) continue;
            if (RankState[rank].lp_state == ASREF) continue;
            if (RankState[rank].lp_state == ASREFX) continue;
            if (RankState[rank].lp_state == SRPDE) continue;
            if (RankState[rank].lp_state == SRPD) continue;
            if (RankState[rank].lp_state == SRPDLP) continue;
            if (RankState[rank].lp_state == SRPDX) continue;
            if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
            all_bank_refresh(rank, sc);
        }
    }

    if (PBR_EN) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
            if (refresh_pbr_has_finish[rank][sc]) {
                refresh_pbr_has_finish[rank][sc] = false;
            }
            if ((!IS_HBM2E && !IS_HBM3) && !PBR_PARA_EN) {
                if (ENH_PBR_EN){     //todo: revise for e-mode
                    unsigned pbr_bank_cnt = 0;
                    unsigned pre_bank_tmp = 0;
                    for (size_t i = 0; i < pbr_sb_group_num; i ++) {
                        for (size_t j = 0; j < pbr_sb_num; j ++) {
                            unsigned bank_tmp = rank * NUM_BANKS + i * pbr_sb_group_num + j;
                            if (refreshPerBank[bank_tmp].refreshWaiting || refreshPerBank[bank_tmp].refreshing) {
                                if ((bank_tmp % pbr_sb_group_num)!=(pre_bank_tmp % pbr_sb_group_num) && (((pbr_bank_cnt==1) && (pre_bank_tmp==0)) || pre_bank_tmp!=0)) {
                                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Enhanced Refresh Wrong Bank pairs, rank="<<rank<<" bank="<<bank_tmp<<" pre bank="<<pre_bank_tmp);
                                    assert(0);
                                }
                                pre_bank_tmp = bank_tmp;
                                pbr_bank_cnt ++;
                                if (pbr_bank_cnt > 2) {
                                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Enhanced Refresh bank exceed 2, rank="<<rank);
                                    assert(0);
                                }
                            }
                        }
                    }
                } else {
                    for (size_t i = 0; i < pbr_bank_num; i ++) {
                        unsigned bank_tmp = rank * NUM_BANKS + i + (sc*sc_bank_num);
                        if (refreshPerBank[bank_tmp].refreshWaiting || refreshPerBank[bank_tmp].refreshing) {
                            for (size_t j = i + 1; j < pbr_bank_num; j ++) {
                                unsigned bank_loop = rank * NUM_BANKS + j + (sc*sc_bank_num);
                                if (refreshPerBank[bank_loop].refreshWaiting || refreshPerBank[bank_loop].refreshing) {
                                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Refresh bank exceed 2, rank="<<rank<<", sc="<<sc);
                                    assert(0);
                                }
                            }
                            break;
                        }
                    }
                }
            }
            if (RankState[rank].lp_state == ASREFE) continue;
            if (RankState[rank].lp_state == ASREF) continue;
            if (RankState[rank].lp_state == ASREFX) continue;
            if (RankState[rank].lp_state == SRPDE) continue;
            if (RankState[rank].lp_state == SRPD) continue;
            if (RankState[rank].lp_state == SRPDLP) continue;
            if (RankState[rank].lp_state == SRPDX) continue;
            if ((RankState[rank].lp_state == PD || RankState[rank].lp_state == PDLP) && !PD_PBR_EN) continue;
            if (ENH_PBR_EN) {
                enh_per_bank_refresh(rank, sc);
            } else {
                per_bank_refresh(rank, sc);
            }
        }
    }
}

void PTC::all_bank_refresh(unsigned rank, unsigned sc) {
    if (RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREF ||
            RankState[rank].lp_state == ASREFX || RankState[rank].lp_state == SRPDE ||
            RankState[rank].lp_state == SRPD || RankState[rank].lp_state == SRPDLP ||
            RankState[rank].lp_state == SRPDX || refreshALL[rank][sc].refresh_cnt == 0) {
        return;
    }

    //guarantee rd/wr cmd with issue_size!=0 not broken by refresh
    bool has_issue_size = false;
    for (auto &trans : transactionQueue) {
        unsigned trans_sc = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
        if (trans->issue_size != 0 && trans->rank == rank && trans_sc == sc) {
            has_issue_size =true;
        }
    }

    if (has_issue_size) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Rd/Wr cmd (issue!=0) must be send before all bank refresh"<<" rank="<<rank<<", sc="<<sc<<endl);
        }
        return;
    }


    // guarantee activate2 followed with activate1 within 8 cycles
    bool act2_left = false;
    for (size_t bank = 0; bank < NUM_BANKS * NUM_RANKS; bank ++) {
        if (act_executing[bank]){
            act2_left = true;
        }        
    }

    if (act2_left){     //todo: revise for e-mode
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Act2 must be send before refresh"<<" rank="<<rank<<", sc="<<sc<<endl);
        }
        return;
    }
    
    unsigned bank_start = sc * (NUM_BANKS/sc_num);
//    unsigned bank_pair_start = sc * pbr_bank_num; 

    if (refreshALL[rank][sc].refreshWaiting && !refreshALL[rank][sc].refreshing) {
        bool idle = true;
        bool precharge_en = true;
        for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
            if (bankStates[bank_tmp].state->currentBankState == RowActive) {
                idle = false;
                break;
            }
        }
        for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
            if ((now() + 1) < bankStates[bank_tmp].state->nextPrecharge || bankStates[bank_tmp].state->act_executing) {
                precharge_en = false;
                break;
            }
        }
        if (arb_enable && !idle && precharge_en) {
            funcState[rank].wakeup = true;
//            if (RankState[rank].lp_state == IDLE) {
            if (RankState[rank].lp_state == IDLE && even_cycle) {    //every other command, even;
                while (!CmdQueue.empty()) CmdQueue.erase(CmdQueue.begin());
                while (!rw_cmdqueue.empty()) rw_cmdqueue.erase(rw_cmdqueue.begin());
                while (!act_cmdqueue.empty()) act_cmdqueue.erase(act_cmdqueue.begin());
                while (!pre_cmdqueue.empty()) pre_cmdqueue.erase(pre_cmdqueue.begin());
                Cmd *c = new Cmd;
                c->rank = rank;
                c->group = 0;
                c->bank = 0;
                c->row = 0;
                c->cmd_type = PRECHARGE_AB_CMD;
                c->bankIndex = rank * NUM_BANKS + sc * sc_bank_num;
                c->cmd_source = 2;
                c->task = 0xFFFFFFFFFFFFFF9;
                CmdQueue.push_back(c);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- REQ :: refresh, precharge bank"<<c->bank<<", task="<<c->task<<", rank="<<rank<<", sc="<<sc<<endl);
                }
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold PRECHARGE in lp state, rank="<<rank<<", sc="<<sc<<endl);
                }
            }
        }

        //if true , it means this rank is in Idle states
        if (idle) {
            bool abr_timing_met = true;
            for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                if ((now() + 1) < bankStates[rank * NUM_BANKS + bank + bank_start].state->nextAllBankRefresh) {
                    abr_timing_met = false;
                    break;
                }
            }
//            if (arb_enable && abr_timing_met) {
            if (arb_enable && abr_timing_met) { 
                funcState[rank].wakeup = true;
//                if (RankState[rank].lp_state == IDLE) {
                if (RankState[rank].lp_state == IDLE && even_cycle) {   //every other command, even;
                    //refresh this rank
                    while (!CmdQueue.empty()) CmdQueue.erase(CmdQueue.begin());
                    while (!rw_cmdqueue.empty()) rw_cmdqueue.erase(rw_cmdqueue.begin());
                    while (!act_cmdqueue.empty()) act_cmdqueue.erase(act_cmdqueue.begin());
                    while (!pre_cmdqueue.empty()) pre_cmdqueue.erase(pre_cmdqueue.begin());
                    //step 2 : construct a refresh command
                    Cmd *c = new Cmd;
                    c->rank = rank;
                    c->bankIndex = rank * NUM_BANKS + sc * sc_bank_num;
                    c->cmd_type = REFRESH_CMD;
                    c->cmd_source = 2;
                    c->task = 0xFFFFFFFFFFFFFFA;
                    CmdQueue.push_back(c);
                    force_pbr_refresh[rank][sc] = false;

                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- REQ :: REFRESH rank"<<c->rank<<", task="<<c->task<<" rank="<<rank<<", sc="<<sc<<endl);
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold REFRESH in lp state, rank="<<rank<<", sc="<<sc<<endl);
                    }
                }
            }
        }
    }

    if (refreshALL[rank][sc].refresh_cnt >= ABR_PSTPND_LEVEL) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- POS :: refresh cnt="<<refreshALL[rank][sc].refresh_cnt<<", force refresh"<<", rank="<<rank<<", sc="<<sc<<endl);
        }
        refreshALL[rank][sc].refreshWaiting = true;
    } else if (refreshALL[rank][sc].refresh_cnt > 0) {
        if (sc_cnt[rank][sc] == 0) {      //todo: revise for e-mode
            if (SBR_IDLE_EN) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- POS :: refresh_cnt="<<refreshALL[rank][sc].refresh_cnt
                            <<", rank="<<rank<<", sc="<<sc<<" is Idle, will refresh by pbr"<<endl);
                }
            } else if (!rank_send_pbr[rank][sc]) {
                bool no_bank_pbr = true;
                for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                    unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
                    if (refreshPerBank[bank_tmp].refreshWaiting || refreshPerBank[bank_tmp].refreshing) {
                        no_bank_pbr = false;
                        break;
                    }
                }
                bool idle_abr_send = true;
                if (no_bank_pbr && idle_abr_send) {
                    refreshALL[rank][sc].refreshWaiting = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- POS :: refresh_cnt="<<refreshALL[rank][sc].refresh_cnt
                                <<", rank="<<rank<<", sc="<<sc<<" is Idle, will refresh by abr"<<endl);
                    }
                }
            }
        }
    }
}
void PTC::per_bank_refresh(unsigned rank, unsigned sc) {
    if (refreshALL[rank][sc].refresh_cnt == 0 || refreshALL[rank][sc].refreshWaiting ||
            RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREF ||
            RankState[rank].lp_state == ASREFX || RankState[rank].lp_state == SRPDE ||
            RankState[rank].lp_state == SRPD || RankState[rank].lp_state == SRPDLP ||
            RankState[rank].lp_state == SRPDX) return;
    if (now() < sbr_gap_cnt[rank]) return;       //todo: revise for e-mode

    // guarantee activate2 followed with activate1 within 8 cycles
    bool act2_left = false;
    for (size_t bank = 0; bank < NUM_BANKS * NUM_RANKS; bank ++) {
        if (act_executing[bank]){
            act2_left = true;
        }        
    }

    if (act2_left){     //todo: revise for e-mode
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Act2 must be send before refresh"<<", sc="<<sc<<endl);
        }
        return;
    }

    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    unsigned bank_pair_start = sc * pbr_bank_num; 
    bool have_bank_refresh = false; // send pbr one by one if set true
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (IS_HBM2E || IS_HBM3 || PBR_PARA_EN) {
            if (!refreshPerBank[bank_tmp].refreshWaiting) continue;
        } else {
            if (!refreshPerBank[bank_tmp].refreshWaiting && !refreshPerBank[bank_tmp].refreshing) continue;
        }
        have_bank_refresh = true;
        break;
    }

    if (!have_bank_refresh) {
        for (size_t i = 0 ; i < pbr_bank_num; i++) {
            PbrWeight[rank][i + bank_pair_start].bank_pair_cnt = 0;
            PbrWeight[rank][i + bank_pair_start].send_pbr = false;
        }
        
        for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
            unsigned has_bank_open = false;
            unsigned has_cmd_vld = false;
            unsigned bankIndex = rank * NUM_BANKS + bank + bank_start;
            unsigned another_bankIndex = rank * NUM_BANKS + bank + pbr_bank_num + bank_start;
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + sbr_bank * pbr_bank_num + bank + bank_start;
                if (bankStates[bank_tmp].state->currentBankState == RowActive) {
                    has_bank_open = 1;
                    break;
                }
            }
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + sbr_bank * pbr_bank_num + bank + bank_start;
                if (bank_cnt[bank_tmp] > 0) {
                    has_cmd_vld = 1;
                    break;
                }
            }

            if (SBR_WEIGHT_MODE == 0) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_bank_open;
            else if (SBR_WEIGHT_MODE == 1) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 2) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_bank_open + has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 3) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_bank_open + has_cmd_vld * 2;
            else PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = bank_cnt[bankIndex] + bank_cnt[another_bankIndex];    // added for SBR_WEIGHT_MODE=4/5, select bank pair with least/most cmd

            //added for SBR_WEIGHT_ENH_MODE=3/4 as bonus, select bank pair with least/most cmd
            //based on SBR_WEIGHT_MODE = 0/1/2/3, 4/5 forbidden
            if (SBR_WEIGHT_ENH_MODE == 3 || SBR_WEIGHT_ENH_MODE == 4) {
                bank_pair_cmd_cnt[rank][bank + bank_pair_start] = bank_cnt[bankIndex] + bank_cnt[another_bankIndex];
            }

        }


#if 1
        // HBM刷新优化：根据SBR_REQ_MODE筛选bank pair
        // if (SBR_REQ_MODE == 0 || refreshALL[rank][sc].refresh_cnt >= PBR_PSTPND_LEVEL) {
        //     // 模式0或刷新计数达到阈值：只检查trfcpb时序
        //     for (size_t i = 0; i < NUM_BANKS / sc_num; i++) {
        //         unsigned bank = rank * NUM_BANKS + i + bank_start;
        //         bool trfcpb_tim_met = (bankStates[bank].state->trfcpb_met_time <= now() + 1);
        //         if (!trfcpb_tim_met) {
        //             unsigned pair_idx = (i % pbr_bank_num) + bank_pair_start;
        //             PbrWeight[rank][pair_idx].bank_pair_cnt = 0xFFFFFFFF;
        //         }
        //     }
        // } else { // SBR_REQ_MODE == 1
        //     // 模式1：HBM推迟刷新
        //     for (size_t i = 0; i < NUM_BANKS / sc_num; i++) {
        //         unsigned bank = rank * NUM_BANKS + i + bank_start;
        //         // 条件1：bank非Active 或 已刷新bank数足够 或 valid bank数超标
        //         bool condition1 = (bankStates[bank].state->currentBankState != RowActive)
        //                     || (refresh_cnt_pb[rank][sc] >= (pbr_bank_num - 2))
        //                     // || (SIMPLE_BANKTABLE_ENABLE && (simple_bank_table->valid_banks.size() > SBR_FRCST_NUM))
        //                     || (!SIMPLE_BANKTABLE_ENABLE && table_use_cnt > SBR_FRCST_NUM);
        //         if (condition1) {
        //             bool trfcpb_tim_met = (bankStates[bank].state->trfcpb_met_time <= now() + 1);
        //             bool ref_ba_slt_vld_num_c = (bank_cnt[bank] > 0) && (!SIMPLE_BANKTABLE_ENABLE);
        //             // bool lc_tb_pbr_met = !((SIMPLE_BANKTABLE_ENABLE && simple_bank_table->isBaSltVld(bank)) && (bankStates[bank].state->trc_met_time > now() + 1));
        //             if (trfcpb_tim_met && !ref_ba_slt_vld_num_c) {
        //                 // 可以刷新，保留权重
        //                 if (DEBUG_BUS) {
        //                     PRINTN(setw(10) << now() << " -- REF :: find bank can refresh, bank=" << bank << endl);
        //                 }
        //             } else {
        //                 unsigned pair_idx = (i % pbr_bank_num) + bank_pair_start;
        //                 PbrWeight[rank][pair_idx].bank_pair_cnt = 0xFFFFFFFF;
        //             }
        //         } else {
        //             unsigned pair_idx = (i % pbr_bank_num) + bank_pair_start;
        //             PbrWeight[rank][pair_idx].bank_pair_cnt = 0xFFFFFFFF;
        //         }
        //     }
        // }

        // select refresh bank pair with searching perf bank cnt
        if (refreshALL[rank][sc].refresh_cnt < PRE_PBR_PSTPND_LEVEL) {    //todo: revise for e-mode
            bool perf_bank_exist_pre = false;
            for (size_t i = 0; i < NUM_BANKS/sc_num; i++) {
                unsigned bank = rank * NUM_BANKS + i + bank_start;
                perf_bank_exist_pre = bankStates[bank].perf_bankrd_conflict || bankStates[bank].perf_bankwr_conflict;
                if (perf_bank_exist_pre) {
                    PbrWeight[rank][i % pbr_bank_num + bank_pair_start].bank_pair_cnt = 0xFFFFFFFF;
                }
            }
        }
        // select refresh bank pair with searching ptc bank cnt
        // lock two banks, backpress two banks
        if (PBR_LOCK_MODE == 0) {
            if (refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {    //todo: revise for e-mode
                bool perf_bank_exist = false;
                for (size_t i = 0; i < NUM_BANKS/sc_num; i++) {
                    unsigned bank = rank * NUM_BANKS + i + bank_start;
                    perf_bank_exist = bankStates[bank].perf_bankrd_conflict || bankStates[bank].perf_bankwr_conflict;
                    if ((bank_cnt[bank] > 0) || (perf_bank_exist && PBR_SEL_MODE==1)) {
                        PbrWeight[rank][i % pbr_bank_num + bank_pair_start].bank_pair_cnt = 0xFFFFFFFF;
                    }
                }
            }
        }
        // lock one bank, backpress two banks
        if (PBR_LOCK_MODE == 1) {
            if (refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {    //todo: revise for e-mode
                for (size_t i = 0; i < pbr_bank_num; i ++) {
                    bool no_cmd_bank = false;
                    for (size_t j = 0; j < pbr_bg_num; j ++) {
                        unsigned bank = rank * NUM_BANKS + i + j * pbr_bank_num + bank_start;
                        if (bank_cnt[bank] == 0) {
                            no_cmd_bank = true;
                            break;
                        }
                    }
                    if (!no_cmd_bank) {
                        PbrWeight[rank][i % pbr_bank_num + bank_pair_start].bank_pair_cnt = 0xFFFFFFFF;
                    }
                }
            }
        }
#endif

        unsigned bank_pair_cnt_min = 0xFFFFFFFF;
        unsigned bank_pair_cnt_min_num = 0xFFFFFFFF;
        bool     pre_sch_bankIndex_met = false;
        unsigned pre_sch_bank_pair_cnt = 0xFFFFFFFF;
        unsigned pre_sch_bank_pair = 0xFFFFFFFF;
        unsigned pbr_bankidx = 0xFFFFFFFF;
        unsigned pbr_bankidx_another = 0xFFFFFFFF;
        for (size_t bank = 0; bank < pbr_bank_num; bank++) {
            if (bank_pair_cnt_min > PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt &&
                    !bankStates[rank * NUM_BANKS + bank + bank_start].finish_refresh_pb &&
                    (!DMC_V580 || PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt != 0xFFFFFFFF) &&
                    (SBR_WEIGHT_MODE!=5)) {   //todo: revise for e-mode
                bank_pair_cnt_min = PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt;
                bank_pair_cnt_min_num = bank + bank_pair_start;
            }

            //added for SBR_WEIGHT_MODE=5, select bank pair with most cmd
            if ((bank_pair_cnt_min < PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt || bank_pair_cnt_min == 0xFFFFFFFF) &&
                    !bankStates[rank * NUM_BANKS + bank + bank_start].finish_refresh_pb &&
                    (!DMC_V580 || PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt != 0xFFFFFFFF) &&
                    (SBR_WEIGHT_MODE==5)) {   //todo: revise for e-mode
                bank_pair_cnt_min = PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt;
                bank_pair_cnt_min_num = bank + bank_pair_start;
            }

            pbr_bankidx_another = rank * NUM_BANKS + bank + pbr_bank_num + bank_start;
            pbr_bankidx = rank * NUM_BANKS + bank + bank_start;
            // added for SBR_WEIGHT_ENH_MODE=2
            if ((pbr_bankidx==pre_sch_bankIndex[rank] || (pbr_bankidx_another==pre_sch_bankIndex[rank]))&& (SBR_WEIGHT_ENH_MODE==2) && 
                    (!bankStates[pbr_bankidx].finish_refresh_pb && !bankStates[pbr_bankidx_another].finish_refresh_pb) &&
                    (!DMC_V580 || PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt != 0xFFFFFFFF)) {
                pre_sch_bankIndex_met = true;
                pre_sch_bank_pair_cnt = PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt; 
                pre_sch_bank_pair     = bank + bank_pair_start; 
            }

        }
        
        //added for SBR_WEIGHT_ENH_MODE=2, most recently scheduled cmd priority
        if (SBR_WEIGHT_ENH_MODE==2 && pre_sch_bankIndex_met &&
                pre_sch_bank_pair_cnt!=0xFFFFFFFF && pre_sch_bank_pair!=0xFFFFFFFF){
            if (bank_pair_cnt_min >= pre_sch_bank_pair_cnt) {
                bank_pair_cnt_min = PbrWeight[rank][pre_sch_bank_pair].bank_pair_cnt;
                bank_pair_cnt_min_num = pre_sch_bank_pair;
            }       
        }

        //added for SBR_WEIGHT_ENH_MODE=3/4, least/most cmd
        for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
            unsigned bank_pair = bank + bank_pair_start;
            unsigned bankIndex = rank * NUM_BANKS + bank + bank_start;
            if (bank_pair_cnt_min_num != 0xFFFFFFFF) {
                if (PbrWeight[rank][bank_pair].bank_pair_cnt == bank_pair_cnt_min &&
                        ((bank_pair_cmd_cnt[rank][bank_pair] < bank_pair_cmd_cnt[rank][bank_pair_cnt_min_num] && SBR_WEIGHT_ENH_MODE==3) ||
                        (bank_pair_cmd_cnt[rank][bank_pair] > bank_pair_cmd_cnt[rank][bank_pair_cnt_min_num] && SBR_WEIGHT_ENH_MODE==4)) &&
                        bank_pair_cnt_min_num != bank_pair && bank_pair_cnt_min_num!=0xFFFFFFFF && !bankStates[bankIndex].finish_refresh_pb){ 
                    bank_pair_cnt_min = PbrWeight[rank][bank_pair].bank_pair_cnt;
                    bank_pair_cnt_min_num = bank_pair;
                }
            }
        }

        if (bank_pair_cnt_min_num != 0xFFFFFFFF) {
            PbrWeight[rank][bank_pair_cnt_min_num].send_pbr = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PBR_WEIGHT :: rank="<<rank<<" bank="
                        <<(bank_pair_cnt_min_num/NUM_MATGRPS)<<" matgrp="
                        <<+bank_pair_cnt_min_num<<" sc="<<sc<<" win the arbitration!"<<endl);
            }
        }
    }

    refresh_cnt_pb[rank][sc] = 0;
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {       //todo: revise for e-mode
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (bankStates[bank_tmp].finish_refresh_pb) refresh_cnt_pb[rank][sc] ++;
        else forceRankBankIndex[rank][sc] = bank + bank_start;

        if (SBR_REQ_MODE == 0) {
            if (bank == pbr_bank_num - 1) {
                if (refresh_cnt_pb[rank][sc] == pbr_bank_num) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    for (size_t bank_sbr = 0; bank_sbr < pbr_bank_num; bank_sbr ++) {
                        for (size_t group_sbr = 0; group_sbr < pbr_bg_num; group_sbr ++) {
                            unsigned bank_idx = rank * NUM_BANKS + bank_sbr + group_sbr * pbr_bank_num + bank_start;
                            bankStates[bank_idx].finish_refresh_pb = false;
                        }
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: RANK "<<rank<<", SC "<<sc<<" has been refreshed by pbr"<<endl);
                    }
                } else {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: Force pbr rank "<<rank<<", SC "<<sc<<" has no idle bank not refreshed"<<endl);
                    }
                }
            }
        } else {
            if (bank == pbr_bank_num - 1) {
                if (refresh_cnt_pb[rank][sc] == pbr_bank_num) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    for (size_t bank_sbr = 0; bank_sbr < pbr_bank_num; bank_sbr ++) {
                        for (size_t group_sbr = 0; group_sbr < pbr_bg_num; group_sbr ++) {
                            unsigned bank_idx = rank * NUM_BANKS + bank_sbr + group_sbr * pbr_bank_num + bank_start;
                            bankStates[bank_idx].finish_refresh_pb = false;
                        }
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: RANK "<<rank<<", SC "<<sc<<" has been refreshed by pbr"<<endl);
                    }
                } else if (refresh_cnt_pb[rank][sc] >= SBR_FRCST_NUM) {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: Force pbr for the left banks in RANK "<<rank<<", SC "<<sc<<endl);
                    }
                }
            }
        }
    }

    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (!bankStates[bank_tmp].finish_refresh_pb) {
            bankStates[bank_tmp].en_refresh_pb = true;
            bankStates[bank_tmp].hold_refresh_pb = true;
        }
    }

    send_pb_precharge(rank, sc);
    send_pb_refresh(rank, sc);
}

void PTC::send_pb_precharge(unsigned rank, unsigned sc) {
    // unrefreshed bank is 2 left, force pbr
    bool have_bank_refresh = false; // send pbr one by one if set true
    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    unsigned bank_pair_start = sc * pbr_bank_num; 
    if ((!IS_HBM2E && !IS_HBM3) && !PBR_PARA_EN ) {
        for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
            if (refreshPerBank[bank_tmp].refreshing){
                have_bank_refresh = true;
                break;
            }
        }
    }
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (refresh_pbr_has_finish[rank][sc]) continue;
        if (force_pbr_refresh[rank][sc] && !bankStates[bank_tmp].finish_refresh_pb
                && !have_bank_refresh && PbrWeight[rank][bank + bank_pair_start].send_pbr) {
            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                if (arb_enable) {
                    for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                        unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                        if (!refreshPerBank[bank_idx].refreshWaiting) {
                            refreshPerBank[bank_idx].refreshWaiting = true;
                            refreshPerBank[bank_idx].refreshWaitingPre = true;
                            refreshPerBank[bank_idx].refreshing = false;
                            if (bankStates[bank_idx].state->currentBankState == RowActive) {
                                refreshPerBank[bank_idx].refreshWaitingPre = false;
                            }
                            pbr_hold_pre[rank] = true;             //todo, revise for e-mode
                            pbr_hold_pre_time[rank] = now() + 6;   //todo, revise for e-mode
                        }
                    }

                    bool ptc_cmd_exist = false;
                    bool rw_switch_trig = ((PreCmd.trans_type == DATA_READ) && que_read_cnt < PBR_LESS_CMD_LEVEL && que_write_cnt>0)
                                            || ((PreCmd.trans_type == DATA_WRITE) && que_write_cnt < PBR_LESS_CMD_LEVEL && que_read_cnt>0);
                    bool force_sch_cmd = false; 
                    for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                        unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                        if (PBR_LESS_CMD_EN && PBR_LESS_CMD_MODE==0) {  // rw_switch_trig
                            for (size_t i = 0; i < transactionQueue.size(); i++) {
                                if (transactionQueue[i]->bankIndex!=bank_idx) continue;
                                if (transactionQueue[i]->transactionType!=PreCmd.trans_type) continue;
                                force_sch_cmd = true;
                            }
                        }

                        if (bank_cnt[bank_idx] > 0 && PBR_LESS_CMD_EN && PBR_LESS_CMD_MODE==0 && rw_switch_trig && force_sch_cmd) {  // PBR_LOCK_MODE=0
                            ptc_cmd_exist = true;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- Force Pbr Hold :: Cmd Exist, rank="<<rank<<", bankIndex="<<bank_idx<<", sc="<<sc<<endl);
                            }
                            break;
                        } else if (bank_cnt[bank_idx] > 0 && PBR_LESS_CMD_EN && PBR_LESS_CMD_MODE==1) { // PBR_LOCK_MODE=1
                            ptc_cmd_exist = true;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- Force Pbr Hold :: Cmd Exist, rank="<<rank<<", bankIndex="<<bank_idx<<", sc="<<sc<<endl);
                            }
                            break;
                        }  
                    }

                    bool ready_send_pbr = true;
                    for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                        unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                        if (bankStates[bank_idx].state->currentBankState != Idle ||
                                (now() + 1) < bankStates[bank_idx].state->nextPerBankRefresh || issue_state[bank_idx]) {
                            ready_send_pbr = false;
                            break;
                        }
                    }
                    if (ready_send_pbr && ((tFAWCountdown[rank].size() < 4 && sc==0)      //todo: revise for e-mode
                            || (tFAWCountdown_sc1[rank].size() < 4 && sc==1)) && !ptc_cmd_exist
                            && (!ASREF_ENABLE || (ASREF_ENABLE && RankState[rank].asref_cnt != 0))) {
                        funcState[rank].wakeup = true;      
                        if (RankState[rank].lp_state == IDLE && even_cycle && (!IS_HBM3 || now()%2==1 || ODD_TDM)) {    // every other command, even;
                            Cmd *c = new Cmd;
                            c->state = working;
                            c->cmd_type = PER_BANK_REFRESH_CMD;
                            c->force_pbr = true;
                            c->bank = bankStates[bank_tmp].bank;
                            c->rank = bankStates[bank_tmp].rank;
                            c->group = bankStates[bank_tmp].group;
                            c->cmd_source = 2;
                            c->task = 0xFFFFFFFFFFFFFFB;
                            c->bankIndex = bankStates[bank_tmp].bankIndex;
                            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                                unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                                bankStates[bank_idx].hold_refresh_pb = false;
                            }
                            CmdQueue.push_back(c);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ :: force pbr, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                            }
                            return;
                        } else {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold FPBR in lp, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                            }
                        }
                    } else if (tFPWCountdown[rank].size() < 4) {     // no need revise for lpddr
                        bool sb_ready_send_pre = true;
                        for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                            unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                            bool cmd_exist = (bank_cnt[bank_idx] > 0) && PBR_LESS_CMD_EN;  
                            if (bankStates[bank_idx].state->currentBankState == RowActive) {
                                for (size_t sbr_bank_w = 0; sbr_bank_w < pbr_bg_num; sbr_bank_w ++) {
                                    unsigned bank_idx_w = bank_tmp + sbr_bank_w * pbr_bank_num;
                                    refreshPerBank[bank_idx_w].refreshWaiting = true;
                                    refreshPerBank[bank_idx_w].refreshing = false;
                                    pbr_hold_pre[rank] = true;       //todo: revise for e-mode
                                    if (bankStates[bank_idx_w].state->currentBankState == RowActive) {
                                        refreshPerBank[bank_idx_w].refreshWaitingPre = false;
                                    }
                                }
                                
                                if (IS_LP4 || IS_LP5 || IS_LP6){
                                    if ((now() + 1) < bankStates[bank_idx].state->nextPrecharge || issue_state[bank_idx]) {
                                        sb_ready_send_pre = false;
                                    }
                                } else {
                                    for (size_t sbr_bank_w = 0; sbr_bank_w < pbr_bg_num; sbr_bank_w ++) {
                                        unsigned bank_idx_w = bank_tmp + sbr_bank_w * pbr_bank_num;
                                        if ((now() + 1) < bankStates[bank_idx_w].state->nextPrecharge) {
                                            sb_ready_send_pre = false;
                                            break;
                                        }
                                    }
                                }

                                if (sb_ready_send_pre && !cmd_exist) {
                                    funcState[rank].wakeup = true;
                                    if (RankState[rank].lp_state == IDLE && even_cycle) {     //every other command, even;
                                        Cmd *c = new Cmd;
                                        c->state = working;
                                        if (IS_LP4 || IS_LP5 || IS_LP6 || IS_HBM2E || IS_HBM3) {
                                            c->cmd_type = PRECHARGE_PB_CMD;
                                            c->bank = bankStates[bank_idx].bank;
                                            c->rank = bankStates[bank_idx].rank;
                                            c->group = bankStates[bank_idx].group;
                                            c->bankIndex = bankStates[bank_idx].bankIndex;
                                            bankStates[bank_idx].hold_refresh_pb = true;
                                            bankStates[bank_idx].finish_refresh_pb = false;
                                            pbr_hold_pre_time[rank] = now() + 4;           //todo: revise for e-mode
                                        } else if (IS_DDR5) {
                                            c->cmd_type = PRECHARGE_SB_CMD;
                                            c->bank = bankStates[bank_tmp].bank;
                                            c->rank = bankStates[bank_tmp].rank;
                                            c->group = bankStates[bank_tmp].group;
                                            c->bankIndex = bankStates[bank_tmp].bankIndex;
                                            for (size_t sbr_bank_i = 0; sbr_bank_i < pbr_bg_num; sbr_bank_i ++) {
                                                unsigned bank_idx_i = bank_tmp + sbr_bank_i * pbr_bank_num;
                                                bankStates[bank_idx_i].hold_refresh_pb = true;
                                                bankStates[bank_idx_i].finish_refresh_pb = false;
                                            }
                                        }
                                        c->force_pbr = true;
                                        c->cmd_source = 2;
                                        c->pri = 0;
                                        c->task = 0xFFFFFFFFFFFFFFD;
                                        //CmdQueue.push_back(c);
                                        pre_cmdqueue.push_back(c);
                                        if (DEBUG_BUS) {
                                            PRINTN(setw(10)<<now()<<" -- REQ :: force precharge, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                                        }
                                        return;
                                    } else {
                                        if (DEBUG_BUS) {
                                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold PRECHARGE PB/SB in lp"
                                                    <<" state, rank"<<rank<<", sc="<<sc<<", bank="<<bank_tmp<<endl);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void PTC::send_pb_refresh(unsigned rank, unsigned sc) {
    
    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    unsigned bank_pair_start = sc * pbr_bank_num; 
    // find Idle bank send pb refresh
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {    //todo: revise for e-mode
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (force_pbr_refresh[rank][sc]) continue;
        if (refresh_pbr_has_finish[rank][sc]) continue;
        if (bankStates[bank_tmp].en_refresh_pb && bankStates[bank_tmp].hold_refresh_pb && PbrWeight[rank][bank + bank_pair_start].send_pbr) {
            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                bool diff_bg_idle = true;
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                    if (bankStates[bank_idx].state->currentBankState != Idle || issue_state[bank_idx]) {
                        diff_bg_idle = false;
                    }
                }

                bool ptc_cmd_exist = false;
                bool rw_switch_trig = ((PreCmd.trans_type == DATA_READ) && que_read_cnt < PBR_LESS_CMD_LEVEL && que_write_cnt>0)
                                        || ((PreCmd.trans_type == DATA_WRITE) && que_write_cnt < PBR_LESS_CMD_LEVEL && que_read_cnt>0);
                bool force_sch_cmd = false; 
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                    if (PBR_LESS_CMD_EN && PBR_LESS_CMD_MODE==0) {
                        for (size_t i = 0; i < transactionQueue.size(); i++) {
                            if (transactionQueue[i]->bankIndex!=bank_idx) continue;
                            if (transactionQueue[i]->transactionType!=PreCmd.trans_type) continue;
                            force_sch_cmd = true;
                        }
                    }
                    if (bank_cnt[bank_idx] > 0 && PBR_LESS_CMD_EN && PBR_LESS_CMD_MODE==0 && rw_switch_trig && force_sch_cmd) {
                        ptc_cmd_exist = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- Idle Pbr Hold :: Cmd Exist, rank="<<rank<<", bankIndex="<<bank_idx<<", sc="<<sc<<endl);
                        }
                        break;
                    } else if (bank_cnt[bank_idx] > 0 && PBR_LESS_CMD_EN && PBR_LESS_CMD_MODE==1 && rw_switch_trig && force_sch_cmd) {
                        ptc_cmd_exist = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- Idle Pbr Hold :: Cmd Exist, rank="<<rank<<", bankIndex="<<bank_idx<<", sc="<<sc<<endl);
                        }
                        break;
                    }
                }

                if (arb_enable && ((now() + 1) >= bankStates[bank_tmp].state->nextPerBankRefresh)
                        && ((tFAWCountdown[rank].size() < 4 && sc==0) ||(tFAWCountdown_sc1[rank].size() < 4 && sc==1)) 
                        && diff_bg_idle && !ptc_cmd_exist) {    //todo: revise for e-mode
                    funcState[rank].wakeup = true;
                    if (RankState[rank].lp_state == IDLE && even_cycle && (!IS_HBM3 || now()%2==1 || ODD_TDM)) {    //every other command, even;
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = PER_BANK_REFRESH_CMD;
                        c->force_pbr = true;
                        c->bank = bankStates[bank_tmp].bank;
                        c->rank = bankStates[bank_tmp].rank;
                        c->group = bankStates[bank_tmp].group;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        c->bankIndex = bankStates[bank_tmp].bankIndex;
                        CmdQueue.push_back(c);
                        for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                            unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                            bankStates[bank_idx].hold_refresh_pb = false;
                        }
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ :: PBR IDLE, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                        }
                        return;
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold FPBR in lp, rank="<<rank<<", sc="<<sc<<", bank="<<bank_tmp<<endl);
                        }
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: rank"<<rank<<", bank="<<bank_tmp<<", arb_enable="<<arb_enable<<", sc="<<sc<<endl);
                    }
                    return;
                }
            }
        }
    }
}

void PTC::enh_per_bank_refresh(unsigned rank, unsigned sc) {
    if (refreshALL[rank][sc].refresh_cnt == 0 || refreshALL[rank][sc].refreshWaiting ||
            RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREF ||
            RankState[rank].lp_state == ASREFX || RankState[rank].lp_state == SRPDE ||
            RankState[rank].lp_state == SRPD || RankState[rank].lp_state == SRPDLP ||
            RankState[rank].lp_state == SRPDX) return;
    if (now() < sbr_gap_cnt[rank]) return;       //todo: revise for e-mode

    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    bool have_bank_refresh = false; // send pbr one by one if set true
    for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (IS_HBM2E || IS_HBM3 || PBR_PARA_EN) {
            if (!refreshPerBank[bank_tmp].refreshWaiting) continue;
        } else {
            if (!refreshPerBank[bank_tmp].refreshWaiting && !refreshPerBank[bank_tmp].refreshing) continue;
        }
        have_bank_refresh = true;
        break;
    }

    if (!have_bank_refresh) {

        for (size_t i = 0; i < pbr_sb_group_num; i++) {   //todo: revise for e-mode
            SbGroupWeight[rank][i].bank_pair_cnt = 0;
// todo: revise            SbGroupWeight[rank][i].send_pbr = false;
        }

        for (size_t i = 0; i < NUM_BANKS; i++) {   //todo: revise for e-mode 
            PbrWeight[rank][i].bank_pair_cnt = 0;
            PbrWeight[rank][i].send_pbr = false;
        }
        
        vector<pbr_weight> SbWeightOrder;
        // calculate weight for diffrent groups, in each of which banks have same ba1ba0;
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++) {
            unsigned has_bank_open = 0;
            unsigned has_cmd_vld = 0;
            unsigned bank_idle = pbr_sb_num;
            unsigned bank_cmd_cnt = 0;
            for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bank  + sb_bank * pbr_sb_num;    //todo: revise for e-mode 
                if (bankStates[bank_tmp].state->currentBankState == RowActive) {
                    bank_idle --;
                    if (bank_idle < 2) {
                        has_bank_open = 1;
                        break;
                    }
                }
            }
            for (size_t sbr_bank = 0; sbr_bank < pbr_sb_num; sbr_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bank + sbr_bank * pbr_sb_num;    //todo: revise for e-mode
                if (bank_cnt[bank_tmp] > 0) {
                    bank_cmd_cnt ++;
                    if (bank_cmd_cnt >2) {
                        has_cmd_vld = 1;
                        break;
                    }
                }
            }

            if (SBR_WEIGHT_MODE == 0) SbGroupWeight[rank][bank].bank_pair_cnt = has_bank_open;
            else if (SBR_WEIGHT_MODE == 1) SbGroupWeight[rank][bank].bank_pair_cnt = has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 2) SbGroupWeight[rank][bank].bank_pair_cnt = has_bank_open + has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 3) SbGroupWeight[rank][bank].bank_pair_cnt = has_bank_open + has_cmd_vld * 2;
            else { // added for SBR_WEIGHT_MODE=4/5, select pbr ba group with least/most cmd
                for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++) {
                    unsigned bankIndex = rank * NUM_BANKS + bank + sb_bank * pbr_sb_num; 
                    SbGroupWeight[rank][bank].bank_pair_cnt += bank_cnt[bankIndex];
                }
            };

            SbGroupWeight[rank][bank].bagroup = bank;

            //reorder the priority of 4 groups: ba1ba0: (00,01,10,11)
            unsigned pre_bank_pair_cnt = 0;
            
            // ba1ba0:00
            if (bank == 0) {
                SbWeightOrder.push_back(SbGroupWeight[rank][bank]);
            }

            //reorder from small to large
            for (size_t bank_pre = 0; bank_pre < bank; bank_pre ++) {
                if ((SbGroupWeight[rank][bank].bank_pair_cnt < SbWeightOrder[bank_pre].bank_pair_cnt) && 
                    (SbGroupWeight[rank][bank].bank_pair_cnt) >= pre_bank_pair_cnt) {
                    SbWeightOrder.insert(SbWeightOrder.begin()+bank_pre, SbGroupWeight[rank][bank]);
                    break;
                }
                if (bank == (bank_pre + 1)) {
                    SbWeightOrder.push_back(SbGroupWeight[rank][bank]);
                }
                pre_bank_pair_cnt = SbWeightOrder[bank_pre].bank_pair_cnt; 
            }

            if (SBR_WEIGHT_MODE == 5) {     // reorder from large to small
                reverse(SbWeightOrder.begin(), SbWeightOrder.end());
            }

        }

        // calculate weight for bank pairs with same ba1ba0
        // 4 different groups: [0,4,8,12], [1,5,9,13], [2,6,10,14], [3,7,11,15]
        // order of SbWeight : (0,4),(0,8),(0,12),(4,8),(4,12),(8,12),(1,5),(1,9)......
        unsigned sbweight_idx = 0;
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++) {
            for (size_t sb_bank = 0 ; sb_bank < pbr_sb_num; sb_bank ++) {
                for (size_t sb_bank_tmp = (sb_bank + 1) ; sb_bank_tmp < pbr_sb_num; sb_bank_tmp ++) {
                    unsigned has_bank_open_sb = 0;
                    unsigned has_cmd_vld_sb = 0;
                    unsigned fst_bank_tmp = rank * NUM_BANKS + bank + sb_bank * pbr_sb_num;          // one bank of a bank pair 
                    unsigned lst_bank_tmp = rank * NUM_BANKS + bank + sb_bank_tmp * pbr_sb_num;      // another bank of a bank pair
                    if (bankStates[fst_bank_tmp].state->currentBankState == RowActive || bankStates[lst_bank_tmp].state->currentBankState == RowActive) {
                        has_bank_open_sb = 1;
                    }
                    if (bank_cnt[fst_bank_tmp] > 0 || bank_cnt[lst_bank_tmp] > 0 ) {
                        has_cmd_vld_sb = 1;
                    }
                    if (SBR_WEIGHT_MODE == 0) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_bank_open_sb;
                    else if (SBR_WEIGHT_MODE == 1) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_cmd_vld_sb;
                    else if (SBR_WEIGHT_MODE == 2) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_bank_open_sb + has_cmd_vld_sb;
                    else if (SBR_WEIGHT_MODE == 3) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_bank_open_sb + has_cmd_vld_sb * 2;
                    else {  // added for SBR_WEIGHT_MODE=4/5, select bank pair with least/most cmd
                        SbWeight[rank][sbweight_idx].bank_pair_cnt = bank_cnt[fst_bank_tmp] + bank_cnt[lst_bank_tmp];  
                    }
                    
                    SbWeight[rank][sbweight_idx].fst_bankIndex = fst_bank_tmp; 
                    SbWeight[rank][sbweight_idx].lst_bankIndex = lst_bank_tmp; 
                    SbWeight[rank][sbweight_idx].bagroup = bank;

                    // added for SBR_WEIGHT_ENH_MODE=3/4, select bank pair with least/most cmd
                    // based on SBR_WEIGHT_MODE
                    if (SBR_WEIGHT_ENH_MODE == 3 || SBR_WEIGHT_ENH_MODE == 4) {
                        bank_pair_cmd_cnt[rank][sbweight_idx] = bank_cnt[fst_bank_tmp] + bank_cnt[lst_bank_tmp];
                    }

                    sbweight_idx ++;

//                    DEBUG(now()<<" sbweight, bank pair="<<sbweight_idx<<" fst_bankIndex="<<fst_bank_tmp<<" lst_bankIndex="<<lst_bank_tmp);
                }
            }
        }
//        DEBUG(now()<<" arb, SbWeight_size="<<SbWeight[rank].size()<<" rank="<<rank);

    if (refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {
        for (size_t i = 0; i < NUM_BANKS; i++) {
            unsigned bank = rank * NUM_BANKS + i;
            if (bank_cnt[bank] > 0) {
                for (size_t j=0; j < SbWeight[rank].size(); j++) {
                    if ((i==SbWeight[rank][j].fst_bankIndex)||(i==SbWeight[rank][j].lst_bankIndex))
                    SbWeight[rank][j].bank_pair_cnt = 0xFFFFFFFF;
                }
            }
        }
    }

        bool pbr_bank_left = true;
        unsigned pbr_bagroup_cnt = 0;
        if (SBR_WEIGHT_ENH_MODE==1){
            for (size_t group = 0; group < pbr_sb_group_num; group++){
                if (group == pre_enh_pbr_bagroup[rank]) {
                    for (size_t i = 0; i < pbr_sb_num; i ++){
                        unsigned bank = NUM_BANKS * rank + group + i * pbr_sb_num; 
                        if (bankStates[bank].finish_refresh_pb){
                            pbr_bagroup_cnt ++;
                            if (pbr_bagroup_cnt >= (pbr_sb_num-1)) {
                                pbr_bank_left = false;
                                break;
                            }
                        }
                    }    
                } 
            }  
        }

        unsigned bank_pair_cnt_min = 0xFFFFFFFF;
        unsigned bank_pair_cnt_min_num_fst = 0xFFFFFFFF;
        unsigned bank_pair_cnt_min_num_lst = 0xFFFFFFFF;
        unsigned bagroup_cnt_min = 0xFFFFFFFF;
        unsigned bank_pair_cmd_cnt_min_num = 0xFFFFFFFF;
        bool     pre_sch_bankIndex_met = false;
        unsigned pre_sch_bank_pair_cnt = 0xFFFFFFFF;
        unsigned pre_sch_bank_pair = 0xFFFFFFFF;
        if (ENH_PBR_CEIL_EN) {  // 1 in 24, the best for enhanced pbr 
            for (size_t group = 0;  group < pbr_sb_group_num; group++) {
                for (size_t bank = 0; bank < SbWeight[rank].size(); bank++) {
                    unsigned fst_bank = SbWeight[rank][bank].fst_bankIndex;
                    unsigned lst_bank = SbWeight[rank][bank].lst_bankIndex;
                    unsigned bagroup_cnt = SbWeight[rank][bank].bagroup;
                    if (group == bagroup_cnt) {
                        if (bank_pair_cnt_min > SbWeight[rank][bank].bank_pair_cnt &&
                             (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                             (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF)) {
                            bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                            bank_pair_cnt_min_num_fst = fst_bank;
                            bank_pair_cnt_min_num_lst = lst_bank;
                            bagroup_cnt_min = bagroup_cnt;
                        }
                        // added for SBR_WEIGHT_ENH_MODE=2
                        if ((fst_bank==pre_sch_bankIndex[rank]||lst_bank==pre_sch_bankIndex[rank])&&(SBR_WEIGHT_ENH_MODE==2) && 
                                (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                                (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF)) {
                            pre_sch_bankIndex_met = true;
                            pre_sch_bank_pair_cnt = SbWeight[rank][bank].bank_pair_cnt; 
                            pre_sch_bank_pair     = bank; 
                        }
                    }
                }
            }
        } else {      //real for RTL: 4 in 1 && 6 in 1
            for (size_t group = 0;  group < pbr_sb_group_num; group++) {     // 4 in 1
                unsigned bank_pair_group = SbWeightOrder[group].bagroup;
                if (bagroup_cnt_min != 0xFFFFFFFF && bagroup_cnt_min != bank_pair_group) break;   // standard 4in1 && 6in1
                if (pbr_bank_left && bank_pair_group != pre_enh_pbr_bagroup[rank] 
                        && pre_enh_pbr_bagroup[rank] != 0xFFFFFFFF && SBR_WEIGHT_ENH_MODE==1) continue;     // same bank pair group priority, added for SBR_WEIGHT_ENH_MODE=1
                for (size_t bank = 0; bank < SbWeight[rank].size(); bank++) {
                    unsigned fst_bank = SbWeight[rank][bank].fst_bankIndex;
                    unsigned lst_bank = SbWeight[rank][bank].lst_bankIndex;
                    unsigned bagroup_cnt = SbWeight[rank][bank].bagroup;
                    if (bank_pair_group == bagroup_cnt) {    // 6 in 1
                        if (bank_pair_cnt_min > SbWeight[rank][bank].bank_pair_cnt &&
                             (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                             (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF) &&
                             (SBR_WEIGHT_MODE!=5)) {
                            bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                            bank_pair_cnt_min_num_fst = fst_bank;
                            bank_pair_cnt_min_num_lst = lst_bank;
                            bagroup_cnt_min = bagroup_cnt;
                            bank_pair_cmd_cnt_min_num = bank;
                        }

                        //added for SBR_WEIGHT_MODE=5, select bank pair with most cmd
                        if ((bank_pair_cnt_min < SbWeight[rank][bank].bank_pair_cnt || bank_pair_cnt_min == 0xFFFFFFFF) &&
                             (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                             (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF) &&
                             (SBR_WEIGHT_MODE==5)) {
                            bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                            bank_pair_cnt_min_num_fst = fst_bank;
                            bank_pair_cnt_min_num_lst = lst_bank;
                            bagroup_cnt_min = bagroup_cnt;
                            bank_pair_cmd_cnt_min_num = bank;
                        }

                        // added for SBR_WEIGHT_ENH_MODE=2
                        if ((fst_bank==pre_sch_bankIndex[rank]||lst_bank==pre_sch_bankIndex[rank])&&(SBR_WEIGHT_ENH_MODE==2) && 
                                (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                                (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF)) {
                            pre_sch_bankIndex_met = true;
                            pre_sch_bank_pair_cnt = SbWeight[rank][bank].bank_pair_cnt; 
                            pre_sch_bank_pair     = bank; 
                        }
                    }
                }
            }
        }
        
        //added for SBR_WEIGHT_ENH_MODE=2, most recently scheduled cmd priority
        if (SBR_WEIGHT_ENH_MODE==2 && pre_sch_bankIndex_met &&
                pre_sch_bank_pair_cnt!=0xFFFFFFFF && pre_sch_bank_pair!=0xFFFFFFFF){
            if (bank_pair_cnt_min >= pre_sch_bank_pair_cnt) {
                bank_pair_cnt_min = SbWeight[rank][pre_sch_bank_pair].bank_pair_cnt;
                bank_pair_cnt_min_num_fst = SbWeight[rank][pre_sch_bank_pair].fst_bankIndex;
                bank_pair_cnt_min_num_lst = SbWeight[rank][pre_sch_bank_pair].lst_bankIndex;
                bagroup_cnt_min = SbWeight[rank][pre_sch_bank_pair].bagroup ;
            }       
        }

        //added for SBR_WEIGHT_ENH_MODE=3/4, select bank pair with least/most cmd
        for (size_t bank = 0; bank < SbWeight[rank].size(); bank++) {
            unsigned fst_bank = SbWeight[rank][bank].fst_bankIndex;
            unsigned lst_bank = SbWeight[rank][bank].lst_bankIndex;
            unsigned bagroup_cnt = SbWeight[rank][bank].bagroup;
            if (bank_pair_cmd_cnt_min_num != 0xFFFFFFFF) {
                if (bagroup_cnt_min == bagroup_cnt && bagroup_cnt_min!=0xFFFFFFFF && bank_pair_cnt_min == SbWeight[rank][bank].bank_pair_cnt && 
                        (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb)){
                    if ((bank_pair_cmd_cnt[rank][bank] < bank_pair_cmd_cnt[rank][bank_pair_cmd_cnt_min_num] && SBR_WEIGHT_ENH_MODE==3) || 
                            (bank_pair_cmd_cnt[rank][bank] > bank_pair_cmd_cnt[rank][bank_pair_cmd_cnt_min_num] && SBR_WEIGHT_ENH_MODE==4)) {
                        bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                        bank_pair_cnt_min_num_fst = fst_bank;
                        bank_pair_cnt_min_num_lst = lst_bank;
                        bagroup_cnt_min = bagroup_cnt;
                        bank_pair_cmd_cnt_min_num = bank;
                    }
                }
            }
        }

        if ((bank_pair_cnt_min_num_fst != 0xFFFFFFFF) && (bank_pair_cnt_min_num_lst != 0xFFFFFFFF) && (bagroup_cnt_min != 0xFFFFFFFF) ) {
//            DEBUG(now()<<" min_fst_bankindex="<<bank_pair_cnt_min_num_fst<<" min_lst_bankindex="<<bank_pair_cnt_min_num_lst<<" min_bagroup="<<bagroup_cnt_min);
            unsigned min_fst_bank = bank_pair_cnt_min_num_fst % NUM_BANKS;
            unsigned min_lst_bank = bank_pair_cnt_min_num_lst % NUM_BANKS;
            PbrWeight[rank][min_fst_bank].send_pbr = true;
            PbrWeight[rank][min_lst_bank].send_pbr = true;
            PbrWeight[rank][min_fst_bank].bagroup = bagroup_cnt_min;
            PbrWeight[rank][min_lst_bank].bagroup = bagroup_cnt_min;
            PbrWeight[rank][min_fst_bank].fst_bankIndex = bank_pair_cnt_min_num_fst;
            PbrWeight[rank][min_fst_bank].lst_bankIndex = bank_pair_cnt_min_num_lst;
            PbrWeight[rank][min_lst_bank].fst_bankIndex = bank_pair_cnt_min_num_lst;
            PbrWeight[rank][min_lst_bank].lst_bankIndex = bank_pair_cnt_min_num_fst;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ENHANCED PBR_WEIGHT :: rank="<<rank<<" fst_bank="
                        <<bank_pair_cnt_min_num_fst<<" lst_bank="
                        <<bank_pair_cnt_min_num_lst<<" ba group="<<bagroup_cnt_min<<" win the arbitration!"<<endl);
            }
            // bankIndex check
            if (bank_pair_cnt_min_num_fst == bank_pair_cnt_min_num_lst) {
                ERROR(setw(10)<<now()<<" Not allowed same bank in a bank pair, bagroup="<<bagroup_cnt_min
                        <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                assert(0);
            } else if (bank_pair_cnt_min_num_fst > bank_pair_cnt_min_num_lst) {
                if (((bank_pair_cnt_min_num_fst-bank_pair_cnt_min_num_lst)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bagroup="<<bagroup_cnt_min
                            <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                    assert(0);
                }
            } else if (bank_pair_cnt_min_num_lst > bank_pair_cnt_min_num_fst) {
                if (((bank_pair_cnt_min_num_lst-bank_pair_cnt_min_num_fst)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bgroup="<<bagroup_cnt_min
                            <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                    assert(0);
                }
            }
            //bagroup and bankIndex combined check
            if (((bank_pair_cnt_min_num_lst % pbr_sb_num)!=bagroup_cnt_min) ||  ((bank_pair_cnt_min_num_fst % pbr_sb_num!=bagroup_cnt_min))) {
                ERROR(setw(10)<<now()<<" Wrong bagroup, bgroup="<<bagroup_cnt_min
                        <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                assert(0);
            }
        }

    }

    refresh_cnt_pb[rank][sc] = 0;
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        if (bankStates[bank_tmp].finish_refresh_pb) refresh_cnt_pb[rank][sc] ++;    //2X refresh commands
        else forceRankBankIndex[rank][sc] = bank;

        if (SBR_REQ_MODE == 0) {
            if (bank == NUM_BANKS - 1) {
                if (refresh_cnt_pb[rank][sc] == NUM_BANKS) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    pre_enh_pbr_bagroup[rank] = 0xFFFFFFFF;
                    for (size_t bank_sbr = 0; bank_sbr < NUM_BANKS; bank_sbr ++) {
                        unsigned bank_idx = rank * NUM_BANKS + bank_sbr;
                        bankStates[bank_idx].finish_refresh_pb = false;
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: RANK "<<rank<<" has been refreshed by pbr"<<endl);
                    }
                } else {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: Force pbr rank "<<rank<<" has no idle bank not refreshed"<<endl);
                    }
                }
            }
        } else {
            if (bank == NUM_BANKS - 1) {
                if (refresh_cnt_pb[rank][sc] == NUM_BANKS) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    pre_enh_pbr_bagroup[rank] = 0xFFFFFFFF;
                    for (size_t bank_sbr = 0; bank_sbr < NUM_BANKS; bank_sbr ++) {
                        unsigned bank_idx = rank * NUM_BANKS + bank_sbr;
                        bankStates[bank_idx].finish_refresh_pb = false;
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: RANK "<<rank<<" has been refreshed by pbr"<<endl);
                    }
                } else if ((refresh_cnt_pb[rank][sc]) >= (2*SBR_FRCST_NUM)) {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: Force pbr for the left banks in RANK "<<rank<<endl);
                    }
                }
            }
        }
    }

    if ((refresh_cnt_pb[rank][sc] % 2)!=0 || (refresh_cnt_pb[rank][sc] > 16)){
        ERROR(setw(10)<<now()<<" Pbr Cnt Not Even or Exceed 16, refresh cnt(x2)="<<refresh_cnt_pb[rank][sc]);
        assert(0);
        
    }

    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        if (!bankStates[bank_tmp].finish_refresh_pb) {
            bankStates[bank_tmp].en_refresh_pb = true;
            bankStates[bank_tmp].hold_refresh_pb = true;
        }
    }

    enh_send_pb_precharge(rank, sc);
    enh_send_pb_refresh(rank, sc);
}

void PTC::enh_send_pb_precharge(unsigned rank, unsigned sc) {
    // unrefreshed bank is 2 left, force pbr
    bool have_bank_refresh = false; // send pbr one by one if set true
    //if (!IS_HBM2E && !IS_HBM3 ) {
    if (!IS_HBM2E && !IS_HBM3 && !PBR_PARA_EN) {
        for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank;
            if (refreshPerBank[bank_tmp].refreshing){
                have_bank_refresh = true;
                break;
            }
        }
    }
    
    vector<unsigned> bp_idx;
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        unsigned fst_bank_idx = PbrWeight[rank][bank].fst_bankIndex; 
        unsigned lst_bank_idx = PbrWeight[rank][bank].lst_bankIndex;
//        unsigned bagroup = PbrWeight[rank][bank].bagroup; 

        if (refresh_pbr_has_finish[rank][sc]) continue;
        
        // bp_idx size check
        if (bp_idx.size()!=0) {
            ERROR(setw(10)<<now()<<" after clear, Vector bp_idx size check failed, size="<<bp_idx.size()<<" bank="<<bank_tmp
                    <<" fst_bankIndex="<<fst_bank_idx<<" lst_bankIndex="<<lst_bank_idx);
            assert(0);
        }
        
        bp_idx.push_back(fst_bank_idx);
        bp_idx.push_back(lst_bank_idx);
        
        // bp_idx size check
        if (bp_idx.size()!=2) {
            for(size_t i = 0; i<bp_idx.size();i++){
                DEBUG(now()<<" bankIndex="<<bp_idx[i]);
            }
            ERROR(setw(10)<<now()<<" Vector bp_idx size check failed, size="<<bp_idx.size()<<" rank="<<rank
                    <<" bank="<<bank_tmp<<" fst_bankIndex="<<fst_bank_idx<<" lst_bankIndex="<<lst_bank_idx);
            assert(0);
        }

//        if (refresh_pbr_has_finish[rank]) continue;
        if (force_pbr_refresh[rank][sc] && !bankStates[bank_tmp].finish_refresh_pb
                && !have_bank_refresh && PbrWeight[rank][bank].send_pbr) {

            // bankIndex check
            if (fst_bank_idx == lst_bank_idx) {
                ERROR(setw(10)<<now()<<" Not allowed same bank in a bank pair, bank="<<bank_tmp
                        <<" fst_bankIndex"<<fst_bank_idx<<" lst_bankIndex"<<lst_bank_idx);
                assert(0);
            } else if (fst_bank_idx > lst_bank_idx) {
                if (((fst_bank_idx-lst_bank_idx)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bank="<<bank_tmp
                            <<" fst_bankIndex"<<fst_bank_idx<<" lst_bankIndex"<<lst_bank_idx);
                    assert(0);
                }
            } else if (lst_bank_idx > fst_bank_idx) {
                if (((lst_bank_idx-fst_bank_idx)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bank="<<bank_tmp
                            <<" fst_bankIndex"<<fst_bank_idx<<" lst_bankIndex"<<lst_bank_idx);
                    assert(0);
                }
            }

            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                if (arb_enable) {
                    for(size_t i = 0; i<bp_idx.size(); i++) {
                        unsigned bank_idx = bp_idx[i];
                        if (!refreshPerBank[bank_idx].refreshWaiting) {
                            refreshPerBank[bank_idx].refreshWaiting = true;
                            refreshPerBank[bank_idx].refreshing = false;
                            refreshPerBank[bank_idx].refreshWaitingPre = false;
                            pbr_hold_pre[rank] = true;
                            pbr_hold_pre_time[rank] = now() + 6;
                        }
                    }

                    bool ready_send_pbr = true;
                    for(size_t i = 0; i<bp_idx.size(); i++) {
                        unsigned bank_idx = bp_idx[i];
                        if ((bankStates[bank_idx].state->currentBankState != Idle || (now() + 1) < bankStates[bank_idx].state->nextPerBankRefresh) || issue_state[bank_idx]) { 
                            ready_send_pbr = false;
                            break;
                        }
                    }
                    
                    if (ready_send_pbr && ((tFAWCountdown[rank].size() < 4 && sc==0) 
                            ||(tFAWCountdown_sc1[rank].size() < 4 && sc==1)) 
                            && (!ASREF_ENABLE || (ASREF_ENABLE && RankState[rank].asref_cnt != 0))) {
                        funcState[rank].wakeup = true;
//                        if (RankState[rank].lp_state == IDLE) {
                        if (RankState[rank].lp_state == IDLE && even_cycle && (!IS_HBM3 || now()%2==1 || ODD_TDM)) {    // every other command, even;
                            
                            //todo: info of fst_bank/lst_bank may be transfered to Cmd -> changed already
                            Cmd *c = new Cmd;
                            c->state = working;
                            c->cmd_type = PER_BANK_REFRESH_CMD;
                            c->force_pbr = true;
                            c->bank = bankStates[bank_tmp].bank;
                            c->rank = bankStates[bank_tmp].rank;
                            c->group = bankStates[bank_tmp].group;
                            c->cmd_source = 2;
                            c->task = 0xFFFFFFFFFFFFFFB;
                            c->bankIndex = bankStates[bank_tmp].bankIndex;
                            c->fst_bankIndex = fst_bank_idx;
                            c->lst_bankIndex = lst_bank_idx;


                            bankStates[fst_bank_idx].hold_refresh_pb = false;
                            bankStates[lst_bank_idx].hold_refresh_pb = false;
                            
                            CmdQueue.push_back(c);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ :: force enhanced pbr, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank="<<lst_bank_idx<<endl);
                            }
                            return;
                        } else {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold enhanced FPBR in lp, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank="<<lst_bank_idx<<endl);
                            }
                        }
                    } else if (tFPWCountdown[rank].size() < 4) {    //stop here 0717
                        bool sb_ready_send_pre = true;
                        for (size_t j = 0; j < bp_idx.size(); j ++) {
                            unsigned bank_index = bp_idx[j]; 
                            if (bankStates[bank_index].state->currentBankState == RowActive) { 
                                refreshPerBank[bank_index].refreshWaiting = true;
                                refreshPerBank[bank_index].refreshing = false;
                                refreshPerBank[bank_index].refreshWaitingPre = false;
                                pbr_hold_pre[rank] = true;
                            }
                        }

                        if (((now() + 1) < bankStates[bank_tmp].state->nextPrecharge || issue_state[bank_tmp]) && bankStates[bank_tmp].state->currentBankState == RowActive) { 
                            sb_ready_send_pre = false;
                    
                        }

                        if (sb_ready_send_pre && (bankStates[bank_tmp].state->currentBankState == RowActive)) {
                            funcState[rank].wakeup = true;
//                                    if (RankState[rank].lp_state == IDLE) {
                            if (RankState[rank].lp_state == IDLE && even_cycle) {     //every other command, evan;
                                Cmd *c = new Cmd;
                                c->state = working;
                                if (IS_LP6) {
                                    //todo: fst_bank/lst_bank info may be transfered? -> changed already
                                    c->cmd_type = PRECHARGE_PB_CMD;
                                    c->bank = bankStates[bank_tmp].bank;
                                    c->rank = bankStates[bank_tmp].rank;
                                    c->group = bankStates[bank_tmp].group;
                                    c->bankIndex = bankStates[bank_tmp].bankIndex;
                                    c->fst_bankIndex = fst_bank_idx;
                                    c->lst_bankIndex = lst_bank_idx;
                                    bankStates[bank_tmp].hold_refresh_pb = true;
                                    bankStates[bank_tmp].finish_refresh_pb = false;
                                    pbr_hold_pre_time[rank] = now() + 4;
                                } else {
                                    ERROR(setw(10)<<now()<<" Ehanced DBR Only Applies to LP6 ");
                                    assert(0);
                                }
                                c->force_pbr = true;
                                c->cmd_source = 2;
                                c->pri = 0;
                                c->task = 0xFFFFFFFFFFFFFFD;
                                //CmdQueue.push_back(c);
                                pre_cmdqueue.push_back(c);
                                if (DEBUG_BUS) {
                                    PRINTN(setw(10)<<now()<<" -- REQ ::  force ENHANCED PRECHARGE, rank="<<rank<<", bank="<<bank_tmp<<endl);
                                }
                                return;
                            } else {
                                if (DEBUG_BUS) {
                                    PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold ENAHNCED PRECHARGE PB/SB in lp"<<" state, rank"<<rank<<", bank="<<bank_tmp<<endl);
                                }
                            }
                        }
                    }
                }
            }
        }
        bp_idx.clear();
        
    }
}

void PTC::enh_send_pb_refresh(unsigned rank, unsigned sc) {
    // find Idle bank send pb refresh
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        unsigned fst_bank_idx = PbrWeight[rank][bank].fst_bankIndex; 
        unsigned lst_bank_idx = PbrWeight[rank][bank].lst_bankIndex;
        
        if (force_pbr_refresh[rank][sc]) continue;
        if (refresh_pbr_has_finish[rank][sc]) continue;
        if (bankStates[bank_tmp].en_refresh_pb && bankStates[bank_tmp].hold_refresh_pb && PbrWeight[rank][bank].send_pbr) {
            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                bool diff_bg_idle = true;
                if (bankStates[fst_bank_idx].state->currentBankState != Idle || 
                        bankStates[lst_bank_idx].state->currentBankState != Idle || issue_state[fst_bank_idx] || issue_state[lst_bank_idx]) {
                    diff_bg_idle = false;
                }
                
                if (arb_enable && ((now() + 1) >= bankStates[fst_bank_idx].state->nextPerBankRefresh)
                        && ((now() + 1) >= bankStates[lst_bank_idx].state->nextPerBankRefresh) 
                        && ((tFAWCountdown[rank].size() < 4 && sc==0) 
                        ||(tFAWCountdown_sc1[rank].size() < 4 && sc==1)) && diff_bg_idle) {
                    funcState[rank].wakeup = true;
//                    if (RankState[rank].lp_state == IDLE) {
                    if (RankState[rank].lp_state == IDLE && even_cycle && (!IS_HBM3 || now()%2==1 || ODD_TDM)) {    //every other command, even;
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = PER_BANK_REFRESH_CMD;
                        c->force_pbr = true;
                        c->bank = bankStates[bank_tmp].bank;
                        c->rank = bankStates[bank_tmp].rank;
                        c->group = bankStates[bank_tmp].group;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        c->bankIndex = bankStates[bank_tmp].bankIndex;
                        c->fst_bankIndex = fst_bank_idx;
                        c->lst_bankIndex = lst_bank_idx;
                        CmdQueue.push_back(c);
                        
                        bankStates[fst_bank_idx].hold_refresh_pb = false;
                        bankStates[lst_bank_idx].hold_refresh_pb = false;
                        
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ :: ENHANCED PBR IDLE, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank"<<lst_bank_idx<<endl);
                        }
                        return;
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold ENHANCED FPBR in lp, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank"<<lst_bank_idx<<endl);
                        }
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: rank"<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank="<<lst_bank_idx<<", arb_enable="<<arb_enable<<endl);
                    }
                    return;
                }
            }
        }
    }
}

void PTC::update_arb_group_pri(Cmd *c, vector<unsigned> &arb_group_pri, unsigned arb_type) {
    unsigned cmd_index = c->arb_group;
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- ARB_CMD :: Arb_Group="<<c->arb_group<<" task="<<c->task
                <<" cmd_type="<<c->cmd_type<<" QoS="<<c->pri<<" rank="<<c->rank<<" sid="<<c->sid
                <<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<endl);
    }
    
    
    // update arb group priority, (ARB_GROUP_NUM-1) is the highest
    for (size_t i=0; i<ARB_GROUP_NUM; i++) {
        if(cmd_index==i) {
            arb_group_pri[i] = 0;
        }
    }

    //ROUND ROBIN
    for (size_t j=0; j<ARB_GROUP_NUM; j++) {
        if (cmd_index == j) continue;
        if (j > cmd_index) {
            arb_group_pri[j] = ARB_GROUP_NUM - (j - cmd_index);
        } else {
            arb_group_pri[j] = cmd_index - j;
        }
        if (arb_group_pri[j] > (ARB_GROUP_NUM-1)) {
            ERROR(setw(10)<<now()<<" Wrong Arb_Group_Pri, Arb_group="<<j<<", cnt="<<arb_group_cnt[j]);
            assert(0);
        }
    }

    if (DEBUG_BUS) {
        for (size_t arb_grp_idx = 0; arb_grp_idx < ARB_GROUP_NUM; arb_grp_idx++) {
            PRINTN(setw(10)<<now()<<" -- Updated Arb_Group_Pri="<<arb_group_pri[arb_grp_idx]<<" Arb_Group="<<arb_grp_idx<<" Arb_Type="<<arb_type<<endl);
        }
    }
}

void PTC::faw_update() {
    for (size_t i = 0; i < NUM_RANKS; i++) {
        if (EM_ENABLE) {
            //sub channel0
            //tfaw decrement all the counters we have going: sub channel0
            for (size_t j = 0; j < tFAWCountdown[i].size(); j++) tFAWCountdown[i][j] --;
            //tfaw the head will always be the smallest counter, so check if it has reached 0
            if (tFAWCountdown[i].size() > 0 && tFAWCountdown[i][0] == 0) tFAWCountdown[i].erase(tFAWCountdown[i].begin());
            
            //sub channel1
            //tfaw decrement all the counters we have going
            for (size_t j = 0; j < tFAWCountdown_sc1[i].size(); j++) tFAWCountdown_sc1[i][j] --;
            //tfaw the head will always be the smallest counter, so check if it has reached 0
            if (tFAWCountdown_sc1[i].size() > 0 && tFAWCountdown_sc1[i][0] == 0) tFAWCountdown_sc1[i].erase(tFAWCountdown_sc1[i].begin());
        } else {
            //tfaw decrement all the counters we have going
            for (size_t j = 0; j < tFAWCountdown[i].size(); j++) tFAWCountdown[i][j] --;
            //tfaw the head will always be the smallest counter, so check if it has reached 0
            if (tFAWCountdown[i].size() > 0 && tFAWCountdown[i][0] == 0) tFAWCountdown[i].erase(tFAWCountdown[i].begin());
        }
    }

    for (size_t i = 0; i < NUM_RANKS; i++) {
        //tfpw decrement all the counters we have going
        for (size_t j = 0; j < tFPWCountdown[i].size(); j++) tFPWCountdown[i][j] --;
        //tfpw the head will always be the smallest counter, so check if it has reached 0
        if (tFPWCountdown[i].size() > 0 && tFPWCountdown[i][0] == 0) tFPWCountdown[i].erase(tFPWCountdown[i].begin());
    }
}

void PTC::calc_occ() {
    if (availability < MAP_CONFIG["DMC_BW_LEVEL"][0]) {
        occ = 0;
        occ_1_cnt ++;
    } else if (availability < MAP_CONFIG["DMC_BW_LEVEL"][1]) {
        occ = 1;
        occ_2_cnt ++;
    } else if (availability < MAP_CONFIG["DMC_BW_LEVEL"][2]) {
        occ = 2;
        occ_3_cnt ++;
    } else {
        occ = 3;
        occ_4_cnt ++;
    }
}

double PTC::calc_sqrt(double sum, double sum_pwr2, unsigned cnt) {
    if (cnt == 0) return 0;
    double avg = sum / cnt;
    double dev = (sum_pwr2 / cnt) - (avg * avg);
    return sqrt(dev);
}

float PTC::CalcBwByByte(uint64_t byte, unsigned cycle) {
    if (IS_LP6) {
        return (100 * float(byte) * 8 * 9 / 8 / 2 / JEDEC_DATA_BUS_BITS / cycle / WCK2DFI_RATIO / PAM_RATIO);
    } else {
        return (100 * float(byte) * 8 / 2 / JEDEC_DATA_BUS_BITS / cycle / WCK2DFI_RATIO / PAM_RATIO);
    }
}

void PTC::update_statistics() {
    if (DMC_BW_WIN != 0 && now() % DMC_BW_WIN == 0) {
        availability = unsigned(CalcBwByByte(bw_totalcmds, DMC_BW_WIN));
        bw_totalcmds   = 0;
        bw_totalwrites = 0;
        bw_totalreads  = 0;

        static unsigned rw_cnt = 0;
        static unsigned act_cnt = 0;
        unsigned row_hit_cnt = 0;
        rw_cnt = read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt - rw_cnt;
        act_cnt = active_cnt - act_cnt;
        row_hit_cnt = (rw_cnt > act_cnt) ? (rw_cnt - act_cnt) : 0;
        row_hit_ratio = (rw_cnt == 0) ? 0 : ((float)row_hit_cnt * 100 / rw_cnt);
        rw_cnt = read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt;
        act_cnt = active_cnt;

        calc_occ();

        sum_avai += availability;
        sum_pwr_avai += availability * availability;
        avai_sqrt = calc_sqrt(double(sum_avai), double(sum_pwr_avai), now() / DMC_BW_WIN + 1);
    }

    if (PRINT_BW && PRINT_BW_WIN != 0 && now() != 0 && now() % PRINT_BW_WIN == 0) {
        float dmc_availability = 0;
        dmc_availability = CalcBwByByte(TotalDmcBytes, PRINT_BW_WIN);
        // TRACE_PRINT(setw(10)<<now()<<" -- Total availability: "<<setw(5)<<fixed<<setprecision(1)<<dmc_availability);
        dmc_availability = CalcBwByByte(TotalDmcRdBytes, PRINT_BW_WIN);
        // TRACE_PRINT(", DMC Rd availability: "<<setw(5)<<fixed<<setprecision(1)<<dmc_availability);
        dmc_availability = CalcBwByByte(TotalDmcWrBytes, PRINT_BW_WIN);
        // TRACE_PRINT(", DMC Wr availability: "<<setw(5)<<fixed<<setprecision(1)<<dmc_availability);
        TotalDmcBytes = 0;
        TotalDmcRdBytes = 0;
        TotalDmcWrBytes = 0;
        // TRACE_PRINT(", Q: "<<setw(3)<<transactionQueue.size());
        // TRACE_PRINT(", QR: "<<setw(3)<<+que_read_cnt);
        // TRACE_PRINT(", QW: "<<setw(3)<<+que_write_cnt);
        unsigned bw_bank_cnt = 0;
        for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
            if ((r_bank_cnt[bank] + w_bank_cnt[bank]) > 0) {
                bw_bank_cnt ++;
            }
        }
        // TRACE_PRINT(", Bank: "<<setw(2)<<bw_bank_cnt<<endl);
    }
}

void PTC::update_state() {
    if (!DEBUG_STATE) return;

    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Total Status: R:"<<que_read_cnt<<" W:"<<que_write_cnt<<" rw_group_state="<<+rw_group_state[0]<<" | in_write_group="
            <<in_write_group<<" | availability="<<availability<<" | row_hit_ratio="<<row_hit_ratio<<" | PreCmd.rank="<<PreCmd.rank
            <<" | PreCmd.type="<<PreCmd.type<<" | rk_grp_state="<<+rk_grp_state<<" | real_rk_grp_state="<<+real_rk_grp_state<<endl)
    for (auto &state : bankStates) {
        unsigned sub_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE) {
            PRINTN("ST time: "<<now()<<" | bank="<<state.bankIndex<<" | sc="<<sub_channel<<" | Rcnt="<<+r_bank_cnt[state.bankIndex]<<" | Wcnt="
                    <<+w_bank_cnt[state.bankIndex]<<" | state="<<bank_state_opcode(state.state->currentBankState)<<" | OpenRowAddr="
                    <<state.state->openRowAddress<<" | nAct1="<<state.state->nextActivate1<<" | nAct2="<<state.state->nextActivate2
                    <<" | nR="<<state.state->nextRead<<" | nW="<<state.state->nextWrite<<" | nWRmw="<<state.state->nextWriteRmw<<" | nMW="
                    <<state.state->nextWriteMask<<" | nRAp="<<state.state->nextReadAp<<" | nWAp="<<state.state->nextWriteAp<<" | nWApRmw="
                    <<state.state->nextWriteApRmw<<" | nMWAp="<<state.state->nextWriteMaskAp<<" | nPre="<<state.state->nextPrecharge
                    <<" | hPre="<<state.hold_precharge<<" | lastCmd="<<state.state->lastCommand<<" | lastCmdType="<<state.state->lastCmdType
                    <<" | lastCmdPri="<<state.state->lastCmdPri<<" | lastCmdSource="<<state.state->lastCmdSource<<" | HasCmdRhit="<<state.has_cmd_rowhit
                    <<" | perf_rd_rowhit="<<state.perf_rd_rowhit<<" | perf_rd_rowmiss="<<state.perf_rd_rowmiss<<" | perf_wr_rowhit="<<state.perf_wr_rowhit 
                    <<" | perf_wr_rowmiss="<<state.perf_wr_rowmiss<<" | act_met="<<state.act_timing_met
                    <<" | samebank_rcnt="<<state.samebank_rcnt<<" | samebank_wcnt="<<state.samebank_wcnt<<" | ptc_samebank_rd="<<state.ptc_samebank_rd 
                    <<" | ptc_samebank_wr="<<state.ptc_samebank_wr<<" | perf_rd_conf="<<state.perf_bankrd_conflict<<" | perf_wr_conf="<<state.perf_bankwr_conflict 
                    <<" | rhit_brk="<<state.has_rhit_break<<" | dummy_tout="<<state.has_dummy_tout<<" | ser_rhit_cnt="<<state.ser_rhit_cnt
                    <<" | row_hit_cnt="<<state.row_hit_cnt<<" | row_miss_cnt="<<state.row_miss_cnt<<" | pbrW="<<refreshPerBank[state.bankIndex].refreshWaiting
                    <<" | pbr="<<refreshPerBank[state.bankIndex].refreshing<<" | pbr_pre="<<refreshPerBank[state.bankIndex].refreshWaitingPre<<" | abrW="
                    <<refreshALL[state.rank][0].refreshWaiting<<" | abr="<<refreshALL[state.rank][0].refreshing
                    <<" | abrW_sc1="<<refreshALL[state.rank][1].refreshWaiting<<" | abr_sc1="<<refreshALL[state.rank][1].refreshing 
                    <<" | stc_cnt="<<state.state->stateChangeCountdown<<" | stc_en="<<state.state->stateChangeEn<<" | POSTPND="
                    <<refreshALL[state.rank][0].refresh_cnt<<" | POSTPND_SC1="<<refreshALL[state.rank][1].refresh_cnt 
                    <<" | Fpbr="<<state.finish_refresh_pb<<" | lp_state="<<RankState[state.rank].lp_state
                    <<" | rank_has_cmd="<<rank_has_cmd[state.rank]<<" | acc_en="<<access_bank_delay[state.bankIndex].enable<<" | acc_cnt="
                    <<access_bank_delay[state.bankIndex].cnt<<" | adpt_openpage_time="
                    <<adpt_openpage_time<<" | opc_cnt="<<opc_cnt<<" | ppc_cnt="<<ppc_cnt<<" | act_executing="<<state.state->act_executing);
        } else {
            PRINTN("ST time: "<<now()<<" | bank="<<state.bankIndex<<" | sc="<<sub_channel<<" | Rcnt="<<+r_bank_cnt[state.bankIndex]<<" | Wcnt="
                    <<+w_bank_cnt[state.bankIndex]<<" | state="<<bank_state_opcode(state.state->currentBankState)<<" | OpenRowAddr="
                    <<state.state->openRowAddress<<" | nAct1="<<state.state->nextActivate1<<" | nAct2="<<state.state->nextActivate2
                    <<" | nR="<<state.state->nextRead<<" | nW="<<state.state->nextWrite<<" | nWRmw="<<state.state->nextWriteRmw<<" | nMW="
                    <<state.state->nextWriteMask<<" | nRAp="<<state.state->nextReadAp<<" | nWAp="<<state.state->nextWriteAp<<" | nWApRmw="
                    <<state.state->nextWriteApRmw<<" | nMWAp="<<state.state->nextWriteMaskAp<<" | nPre="<<state.state->nextPrecharge
                    <<" | hPre="<<state.hold_precharge<<" | lastCmd="<<state.state->lastCommand<<" | lastCmdType="<<state.state->lastCmdType
                    <<" | lastCmdPri="<<state.state->lastCmdPri<<" | lastCmdSource="<<state.state->lastCmdSource<<" | HasCmdRhit="<<state.has_cmd_rowhit
                    <<" | perf_rd_rowhit="<<state.perf_rd_rowhit<<" | perf_rd_rowmiss="<<state.perf_rd_rowmiss<<" | perf_wr_rowhit="<<state.perf_wr_rowhit 
                    <<" | perf_wr_rowmiss="<<state.perf_wr_rowmiss<<" | act_met="<<state.act_timing_met
                    <<" | samebank_rcnt="<<state.samebank_rcnt<<" | samebank_wcnt="<<state.samebank_wcnt<<" | ptc_samebank_rd="<<state.ptc_samebank_rd 
                    <<" | ptc_samebank_wr="<<state.ptc_samebank_wr<<" | perf_rd_conf="<<state.perf_bankrd_conflict<<" | perf_wr_conf="<<state.perf_bankwr_conflict 
                    <<" | rhit_brk="<<state.has_rhit_break<<" | dummy_tout="<<state.has_dummy_tout<<" | ser_rhit_cnt="<<state.ser_rhit_cnt
                    <<" | row_hit_cnt="<<state.row_hit_cnt<<" | row_miss_cnt="<<state.row_miss_cnt<<" | pbrW="<<refreshPerBank[state.bankIndex].refreshWaiting
                    <<" | pbr="<<refreshPerBank[state.bankIndex].refreshing<<" | pbr_pre="<<refreshPerBank[state.bankIndex].refreshWaitingPre<<" | abrW="
                    <<refreshALL[state.rank][0].refreshWaiting<<" | abr="<<refreshALL[state.rank][0].refreshing<<" | stc_cnt="
                    <<state.state->stateChangeCountdown<<" | stc_en="<<state.state->stateChangeEn<<" | POSTPND="
                    <<refreshALL[state.rank][0].refresh_cnt<<" | Fpbr="<<state.finish_refresh_pb<<" | lp_state="<<RankState[state.rank].lp_state
                    <<" | rank_has_cmd="<<rank_has_cmd[state.rank]<<" | acc_en="<<access_bank_delay[state.bankIndex].enable<<" | acc_cnt="
                    <<access_bank_delay[state.bankIndex].cnt<<" | adpt_openpage_time="
                    <<adpt_openpage_time<<" | opc_cnt="<<opc_cnt<<" | ppc_cnt="<<ppc_cnt<<" | act_executing="<<state.state->act_executing);
        }
        PRINTN(endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Command Queue Status: Qsize="<<transactionQueue.size()<<" | QRcnt:"<<que_read_cnt<<" | QWcnt:"
            <<que_write_cnt<<endl)
    for (auto &trans : transactionQueue) {
        PRINTN("Que time: "<<now()<<" | task="<<trans->task<<" | bank="<<trans->bankIndex<<" | rank="<<trans->rank<<" | row="<<trans->row
                <<" | addr_col="<<trans->addr_col<<" | arb_time="<<trans->arb_time<<" | type="<<trans_type_opcode(trans->transactionType)
                <<" | nextCmd="<<trans_cmd_opcode(trans->nextCmd)<<" | arb_grp="<<trans->arb_group<<" | arb_grp_pri_col="
                <<arb_group_pri_col[trans->arb_group]<<" | arb_grp_pri_row="<<arb_group_pri_row[trans->arb_group]<<" | arb_grp_cnt="<<arb_group_cnt[trans->arb_group]
                <<" | address="<<hex<<trans->address<<dec<<" | length="<<trans->burst_length<<" | data_size="<<trans->data_size<<" | issue_size="
                <<trans->issue_size<<" | data_ready_cnt="<<trans->data_ready_cnt<<" | timeout="<<trans->timeout<<" | bp_by_tout="
                <<trans->bp_by_tout<<" | qos="<<trans->qos<<" | pri="<<trans->pri<<" | addrconf="<<trans->addrconf<<" | timeout_th="
                <<trans->timeout_th<<" | ptc_timeAdded="<<trans->ptc_timeAdded<<" | pri_adapt_th="<<trans->pri_adapt_th<<" | timeAdded="<<trans->timeAdded
                <<" | act_executing="<<trans->act_executing<<" | pre_act="<<trans->pre_act<<" mask_wcmd="<<trans->mask_wcmd<<endl)
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    if (ENH_PAGE_ADPT_EN && now() % ENH_PAGE_ADPT_WIN == 0 && now() != 0) {
        PRINTN("EHS_PAGE_R: "<<now());
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++)
            PRINTN(" | R"<<i<<"= "<<setw(3)<<page_timeout_rd[i]);
        PRINTN(endl);
        PRINTN("EHS_PAGE_W: "<<now());
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++)
            PRINTN(" | R"<<i<<"= "<<setw(3)<<page_timeout_wr[i]);
        PRINTN(endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
}

std::string PTC::bank_state_opcode(CurrentBankState state) {
    switch (state) {
        case Idle        : {return "Idle"; break;}
        case RowActive   : {return "RowActive"; break;}
        case Precharging : {return "Precharging"; break;}
        case Refreshing  : {return "Refreshing"; break;}
        default          : break;
    }
    return "Unkown opcode";
}

std::string PTC::trans_cmd_opcode(TransactionCmd state) {
    switch (state) {
        case PRECHARGE_SB_CMD     : {return "PRECHARGE_SB"; break;}
        case PRECHARGE_PB_CMD     : {return "PRECHARGE_PB"; break;}
        case PRECHARGE_AB_CMD     : {return "PRECHARGE_AB"; break;}
        case ACTIVATE1_CMD        : {return "ACTIVATE1"; break;}
        case ACTIVATE2_CMD        : {return "ACTIVATE2"; break;}
        case WRITE_CMD            : {return "WRITE"; break;}
        case WRITE_P_CMD          : {return "WRITE_P"; break;}
        case WRITE_MASK_CMD       : {return "WRITE_MASK"; break;}
        case WRITE_MASK_P_CMD     : {return "WRITE_MASK_P"; break;}
        case READ_CMD             : {return "READ"; break;}
        case READ_P_CMD           : {return "READ_P"; break;}
        case REFRESH_CMD          : {return "REFRESH"; break;}
        case INVALID              : {return "INVALID"; break;}
        case PER_BANK_REFRESH_CMD : {return "PER_BANK_REFRESH"; break;}
        case PDE_CMD              : {return "PDE"; break;}
        case PD_CMD               : {return "PD"; break;}
        case PDX_CMD              : {return "PDX"; break;}
        case ASREF_CMD            : {return "ASREF"; break;}
        case ASREFX_CMD           : {return "ASREFX"; break;}
        case SRPDE_CMD            : {return "SRPDE"; break;}
        case SRPD_CMD             : {return "SRPD"; break;}
        case SRPDX_CMD            : {return "SRPDX"; break;}
        default                   : break;
    }
    return "Unkown opcode";
}

std::string PTC::trans_type_opcode(TransactionType state) {
    switch (state) {
        case DATA_READ       : {return "READ"; break;}
        case DATA_WRITE      : {return "WRITE"; break;}
        default              : break;
    }
    return "Unkown opcode";
}

/***************************************************************************************************
descriptor: main update ,every cycle will execute it
****************************************************************************************************/
void PTC::update() {

    update_even_cycle();

    update_statistics();

    update_state_pre();

    update_state();

    power_event_stat();

    update_lp_state();

    update_wdata();
    
    ent_pop();

    rw_pop();

    for (size_t i = 0; i < CORE_CONCURR; i ++) {
        if (now() % CORE_CONCURR_PRD != 0 && i == 1) break;
        core_concurr_en = (i == 0);

        dfi_pipe();

        state_fresh();

        scheduler();
    }

    
    faw_update();

    queue_update();

    sch_pipe();

    bsc_update();

    page_timeout_policy();

    for (size_t i=0 ; i<sc_num; i++) {
        refresh(i);
    }

    data_fresh();

    update_state_post();

    ehs_page_adapt_policy();

    update_deque_fifo();
}

void PTC::rank_group_weight(unsigned * rank, unsigned * type) {
    vector <unsigned> weight;
    weight.resize(NUM_RANKS * 2);
    // 0->R0R, 1->R0W, 2->R1R, 3->R1W
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        // Read command weight high
        weight[i * NUM_RANKS] = (*r_rank_mux[i] == 0) ? 0 : *r_rank_mux[i] + 24;
        weight[i * NUM_RANKS + 1] = *w_rank_mux[i];
        // Same rank weight high, will reduce rank switch to reduce power consumption 
        if (real_rk_grp_state != NO_RGRP && unsigned(real_rk_grp_state >> 1) == i) {
            if (*r_rank_mux[i] != 0) weight[i * NUM_RANKS] += 4;
            if (*w_rank_mux[i] != 0) weight[i * NUM_RANKS + 1] += 4;
        }
    }

    unsigned bigger = 0;
    for (size_t i = 0; i < 2; i ++) {
        for (size_t j = 0; j < NUM_RANKS; j ++) {
            size_t index = j * 2 + i;
            if (index == real_rk_grp_state) continue;
            if (weight[index] > bigger) {
                bigger = weight[index];
                *type = i;
                *rank = j;
            }
        }
    }
}

// Interface with perf queue
unsigned PTC::getPtcQueSize(bool isRd) {
    uint32_t queRdNum = 0;
    uint32_t queWrNum = 0;

    uint32_t size = transactionQueue.size();

    for (uint32_t index = 0; index < size; index++) {
        if (transactionQueue.at(index)->transactionType == DATA_READ) {
            queRdNum++;
        } else {
            queWrNum++;
        }
    }
    if (isRd) {
        return queRdNum;
    } else {
        return queWrNum;
    }
}

BankTableState PTC::get_bank_state(unsigned bankIndex) {
    for (auto &state: bankStates) {
        if (state.bankIndex == bankIndex) {
            return state;
        }
    }
    // no match bankState
    ERROR(setw(10)<<now()<<" -- Error parameter get_bank_state, bankIndex="<<bankIndex<<", num_banks="<<(NUM_RANKS*NUM_BANKS));
    assert(0);
}

bool PTC::get_bs_full(Transaction *trans) {
    unsigned bs_idle = 0;
    bs_cnt++;
    for(size_t i=0;i<bank_slot.size();i++){
        if(bank_slot[i] == 0xFFFF){
            bs_idle ++;
        }
    }
    //used to statistic
    bs_total = bs_total + (BANK_SLOT_NUM - bs_idle);
    if((BANK_SLOT_NUM - bs_idle) > bs_max) bs_max = (BANK_SLOT_NUM - bs_idle);

    bool bankidx_in_slot = false;
    for(size_t i=0;i<bank_slot.size();i++){
        if(trans->bankIndex == bank_slot[i]){
            bankidx_in_slot = true;
        } 
    }
    return (!bankidx_in_slot && (bs_idle<=BS_BP_LVL));
}

unsigned PTC::get_rank_cnt(unsigned rank, bool isRd) {    
    if (rank >= NUM_RANKS) {
        ERROR(setw(10)<<now()<<" -- Error parameter get_rank_cnt, rank="<<rank<<", num_ranks="<<NUM_RANKS);
        assert(0);
    }

    if (isRd) {
        return r_rank_cnt[rank];
    } else {
        return w_rank_cnt[rank];
    }     
}

unsigned PTC::get_bg_cnt(unsigned rank, unsigned group, bool isRd) {    
    if (rank >= NUM_RANKS || group >= NUM_GROUPS) {
        ERROR(setw(10)<<now()<<" -- Error parameter get_bg_cnt, rank="<<rank<<", num_ranks="<<NUM_RANKS
                <<", group="<<group<<", num_groups="<<NUM_GROUPS);
        assert(0);
    }

    if (isRd) {
        return r_bg_cnt[rank][group];
    } else {
        return w_bg_cnt[rank][group];
    }     
}

unsigned PTC::get_bank_cnt(unsigned bankIndex, bool isRd) {
    if (bankIndex >= (NUM_RANKS*NUM_BANKS)) {
        ERROR(setw(10)<<now()<<" -- Error parameter get_bank_cnt, bankIndex="<<bankIndex<<", num_banks="<<(NUM_RANKS*NUM_BANKS));
        assert(0);
    }

    if (isRd) {
        return r_bank_cnt[bankIndex];
    } else {
        return w_bank_cnt[bankIndex];
    }     
}

bool PTC::get_refresh_state(unsigned bankIndex) {
    if (bankIndex >= (NUM_RANKS*NUM_BANKS)) {
        ERROR(setw(10)<<now()<<" -- Error parameter get_refresh_state, bankIndex="<<bankIndex<<", num_banks="<<(NUM_RANKS*NUM_BANKS));
        assert(0);
    }
    unsigned rank = bankIndex / NUM_BANKS;
    unsigned sub_channel = (bankIndex % NUM_BANKS) / sc_bank_num;
    if (refreshALL[rank][sub_channel].refreshing || refreshALL[rank][sub_channel].refreshWaiting
            || refreshPerBank[bankIndex].refreshing || refreshPerBank[bankIndex].refreshWaiting) {
        return true;
    } else {
        return false;
    }

}

unsigned PTC::get_perf_timeout_rank_cnt(unsigned rank) {
    unsigned perf_timeout_cnt = 0;
    for (auto &trans : transactionQueue) {
        if (trans->rank != rank) continue;
        if ((trans->timeout && trans->timeout_type==0) || trans->hqos_timeout) {
            perf_timeout_cnt ++;
        }
    }
    return perf_timeout_cnt; 
}

void PTC::queue_update() {
    // pre_act
    size_t len = transactionQueue.size();
    uint8_t delete_cnt = 0;
    for (size_t i = 0; i < len; i++) {
        size_t real_i = i - delete_cnt;
        Transaction *t = transactionQueue.at(real_i);
        unsigned sub_channel_act = (t->bankIndex % NUM_BANKS) / sc_bank_num;
        if (t->issue_size < t->data_size) continue;
        if (t->pre_act || (t->fast_rd && t->act_only)) {
            bool act_type = false;
            if (t->pre_act) {
                pre_act_cnt --;
            } else {
                fast_rd_cnt --;
                fast_act_cnt --;
                r_rank_cnt[t->rank] --;
                r_rank_bst[t->rank] -= ceil(float(t->data_size) / max_bl_data_size);
                que_read_cnt --;
                r_bank_cnt[t->bankIndex] --;
                r_bg_cnt[t->rank][t->group] --;
                rank_cnt[t->rank] --;
                sc_cnt[t->rank][sub_channel_act] --;
                bank_cnt[t->bankIndex] --;
                bg_cnt[t->rank][t->group] --;
                act_type = true;
            }
            //bankstate info update
            if (bankStates[t->bankIndex].ptc_samebank_rd == 0) {
                ERROR(setw(10)<<now()<<" -- Error BankState Info, task="<<t->task);
                assert(0);
            }
            bankStates[t->bankIndex].ptc_samebank_rd --;
            ptc_que_slot[t->ptc_slot] = 0;
            t->reset();
            delete t;
            delete_cnt ++;
            transactionQueue.erase(transactionQueue.begin() + real_i);
            if (DEBUG_BUS) {
                if (!act_type) {
                    PRINTN(setw(10)<<now()<<" -- DELETE PRE_ACT :: task="<<t->task<<", bank="
                            <<t->bankIndex<<", row"<<t->row<<", pre_act="<<t->pre_act<<endl);
                } else {
                    PRINTN(setw(10)<<now()<<" -- DELETE FAST_READ_ACT :: task="<<t->task<<", bank="
                            <<t->bankIndex<<", row"<<t->row<<", byp_act="<<t->byp_act<<endl);
                }
            }
        }
    }


    len = transactionQueue.size();
    for (size_t i = 0; i < len; i++) {
        Transaction *t = transactionQueue[i];
        unsigned sub_channel = (t->bankIndex % NUM_BANKS) / sc_bank_num;
        if (t->issue_size < t->data_size) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- issue_size < data_size"<<t->issue_size<<","<<t->data_size<<endl);
            }
            continue;
        }
        if (t->issue_size > t->data_size) {
            ERROR(setw(10)<<now()<<" -- Error issue_size, task="<<t->task<<" data_size="
                    <<t->data_size<<" issue_size="<<t->issue_size);
            assert(0);
        }
        
        //bankstate info update
        if (t->transactionType == DATA_READ) {
            r_rank_cnt[t->rank] --;
            r_rank_bst[t->rank] -= ceil(float(t->data_size) / max_bl_data_size);
            que_read_cnt --;
            r_bank_cnt[t->bankIndex] --;
            r_bg_cnt[t->rank][t->group] --;
            ptc_r_bank_cnt[t->bankIndex] ++;
            ptc_r_bg_cnt[t->rank][t->group] ++;
            r_sid_cnt[t->rank][t->sid] --;
            r_qos_cnt[t->qos] --;
            if (t->fast_rd) fast_rd_cnt --; 
            if (RDATA_TYPE == 1 && t->mask_wcmd==false) {
                pfq_->gen_rresp(t->task, t->channel);
            }
            // ptc bankstate info update
            if (bankStates[t->bankIndex].ptc_samebank_rd == 0 || bankStates[t->bankIndex].samebank_rcnt == 0) {
                ERROR(setw(10)<<now()<<" -- Error BankState Info, task="<<t->task);
                assert(0);
            }
            bankStates[t->bankIndex].samebank_rcnt --;
            bankStates[t->bankIndex].ptc_samebank_rd --;
            if (bankStates[t->bankIndex].samebank_rcnt == 0) {
                bankStates[t->bankIndex].perf_bankrd_conflict = false;
            }
            
            // PTC inform PERF queue to release rd slot
            if (PERF_RD_RELEASE_MODE == 0) {
                pfq_->perf_release_rd(t->task, true);
            }
            //remove conflict
            for (size_t j = i + 1; j < len; j++) {
                Transaction *tmp = transactionQueue[j];
//                if (tmp->task == t->task) continue;
                if (tmp->addrconf && ((tmp->address & ~ALIGNED_SIZE) == (t->address & ~ALIGNED_SIZE)) &&
                        tmp->transactionType != DATA_READ && (tmp->addr_block_source_id == t->task)) {
                    tmp->addrconf = false;
                    if (DEBUG_BUS) {
                         PRINTN(setw(10)<<now()<<" -- RL_CONF :: release ,task="<<tmp->task<<endl);
                    }
                }
            }
            if (t->qos >= PERF_SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[t->rank] --;
                highqos_r_bank_cnt[t->bankIndex] --;
            }
        } else {
            w_rank_cnt[t->rank] --;
            w_rank_bst[t->rank] -= ceil(float(t->data_size) / max_bl_data_size);
            if (!IECC_ENABLE || !tasks_info[t->task].wr_ecc) {
                pfq_->gen_wresp(t->task, t->channel);
            }
            // ptc bankstate info update
            if (bankStates[t->bankIndex].ptc_samebank_wr == 0 || bankStates[t->bankIndex].samebank_wcnt == 0) {
                ERROR(setw(10)<<now()<<" -- Error BankState Info, task="<<t->task);
                assert(0);
            }
            // ptc bankstate info update
            bankStates[t->bankIndex].samebank_wcnt --;
            bankStates[t->bankIndex].ptc_samebank_wr --;
            if (DEBUG_BUS) {

            }
            if (bankStates[t->bankIndex].samebank_wcnt == 0) {
                bankStates[t->bankIndex].perf_bankwr_conflict = false;
            }
            // PTC inform PERF queue to release wr SLOT_FIFO
                pfq_->perf_release_wr(t->task, true);
                if (DEBUG_BUS) {
                     PRINTN(setw(10)<<now()<<" -- perf_release_wr :: release ,task="<<t->task<<endl);
                }
                //wait for wdata transfered from perf
                Transaction *wtrans = new Transaction(t);
                wtrans->data_ready_cnt = 0;
                WtransQueue.push_back(wtrans);
                que_write_cnt --;
                w_qos_cnt[t->qos] --;
                w_bank_cnt[t->bankIndex] --;
                w_bg_cnt[t->rank][t->group] --;
                ptc_w_bank_cnt[t->bankIndex] ++;
                ptc_w_bg_cnt[t->rank][t->group] ++;
                w_sid_cnt[t->rank][t->sid] --;
                //remove conflict
                for (size_t j = i + 1; j < len; j++) {
                    Transaction *tmp = transactionQueue[j];
//                    if (tmp->task == t->task) continue;
                    if (tmp->addrconf && ((tmp->address & ~ALIGNED_SIZE) == (t->address & ~ALIGNED_SIZE))
                            && (tmp->addr_block_source_id == t->task)) {
                        tmp->addrconf = false;
                        if (DEBUG_BUS) {
                             PRINTN(setw(10)<<now()<<" -- RL_CONF :: release ,task="<<tmp->task<<endl);
                        }
                    }
                }
        }

        if (WRITE_BUFFER_ENABLE) pfq_->dmc_release_conflict(t);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- DELETE :: task="<<t->task<<", bank="
                    <<t->bankIndex<<", rowhit="<<!t->act_executing<<", fast_rd="<<t->fast_rd<<endl);
        }

        rank_cnt[t->rank] --;
        sc_cnt[t->rank][sub_channel] --;
        bank_cnt[t->bankIndex] --;
        bg_cnt[t->rank][t->group] --;
        sid_cnt[t->rank][t->sid] --;
        bankStates[t->bankIndex].row_hit_cnt --;
        if (t->timeout && !t->dummy_push) {
            dmc_timeout_cnt ++;
            qos_timeout_cnt[t->qos] ++;
            // statistics fast timeout and slow timeout
            if (((now() - t->time_timeout) * tDFI) <= TOUT_SCH_TIME) {
                qos_fast_timeout_cnt[t->qos] ++;
                dmc_fast_timeout_cnt ++;
            } else {
                qos_slow_timeout_cnt[t->qos] ++;
                dmc_slow_timeout_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Unqualified Timeout :: task="<<t->task<<", bank="
                            <<t->bankIndex<<", rank="<<t->rank<<", time_timeout="<<t->time_timeout<<", tDFI="<<tDFI<<endl);
                }
                if (PRINT_TIMEOUT) {
                    PRINTN(setw(10)<<now()<<" -- Unqualified Timeout :: task="<<t->task<<", bank="
                            <<t->bankIndex<<", rank="<<t->rank<<", time_timeout="<<t->time_timeout<<", tDFI="<<tDFI<<endl);
                }
            }
            if (t->cmd_rt_type) RtCmdCnt ++;
        }

        if (ENH_PAGE_ADPT_EN && !t->has_send_act) bank_cnt_ehs[t->bankIndex] ++;

        ptc_que_slot[t->ptc_slot] = 0;
        t->reset();
        delete t;


        transactionQueue.erase(transactionQueue.begin() + i);
        dmc_cmd_cnt ++;
        rw_cmd_num ++;
        break;
    }
    
    // update act_timing_met for PERF queue
    for (auto &state : bankStates) {
        state.act_timing_met = false;
        if ((now()+ACT_LEFT_CYCLE) >= state.state->nextActivate1) {
            state.act_timing_met = true;
            // if (DEBUG_BUS) {
                // PRINTN(setw(10)<<now()<<" -- Trp Timing Met, rank="<<state.rank<<", group="<<state.group
                        // <<", bank="<<state.bank<<", bankIndex="<<state.bankIndex<<", state="<<bank_state_opcode(state.state->currentBankState)<<endl);
            // }
        } 
    }
}

void PTC::bsc_update() {
    // update bank_slot
    if(BSC_EN){
        for (auto &state : bankStates) {
            if (now()>= state.state->nextActivate2 && (state.state->lastCommand == PRECHARGE_SB_CMD || state.state->lastCommand == PRECHARGE_PB_CMD || state.state->lastCommand == PRECHARGE_AB_CMD)) {
                bool sameidx_in_ptc = false;
                for (size_t i = 0; i < transactionQueue.size(); i++) {
                    if(state.bankIndex == transactionQueue[i]->bankIndex){
                        sameidx_in_ptc = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- PTC QUE still have same bank cmd, rank="<<state.rank<<", group="<<state.group
                                    <<", bank="<<state.bank<<", bankIndex="<<state.bankIndex<<", state="<<bank_state_opcode(state.state->currentBankState)<<" nextActivate2_time="<<state.state->nextActivate2<<" task="<<transactionQueue[i]->task<<" i="<<i<<endl);

                        }
                        break;
                    }
                }
                if(!sameidx_in_ptc){
                    for(size_t j=0;j<bank_slot.size();j++){
                        if(bank_slot[j] == state.bankIndex){
                            bank_slot[j] = 0xFFFF;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- PTC QUE do not have same bank cmd recycle slot, rank="<<state.rank<<", group="<<state.group
                                        <<", bank="<<state.bank<<", bankIndex="<<state.bankIndex<<", state="<<bank_state_opcode(state.state->currentBankState)<<" nextActivate2_time="<<state.state->nextActivate2<<endl);
                            }
                        }
                        
                    }
                }
            }
        }
        if(DEBUG_BUS){
            PRINTN(setw(10)<<now()<<" -- BANK SLOT after update_que:: ");
            for(unsigned i=0;i<bank_slot.size();i++){
                if(i!=0 && i%4 == 0){
                    PRINTN("    ");
                }
                PRINTN(bank_slot[i]<<" ");
            }
            PRINTN(endl);
        }
    }
}

void PTC::sch_pipe() {
    for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        bankStates[i].has_cmd_rowhit = false;
        bankStates[i].hold_precharge = false;
        bankStates[i].has_timeout = false;
//        bankStates[i].has_rhit_break = false;
        bankStates[i].has_highqos_cmd_rowhit = false;
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        has_timeout_rank[rank] = false;
    }

    if (PreCmdTime!=0xffffffff) {
        PreCmdTime ++;
    }

    rw_exec_cnt = 0;
    for (auto &trans : transactionQueue) {
        if (trans->issue_size != 0) rw_exec_cnt ++;
        // if (DEBUG_BUS) {
            // PRINTN(setw(10)<<now()<<" -- Trans issue_size="<<trans->issue_size<<", task="<<trans->task<<endl);
        // }
    }
    if (PERF_TIMEOUT_MODE == 0) {
        check_timeout_and_aging();
    } else if (PERF_TIMEOUT_MODE == 1) {
        ptc_check_timeout_and_aging();
    }
    if (dfs_backpress_en) total_dfs_bp_cnt ++;

    for (auto &t : transactionQueue) {
        unsigned t_state = (t->rank << 1) | t->transactionType;
        unsigned sub_channel = (t->bankIndex % NUM_BANKS) / sc_bank_num;
        if (t->addrconf) continue;
        if (t->pre_act) continue;
        // if (now() < t->arb_time) continue;
        if (t->bp_by_tout) continue;
        if (refreshALL[t->rank][sub_channel].refreshing) continue;    //todo: revise for e-mode
        //if (refreshPerBank[t->bankIndex].refreshing) continue;
        if (bankStates[t->bankIndex].state->currentBankState == RowActive &&
                t->row == bankStates[t->bankIndex].state->openRowAddress) {
            if ((rw_group_state[0] == READ_GROUP && t->transactionType == DATA_READ) || (rk_grp_state == t_state)
                    || (rw_group_state[0] == WRITE_GROUP && t->transactionType == DATA_WRITE)) {
                bankStates[t->bankIndex].has_cmd_rowhit = true;
            }
            if (rw_group_state[0] == READ_GROUP && t->qos >= PERF_SWITCH_HQOS_LEVEL && t->transactionType == DATA_READ) {
                bankStates[t->bankIndex].has_highqos_cmd_rowhit = RHIT_HQOS_BREAK_EN;
            }
        }
    }

    for (auto &trans : transactionQueue) {
        if ((now() - trans->timeAdded) > 1000000 || (now() - trans->ptc_timeAdded) > 100000) {
            ERROR(setw(10)<<now()<<" -- task="<<trans->task<<" address="<<hex<<trans->address<<dec
                    <<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" matgrp="
                    <<(trans->row&(NUM_MATGRPS-1)));
            ERROR(setw(10)<<now()<<" -- error, qos="<<trans->qos<<", pri="<<trans->pri);
            ERROR(setw(10)<<now()<<" -- FATAL ERROR == big latency"<<", chnl:"<<channel);
            assert(0);
        }

        if (trans->transactionType != DATA_READ) {
            if (now() - trans->ptc_timeAdded > 100000 && trans->data_ready_cnt <= trans->burst_length) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] task="<<trans->task<<" Wdata number miss match, EXP="
                        <<trans->burst_length<<", ACT="<<trans->data_ready_cnt);
                assert(0);
            }
        }
    }
    
    // perf grp state switch cnt update
    if (que_read_cnt == 0 && que_write_cnt == 0) {
        perf_grpst_switch_cnt = 0;
        perf_state_pre = pfq_->wbuff_state;
    } else if ((que_read_cnt > 0 && que_write_cnt == 0) || (que_read_cnt == 0 && que_write_cnt > 0)) {
        perf_grpst_switch_cnt = 0;
        perf_state_pre = pfq_->wbuff_state;
    } else if ((que_read_cnt > 0 && que_write_cnt > 0) || (que_read_cnt > 0 && que_write_cnt > 0)) {
        if (pfq_->wbuff_state != perf_state_pre) {
            perf_grpst_switch_cnt ++;
        }
        perf_state_pre = pfq_->wbuff_state;
    }

    // statistics of long-short-long (r-w-r or w-r-w)
    bool rw_coexist = que_read_cnt > 0 && que_write_cnt > 0;
    bool read_idle = PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD && pfq_->wbuff_state == WBUFF_WRITE;
    bool write_idle = PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD && pfq_->wbuff_state == WBUFF_IDLE;
    if (rw_coexist && (read_idle || write_idle)) {
        rw_idle_cnt ++; 
    }
    
    //check if vld cmd of other ranks different from previous executed cmd
    has_other_rank_cmd = false;
    for (auto &trans : transactionQueue) {
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
        if (trans->rank == PreCmd.rank) continue;
        if (trans->transactionType != PreCmd.trans_type) continue;
        if (trans->addrconf) continue;
        if (trans->pre_act) continue;
        if (trans->fast_rd && trans->act_only) continue;
//        if (now() < trans->enter_que_time) continue;
        if (refreshALL[trans->rank][sub_channel].refreshing) continue;
        if (refreshPerBank[trans->bankIndex].refreshing) continue;
        if (trans->bp_by_tout) continue;
        if (r_rank_cnt[trans->rank] < PTC_RRANK_SWITCH_LEVEL && trans->transactionType == DATA_READ) continue;
        if (w_rank_cnt[trans->rank] < PTC_WRANK_SWITCH_LEVEL && trans->transactionType == DATA_WRITE) continue;
        if (trans->transactionType == DATA_READ) has_other_rank_cmd = true;
        else if (trans->data_ready_cnt == (trans->burst_length + 1)) has_other_rank_cmd = true;
    }

    //check if high qos hit hqos_rank
    has_hqos_rank_rcmd = false;
    for (auto &trans : transactionQueue) {
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
        if (trans->transactionType == DATA_WRITE) continue;
        if (trans->pre_act) continue;
        if (trans->fast_rd && trans->act_only) continue;
//        if (now() < trans->enter_que_time) continue;
        if (refreshALL[trans->rank][sub_channel].refreshing) continue;
        if (refreshPerBank[trans->bankIndex].refreshing) continue;
        if (trans->bp_by_tout) continue;
        if (trans->qos < PERF_SWITCH_HQOS_LEVEL) continue;
        if (trans->rank == pfq_->hqos_rank) {
            has_hqos_rank_rcmd = true;
        }
    }

    table_use_cnt = 0;
    for (auto &state : bankStates) {
        if (state.state->currentBankState == RowActive || state.state->currentBankState == Precharging)
            table_use_cnt ++;
    }

    for (size_t bank_idx = 0; bank_idx < NUM_RANKS * NUM_BANKS; bank_idx ++) {
        issue_state[bank_idx] = false;
    }
    

    //hqos_push_mode = 2
    if (FAST_HQOS_PUSH_EN && FAST_HQOS_PUSH_MODE == 2) {
        for (auto &trans : transactionQueue) {
            unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
            if (trans->transactionType == DATA_WRITE) continue;
            // if (now() < trans->arb_time) continue;
            if (refreshALL[trans->rank][sub_channel].refreshing) continue;
            if (refreshPerBank[trans->bankIndex].refreshing) continue;
            if (trans->timeout) continue;
            if (trans->pre_act) continue;
            if (!trans->hqos_push) continue;
            if (PreCmd.trans_type != trans->transactionType) continue;
            if (PreCmd.rank != trans->rank) {
                trans->timeout = true;
                trans->pri = PTC_QOS_MAX;
            }
        }
    }

    if (PTC_HQOS_RANK_SWITCH_EN) {
        all_rank_has_cmd = true;
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (rank_cmd_high_qos[i]) rank_cmd_high_qos[i] = que_read_highqos_cnt[i] >= PERF_RCMD_HQOS_RANK_SWITCH_LEVELL;
            else rank_cmd_high_qos[i] = que_read_highqos_cnt[i] >= PERF_RCMD_HQOS_RANK_SWITCH_LEVELH;
            if (rank_cnt[i] < PTC_HQOS_RANK_SWITCH_LEVEL1 && !rank_cmd_high_qos[i]) {
                all_rank_has_cmd = false;
            }
            for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- read trig,rank_cmd_high_qos="<<rank_cmd_high_qos[i]
                                <<",rank="<<i<<",que_read_highqos_cnt="<<que_read_highqos_cnt[i]<<endl);
            }
            }
        }
        for (auto &trans : transactionQueue) {
            if (trans->transactionType == DATA_WRITE) continue;
            if (trans->pri < PERF_SWITCH_HQOS_LEVEL) continue;
            rank_cmd_high_qos[trans->rank] = 1;//write cmd or read cmd was hqos pushed ,both need mark hqos rank
            for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- hqos push trig,rank_cmd_high_qos="<<rank_cmd_high_qos[i]
                                <<",rank="<<i<<",que_read_highqos_cnt="<<que_read_highqos_cnt[i]<<endl);
            }
            }
        }
        hqos_rank = 0xffffffff;
        //only one rank has hqos cmd
        if (((unsigned)accumulate(rank_cmd_high_qos.begin(), rank_cmd_high_qos.end(), 0) == 1) && all_rank_has_cmd) {
            for (size_t i = 0; i < NUM_RANKS; i ++) {
                if (rank_cmd_high_qos[i]) {
                    hqos_rank = i;
                }
            }
        }

        if (hqos_rank != 0xffffffff && PTC_HQOS_RANK_SWITCH_MODE == 0) {
            // label hqos with timeout
            for (auto &trans : transactionQueue) {
                unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
                if (trans->transactionType == DATA_WRITE) continue;
                if (trans->qos < PERF_SWITCH_HQOS_LEVEL) continue;
                // if (now() < trans->arb_time) continue;
                if (refreshALL[trans->rank][sub_channel].refreshing) continue;
                if (refreshPerBank[trans->bankIndex].refreshing) continue;
                if (trans->timeout) continue;
                if (trans->pre_act) continue;
                if (trans->rank == hqos_rank) {
                    trans->timeout = true;
                    trans->pri = PTC_QOS_MAX;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- Label Timeout for Hqos, task="<<trans->task<<", rank="<<trans->rank
                                <<", bankIndex="<<trans->bankIndex<<", hqos_rank="<<hqos_rank<<endl)
                    }
                }
            }
        }
    }

    //PTC constraint check
    ptc_constraint_check();

    // cancel fast rd cmd under full command mode
    if (FAST_READ_EN && FAST_READ_MODE == 1 && FAST_RD_CANCEL_EN) {
        for (auto & trans : transactionQueue) {
            auto &state = bankStates[trans->bankIndex];
            if (trans->transactionType == DATA_WRITE) continue;
            if (!trans->fast_rd) continue;
            if (trans->act_only) continue;
            if (pfq_->perf_conflict_intf(trans->task)) {
                if (trans->issue_size !=0) {
                    ERROR(setw(10)<<now()<<" Fast Rd Cancel Failed, task="<<trans->task);
                    assert(0);
                }
                if (FAST_RD_CANCEL_MODE == 1) {
                    ERROR(setw(10)<<now()<<" Cancel Mode1 No conflict, task="<<trans->task);
                    assert(0);
                }
                trans->act_only = true;
                trans->issue_size = trans->data_size;
                total_fastrd_cancel_cnt ++;
                fast_act_cnt ++;
                total_fast_act_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" Fast Rd Cancel Success, task="<<trans->task<<endl);
                }

                // statistics decrease
                acc_rank_cnt[trans->rank] --;
                acc_bank_cnt[trans->bankIndex] --;
                totalReads --;
                r_sid_cnt[trans->rank][trans->sid] --;
                r_qos_cnt[trans->qos] --;
                racc_rank_cnt[trans->rank] --;
                racc_bank_cnt[trans->bankIndex] --;
                TotalDmcRdBytes -= trans->data_size;
                if (trans->data_size == 32) TotalDmcRd32B --;
                else if (trans->data_size == 64) TotalDmcRd64B --;
                else if (trans->data_size == 128) TotalDmcRd128B --;
                else if (trans->data_size == 256) TotalDmcRd256B --;
                if ((trans->address % trans->data_size) == 0) rd_inc_cnt --;
                else rd_wrap_cnt --;
                if (trans->qos >= PERF_SWITCH_HQOS_LEVEL) {
                    que_read_highqos_cnt[trans->rank] --;
                    highqos_r_bank_cnt[trans->bankIndex] --;
                }
                // update bankState info
                state.perf_rd_rowhit = false;           // perf_addrconf_rd not considered as rd_rowhit
                state.perf_wr_rowhit = true;            // perf_addrconf_rd -> wr_rowhit must exist
            }
        }
    }
    
    if (!SLOT_FIFO) {
        if (DEBUG_BUS) {
            for (size_t arb_grp_idx = 0; arb_grp_idx < ARB_GROUP_NUM; arb_grp_idx++) {
                PRINTN(setw(10)<<now()<<" -- Current Col Arb_Group_Pri="<<arb_group_pri_col[arb_grp_idx]<<" Arb_Group="<<arb_grp_idx<<endl);
                PRINTN(setw(10)<<now()<<" -- Current Row Arb_Group_Pri="<<arb_group_pri_row[arb_grp_idx]<<" Arb_Group="<<arb_grp_idx<<endl);
            }
        }

        // select tran from high rr pri to low rr pri, keep rd/wr pri and act pri seperately
        Col_Row_lc(arb_group_pri_col, 0);
        Col_Row_lc(arb_group_pri_row, 1);
    } else {    
        for (auto &trans : transactionQueue) {
            if (dfs_backpress_en) {
                trans->timeAdded ++;
                continue;
            }

            // if (arb_enable  && even_cycle && now() >= trans->arb_time) cmd_select(trans);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- cmd_select, task="<<trans->task<<endl);
            }
            if (arb_enable  && even_cycle ) cmd_select(trans);
            
            if (trans->issue_size != 0) {
                issue_state[trans->bankIndex] = true;
            }
        }
        Cmd *rwc = NULL;
        for (auto &cmd : rw_cmdqueue) {
            if(prerw.task != -1){
                if(cmd->task == prerw.task && cmd->cmd_type == prerw.cmd_type && cmd->issue_size == prerw.issue_size) {
                    if (DEBUG_BUS&&rwc!=NULL) {
                        PRINTN(setw(10)<<now()<<" -- has same cmd    ,task="<<cmd->task<<" type="<<cmd->cmd_type<<" issue_size="<<cmd->issue_size<<endl);
                        PRINTN(setw(10)<<now()<<" -- has same cmd pre,task="<<prerw.task<<" type="<<prerw.cmd_type<<" issue_size="<<prerw.issue_size<<endl);
                    }
                    continue;
                }
            }
            if (cmd->issue_size != 0) {rwc = cmd; break;}
            if (rwc == NULL) {rwc = cmd; continue;}
            if (priority_pri(cmd) > priority_pri(rwc)) {
                rwc = cmd;
            } else if (priority_pri(cmd) == priority_pri(rwc)){
                if(lru_arb(cmd->ptc_slot, rwc->ptc_slot, 0)) rwc = cmd;
            }
            // if (DEBUG_BUS&&rwc!=NULL) {
                // PRINTN(setw(10)<<now()<<" -- rw_cmdque win the arg,task="<<rwc->task<<" type="<<rwc->cmd_type<<" issue_size="<<rwc->issue_size<<endl);
                // PRINTN(setw(10)<<now()<<" -- rw_cmdque pop to pre,task="<<prerw.task<<" type="<<prerw.cmd_type<<" issue_size="<<prerw.issue_size<<endl);
            // }
        }
        // if (DEBUG_BUS&&rwc!=NULL) {
        //     PRINTN(setw(10)<<now()<<" -- rw_cmdque pop to delay_buf,task="<<rwc->task<<" type="<<rwc->cmd_type<<" issue_size="<<rwc->issue_size<<endl);
        //     PRINTN(setw(10)<<now()<<" -- rw_cmdque pop to pre,task="<<prerw.task<<" type="<<prerw.cmd_type<<" issue_size="<<prerw.issue_size<<endl);
        // }
        if (rwc!=NULL) {
            rw_delay_buf.push_back(rwc);
            prerw.creat_Cmd(rwc);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- rwcmd to rwdelay, task="<<rwc->task<<endl);
            }
        }
        Cmd *actc = NULL;
        for (auto &cmd : act_cmdqueue) {
            if (cmd->issue_size != 0) {actc = cmd; break;}
            if (actc == NULL) {actc = cmd; continue;}
            if (priority_pri(cmd) > priority_pri(actc)) {
                actc = cmd;
            } else if (priority_pri(cmd) == priority_pri(actc)) {
                if(lru_arb(cmd->ptc_slot, actc->ptc_slot, 0)) actc = cmd;
            }
        }
        if (actc!=NULL) {
            CmdQueue.push_back(actc);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- CmdQueue_size="<<CmdQueue.size()<<", actc task="<<actc->task<<endl);
            }
        }

        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- CmdQueue_size="<<CmdQueue.size()<<endl);
        }
    }

}

void PTC::Col_Row_lc(vector<unsigned> &arb_group_pri, unsigned arb_type) {
    for (size_t i=0; i<ARB_GROUP_NUM; i++) {
        for (size_t j=0; j<ARB_GROUP_NUM; j++){
            if (arb_group_pri[j] == ((ARB_GROUP_NUM-1)-i)) {
                for (auto &trans: transactionQueue) {
                    if (trans->arb_group == j) {
                        if (dfs_backpress_en) {
                            trans->timeAdded ++;
                            continue;
                        }
                        
                        if (arb_type == 0 && bankStates[trans->bankIndex].state->currentBankState!=RowActive) continue;
                        else if (arb_type == 1 && bankStates[trans->bankIndex].state->currentBankState==RowActive) continue;
                        
                        if (arb_enable  && even_cycle) cmd_select(trans);
                        
                        if (trans->issue_size != 0) {
                            issue_state[trans->bankIndex] = true;
                        }
                    }
                } 
            }
        }
    }
}

void PTC::ptc_constraint_check() {
    //maximal cmd for each bank group in PTC

    //romiss && maximal rowhit check for each bank in PTC
    for (auto &state: bankStates) {
        unsigned samebank_rd_rowhit_cnt = 0;
        unsigned samebank_wr_rowhit_cnt = 0;
        unsigned samebank_rowmiss_cnt = 0;
        for (auto &trans : transactionQueue) {
            if (state.bankIndex != trans->bankIndex) continue;
            if (trans->row != state.state->openRowAddress) {
                samebank_rowmiss_cnt ++;
                if (samebank_rowmiss_cnt > SAME_BANK_CNT_ROWMISS && SAME_BANK_CNT_ROWMISS!=0) {
                    ERROR(setw(10)<<now()<<" row_miss cmd forbidden in PTC, task="<<trans->task<<", bankIndex="<<trans->bankIndex
                            <<", row="<<trans->row<<", openrow="<<state.state->openRowAddress);
                    assert(0);
                }
            } else {
                if (trans->transactionType == DATA_READ) {
                    if (!((trans->timeout && trans->timeout_type==0) || trans->hqos_timeout)) {  // perf timeout/perf hqos pesudo timeout exclude
                        samebank_rd_rowhit_cnt ++;
                    }
                } else {
                    if (!(trans->timeout && trans->timeout_type==0)) {  // perf timeout exclude
                        samebank_wr_rowhit_cnt ++;
                    }
                }
                if (samebank_rd_rowhit_cnt > SAME_BANK_CNT_RD && SAME_BANK_CNT_RD!=0) {
                    ERROR(setw(10)<<now()<<" rd_row_hit cmd exceed 2 in PTC, task="<<trans->task<<", bankIndex="<<trans->bankIndex
                            <<", row="<<trans->row<<", openrow="<<state.state->openRowAddress);
                    assert(0);
                } else if (samebank_wr_rowhit_cnt > SAME_BANK_CNT_WR && SAME_BANK_CNT_WR!=0) {
                    ERROR(setw(10)<<now()<<" wr_row_hit cmd exceed 2 in PTC, task="<<trans->task<<", bankIndex="<<trans->bankIndex
                            <<", row="<<trans->row<<", openrow="<<state.state->openRowAddress);
                    assert(0);
                }
            }
        }
    }
    
    // rank cmd check in PTC
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        unsigned samerank_cnt = 0;
        for (auto &trans : transactionQueue) {
//            if (trans->fast_rd || trans->pre_act) continue;
            if (trans->timeout && trans->timeout_type==0) continue;  // perf timeout exclude
            if (trans->hqos_timeout) continue;  // perf hqos timeout exclude
            if (trans->rank == rank) {
                samerank_cnt ++;
                // if (PTC_RANK_NUM!=0) {
                //     if (samerank_cnt > (TRANS_QUEUE_DEPTH / PTC_RANK_NUM)) {
                //         ERROR(setw(10)<<now()<<" Same Rank cnt exceed upper limit in PTC, task="<<trans->task<<", rank="<<rank
                //                 <<", bankIndex="<<trans->bankIndex<<", row="<<trans->row);
                //         assert(0);
                //     }
                // }
            }
        }
    }

}

void PTC::page_timeout_policy() {
    if (!PAGE_TIMEOUT_EN && !PERF_PRE_EN && !PTC_RHIT_BREAK_EN && !PERF_DUMMY_TOUT_EN) return;
    
    for (auto &state : bankStates) {
        unsigned sub_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;

        // precharge force by rowmiss
        bool hold_pre = (state.perf_rd_rowhit && pfq_->wbuff_state == WBUFF_IDLE) 
                            || (state.perf_wr_rowhit && pfq_->wbuff_state == WBUFF_WRITE)
                            || ((state.perf_rd_rowhit || state.perf_wr_rowhit) && pfq_->wbuff_state == WBUFF_NO_GROUP);
        bool perf_force_pre = ((!hold_pre && (state.perf_rd_rowmiss || state.perf_wr_rowmiss)) || state.perf_rowhit_break) && PERF_PRE_EN;
        
        // precharge forced by page tiemout
        bool perf_bank_cmd_exist = state.perf_rd_rowhit || state.perf_wr_rowhit || state.perf_rd_rowmiss || state.perf_wr_rowmiss;
        bool page_force_pre = ((access_bank_delay[state.bankIndex].cnt >= page_timeout_rd[state.bankIndex] && state.state->lastCmdType == DATA_READ) 
                                || (state.state->lastCmdType == DATA_WRITE && access_bank_delay[state.bankIndex].cnt >= page_timeout_wr[state.bankIndex]))
                                && !perf_bank_cmd_exist && PAGE_TIMEOUT_EN;

        // precharge forced by rowhit break
        bool rhit_break_force_pre = state.has_rhit_break;

        //precharge forced by dummy timeout
        bool dummy_tout_force_pre = state.has_dummy_tout;

        if (EM_ENABLE && EM_MODE==2 && state.rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode
        bank_cas_delay[state.bankIndex] ++;
        if ((w_bank_cnt[state.bankIndex] + r_bank_cnt[state.bankIndex]) > 0) continue;
        if (state.state->currentBankState != RowActive) continue;
        if (page_force_pre || perf_force_pre || rhit_break_force_pre || dummy_tout_force_pre) {
            if ((now() + 1) >= state.state->nextPrecharge && tFPWCountdown[state.rank].size() < 4 && even_cycle) {   //every other command, even;
                if (arb_enable) {
                    funcState[state.rank].wakeup = true;
                    if (RankState[state.rank].lp_state == IDLE) {
                        Cmd *c = new Cmd;
                        c->rank = state.rank;
//                        c->channel = sub_channel;
                        c->group = state.group;
                        c->bank = state.bank;
                        c->pri = 0;
                        c->row = state.state->openRowAddress;
                        c->cmd_type = PRECHARGE_PB_CMD;
                        c->bankIndex = state.bankIndex;
                        if (perf_force_pre) {
                            c->cmd_source = 0;
                        } else if (page_force_pre) { 
                            c->cmd_source = 1;
                        } else if (rhit_break_force_pre) {
                            c->cmd_source = 3;
                        } else if (dummy_tout_force_pre) {
                            c->cmd_source = 4;
                        } else {
                            ERROR(setw(10)<<now()<<" Precharge Failed, bank="<<c->bankIndex);
                            assert(0);
                        }
                        c->task = 0xFFFFFFFFFFFFFFF;
                        //CmdQueue.push_back(c);
                        pre_cmdqueue.push_back(c);
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ :: PAGE/PERF/RHIT_BRK/DUMMY, precharge bank="
                                    <<c->bankIndex<<", task="<<c->task<<", rank="<<c->rank<<", bank="<<c->bankIndex
                                    <<", cmd_source="<<c->cmd_source<<endl);
                        }
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold PRECHARGE in"
                                    <<" lp state, rank"<<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                    }
                }
            }
        } else if (access_bank_delay[state.bankIndex].enable) {
            access_bank_delay[state.bankIndex].cnt ++;
        }
    }
}


/***************************************************************************************************
descriptor: The purpose of this function is receive the data packet and transmit packet
****************************************************************************************************/
void PTC::data_fresh() {
    for (auto &cmd : CmdQueue) {
        if (cmd == NULL) continue;
        if (RankState[cmd->rank].lp_state != IDLE) {
            ERROR(setw(10)<<now()<<" -- DMC["<<getID()<<"] Send command in lp state, cmd_type="
                    <<cmd->cmd_type<<", task="<<cmd->task<<", rank="<<cmd->rank
                    <<", bank="<<cmd->bankIndex);
            assert(0);
        }
    }

    unsigned size = writeDataToSend.size();
    if (size > 0) {
        for (size_t i = 0; i < size; i ++) {
            if (writeDataToSend[i].delay > 0)
                writeDataToSend[i].delay --;
        }

        if (writeDataToSend[0].delay == 0) {
            //send to bus and print debug stuff
            for (size_t j = 0; j < WtransQueue.size(); j++) {
                if (writeDataToSend[0].task != WtransQueue[j]->task) continue;
                if (WtransQueue[j]->data_ready_cnt < (WtransQueue[j]->burst_length + 1)) continue;
                auto it = wdata_info.find(writeDataToSend[0].task);
                if (it != wdata_info.end()) {
                    wdata_info[writeDataToSend[0].task]--;
                }
                if (wdata_info[writeDataToSend[0].task] == 0) {
                    wdata_info.erase(writeDataToSend[0].task);
                    delete WtransQueue[j];
                    WtransQueue.erase(WtransQueue.begin() + j);
                    if (IECC_ENABLE) tasks_info[writeDataToSend[0].task].wr_finish = true;
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- T_DDR :: write data on Data Bus, task="
                            <<writeDataToSend[0].task<<endl);
                }
                writeDataToSend.erase(writeDataToSend.begin());
            }
        }
    }

    //check for outstanding data to return to the CPU
//     size = read_data_buffer.size();
//     for (size_t i = 0; i < size; i ++) {
//         if (read_data_buffer[i].delay == 0) {
//             unsigned long long task = read_data_buffer[i].task;
//             if (DEBUG_BUS) {
//                 PRINTN(setw(10)<<now()<<" -- T_CPU :: Issuing to CPU bus, task="<<task<<endl);
//             }
//             auto it = pending_TransactionQue.find(task);
//             if (it == pending_TransactionQue.end()) {
//                 ERROR(setw(10)<<now()<<" -- [DMC"<<channel<<"]"<<" mismatch data, task="<<task);
//                 assert(0);
// //                read_data_buffer.erase(read_data_buffer.begin() + i);
// //                break;
//             }

//             TRANS_MSG msg = it->second;
//             msg.burst_cnt ++;
//             read_data_buffer[i].channel = msg.channel;
//             if (DEBUG_BUS) {
//                 PRINTN(setw(10)<<now()<<" -- Rdata burst_cnt="<<static_cast<int>(msg.burst_cnt)<<endl);
//             }

//             //wait nums of rdata to return to HA
//             unsigned wait_rdata_num = 0;
//             wait_rdata_num = RDATA_RETURN_NUM;
//             // PTC inform PERF queue to release rd slot
//             if (PERF_RD_RELEASE_MODE == 1) { 
//                 if (msg.burst_cnt * BLEN *JEDEC_DATA_BUS_BITS == ((msg.burst_length + 1)*DMC_DATA_BUS_BITS)) {
//                     pfq_->perf_release_rd(task, true);
//                 }
//             }

// #ifdef SYSARCH_PLATFORM
//             unsigned rdata_type = 0;
//             if (msg.burst_cnt == (msg.burst_length + 1)) rdata_type |= 1; // bit[0] is rdata_end
//             if (msg.burst_cnt == 0) rdata_type |= (1 << 1); // bit[1] is rdata_start
//             rdata_type |= (msg.qos << 2); // bit[5:2] is qos
//             rdata_type |= (msg.pf_type << 6); // bit[7:6] is pf_type
//             rdata_type |= (msg.sub_pftype << 8); // bit[11:8] is pf_type
//             rdata_type |= (msg.sub_src << 12); // bit[13:12] is pf_type
//             msg.reqAddToDmcTime = double(rdata_type);
// #endif
//             if ((!IECC_ENABLE || !tasks_info[task].rd_ecc) && (!RMW_ENABLE_PERF || (!read_data_buffer[i].mask_wcmd && RMW_ENABLE_PERF))) {
//                 read_data_buffer[i].readDataEnterDmcTime = now() * tDFI;
//                 if (!returnReadData(read_data_buffer[i].channel, task,
//                         read_data_buffer[i].readDataEnterDmcTime,
//                         msg.reqAddToDmcTime, msg.reqEnterDmcBufTime)) {
//                     if (PRINT_READ) {
//                         TRACE_PRINT(setw(10)<<now()<<" -- Rdata Back Pressure :: task="<<task<<" ch="<<channel<<endl);
//                     }
//                     if (DEBUG_BUS) {
//                         PRINTN(setw(10)<<now()<<" -- Rdata Back Pressure or rdata not fully collected:: task="<<task<<" ch="<<channel<<" burst_cnt="<<static_cast<int>(msg.burst_cnt)<<" wait_rdata_num"<<wait_rdata_num<<endl);
//                     }
//                     return;
//                 } else {
//                     pre_rdata_time = now();
//                     rdata_cnt ++;
//                     if (PRINT_READ) {
//                         TRACE_PRINT(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
//                                 <<ceil(((read_data_buffer[i].readDataEnterDmcTime
//                                 - msg.reqEnterDmcBufTime) / tDFI))<<" ch="<<channel<<endl);
//                     }
//                     if (PRINT_IDLE_LAT) {
//                         DEBUG(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
//                                 <<ceil(((read_data_buffer[i].readDataEnterDmcTime
//                                 - msg.reqEnterDmcBufTime) / tDFI)));
//                     }
//                     if (DEBUG_BUS) {
//                         PRINTN(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
//                                 <<ceil(((read_data_buffer[i].readDataEnterDmcTime
//                                 - msg.reqEnterDmcBufTime) / tDFI))<<endl);
//                     }
//                 }
//             }
//             if (msg.burst_cnt== ((msg.burst_length + 1))) {
//                 ReturnData_statistics(task, msg.time, msg.qos, msg.mid, msg.pf_type, msg.rank);
//             }
//             //return latency
//             if (msg.burst_cnt== ((msg.burst_length + 1))) {
//                 pending_TransactionQue.erase(task);
//                 if (IECC_ENABLE) tasks_info[task].rd_finish = true;
//                 if (RMW_ENABLE && read_data_buffer[i].mask_wcmd==true)  {
//                     auto iter = rmw_rd_finish.find(task);
//                     if (iter == rmw_rd_finish.end()){
//                         ERROR(setw(10)<<now()<<" -- Merge Read Data Mismatch, task="<<task);
//                         assert(0);
//                     }
//                     rmw_rd_finish[task] = true;
//                 }
//                 if (DEBUG_BUS) {
//                     PRINTN(setw(10)<<now()<<" -- Finish :: Issuing to CPU bus, task="<<task<<endl);
//                 }
//             } else {
//                 it->second.burst_cnt = msg.burst_cnt;
//             }
//             read_data_buffer.erase(read_data_buffer.begin() + i);
//             break;
//         } else {
//             if (IS_G3D) break;
//         }
//     }
    size = read_data_buffer.size();
    for (size_t i = 0; i < size; i++) {
        if (read_data_buffer[i].delay > 0) read_data_buffer[i].delay--;
    }
}

/***************************************************************************************************
descriptor: update state var pre
****************************************************************************************************/
void PTC::update_state_pre() {
    for (auto &state : bankStates) {
        unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE && EM_MODE==2 && state_channel==1 && state.rank==1) continue;     //rank1, sc1 forbidden under combo e-mode 
        if (state.state->currentBankState == Idle) bank_idle_cnt[state.rank] ++;
        if (state.state->currentBankState == RowActive) bank_act_cnt[state.rank] ++;
    }

    for (auto &trans : transactionQueue) {
        // if (now() < trans->arb_time) continue;
        if (trans->transactionType != DATA_READ) continue;
        if (RankState[trans->rank].lp_state == IDLE) continue;
        auto &st = RankState[trans->rank].lp_state;
        if (st >= PDE && st <= PDX) rd_met_pd_cnt ++;
        else if (st >= ASREFE && st <= SRPDX) rd_met_asref_cnt ++;
    }

    if (TRFC_CC_EN) {     // tdo: revise for e-mode
        bool has_pbr = false, has_abr = false;
        unsigned cc = 0, lp_cnt = 0;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            for (size_t i = 0; i < sc_num; i++){
                if (refreshALL[rank][i].refreshing) has_abr = true;
                break;
            }
//            if (refreshALL[rank].refreshing) has_abr = true;
            if (RankState[rank].lp_state != IDLE) lp_cnt ++;
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bank;
                if (refreshPerBank[bank_tmp].refreshing) has_pbr = true;
                if (bankStates[bank_tmp].state->currentBankState == RowActive) break;
                if (bankStates[bank_tmp].state->currentBankState == Precharging) break;
                cc ++;
            }
        }
        if ((cc == NUM_RANKS * NUM_BANKS) && (lp_cnt != NUM_RANKS)) {
            if (has_abr) abr_cc_cnt ++;
            else if (has_pbr) pbr_cc_cnt ++;
        }
    }

    bool has_casfs = false;
    for (size_t i = 0; i < NUM_RANKS; i ++) if (send_wckfs[i]) has_casfs = true;
    if (has_casfs) casfs_time ++;

    // useless?
    for (size_t rank = 0; rank < NUM_RANKS; rank++) {
        if (!pbr_hold_pre[rank]) continue;
        bool pbr_bank_open = false;
        for (size_t bank = 0; bank < NUM_BANKS; bank++) {
            unsigned sub_channel = bank / sc_bank_num;
            if (EM_ENABLE && EM_MODE==2 && rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode 
            if (!refreshPerBank[rank * NUM_BANKS + bank].refreshWaiting) continue;
            if (bankStates[rank * NUM_BANKS + bank].state->currentBankState != RowActive) continue;
            pbr_bank_open = true;
            break;
        }
        if (pbr_bank_open) continue;
        if (now() >= pbr_hold_pre_time[rank]) pbr_hold_pre[rank] = false;
    }
}

/***************************************************************************************************
descriptor: update state var
****************************************************************************************************/
void PTC::update_state_post() {
    // Command bp erase
    uint8_t bp_size = bp_cycle.size();
    uint8_t erase_cnt = 0;
    for (size_t i = 0; i < bp_size; i ++) {
        if (now() >= bp_cycle[i - erase_cnt]) {
            bp_cycle.erase(bp_cycle.begin() + i - erase_cnt);
            erase_cnt ++;
        }
    }
}

/***************************************************************************************************
descriptor: Load tFPW
****************************************************************************************************/
void PTC::LoadTfpw(uint8_t rank, unsigned tfpw) {
    if (IS_GD2 && tfpw != 0) tFPWCountdown[rank].push_back(tfpw);
}

/***************************************************************************************************
descriptor: Load tFAW
****************************************************************************************************/
void PTC::LoadTfaw(uint8_t rank, unsigned tfaw, unsigned sc) {
    if (!IS_GD1 && !IS_G3D) {
        if (sc == 0) {   // sub channel 0
            tFAWCountdown[rank].push_back(tfaw);
        } else if (sc == 1 && EM_ENABLE) {   // sub channel 1 under e-mode
            tFAWCountdown_sc1[rank].push_back(tfaw);
        } else {
            ERROR(setw(10)<<now()<<" -- FAW Mode Config Wrong, sc="<<sc<<", e-mode="<<EM_ENABLE);
            assert(0);
        }
    }
}

/***************************************************************************************************
descriptor: generate for DDR packet.
****************************************************************************************************/
void PTC::generate_packet(Cmd *c) {
    //now that we know there is room in the command queue, we can remove from the transaction queue
    bool hit = false;
    unsigned sub_channel = (c->bankIndex % NUM_BANKS) / sc_bank_num;
//    unsigned bank_start = sub_channel * NUM_BANKS / sc_num; 
//    unsigned bank_pair_start = sub_channel * pbr_bank_num; 
    command.reset();
    switch (c->cmd_type) {
        case PRECHARGE_PB_CMD : {
            command_pend = rw_cycle;
            LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: PRECHARGE_PB Remove bank="
                        <<c->bankIndex<<" Hold Precharge, task="<<c->task<<endl);
            }
            break;
        }
        case PRECHARGE_SB_CMD : {
            command_pend = pre_cycle;
            LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: PRECHARGE_SB Remove bank="
                        <<c->bankIndex<<" Hold Precharge, task="<<c->task<<endl);
            }
            break;
        }
        case PRECHARGE_AB_CMD : {
            command_pend = pre_cycle;
            LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: PRECHARGE_AB Remove rank="
                        <<c->rank<<" Hold Precharge, task="<<c->task<<endl);
            }
            break;
        }
        case READ_CMD :
        case READ_P_CMD : {
            if (c->cmd_type == READ_P_CMD && c->fg_ref) command_pend = pre_cycle;
            else if (!RankState[c->rank].wck_on) command_pend = rw_cycle * 2;
            else command_pend = rw_cycle;
            TotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            flowStatisTotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            bw_totalcmds += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            TotalReadBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (IECC_ENABLE && tasks_info[c->task].rd_ecc) {
                ecc_total_bytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
                ecc_total_reads += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            }
            if (RMW_ENABLE_PERF && c->mask_wcmd) {
                rmw_total_bytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
                rmw_total_reads += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            }
            TotalBytesRank[c->rank] += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (c->cmd_type == READ_P_CMD) LoadTfpw(c->rank, tFPW);
            
            if (RMW_ENABLE && IECC_ENABLE && tasks_info[c->task].rd_ecc && c->mask_wcmd) {
                ERROR(setw(10)<<now()<<" -- Merge read can not coexisit with ECC read, task="<<c->task);
                assert(0);
            }
              
            break;
        }
        case WRITE_CMD :
        case WRITE_P_CMD :
        case WRITE_MASK_CMD :
        case WRITE_MASK_P_CMD : {
            if ((c->cmd_type == WRITE_P_CMD || c->cmd_type == WRITE_MASK_P_CMD) && c->fg_ref) command_pend = pre_cycle;
            else if (!RankState[c->rank].wck_on) command_pend = rw_cycle * 2;
            else command_pend = rw_cycle;
            //create read or write command and enqueue it
            DataPacket dp;
            //dp.delay = WL;
            dp.task = c->task;
            dp.bl = c->bl;
            writeDataToSend.push_back(dp);
            this->wdata_info[c->task]++;
            if (c->cmd_type == WRITE_P_CMD || c->cmd_type == WRITE_MASK_P_CMD
                    || (c->issue_size + max_bl_data_size) >= c->data_size) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMHOLD :: WRITE Remove bank="
                            <<c->bankIndex<<" Hold Precharge,task="<<c->task<<endl);
                }
            }
            TotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            flowStatisTotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            bw_totalcmds += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            bw_totalwrites += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            TotalWriteBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_total_bytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
                ecc_total_writes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            }
            TotalBytesRank[c->rank] += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (c->cmd_type == WRITE_P_CMD) LoadTfpw(c->rank, tFPW);
            if (c->cmd_type == WRITE_MASK_P_CMD) LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: Remove bank="
                        <<c->bankIndex<<" Hold Precharge,task="<<c->task<<endl);
            }
            break;
        }
        case ACTIVATE1_CMD : {
            command_pend = cmd_cycle;
            break;
        }
        case ACTIVATE2_CMD: {
            command_pend = cmd_cycle;
            if (DEBUG_BUS) {
                if (c->pre_act) {
                    PRINTN(setw(10)<<now()<<" -- HOLD :: bank="<<c->bankIndex
                            <<" PreAct Not Hold Precharge, task="<<c->task<<endl);
                } else {
                    PRINTN(setw(10)<<now()<<" -- HOLD :: bank="<<c->bankIndex
                            <<" Hold Precharge, task="<<c->task<<endl);
                }
            }

            if (c->type == DATA_READ) bankStates[c->bankIndex].write_hold = false;
            else bankStates[c->bankIndex].write_hold = true;
            //if its an activate, add a t_faw counter
            LoadTfaw(c->rank, tFAW, sub_channel);
            break;
        }
        case REFRESH_CMD : {
            command_pend = cmd_cycle;
            break;
        }
        case PER_BANK_REFRESH_CMD : {
            command_pend = cmd_cycle;
            LoadTfaw(c->rank, tFAW, sub_channel);
            if (ENH_PBR_EN) {
                bankStates[c->fst_bankIndex].finish_refresh_pb = true;
                bankStates[c->lst_bankIndex].finish_refresh_pb = true;
            } else {
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    bankStates[c->bankIndex + sbr_bank * pbr_bank_num].finish_refresh_pb = true;
                }
            }
            break;
        }
        default : break;
    }

    if (c->cmd_type == READ_CMD || c->cmd_type == READ_P_CMD) {
        for (auto &t : transactionQueue) {
            if (t->task != c->task) continue;
            if (t->issue_size == 0) {
                uint64_t rd_lat = (now() + 1 - t->inject_time);
                cmd_in2dfi_lat += rd_lat;
                cmd_in2dfi_cnt ++;
                cmd_rdmet_cnt --;
//                Cmd2Dfi_statistics(t->task, t->timeAdded, t->perf_qos, t->mid, t->pf_type, t->rank);
                Cmd2Dfi_statistics(t->task, t->inject_time, t->perf_qos, t->mid, t->pf_type, t->rank);
                if (PRINT_LATENCY) {
                    DEBUG(setw(10)<<now()<<" -- LAT :: cnt="<<cmd_in2dfi_cnt<<", lat="<<rd_lat<<", lat_all="
                            <<cmd_in2dfi_lat<<", ave_lat="<<float(cmd_in2dfi_lat)/float(cmd_in2dfi_cnt)<<", task="
                            <<t->task<<", inject_time="<<t->inject_time<<", timeAdded="<<t->timeAdded
                            <<", qos="<<t->qos<<", pri="<<t->pri<<", perf_qos="<<t->perf_qos<<", group="<<t->group
                            <<", bank="<<t->bankIndex);
                }
            }
            break;
        }
    }

    if (command_pend == 1) arb_enable = true;
    command.creat(c);
    exec_valid = true;
    for (auto &trans : transactionQueue) {
        if (trans->task != c->task) continue;
        if (trans->nextCmd >= WRITE_CMD && trans->nextCmd <= READ_P_CMD) {
            trans->issue_size += trans->trans_size;
            trans->addr_col += trans->trans_size;
            if (DEBUG_BUS) {
                PRINT(setw(10)<<now()<<" -- issue size +, task="<<trans->task<<",issue_size="<<trans->issue_size<<endl);
            }
        } else if (trans->nextCmd == ACTIVATE2_CMD) {
            trans->has_active = true;
        }
        if ((trans->pre_act || (trans->fast_rd && trans->act_only)) && trans->nextCmd == ACTIVATE2_CMD) {
            trans->issue_size = trans->data_size;
        }
    }

    tsc_update(command,hit);
}

/***************************************************************************************************
descriptor: check timeout and aging.
****************************************************************************************************/
void PTC::check_timeout_and_aging() {
    // priority adapt
    if (PTC_PRI_ADPT_ENABLE || MPAM_MAPPING_TIMEOUT) {
        for (auto &trans : transactionQueue) {
            // if (now() < trans->arb_time) continue;
            if (trans->addrconf) continue;
            if (trans->pri_adapt_th == 0 || trans->qos == PTC_QOS_MAX) continue;
            if (now() - trans->ptc_timeAdded >= ((trans->improve_cnt + 1) * trans->pri_adapt_th)) {
                trans->improve_cnt ++;
                if (trans->pri < PTC_QOS_MAX) {
                    trans->pri ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PRI_APAPT :: task="<<trans->task<<" pri="<<trans->pri<<endl);
                    }
                }
            }
        }
    }

    // timeout check
    if (!PTC_TIMEOUT_ENABLE && !MPAM_MAPPING_TIMEOUT) return;

    // calculate the highest pri for all commands in DMC Queue
    unsigned highest_pri = 0;
    if (QOS_POLICY == 2) {
        for (auto &trans : transactionQueue) {
            // if (now() < trans->arb_time) continue;
            if (trans->pri > highest_pri) highest_pri = trans->pri;
        }
    }


    bool has_tout_cmd = false;
    bool has_rt_tout_cmd = false;
    bool has_hqos_tout_cmd = false;
    // generate original timeout flag
    for (auto &trans : transactionQueue) {
        // if (now() < trans->arb_time) continue;
        if (trans->addrconf) continue;
        if ((trans->transactionType == DATA_WRITE) && (trans->data_ready_cnt < (trans->burst_length+1))) continue;
        if (!trans->timeout && ((now() - trans->ptc_timeAdded >= trans->timeout_th &&
                trans->timeout_th != 0) || trans->qos == PTC_QOS_MAX)) {
            trans->timeout = true;
            trans->timeout_type = 1;
            trans->time_timeout = now();
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<endl);
            }
            if (PRINT_TIMEOUT) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<endl);
            }
        } else if (trans->timeout && trans->pri != PTC_QOS_MAX && ((QOS_POLICY == 1) || (QOS_POLICY == 2 && trans->pri >= highest_pri))) {
            trans->pri = PTC_QOS_MAX;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT_PRI :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<endl);
            }
            if (PRINT_TIMEOUT) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT_PRI :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<endl);
            }
        }
        if (trans->timeout && !has_tout_cmd) has_tout_cmd = true;
        if (trans->timeout && trans->cmd_rt_type && !has_rt_tout_cmd) has_rt_tout_cmd = true;
        if (trans->timeout && trans->cmd_hqos_type && !has_hqos_tout_cmd) has_hqos_tout_cmd = true;
    }

    tout_high_pri = 0;
    // generate real timeout flag
    for (auto &trans : transactionQueue) {
        if (!trans->timeout) continue;
        if (trans->pri > tout_high_pri) tout_high_pri = trans->pri;
    }

    // set bankStates has_timeout flag
    for (auto &trans : transactionQueue) {
        if (trans->timeout && trans->pri >= tout_high_pri) {
            bankStates[trans->bankIndex].has_timeout = true;
        }
    }

    // generate DMC Queue bp flag
    for (auto &trans : transactionQueue) {
        trans->bp_by_tout = false;
        if (has_rt_tout_cmd && !trans->timeout) trans->bp_by_tout = true;
        else if (has_hqos_tout_cmd && !trans->timeout) trans->bp_by_tout = true;
        else if (QOS_POLICY == 1 && has_tout_cmd && !trans->timeout) trans->bp_by_tout = true;
        else if (QOS_POLICY == 2 && has_tout_cmd && (trans->pri < tout_high_pri || (!trans->timeout && trans->pri == tout_high_pri))) {
            trans->bp_by_tout = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC Backpress by tout :: task="<<trans->task<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<" tout_high_pri="<<tout_high_pri<<endl);
            }
        }
    }
}

/***************************************************************************************************
descriptor: ptc check timeout and aging.
****************************************************************************************************/
void PTC::ptc_check_timeout_and_aging() {
    // priority adapt
    if (PTC_PRI_ADPT_ENABLE || MPAM_MAPPING_TIMEOUT) {
        for (auto &trans : transactionQueue) {
            // if (now() < trans->arb_time) continue;
            if (trans->addrconf) continue;
            if (trans->pri_adapt_th == 0 || trans->qos == PTC_QOS_MAX) continue;
            if (now() - trans->timeAdded >= ((trans->improve_cnt + 1) * trans->pri_adapt_th)) {
                trans->improve_cnt ++;
                if (trans->pri < PTC_ADAPT_PRI_MAX) {  // adapt low qos to pri_max && no adapt for hqos
                    trans->pri ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PTC PRI_APAPT :: task="<<trans->task<<" pri="<<trans->pri
                                <<" qos="<<trans->qos<<endl);
                    }
                }
            }
        }
    }

    //timeout check
    if (!PTC_TIMEOUT_ENABLE && !MPAM_MAPPING_TIMEOUT) return;

    // calculate the highest pri for all commands in DMC Queue
    unsigned highest_pri = 0;
    if (QOS_POLICY == 2) {
        for (auto &trans : transactionQueue) {
            // if (now() < trans->arb_time) continue;
            if (trans->pri > highest_pri) highest_pri = trans->pri;
        }
    }


    bool has_tout_cmd = false;
    bool has_rt_tout_cmd = false;
    bool has_hqos_tout_cmd = false;
    bool has_timeout_rd = false;
    bool has_timeout_wr = false;
    // generate original timeout flag
    for (auto &trans : transactionQueue) {
        // if (now() < trans->arb_time) continue;
        if (trans->addrconf) continue;
        if ((trans->transactionType == DATA_WRITE) && (trans->data_ready_cnt < (trans->burst_length+1))) continue;
        if (!trans->timeout && ((now() - trans->timeAdded >= trans->timeout_th &&
                trans->timeout_th != 0) || trans->qos == PTC_QOS_MAX)) {
            trans->timeout = true;
            trans->timeout_type = 1;
            trans->time_timeout = now();
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<endl);
            }
            if (PRINT_TIMEOUT) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<endl);
            }
        } else if (trans->timeout && trans->pri != PTC_QOS_MAX && ((QOS_POLICY == 1) || (QOS_POLICY == 2 && trans->pri >= highest_pri))) {
            trans->pri = PTC_QOS_MAX;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT_PRI :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<endl);
            }
            if (PRINT_TIMEOUT) {
                PRINTN(setw(10)<<now()<<" -- PTC TIMEOUT_PRI :: task="<<trans->task<<" type="<<trans->transactionType<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<endl);
            }
        }
        if (trans->timeout && !has_tout_cmd) has_tout_cmd = true;
        if (trans->timeout && trans->cmd_rt_type && !has_rt_tout_cmd) has_rt_tout_cmd = true;
        if (trans->timeout && trans->cmd_hqos_type && !has_hqos_tout_cmd) has_hqos_tout_cmd = true;
    }
    
    tout_high_pri = 0;
    // generate real timeout flag
    for (auto &trans : transactionQueue) {
        if (!trans->timeout) continue;
        if (trans->pri > tout_high_pri) tout_high_pri = trans->pri;
    }

    // set bankStates has_timeout flag
    for (auto &trans : transactionQueue) {
        if (trans->timeout && trans->pri >= tout_high_pri) {
            bankStates[trans->bankIndex].has_timeout = true;
        }
    }

    // set has_timeout flag for each rank
    for (auto &trans : transactionQueue) {
        if (!trans->timeout) continue;
        has_timeout_rank[trans->rank] = true;
    }
    
    // set has_timeout flag for each transactionType 
    for (auto &trans : transactionQueue) {
        if (!trans->timeout) continue;
        if (trans->transactionType == DATA_READ) has_timeout_rd = true;
        else has_timeout_wr = true;
    }

    // generate DMC Queue bp flag
    for (auto &trans : transactionQueue) {
        trans->bp_by_tout = false;
        if (has_rt_tout_cmd && !trans->timeout) trans->bp_by_tout = true;//Wasted
        else if (has_hqos_tout_cmd && !trans->timeout) trans->bp_by_tout = true;//Wasted
        else if (QOS_POLICY == 1 && has_tout_cmd && !trans->timeout) {
            if (PTC_TIMEOUT_MODE == 0) {
                trans->bp_by_tout = true;
            } else if (PTC_TIMEOUT_MODE == 1) {
                // backpress by diff_rank timeout
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    if (rank == trans->rank) continue;
                    if (has_timeout_rank[rank]) trans->bp_by_tout = true;
                }
                // backpress by diff_type timeout
                if (has_timeout_rd && trans->transactionType == DATA_WRITE) trans->bp_by_tout = true;
                else if (has_timeout_wr && trans->transactionType == DATA_READ) trans->bp_by_tout = true;
            }
        }
        else if (QOS_POLICY == 2 && has_tout_cmd && (trans->pri < tout_high_pri || (!trans->timeout && trans->pri == tout_high_pri))) {
            trans->bp_by_tout = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC Backpress by tout :: task="<<trans->task<<" type="<<trans->transactionType
                        <<" qos="<<trans->qos<<" pri="<<trans->pri<<" tout_high_pri="<<tout_high_pri<<" has_timeout_rd="<<has_timeout_rd
                        <<" has_timeout_wr="<<has_timeout_wr<<endl);
            }
        }
    }
}

/***************************************************************************************************
descriptor: The purpose of this function is to Calculate burst length of DDR command
****************************************************************************************************/
void PTC::CalcBl(Transaction *t) {
    // Calc DDR command BL
    if (t->transactionType == DATA_READ) {
        t->trans_size = max_bl_data_size - t->addr_col % max_bl_data_size;
        if (t->data_size - t->issue_size < t->trans_size) t->trans_size = t->data_size - t->issue_size;
    } else {
        // address aligned check for lpddr
        if ((t->addr_col % min_bl_data_size != 0) && (IS_LP5 || IS_LP6 || IS_LP4)){
            ERROR(setw(10)<<now()<<" -- Address not 32B aligned, task="<<t->task);
            assert(0);
        }
        // Dif between data_size and issue_size check for lpddr
        if ((t->data_size - t->issue_size < min_bl_data_size) && (IS_LP5 || IS_LP6 || IS_LP4)){
            ERROR(setw(10)<<now()<<" -- Issue size wrong, task="<<t->task<<" data_size="<<t->data_size<<" issue_size="<<t->issue_size<<" min_bl_data_size="<<min_bl_data_size);
            assert(0);
        }

        if (t->mask_wcmd && !IS_LP6) {                                     //mask write condition, addr_col 32B aligned, up to mask_wcmd
            if (t->issue_size == 0) {                                      //first must be BL16 mask write
                t->trans_size = min_bl_data_size;
            } else {  
                for (auto it = bl_data_size.end(); it != bl_data_size.begin();) { // From max BL to min BL
                    it --;
                    if ((t->data_size - t->issue_size) >= it->second && t->addr_col % it->second == 0) {
                        t->trans_size = it->second;
                        break;
                    }
                }
            } 
        } else {                                                           //mask write original condition, addr_col not 32B aligned 
            if (t->data_size - t->issue_size < min_bl_data_size) {
                t->trans_size = t->data_size - t->issue_size;
            } else if (t->addr_col % min_bl_data_size == 0) {  
                for (auto it = bl_data_size.end(); it != bl_data_size.begin();) { // From max BL to min BL
                    it --;
                    if ((t->data_size - t->issue_size) >= it->second && t->addr_col % it->second == 0) {
                        t->trans_size = it->second;
                        break;
                    }
                }
            } else {
                t->trans_size = min_bl_data_size - t->addr_col % min_bl_data_size;
            }
        }
    }
    
    if (t->mask_wcmd && !IS_LP6) {  //todo: 0802  
        if (t->issue_size == 0) {   
            uint8_t bl_map_size = MAP_CONFIG["BL"].size();
            unsigned bl_min = MAP_CONFIG["BL"][bl_map_size - 1];
            t->bl = bl_min;
        } else {
            for (auto it = bl_data_size.begin(); it != bl_data_size.end(); it ++) { // From min BL to max BL
                if (t->addr_col % it->second + t->trans_size <= it->second) {
                    t->bl = it->first;
                    break;
                }
            }
        } 
    } else {
        for (auto it = bl_data_size.begin(); it != bl_data_size.end(); it ++) { // From min BL to max BL
            if (t->addr_col % it->second + t->trans_size <= it->second) {
                t->bl = it->first;
                break;
            }
        }
    }
}

/***************************************************************************************************
descriptor: The purpose of this function is to determine what kind of command to send next.
****************************************************************************************************/
void PTC::need_issue(Transaction *trans) {
    if (bankStates[trans->bankIndex].state->currentBankState == RowActive) {
        uint32_t &openRow = bankStates[trans->bankIndex].state->openRowAddress;
        if (trans->row == openRow) {
            if (rw_exec_cnt < EXEC_NUMBER || trans->issue_size != 0 || EXEC_UNLIMIT_EN) {
                CalcBl(trans);
                bool row_hit = false;
                if ((trans->issue_size + trans->trans_size) < trans->data_size) {
                    row_hit = true;
                } else {
                    for (auto &t : transactionQueue) {
                        if (t->task == trans->task) continue;
                        if (t->bankIndex != trans->bankIndex) continue;
                        if (trans->row != t->row) continue;
                        row_hit = true;
                        break;
                    }
                    if (bankStates[trans->bankIndex].perf_rd_rowhit || bankStates[trans->bankIndex].perf_wr_rowhit) {   // check PERF has rowhit cmd 
                        row_hit = true;
                    }
                }


                if (trans->transactionType == DATA_READ) {
                    if (RD_APRE_EN || trans->ap_cmd || page_timeout_rd[trans->bankIndex] == 0) {
                        if (row_hit) {
                            trans->nextCmd = READ_CMD;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- nextCmd = READ_CMD, task="<<trans->task<<endl);
                            }
                        }
                        else {
                            trans->nextCmd = READ_P_CMD;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- nextCmd = READ_P_CMD, task="<<trans->task<<endl);
                            }
                        }
                    } else if (ENHAN_RD_AP_EN && !row_hit) {
                        if (bank_cnt[trans->bankIndex] <= 1) trans->nextCmd = READ_CMD;
                        else trans->nextCmd = READ_P_CMD;
                    } else {
                        trans->nextCmd = READ_CMD;
                    }
                } else if (trans->transactionType == DATA_WRITE) {
                    bool is_mask = false;

                    // write address in DMC must be aligned, WRITE_MASK_CMD not allowed
                    if (IS_LP6){
                        is_mask = false;
                    } else {
                        if (trans->mask_wcmd) {
                            is_mask = (trans->issue_size==0);          //first wcmd with mask flag must be BL16 mask write
                        } else {
                            is_mask = (trans->trans_size < (trans->bl * JEDEC_DATA_BUS_BITS / 8));
                        }
                    }
                    if (WR_APRE_EN || trans->ap_cmd || page_timeout_wr[trans->bankIndex] == 0) {
                        if (row_hit) trans->nextCmd = is_mask ? WRITE_MASK_CMD : WRITE_CMD;
                        else trans->nextCmd = is_mask ? WRITE_MASK_P_CMD : WRITE_P_CMD;
                    } else if (ENHAN_WR_AP_EN && !row_hit) {
                        if (bank_cnt[trans->bankIndex] <= 1) trans->nextCmd = WRITE_CMD;
                        else trans->nextCmd = WRITE_P_CMD;
                    } else {
                        trans->nextCmd = is_mask ? WRITE_MASK_CMD : WRITE_CMD;
                    }
                }
            } else {
                trans->nextCmd = INVALID;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- nextCmd = INVALID, task="<<trans->task<<endl);
                }
            }
        } else {  // PTC forbidden
            ERROR(setw(10)<<now()<<" -- Error Send Precharge Cmd! task="<<trans->task<<", issue_size="<<trans->issue_size
                    <<", trans_size="<<trans->trans_size<<", data_size="<<trans->data_size);
            assert(0);
            
            if (!bankStates[trans->bankIndex].hold_precharge) {
                trans->nextCmd = PRECHARGE_PB_CMD;
            }
            else trans->nextCmd = INVALID;
        }
    } else {
        if (IS_LP4 || IS_LP5 || IS_LP6) {
            if (trans->act_executing) trans->nextCmd = ACTIVATE2_CMD;
            else if (bankStates[trans->bankIndex].state->act_executing) trans->nextCmd = INVALID;
            else trans->nextCmd = ACTIVATE1_CMD;
        } else {
            trans->nextCmd = ACTIVATE2_CMD;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- nextCmd = ACTIVATE2_CMD, task="<<trans->task<<endl);
            }
        }
    }

    if (trans->nextCmd == READ_P_CMD || trans->nextCmd == WRITE_P_CMD || trans->nextCmd == WRITE_MASK_P_CMD) {
        if (trans->issue_size + trans->trans_size < trans->data_size) {
            ERROR(setw(10)<<now()<<" -- Error Send AP Cmd! task="<<trans->task<<", issue_size="<<trans->issue_size
                    <<", trans_size="<<trans->trans_size<<", data_size="<<trans->data_size);
            assert(0);
        }
    }
}

/***************************************************************************************************
descriptor: timing check
****************************************************************************************************/
void PTC::cmd_select(Transaction *t) {
    unsigned sub_channel = (t->bankIndex % NUM_BANKS) / sc_bank_num;
    unsigned bank_start = sub_channel * sc_bank_num;
    if (t->pre_act && bankStates[t->bankIndex].state->currentBankState == RowActive) {
        t->issue_size = t->data_size;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: PRE-ACT. pre active meet row active"<<", task="<<t->task<<endl);
        }
        return;
    }
    if (t->fast_rd && t->act_only && bankStates[t->bankIndex].state->currentBankState == RowActive) {
        t->issue_size = t->data_size;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: FAST-RD-ACT. fast rd active meet row active"<<", task="<<t->task<<endl);
        }
        return;
    }

    if (RankState[t->rank].lp_state != IDLE) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LP_BP_CMD :: task="<<t->task<<" is BP by lowpower mode. rank="
                    <<t->rank<<", lp_state="<<RankState[t->rank].lp_state<<endl);
        }
        return;
    }

    // todo: change for enahnced DBR?
    // for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
    //     if (force_pbr_refresh[rank][sub_channel] && SBR_REQ_MODE == 1 && t->issue_size == 0) {
    //         for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++){
    //             if ((forceRankBankIndex[rank][sub_channel] + sbr_bank * pbr_bank_num + rank * NUM_BANKS) == t->bankIndex) {
    //                 if (DEBUG_BUS) {
    //                     PRINTN(setw(10)<<now()<<" -- LC :: force the same bank request in Rank "<<rank<<", SC "<<sub_channel<<". task="<<t->task<<endl);
    //                 }
    //                 return;
    //             }
    //         }
    //     }
    // }

    bool rw_switch_trig = ((PreCmd.trans_type==DATA_READ)&&(t->transactionType==DATA_READ) && que_read_cnt < PBR_LESS_CMD_LEVEL && que_write_cnt>0)
                            || ((PreCmd.trans_type==DATA_WRITE)&&(t->transactionType==DATA_WRITE) && que_write_cnt < PBR_LESS_CMD_LEVEL && que_read_cnt>0);
    bool  pbr_block_en = PBR_LESS_CMD_EN && (!rw_switch_trig && PBR_LESS_CMD_MODE==0);

    if (((refreshPerBank[t->bankIndex].refreshWaiting && (!PBR_LESS_CMD_EN || pbr_block_en)) || refreshPerBank[t->bankIndex].refreshing)
            && t->issue_size == 0 && !t->act_executing) {
        if (t->transactionType == DATA_READ) rd_met_pbr_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: this bank is refreshing. bank="<<t->bankIndex<<" sc="<<sub_channel<<" task="
                    <<t->task<<" postpnd="<<refreshALL[t->rank][sub_channel].refresh_cnt<<" refreshWaiting="
                    <<refreshPerBank[t->bankIndex].refreshWaiting<<" refreshing="
                    <<refreshPerBank[t->bankIndex].refreshing<<endl);
        }
        return;
    }

    if ((refreshALL[t->rank][sub_channel].refreshWaiting || refreshALL[t->rank][sub_channel].refreshing)
            && t->issue_size == 0 && !t->act_executing) {
        if (t->transactionType == DATA_READ) rd_met_abr_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: this rank is refreshing. rank="<<t->rank<<", sc="<<sub_channel<<" task="
                    <<t->task<<" refreshWaiting="<<refreshALL[t->rank][sub_channel].refreshWaiting
                    <<" refreshing="<<refreshALL[t->rank][sub_channel].refreshing<<endl);
        }
        return;
    }

    need_issue(t);

    if (dresp_cnt >= DRESP_BP_TH && t->nextCmd >= WRITE_CMD && t->nextCmd <= READ_P_CMD
            && t->issue_size == 0) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: dresp backpress ing---. dresponse counter="<<dresp_cnt<<endl);
        }
        return;
    }
    
    // check cancel full fast read cmd due to address/ID conflict
    if (FAST_RD_CANCEL_EN && t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD && t->fast_rd && t->act_only) {
        ERROR(setw(10)<<now()<<" -- Rd not allowed for act_only: task="<<t->task);
        assert(0);
        
    }

    if (t->bp_by_tout && t->issue_size == 0 && !t->act_executing && 
            (bankStates[t->bankIndex].has_timeout || t->nextCmd != PRECHARGE_PB_CMD)) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Block by timeout cmd. task="<<t->task<<", bank="
                    <<t->bankIndex<<", pri="<<t->pri<<", tout_high_pri="<<tout_high_pri<<endl);
        }
        return;
    }

    if (t->transactionType != DATA_READ && t->nextCmd != ACTIVATE1_CMD &&
            t->nextCmd != ACTIVATE2_CMD && t->nextCmd != PRECHARGE_PB_CMD) {
        if (t->data_ready_cnt <= t->burst_length) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: write data not ready. cnt="<<t->data_ready_cnt
                        <<" issue_size="<<t->issue_size<<" task="<<t->task<<endl);
            }
            return;
        }
    }

    //simple rd/wr group in PTC, mode 0
    bool has_hqos = (t->qos >= PERF_SWITCH_HQOS_LEVEL || (t->pri >= PERF_SWITCH_HQOS_LEVEL && t->hqos_push)); 
    if (SIMPLE_RWGRP_EN && SIMPLE_RWGRP_MODE == 0 && (!has_hqos) ) {  
        if (!t->timeout && t->issue_size == 0 && (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) && (que_write_cnt > 0)) {
            if (PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Write group backpress a read command. task="
                            <<t->task<<", nextCmd="<<t->nextCmd<<" rank="<<t->rank<<" qos="<<t->qos<<" pri="<<t->pri<<" hqos_push="<<t->hqos_push<<" rank="<<t->rank<<" que_read_cnt"<<que_read_cnt<<endl);
                }
                return;
            } 
        }
        if (!t->timeout && t->issue_size == 0 && (t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) && (que_read_cnt > 0)) {
            if (PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a write command. task="
                            <<t->task<<", nextCmd="<<t->nextCmd<<" rank="<<t->rank<<" qos="<<t->qos<<" pri="<<t->pri<<" hqos_push="<<t->hqos_push<<" rank="<<t->rank<<" que_read_cnt"<<que_read_cnt<<endl);
                }
                return;
            } 
        }
    }


    //simple rd/wr group in PTC, mode 1
    //check if early write or early read
    bool has_early_write = false;
    bool has_early_read = false;
    if (SIMPLE_RWGRP_EN && SIMPLE_RWGRP_MODE == 1) {
        for (auto &trans: transactionQueue) {
            if (t->task == trans->task) continue;
            if (t->transactionType == trans->transactionType) continue;
            if (t->ptc_timeAdded < trans->ptc_timeAdded) continue;
            if (t->transactionType == DATA_READ) has_early_write = true;
            else has_early_read = true;
        }
    }

    if (SIMPLE_RWGRP_EN && SIMPLE_RWGRP_MODE == 1) {  
        if (!t->timeout && t->issue_size == 0 && (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) && has_early_write) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: Early Write backpress a read command. task="
                        <<t->task<<", nextCmd="<<t->nextCmd<<endl);
            }
            return;
        }
        if (!t->timeout && t->issue_size == 0 && (t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) && has_early_read) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: Early Read backpress a write command. task="
                        <<t->task<<", nextCmd="<<t->nextCmd<<endl);
            }
            return;
        }
    }


    //simple rd/wr group in PTC, mode 2
    if (SIMPLE_RWGRP_EN && SIMPLE_RWGRP_MODE==2) {
        bool ptc_no_rwgrp = ((que_write_cnt > 0) && (que_read_cnt > 0) 
                                && ((pfq_->wbuff_state == WBUFF_NO_GROUP) || (perf_grpst_switch_cnt > PERF_GRPST_SWITCH_LVL1)))
                                || (que_write_cnt == 0) || (que_read_cnt == 0);
        if (!ptc_no_rwgrp) {
            if (!t->timeout && t->issue_size == 0 && (PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD) 
                    && (pfq_->wbuff_state == WBUFF_IDLE) && (perf_grpst_switch_cnt <= PERF_GRPST_SWITCH_LVL0)) {
                if (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Write group backpress a read command. task="
                                <<t->task<<", nextCmd="<<t->nextCmd<<", perf_grpst_cnt="<<perf_grpst_switch_cnt<<endl);
                    }
                    return;
                } 
            }
            if (!t->timeout && t->issue_size == 0 && (PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD) 
                    && (pfq_->wbuff_state == WBUFF_WRITE) && (perf_grpst_switch_cnt <= PERF_GRPST_SWITCH_LVL0)) {
                if (t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a write command. task="
                                <<t->task<<", nextCmd="<<t->nextCmd<<", perf_grpst_cnt="<<perf_grpst_switch_cnt<<endl);
                    }
                    return;
                } 
            }
            if (!t->timeout && t->issue_size == 0 && (PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD) 
                    && (pfq_->wbuff_state == WBUFF_WRITE) && (perf_grpst_switch_cnt > PERF_GRPST_SWITCH_LVL0) 
                    && (perf_grpst_switch_cnt <= PERF_GRPST_SWITCH_LVL1)) {
                if (t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Write group backpress a write command. task="
                                <<t->task<<", nextCmd="<<t->nextCmd<<", perf_grpst_cnt="<<perf_grpst_switch_cnt<<endl);
                    }
                    return;
                } 
            }
            if (!t->timeout && t->issue_size == 0 && (PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD) 
                    && (pfq_->wbuff_state == WBUFF_IDLE) && (perf_grpst_switch_cnt > PERF_GRPST_SWITCH_LVL0)
                    && (perf_grpst_switch_cnt <= PERF_GRPST_SWITCH_LVL0)) {
                if (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a read command. task="
                                <<t->task<<", nextCmd="<<t->nextCmd<<", perf_grpst_cnt="<<perf_grpst_switch_cnt<<endl);
                    }
                    return;
                } 
            }
        }
    }


    //simple rd/wr group in PTC, mode 3
    if (SIMPLE_RWGRP_EN && SIMPLE_RWGRP_MODE==3) {
        bool ptc_nogrp = ((que_write_cnt > 0) && (que_read_cnt > 0) && (pfq_->wbuff_state == WBUFF_NO_GROUP))
                                || (que_write_cnt == 0) || (que_read_cnt == 0);
        if (!ptc_nogrp) {
            if (!t->timeout && t->issue_size == 0 && (PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD) && (pfq_->wbuff_state == WBUFF_IDLE)) {
                if (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Write group backpress a read command. task="
                                <<t->task<<", nextCmd="<<t->nextCmd<<endl);
                    }
                    return;
                } 
            }
            if (!t->timeout && t->issue_size == 0 && (PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD) && (pfq_->wbuff_state == WBUFF_WRITE)) {
                if (t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a write command. task="
                                <<t->task<<", nextCmd="<<t->nextCmd<<endl);
                    }
                    return;
                } 
            }

            bool hold_wcmd = (pfq_->wbuff_state == WBUFF_WRITE) && (PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD) 
                                && (t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) && (t->rank == PreCmd.rank) 
                                && (PreCmdTime!=0xffffffff) && (PreCmdTime > PTC_W2R_SWITCH_TH);
                               
            if (!t->timeout && t->issue_size == 0 && hold_wcmd) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Write group backpress a write command. task="
                            <<t->task<<", nextCmd="<<t->nextCmd<<endl);
                }
                return;
            }

            bool hold_rcmd = (pfq_->wbuff_state == WBUFF_IDLE) && (PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD) 
                                && (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) && (t->rank == PreCmd.rank) 
                                && (PreCmdTime!=0xffffffff) && (PreCmdTime > PTC_R2W_SWITCH_TH);

            if (!t->timeout && t->issue_size == 0 && hold_rcmd) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a read command. task="
                            <<t->task<<", nextCmd="<<t->nextCmd<<endl);
                }
                return;
            }
        }
    }

    
    bool same_cmdtype = (((t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD) && (PreCmd.type >= WRITE_CMD && PreCmd.type <= WRITE_MASK_P_CMD)) ||
                        ((t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD) && (PreCmd.type >= READ_CMD && PreCmd.type <= READ_P_CMD)));
    
    // force rank switch && hqos force rank switch
    bool force_rank_switch = false;
    if (((PreCmdTime > PTC_RSCH_GAP_CNT && t->transactionType == DATA_READ) 
            || (PreCmdTime > PTC_WSCH_GAP_CNT && t->transactionType == DATA_WRITE)) 
            && (PreCmdTime != 0xffffffff) && (PreCmd.rank == t->rank) && (PreCmd.trans_type == t->transactionType)) {
        force_rank_switch = true;
    }

    // perf hqos force rank switch flag
    bool perf_hqos_rank_miss = (t->rank != pfq_->hqos_rank && pfq_->hqos_rank != 0xffffffff);
    bool perf_hqos_rank_hit  = (t->rank == pfq_->hqos_rank);
    bool perf_hqos_rank_exclude = ((t->transactionType == DATA_WRITE) || (t->transactionType == DATA_READ && !perf_hqos_rank_hit)) 
                                    || (!PERF_RCMD_HQOS_RANK_SWITCH_EN);

    //ptc hqos force rank switch flag
    bool ptc_hqos_rank_miss = (t->rank != hqos_rank && hqos_rank != 0xffffffff); 
    bool ptc_hqos_rank_hit  = (t->rank == hqos_rank);
    bool ptc_hqos_rank_exclude = ((t->transactionType == DATA_WRITE) || (t->transactionType == DATA_READ && !ptc_hqos_rank_hit)) 
                                    || (!PTC_HQOS_RANK_SWITCH_EN) || (PTC_HQOS_RANK_SWITCH_MODE == 0);

    //force rank switch seperatly in rd/wr group
    if (PTC_RANK_SWITCH_EN && !t->timeout && t->issue_size == 0 && perf_hqos_rank_exclude 
            && ptc_hqos_rank_exclude && same_cmdtype && force_rank_switch && has_other_rank_cmd) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Force Rank Switch backpress a command. task="
                    <<t->task<<", nextCmd="<<t->nextCmd<<", rank="<<t->rank<<", pre_cmd_rank="<<PreCmd.rank
                    <<", has_other_rank_cmd="<<has_other_rank_cmd<<", force_rank_switch="<<force_rank_switch
                    <<", perf_hqos_rank="<<pfq_->hqos_rank<<", ptc_hqos_rank="<<hqos_rank<<endl);
        }
        return;
    }
    
    // perf hoqs force rank switch
    if (PERF_RCMD_HQOS_RANK_SWITCH_EN && !t->timeout && t->issue_size == 0 && perf_hqos_rank_miss 
            && has_hqos_rank_rcmd && (t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD)) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Perf Hqos Force Rank Switch backpress a command. task="
                    <<t->task<<", nextCmd="<<t->nextCmd<<", rank="<<t->rank<<", perf_has_hqos_rank_rcmd="
                    <<has_hqos_rank_rcmd<<", hqos_rank="<<pfq_->hqos_rank<<endl);
        }
        return;
    }

    // ptc hoqs force rank switch
    if (PTC_HQOS_RANK_SWITCH_EN && !t->timeout && t->issue_size == 0 && ptc_hqos_rank_miss) {
        if ((PTC_HQOS_RANK_SWITCH_MODE == 1 && t->nextCmd >= WRITE_CMD && t->nextCmd <= READ_P_CMD)
                || (PTC_HQOS_RANK_SWITCH_MODE == 2 && t->nextCmd >= READ_CMD && t->nextCmd <= READ_P_CMD)
                || (PTC_HQOS_RANK_SWITCH_MODE == 3 && t->nextCmd >= WRITE_CMD && t->nextCmd <= READ_P_CMD)) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: Ptc Hqos Force Rank Switch backpress a diff rank rd/wr command. task="
                        <<t->task<<", nextCmd="<<t->nextCmd<<", rank="<<t->rank<<", perf_has_hqos_rank_rcmd="
                        <<has_hqos_rank_rcmd<<", ptc_hqos_rank="<<hqos_rank<<endl);
            }
            return;
        }
    }
    //it may decrease perf
    if (PTC_HQOS_RANK_SWITCH_EN && !t->timeout && t->issue_size == 0 && ptc_hqos_rank_hit) {
        if ((PTC_HQOS_RANK_SWITCH_MODE == 3 && t->nextCmd >= WRITE_CMD && t->nextCmd <= WRITE_MASK_P_CMD && !t->hqos_push)) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: Ptc Hqos Force Rank Switch backpress a same rank write command. task="
                        <<t->task<<", nextCmd="<<t->nextCmd<<", rank="<<t->rank<<", perf_has_hqos_rank_rcmd="
                        <<has_hqos_rank_rcmd<<", ptc_hqos_rank="<<hqos_rank<<"rank0_ptc_hqos_rcmd_cnt="<<que_read_highqos_cnt[0]<<endl);
            }
            return;
        }
    }

    // samerank precharge blocked before pbr precharge sent
    if (!IS_DDR5 && t->nextCmd == PRECHARGE_PB_CMD) {   //todo: revise for e-mode
        for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
            unsigned bank_tmp = t->rank * NUM_BANKS + bank;
            if (refreshPerBank[bank_tmp].refreshWaiting && !refreshPerBank[bank_tmp].refreshWaitingPre) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PBR hold PRE command. task="<<t->task<<endl);
                }
                return;
            }
        }
    }


    if (t->nextCmd == INVALID) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Invalid command. task="<<t->task<<endl);
        }
        return;
    }

    bool timing_met = false;
    switch (t->nextCmd) {
        case READ_CMD : {
            if (has_cmd_bp()) break;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: READ timing, bank="<<t->bankIndex<<", task="<<t->task<<", state->nextRead="<<bankStates[t->bankIndex].state->nextRead<<endl);
            }
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextRead) {
                timing_met = true;
                if (t->issue_size == 0) cmd_rdmet_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: READ timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case READ_P_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextReadAp) {
                timing_met = true;
                if (t->issue_size == 0) cmd_rdmet_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: READ_P timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case WRITE_CMD : {
            if (has_cmd_bp()) break;
            if (IS_DDR5) {
                if (DDR_MODE == "_x4" && t->bl == BL16) {
                    // next command is RMW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteRmw) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE RMW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else if ((DDR_MODE == "_x4" && t->bl == BL32) || ((DDR_MODE == "_x8" || DDR_MODE == "_x16") && t->bl == BL16)) {
                    // next command is JW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWrite) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE JW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else {
                    ERROR(setw(10)<<now()<<" -- Error DDR Mode: "<<DDR_MODE<<", BL="<<t->bl);
                    assert(0);
                }
            } else {
                if ((now() + 1) >= bankStates[t->bankIndex].state->nextWrite) {
                    timing_met = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: WRITE timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                    }
                }
            }
            break;
        }
        case WRITE_P_CMD : {
            if (has_cmd_bp()) break;
            if (IS_DDR5) {
                if (DDR_MODE == "_x4" && t->bl == BL16) {
                    // next command is RMW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteApRmw) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE_P RMW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else if ((DDR_MODE == "_x4" && t->bl == BL32) || ((DDR_MODE == "_x8" || DDR_MODE == "_x16") && t->bl == BL16)) {
                    // next command is JW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteAp) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE_P JW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else {
                    ERROR(setw(10)<<now()<<" -- Error DDR Mode: "<<DDR_MODE<<", BL="<<t->bl);
                    assert(0);
                }
            } else {
                if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteAp) {
                    timing_met = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: WRITE_P timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                    }
                }
            }
            break;
        }
        case WRITE_MASK_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteMask) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: WRITE_MASK timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case WRITE_MASK_P_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteMaskAp) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: WRITE_MASK_P timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case PRECHARGE_PB_CMD : {
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextPrecharge && tFPWCountdown[t->rank].size() < 4) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PRECHARGE_PB timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case PRECHARGE_SB_CMD : {
            uint8_t met_cnt = 0;
            for (size_t i = 0; i < pbr_bg_num; i ++) {
                uint32_t bank = t->rank * NUM_BANKS + i * pbr_bank_num + t->bankIndex % pbr_bank_num;
                if ((now() + 1) >= bankStates[bank].state->nextPrecharge) met_cnt ++;
            }
            if (met_cnt == pbr_bg_num && tFPWCountdown[t->rank].size() < 4) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PRECHARGE_SB timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case PRECHARGE_AB_CMD : {   //todo: revise for e-mode
            uint8_t met_cnt = 0;
            for (size_t i = 0; i < NUM_BANKS/sc_num; i ++) {
                uint32_t bank = i + t->rank * NUM_BANKS + bank_start;
                if ((now() + 1) >= bankStates[bank].state->nextPrecharge) met_cnt ++;
            }
            if ((met_cnt == NUM_BANKS/sc_num) && tFPWCountdown[t->rank].size() < 4) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PRECHARGE_AB timing met, bank="<<t->bankIndex<<", sc="<<sub_channel<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case ACTIVATE1_CMD : {
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextActivate1) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: ACTIVE1 timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case ACTIVATE2_CMD : {
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextActivate2 && ((tFAWCountdown[t->rank].size() < 4 && sub_channel==0) 
                    ||(tFAWCountdown_sc1[t->rank].size() < 4 && sub_channel==1)) && (!IS_HBM3 ||  now()%2==1 || ODD_TDM)) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: ACTIVE2 timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        default:break;
    }

    if (timing_met) {
        Cmd *c = new Cmd;
        if (((t->issue_size + t->trans_size) >= t->data_size) &&
                ((t->nextCmd != PRECHARGE_PB_CMD && t->nextCmd != ACTIVATE2_CMD))) {
            *c = Cmd(*t, false);
        } else {
            *c = Cmd(*t, true);
        }
        if(c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) {
            rw_cmdqueue.push_back(c);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- rwcmd in rw_cmdqueue, task="<<c->task<<", cmd_type="<<c->cmd_type<<endl);
            }
        }
        else act_cmdqueue.push_back(c);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- act_cmdqueue_empty="<<act_cmdqueue.empty()<<endl);
        }
        //CmdQueue.push_back(c);
    }
}

bool PTC::WillAcceptTransaction() {
    return GetDmcQsize() < TRANS_QUEUE_DEPTH;
}
/***************************************************************************************************
descriptor: check conflict,The purpose of this approach is to keep order
****************************************************************************************************/
void PTC::check_conflict(Transaction *trans) {
    if (!CONF_EN) return;
    for (auto &t : transactionQueue) {
        if (((trans->address & ~ALIGNED_SIZE) == (t->address & ~ALIGNED_SIZE))
          && ((t->transactionType != DATA_READ && trans->transactionType == DATA_READ)
          || (t->transactionType == DATA_READ && trans->transactionType != DATA_READ)
          || (t->transactionType != DATA_READ && trans->transactionType != DATA_READ))) {
            trans->addrconf = true;
            addrconf_cnt++;
            if (trans->pri < t->pri && PRIORITY_PASS_ENABLE) {
                t->pri = trans->pri;
                t->qos = trans->qos;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADD_CONF_PUSH :: push add conflict pri, src task="
                            <<trans->task<<" dst task="<<t->task<<" qos="<<trans->qos<<" pri="
                            <<trans->pri<<endl);
                }
            }
            trans->addr_block_source_id = t->task;
        }
    }
}

void PTC::ehs_page_adapt_policy() {
    if (!ENH_PAGE_ADPT_EN) return;
    if (now() % ENH_PAGE_ADPT_WIN == 0 && now() != 0) {
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            if (bank_cnt_ehs[i] >= MAP_CONFIG["ENH_PAGE_ADPT_LVL"][2]) {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][3];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][3];
                ehs_page_adapt_cnt[i/NUM_BANKS][3] ++;
            } else if (bank_cnt_ehs[i] >= MAP_CONFIG["ENH_PAGE_ADPT_LVL"][1]) {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][2];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][2];
                ehs_page_adapt_cnt[i/NUM_BANKS][2] ++;
            } else if (bank_cnt_ehs[i] >= MAP_CONFIG["ENH_PAGE_ADPT_LVL"][0]) {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][1];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][1];
                ehs_page_adapt_cnt[i/NUM_BANKS][1] ++;
            } else {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][0];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][0];
                ehs_page_adapt_cnt[i/NUM_BANKS][0] ++;
            }
            bank_cnt_ehs[i] = 0;
        }
    }
}

void PTC::page_adapt_policy(Transaction *trans) {
    if (!PAGE_ADAPT_EN) return;

    if (now() % 800 == 0 && now() != 0 && 0) {
        unsigned rowhit_ratio = 0;
        unsigned page_tout_rd = 0;
        unsigned page_tout_wr = 0;
        if (page_rw_cnt != 0) rowhit_ratio = 1000 * page_act_cnt / page_rw_cnt;

        if (rowhit_ratio >= 750) {
            page_tout_rd = 5;
            page_tout_wr = 5;
        } else if (rowhit_ratio >= 500) {
            page_tout_rd = 50;
            page_tout_wr = 50;
        } else if (rowhit_ratio >= 250) {
            page_tout_rd = 100;
            page_tout_wr = 100;
        } else {
            page_tout_rd = 200;
            page_tout_wr = 200;
        }

        for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
            page_timeout_rd[bank] = page_tout_rd;
            page_timeout_wr[bank] = page_tout_wr;
        }
        page_act_cnt = 0;
        page_rw_cnt = 0;
    }

    unsigned bank = trans->bankIndex;
    unsigned row = trans->row;
    for (uint32_t i = 0; i < sizeof(page_timeout_window)/sizeof(page_timeout_window[0]); i ++) {
        if (row == bankStates[bank].last_activerow) {
            if (bank_cas_delay[bank] < page_timeout_window[i]) page_row_hit[bank][i] ++;
            else page_row_miss[bank][i] ++;
        } else {
            uint32_t gap;
            gap = (bank_cas_delay[bank] < tRPpb) ? 0 : bank_cas_delay[bank] - tRPpb;
            if (gap < page_timeout_window[i]) page_row_conflict[bank][i] ++;
            else page_row_miss[bank][i] ++;
        }
    }

    page_cmd_cnt[bank] ++;
    if ((trans->transactionType == DATA_READ && page_cmd_cnt[bank] >= OPENPAGE_TIME_RD) ||
            (trans->transactionType == DATA_WRITE && page_cmd_cnt[bank] >= OPENPAGE_TIME_WR)) {
        uint32_t index = 0;
        int32_t best_timeout = page_row_hit[bank][0] - page_row_conflict[bank][0];

        for (uint32_t i = 1; i < sizeof(page_timeout_window)/sizeof(page_timeout_window[0]); i ++) {
            if ((page_row_hit[bank][i] - page_row_conflict[bank][i]) > best_timeout) {
                best_timeout = page_row_hit[bank][i] - page_row_conflict[bank][i];
                index = i;
            }
        }

        if (trans->transactionType == DATA_READ) page_timeout_rd[bank] = page_timeout_window[index];
        else page_timeout_wr[bank] = page_timeout_window[index];

        for (uint32_t i = 0; i < sizeof(page_timeout_window)/sizeof(page_timeout_window[0]); i ++) {
            page_row_hit[bank][i] = 0;
            page_row_miss[bank][i] = 0;
            page_row_conflict[bank][i] = 0;
        }
        page_cmd_cnt[bank] = 0;
    }
}

void PTC::page_adpt_policy(Transaction *trans) {
    if (!PAGE_ADPT_EN) return;

    if (PAGE_WIN_MODE == 1) page_adpt_win_cnt ++;

    auto &state = bankStates[trans->bankIndex].state;
    // Has no same bank command, bank open, row miss, precharge timing met
    // Overdue page close, add opc
    if (bank_cnt[trans->bankIndex] == 0 && state->currentBankState == RowActive &&
            state->openRowAddress != trans->row && now() >= state->nextPrecharge) {
        opc_cnt ++;
        if (PAGE_WIN_MODE == 0) page_adpt_win_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- OPC_ADD, opc_cnt="<<opc_cnt<<", ppc_cnt="
                    <<ppc_cnt<<", task="<<trans->task<<", bank="<<trans->bankIndex
                    <<", adpt_openpage_time="<<adpt_openpage_time<<endl);
        }
    }
    // Has no same bank command, bank not open, row hit with last row
    // Premature page close, add ppc
    if (bank_cnt[trans->bankIndex] == 0 && state->currentBankState != RowActive && state->lastRow == trans->row
            && (!DMC_V596 || state->lastCmdSource != 2)) {
        ppc_cnt ++;
        if (PAGE_WIN_MODE == 0) page_adpt_win_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PPC_ADD, opc_cnt="<<opc_cnt<<", ppc_cnt="
                    <<ppc_cnt<<", task="<<trans->task<<", bank="<<trans->bankIndex
                    <<", adpt_openpage_time="<<adpt_openpage_time<<endl);
        }
    }

    if (page_adpt_win_cnt == PAGE_WIN_SIZE) {
        if (ppc_cnt >= PAGE_PPC_TH && opc_cnt < PAGE_OPC_TH) {
            if (adpt_openpage_time >= PAGE_TIME_MAX) adpt_openpage_time = PAGE_TIME_MAX;
            else adpt_openpage_time += PAGE_ADPT_STEP;
        } else if (ppc_cnt < PAGE_PPC_TH && opc_cnt >= PAGE_OPC_TH) {
            if (adpt_openpage_time < PAGE_ADPT_STEP) adpt_openpage_time = 0;
            else adpt_openpage_time -= PAGE_ADPT_STEP;
        }

        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            page_timeout_rd[i] = adpt_openpage_time;
            page_timeout_wr[i] = adpt_openpage_time;  
        }

        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WIN_EXCEED, page_adpt_win_cnt="<<page_adpt_win_cnt
                    <<", opc_cnt="<<opc_cnt<<", ppc_cnt="<<ppc_cnt<<", adpt_openpage_time="
                    <<adpt_openpage_time<<endl);
        }
        opc_cnt = 0;
        ppc_cnt = 0;
        page_adpt_win_cnt = 0;
    }
}

void PTC::update_deque_fifo() {
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        if (LP_WAKEUP_MODE==0) {
            deqCmdWakeupLp[i].push_back(pfq_->rank_cnt[i] + pfq_->rel_rank_cnt[i]);
        } else if (LP_WAKEUP_MODE==1) {
            deqCmdWakeupLp[i].push_back(rank_cnt[i]);
        } else if (LP_WAKEUP_MODE==2) {
            deqCmdWakeupLp[i].push_back(pfq_->rb_rank_cnt[i] + pfq_->rrel_rank_cnt[i]);
        }
        deqCmdWakeupLp[i].pop_front();
    }
    
}

void PTC::pushQosForSameMpamTrans(Transaction *trans) {
    for (auto &trans_t : transactionQueue) {
        if (trans_t->mpam_id == trans->mpam_id) {
            if (trans->pri < trans_t->pri && trans_t->transactionType == DATA_READ) trans_t->pri = trans->pri;
        }
    }
}

void PTC::pushQosForSameMidTrans(Transaction *trans) {
    for (auto &trans_t : transactionQueue) {
        if (trans_t->mid == trans->mid) {
            if (trans->pri < trans_t->pri && trans_t->transactionType == DATA_READ) trans_t->pri = trans->pri;
        }
    }
}

void PTC::noc_read_inform(bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty) {
    bool wakeup[2];
    wakeup[0] = fast_wakeup_rank0;
    wakeup[1] = fast_wakeup_rank1;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        fast_wakeup[rank] = wakeup[rank];
        if (FASTWAKEUP_EN && !PREDICT_FASTWAKEUP) fast_wakeup_cnt[rank] += int(wakeup[rank]);
    }
    if (DEBUG_BUS) {
        if (fast_wakeup_rank0 || fast_wakeup_rank1) {
            PRINTN(setw(10)<<now()<<" -- Fast Wakeup, rank0="<<fast_wakeup_rank0<<", rank1="<<fast_wakeup_rank1<<endl);
        }
    }
}

//allows outside source to make request of memory system
bool PTC::addTransaction(Transaction *trans, bool fastread) {

    // if (PERFECT_DMC_EN) {
    //     if (trans->transactionType == DATA_READ) {
    //         for (size_t i = 0; i <= trans->burst_length; i ++) {
    //             unsigned cnt = i % 2 + 1;
    //             gen_rdata(trans->task, cnt, PERFECT_DMC_DELAY, trans->mask_wcmd);
    //         }
    //         push_pending_TransactionQue(trans);
    //     }
    //     return true;
    // }

    if (dropPreAct(trans)) {
        return true;
    }

    unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
    auto &state = bankStates[trans->bankIndex];

    // PTC bypass active: perf add -> ptc add

    bool fast_read_req = ((!get_bs_full(trans))&&(state.state->currentBankState==Idle && now()>=state.state->nextActivate1)                                                 // bank closed
                       && (!refreshPerBank[trans->bankIndex].refreshWaiting && !refreshALL[trans->rank][sub_channel].refreshWaiting)                                        // not refresh bank pairs
                       && (!refreshPerBank[trans->bankIndex].refreshing && !refreshALL[trans->rank][sub_channel].refreshing)                                                // not refresh bank pairs
                       && ((!state.perf_bankrd_conflict && state.ptc_samebank_rd==0 && state.ptc_samebank_wr==0 && FAST_RD_CANCEL_MODE==0)                                  // not same bank cmd in perf/ptc mode 0
                       || (!state.perf_bankrd_conflict && state.ptc_samebank_rd==0 && !state.perf_bankwr_conflict && state.ptc_samebank_wr==0 && FAST_RD_CANCEL_MODE==1))); // not same bank cmd in perf/ptc mode 1

    if (FAST_READ_EN  && trans->transactionType==DATA_READ && fast_read_req && fastread) {
        Transaction *fast_rtrans = new Transaction(trans);
        fast_rtrans->fast_rd = true;
        fastrd_fifo.push_back(fast_rtrans);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PTC : Fast Read Generate, task="<<fast_rtrans->task<<", rank="<<fast_rtrans->rank
                    <<", bankIndex="<<fast_rtrans->bankIndex<<", fast_rd="<<fast_rtrans->fast_rd<<" Bank slot idle="<<get_bs_full(trans)<<endl);
        }
    }

    //hqos push cmd in ptc
    if (FAST_HQOS_PUSH_EN &&  !trans->pre_act && !trans->fast_rd &&  trans->transactionType == DATA_READ && trans->qos >= PERF_SWITCH_HQOS_LEVEL && fastread ) {
        for (auto &t: transactionQueue) {
            if (t->bankIndex != trans->bankIndex) continue;
            if (t->issue_size > 0) continue;
            if (t->timeout) continue;
            if (t->transactionType == DATA_READ) {
                if (FAST_HQOS_PUSH_MODE == 0)       t->pri = PTC_QOS_MAX - 1;
                else if (FAST_HQOS_PUSH_MODE == 1)  {t->pri = PTC_QOS_MAX;     t->timeout   = true;}
                else if (FAST_HQOS_PUSH_MODE == 2)  {t->pri = PTC_QOS_MAX - 1; t->hqos_push = true;}
                else if (FAST_HQOS_PUSH_MODE == 3)  {t->pri = PTC_QOS_MAX;     t->hqos_push = true;}
                     
            } else {
                if(FAST_HQOS_PUSH_MODE == 3) {t->pri = PTC_QOS_MAX; t->hqos_push = true;} 
                else {t->pri = PTC_QOS_MAX;  t->timeout = true;}
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PTC : Fast Hqos Push, task="<<t->task<<", type="<<t->transactionType<<" pri="<<t->pri
                        <<", rank="<<t->rank<<", bankIndex="<<t->bankIndex<<", hqos_task="<<trans->task
                        <<", rank="<<trans->rank<<", bankIndex="<<trans->bankIndex<<endl);
            }
        } 
    }
  
    //update bankState info by  fast read path
    if (!trans->pre_act && fastread) {
        if ((trans->transactionType == DATA_READ  && pfq_->wbuff_state == WBUFF_IDLE)   
        ||  (trans->transactionType == DATA_WRITE && pfq_->wbuff_state == WBUFF_WRITE) 
        ||  PTC_BYP_UPDATE_EN) {
            if (state.state->openRowAddress == trans->row) {
                if (trans->transactionType == DATA_READ) state.perf_rd_rowhit = true;
                else state.perf_wr_rowhit = true;
            } else {
                if (trans->transactionType == DATA_READ) state.perf_rd_rowmiss = true;
                else state.perf_wr_rowmiss = true;
            }
            
            //record same_bank cnt in perf by fast read path
            if (trans->transactionType == DATA_READ) {
                state.perf_bankrd_conflict = true;
                state.samebank_rcnt ++;
            } else if (trans->transactionType == DATA_WRITE) {
                state.perf_bankwr_conflict = true; 
                state.samebank_wcnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- same_bank_wcnt++ task="<<trans->task<<endl);
                }
            }
        }
        if (IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS)) total_iecc_cnt ++;
        else total_noiecc_cnt ++;
    }

    if (fastread) {
        DmcTotalBytes += trans->data_size;
        if (trans->transactionType == DATA_READ) DmcTotalReadBytes += trans->data_size;
        else DmcTotalWriteBytes += trans->data_size;
    
        if (!(IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS))
                && trans->transactionType == DATA_READ && !trans->pre_act) {
            if (pending_TransactionQue.size() >= 1000) {
                ERROR(setw(10)<<now()<<" -- DMC["<<getID()<<"] pending_TransactionQue size is too big");
                assert(0);
            }
            if (pending_TransactionQue.find(trans->task) != pending_TransactionQue.end()) {
                ERROR(setw(10)<<now()<<" -- [DMC"<<getID()<<"] should be error, task="<<trans->task);
                assert(0);
            }
            push_pending_TransactionQue(trans);
        }
        return true;
    }

    if (pre_req_time == now() && !fastread) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- One Cycle One Cmd, pre_req_time="<<pre_req_time<<endl);
        }
        return false;
    }

    if (((trans->timeout && trans->dummy_timeout) || trans->dummy_hqos) && !fastread) {
        bankStates[trans->bankIndex].has_dummy_tout = true;
        pre_req_time = now();
        if (trans->dummy_timeout) {
            dummy_timeout_cnt ++;
        } else if (trans->dummy_hqos) {
            dummy_hqos_cnt ++;
        }
        if (DEBUG_BUS) {
            if (trans->dummy_timeout) {  // dummy timeout
                PRINTN(setw(10)<<now()<<" -- ADD_DUMMY_TOUT :: type="<<trans->transactionType<<" addr="<<hex
                        <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                        <<trans->row<<" timeout="<<trans->timeout<<" dummy_tout="<<trans->dummy_timeout
                        <<" dummy_hqos="<<trans->dummy_hqos<<endl);
            } else if (trans->dummy_hqos) {  // dummy hqos
                PRINTN(setw(10)<<now()<<" -- ADD_DUMMY_HQOS :: type="<<trans->transactionType<<" addr="<<hex
                        <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                        <<trans->row<<" timeout="<<trans->timeout<<" dummy_tout="<<trans->dummy_timeout
                        <<" dummy_hqos="<<trans->dummy_hqos<<endl);
            }
        }
        for (auto &t : transactionQueue) {
            if (trans->rank == t->rank && trans->bankIndex == t->bankIndex) {
                if (trans->dummy_timeout) {  // dummy timeout
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- DUMMY_TOUT_PUSH :: task="<<t->task<<hex<<" address="<<t->address<<dec
                                <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" qos="<<t->qos<<" pri="<<t->pri
                                <<" dummy_task="<<trans->task<<" dummy_bank="<<trans->bankIndex<<" dummy_row="<<trans->row
                                <<" dummy_qos="<<trans->qos<<" dummy_pri="<<trans->pri<<endl);
                    }
                    if (!t->timeout) {
                        t->dummy_push = true;
                    }
                    t->pri = PTC_QOS_MAX;
                    t->timeout = true;
                } else if (trans->dummy_hqos) {  // dummy hqos
                    if (!t->timeout) {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- DUMMY_HQOS_PUSH :: task="<<t->task<<hex<<" address="<<t->address<<dec
                                    <<" type="<<t->transactionType<<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row
                                    <<" qos="<<t->qos<<" pri="<<t->pri<<" dummy_task="<<trans->task<<" dummy_bank="<<trans->bankIndex
                                    <<" dummy_row="<<trans->row<<" dummy_qos="<<trans->qos<<" dummy_pri="<<trans->pri<<endl);
                        }
                        if (t->transactionType == DATA_WRITE) {
                            t->timeout = true;
                            t->pri = PTC_QOS_MAX;
                        } else {
                            t->pri = PTC_QOS_MAX - 1;
                        }
                    }
                }
            }
        }
        return false;
    }

    if (!full() && !fastread) {
        if(BSC_EN){
            bool in_slot = false;
            for(size_t i=0;i<bank_slot.size();i++){
                if(trans->bankIndex == bank_slot[i]){
                    in_slot = true;
                } 
            }
            if(get_bs_full(trans)){
                if(DEBUG_BUS){
                    PRINTN(setw(10)<<now()<<" -- BANK SLOT already reached the BS_BP_LVL:: "<<" bankIndex="
                            <<trans->bankIndex<<" qos="<<trans->qos<<" pri="<<trans->pri<<" row="<<trans->row
                            <<" type="<<trans->transactionType<<" rank="<<trans->rank<<" task="<<trans->task<<"\n");
                    for(unsigned i=0;i<bank_slot.size();i++){
                        if(i!=0 && i%4 == 0){
                            PRINTN("    ");
                        }
                        PRINTN(bank_slot[i]<<" ");
                    }
                    PRINTN(endl);
                }
                return false; 
            }else{
                for(size_t i=0;i<bank_slot.size();i++){
                    if(bank_slot[i] == 0xFFFF){
                        bank_slot[i] = (in_slot)? bank_slot[i]:trans->bankIndex ;
                        bankStates[trans->bankIndex].row = trans->row;
                        break;
                    } 
                } 
            }
            if(DEBUG_BUS){
                PRINTN(setw(10)<<now()<<" -- BANK SLOT after addTransaction:: ");
                for(unsigned i=0;i<bank_slot.size();i++){
                    if(i!=0 && i%4 == 0){
                        PRINTN("    ");
                    }
                    PRINTN(bank_slot[i]<<" ");
                }
                PRINTN(endl);
            }
        }
       
        auto &state = bankStates[trans->bankIndex];
        trans_state_init(trans);
        if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- perf_qos="<<trans->perf_qos<<" pri="<<trans->pri<<" arb_time="<<trans->arb_time<<endl);
        }

        // no e-mode : only 0 ; e-mode: upto subchannel
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;

        if (trans->pre_act) {
            if (RankState[trans->rank].lp_state != IDLE) return true;
            pre_act_cnt ++;
            total_pre_act_cnt ++;
            if (trans->transactionType != DATA_READ) {
                trans->data_ready_cnt = trans->burst_length + 1;
            }
            trans->arb_time = now() + tCMD2SCH_BYPACT;
            ent_delay_buf.push_back(trans);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PRE_ACT :: type"<<trans->transactionType<<" addr="<<hex
                        <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                        <<trans->row<<endl);
            }
            return true;
        }

        pre_req_time = now();

        // bypass_act
        if (BYP_ACT_EN && trans->transactionType == DATA_READ && (GetDmcQsize() == 0 || BYP_ACT_PIPE3_EN == true ) 
         && RankState[trans->rank].lp_state == IDLE && state.state->currentBankState == Idle && (bank_cnt[trans->bankIndex] == 0 || BYP_ACT_PIPE3_EN == false )) {
            bool act_met = false;
            if (DMC_V596 || BYP_ACT_PIPE3_EN) {
                if (now() >= state.state->nextActivate2) act_met = true;
                if(BYP_ACT_PIPE3_EN){
                    for(auto &cmd : CmdQueue){
                        if(cmd->cmd_type == ACTIVATE1_CMD || cmd->cmd_type == ACTIVATE2_CMD){
                            act_met = false;
                            break;
                        }
                    }
                }
            } else {
                act_met = true;
                for (auto &b : bankStates) {
                    if (now() < b.state->nextActivate2) {
                        act_met = false;
                        break;
                    }
                }
            }

            if (act_met) {
                has_bypact_exec = true;
                trans->arb_time = now() + tCMD2SCH_BYPACT;
                trans->byp_act = true;
                bypass_active_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- BYP_ACT :: addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<endl);
                }
            }
        }
        
        // fast read command: active mode && full command mode
        if (trans->fast_rd) {
            // fast read cmd simple check
            if (trans->transactionType!=DATA_READ || state.state->currentBankState!=Idle) {
                ERROR(setw(10)<<now()<<" Wrong Fast Read Command, task="<<trans->task<<", type="<<trans->transactionType
                        <<", state="<<bank_state_opcode(state.state->currentBankState)<<", rank="<<trans->rank<<" bank="<<trans->bankIndex);
                assert(0);
            }

            fast_rd_cnt ++;
            total_fast_rd_cnt ++;

            if (FAST_READ_MODE==0) {
                trans->act_only = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- FAST_READ_ACT :: type="<<trans->transactionType<<" addr="<<hex
                            <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                            <<trans->row<<" act_only="<<trans->act_only<<" byp_act="<<trans->byp_act<<endl);
                }
                rank_cnt[trans->rank] ++;
                sc_cnt[trans->rank][sub_channel] ++;       //todo: revise for e-mode
                bank_cnt[trans->bankIndex] ++;
                bg_cnt[trans->rank][trans->group] ++;
                r_rank_cnt[trans->rank] ++;
                r_rank_bst[trans->rank] += ceil(float(trans->data_size) / max_bl_data_size);
                que_read_cnt ++;
                r_bank_cnt[trans->bankIndex] ++;
                r_bg_cnt[trans->rank][trans->group] ++;
                fast_act_cnt ++;
                total_fast_act_cnt ++;
                transactionQueue.push_back(trans);
        
                // update PTC BankState info interfaced with PERF queue
                state.state->openRowAddress = trans->row;
                state.ptc_samebank_rd ++;
                state.perf_rd_rowhit = true;
                state.perf_rd_rowmiss = false;
                // todo: confirm wr info???
                state.perf_wr_rowhit = false;
                state.perf_wr_rowmiss = true;

                return true;
            } else if (FAST_READ_MODE==1) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- FAST_READ_RD :: type="<<trans->transactionType<<" addr="<<hex
                            <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                            <<trans->row<<" act_only="<<trans->act_only<<" byp_act="<<trans->byp_act<<endl);
                }
            }
        }

        totalTransactions++;
        check_conflict(trans);
        page_adapt_policy(trans);
        page_adpt_policy(trans);
        rank_cnt[trans->rank] ++;
        sc_cnt[trans->rank][sub_channel] ++;       //todo: revise for e-mode
        bank_cnt[trans->bankIndex] ++;
        bg_cnt[trans->rank][trans->group] ++;
        sid_cnt[trans->rank][trans->sid] ++;
        acc_rank_cnt[trans->rank] ++;
        acc_bank_cnt[trans->bankIndex] ++;

        if (MPAM_PUSH_EN) pushQosForSameMpamTrans(trans);
        if (MID_PUSH_EN) pushQosForSameMidTrans(trans);

        if (DMC_V590 && !SBR_IDLE_EN && !rank_send_pbr[trans->rank][sub_channel] &&
                refreshALL[trans->rank][sub_channel].refresh_cnt < ABR_PSTPND_LEVEL) {
            if (refreshALL[trans->rank][sub_channel].refreshWaiting && !refreshALL[trans->rank][sub_channel].refreshing)
                refreshALL[trans->rank][sub_channel].refreshWaiting = false;
        }

        TotalDmcBytes += trans->data_size;
        if (trans->transactionType == DATA_READ) {
            r_rank_cnt[trans->rank] ++;
            r_rank_bst[trans->rank] += ceil(float(trans->data_size) / max_bl_data_size);
            que_read_cnt ++;
            totalReads++;
            r_bank_cnt[trans->bankIndex] ++;
            r_bg_cnt[trans->rank][trans->group] ++;
            r_sid_cnt[trans->rank][trans->sid] ++;
            r_qos_cnt[trans->qos] ++;
            racc_rank_cnt[trans->rank] ++;
            racc_bank_cnt[trans->bankIndex] ++;
            TotalDmcRdBytes += trans->data_size;
            if (trans->data_size == 32) TotalDmcRd32B ++;
            else if (trans->data_size == 64) TotalDmcRd64B ++;
            else if (trans->data_size == 128) TotalDmcRd128B ++;
            else if (trans->data_size == 256) TotalDmcRd256B ++;
            if ((trans->address % trans->data_size) == 0) rd_inc_cnt ++;
            else rd_wrap_cnt ++;
            if (trans->qos >= PERF_SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[trans->rank] ++;
                highqos_r_bank_cnt[trans->bankIndex] ++;
            }
            // if (RCMD_HQOS_EN && trans->qos <= RCMD_HQOS_LEVEL) trans->cmd_hqos_type = true;
            if (trans->hqos_timeout) trans->timeout = true;            

            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD_ENT :: [R]B["<<trans->burst_length<<"]QOS["<<trans->qos
                        <<"]MID["<<trans->mid<<"] addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task
                        <<" rank="<<trans->rank<<" group="<<trans->group<<" bank="<<trans->bank<<" bankIndex="
                        <<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col
                        <<" data_size="<<trans->data_size<<" Q="<<GetDmcQsize()<<" QR="<<que_read_cnt<<" QW="
                        <<que_write_cnt<<" ptc_timeAdded="<<trans->ptc_timeAdded<<" timeout_th="<<trans->timeout_th
                        <<" timeAdded="<<trans->timeAdded<<" mask_wcmd="<<trans->mask_wcmd<<" perf_rd_rowhit="
                        <<trans->perf_rd_rowhit<<" perf_rd_rowmiss="<<trans->perf_rd_rowmiss<<" perf_wr_rowhit="<<trans->perf_wr_rowhit 
                        <<" perf_wr_rowmiss="<<trans->perf_wr_rowmiss<<" perf_maxbg_hit="<<trans->perf_maxbg_hit<<" perf_qos="
                        <<trans->perf_qos<<" pri="<<trans->pri<<" fast_rd="<<trans->fast_rd<<endl);
            }
        } else {
            w_rank_cnt[trans->rank] ++;
            w_rank_bst[trans->rank] += ceil(float(trans->data_size) / max_bl_data_size);
            w_bank_cnt[trans->bankIndex] ++;
            w_bg_cnt[trans->rank][trans->group] ++;
            w_sid_cnt[trans->rank][trans->sid] ++;
            w_qos_cnt[trans->qos] ++;
            que_write_cnt ++;
            totalWrites++;
            wacc_rank_cnt[trans->rank] ++;
            wacc_bank_cnt[trans->bankIndex] ++;
            TotalDmcWrBytes += trans->data_size;
            if (trans->data_size == 32) TotalDmcWr32B ++;
            else if (trans->data_size == 64) TotalDmcWr64B ++;
            else if (trans->data_size == 128) TotalDmcWr128B ++;
            else if (trans->data_size == 256) TotalDmcWr256B ++;
            if ((trans->address % trans->data_size) == 0) wr_inc_cnt ++;
            else wr_wrap_cnt ++;

            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD_ENT :: [W]B["<<trans->burst_length<<"]QOS["<<trans->qos
                        <<"]MID["<<trans->mid<<"] addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task
                        <<" rank="<<trans->rank<<" group="<<trans->group<<" bank="<<trans->bank<<" bankIndex="
                        <<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col
                        <<" data_size="<<trans->data_size<<" Q="<<GetDmcQsize()<<" QR="<<que_read_cnt<<" QW="
                        <<que_write_cnt<<" ptc_timeAdded="<<trans->ptc_timeAdded<<" timeout_th="<<trans->timeout_th
                        <<" timeAdded="<<trans->timeAdded<<" mask_wcmd="<<trans->mask_wcmd<<" perf_rd_rowhit="
                        <<trans->perf_rd_rowhit<<" perf_rd_rowmiss="<<trans->perf_rd_rowmiss<<" perf_wr_rowhit="<<trans->perf_wr_rowhit 
                        <<" perf_wr_rowmiss="<<trans->perf_wr_rowmiss<<" perf_maxbg_hit="<<trans->perf_maxbg_hit<<" perf_qos="
                        <<trans->perf_qos<<" pri="<<trans->pri<<endl);
            }
        }
        
        //update PTC BankState info
        if (!trans->fast_rd) {
            state.perf_rd_rowhit = trans->perf_rd_rowhit;
            state.perf_wr_rowhit = trans->perf_wr_rowhit;
            state.perf_rd_rowmiss = trans->perf_rd_rowmiss;
            state.perf_wr_rowmiss = trans->perf_wr_rowmiss;
            state.perf_rowhit_break = trans->perf_rowhit_break;
        } else {
            state.perf_rd_rowhit = false;
            state.perf_wr_rowhit = false;
            state.perf_rd_rowmiss = false;
            state.perf_wr_rowmiss = true;
            state.perf_rowhit_break = trans->perf_rowhit_break;
        }

        if (trans->transactionType == DATA_READ) {
            state.ptc_samebank_rd ++;
        } else {
            state.ptc_samebank_wr ++;
            if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ptc_same_bank_wcnt++ task="<<trans->task<<endl);
            }
        }
        

        // update PTC BankState rowhit breakinfo
        if (state.perf_rd_rowmiss || state.perf_wr_rowmiss) {
            bankStates[trans->bankIndex].ser_rhit_cnt ++; 
            if (PTC_RHIT_BREAK_EN && bankStates[trans->bankIndex].ser_rhit_cnt >= PTC_RHIT_BREAK_LEVEL 
                    && (state.perf_rd_rowhit || state.perf_wr_rowhit)) {
                    bankStates[trans->bankIndex].has_rhit_break = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- Rhit Break Trig :: addr=0x"<<hex<<trans->address<<dec
                                <<" task="<<trans->task<<" rank="<<trans->rank<<" group="<<trans->group
                                <<" bank="<<trans->bankIndex<<endl);
                    }
            }
        }

        // update open row address
        if (PERF_ACT_TCHK_EN) {        
            state.state->openRowAddress = trans->row;
        }

        for (auto &t : transactionQueue) {
            if (trans->task == t->task && !(t->fast_rd && t->act_only)) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] task:"<<trans->task<<", task duplication!");
                assert(0);
            }
        }

        
        //label no of arb group for each tran in transactionQue
        if (!SLOT_FIFO) {
            for (size_t grp_index = 0; grp_index < ARB_GROUP_NUM; grp_index ++) {
                if(arb_group_cnt[grp_index] < arb_per_group) {
                    trans->arb_group = grp_index;
                    arb_group_cnt[grp_index]++;
                    break;
                }
            }

            if (trans->arb_group >= ARB_GROUP_NUM) {
                ERROR(setw(10)<<now()<<" Wrong Arb Group, task="<<trans->task<<", arb_group="<<trans->arb_group);
                assert(0);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- Label New Cmd :: addr=0x"<<hex<<trans->address<<dec
                        <<" task="<<trans->task<<" arb_group="<<trans->arb_group<<" arb_group_cnt="<<arb_group_cnt[trans->arb_group]
                        <<" bank="<<trans->bankIndex<<endl);
            }
        }
        for(size_t i=0;i<TRANS_QUEUE_DEPTH;i++){
            if(ptc_que_slot[i] == 0) {
                trans->ptc_slot = i;
                ptc_que_slot[i] = 1;
                break;
            }
        }
                
        if(trans->fast_rd || trans->byp_act) transactionQueue.push_back(trans); 
        else ent_delay_buf.push_back(trans);           

        if (((IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS)))   
                && trans->transactionType == DATA_READ && !trans->pre_act) {
            if (pending_TransactionQue.size() >= 1000) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] pending_TransactionQue size is too big");
                assert(0);
            }
            if (pending_TransactionQue.find(trans->task) != pending_TransactionQue.end()) {
                ERROR(setw(10)<<now()<<" -- [DMC"<<channel<<"] should be error, task="<<trans->task);
                assert(0);
            }
            push_pending_TransactionQue(trans);
        }
        if (FASTWAKEUP_EN && !PREDICT_FASTWAKEUP && trans->transactionType == DATA_READ) {
            if (fast_wakeup_cnt[trans->rank] == 0 && now() >= 100) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] rank:"<<trans->rank<<", Error fast wakeup count!");
                assert(0);
            }
            fast_wakeup_cnt[trans->rank] -= 1;
        }

        if (ENH_PAGE_ADPT_EN && state.state->currentBankState == RowActive && trans->row == state.state->openRowAddress)
            bank_cnt_ehs[trans->bankIndex] ++;

        if (state.state->currentBankState == RowActive && trans->row == state.state->openRowAddress) state.row_hit_cnt ++;
        else state.row_miss_cnt ++;

        auto &st = RankState[trans->rank].lp_state;
        if (st >= PDE && st <= PDX) cmd_met_pd_cnt ++;
        if (st >= ASREFE && st <= SRPDX) cmd_met_asref_cnt ++;
        rank_cnt_asref[trans->rank] ++;
        rank_cnt_sbridle[trans->rank][sub_channel] ++;       //todo: revise for e-mode
        return true;
    } else if (!fastread) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- CMD_BP :: addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task
                    <<" bank="<<trans->bankIndex<<" QR="<<que_read_cnt<<" QW="<<que_write_cnt<<endl);
        }
        return false;
    }
}

bool PTC::dropPreAct(Transaction *trans) {
    if (!trans->pre_act) {
        return false;
    }
    bool drop_preact = false;
    if (PREACT_FLOW_CTRL_TYPE == 0 && PREACT_FLOW_CTRL_EN) {
        // if(SIMPLE_BANKTABLE_ENABLE && simple_bank_table->valid_banks.size() > PREACT_TBL_THD){
        //     drop_preact = true;
        // }
        if (!SIMPLE_BANKTABLE_ENABLE && table_use_cnt > PREACT_TBL_THD) {
            drop_preact = true;
        }
    } else if (PREACT_FLOW_CTRL_TYPE == 1 && PREACT_FLOW_CTRL_EN) {
        if (availability > PREACT_BW_THD) {
            drop_preact = true;
        }
    }
    if (drop_preact) {
        if(DEBUG_BUS){
            PRINTN(setw(10)<<now()<<" -- PRE_ACT :: DROP, type"<<trans->transactionType<<" addr="<<hex
                   <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                   <<trans->row<<", FLOW_CTRL_TYPE="<<PREACT_FLOW_CTRL_TYPE<<", bank_size="
                   <<table_use_cnt<<", bw="<<availability<<endl);
        }
        trans->drop_pre_act = true;
        return true;
    }

    if (RankState[trans->rank].lp_state != IDLE) drop_preact = true;
    if (refreshALL[trans->rank][0].refreshWaiting || refreshALL[trans->rank][0].refreshing
            || refreshPerBank[trans->rank].refreshing || refreshPerBank[trans->rank].refreshWaiting){
        drop_preact = true;
    }

    if (drop_preact) {
        if(DEBUG_BUS){
            PRINTN(setw(10)<<now()<<" -- PRE_ACT :: DROP, type"<<trans->transactionType<<" addr="<<hex
                   <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
                   <<trans->row<<", bank is refreshing"<<endl);
        }
        trans->drop_pre_act = true;
        return true;
    }

    // bool load_table = TABLE_DEPTH !=0 && table_use_cnt < TABLE_DEPTH;
    // if (!load_table) {
    //     drop_preact = true;
    // }
    // if (drop_preact) {
    //     if(DEBUG_BUS){
    //         PRINTN(setw(10)<<now()<<" -- PRE_ACT :: DROP, type"<<trans->transactionType<<" addr="<<hex
    //                <<trans->address<<dec<<" task="<<trans->task<<" bank="<<trans->bankIndex<<" row="
    //                <<trans->row<<", load table failed"<<endl);
    //     }
    //     trans->drop_pre_act = true;
    //     return true;
    // }

    return false;
}

void PTC::update_even_cycle() {
    even_cycle = ((now()%2==1 && IS_LP6 && DMC_RATE<=6400) || !IS_LP6 || (IS_LP6 && DMC_RATE>6400));
    odd_cycle  = ((now()%2==0 && IS_LP6 && DMC_RATE<=6400) || !IS_LP6 || (IS_LP6 && DMC_RATE>6400));
}

void PTC::trans_state_init(Transaction *trans) {
    
    // initialize pri && qos
    trans->ptc_timeAdded = now();
    unsigned perf_qos = trans->qos;
    unsigned perf_pri = trans->pri;
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- perf_qos="<<perf_qos<<", perf_pri="<<perf_pri<<", task="<<trans->task<<endl);
    }
    if (!PTC_QOS_HOLD_EN) {
        trans->qos = trans->qos >> 1;   // PTC priority, qos2 -> qos0 (high -> low), qos3(green path)
        trans->pri = trans->pri >> 1;
        if (trans->qos > 0 && perf_qos!=QOS_MAX) {
            trans->qos = trans->qos - 1;
        }
        if (trans->pri > 0 && perf_pri!=QOS_MAX) {
            trans->pri = trans->pri - 1;
        }
    }
    
    //initilaize several paramters
    trans->reqAddToDmcTime = now() * tDFI;
    trans->arb_time = now() + tCMD2SCH;
    trans->enter_que_time = now() + tCMD_CONF;
    trans->data_ready_cnt = trans->burst_length + 1;
    if (PERF_TIMEOUT_MODE == 0) {
        trans->improve_cnt = 0;
    }
    
    // initialize timeout_th && adapt_th
    string mpam_timeout, mpam_adapt;
    unsigned timeout = 0, pri_adapt = 0;
    if (MPAM_MAPPING_TIMEOUT && trans->mpam_id != 0) { // mpam_id mapping timeout
        if (trans->transactionType == DATA_READ) {
            mpam_timeout = "MPAM_TIMEOUT_RD";
            mpam_adapt = "MPAM_ADAPT_RD";
        } else {
            mpam_timeout = "MPAM_TIMEOUT_WR";
            mpam_adapt = "MPAM_ADAPT_WR";
        }
        if (PTC_TIMEOUT_ENABLE) timeout = MAP_CONFIG[mpam_timeout][trans->mpam_id];
        if (PTC_PRI_ADPT_ENABLE) pri_adapt = MAP_CONFIG[mpam_adapt][trans->mpam_id];
    } else if (trans->qos < PTC_QOS_MAX) { // qos mapping timeout
        if (PERF_TIMEOUT_MODE == 0) {
            if (trans->transactionType == DATA_READ) {
                if (PTC_QOS_HOLD_EN) {
                    mpam_timeout = "TIMEOUT_PRI_RD";
                    mpam_adapt = "ADAPT_PRI_RD";
                } else {
                    mpam_timeout = "PTC_TIMEOUT_PRI_RD";
                    mpam_adapt = "PTC_ADAPT_PRI_RD";
                }
            } else {
                if (PTC_QOS_HOLD_EN) {
                    mpam_timeout = "TIMEOUT_PRI_WR";
                    mpam_adapt = "ADAPT_PRI_WR";
                } else {
                    mpam_timeout = "PTC_TIMEOUT_PRI_WR";
                    mpam_adapt = "PTC_ADAPT_PRI_WR";
                }
            }
        } else if (PERF_TIMEOUT_MODE == 1) {
            if (trans->transactionType == DATA_READ) {
                if (PTC_QOS_HOLD_EN) {
                    mpam_timeout = "PERF_TIMEOUT_PRI_RD";
                    mpam_adapt = "PERF_ADAPT_PRI_RD";
                } else {
                    mpam_timeout = "PTC_TIMEOUT_PRI_RD";
                    mpam_adapt = "PTC_ADAPT_PRI_RD";
                }
            } else {
                if (PTC_QOS_HOLD_EN) {
                    mpam_timeout = "PERF_TIMEOUT_PRI_WR";
                    mpam_adapt = "PERF_ADAPT_PRI_WR";
                } else {
                    mpam_timeout = "PTC_TIMEOUT_PRI_WR";
                    mpam_adapt = "PTC_ADAPT_PRI_WR";
                }
            }
        }
        if (PTC_TIMEOUT_ENABLE) timeout = MAP_CONFIG[mpam_timeout][trans->qos];
        if (PTC_PRI_ADPT_ENABLE) pri_adapt = MAP_CONFIG[mpam_adapt][trans->qos];
    }
    if (timeout == 0) trans->timeout_th = 0;
    else trans->timeout_th = timeout;
    trans->pri_adapt_th = pri_adapt;
}

bool PTC::check_samebank(unsigned int bank) {
    for (auto &trans : transactionQueue) {
        if (bank == trans->bankIndex) return true;
    }
    return false;
}

void PTC::dfs_backpress(bool backpress) {
    dfs_backpress_en = backpress;
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- DFS :: set dfs_backpress_en="<<dfs_backpress_en<<endl);
    }
}


// void PTC::gen_rdata(uint64_t task, unsigned cnt, unsigned delay, bool mask_wcmd) {
//     data_packet pkt;
//     //the goal of mins 1 is that data path can start to counter early
//     pkt.task = task;
//     pkt.cnt = cnt;
//     pkt.delay = delay;
//     pkt.mask_wcmd = mask_wcmd;
//     pkt.channel = getID();
//     read_data_buffer.push_back(pkt);
// }

void PTC::gen_rdata(uint64_t task, unsigned cnt, unsigned delay, bool mask_wcmd) {
    if (!IS_HBM3) {
        // 非 HBM3：直接压包，每次调用都压
        data_packet pkt;
        pkt.task = task;
        pkt.cnt = cnt;
        pkt.delay = delay;
        pkt.mask_wcmd = mask_wcmd;
        pkt.channel = getID();
        read_data_buffer.push_back(pkt);
        if (DEBUG_BUS) {
            PRINTN(setw(10) << now() << " -- rdata (non-HBM3), task=" << task << endl);
        }
        return;
    }

    // HBM3 模式：按 task 合并，每两次调用压一个包
    auto it = rdata_toggle_map.find(task);
    if (it == rdata_toggle_map.end()) {
        // 首次遇到该 task，插入标志并设为 false（表示第一次调用）
        it = rdata_toggle_map.emplace(task, false).first;
    }

    bool& toggle = it->second;
    if (toggle) {
        // 第二次调用：压入数据包（使用本次参数）
        data_packet pkt;
        pkt.task = task;
        pkt.cnt = cnt;
        pkt.delay = delay;
        pkt.mask_wcmd = mask_wcmd;
        pkt.channel = getID();
        read_data_buffer.push_back(pkt);
        rdata_toggle_map.erase(task);
        if (DEBUG_BUS) {
            PRINTN(setw(10) << now() << " -- second rdata (HBM3), task=" << task << endl);
        }
    } else {
        // 第一次调用：仅翻转标志，不压包
        toggle = true;
        if (DEBUG_BUS) {
            PRINTN(setw(10) << now() << " -- first rdata (HBM3), task=" << task << endl);
        }
    }
}

void PTC::push_pending_TransactionQue(Transaction *trans) {
    TRANS_MSG msg;
    if (LAT_INC_BP) msg.time = uint64_t(trans->reqEnterDmcBufTime);
    else msg.time = now();
    msg.reqEnterDmcBufTime = trans->reqEnterDmcBufTime;
    msg.reqAddToDmcTime = trans->reqAddToDmcTime;
    msg.burst_cnt = 0;
    msg.channel = trans->channel;
    msg.rank = trans->rank;
    msg.data_size = trans->data_size;
    msg.burst_length = trans->burst_length;
    msg.qos = trans->qos;
    msg.mid = trans->mid;
    msg.address = trans->address;
    msg.pf_type = trans->pf_type;
    msg.sub_pftype = trans->sub_pftype;
    msg.sub_src = trans->sub_src;
    msg.ecc_flag = trans->ecc_flag;
    pending_TransactionQue[trans->task] = msg;
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- push pending_TransactionQue, task="<<trans->task<<endl);
    }
}

bool PTC::has_cmd_bp() {
    for (auto &bpc : bp_cycle) {
        if ((now() + 1) == bpc) return true;
    }
    return false;
}

unsigned  PTC::lru_arb(uint64_t index1, uint64_t index2, uint64_t sel) {
    if (sel == 0) return ptc_lru_matrix[index1][index2];
    else if (sel == 1) return bank_lru_matrix[index1][index2];
    else return -1;
}
void PTC::update_lru (uint64_t index, uint64_t sel,Cmd *cmd){
    if(sel == 0){
        if (IS_LP5) {
            if (cmd->issue_size+cmd->bl*DMC_DATA_BUS_BITS/8 != cmd->data_size) {
                return;
            }
        } else if (IS_LP6) {
            if (cmd->issue_size+cmd->bl*DMC_DATA_BUS_BITS/9 != cmd->data_size) {
                return;
            }
        }
        for(size_t i=0;i<TRANS_QUEUE_DEPTH;i++){
                if(i==index) continue;
                ptc_lru_matrix[i][index] = 1;
                ptc_lru_matrix[index][i] = 0;
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LRU :: update lru matrix="<<sel<<", index="<<index<<", task="<<cmd->task<<endl);
        }
    }
    if(sel == 1){
        for(size_t i=0;i<(NUM_BANKS * NUM_RANKS);i++){
                if(i==index) continue;
                bank_lru_matrix[i][index] = 1;
                bank_lru_matrix[index][i] = 0;
        }
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LRU :: update lru matrix="<<sel<<", index="<<index<<", task="<<cmd->task<<endl);
        }
    }

}
float PTC::calc_power() {
    float power_sum = 0;
    unsigned power_cnt = 0;
    power_sum += POWER_RDINC_K * rd_inc_cnt;
    power_sum += POWER_RDWRAP_K * rd_wrap_cnt;
    power_sum += POWER_WRINC_K * wr_inc_cnt;
    power_sum += POWER_WRWRAP_K * wr_wrap_cnt;
    power_sum += POWER_RDATA_K * rdata_cnt;
    power_sum += POWER_WDATA_K * wdata_cnt;
    power_sum += POWER_ACT_K * active_cnt;
    power_sum += POWER_PREP_K * (precharge_pb_cnt);
    power_sum += POWER_PRES_K * precharge_sb_cnt;
    power_sum += POWER_PREA_K * precharge_ab_cnt;
    power_sum += MAP_CONFIG["POWER_RD_K"][0] * RdCntBl[BL8];
    power_sum += MAP_CONFIG["POWER_RD_K"][1] * RdCntBl[BL16];
    power_sum += MAP_CONFIG["POWER_RD_K"][2] * RdCntBl[BL24];
    power_sum += MAP_CONFIG["POWER_RD_K"][3] * RdCntBl[BL32];
    power_sum += MAP_CONFIG["POWER_RD_K"][4] * RdCntBl[BL48];
    power_sum += MAP_CONFIG["POWER_RD_K"][5] * RdCntBl[BL64];
    power_sum += MAP_CONFIG["POWER_WR_K"][0] * WrCntBl[BL8];
    power_sum += MAP_CONFIG["POWER_WR_K"][1] * WrCntBl[BL16];
    power_sum += MAP_CONFIG["POWER_WR_K"][2] * WrCntBl[BL24];
    power_sum += MAP_CONFIG["POWER_WR_K"][3] * WrCntBl[BL32];
    power_sum += MAP_CONFIG["POWER_WR_K"][4] * WrCntBl[BL48];
    power_sum += MAP_CONFIG["POWER_WR_K"][5] * WrCntBl[BL64];
    power_sum += POWER_PBR_K * refresh_pb_cnt;
    power_sum += POWER_ABR_K * refresh_ab_cnt;
    power_sum += POWER_R2W_K * r2w_switch_cnt;
    power_sum += POWER_W2R_K * w2r_switch_cnt;
    power_sum += POWER_RNKSW_K * rank_switch_cnt;
    power_sum += POWER_IDLE_K * phy_notlp_cnt;
    power_sum += POWER_PDCC_K * phy_lp_cnt;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        power_cnt += POWER_PDE_K * PdEnterCnt[rank];
        power_cnt += POWER_ASREFE_K * AsrefEnterCnt[rank];
        power_cnt += POWER_SRPDE_K * SrpdEnterCnt[rank];
        power_cnt += POWER_PDX_K * PdExitCnt[rank];
        power_cnt += POWER_ASREFX_K * AsrefExitCnt[rank];
        power_cnt += POWER_SRPDX_K * SrpdExitCnt[rank];
    }
    for (size_t que = 0; que < TRANS_QUEUE_DEPTH; que ++) {
        power_cnt += MAP_CONFIG["POWER_QUEUE_K"][que] * memorySystemTop_->que_cnt[getID()][que];
    }
    rd_inc_cnt = 0;
    rd_wrap_cnt = 0;
    wr_inc_cnt = 0;
    wr_wrap_cnt = 0;
    rdata_cnt = 0;
    wdata_cnt = 0;
    return power_sum;
}
void PTC::ent_pop() {
    if(!ent_delay_buf.empty()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- ADD_PTC :: ["<<trans_type_opcode(ent_delay_buf[0]->transactionType)<<"]"<<" task="<<ent_delay_buf[0]->task<<endl);
        }
        transactionQueue.push_back(ent_delay_buf.front());
	    ent_delay_buf.erase(ent_delay_buf.begin());
    }
}
void PTC::rw_pop() {
    if(!rw_delay_buf.empty()) {
        Cmd *temp_cmd = new Cmd;
        temp_cmd->creat_Cmd(rw_delay_buf.front());
        CmdQueue.push_back(temp_cmd);
        pre_rwcmd.cmd_type = rw_delay_buf.front()->cmd_type;
        pre_rwcmd.task = rw_delay_buf.front()->task;
        rw_delay_buf.erase(rw_delay_buf.begin());
    }
}

PTC::~PTC() {
//    DEBUG("CAS_FS Stat: Total DMC Command: "<<totalTransactions<<", Rank Switch Count: "
//            <<rank_switch_cnt<<", CAS_FS Count: "<<casfs_cnt<<", CAS_FS Next Rank Time: "
//            <<casfs_time<<", AVG: "<<(double(casfs_time)/casfs_cnt));
}
