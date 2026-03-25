#include <errno.h>
#include <sstream> //stringstream
#include <stdlib.h> // getenv()
#include <sys/stat.h>
#include <sys/types.h>
#include <iomanip>
#include <math.h>
#include "MemorySystemTop.h"
#include "AddressMapping.h"
#include "TimingCalculate.h"
#include "IniReader.h"
#include "MRAS.h"
#include "MPFQ.h"
#include "MPTC.h"
#include "Inline_ECC.h"
#include "PFQ.h"
#include "PTC.h"
#include "Rank.h"
#include <algorithm>

using namespace DRAMSim;
//============================================================================== 
namespace DRAMSim {
#ifdef SYSARCH_PLATFORM
MemorySystemTop::MemorySystemTop(unsigned hhaId, string IniFilePath, string LogPath,
        HALib::Configurable* cfg) : 
        hhaId(hhaId), log_path(LogPath), start_cycle(0), end_cycle(0) {
#else
MemorySystemTop::MemorySystemTop(unsigned hhaId, string IniFilePath, string LogPath,
        int argc, char *argv[]) : 
        hhaId(hhaId), log_path(LogPath), start_cycle(0), end_cycle(0) {
#endif

#ifdef SYSARCH_PLATFORM
    IniFilename = "parameter/public.ini";
    if (hhaId == 0) DEBUG("== Loading public model file '"<<IniFilename<<"' ==");
    mininf::EmbededFile embed;
    string lpddr_pub_str = embed.getString(IniFilename);
    std::istringstream lpddr_sys_stream(lpddr_pub_str);
    IniReader::ReadIniFile(lpddr_pub_stream, true);

    SYSTEM_CONFIG = cfg->getString("SYSTEM_CONFIG");
    STATE_LOG = cfg->getBool("STATE_LOG");
    DEBUG_PERF = cfg->getBool("DEBUG_PERF");
    STATE_TIME = cfg->getNumber("STATE_TIME");
    DEBUG_BUS = cfg->getBool("DEBUG_BUS");
    DEBUG_STATE = cfg->getBool("DEBUG_STATE");
    DEBUG_GBUF_STATE = cfg->getBool("DEBUG_GBUF_STATE");
    DEBUG_RMW_STATE = cfg->getBool("DEBUG_RMW_STATE");
    DEBUG_PDU = cfg->getBool("DEBUG_PDU");
    ECC_MERGE_ENABLE = cfg->getBool("ECC_MERGE_ENABLE");
    DEBUG_START_TIME = cfg->getNumber("DEBUG_START_TIME");
    DEBUG_END_TIME = cfg->getNumber("DEBUG_END_TIME");
    PRINT_TRACE = cfg->getBool("PRINT_TRACE");
    PRINT_READ = cfg->getBool("PRINT_READ");
    PRINT_RDATA = cfg->getBool("PRINT_RDATA");
    PRINT_SCH = cfg->getBool("PRINT_SCH");
    PRINT_EXEC = cfg->getBool("PRINT_EXEC");
    PRINT_IDLE_LAT = cfg->getBool("PRINT_IDLE_LAT");
    PRINT_LATENCY = cfg->getBool("PRINT_LATENCY");
    PRINT_DRAM_TRACE = cfg->getBool("PRINT_DRAM_TRACE");
    PRINT_UT_TRACE = cfg->getBool("PRINT_UT_TRACE");
    PRINT_CMD_NUM = cfg->getBool("PRINT_CMD_NUM");
    PRINT_CH_OHOT = cfg->getNumber("PRINT_CH_OHOT");
    PRINT_BW = cfg->getBool("PRINT_BW");
    PRINT_BW_WIN = cfg->getNumber("PRINT_BW_WIN");
    PRINT_BANK = cfg->getBool("PRINT_BANK");
    PRINT_TIMEOUT = cfg->getBool("PRINT_TIMEOUT");
    PERFECT_DMC_EN = cfg->getBool("PERFECT_DMC_EN");
    PERFECT_DMC_DELAY = cfg->getNumber("PERFECT_DMC_DELAY");
    DROP_WRITE_CMD = cfg->getBool("DROP_WRITE_CMD");
    LAT_INC_BP = cfg->getBool("LAT_INC_BP");
    FORCE_BAINTLV_EN = cfg->getBool("FORCE_BAINTLV_EN");
    FORCE_BAINTLV_MODE = cfg->getNumber("FORCE_BAINTLV_MODE");
    POWER_EN = cfg->getBool("POWER_EN");
    BA_SHIFT_DIR = cfg->getBool("BA_SHIFT_DIR");
    BA_SHIFT_BIT = cfg->getNumber("BA_SHIFT_BIT");
    SLOT_FIFO = cfg->getBool("SLOT_FIFO");

    if (SYSTEM_CONFIG=="ddrsystem") IniFilename = IniFilePath + "/" + "DDR/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="lpsystem")  IniFilename = IniFilePath + "/" + "LP/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="lp6system") IniFilename = IniFilePath + "/" + "LP/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="hbmsystem") IniFilename = IniFilePath + "/" + "HBM/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="wsesystem") IniFilename = IniFilePath + "/" + "WSE/" + SYSTEM_CONFIG + ".ini";
    // IniFilename = "parameter/" + "LP/" + SYSTEM_CONFIG + ".ini";
    if (hhaId == 0) DEBUG("== Loading system model file '"<<IniFilename<<"' ==");
    string lpddr_sys_str = embed.getString(IniFilename);
    std::istringstream lpddr_sys_stream(lpddr_sys_str);
    IniReader::ReadIniFile(lpddr_sys_stream, true);

    // Modify Parameter From Config File
    NUM_CHANS = cfg->getNumber("NUM_CHANS");
    NUM_PFQS = cfg->getNumber("NUM_PFQS");
    DDR_TYPE = cfg->getString("DDR_TYPE");
    DDR_MODE = cfg->getString("DDR_MODE");
    DMC_RATE = cfg->getNumber("DMC_RATE");
    DRAM_VENDOR = cfg->getString("DRAM_VENDOR");
    DRAM_MODE = cfg->getString("DRAM_MODE");
    TRANS_QUEUE_DEPTH = cfg->getNumber("TRANS_QUEUE_DEPTH");
    EXEC_NUMBER = cfg->getNumber("EXEC_NUMBER");
    RD_APRE_EN = cfg->getBool("RD_APRE_EN");
    WR_APRE_EN = cfg->getBool("WR_APRE_EN");
    ENHAN_RD_AP_EN = cfg->getBool("ENHAN_RD_AP_EN");
    ENHAN_WR_AP_EN = cfg->getBool("ENHAN_WR_AP_EN");
    AREF_EN = cfg->getBool("AREF_EN");
    PBR_EN = cfg->getBool("PBR_EN");
    ENH_PBR_EN = cfg->getBool("ENH_PBR_EN");
    ENH_PBR_CEIL_EN = cfg->getBool("ENH_PBR_CEIL_EN");
    DERATING_EN = cfg->getBool("DERATING_EN");
    WSRAM_QUEUE_WIDTH = cfg->getNumber("WSRAM_QUEUE_WIDTH");
    WSRAM_QUEUE_DEPTH = cfg->getNumber("WSRAM_QUEUE_DEPTH");
    BANK_SLOT_NUM = cfg->getNumber("BANK_SLOT_NUM");
    BS_BP_LVL = cfg->getNumber("BS_BP_LVL");
    DERATING_RATIO = cfg->getNumber("DERATING_RATIO");
    PBR_PARA_EN = cfg->getBool("PBR_PARA_EN");
    AREF_OFFSET_EN = cfg->getBool("AREF_OFFSET_EN");
    SBR_IDLE_EN = cfg->getBool("SBR_IDLE_EN");
    SBR_REQ_MODE = cfg->getNumber("SBR_REQ_MODE");
    SBR_FRCST_NUM = cfg->getNumber("SBR_FRCST_NUM");
    SBR_WEIGHT_MODE = cfg->getNumber("SBR_WEIGHT_MODE");
    SBR_WEIGHT_ENH_MODE = cfg->getNumber("SBR_WEIGHT_ENH_MODE");
    SBR_GAP_CNT = cfg->getNumber("SBR_GAP_CNT");
    SBR_IDLE_ADAPT_EN = cfg->getBool("SBR_IDLE_ADAPT_EN");
    SBR_IDLE_ADAPT_WIN = cfg->getNumber("SBR_IDLE_ADAPT_WIN");
    SBR_IDLE_ADAPT_LEVEL = cfg->getNumber("SBR_IDLE_ADAPT_LEVEL");
    PD_PBR_EN = cfg->getBool("PD_PBR_EN");
    FASTWAKEUP_EN = cfg->getBool("FASTWAKEUP_EN");
    PREDICT_FASTWAKEUP = cfg->getBool("PREDICT_FASTWAKEUP");
    MAP_CONFIG["TIMEOUT_PRI_RD"] = cfg->getNumberArray("TIMEOUT_PRI_RD");
    MAP_CONFIG["TIMEOUT_PRI_WR"] = cfg->getNumberArray("TIMEOUT_PRI_WR");
    MAP_CONFIG["ADAPT_PRI_RD"] = cfg->getNumberArray("ADAPT_PRI_RD");
    MAP_CONFIG["ADAPT_PRI_WR"] = cfg->getNumberArray("ADAPT_PRI_WR");
    MPAM_MAPPING_TIMEOUT = cfg->getBool("MPAM_MAPPING_TIMEOUT");
    MAP_CONFIG["MPAM_TIMEOUT_RD"] = cfg->getNumberArray("MPAM_TIMEOUT_RD");
    MAP_CONFIG["MPAM_TIMEOUT_WR"] = cfg->getNumberArray("MPAM_TIMEOUT_WR");
    MAP_CONFIG["MPAM_ADAPT_RD"] = cfg->getNumberArray("MPAM_ADAPT_RD");
    MAP_CONFIG["MPAM_ADAPT_WR"] = cfg->getNumberArray("MPAM_ADAPT_WR");
    PRIORITY_PASS_ENABLE = cfg->getBool("PRIORITY_PASS_ENABLE");
    QOS_POLICY = cfg->getNumber("QOS_POLICY");
    RDATA_TYPE = cfg->getNumber("RDATA_TYPE");
    PAGE_ADAPT_EN = cfg->getBool("PAGE_ADAPT_EN");
    PAGE_TIMEOUT_EN = cfg->getBool("PAGE_TIMEOUT_EN");
    PERF_PRE_EN = cfg->getBool("PERF_PRE_EN");
    PTC_SEARCH_EN = cfg->getBool("PTC_SEARCH_EN");
    PERF_STATE_PRI_EN = cfg->getBool("PERF_STATE_PRI_EN");
    PERF_RHIT_PRI_EN = cfg->getBool("PERF_RHIT_PRI_EN");
    PERF_RANK_PRI_EN = cfg->getBool("PERF_RANK_PRI_EN");
    PERF_RD_RELEASE_MODE = cfg->getNumber("PERF_RD_RELEASE_MODE");
    PTC_BYP_UPDATE_EN = cfg->getBool("PTC_BYP_UPDATE_EN");
    EXEC_UNLIMIT_EN = cfg->getBool("EXEC_UNLIMIT_EN");
    PBR_LESS_CMD_EN = cfg->getBool("PBR_LESS_CMD_EN");
    PBR_LESS_CMD_LEVEL = cfg->getNumber("PBR_LESS_CMD_LEVEL");
    PBR_LESS_CMD_MODE = cfg->getNumber("PBR_LESS_CMD_MODE");
    PBR_LOCK_MODE = cfg->getNumber("PBR_LOCK_MODE");
    PBR_SEL_MODE = cfg->getNumber("PBR_SEL_MODE");
    LP_WAKEUP_MODE = cfg->getNumber("LP_WAKEUP_MODE");
    FAST_READ_EN = cfg->getBool("FAST_READ_EN");
    FAST_READ_MODE = cfg->getNumber("FAST_READ_MODE");
    FAST_RD_CANCEL_EN = cfg->getBool("FAST_RD_CANCEL_EN");
    FAST_RD_CANCEL_MODE = cfg->getNumber("FAST_RD_CANCEL_MODE");
    FAST_HQOS_PUSH_EN = cfg->getBool("FAST_HQOS_PUSH_EN");
    FAST_HQOS_PUSH_MODE = cfg->getNumber("FAST_HQOS_PUSH_MODE");
    PTC_RANK_SWITCH_EN = cfg->getBool("PTC_RANK_SWITCH_EN");
    PTC_HQOS_RANK_SWITCH_EN = cfg->getBool("PTC_HQOS_RANK_SWITCH_EN");
    WSRAM_MAP_EN = cfg->getBool("WSRAM_MAP_EN");
    BSC_EN = cfg->getBool("BSC_EN");
    PTC_HQOS_RANK_SWITCH_MODE = cfg->getNumber("PTC_HQOS_RANK_SWITCH_MODE");
    PTC_HQOS_RANK_SWITCH_LEVEL1 = cfg->getNumber("PTC_HQOS_RANK_SWITCH_LEVEL1");
    PTC_QOS_HOLD_EN = cfg->getBool("PTC_QOS_HOLD_EN");
    PERF_RANK_WMERGE_EN = cfg->getBool("PERF_RANK_WMERGE_EN");
    PERF_RANK_RMERGE_EN = cfg->getBool("PERF_RANK_RMERGE_EN");
    PERF_READ_SCH_THH = cfg->getNumber("PERF_READ_SCH_THH");
    PERF_READ_SCH_THL = cfg->getNumber("PERF_READ_SCH_THL");
    PERF_WRITE_SCH_THH = cfg->getNumber("PERF_WRITE_SCH_THH");
    PERF_WRITE_SCH_THL = cfg->getNumber("PERF_WRITE_SCH_THL");
    PTC_RSCH_GAP_CNT = cfg->getNumber("PTC_RSCH_GAP_CNT");
    PTC_WSCH_GAP_CNT = cfg->getNumber("PTC_WSCH_GAP_CNT");
    PTC_RRANK_SWITCH_LEVEL = cfg->getNumber("PTC_RRANK_SWITCH_LEVEL");
    PTC_WRANK_SWITCH_LEVEL = cfg->getNumber("PTC_WRANK_SWITCH_LEVEL");
    PTC_EMPTY_RD_TH = cfg->getNumber("PTC_EMPTY_RD_TH");
    PTC_EMPTY_WR_TH = cfg->getNumber("PTC_EMPTY_WR_TH");
    PTC_R2W_SWITCH_TH = cfg->getNumber("PTC_R2W_SWITCH_TH");
    PTC_W2R_SWITCH_TH = cfg->getNumber("PTC_W2R_SWITCH_TH");
    OPENPAGE_TIME_RD = cfg->getNumber("OPENPAGE_TIME_RD");
    OPENPAGE_TIME_WR = cfg->getNumber("OPENPAGE_TIME_WR");
    ENH_PAGE_ADPT_EN = cfg->getBool("ENH_PAGE_ADPT_EN");
    ENH_PAGE_ADPT_WIN = cfg->getNumber("ENH_PAGE_ADPT_WIN");
    MAP_CONFIG["ENH_PAGE_ADPT_LVL"] = cfg->getNumberArray("ENH_PAGE_ADPT_LVL");
    MAP_CONFIG["ENH_PAGE_ADPT_TIME"] = cfg->getNumberArray("ENH_PAGE_ADPT_TIME");
    DMC_BW_WIN = cfg->getNumber("DMC_BW_WIN");
    MAP_CONFIG["DMC_BW_LEVEL"] = cfg->getNumberArray("DMC_BW_LEVEL");
    PAGE_ADPT_EN = cfg->getBool("PAGE_ADPT_EN");
    PAGE_WIN_MODE = cfg->getNumber("PAGE_WIN_MODE");
    PAGE_WIN_SIZE = cfg->getNumber("PAGE_WIN_SIZE");
    PAGE_OPC_TH = cfg->getNumber("PAGE_OPC_TH");
    PAGE_PPC_TH = cfg->getNumber("PAGE_PPC_TH");
    PAGE_ADPT_STEP = cfg->getNumber("PAGE_ADPT_STEP");
    PAGE_TIME_MAX = cfg->getNumber("PAGE_TIME_MAX");
    DRESP_BP_TH = cfg->getNumber("DRESP_BP_TH");
    BUSYSTATE_INC_WCMD = cfg->getBool("BUSYSTATE_INC_WCMD");
    BUSYSTATE_TH = cfg->getNumber("BUSYSTATE_TH");
    IECC_ENABLE = cfg->getBool("IECC_ENABLE");
    PDU_DEPTH = cfg->getNumber("PDU_DEPTH");
    IECC_CONFLICT_CNT = cfg->getNumber("IECC_CONFLICT_CNT");
    IECC_PARTIAL_BYPASS = cfg->getBool("IECC_PARTIAL_BYPASS");
    IECC_BYPASS_ADDRESS = cfg->getNumber("IECC_BYPASS_ADDRESS");
    IECC_BL32_MODE = cfg->getBool("IECC_BL32_MODE");
    IECC_PRI = cfg->getNumber("IECC_PRI");
    IECC_CAP_RATIO = cfg->getNumber("IECC_CAP_RATIO");
    DMC_THEORY_BW = cfg->getNumber("DMC_THEORY_BW");
    FLOW_STAT_TIME = cfg->getNumber("FLOW_STAT_TIME");
    MPAM_PUSH_EN = cfg->getBool("MPAM_PUSH_EN");
    MID_PUSH_EN = cfg->getBool("MID_PUSH_EN");
    PD_ENABLE = cfg->getBool("PD_ENABLE");
    PD_PRD = cfg->getNumber("PD_PRD");
    ASREF_ENABLE = cfg->getBool("ASREF_ENABLE");
    ASREF_PRD = cfg->getNumber("ASREF_PRD");
    ASREF_ADAPT_EN = cfg->getBool("ASREF_ADAPT_EN");
    ASREF_ADAPT_WIN = cfg->getNumber("ASREF_ADAPT_WIN");
    MAP_CONFIG["ASREF_ADAPT_LEVEL"] = cfg->getNumberArray("ASREF_ADAPT_LEVEL");
    MAP_CONFIG["ASREF_ADAPT_PRD"] = cfg->getNumberArray("ASREF_ADAPT_PRD");
    WCK_ALWAYS_ON = cfg->getBool("WCK_ALWAYS_ON");
    CAS_FS_EN = cfg->getBool("CAS_FS_EN");
    CAS_FS_TH = cfg->getNumber("CAS_FS_TH");
    CORE_CONCURR = cfg->getNumber("CORE_CONCURR");
    CORE_CONCURR_PRD = cfg->getNumber("CORE_CONCURR_PRD");
    RWGRP_TRANS_BY_TOUT = cfg->getBool("RWGRP_TRANS_BY_TOUT");
    GRP_RANK_MODE = cfg->getNumber("GRP_RANK_MODE");
    CPU_MID_START = cfg->getNumber("CPU_MID_START");
    CPU_MID_END = cfg->getNumber("CPU_MID_END");
    RHIT_HQOS_BREAK_EN = cfg->getBool("RHIT_HQOS_BREAK_EN");
    RHIT_HQOS_BREAK_OTH_RCMD_LEVEL = cfg->getNumber("RHIT_HQOS_BREAK_OTH_RCMD_LEVEL");
    BYP_ACT_EN = cfg->getBool("BYP_ACT_EN");
    BYP_ACT_PIPE3_EN = cfg->getBool("BYP_ACT_PIPE3_EN");
    PTC_RHIT_BREAK_EN = cfg->getBool("PTC_RHIT_BREAK_EN");
    PTC_RHIT_BREAK_LEVEL = cfg->getNumber("PTC_RHIT_BREAK_LEVEL");
    TOUT_SCH_TIME = cfg->getNumber("TOUT_SCH_TIME");
    DMC_V580 = cfg->getBool("DMC_V580");
    DMC_V590 = cfg->getBool("DMC_V590");
    DMC_V596 = cfg->getBool("DMC_V596");
    TRFC_CC_EN = cfg->getBool("TRFC_CC_EN");
    MERGE_ENABLE = cfg->getBool("MERGE_ENABLE");
    FORWARD_ENABLE = cfg->getBool("FORWARD_ENABLE");
    RQ_ADCONF_PUSH_EN = cfg->getBool("RQ_ADCONF_PUSH_EN");
    CMD_ROW_ORDER = cfg->getNumber("CMD_ROW_ORDER");
    TOTAL_RCMD_MODE = cfg->getNumber("TOTAL_RCMD_MODE");
    RO_HIT_EN = cfg->getBool("RO_HIT_EN");
    MAP_CONFIG["GRP_MODE_LEVEL0"] = cfg->getNumberArray("GRP_MODE_LEVEL0");
    MAP_CONFIG["GRP_MODE_LEVEL1"] = cfg->getNumberArray("GRP_MODE_LEVEL1");
    MAP_CONFIG["GRP_MODE_LEVEL2"] = cfg->getNumberArray("GRP_MODE_LEVEL2");
    MAP_CONFIG["GRP_MODE_LEVEL3"] = cfg->getNumberArray("GRP_MODE_LEVEL3");
    MAP_CONFIG["BANK_CMD_TH"] = cfg->getNumberArray("BANK_CMD_TH");
    MAP_CONFIG["NO_CMD_SCH_TH"] = cfg->getNumberArray("NO_CMD_SCH_TH");
    MAP_CONFIG["WR_LEVEL0"] = cfg->getNumberArray("WR_LEVEL0");
    MAP_CONFIG["WR_LEVEL1"] = cfg->getNumberArray("WR_LEVEL1");
    MAP_CONFIG["WR_LEVEL2"] = cfg->getNumberArray("WR_LEVEL2");
    MAP_CONFIG["WR_LEVEL3"] = cfg->getNumberArray("WR_LEVEL3");
    MAP_CONFIG["WR_MOST_LEVEL0"] = cfg->getNumberArray("WR_MOST_LEVEL0");
    MAP_CONFIG["WR_MOST_LEVEL1"] = cfg->getNumberArray("WR_MOST_LEVEL1");
    MAP_CONFIG["WR_MOST_LEVEL2"] = cfg->getNumberArray("WR_MOST_LEVEL2");
    MAP_CONFIG["WR_MOST_LEVEL3"] = cfg->getNumberArray("WR_MOST_LEVEL3");
    MAP_CONFIG["RD_LEVEL0"] = cfg->getNumberArray("RD_LEVEL0");
    MAP_CONFIG["RD_LEVEL1"] = cfg->getNumberArray("RD_LEVEL1");
    MAP_CONFIG["RD_LEVEL2"] = cfg->getNumberArray("RD_LEVEL2");
    MAP_CONFIG["RD_LEVEL3"] = cfg->getNumberArray("RD_LEVEL3");
    MAP_CONFIG["PERF_TIMEOUT_PRI_RD"] = cfg->getNumberArray("PERF_TIMEOUT_PRI_RD");
    MAP_CONFIG["PERF_TIMEOUT_PRI_WR"] = cfg->getNumberArray("PERF_TIMEOUT_PRI_WR");
    MAP_CONFIG["PERF_ADAPT_PRI_RD"] = cfg->getNumberArray("PERF_ADAPT_PRI_RD");
    MAP_CONFIG["PERF_ADAPT_PRI_WR"] = cfg->getNumberArray("PERF_ADAPT_PRI_WR");
    WRITE_BUFFER_ENABLE = cfg->getBool("WRITE_BUFFER_ENABLE");
    PERF_TIMEOUT_EN = cfg->getBool("PERF_TIMEOUT_EN");
    PERF_TIMEOUT_MODE = cfg->getNumber("PERF_TIMEOUT_MODE");
    TOUT_FORCE_RWGRP_EN = cfg->getBool("TOUT_FORCE_RWGRP_EN");
    PERF_DUMMY_TOUT_EN = cfg->getBool("PERF_DUMMY_TOUT_EN");
    EM_ENABLE = cfg->getBool("EM_ENABLE");
    EM_MODE = cfg->getNumber("EM_MODE");
    RMW_ENABLE = cfg->getBool("RMW_ENABLE");
    RMW_ENABLE_PERF = cfg->getBool("RMW_ENABLE_PERF");
    RMW_RDATA_FIFO_DEPTH = cfg->getNumber("RMW_RDATA_FIFO_DEPTH");
    RMW_CMD_MODE = cfg->getNumber("RMW_CMD_MODE");
    RMW_QUE_DEPTH = cfg->getNumber("RMW_QUE_DEPTH");
    RMW_CONF_SIZE = cfg->getNumber("RMW_CONF_SIZE");
    ROW_SEL = cfg->getNumber("ROW_SEL");
	CONF_EN = cfg->getBool("CONF_EN");
    SIMPLE_RWGRP_EN = cfg->getBool("SIMPLE_RWGRP_EN");
    SIMPLE_RWGRP_MODE = cfg->getNumber("SIMPLE_RWGRP_MODE");
    PERF_GRPST_SWITCH_LVL0 = cfg->getNumber("PERF_GRPST_SWITCH_LVL0");
    PERF_GRPST_SWITCH_LVL1 = cfg->getNumber("PERF_GRPST_SWITCH_LVL1");
    ARB_GROUP_NUM = cfg->getNumber("ARB_GROUP_NUM");
    ACT_LEFT_CYCLE = cfg->getNumber("ACT_LEFT_CYCLE");
    PTC_RANK_NUM = cfg->getNumber("PTC_RANK_NUM");
    PTC_TIMEOUT_ENABLE = cfg->getBool("PTC_TIMEOUT_ENABLE");
    PTC_TIMEOUT_MODE = cfg->getNumber("PTC_TIMEOUT_MODE");
    PTC_PRI_ADPT_ENABLE = cfg->getBool("PTC_PRI_ADPT_ENABLE");
    PERF_CLK_MODE = cfg->getNumber("PERF_CLK_MODE");
    PERF_DEPTH = cfg->getNumber("PERF_DEPTH");
    PERF_SUBQ_NUM = cfg->getNumber("PERF_SUBQ_NUM");
    SUBQ_SEL_MODE = cfg->getNumber("SUBQ_SEL_MODE");
    PERF_RELEASE_REC_WIN = cfg->getNumber("PERF_RELEASE_REC_WIN");
    CLKH2CLKL = cfg->getNumber("CLKH2CLKL");
    PERF_PRI_ADAPT_ENABLE = cfg->getBool("PERF_PRI_ADAPT_ENABLE");
    PERF_QOS_POLICY = cfg->getNumber("PERF_QOS_POLICY");
    QOS_MAX = cfg->getNumber("QOS_MAX");
    PERF_ADAPT_PRI_MAX = cfg->getNumber("PERF_ADAPT_PRI_MAX");
    PTC_MAX_RANK = cfg->getNumber("PTC_MAX_RANK");
    PTC_QOS_AMX = cfg->getNumber("PTC_QOS_MAX");
    PTC_ADAPT_PRI_MAX = cfg->getNumber("PTC_ADAPT_PRI_MAX");
    PTC_MAX_CNT_PER_RANK = cfg->getNumber("PTC_MAX_CNT_PER_RANK");
    SAME_BANK_CNT_RD = cfg->getNumber("SAME_BANK_CNT_RD");
    SAME_BANK_CNT_WR = cfg->getNumber("SAME_BANK_CNT_WR");
    SAME_BANK_CNT_ROWMISS = cfg->getNumber("SAME_BANK_CNT_ROWMISS");
    PERF_ROWHIT_BREAK_TH = cfg->getNumber("PERF_ROWHIT_BREAK_TH");
    IDLE_TRIG_TIME = cfg->getNumber("IDLE_TRIG_TIME");
    GAP_CNT_BASE = cfg->getNumber("GAP_CNT_BASE");
    GAP_CNT_TIMES = cfg->getNumber("GAP_CNT_TIMES");
    SAME_BG_CNT_RD = cfg->getNumber("SAME_BG_CNT_RD");
    SAME_BG_CNT_WR = cfg->getNumber("SAME_BG_CNT_WR");
    BG_PRE_N_CMD = cfg->getNumber("BG_PRE_N_CMD");
    RANK_GROUP_MODE = cfg->getNumber("RANK_GROUP_MODE");
    PERF_RWGRP_MODE = cfg->getNumber("PERF_RWGRP_MODE");
    PERF_NG_HOLD_WR_EN = cfg->getBool("PERF_NG_HOLD_WR_EN");
    PERF_NG_HOLD_WR_MODE = cfg->getNumber("PERF_NG_HOLD_WR_MODE");
    PERF_NG_HOLD_WR_CNT = cfg->getNumber("PERF_NG_HOLD_WR_CNT");
    PERF_FAST_R2W_EN = cfg->getBool("PERF_FAST_R2W_EN");
    PERF_RCMD_HQOS_W2R_SWITCH_EN = cfg->getBool("PERF_RCMD_HQOS_W2R_SWITCH_EN");
    PERF_RCMD_HQOS_W2R_RLEVELH = cfg->getNumber("PERF_RCMD_HQOS_W2R_RLEVELH");
    WDDA = cfg->getNumber("WDDA");
    PERF_SWITCH_HQOS_LEVEL = cfg->getNumber("PERF_SWITCH_HQOS_LEVEL");
    PERF_SCH_RRANK_LEVELH = cfg->getNumber("PERF_SCH_RRANK_LEVELH");
    PERF_SCH_RRANK_LEVELL = cfg->getNumber("PERF_SCH_RRANK_LEVELL");
    PERF_HOLD_RRANK_LEVELH = cfg->getNumber("PERF_HOLD_RRANK_LEVELH");
    PERF_HOLD_RRANK_LEVELL = cfg->getNumber("PERF_HOLD_RRANK_LEVELL");
    PERF_SCH_WRANK_LEVELH = cfg->getNumber("PERF_SCH_WRANK_LEVELH");
    PERF_SCH_WRANK_LEVELL = cfg->getNumber("PERF_SCH_WRANK_LEVELL");
    PERF_HOLD_WRANK_LEVELH = cfg->getNumber("PERF_HOLD_WRANK_LEVELH");
    PERF_HOLD_WRANK_LEVELL = cfg->getNumber("PERF_HOLD_WRANK_LEVELL");
    PERF_RRNK_ENGRP_LEVEL = cfg->getNumber("PERF_RRNK_ENGRP_LEVEL");
    PERF_RRNK_EXGRP_LEVEL = cfg->getNumber("PERF_RRNK_EXGRP_LEVEL");
    PERF_WRNK_ENGRP_LEVEL = cfg->getNumber("PERF_WRNK_ENGRP_LEVEL");
    PERF_WRNK_EXGRP_LEVEL = cfg->getNumber("PERF_WRNK_EXGRP_LEVEL");
    PERF_ENGRP_LEVEL = cfg->getNumber("PERF_ENGRP_LEVEL");
    PERF_EXGRP_LEVEL = cfg->getNumber("PERF_EXGRP_LEVEL");
    PERF_CMD_WLEVELH = cfg->getNumber("PERF_CMD_WLEVELH");
    PERF_ACT_TCHK_EN = cfg->getBool("PERF_ACT_TCHK_EN");
    RPFIFO_EN = cfg->getBool("RPFIFO_EN");
    RPFIFO_DEPTH = cfg->getNumber("RPFIFO_DEPTH");
    RPFIFO_AMFULL_TH = cfg->getNumber("RPFIFO_AMFULL_TH");
    ODD_TDM = cfg->getBool("ODD_TDM");
    PREACT_FLOW_CTRL_EN = cfg->getBool("PREACT_FLOW_CTRL_EN");
    PREACT_FLOW_CTRL_TYPE = cfg->getNumber("PREACT_FLOW_CTRL_TYPE");
    SIMPLE_BANKTABLE_ENABLE = cfg->getBool("PREACT_FLOW_CTRL_EN");
    PREACT_TBL_THD = cfg->getNumber("PREACT_TBL_THD");
    PREACT_BW_THD = cfg->getNumber("PREACT_BW_THD");
    TABLE_DEPTH = cfg->getNumber("TABLE_DEPTH");
    SID_LOOSE = cfg->getBool("SID_LOOSE");
    SIMPLE_GRP_SID_EN = cfg->getBool("SIMPLE_GRP_SID_EN");
    TIMEOUT_SID = cfg->getNumber("TIMEOUT_SID");
    TIMEOUT_SID_ENABLE = cfg->getBool("TIMEOUT_SID_ENABLE");
    SERIAL_PRE_SIDGRP = cfg->getNumber("SERIAL_PRE_SIDGRP");
    ENGRP_LR_LEVEL = cfg->getNumber("ENGRP_LR_LEVEL");
    EX_LR_LEVEL = cfg->getNumber("EX_LR_LEVEL");
    LR_LEVELH = cfg->getNumber("LR_LEVELH");
    LR_LEVELL = cfg->getNumber("LR_LEVELL");
    SERIAL_LR_LEVELH = cfg->getNumber("SERIAL_LR_LEVELH");
    SERIAL_LR_LEVELL = cfg->getNumber("SERIAL_LR_LEVELL");
    SID_GRP_PIPE = cfg->getNumber("SID_GRP_PIPE");
    ROW_SEL_MSB = cfg->getNumber("ROW_SEL_MSB");
    ROW_SEL_SMSB = cfg->getNumber("ROW_SEL_SMSB");
    ADDR_EXP_EN = cfg->getBool("ADDR_EXP_EN");
    ZHUQUE_ENABLE = cfg->getBool("ZHUQUE_ENABLE");
    ZHUQUE_BA_MODE = cfg->getNumber("ZHUQUE_BA_MODE");
    RAND_BABG = cfg->getBool("RAND_BABG");
    FORCE_SID_SWITCH = cfg->getBool("FORCE_SID_SWITCH");
    SID_SW_RATIO = cfg->getNumber("SID_SW_RATIO");

    if (SYSTEM_CONFIG=="ddrsystem") IniFilename = IniFilePath + "/" + "DDR/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="lpsystem")  IniFilename = IniFilePath + "/" + "LP/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="lp6system") IniFilename = IniFilePath + "/" + "LP/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="hbmsystem") IniFilename = IniFilePath + "/" + "HBM/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="wsesystem") IniFilename = IniFilePath + "/" + "WSE/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    // IniFilename = "parameter/" + "LP/" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DDR_MODE + ".ini";
    if (hhaId == 0) DEBUG("== Loading device model file '"<<IniFilename<<"' ==");
    string lpddr_dev_str = embed.getString(IniFilename);
    std::istringstream lpddr_dev_stream(lpddr_dev_str);
    IniReader::ReadIniFile(lpddr_dev_stream, false);

    DMC_DATA_BUS_BITS = cfg->getNumber("DMC_DATA_BUS_BITS");

    CalcMatrixNum();
    if (SYSTEM_CONFIG=="ddrsystem") IniFilename = IniFilePath + "/" + "DDR/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="lpsystem")  IniFilename = IniFilePath + "/" + "LP/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="lp6system") IniFilename = IniFilePath + "/" + "LP/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="hbmsystem") IniFilename = IniFilePath + "/" + "HBM/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="wsesystem") IniFilename = IniFilePath + "/" + "WSE/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    // IniFilename = "parameter/" + "LP/" +  DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
    //         "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (hhaId == 0) DEBUG("== Loading DRAM power model file '"<<IniFilename<<"' ==");
    string drampower_str = embed.getString(IniFilename);
    std::istringstream drampower_stream(drampower_str);
    IniReader::ReadIniFile(drampower_stream, false);
#else
    IniFilename = IniFilePath + "/public.ini";
    if (hhaId == 0) DEBUG("== Loading public model file '"<<IniFilename<<"' ==");
    ifstream iniFile;
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        IniReader::ReadIniFile(iniFile, true);
    } else {
        ERROR("Unable to load ini file "<<IniFilename);
        abort();
    }
    iniFile.close();

    Configurable cfg;
    for (int i = 1; i < argc; i ++) {
        cfg.getString(argv[i]);
    }

    GET_PARAM(SYSTEM_CONFIG, "SYSTEM_CONFIG", get);
    GET_PARAM(STATE_LOG, "STATE_LOG", getBool);
    GET_PARAM(DEBUG_PERF, "DEBUG_PERF", getBool);
    GET_PARAM(STATE_TIME, "STATE_TIME", getUint);
    GET_PARAM(DEBUG_BUS, "DEBUG_BUS", getBool);
    GET_PARAM(DEBUG_STATE, "DEBUG_STATE", getBool);
    GET_PARAM(DEBUG_GBUF_STATE, "DEBUG_GBUF_STATE", getBool);
    GET_PARAM(DEBUG_RMW_STATE, "DEBUG_RMW_STATE", getBool);
    GET_PARAM(DEBUG_PDU, "DEBUG_PDU", getBool);
    GET_PARAM(ECC_MERGE_ENABLE, "ECC_MERGE_ENABLE", getBool);
    GET_PARAM(DEBUG_START_TIME, "DEBUG_START_TIME", getUint64);
    GET_PARAM(DEBUG_END_TIME, "DEBUG_END_TIME", getUint64);
    GET_PARAM(PRINT_TRACE, "PRINT_TRACE", getBool);
    GET_PARAM(PRINT_READ, "PRINT_READ", getBool);
    GET_PARAM(PRINT_RDATA, "PRINT_RDATA", getBool);
    GET_PARAM(PRINT_SCH, "PRINT_SCH", getBool);
    GET_PARAM(PRINT_EXEC, "PRINT_EXEC", getBool);
    GET_PARAM(PRINT_IDLE_LAT, "PRINT_IDLE_LAT", getBool);
    GET_PARAM(PRINT_LATENCY, "PRINT_LATENCY", getBool);
    GET_PARAM(PRINT_DRAM_TRACE, "PRINT_DRAM_TRACE", getBool);
    GET_PARAM(PRINT_UT_TRACE, "PRINT_UT_TRACE", getBool);
    GET_PARAM(PRINT_CMD_NUM, "PRINT_CMD_NUM", getBool);
    GET_PARAM(PRINT_CH_OHOT, "PRINT_CH_OHOT", getUint64);
    GET_PARAM(PRINT_BW, "PRINT_BW", getBool);
    GET_PARAM(PRINT_BW_WIN, "PRINT_BW_WIN", getUint);
    GET_PARAM(PRINT_BANK, "PRINT_BANK", getBool);
    GET_PARAM(PRINT_TIMEOUT, "PRINT_TIMEOUT", getBool);
    GET_PARAM(PERFECT_DMC_EN, "PERFECT_DMC_EN", getBool);
    GET_PARAM(PERFECT_DMC_DELAY, "PERFECT_DMC_DELAY", getUint);
    GET_PARAM(DROP_WRITE_CMD, "DROP_WRITE_CMD", getBool);
    GET_PARAM(LAT_INC_BP, "LAT_INC_BP", getBool);
    GET_PARAM(FORCE_BAINTLV_EN, "FORCE_BAINTLV_EN", getBool);
    GET_PARAM(FORCE_BAINTLV_MODE, "FORCE_BAINTLV_MODE", getUint);
    GET_PARAM(POWER_EN, "POWER_EN", getBool);
    GET_PARAM(BA_SHIFT_DIR, "BA_SHIFT_DIR", getBool);
    GET_PARAM(BA_SHIFT_BIT, "BA_SHIFT_BIT", getUint);
    GET_PARAM(SLOT_FIFO, "SLOT_FIFO", getBool);
    GET_PARAM(ZHUQUE_ENABLE, "ZHUQUE_ENABLE", getBool);
    GET_PARAM(ZHUQUE_BA_MODE, "ZHUQUE_BA_MODE", getUint);
        
    if (SYSTEM_CONFIG=="ddrsystem") IniFilename = IniFilePath + "/" + "DDR/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="lpsystem")  IniFilename = IniFilePath + "/" + "LP/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="lp6system") IniFilename = IniFilePath + "/" + "LP/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="hbmsystem") {
        if(ZHUQUE_ENABLE){
            IniFilename = IniFilePath + "/" + "HBM/" +  "/zhuque_" + SYSTEM_CONFIG + ".ini";
        } else {
            IniFilename = IniFilePath + "/" + "HBM/" + SYSTEM_CONFIG + ".ini";
        }
    }
    // if (SYSTEM_CONFIG=="zhuque_hbmsystem") IniFilename = IniFilePath + "/" + "HBM/" + SYSTEM_CONFIG + ".ini";
    if (SYSTEM_CONFIG=="wsesystem") IniFilename = IniFilePath + "/" + "WSE/" + SYSTEM_CONFIG + ".ini";
    // IniFilename = IniFilePath + "/" + "LP/" + SYSTEM_CONFIG + ".ini";
    if (hhaId == 0) DEBUG("== Loading system model file '"<<IniFilename<<"' ==");
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        IniReader::ReadIniFile(iniFile, true);
    } else {
        ERROR("Unable to load ini file "<<IniFilename);
        abort();
    }
    iniFile.close();
    
    GET_PARAM(WSRAM_QUEUE_WIDTH, "WSRAM_QUEUE_WIDTH", getUint);
    GET_PARAM(WSRAM_QUEUE_DEPTH, "WSRAM_QUEUE_DEPTH", getUint);
    GET_PARAM(BANK_SLOT_NUM, "BANK_SLOT_NUM", getUint);
    GET_PARAM(BS_BP_LVL, "BS_BP_LVL", getUint);
    GET_PARAM(NUM_CHANS, "NUM_CHANS", getUint);
    GET_PARAM(NUM_PFQS, "NUM_PFQS", getUint);
    GET_PARAM(DDR_TYPE, "DDR_TYPE", get);
    GET_PARAM(DDR_MODE, "DDR_MODE", get);
    GET_PARAM(DMC_RATE, "DMC_RATE", getUint);
    GET_PARAM(DRAM_VENDOR, "DRAM_VENDOR", get);
    GET_PARAM(DRAM_MODE, "DRAM_MODE", get);
    GET_PARAM(TRANS_QUEUE_DEPTH, "TRANS_QUEUE_DEPTH", getUint);
    GET_PARAM(EXEC_NUMBER, "EXEC_NUMBER", getUint);
    GET_PARAM(RD_APRE_EN, "RD_APRE_EN", getBool);
    GET_PARAM(WR_APRE_EN, "WR_APRE_EN", getBool);
    GET_PARAM(ENHAN_RD_AP_EN, "ENHAN_RD_AP_EN", getBool);
    GET_PARAM(ENHAN_WR_AP_EN, "ENHAN_WR_AP_EN", getBool);
    GET_PARAM(AREF_EN, "AREF_EN", getBool);
    GET_PARAM(PBR_EN, "PBR_EN", getBool);
    GET_PARAM(ENH_PBR_EN, "ENH_PBR_EN", getBool);
    GET_PARAM(ENH_PBR_CEIL_EN, "ENH_PBR_CEIL_EN", getBool);
    GET_PARAM(DERATING_EN, "DERATING_EN", getBool);
    GET_PARAM(DERATING_RATIO, "DERATING_RATIO", getUint);
    GET_PARAM(PBR_PARA_EN, "PBR_PARA_EN", getBool);
    GET_PARAM(AREF_OFFSET_EN, "AREF_OFFSET_EN", getBool);
    GET_PARAM(SBR_IDLE_EN, "SBR_IDLE_EN", getBool);
    GET_PARAM(SBR_REQ_MODE, "SBR_REQ_MODE", getUint);
    GET_PARAM(SBR_FRCST_NUM, "SBR_FRCST_NUM", getUint);
    GET_PARAM(SBR_WEIGHT_MODE, "SBR_WEIGHT_MODE", getUint);
    GET_PARAM(SBR_WEIGHT_ENH_MODE, "SBR_WEIGHT_ENH_MODE", getUint);
    GET_PARAM(SBR_GAP_CNT, "SBR_GAP_CNT", getUint);
    GET_PARAM(SBR_IDLE_ADAPT_EN, "SBR_IDLE_ADAPT_EN", getBool);
    GET_PARAM(SBR_IDLE_ADAPT_WIN, "SBR_IDLE_ADAPT_WIN", getUint);
    GET_PARAM(SBR_IDLE_ADAPT_LEVEL, "SBR_IDLE_ADAPT_LEVEL", getUint);
    GET_PARAM(PD_PBR_EN, "PD_PBR_EN", getBool);
    GET_PARAM(FASTWAKEUP_EN, "FASTWAKEUP_EN", getBool);
    GET_PARAM(PREDICT_FASTWAKEUP, "PREDICT_FASTWAKEUP", getBool);
    GET_PARAM(MAP_CONFIG["TIMEOUT_PRI_RD"], "TIMEOUT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["TIMEOUT_PRI_WR"], "TIMEOUT_PRI_WR", getUintArray);
    GET_PARAM(MAP_CONFIG["PTC_TIMEOUT_PRI_RD"], "PTC_TIMEOUT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["PTC_TIMEOUT_PRI_WR"], "PTC_TIMEOUT_PRI_WR", getUintArray);
    GET_PARAM(MAP_CONFIG["ADAPT_PRI_RD"], "ADAPT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["ADAPT_PRI_WR"], "ADAPT_PRI_WR", getUintArray);
    GET_PARAM(MAP_CONFIG["PTC_ADAPT_PRI_RD"], "PTC_ADAPT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["PTC_ADAPT_PRI_WR"], "PTC_ADAPT_PRI_WR", getUintArray);
    GET_PARAM(MPAM_MAPPING_TIMEOUT, "MPAM_MAPPING_TIMEOUT", getBool);
    GET_PARAM(MAP_CONFIG["MPAM_TIMEOUT_RD"], "MPAM_TIMEOUT_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["MPAM_TIMEOUT_WR"], "MPAM_TIMEOUT_WR", getUintArray);
    GET_PARAM(MAP_CONFIG["MPAM_ADAPT_RD"], "MPAM_ADAPT_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["MPAM_ADAPT_WR"], "MPAM_ADAPT_WR", getUintArray);
    GET_PARAM(PRIORITY_PASS_ENABLE, "PRIORITY_PASS_ENABLE", getBool);
    GET_PARAM(QOS_POLICY, "QOS_POLICY", getUint);
    GET_PARAM(RDATA_TYPE, "RDATA_TYPE", getUint);
    GET_PARAM(PAGE_ADAPT_EN, "PAGE_ADAPT_EN", getBool);
    GET_PARAM(PAGE_TIMEOUT_EN, "PAGE_TIMEOUT_EN", getBool);
    GET_PARAM(PERF_PRE_EN, "PERF_PRE_EN", getBool);
    GET_PARAM(PTC_SEARCH_EN, "PTC_SEARCH_EN", getBool);
    GET_PARAM(PERF_STATE_PRI_EN, "PERF_STATE_PRI_EN", getBool);
    GET_PARAM(PERF_RHIT_PRI_EN, "PERF_RHIT_PRI_EN", getBool);
    GET_PARAM(PERF_RANK_PRI_EN, "PERF_RANK_PRI_EN", getBool);
    GET_PARAM(PERF_RD_RELEASE_MODE, "PERF_RD_RELEASE_MODE", getUint);
    GET_PARAM(PTC_BYP_UPDATE_EN, "PTC_BYP_UPDATE_EN", getBool);
    GET_PARAM(EXEC_UNLIMIT_EN, "EXEC_UNLIMIT_EN", getBool);
    GET_PARAM(PBR_LESS_CMD_EN, "PBR_LESS_CMD_EN", getBool);
    GET_PARAM(PBR_LESS_CMD_LEVEL, "PBR_LESS_CMD_LEVEL", getUint);
    GET_PARAM(PBR_LESS_CMD_MODE, "PBR_LESS_CMD_MODE", getUint);
    GET_PARAM(PBR_LOCK_MODE, "PBR_LOCK_MODE", getUint);
    GET_PARAM(PBR_SEL_MODE, "PBR_SEL_MODE", getUint);
    GET_PARAM(LP_WAKEUP_MODE, "LP_WAKEUP_MODE", getUint);
    GET_PARAM(FAST_READ_EN, "FAST_READ_EN", getBool);
    GET_PARAM(FAST_READ_MODE, "FAST_READ_MODE", getUint);
    GET_PARAM(FAST_RD_CANCEL_EN, "FAST_RD_CANCEL_EN", getBool);
    GET_PARAM(FAST_RD_CANCEL_MODE, "FAST_RD_CANCEL_MODE", getUint);
    GET_PARAM(FAST_HQOS_PUSH_EN, "FAST_HQOS_PUSH_EN", getBool);
    GET_PARAM(FAST_HQOS_PUSH_MODE, "FAST_HQOS_PUSH_MODE", getUint);
    GET_PARAM(PTC_RANK_SWITCH_EN, "PTC_RANK_SWITCH_EN", getBool);
    GET_PARAM(PTC_HQOS_RANK_SWITCH_EN, "PTC_HQOS_RANK_SWITCH_EN", getBool);
    GET_PARAM(WSRAM_MAP_EN, "WSRAM_MAP_EN", getBool);
    GET_PARAM(BSC_EN, "BSC_EN", getBool);
    GET_PARAM(PTC_HQOS_RANK_SWITCH_MODE, "PTC_HQOS_RANK_SWITCH_MODE", getUint);
    GET_PARAM(PTC_HQOS_RANK_SWITCH_LEVEL1, "PTC_HQOS_RANK_SWITCH_LEVEL1", getUint);
    GET_PARAM(PTC_QOS_HOLD_EN, "PTC_QOS_HOLD_EN", getBool);
    GET_PARAM(PERF_RANK_WMERGE_EN, "PERF_RANK_WMERGE_EN", getBool);
    GET_PARAM(PERF_RANK_RMERGE_EN, "PERF_RANK_RMERGE_EN", getBool);
    GET_PARAM(PERF_READ_SCH_THH, "PERF_READ_SCH_THH", getUint);
    GET_PARAM(PERF_READ_SCH_THL, "PERF_READ_SCH_THL", getUint);
    GET_PARAM(PERF_WRITE_SCH_THH, "PERF_WRITE_SCH_THH", getUint);
    GET_PARAM(PERF_WRITE_SCH_THL, "PERF_WRITE_SCH_THL", getUint);
    GET_PARAM(PTC_RSCH_GAP_CNT, "PTC_RSCH_GAP_CNT", getUint);
    GET_PARAM(PTC_WSCH_GAP_CNT, "PTC_WSCH_GAP_CNT", getUint);
    GET_PARAM(PTC_RRANK_SWITCH_LEVEL, "PTC_RRANK_SWITCH_LEVEL", getUint);
    GET_PARAM(PTC_WRANK_SWITCH_LEVEL, "PTC_WRANK_SWITCH_LEVEL", getUint);
    GET_PARAM(PTC_EMPTY_RD_TH, "PTC_EMPTY_RD_TH", getUint);
    GET_PARAM(PTC_EMPTY_WR_TH, "PTC_EMPTY_WR_TH", getUint);
    GET_PARAM(PTC_R2W_SWITCH_TH, "PTC_R2W_SWITCH_TH", getUint);
    GET_PARAM(PTC_W2R_SWITCH_TH, "PTC_W2R_SWITCH_TH", getUint);
    GET_PARAM(OPENPAGE_TIME_RD, "OPENPAGE_TIME_RD", getUint);
    GET_PARAM(OPENPAGE_TIME_WR, "OPENPAGE_TIME_WR", getUint);
    GET_PARAM(ENH_PAGE_ADPT_EN, "ENH_PAGE_ADPT_EN", getBool);
    GET_PARAM(ENH_PAGE_ADPT_WIN, "ENH_PAGE_ADPT_WIN", getUint);
    GET_PARAM(MAP_CONFIG["ENH_PAGE_ADPT_LVL"], "ENH_PAGE_ADPT_LVL", getUintArray);
    GET_PARAM(MAP_CONFIG["ENH_PAGE_ADPT_TIME"], "ENH_PAGE_ADPT_TIME", getUintArray);
    GET_PARAM(DMC_BW_WIN, "DMC_BW_WIN", getUint);
    GET_PARAM(MAP_CONFIG["DMC_BW_LEVEL"], "DMC_BW_LEVEL", getUintArray);
    GET_PARAM(PAGE_ADPT_EN, "PAGE_ADPT_EN", getBool);
    GET_PARAM(PAGE_WIN_MODE, "PAGE_WIN_MODE", getUint);
    GET_PARAM(PAGE_WIN_SIZE, "PAGE_WIN_SIZE", getUint);
    GET_PARAM(PAGE_OPC_TH, "PAGE_OPC_TH", getUint);
    GET_PARAM(PAGE_PPC_TH, "PAGE_PPC_TH", getUint);
    GET_PARAM(PAGE_ADPT_STEP, "PAGE_ADPT_STEP", getUint);
    GET_PARAM(PAGE_TIME_MAX, "PAGE_TIME_MAX", getUint);
    GET_PARAM(DRESP_BP_TH, "DRESP_BP_TH", getUint);
    GET_PARAM(BUSYSTATE_INC_WCMD, "BUSYSTATE_INC_WCMD", getBool);
    GET_PARAM(BUSYSTATE_TH, "BUSYSTATE_TH", getUint);
    GET_PARAM(IECC_ENABLE, "IECC_ENABLE", getBool);
    GET_PARAM(PDU_DEPTH, "PDU_DEPTH", getUint);
    GET_PARAM(IECC_CONFLICT_CNT, "IECC_CONFLICT_CNT", getUint);
    GET_PARAM(IECC_PARTIAL_BYPASS, "IECC_PARTIAL_BYPASS", getBool);
    GET_PARAM(IECC_BYPASS_ADDRESS, "IECC_BYPASS_ADDRESS", getUint);
    GET_PARAM(IECC_BL32_MODE, "IECC_BL32_MODE", getBool);
    GET_PARAM(IECC_PRI, "IECC_PRI", getUint);
    GET_PARAM(IECC_CAP_RATIO, "IECC_CAP_RATIO", getUint);
    GET_PARAM(DMC_THEORY_BW, "DMC_THEORY_BW", getUint);
    GET_PARAM(FLOW_STAT_TIME, "FLOW_STAT_TIME", getUint);
    GET_PARAM(MPAM_PUSH_EN, "MPAM_PUSH_EN", getBool);
    GET_PARAM(MID_PUSH_EN, "MID_PUSH_EN", getBool);
    GET_PARAM(PD_ENABLE, "PD_ENABLE", getBool);
    GET_PARAM(PD_PRD, "PD_PRD", getUint);
    GET_PARAM(ASREF_ENABLE, "ASREF_ENABLE", getBool);
    GET_PARAM(ASREF_PRD, "ASREF_PRD", getUint);
    GET_PARAM(ASREF_ADAPT_EN, "ASREF_ADAPT_EN", getBool);
    GET_PARAM(ASREF_ADAPT_WIN, "ASREF_ADAPT_WIN", getUint);
    GET_PARAM(MAP_CONFIG["ASREF_ADAPT_LEVEL"], "ASREF_ADAPT_LEVEL", getUintArray);
    GET_PARAM(MAP_CONFIG["ASREF_ADAPT_PRD"], "ASREF_ADAPT_PRD", getUintArray);
    GET_PARAM(WCK_ALWAYS_ON, "WCK_ALWAYS_ON", getBool);
    GET_PARAM(CAS_FS_EN, "CAS_FS_EN", getBool);
    GET_PARAM(CAS_FS_TH, "CAS_FS_TH", getUint);
    GET_PARAM(CORE_CONCURR, "CORE_CONCURR", getUint);
    GET_PARAM(CORE_CONCURR_PRD, "CORE_CONCURR_PRD", getUint);
    GET_PARAM(RWGRP_TRANS_BY_TOUT, "RWGRP_TRANS_BY_TOUT", getBool);
    GET_PARAM(GRP_RANK_MODE, "GRP_RANK_MODE", getUint);
    GET_PARAM(CPU_MID_START, "CPU_MID_START", getUint);
    GET_PARAM(CPU_MID_END, "CPU_MID_END", getUint);
    GET_PARAM(RHIT_HQOS_BREAK_EN, "RHIT_HQOS_BREAK_EN", getBool);
    GET_PARAM(RHIT_HQOS_BREAK_OTH_RCMD_LEVEL, "RHIT_HQOS_BREAK_OTH_RCMD_LEVEL", getUint);
    GET_PARAM(BYP_ACT_EN, "BYP_ACT_EN", getBool);
    GET_PARAM(BYP_ACT_PIPE3_EN, "BYP_ACT_PIPE3_EN", getBool);
    GET_PARAM(PTC_RHIT_BREAK_EN, "PTC_RHIT_BREAK_EN", getBool);
    GET_PARAM(PTC_RHIT_BREAK_LEVEL, "PTC_RHIT_BREAK_LEVEL", getUint);
    GET_PARAM(TOUT_SCH_TIME, "TOUT_SCH_TIME", getUint);
    GET_PARAM(DMC_V580, "DMC_V580", getBool);
    GET_PARAM(DMC_V590, "DMC_V590", getBool);
    GET_PARAM(DMC_V596, "DMC_V596", getBool);
    GET_PARAM(TRFC_CC_EN, "TRFC_CC_EN", getBool);
    GET_PARAM(MERGE_ENABLE, "MERGE_ENABLE", getBool);
    GET_PARAM(FORWARD_ENABLE, "FORWARD_ENABLE", getBool);
    GET_PARAM(RQ_ADCONF_PUSH_EN, "RQ_ADCONF_PUSH_EN", getBool);
    GET_PARAM(CMD_ROW_ORDER, "CMD_ROW_ORDER", getUint);
    GET_PARAM(TOTAL_RCMD_MODE, "TOTAL_RCMD_MODE", getUint);
    GET_PARAM(RO_HIT_EN, "RO_HIT_EN", getBool);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL0"], "GRP_MODE_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL1"], "GRP_MODE_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL2"], "GRP_MODE_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL3"], "GRP_MODE_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["BANK_CMD_TH"], "BANK_CMD_TH", getUintArray);
    GET_PARAM(MAP_CONFIG["NO_CMD_SCH_TH"], "NO_CMD_SCH_TH", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL0"], "WR_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL1"], "WR_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL2"], "WR_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL3"], "WR_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL0"], "WR_MOST_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL1"], "WR_MOST_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL2"], "WR_MOST_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL3"], "WR_MOST_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL0"], "RD_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL1"], "RD_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL2"], "RD_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL3"], "RD_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["PERF_TIMEOUT_PRI_RD"], "PERF_TIMEOUT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["PERF_TIMEOUT_PRI_WR"], "PERF_TIMEOUT_PRI_WR", getUintArray);
    GET_PARAM(MAP_CONFIG["PERF_ADAPT_PRI_RD"], "PERF_ADAPT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["PERF_ADAPT_PRI_WR"], "PERF_ADAPT_PRI_WR", getUintArray);
    GET_PARAM(WRITE_BUFFER_ENABLE, "WRITE_BUFFER_ENABLE", getBool);
    GET_PARAM(PERF_TIMEOUT_EN, "PERF_TIMEOUT_EN", getBool);
    GET_PARAM(PERF_TIMEOUT_MODE, "PERF_TIMEOUT_MODE", getUint);
    GET_PARAM(TOUT_FORCE_RWGRP_EN, "TOUT_FORCE_RWGRP_EN", getBool);
    GET_PARAM(PERF_DUMMY_TOUT_EN, "PERF_DUMMY_TOUT_EN", getBool);
    GET_PARAM(EM_ENABLE, "EM_ENABLE", getBool);
    GET_PARAM(EM_MODE, "EM_MODE", getUint);
    GET_PARAM(RMW_ENABLE, "RMW_ENABLE", getBool);
    GET_PARAM(RMW_ENABLE_PERF, "RMW_ENABLE_PERF", getBool);
    GET_PARAM(RMW_RDATA_FIFO_DEPTH, "RMW_RDATA_FIFO_DEPTH", getUint);
    GET_PARAM(RMW_CMD_MODE, "RMW_CMD_MODE", getUint);
    GET_PARAM(RMW_QUE_DEPTH, "RMW_QUE_DEPTH", getUint);
    GET_PARAM(RMW_CONF_SIZE, "RMW_CONF_SIZE", getUint);
    GET_PARAM(ROW_SEL, "ROW_SEL", getUint);
	GET_PARAM(CONF_EN, "CONF_EN", getBool);
    GET_PARAM(SIMPLE_RWGRP_EN, "SIMPLE_RWGRP_EN", getBool);
    GET_PARAM(SIMPLE_RWGRP_MODE, "SIMPLE_RWGRP_MODE", getUint);
    GET_PARAM(PERF_GRPST_SWITCH_LVL0, "PERF_GRPST_SWITCH_LVL0", getUint);
    GET_PARAM(PERF_GRPST_SWITCH_LVL1, "PERF_GRPST_SWITCH_LVL1", getUint);
    GET_PARAM(ARB_GROUP_NUM, "ARB_GROUP_NUM", getUint);
    GET_PARAM(ACT_LEFT_CYCLE, "ACT_LEFT_CYCLE", getUint);
    GET_PARAM(PTC_RANK_NUM, "PTC_RANK_NUM", getUint);
    GET_PARAM(PTC_TIMEOUT_ENABLE, "PTC_TIMEOUT_ENABLE", getBool);
    GET_PARAM(PTC_TIMEOUT_MODE, "PTC_TIMEOUT_MODE", getUint);
    GET_PARAM(PTC_PRI_ADPT_ENABLE, "PTC_PRI_ADPT_ENABLE", getBool);
    GET_PARAM(PERF_CLK_MODE,"PERF_CLK_MODE",getUint);
    GET_PARAM(PERF_DEPTH,"PERF_DEPTH",getUint);
    GET_PARAM(PERF_SUBQ_NUM,"PERF_SUBQ_NUM",getUint);
    GET_PARAM(SUBQ_SEL_MODE,"SUBQ_SEL_MODE",getUint);
    GET_PARAM(PERF_RELEASE_REC_WIN,"PERF_RELEASE_REC_WIN",getUint);
    GET_PARAM(CLKH2CLKL,"CLKH2CLKL",getUint);
    GET_PARAM(PERF_PRI_ADAPT_ENABLE,"PERF_PRI_ADAPT_ENABLE",getBool);
    GET_PARAM(PERF_QOS_POLICY,"PERF_QOS_POLICY",getUint);
    GET_PARAM(QOS_MAX,"QOS_MAX",getUint);
    GET_PARAM(PERF_ADAPT_PRI_MAX,"PERF_ADAPT_PRI_MAX",getUint);
    GET_PARAM(PTC_MAX_RANK,"PTC_MAX_RANK",getUint);
    GET_PARAM(PTC_QOS_MAX,"PTC_QOS_MAX",getUint);
    GET_PARAM(PTC_ADAPT_PRI_MAX,"PTC_ADAPT_PRI_MAX",getUint);
    GET_PARAM(PTC_MAX_CNT_PER_RANK,"PTC_MAX_CNT_PER_RANK",getUint);
    GET_PARAM(SAME_BANK_CNT_RD,"SAME_BANK_CNT_RD",getUint);
    GET_PARAM(SAME_BANK_CNT_WR,"SAME_BANK_CNT_WR",getUint);
    GET_PARAM(SAME_BANK_CNT_ROWMISS,"SAME_BANK_CNT_ROWMISS",getUint);
    GET_PARAM(PERF_ROWHIT_BREAK_TH,"PERF_ROWHIT_BREAK_TH",getUint);
    GET_PARAM(IDLE_TRIG_TIME,"IDLE_TRIG_TIME",getUint);
    GET_PARAM(GAP_CNT_BASE,"GAP_CNT_BASE",getUint);
    GET_PARAM(GAP_CNT_TIMES,"GAP_CNT_TIMES",getUint);
    GET_PARAM(SAME_BG_CNT_RD,"SAME_BG_CNT_RD",getUint);
    GET_PARAM(SAME_BG_CNT_WR,"SAME_BG_CNT_WR",getUint);
    GET_PARAM(BG_PRE_N_CMD,"BG_PRE_N_CMD",getUint);
    GET_PARAM(RANK_GROUP_MODE,"RANK_GROUP_MODE",getUint);
    GET_PARAM(PERF_RWGRP_MODE,"PERF_RWGRP_MODE",getUint);
    GET_PARAM(PERF_NG_HOLD_WR_EN,"PERF_NG_HOLD_WR_EN",getBool);
    GET_PARAM(PERF_NG_HOLD_WR_MODE,"PERF_NG_HOLD_WR_MODE",getUint);
    GET_PARAM(PERF_NG_HOLD_WR_CNT,"PERF_NG_HOLD_WR_CNT",getUint);
    GET_PARAM(PERF_FAST_R2W_EN,"PERF_FAST_R2W_EN",getBool);
    GET_PARAM(PERF_RCMD_HQOS_W2R_SWITCH_EN,"PERF_RCMD_HQOS_W2R_SWITCH_EN",getBool);
    GET_PARAM(PERF_RCMD_HQOS_W2R_RLEVELH,"PERF_RCMD_HQOS_W2R_RLEVELH",getUint);
    GET_PARAM(WDDA,"WDDA",getUint);
    GET_PARAM(PERF_SWITCH_HQOS_LEVEL,"PERF_SWITCH_HQOS_LEVEL",getUint);
    GET_PARAM(PERF_SCH_RRANK_LEVELH,"PERF_SCH_RRANK_LEVELH",getUint);
    GET_PARAM(PERF_SCH_RRANK_LEVELL,"PERF_SCH_RRANK_LEVELL",getUint);
    GET_PARAM(PERF_HOLD_RRANK_LEVELH,"PERF_HOLD_RRANK_LEVELH",getUint);
    GET_PARAM(PERF_HOLD_RRANK_LEVELL,"PERF_HOLD_RRANK_LEVELL",getUint);
    GET_PARAM(PERF_SCH_WRANK_LEVELH,"PERF_SCH_WRANK_LEVELH",getUint);
    GET_PARAM(PERF_SCH_WRANK_LEVELL,"PERF_SCH_WRANK_LEVELL",getUint);
    GET_PARAM(PERF_HOLD_WRANK_LEVELH,"PERF_HOLD_WRANK_LEVELH",getUint);
    GET_PARAM(PERF_HOLD_WRANK_LEVELL,"PERF_HOLD_WRANK_LEVELL",getUint);
    GET_PARAM(PERF_RRNK_ENGRP_LEVEL,"PERF_RRNK_ENGRP_LEVEL",getUint);
    GET_PARAM(PERF_RRNK_EXGRP_LEVEL,"PERF_RRNK_EXGRP_LEVEL",getUint);
    GET_PARAM(PERF_WRNK_ENGRP_LEVEL,"PERF_WRNK_ENGRP_LEVEL",getUint);
    GET_PARAM(PERF_WRNK_EXGRP_LEVEL,"PERF_WRNK_EXGRP_LEVEL",getUint);
    GET_PARAM(PERF_ENGRP_LEVEL,"PERF_ENGRP_LEVEL",getUint);
    GET_PARAM(PERF_EXGRP_LEVEL,"PERF_EXGRP_LEVEL",getUint);
    GET_PARAM(PERF_CMD_WLEVELH,"PERF_CMD_WLEVELH",getUint);
    GET_PARAM(PERF_ACT_TCHK_EN, "PERF_ACT_TCHK_EN", getBool);
    GET_PARAM(RPFIFO_EN, "RPFIFO_EN", getBool);
    GET_PARAM(RPFIFO_DEPTH, "RPFIFO_DEPTH", getUint);
    GET_PARAM(RPFIFO_AMFULL_TH, "RPFIFO_AMFULL_TH", getUint);
    GET_PARAM(ODD_TDM, "ODD_TDM", getBool);
    GET_PARAM(PREACT_FLOW_CTRL_EN, "PREACT_FLOW_CTRL_EN", getBool);
    GET_PARAM(PREACT_FLOW_CTRL_TYPE, "PREACT_FLOW_CTRL_TYPE", getUint);
    GET_PARAM(SIMPLE_BANKTABLE_ENABLE, "SIMPLE_BANKTABLE_ENABLE", getBool);
    GET_PARAM(PREACT_TBL_THD, "PREACT_TBL_THD", getUint);
    GET_PARAM(PREACT_BW_THD, "PREACT_BW_THD", getUint);
    GET_PARAM(TABLE_DEPTH, "TABLE_DEPTH", getUint);
    GET_PARAM(SID_LOOSE, "SID_LOOSE", getBool);
    GET_PARAM(SIMPLE_GRP_SID_EN, "SIMPLE_GRP_SID_EN", getBool);
    GET_PARAM(TIMEOUT_SID, "TIMEOUT_SID", getUint);
    GET_PARAM(TIMEOUT_SID_ENABLE, "TIMEOUT_SID_ENABLE", getBool);
    GET_PARAM(ENGRP_LR_LEVEL, "ENGRP_LR_LEVEL", getUint);
    GET_PARAM(EX_LR_LEVEL, "EX_LR_LEVEL", getUint);
    GET_PARAM(LR_LEVELH, "LR_LEVELH", getUint);
    GET_PARAM(LR_LEVELL, "LR_LEVELL", getUint);
    GET_PARAM(SERIAL_LR_LEVELH, "SERIAL_LR_LEVELH", getUint);
    GET_PARAM(SERIAL_LR_LEVELL, "SERIAL_LR_LEVELL", getUint); 
    GET_PARAM(SID_GRP_PIPE, "SID_GRP_PIPE", getUint);
    GET_PARAM(ROW_SEL_MSB, "ROW_SEL_MSB", getUint);
    GET_PARAM(ROW_SEL_SMSB, "ROW_SEL_SMSB", getUint);
    GET_PARAM(ADDR_EXP_EN, "ADDR_EXP_EN", getBool);
    GET_PARAM(RAND_BABG, "RAND_BABG", getBool);
    GET_PARAM(FORCE_SID_SWITCH, "FORCE_SID_SWITCH", getBool);
    GET_PARAM(SID_SW_RATIO, "SID_SW_RATIO", getUint);

    if (SYSTEM_CONFIG=="ddrsystem") IniFilename = IniFilePath + "/" + "DDR/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="lpsystem")  IniFilename = IniFilePath + "/" + "LP/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="lp6system") IniFilename = IniFilePath + "/" + "LP/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="hbmsystem") {
        if (ZHUQUE_ENABLE) {
            IniFilename = IniFilePath + "/" + "HBM/" + to_string(ZHUQUE_BA_MODE) + "BA_zhuque_" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
        } else {
            IniFilename = IniFilePath + "/" + "HBM/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
        }
    }
    // if (SYSTEM_CONFIG=="zhuque_hbmsystem") IniFilename = IniFilePath + "/" + "HBM/" + to_string(ZHUQUE_BA_MODE) + "BA_zhuque_" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (SYSTEM_CONFIG=="wsesystem") IniFilename = IniFilePath + "/" + "WSE/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    // IniFilename = IniFilePath+"/"+ "LP/" + DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (hhaId == 0) DEBUG("== Loading device model file '"<<IniFilename<<"' ==");
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        IniReader::ReadIniFile(iniFile, false);
    } else {
        ERROR("Unable to load ini file "<<IniFilename);
        abort();
    }
    iniFile.close();

    //TimingInit();
    //TimingCalc();

    GET_PARAM(JEDEC_DATA_BUS_BITS, "JEDEC_DATA_BUS_BITS", getUint);
    GET_PARAM(DMC_DATA_BUS_BITS, "DMC_DATA_BUS_BITS", getUint);
    GET_PARAM(RDATA_RETURN_NUM, "RDATA_RETURN_NUM", getUint);
    GET_PARAM(WCK2DFI_RATIO, "WCK2DFI_RATIO", getFloat);
    GET_PARAM(OFREQ_EN, "OFREQ_EN", getBool);
    GET_PARAM(OFREQ_RATIO, "OFREQ_RATIO", getFloat);
    GET_PARAM(PAM_RATIO, "PAM_RATIO", getFloat);
    GET_PARAM(tDFI, "tDFI", getFloat);
    GET_PARAM(MAP_CONFIG["BL"], "BL", getUintArray);
    GET_PARAM(tREFI, "tREFI", getUint);
    GET_PARAM(WL, "WL", getUint);
    GET_PARAM(RL, "RL", getUint);
    GET_PARAM(tRAS, "tRAS", getUint);
    GET_PARAM(tRCD_WR, "tRCD_WR", getUint);
    GET_PARAM(tRCD, "tRCD", getUint);
    GET_PARAM(tRRD_L, "tRRD_L", getUint);
    GET_PARAM(tRRD_S, "tRRD_S", getUint);
    GET_PARAM(tRRD_Sdlr, "tRRD_Sdlr", getUint);
    GET_PARAM(tRPpb, "tRPpb", getUint);
    GET_PARAM(tRPab, "tRPab", getUint);
    GET_PARAM(tRPfg, "tRPfg", getUint);
    GET_PARAM(tPPD, "tPPD", getUint);
    GET_PARAM(tPPD_L, "tPPD_L", getUint);
    GET_PARAM(tFPW, "tFPW", getUint);
    GET_PARAM(tCCD_S, "tCCD_S", getUint);
    GET_PARAM(tCCD_L, "tCCD_L", getUint);
    GET_PARAM(tCCD_R, "tCCD_R", getUint);
    GET_PARAM(tCCD_M, "tCCD_M", getUint);
    GET_PARAM(tCCD_M_WR, "tCCD_M_WR", getUint);
    GET_PARAM(tCCD_L_WR, "tCCD_L_WR", getUint);
    GET_PARAM(tCCD_L_WR2, "tCCD_L_WR2", getUint);
    GET_PARAM(tCCD_NSR, "tCCD_NSR", getUint);
    GET_PARAM(tCCD_NSW, "tCCD_NSW", getUint);
    GET_PARAM(tCCDMW, "tCCDMW", getUint);
    GET_PARAM(tCCD_L24, "tCCD_L24", getUint);
    GET_PARAM(tCCD_L48, "tCCD_L48", getUint);
    GET_PARAM(PCFG_TWR, "PCFG_TWR", getUint);
    GET_PARAM(PCFG_TRTP, "PCFG_TRTP", getUint);
    GET_PARAM(PCFG_TRTW, "PCFG_TRTW", getUint);
    GET_PARAM(PCFG_TRTW_L, "PCFG_TRTW_L", getUint);
    GET_PARAM(PCFG_TWTR, "PCFG_TWTR", getUint);
    GET_PARAM(PCFG_TWTR_L, "PCFG_TWTR_L", getUint);
    GET_PARAM(PCFG_TWTR_SB, "PCFG_TWTR_SB", getUint);
    GET_PARAM(PCFG_RANKTRTR, "PCFG_RANKTRTR", getUint);
    GET_PARAM(PCFG_RANKTRTW, "PCFG_RANKTRTW", getUint);
    GET_PARAM(PCFG_RANKTWTR, "PCFG_RANKTWTR", getUint);
    GET_PARAM(PCFG_RANKTWTW, "PCFG_RANKTWTW", getUint);
    GET_PARAM(tFAW, "tFAW", getUint);
    GET_PARAM(tCMD2SCH, "tCMD2SCH", getUint);
    GET_PARAM(tCMD2SCH_BYPACT, "tCMD2SCH_BYPACT", getUint);
    GET_PARAM(tPERF2PTC, "tPERF2PTC", getUint);
    GET_PARAM(tPERF_CONF, "tPERF_CONF", getUint);
    GET_PARAM(tCMD_CONF, "tCMD_CONF", getUint);
    GET_PARAM(tD_D, "tD_D", getUint);
    GET_PARAM(tPIPE_PRE_DMC, "tPIPE_PRE_DMC", getUint);
    GET_PARAM(tCMD_PHY, "tCMD_PHY", getUint);
    GET_PARAM(tDAT_PHY, "tDAT_PHY", getUint);
    GET_PARAM(tCMD_RASC, "tCMD_RASC", getUint);
    GET_PARAM(tDAT_RASC, "tDAT_RASC", getUint);
    GET_PARAM(tCMD_ADAPT, "tCMD_ADAPT", getUint);
    GET_PARAM(tWDATA_DMC, "tWDATA_DMC", getUint);
    GET_PARAM(tRFCab, "tRFCab", getUint);
    GET_PARAM(tRFCpb, "tRFCpb", getUint);
    GET_PARAM(tPBR2PBR, "tPBR2PBR", getUint);
    GET_PARAM(tPBR2PBR_L, "tPBR2PBR_L", getUint);
    GET_PARAM(tPBR2ACT, "tPBR2ACT", getUint);
    GET_PARAM(tCMD_WAKEUP, "tCMD_WAKEUP", getUint);
    GET_PARAM(tXP_V570, "tXP_V570", getUint);
    GET_PARAM(tXP_V580, "tXP_V580", getUint);
    GET_PARAM(tXP_V590, "tXP_V590", getUint);
    GET_PARAM(tXSR, "tXSR", getUint);
    GET_PARAM(tCMDPD, "tCMDPD", getUint);
    GET_PARAM(tnACU, "tnACU", getUint);
    GET_PARAM(tRDPD, "tRDPD", getUint);
    GET_PARAM(tRDAPPD, "tRDAPPD", getUint);
    GET_PARAM(tWRPD, "tWRPD", getUint);
    GET_PARAM(tWRAPPD, "tWRAPPD", getUint);
    GET_PARAM(tPDE, "tPDE", getUint);
    GET_PARAM(tCSPD, "tCSPD", getUint);
    GET_PARAM(tPDLP, "tPDLP", getUint);
    GET_PARAM(tPHYLPE, "tPHYLPE", getUint);
    GET_PARAM(tPHYLPX, "tPHYLPX", getUint);
    GET_PARAM(tASREFE, "tASREFE", getUint);
    GET_PARAM(ABR_PSTPND_LEVEL, "ABR_PSTPND_LEVEL", getUint);
    GET_PARAM(PBR_PSTPND_LEVEL, "PBR_PSTPND_LEVEL", getUint);
    GET_PARAM(PRE_PBR_PSTPND_LEVEL, "PRE_PBR_PSTPND_LEVEL", getUint);
    GET_PARAM(PRE_NUM_SEND_PBR, "PRE_NUM_SEND_PBR", getUint);
    GET_PARAM(POWER_RDINC_K, "POWER_RDINC_K", getFloat);
    GET_PARAM(POWER_RDWRAP_K, "POWER_RDWRAP_K", getFloat);
    GET_PARAM(POWER_WRINC_K, "POWER_WRINC_K", getFloat);
    GET_PARAM(POWER_WRWRAP_K, "POWER_WRWRAP_K", getFloat);
    GET_PARAM(POWER_RDATA_K, "POWER_RDATA_K", getFloat);
    GET_PARAM(POWER_WDATA_K, "POWER_WDATA_K", getFloat);
    GET_PARAM(POWER_ACT_K, "POWER_ACT_K", getFloat);
    GET_PARAM(POWER_PREP_K, "POWER_PREP_K", getFloat);
    GET_PARAM(POWER_PRES_K, "POWER_PRES_K", getFloat);
    GET_PARAM(POWER_PREA_K, "POWER_PREA_K", getFloat);
    GET_PARAM(MAP_CONFIG["POWER_RD_K"], "POWER_RD_K", getUintArray);
    GET_PARAM(MAP_CONFIG["POWER_WR_K"], "POWER_WR_K", getUintArray);
    GET_PARAM(POWER_PBR_K, "POWER_PBR_K", getFloat);
    GET_PARAM(POWER_ABR_K, "POWER_ABR_K", getFloat);
    GET_PARAM(POWER_R2W_K, "POWER_R2W_K", getFloat);
    GET_PARAM(POWER_W2R_K, "POWER_W2R_K", getFloat);
    GET_PARAM(POWER_RNKSW_K, "POWER_RNKSW_K", getFloat);
    GET_PARAM(POWER_PDE_K, "POWER_PDE_K", getFloat);
    GET_PARAM(POWER_ASREFE_K, "POWER_ASREFE_K", getFloat);
    GET_PARAM(POWER_SRPDE_K, "POWER_SRPDE_K", getFloat);
    GET_PARAM(POWER_PDX_K, "POWER_PDX_K", getFloat);
    GET_PARAM(POWER_ASREFX_K, "POWER_ASREFX_K", getFloat);
    GET_PARAM(POWER_SRPDX_K, "POWER_SRPDX_K", getFloat);
    GET_PARAM(POWER_IDLE_K, "POWER_IDLE_K", getFloat);
    GET_PARAM(POWER_PDCC_K, "POWER_PDCC_K", getFloat);
    GET_PARAM(MAP_CONFIG["POWER_QUEUE_K"], "POWER_QUEUE_K", getUintArray);
    GET_PARAM(MATRIX_ROW23, "MATRIX_ROW23", getUint64);
    GET_PARAM(MATRIX_ROW22, "MATRIX_ROW22", getUint64);
    GET_PARAM(MATRIX_ROW21, "MATRIX_ROW21", getUint64);
    GET_PARAM(MATRIX_ROW20, "MATRIX_ROW20", getUint64);
    GET_PARAM(MATRIX_ROW19, "MATRIX_ROW19", getUint64);
    GET_PARAM(MATRIX_ROW18, "MATRIX_ROW18", getUint64);
    GET_PARAM(MATRIX_ROW17, "MATRIX_ROW17", getUint64);
    GET_PARAM(MATRIX_ROW16, "MATRIX_ROW16", getUint64);
    GET_PARAM(MATRIX_ROW15, "MATRIX_ROW15", getUint64);
    GET_PARAM(MATRIX_ROW14, "MATRIX_ROW14", getUint64);
    GET_PARAM(MATRIX_ROW13, "MATRIX_ROW13", getUint64);
    GET_PARAM(MATRIX_ROW12, "MATRIX_ROW12", getUint64);
    GET_PARAM(MATRIX_ROW11, "MATRIX_ROW11", getUint64);
    GET_PARAM(MATRIX_ROW10, "MATRIX_ROW10", getUint64);
    GET_PARAM(MATRIX_ROW9, "MATRIX_ROW9", getUint64);
    GET_PARAM(MATRIX_ROW8, "MATRIX_ROW8", getUint64);
    GET_PARAM(MATRIX_ROW7, "MATRIX_ROW7", getUint64);
    GET_PARAM(MATRIX_ROW6, "MATRIX_ROW6", getUint64);
    GET_PARAM(MATRIX_ROW5, "MATRIX_ROW5", getUint64);
    GET_PARAM(MATRIX_ROW4, "MATRIX_ROW4", getUint64);
    GET_PARAM(MATRIX_ROW3, "MATRIX_ROW3", getUint64);
    GET_PARAM(MATRIX_ROW2, "MATRIX_ROW2", getUint64);
    GET_PARAM(MATRIX_ROW1, "MATRIX_ROW1", getUint64);
    GET_PARAM(MATRIX_ROW0, "MATRIX_ROW0", getUint64);
    GET_PARAM(MATRIX_CH,  "MATRIX_CH", getUint64);
    GET_PARAM(MATRIX_RA2, "MATRIX_RA2", getUint64);
    GET_PARAM(MATRIX_RA1, "MATRIX_RA1", getUint64);
    GET_PARAM(MATRIX_RA0, "MATRIX_RA0", getUint64);
    GET_PARAM(MATRIX_BA6, "MATRIX_BA6", getUint64);
    GET_PARAM(MATRIX_BA5, "MATRIX_BA5", getUint64);
    GET_PARAM(MATRIX_BA4, "MATRIX_BA4", getUint64);
    GET_PARAM(MATRIX_BA3, "MATRIX_BA3", getUint64);
    GET_PARAM(MATRIX_BA2, "MATRIX_BA2", getUint64);
    GET_PARAM(MATRIX_BA1, "MATRIX_BA1", getUint64);
    GET_PARAM(MATRIX_BA0, "MATRIX_BA0", getUint64);
    GET_PARAM(MATRIX_BG4, "MATRIX_BG4", getUint64);
    GET_PARAM(MATRIX_BG3, "MATRIX_BG3", getUint64);
    GET_PARAM(MATRIX_BG2, "MATRIX_BG2", getUint64);
    GET_PARAM(MATRIX_BG1, "MATRIX_BG1", getUint64);
    GET_PARAM(MATRIX_BG0, "MATRIX_BG0", getUint64);
    GET_PARAM(MATRIX_COL10, "MATRIX_COL10", getUint64);
    GET_PARAM(MATRIX_COL9, "MATRIX_COL9", getUint64);
    GET_PARAM(MATRIX_COL8, "MATRIX_COL8", getUint64);
    GET_PARAM(MATRIX_COL7, "MATRIX_COL7", getUint64);
    GET_PARAM(MATRIX_COL6, "MATRIX_COL6", getUint64);
    GET_PARAM(MATRIX_COL5, "MATRIX_COL5", getUint64);
    GET_PARAM(MATRIX_COL4, "MATRIX_COL4", getUint64);
    GET_PARAM(MATRIX_COL3, "MATRIX_COL3", getUint64);
    GET_PARAM(MATRIX_COL2, "MATRIX_COL2", getUint64);
    GET_PARAM(MATRIX_COL1, "MATRIX_COL1", getUint64);
    GET_PARAM(MATRIX_COL0, "MATRIX_COL0", getUint64);

    CalcMatrixNum();
    if (SYSTEM_CONFIG=="ddrsystem") IniFilename = IniFilePath + "/" + "DDR/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="lpsystem")  IniFilename = IniFilePath + "/" + "LP/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="lp6system") IniFilename = IniFilePath + "/" + "LP/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="hbmsystem") IniFilename = IniFilePath + "/" + "HBM/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (SYSTEM_CONFIG=="wsesystem") IniFilename = IniFilePath + "/" + "WSE/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    // IniFilename = IniFilePath + "/" + "LP/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
    //         "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        if (hhaId == 0) DEBUG("== Loading DRAM power model file '"<<IniFilename<<"' ==");
        IniReader::ReadIniFile(iniFile, false);
        DRAM_POWER_EN = true;
    } else {
        if (hhaId == 0) DEBUG("== Unable to load "<<IniFilename<<", DRAM power model not enable ==");
        DRAM_POWER_EN = false;
    }
    iniFile.close();

    GET_PARAM(IDD01, "IDD01", getFloat);
    GET_PARAM(IDD02H, "IDD02H", getFloat);
    GET_PARAM(IDD02L, "IDD02L", getFloat);
    GET_PARAM(IDD0Q, "IDD0Q", getFloat);
    GET_PARAM(IDD2P1, "IDD2P1", getFloat);
    GET_PARAM(IDD2P2H, "IDD2P2H", getFloat);
    GET_PARAM(IDD2P2L, "IDD2P2L", getFloat);
    GET_PARAM(IDD2PQ, "IDD2PQ", getFloat);
    GET_PARAM(IDD2PS1, "IDD2PS1", getFloat);
    GET_PARAM(IDD2PS2H, "IDD2PS2H", getFloat);
    GET_PARAM(IDD2PS2L, "IDD2PS2L", getFloat);
    GET_PARAM(IDD2PSQ, "IDD2PSQ", getFloat);
    GET_PARAM(IDD2N1, "IDD2N1", getFloat);
    GET_PARAM(IDD2N2H, "IDD2N2H", getFloat);
    GET_PARAM(IDD2N2L, "IDD2N2L", getFloat);
    GET_PARAM(IDD2NQ, "IDD2NQ", getFloat);
    GET_PARAM(IDD2NS1, "IDD2NS1", getFloat);
    GET_PARAM(IDD2NS2H, "IDD2NS2H", getFloat);
    GET_PARAM(IDD2NS2L, "IDD2NS2L", getFloat);
    GET_PARAM(IDD2NSQ, "IDD2NSQ", getFloat);
    GET_PARAM(IDD3P1, "IDD3P1", getFloat);
    GET_PARAM(IDD3P2H, "IDD3P2H", getFloat);
    GET_PARAM(IDD3P2L, "IDD3P2L", getFloat);
    GET_PARAM(IDD3PQ, "IDD3PQ", getFloat);
    GET_PARAM(IDD3PS1, "IDD3PS1", getFloat);
    GET_PARAM(IDD3PS2H, "IDD3PS2H", getFloat);
    GET_PARAM(IDD3PS2L, "IDD3PS2L", getFloat);
    GET_PARAM(IDD3PSQ, "IDD3PSQ", getFloat);
    GET_PARAM(IDD3N1, "IDD3N1", getFloat);
    GET_PARAM(IDD3N2H, "IDD3N2H", getFloat);
    GET_PARAM(IDD3N2L, "IDD3N2L", getFloat);
    GET_PARAM(IDD3NQ, "IDD3NQ", getFloat);
    GET_PARAM(IDD3NS1, "IDD3NS1", getFloat);
    GET_PARAM(IDD3NS2H, "IDD3NS2H", getFloat);
    GET_PARAM(IDD3NS2L, "IDD3NS2L", getFloat);
    GET_PARAM(IDD3NSQ, "IDD3NSQ", getFloat);
    GET_PARAM(IDD4R1_BG, "IDD4R1_BG", getFloat);
    GET_PARAM(IDD4R2H_BG, "IDD4R2H_BG", getFloat);
    GET_PARAM(IDD4R2L_BG, "IDD4R2L_BG", getFloat);
    GET_PARAM(IDD4RQ_BG, "IDD4RQ_BG", getFloat);
    GET_PARAM(IDD4R1_BK, "IDD4R1_BK", getFloat);
    GET_PARAM(IDD4R2H_BK, "IDD4R2H_BK", getFloat);
    GET_PARAM(IDD4R2L_BK, "IDD4R2L_BK", getFloat);
    GET_PARAM(IDD4RQ_BK, "IDD4RQ_BK", getFloat);
    GET_PARAM(IDD4W1_BG, "IDD4W1_BG", getFloat);
    GET_PARAM(IDD4W2H_BG, "IDD4W2H_BG", getFloat);
    GET_PARAM(IDD4W2L_BG, "IDD4W2L_BG", getFloat);
    GET_PARAM(IDD4WQ_BG, "IDD4WQ_BG", getFloat);
    GET_PARAM(IDD4W1_BK, "IDD4W1_BK", getFloat);
    GET_PARAM(IDD4W2H_BK, "IDD4W2H_BK", getFloat);
    GET_PARAM(IDD4W2L_BK, "IDD4W2L_BK", getFloat);
    GET_PARAM(IDD4WQ_BK, "IDD4WQ_BK", getFloat);
    GET_PARAM(IDD51, "IDD51", getFloat);
    GET_PARAM(IDD52H, "IDD52H", getFloat);
    GET_PARAM(IDD52L, "IDD52L", getFloat);
    GET_PARAM(IDD5Q, "IDD5Q", getFloat);
    GET_PARAM(IDD5AB1, "IDD5AB1", getFloat);
    GET_PARAM(IDD5AB2H, "IDD5AB2H", getFloat);
    GET_PARAM(IDD5AB2L, "IDD5AB2L", getFloat);
    GET_PARAM(IDD5ABQ, "IDD5ABQ", getFloat);
    GET_PARAM(IDD5PB1, "IDD5PB1", getFloat);
    GET_PARAM(IDD5PB2H, "IDD5PB2H", getFloat);
    GET_PARAM(IDD5PB2L, "IDD5PB2L", getFloat);
    GET_PARAM(IDD5PBQ, "IDD5PBQ", getFloat);
    GET_PARAM(IDD61, "IDD61", getFloat);
    GET_PARAM(IDD62H, "IDD62H", getFloat);
    GET_PARAM(IDD62L, "IDD62L", getFloat);
    GET_PARAM(IDD6Q, "IDD6Q", getFloat);
    GET_PARAM(IDD6DS1, "IDD6DS1", getFloat);
    GET_PARAM(IDD6DS2H, "IDD6DS2H", getFloat);
    GET_PARAM(IDD6DS2L, "IDD6DS2L", getFloat);
    GET_PARAM(IDD6DSQ, "IDD6DSQ", getFloat);
    GET_PARAM(VDD1, "VDD1", getFloat);
    GET_PARAM(VDD2H, "VDD2H", getFloat);
    GET_PARAM(VDD2L, "VDD2L", getFloat);
    GET_PARAM(VDDQH, "VDDQH", getFloat);
    GET_PARAM(VDDQL, "VDDQL", getFloat);
#endif
    channel_ohot = 0;
    IniReader::ModifyParameter(cfg);
    CalcMatrixNum();
    // If we have any overrides, set them now before creating all of the memory objects
    IniReader::InitEnumsFromStrings();
    if (DRAM_POWER_EN && !IniReader::CheckIfAllSet()) {
        assert(0);
    }
    IniReader::CheckParameter();

    DDRSim_log.resize(NUM_CHANS);
    DDRSim_iecc_log.resize(NUM_PFQS);
    DDRSim_pfq_log.resize(NUM_PFQS);
    DDRSim_ptc_log.resize(NUM_CHANS);
    state_log.resize(NUM_CHANS);
    trace_log.resize(NUM_CHANS);
    cmdnum_log.resize(NUM_CHANS);
    dram_log.resize(NUM_CHANS);
    for (unsigned channel = 0; channel < NUM_CHANS; channel++) {
        InitOutputFiles(channel);
    }

    write_map.clear();

    mptc_ = std::make_unique<MPTC>(this, DDRSim_ptc_log, trace_log, cmdnum_log, dram_log);
    mpfq_ = std::make_unique<MPFQ>(this, DDRSim_pfq_log, trace_log);
    mras_ = std::make_unique<MRAS>(this, DDRSim_iecc_log, trace_log, cmdnum_log);

    initialize();

    string ddr_type;
    ddr_type = DDR_TYPE;
    transform(ddr_type.begin(), ddr_type.end(), ddr_type.begin(), ::tolower);
    if (hhaId == 0) DEBUG("== "<<ddr_type<<" "<<DMC_RATE<<" Mbps, "<<NUM_CHANS<<" Channels, "<<NUM_RANKS
            <<" Ranks, Gbuf "<<boolalpha<<WRITE_BUFFER_ENABLE<<", RMW "<<RMW_ENABLE<<", RMW_PERF "<<RMW_ENABLE_PERF<<", IECC "<<IECC_ENABLE<<noboolalpha<<" ==");

#ifndef OLD_STATES_MODE   
    start_cycle = 0;
    end_cycle = 0;
    flow_statis_start_cycle = 0;
    flow_statis_end_cycle = 0;

    curFlowPressureLevel.resize(NUM_CHANS, 0);

    task_cnt.resize(NUM_CHANS, 0);
    total_task_cnt.resize(NUM_CHANS, 0);
    enable_statistics = false;

    access_cnt.resize(NUM_CHANS, 0);
    bp_cnt.resize(NUM_CHANS, 0);
    total_access_cnt.resize(NUM_CHANS, 0);
    total_bp_cnt.resize(NUM_CHANS, 0);

    trans_fifo_data_cnt = 0;
    trans_fifo_full = false;

    que_cnt.resize(NUM_CHANS, vector<uint32_t>(TRANS_QUEUE_DEPTH + 1, 0));
    que_rd_cnt.resize(NUM_CHANS, vector<uint32_t>(TRANS_QUEUE_DEPTH + 1, 0));
    que_wr_cnt.resize(NUM_CHANS, vector<uint32_t>(TRANS_QUEUE_DEPTH + 1, 0));
    perf_que_cnt.resize(NUM_CHANS, vector<uint32_t>(PERF_DEPTH + 1, 0));

    occ_1_cnt = 0;
    occ_2_cnt = 0;
    occ_3_cnt = 0;
    occ_4_cnt = 0;

    pre_sch_level_cnt.clear(); 
    pre_sch_level_cnt.resize(NUM_CHANS, vector<unsigned>(7, 0));

    trans_baintlv.resize(NUM_RANKS, vector<unsigned>(2,0));

    pre_merge_read_cnt.resize(NUM_CHANS, 0);
    pre_fast_read_cnt.resize(NUM_CHANS, 0);
    pre_fast_act_cnt.resize(NUM_CHANS, 0);
    pre_ecc_read_cnt.resize(NUM_CHANS, 0);
    pre_ecc_write_cnt.resize(NUM_CHANS, 0);
    pre_reads.resize(NUM_CHANS, 0);
    pre_writes.resize(NUM_CHANS, 0);
    pre_totals.resize(NUM_CHANS, 0);
    pre_gbuf_reads.resize(NUM_CHANS, 0);
    pre_gbuf_writes.resize(NUM_CHANS, 0);
    pre_address_conf_cnt .resize(NUM_CHANS, 0);
    pre_perf_address_conf_cnt .resize(NUM_CHANS, 0);
    pre_id_conf_cnt.resize(NUM_CHANS, 0);
    pre_ba_conf_cnt.resize(NUM_CHANS, 0);
    pre_total_conf.resize(NUM_CHANS, 0);
    pre_act_cnt.resize(NUM_CHANS, 0);
    pre_act_dst_cnt.resize(NUM_CHANS, 0);
    pre_pre_sb_cnt.resize(NUM_CHANS, 0);
    pre_pre_pb_cnt.resize(NUM_CHANS, 0);
    pre_pre_pb_dst_cnt.resize(NUM_CHANS, 0);
    pre_pre_ab_cnt.resize(NUM_CHANS, 0);
    pre_read_cnt.resize(NUM_CHANS, 0);
    pre_write_cnt.resize(NUM_CHANS, 0);
    pre_read_p_cnt.resize(NUM_CHANS, 0);
    pre_write_p_cnt.resize(NUM_CHANS, 0);
    pre_mwrite_cnt.resize(NUM_CHANS, 0);
    pre_mwrite_p_cnt.resize(NUM_CHANS, 0);
    pre_timeout_cnt.resize(NUM_CHANS, 0);
    pre_fast_timeout_cnt.resize(NUM_CHANS, 0);
    pre_slow_timeout_cnt.resize(NUM_CHANS, 0);
    pre_rt_timeout_cnt.resize(NUM_CHANS, 0);
    pre_perf_rd_timeout_cnt.resize(NUM_CHANS, 0);
    pre_perf_wr_timeout_cnt.resize(NUM_CHANS, 0);
    pre_dummy_timeout_cnt.resize(NUM_CHANS, 0);
    pre_dummy_hqos_cnt.resize(NUM_CHANS, 0);
    pre_row_hit_cnt.resize(NUM_CHANS, 0);
    pre_row_miss_cnt.resize(NUM_CHANS, 0);
    pre_rw_switch_cnt.resize(NUM_CHANS, 0);
    pre_rank_switch_cnt.resize(NUM_CHANS, 0);
    pre_r_rank_switch_cnt.resize(NUM_CHANS, 0);
    pre_w_rank_switch_cnt.resize(NUM_CHANS, 0);
    pre_sid_switch_cnt.resize(NUM_CHANS, 0);
    pre_rw_idle_cnt.resize(NUM_CHANS, 0);
    pre_refresh_pb_cnt.resize(NUM_CHANS, 0);
    pre_refresh_ab_cnt.resize(NUM_CHANS, 0);
    pre_r2w_switch_cnt.resize(NUM_CHANS, 0);
    pre_w2r_switch_cnt.resize(NUM_CHANS, 0);
    pre_phy_notlp_cnt.resize(NUM_CHANS, 0);
    pre_phy_lp_cnt.resize(NUM_CHANS, 0);
    pre_power.resize(NUM_CHANS, 0);
    pre_pde_cnt.resize(NUM_CHANS, 0);
    pre_asrefe_cnt.resize(NUM_CHANS, 0);
    pre_srpde_cnt.resize(NUM_CHANS, 0);
    pre_pdx_cnt.resize(NUM_CHANS, 0);
    pre_asrefx_cnt.resize(NUM_CHANS, 0);
    pre_srpdx_cnt.resize(NUM_CHANS, 0);
    PreRdCntBl.clear();   PreWrCntBl.clear();
    PreRdCntBl[BL8] = 0;  PreWrCntBl[BL8] = 0;
    PreRdCntBl[BL16] = 0; PreWrCntBl[BL16] = 0;
    PreRdCntBl[BL24] = 0; PreWrCntBl[BL24] = 0;
    PreRdCntBl[BL32] = 0; PreWrCntBl[BL32] = 0;
    PreRdCntBl[BL48] = 0; PreWrCntBl[BL48] = 0;
    PreRdCntBl[BL64] = 0; PreWrCntBl[BL64] = 0;

    Total_func_pre_cnt.resize(NUM_CHANS, 0);
    Total_rhit_break_pre_cnt.resize(NUM_CHANS, 0);
    Total_dummy_tout_pre_cnt.resize(NUM_CHANS, 0);

    rd_one = true;
    rd_two = false;
    rd_three = false;

    wr_one = true;
    wr_two = false;
    wr_three = false;

    pre_total_latency.resize(NUM_CHANS, 0);
    pre_com_read_cnt.resize(NUM_CHANS, 0);

    pre_cmd_in2dfi_lat.resize(NUM_CHANS, 0);
    pre_cmd_in2dfi_cnt.resize(NUM_CHANS, 0);

    cmdlat_offset_en = false;
#endif
    pre_acc_rank_cnt.clear();
    pre_acc_rank_cnt.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));
    pre_acc_bank_cnt.clear();
    pre_acc_bank_cnt.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS * NUM_BANKS, 0));
    pre_racc_rank_cnt.clear();
    pre_racc_rank_cnt.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));
    pre_racc_bank_cnt.clear();
    pre_racc_bank_cnt.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS * NUM_BANKS, 0));
    pre_wacc_rank_cnt.clear();
    pre_wacc_rank_cnt.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));
    pre_wacc_bank_cnt.clear();
    pre_wacc_bank_cnt.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS * NUM_BANKS, 0));
    pre_perf2ptc_bank_rcnt.clear();
    pre_perf2ptc_bank_rcnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    pre_perf2ptc_bank_wcnt.clear();
    pre_perf2ptc_bank_wcnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    pre_perf2ptc_bank_cnt.clear();
    pre_perf2ptc_bank_cnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    pre_perf_bank_rcnt.clear();
    pre_perf_bank_rcnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    pre_perf_bank_wcnt.clear();
    pre_perf_bank_wcnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    pre_ptc_r_bank_cnt.clear();
    pre_ptc_r_bank_cnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    pre_ptc_w_bank_cnt.clear();
    pre_ptc_w_bank_cnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));
    
    pre_perf_bg_rcnt.clear();
    pre_perf_bg_rcnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0)));
    pre_perf_bg_wcnt.clear();
    pre_perf_bg_wcnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0)));
    pre_perf2ptc_bg_rcnt.clear();
    pre_perf2ptc_bg_rcnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0)));
    pre_perf2ptc_bg_wcnt.clear();
    pre_perf2ptc_bg_wcnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0)));
    pre_ptc_r_bg_cnt.clear();
    pre_ptc_r_bg_cnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0)));
    pre_ptc_w_bg_cnt.clear();
    pre_ptc_w_bg_cnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_GROUPS, 0)));

    PreBankRowActCnt.clear();
    PreBankRowActCnt.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
        PreBankRowActCnt.push_back(0);
    }

    PreDmcPipeQueue.resize(NUM_CHANS);
    for (size_t ch = 0; ch < NUM_CHANS; ch++) {
        PreDmcPipeQueue[ch].reserve(tPIPE_PRE_DMC);
    }

    pre_abr_cnt.clear();
    pre_abr_cnt.resize(NUM_CHANS, vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(mptc_->getPTC(0)->sc_num, 0)));

    pre_pbr_cnt.clear();     //todo: revise for e-mode
    if (ENH_PBR_EN) {       
        pre_pbr_cnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * NUM_BANKS, 0));       
    } else {
        pre_pbr_cnt.resize(NUM_CHANS, vector<unsigned>(NUM_RANKS * mptc_->getPTC(0)->pbr_bank_num * mptc_->getPTC(0)->sc_num, 0));
    }

    prePdTime.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));
    preAsrefTime.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));
    preSrpdTime.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));
    preWakeUpTime.resize(NUM_CHANS, vector<uint32_t>(NUM_RANKS, 0));

    BusStateAsync.resize(NUM_CHANS);
}

bool fileExists(string &path) {
    struct stat stat_buf;
    if (stat(path.c_str(), &stat_buf) != 0) {
        if (errno == ENOENT) {
            return false;
        }
        ERROR("Some other kind of error happened with stat(), should probably check");
    }
    return true;
}

string FilenameWithNumberSuffix(const string &filename, const string &extension,
        unsigned maxNumber=100) {
    string currentFilename = filename+extension;
    if (!fileExists(currentFilename)) {
        return currentFilename;
    }

    // otherwise, add the suffixes and test them out until we find one that works
    stringstream tmpNum;
    tmpNum<<"."<<1;
    for (unsigned i = 1; i < maxNumber; i++) {
        currentFilename = filename+tmpNum.str() + extension;
        if (fileExists(currentFilename)) {
            currentFilename = filename;
            tmpNum.seekp(0);
            tmpNum << "." << i;
        } else {
            return currentFilename;
        }
    }
    // if we can't find one, just give up and return whatever is the current filename
    ERROR("Warning: Couldn't find a suitable suffix for "<<filename);
    return currentFilename;
}

void MemorySystemTop::initialize() {
    if (mpfq_ && mptc_) {
        mpfq_->setMPTC(mptc_.get());
        mpfq_->associateWithPTCs();
        mptc_->associateWithPFQs(mpfq_.get());
    }
    if (mras_ && mpfq_) {
        mras_->setMPFQ(mpfq_.get());
        mras_->initializeIECCs();
    }
}

//==============================================================================
void MemorySystemTop::InitOutputFiles(unsigned channel) {
    uint64_t chl_ohot = 1ull << channel;
    if ((DEBUG_BUS || DEBUG_STATE || DEBUG_GBUF_STATE || PRINT_BANK || PRINT_TIMEOUT) && (chl_ohot == (chl_ohot & PRINT_CH_OHOT))) {
        dmc_log = log_path + "/lpddr_sim" + std::to_string(channel) + ".log";
        DDRSim_log[channel].open(dmc_log.c_str(),ios_base::out | ios_base::trunc);
        if (!DDRSim_log[channel]) {
            ERROR("Cannot open "<<dmc_log);
            assert(0);
        }
        if (channel % (NUM_CHANS/NUM_PFQS) == 0 && IECC_ENABLE) {
            string iecc_log = log_path + "/lpddr_sim" + "_iecc" + std::to_string(channel/(NUM_CHANS/NUM_PFQS)) + ".log";
            DDRSim_iecc_log[channel/(NUM_CHANS/NUM_PFQS)].open(iecc_log.c_str(),ios_base::out | ios_base::trunc);
            if (!DDRSim_iecc_log[channel/(NUM_CHANS/NUM_PFQS)]) {
                ERROR("Cannot open "<<iecc_log);
                assert(0);
            }
        }
        if (channel % (NUM_CHANS/NUM_PFQS) == 0 && WRITE_BUFFER_ENABLE) {
            string pfq_log = log_path + "/lpddr_sim" + "_pfq" + std::to_string(channel/(NUM_CHANS/NUM_PFQS)) + ".log";
            // string pfq_log = log_path + "/lpddr_sim" + ".log";
            DDRSim_pfq_log[channel/(NUM_CHANS/NUM_PFQS)].open(pfq_log.c_str(),ios_base::out | ios_base::trunc);
            if (!DDRSim_pfq_log[channel/(NUM_CHANS/NUM_PFQS)]) {
                ERROR("Cannot open "<<pfq_log);
                assert(0);
            }
        }
        string ptc_log = log_path + "/lpddr_sim" + "_ptc" + std::to_string(channel) + ".log";
        // string ptc_log = log_path + "/lpddr_sim" + ".log";
        DDRSim_ptc_log[channel].open(ptc_log.c_str(),ios_base::out | ios_base::trunc);
        if (!DDRSim_ptc_log[channel]) {
            ERROR("Cannot open "<<ptc_log);
            assert(0);
        }
    }

    if (STATE_LOG) {
        string log_prefix = "lpddr_state";
        if (SYSTEM_CONFIG == "hbmsystem") {
            log_prefix = "hbm_state";
        } else if (SYSTEM_CONFIG == "ddrsystem") {
            log_prefix = "ddr_state";
        } else if (SYSTEM_CONFIG == "wsesystem") {
            log_prefix = "wse_state";
        }
        string st_log = log_path + "/" + log_prefix + std::to_string(channel) + ".log";
        state_log[channel].open(st_log.c_str(),ios_base::out | ios_base::trunc);
        if (!state_log[channel]) {
             ERROR("Cannot open "<<st_log);
             assert(0);
        }
    }

    if ((PRINT_TRACE || PRINT_UT_TRACE || PRINT_READ || PRINT_RDATA || PRINT_BW) &&
            (chl_ohot == (chl_ohot & PRINT_CH_OHOT))) {
        string log_prefix = "lpddr_trace";
        if (SYSTEM_CONFIG == "hbmsystem") {
            log_prefix = "hbm_trace";
        } else if (SYSTEM_CONFIG == "ddrsystem") {
            log_prefix = "ddr_trace";
        } else if (SYSTEM_CONFIG == "wsesystem") {
            log_prefix = "wse_trace";
        }
        string t_log = log_path + "/" + log_prefix + "_" + std::to_string(channel) + ".log";
        trace_log[channel].open(t_log.c_str(),ios_base::out | ios_base::trunc);
        if (!trace_log[channel]) {
             ERROR("Cannot open "<<t_log);
             assert(0);
        }
        if (PRINT_UT_TRACE) {
            TRACE_PRINT(channel, "type,address,trans_size,burst_len,mid,delay(ns),ATIME,ch_num,qos,gid"<<endl);
        } else {
            TRACE_PRINT(channel, "Message: DMC_RATE="<<DMC_RATE<<", tDFI="<<tDFI<<", DMC_DATA_BUS_BITS="
                    <<DMC_DATA_BUS_BITS<<", JEDEC_DATA_BUS_BITS="<<JEDEC_DATA_BUS_BITS<<endl);
        }
    }

    if (PRINT_CMD_NUM && (chl_ohot == (chl_ohot & PRINT_CH_OHOT))) {
        string log_prefix = "lpddr_cmdnum";
        if (SYSTEM_CONFIG == "hbmsystem") {
            log_prefix = "hbm_cmdnum";
        } else if (SYSTEM_CONFIG == "ddrsystem") {
            log_prefix = "ddr_cmdnum";
        } else if (SYSTEM_CONFIG == "wsesystem") {
            log_prefix = "wse_cmdnum";
        }
        string c_log = log_path + "/" + log_prefix + "_" + std::to_string(channel) + ".log";
        cmdnum_log[channel].open(c_log.c_str(),ios_base::out | ios_base::trunc);
        if (!cmdnum_log[channel]) {
             ERROR("Cannot open "<<c_log);
             assert(0);
        }
    }

    for (auto& row : dram_log) {
        row.resize(NUM_RANKS);
    }
    if (PRINT_DRAM_TRACE && (chl_ohot == (chl_ohot & PRINT_CH_OHOT))) {
        string log_prefix = "lpddr_dram";
        if (SYSTEM_CONFIG == "hbmsystem") {
            log_prefix = "hbm_dram";
        } else if (SYSTEM_CONFIG == "ddrsystem") {
            log_prefix = "ddr_dram";
        } else if (SYSTEM_CONFIG == "wsesystem") {
            log_prefix = "wse_dram";
        }
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            string d_log = log_path + "/" + log_prefix + "_" + std::to_string(channel) +
                "_" + std::to_string(rank) + ".log";
            dram_log[channel][rank].open(d_log.c_str(),ios_base::out | ios_base::trunc);
            if (!dram_log[channel][rank]) {
                 ERROR("Cannot open "<<d_log);
                 assert(0);
            }
        }
    }
}

MemorySystemTop::~MemorySystemTop() {
    if (DRAM_POWER_EN) {
        for (size_t ch = 0; ch < NUM_CHANS; ch++) {
            for (size_t i = 0; i < NUM_RANKS; i++) {
                mptc_->getRank(ch,i)->CalcDramPower();
            }
        }
    }
    for (size_t ch = 0; ch < NUM_CHANS; ch++) {
        // register_write(0,0,ch); // DON'T call register_write(0, 0, ch) here, as it will reset start_cycle to the current time.
        register_write(4,0,ch);
    }

    // flush our streams and close them up
#ifdef LOG_OUTPUT
    DDRSim_log.flush();
    DDRSim_log.close();
#endif
}

void MemorySystemTop::noc_read_inform(uint32_t channel, bool fast_wakeup_rank0,
        bool fast_wakeup_rank1, bool bus_rempty) {
    if (EM_ENABLE && EM_MODE==0) channel = 0;
    bus_state BusSt;
    BusSt.fast_wakeup_rank0 = fast_wakeup_rank0;
    BusSt.fast_wakeup_rank1 = fast_wakeup_rank1;
    BusSt.bus_rempty = bus_rempty;
    BusSt.valid_time = now() + 2;
    BusStateAsync[channel].push_back(BusSt);
    if (DEBUG_BUS) {
        if (fast_wakeup_rank0 || fast_wakeup_rank1) {
            PRINTN_M(channel, setw(10)<<now()<<" -- channel="<<channel<<", Fast Wakeup, rank0="<<fast_wakeup_rank0<<", rank1="<<fast_wakeup_rank1<<endl);
        }
    }
}

bool MemorySystemTop::addTransaction(const hha_command &command) {
    command_check(command);
    uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : addr_map_ch(command);

    //GRANT FIFO BP under LP6 Nomal mode
    if (!EM_ENABLE && IS_LP6 && mptc_->getPTC(1)->grt_fifo_bp) {
        return false;
    }
    
    bool ret = false;
    Transaction *trans = new Transaction(command);
    trans_init(trans, now());
    if (DEBUG_BUS) {
        PRINTN_M(ch, setw(10)<<now()<<" -- inject_time="<<trans->inject_time<<", task="<<trans->task<<endl);
    }
    addressMapping(*trans);
    trans_check(trans);

    // compensate latency
    if (cmdlat_offset_en && !LAT_INC_BP) {
        if (((now() % CLKH2CLKL == 1) && PERF_CLK_MODE==1) || trans->inject_time < 1) {
            ERROR(setw(10)<<now()<<" Cmd Lat Compensate Wrong!, task="<<trans->task<<", inject_time="<<trans->inject_time);
        }
        trans->inject_time = trans->inject_time - 1;
        if (DEBUG_BUS) {
            PRINTN_M(ch, setw(10)<<now()<<" -- Cmd Lat Compensate, task="<<trans->task<<endl);
        }
    }
    cmdlat_offset_en = false; // compensate latency due to two cycle one cmd
    if (WRITE_BUFFER_ENABLE && ((mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_pre_req_time == mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->now())
            || (mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->m_pre_req_time == mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->now()))) {
        if (DEBUG_BUS) {
            PRINTN_M(ch, setw(10)<<now()<<" -- Two Cycle One Cmd, perf_pre_req_time="<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_pre_req_time<<endl);
        }
        if (mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->m_pre_req_time == mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->now()) cmdlat_offset_en = true;
        return false;
    }
    ret = false;

    if (FORCE_BAINTLV_EN) {
        if (FORCE_BAINTLV_MODE == 0) {
        trans->rank = 0;
        trans->group = trans->task % NUM_GROUPS;
        trans->bank = trans->task / NUM_GROUPS % (NUM_BANKS / NUM_SIDS / NUM_GROUPS);
        trans->sid = trans->task / (NUM_BANKS / NUM_SIDS) % NUM_SIDS;
        trans->bankIndex = trans->bank + trans->group * (NUM_BANKS / NUM_SIDS / NUM_GROUPS) +
                trans->rank * NUM_BANKS + trans->sid * (NUM_BANKS / NUM_SIDS);
        } else if (FORCE_BAINTLV_MODE == 1) {
            unsigned trans_id = trans_baintlv[trans->rank][trans->transactionType];
            trans->group = trans_id % NUM_GROUPS;
            trans->bank = trans_id / NUM_GROUPS % (NUM_BANKS / NUM_GROUPS);
            trans->bankIndex = trans->bank + trans->group * (NUM_BANKS / NUM_GROUPS) +
                    trans->rank * NUM_BANKS;
            trans_baintlv[trans->rank][trans->transactionType]++;
            if (trans_id == (NUM_BANKS-1)) {
                trans_baintlv[trans->rank][trans->transactionType] = 0;
            }
        }
    } 
    write_msg msg;
    msg.pt = DMC_PATH;
    msg.num_256bit = trans->burst_length + 1;

    if (tPIPE_PRE_DMC == 0) {       
        ret = mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->addTransaction(trans);    
    } else {
        if (PreDmcPipeQueue[ch].size() >= tPIPE_PRE_DMC) {
            ret = false;
        } else {
            trans->async_delay_time = now() + tPIPE_PRE_DMC;
            PreDmcPipeQueue[ch].push_back(trans);
            ret = true;
        }
    }
    if (ret) {
        access_cnt[ch]++;
        total_access_cnt[ch]++;
        if (trans->transactionType != DATA_READ) {
            write_map[trans->task] = msg;
        }
    } else {
        bp_cnt[ch]++;
        total_bp_cnt[ch]++;
    }
    task_cnt[ch]++;
    total_task_cnt[ch]++;

    if (ret && !trans->pre_act) {
        if (trans->transactionType == DATA_READ) {
            if (PRINT_READ || PRINT_TRACE) {
                TRACE_PRINT(ch, setw(10)<<now()<<" -- ADD :: [R]channel="<<ch<<" B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
            if (PRINT_IDLE_LAT) {
                DEBUG(setw(10)<<now()<<" -- ADD :: [R]channel="<<ch<<" B["<<trans->burst_length<<"]QOS["<<trans->qos<<"] addr=0x"
                        <<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank<<" sid="<<trans->sid
                        <<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col
                        <<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded);
            }
            if (DEBUG_BUS) {
                PRINTN_M(ch, setw(10)<<now()<<" -- ADD :: [R]channel="<<ch<<" B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
        } else {
            if (PRINT_TRACE) {
                TRACE_PRINT(ch, setw(10)<<now()<<" -- ADD :: [W]channel="<<ch<<" B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN_M(ch, setw(10)<<now()<<" -- ADD :: [W]Bchannel="<<ch<<" B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
        }
        if (PRINT_UT_TRACE) {
            string ut_type;
            unsigned ut_mid = 0, ut_gid = 0;
            static uint64_t ut_pretime = 0;
            double ut_interval = double(now() - ut_pretime) * tDFI;
            double ut_time = double(now()) * tDFI;
            unsigned ut_qos = 7 - trans->qos;
            if (trans->transactionType == DATA_READ) ut_type = "nr";
            else ut_type = "nw";
            TRACE_PRINT(ch, fixed<<ut_type<<" "<<hex<<trans->address<<dec<<" "<<trans->data_size<<" "
                    <<trans->burst_length<<" "<<hex<<ut_mid<<dec<<setprecision(3)<<" "<<ut_interval
                    <<" "<<ut_time<<" "<<ch<<" "<<ut_qos<<" "<<ut_gid<<" "<<DMC_RATE<<endl);
            ut_pretime = now();
        }
    } else {
        if (DEBUG_BUS) {
            PRINTN_M(ch, setw(10)<<now()<<" -- DROP :: channel="<<ch<<" addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task
                    <<" QR="<<mptc_->getPTC(ch)->Read_Cnt()<<" QW="<<mptc_->getPTC(ch)->Write_Cnt()<<endl);
        }
    }
    if (PERFECT_DMC_EN) delete trans;

    if (!ret) {
        delete trans;
    }
    return ret;
}

void MemorySystemTop::trans_init(Transaction *trans, uint64_t inject_time) {
    trans->data_size = (trans->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
    trans->inject_time = inject_time;
    if (!LAT_INC_BP) trans->reqEnterDmcBufTime = now() * tDFI;
}

void MemorySystemTop::trans_check(Transaction *t) {
    if (IS_HBM2E || IS_HBM3) {
        if (NUM_SIDS == 3 && t->sid == 3) {
            ERROR(setw(10)<<now()<<" Error Sid! task="<<t->task<<" address="<<hex<<t->address<<dec<<" sid="<<t->sid);
            assert(0);
        }
    }
    if (IS_LP6 && EM_ENABLE && EM_MODE==2) {
        if (t->sc == 1 && t->rank == 1) {
            ERROR(setw(10)<<now()<<" Error SC under Combo e-mode! task="<<t->task<<" address="<<hex<<t->address<<dec<<" sc="<<t->sc<<", rank="<<t->rank);
            assert(0);
        }
    }
} 

uint32_t MemorySystemTop::getDmcPressureLevel() {
    uint32_t maxPressureLevel = 0;
    uint32_t pressureLevel = 0;
    for (uint32_t ch = 0; ch < NUM_CHANS; ch++) {
        pressureLevel = curFlowPressureLevel[ch];
        if (pressureLevel > maxPressureLevel) {
            maxPressureLevel = pressureLevel;
        }
    }
    return maxPressureLevel;
}

bool MemorySystemTop::addData(uint32_t *data, uint32_t channel, uint64_t id) {
    if (EM_ENABLE && EM_MODE == 0) channel = 0;
    if (WRITE_BUFFER_ENABLE && ((mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->perf_pre_data_time == mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->now())
            || (mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->m_pre_data_time == mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->now()))) {
        if (DEBUG_BUS) {
            PRINTN_M(channel, setw(10)<<now()<<" -- WDATA :: BP Wdata, Two Cycle One Data :: task="<<id
                        <<", mem_pre_wdata_time="<<mptc_->getPTC(channel)->pre_req_data_time<<
                        ", perf_pre_wdata_time="<<mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->perf_pre_data_time<<endl);
        }
        return false;
    }

    if (DROP_WRITE_CMD || PERFECT_DMC_EN) return true;

    auto it = write_map.find(id);
    if (it == write_map.end()) {
        if (DEBUG_BUS) {
            PRINTN_M(channel, setw(10)<<now()<<" -- WDATA :: BP Wdata, Wcmd must be send first, task="<<id<<endl);
        }
        return false;
    } else {
        if (DEBUG_BUS) {
            PRINTN_M(channel, setw(10)<<now()<<" -- WDATA :: Add Wdata, task="<<id<<endl);
        }
    }

    if (WRITE_BUFFER_ENABLE) {
        mras_->getIECC(channel / (NUM_CHANS / NUM_PFQS))->addData(data, channel, id) ;
    } 

    if (it->second.num_256bit == 0) {
        ERROR(setw(10)<<now()<<" -- ERROR, burst number mismatch, ID="
                <<id<<", chnl: "<<channel);
    } else {
        it->second.num_256bit--;
    }
    if (it->second.num_256bit == 0) {
        write_map.erase(id);
    }
    return true;
}

void MemorySystemTop::update() {
    if (NUM_CHANS % NUM_PFQS != 0) {
        ERROR("NUM_CHANS / NUM_PFQS must be an integer.");
        ERROR("Current NUM_CHANS = " << NUM_CHANS);
        ERROR("Current NUM_PFQS = " << NUM_PFQS);
        assert(0);
    }

    vector<uint8_t> bus_ptr;
    bus_ptr.resize(NUM_CHANS);
    for (size_t ch = 0; ch < NUM_CHANS; ch++) {
        bus_ptr[ch] = 0;
        for (auto bus : BusStateAsync[ch]) {
            if (now() >= bus.valid_time) {
                mptc_->getPTC(ch)->noc_read_inform(bus.fast_wakeup_rank0,
                        bus.fast_wakeup_rank1, bus.bus_rempty);
                bus_ptr[ch]++;
            } else {
                break;
            }
        }
        if (bus_ptr[ch] > 0) {
            BusStateAsync[ch].erase(BusStateAsync[ch].begin(), BusStateAsync[ch].begin() + bus_ptr[ch]);
        }

        if (ch % (NUM_CHANS / NUM_PFQS) == 0) {
            size_t pfq_idx = ch / (NUM_CHANS / NUM_PFQS);
            for (size_t i = 0; i < NUM_RANKS; i ++) {
                mptc_->getRank(ch, i)->update();
            }
            if (IECC_ENABLE) {
                mras_->getIECC(pfq_idx)->update();
            }
            mpfq_->getPFQ(pfq_idx)->update();
            mpfq_->addFastRead(pfq_idx);
            mptc_->getPTC(pfq_idx)->update();

            if (IECC_ENABLE) {
                mras_->getIECC(pfq_idx)->step();
            }
            mpfq_->getPFQ(pfq_idx)->step();
            mptc_->getPTC(pfq_idx)->step();
        } else {
            mptc_->getPTC(ch)->update();
            mptc_->getPTC(ch)->step();
        }
        // mpfq_->step(); // for mpfq->addfastread
        // this->step();

        if (STATE_TIME != 0) {
            if ((now() % STATE_TIME) == 0) {
                register_write(0,0,ch);
                register_write(4,0,ch);
            }
        }

        perf_check_cnt(ch);
        check_cnt(ch);

        if (FLOW_STAT_TIME != 0) {
            if ((now() % FLOW_STAT_TIME) == 0) {
                float total_bw = flowStatistic(ch);
                curFlowPressureLevel[ch] = updateFlowState(total_bw);
            }
        }

        if (!PreDmcPipeQueue[ch].empty()) {
            auto trans = PreDmcPipeQueue[ch][0];
            if (now() >= trans->async_delay_time && mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->addTransaction(trans)) {
                PreDmcPipeQueue[ch].erase(PreDmcPipeQueue[ch].begin());
            }
        }
    }
    mpfq_->step(); // for mpfq->addfastread
    this->step();
}

void MemorySystemTop::RegisterCallbacks(
    TransactionCompleteCB *readData,
    TransactionCompleteCB *writeDone,
    TransactionCompleteCB *readDone,
    TransactionCompleteCB *cmdDone ) {
    for (size_t i = 0; i < NUM_CHANS; i++) {
        mpfq_->RegisterCallbacks(readData, writeDone, readDone, cmdDone);
    }
}

void MemorySystemTop::dfs_backpress(unsigned ch, bool backpress) {
    if (EM_ENABLE && EM_MODE == 0) ch = 0;
    mptc_->getPTC(ch)->dfs_backpress(backpress);
}

uint32_t MemorySystemTop::getTransQueSize(uint32_t dmc_id, bool isRd) {
    if (EM_ENABLE && EM_MODE==0) dmc_id = 0;
    uint32_t queRdNum = 0;
    uint32_t queWrNum = 0;
    uint32_t size = mptc_->getPTC(dmc_id)->GetDmcQsize();

    for (uint32_t index = 0; index < size; index++) {
        if (mptc_->getPTC(dmc_id)->transactionQueue.at(index)->transactionType == DATA_READ) {
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

void MemorySystemTop::GetQueueCmdNum(uint32_t channel, unsigned *ptc_rd_num,
        unsigned *ptc_wr_num, unsigned *pfq_rd_num, unsigned *pfq_wr_num) { 
    if (EM_ENABLE && EM_MODE == 0) channel = 0;        
    *ptc_rd_num = mptc_->getPTC(channel)->que_read_cnt;
    *ptc_wr_num = mptc_->getPTC(channel)->que_write_cnt;
    *pfq_rd_num = mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->rcmd_num();
    *pfq_wr_num = mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->wcmd_num();
}

void MemorySystemTop::GetDmcBusyStatus(uint32_t channel, bool *dmc_busy) {
    if (EM_ENABLE && EM_MODE == 0) channel = 0;
    unsigned cmd_cnt = 0;
    cmd_cnt = mptc_->getPTC(channel)->que_read_cnt + mpfq_->getPFQ(channel / (NUM_CHANS / NUM_PFQS))->rcmd_num();
    if (BUSYSTATE_INC_WCMD) {
        cmd_cnt += mptc_->getPTC(channel)->que_write_cnt;
    }
    if (cmd_cnt >= BUSYSTATE_TH) {
        *dmc_busy = true;
    } else {
        *dmc_busy = false;
    }
}

uint8_t MemorySystemTop::addr_map_ch(const hha_command &c) {
    if (IS_LP6 || IS_HBM3) {
        return c.channel;
    } else if (MATRIX_CH == 0) {
        return c.channel;
    } else {
        return (bit_xor(MATRIX_CH, c.address));
    }
}

void MemorySystemTop::command_check(const hha_command &c) {
    if (c.type != DATA_READ && c.type != DATA_WRITE) {
        ERROR("Error command type! task="<<c.task<<", type="<<c.type);
        assert(0);
    }
    if (c.pf_type >= 4) {
        ERROR("Error pf_type! task="<<c.task<<", pf_type="<<c.pf_type);
        assert(0);
    }
    if (c.sub_pftype >= 13) {
        ERROR("Error sub_pftype! task="<<c.task<<", sub_pftype="<<c.sub_pftype);
        assert(0);
    }
    if (c.sub_src >= 4) {
        ERROR("Error sub_src! task="<<c.task<<", sub_src="<<c.sub_src);
        assert(0);
    }
    if (c.qos >= 8) {
        ERROR("Error qos value! task="<<c.task<<", qos="<<c.qos);
        assert(0);
    }
    if (c.mid >= MidMax) {
        ERROR("Error mid value! task="<<c.task<<", mid="<<c.mid);
        assert(0);
    }
    if (c.channel > NUM_CHANS) {
        ERROR("Error command channel number! task="<<c.task<<", channel="<<c.channel);
        assert(0);
    }
}

//==============================================================================
void MemorySystemTop::register_read(uint64_t address, uint32_t &data) { //Todo delete?
    uint32_t offset = address & 0xff;
    switch (offset) {
        case 0x0: break;
        case 0x4: break;
        default:  break;
    }
}

//==============================================================================
void MemorySystemTop::register_write(uint64_t address, uint32_t data, uint32_t ch) {
    uint32_t offset = ((address != 0) ? 4 : 0);
    switch (offset) {
        case 0x0:{
            start_cycle = this->now();
            break;
        }
        case 0x4:{
            end_cycle = this->now();
            if (start_cycle != end_cycle && STATE_LOG == true) {
                statistics(ch);
            }
            break;
        }
        default: break;
    }
}

//==============================================================================
void MemorySystemTop::perf_check_cnt(uint32_t ch) {
    uint32_t size = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->GetPerfQsize();
    perf_que_cnt[ch].at(size)++;
}

//==============================================================================
void MemorySystemTop::check_cnt(uint32_t ch) {
    uint32_t queRdNum = 0;
    uint32_t queWrNum = 0;

    uint32_t size = mptc_->getPTC(ch)->GetDmcQsize();
    que_cnt[ch].at(size)++;

    for (uint32_t index = 0; index < size; index++) {
        if (mptc_->getPTC(ch)->transactionQueue.at(index)->transactionType == DATA_READ) {
            queRdNum++;
        } else {
            queWrNum++;
        }
    }
    que_rd_cnt[ch].at(queRdNum)++;
    que_wr_cnt[ch].at(queWrNum)++;
}

//==============================================================================
float MemorySystemTop::flowStatistic(uint32_t ch) {
    flow_statis_end_cycle = now();
    if (flow_statis_end_cycle == flow_statis_start_cycle) return 0.0;

    uint64_t bytes = mptc_->getPTC(ch)->flowStatisTotalBytes;
    float total_bw = (float(bytes)) / ((flow_statis_end_cycle - flow_statis_start_cycle)* tDFI);

    flow_statis_start_cycle = now();
    mptc_->getPTC(ch)->flowStatisTotalBytes = 0;
    return total_bw;
}

//==============================================================================
uint32_t MemorySystemTop::updateFlowState(float total_bw) {
    if (DMC_THEORY_BW <= 0.0) return 0;
    if (total_bw < DMC_THEORY_BW*0.375) {
        return 0;
    } else if (total_bw < DMC_THEORY_BW*0.625) {
        return 1;
    } else if (total_bw < DMC_THEORY_BW*0.75) {
        return 2;
    } else {
        return 3;
    }
}

void MemorySystemTop::UnitConvert(double *oenergy, string *ouint, double ienergy) {
    unsigned convert_cnt = 0;
    double energy = ienergy;
    while (energy >= 10000) {
        energy /= 1000;
        convert_cnt ++;
    }
    *oenergy = energy;
    if (convert_cnt == 0) *ouint = "pJ";
    else if (convert_cnt == 1) *ouint = "nJ";
    else if (convert_cnt == 2) *ouint = "uJ";
    else if (convert_cnt == 3) *ouint = "mJ";
    else if (convert_cnt == 4) *ouint = "J";
    else if (convert_cnt == 5) *ouint = "kJ";
    else {ERROR("No such unit, energy:"<<ienergy); assert(0);}
}

//==============================================================================
void MemorySystemTop::statistics(uint32_t ch) {
    unsigned size = 0;
    STATE_PRINTN(ch, setiosflags(ios::left));
    STATE_PRINTN(ch, "======================================== START ========================================\n");
    STATE_PRINTN(ch, "-------------------- Base Message -----------------------------------------------------\n");
    STATE_PRINTN(ch, DDR_TYPE<<" "<<DMC_RATE<<"Mbps, x"<<JEDEC_DATA_BUS_BITS<<", DMC Data Width: "
            <<DMC_DATA_BUS_BITS<<", CKR: "<<setprecision(1)<<WCK2DFI_RATIO<<endl);
    uint64_t bytes = mptc_->getPTC(ch)->TotalBytes - mptc_->getPTC(ch)->ecc_total_bytes - mptc_->getPTC(ch)->rmw_total_bytes;
    STATE_PRINTN(ch, "Current time: "<<fixed<<now()<<", tDFI: "<<setprecision(4)<<tDFI<<" ns, DMC Total bytes: ");
    STATE_PRINTN(ch, mptc_->getPTC(ch)->DmcTotalBytes<<", DDR Total bytes: "<<bytes<<endl);
    STATE_PRINTN(ch, "-------------------- DFI Performance Statistics (GB/s) --------------------------------\n");
    if (end_cycle == start_cycle) return;
    bytes = mptc_->getPTC(ch)->TotalReadBytes - mptc_->getPTC(ch)->ecc_total_reads - mptc_->getPTC(ch)->rmw_total_reads;
    float data_read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->TotalWriteBytes - mptc_->getPTC(ch)->ecc_total_writes;
    float data_write_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->TotalBytes - mptc_->getPTC(ch)->ecc_total_bytes - mptc_->getPTC(ch)->rmw_total_bytes;
    float data_total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    bytes = mptc_->getPTC(ch)->ecc_total_bytes;
    float ecc_total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->ecc_total_writes;
    float ecc_write_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->ecc_total_reads;
    float ecc_read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    bytes = mptc_->getPTC(ch)->rmw_total_bytes;
    float rmw_total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->rmw_total_reads;
    float rmw_read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    bytes = mptc_->getPTC(ch)->TotalReadBytes;
    float read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->TotalWriteBytes;
    float write_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = mptc_->getPTC(ch)->TotalBytes;
    float total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    STATE_PRINTN(ch, "Data Read bandwidth   : "<<setprecision(2)<<setw(6)<<data_read_bw);
    STATE_PRINTN(ch, " | Data Write bandwidth   : "<<setprecision(2)<<setw(6)<<data_write_bw);
    STATE_PRINTN(ch, " | Data Total bandwidth  : "<<setprecision(2)<<data_total_bw<<endl);

    STATE_PRINTN(ch, "ECC Read bandwidth    : "<<setprecision(2)<<setw(6)<<ecc_read_bw);
    STATE_PRINTN(ch, " | ECC Write bandwidth    : "<<setprecision(2)<<setw(6)<<ecc_write_bw);
    STATE_PRINTN(ch, " | ECC Total bandwidth   : "<<setprecision(2)<<ecc_total_bw<<endl);

    STATE_PRINTN(ch, "RMW Read bandwidth    : "<<setprecision(2)<<setw(6)<<rmw_read_bw);
    STATE_PRINTN(ch, " | RMW Total bandwidth    : "<<setprecision(2)<<rmw_total_bw<<endl);

    STATE_PRINTN(ch, "Total Read bandwidth  : "<<setprecision(2)<<setw(6)<<read_bw);
    STATE_PRINTN(ch, " | Total Write bandwidth  : "<<setprecision(2)<<setw(6)<<write_bw);
    STATE_PRINTN(ch, " | Total bandwidth       : "<<setprecision(2)<<total_bw<<endl);

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        bytes = mptc_->getPTC(ch)->TotalBytesRank[rank];
        float total_bw_rank = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
        STATE_PRINTN(ch, "Total Rank"<<rank<<" bandwidth : "<<fixed<<setw(6)<<total_bw_rank<<" | ");
        float rank_avail = 0;
        if (IS_G3D){
            rank_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                    (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
        } else {
            rank_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                    (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
        }
        if (IS_LP6) rank_avail = rank_avail * 9 / 8;
        STATE_PRINTN(ch, "Total Rank"<<rank<<" efficiency : "<<setprecision(2)<<rank_avail<<"%"<<endl);
    }

    bytes = mptc_->getPTC(ch)->TotalBytes - mptc_->getPTC(ch)->ecc_total_bytes - mptc_->getPTC(ch)->rmw_total_bytes;
    float data_avail = 0;
    if (IS_G3D){
        data_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        data_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    bytes = mptc_->getPTC(ch)->ecc_total_bytes;
    float ecc_avail = 0;
    if (IS_G3D){
        ecc_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        ecc_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    bytes = mptc_->getPTC(ch)->rmw_total_bytes;
    float rmw_avail = 0;
    if (IS_G3D){
        rmw_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        rmw_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    bytes = mptc_->getPTC(ch)->TotalBytes;
    float avail = 0;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;  
    }
    if (IS_LP6) data_avail = data_avail * 9 / 8;
    if (IS_LP6) ecc_avail = ecc_avail * 9 / 8;
    if (IS_LP6) rmw_avail = rmw_avail * 9 / 8;
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN(ch, "Data efficiency       : "<<setprecision(2)<<setw(5)<<data_avail<<"%");
    STATE_PRINTN(ch, " | ECC efficiency         : "<<setprecision(2)<<setw(5)<<ecc_avail<<"%");
    STATE_PRINTN(ch, " | RMW efficiency         : "<<setprecision(2)<<setw(5)<<rmw_avail<<"%"<<endl);
    STATE_PRINTN(ch, "DFI Total efficiency  : "<<setprecision(2)<<setw(5)<<avail<<"%"<<endl);

    STATE_PRINTN(ch, "-------------------- DMC Performance Statistics (GB/s) --------------------------------\n");
    bytes = mptc_->getPTC(ch)->DmcTotalReadBytes;
    avail = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    STATE_PRINTN(ch, "Dmc Read bandwidth    : "<<setprecision(2)<<setw(6)<<avail);
    bytes = mptc_->getPTC(ch)->DmcTotalWriteBytes;
    avail = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    STATE_PRINTN(ch, " | Dmc Write bandwidth    : "<<setprecision(2)<<setw(6)<<avail);
    bytes = mptc_->getPTC(ch)->DmcTotalBytes;
    avail = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    STATE_PRINTN(ch, " | Dmc Total bandwidth   : "<<setprecision(2)<<setw(6)<<avail<<endl);
    bytes = mptc_->getPTC(ch)->DmcTotalReadBytes;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN(ch, "Dmc Read efficiency   : "<<setprecision(2)<<setw(5)<<avail<<"%");
    bytes = mptc_->getPTC(ch)->DmcTotalWriteBytes;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN(ch, " | Dmc Write efficiency   : "<<setprecision(2)<<setw(5)<<avail<<"%");
    bytes = mptc_->getPTC(ch)->DmcTotalBytes;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN(ch, " | Dmc Total efficiency  : "<<setprecision(2)<<setw(5)<<avail<<"%"<<endl);

    STATE_PRINTN(ch, "-------------------- Rank LP Time Statistics (DFI Clock) ------------------------------\n");
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Current PdTime      : "<<setw(12)
                <<(mptc_->getPTC(ch)->PdTime[i] - prePdTime[ch][i])<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Current AsrefTime   : "<<setw(12)
                <<(mptc_->getPTC(ch)->AsrefTime[i] - preAsrefTime[ch][i])<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Current SrpdTime    : "<<setw(12)
                <<(mptc_->getPTC(ch)->SrpdTime[i] - preSrpdTime[ch][i])<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Current WakeUpTime  : "<<setw(12)
                <<(mptc_->getPTC(ch)->WakeUpTime[i] - preWakeUpTime[ch][i])<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total PdTime        : "<<setw(12)
                <<mptc_->getPTC(ch)->PdTime[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total AsrefTime     : "<<setw(12)
                <<mptc_->getPTC(ch)->AsrefTime[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total SrpdTime      : "<<setw(12)
                <<mptc_->getPTC(ch)->SrpdTime[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total WakeUpTime    : "<<setw(12)
                <<mptc_->getPTC(ch)->WakeUpTime[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total PdEnterCnt    : "<<setw(12)
                <<mptc_->getPTC(ch)->PdEnterCnt[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total PdExitCnt     : "<<setw(12)
                <<mptc_->getPTC(ch)->PdExitCnt[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total AsrefEnterCnt : "<<setw(12)
                <<mptc_->getPTC(ch)->AsrefEnterCnt[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" Total AsrefExitCnt  : "<<setw(12)
                <<mptc_->getPTC(ch)->AsrefExitCnt[i]<<" | ");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "Phy LP Cnt(Close Clock)   : "<<setw(12)<<mptc_->getPTC(ch)->phy_lp_cnt<<" |"<<endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        prePdTime[ch][i] = mptc_->getPTC(ch)->PdTime[i];
        preAsrefTime[ch][i] = mptc_->getPTC(ch)->AsrefTime[i];
        preSrpdTime[ch][i] = mptc_->getPTC(ch)->SrpdTime[i];
        preWakeUpTime[ch][i] = mptc_->getPTC(ch)->WakeUpTime[i];
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        uint64_t page_idle_cnt = mptc_->getPTC(ch)->bank_idle_cnt[rank];
        uint64_t page_act_cnt = mptc_->getPTC(ch)->bank_act_cnt[rank];
        STATE_PRINTN(ch, "Rank"<<rank<<" Total Idle ratio    : "<<setw(12)
                <<((long double)page_idle_cnt * 100 / now() / NUM_BANKS)<<" | ");
        STATE_PRINTN(ch, "Rank"<<rank<<" Total act ratio     : "<<setw(12)
                <<((long double)page_act_cnt * 100 / now() / NUM_BANKS)<<" | "<<endl);
    }
    mptc_->getPTC(ch)->TotalBytes = 0;
    mptc_->getPTC(ch)->TotalReadBytes = 0;
    mptc_->getPTC(ch)->TotalWriteBytes = 0;
    mptc_->getPTC(ch)->ecc_total_bytes = 0;
    mptc_->getPTC(ch)->ecc_total_reads = 0;
    mptc_->getPTC(ch)->ecc_total_writes = 0;
    mptc_->getPTC(ch)->rmw_total_bytes = 0;
    mptc_->getPTC(ch)->rmw_total_reads = 0;
    mptc_->getPTC(ch)->DmcTotalBytes = 0;
    mptc_->getPTC(ch)->DmcTotalReadBytes = 0;
    mptc_->getPTC(ch)->DmcTotalWriteBytes = 0;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        mptc_->getPTC(ch)->TotalBytesRank[rank] = 0;
    }

    unsigned reads = mptc_->getPTC(ch)->totalReads;
    unsigned writes = mptc_->getPTC(ch)->totalWrites;
    unsigned totals = mptc_->getPTC(ch)->totalTransactions;
    unsigned address_conf_cnt = mptc_->getPTC(ch)->addrconf_cnt;
    unsigned perf_address_conf_cnt = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_addrconf_cnt;
    unsigned id_conf_cnt = mptc_->getPTC(ch)->idconf_cnt;
    unsigned ba_conf_cnt = mptc_->getPTC(ch)->baconf_cnt;
    unsigned total_conf = mptc_->getPTC(ch)->totalconf_cnt;
    unsigned act_cnt = mptc_->getPTC(ch)->active_cnt;
    unsigned act_dst_cnt = mptc_->getPTC(ch)->active_dst_cnt;
    unsigned byp_act_cnt = mptc_->getPTC(ch)->bypass_active_cnt;
    unsigned pre_sb_cnt = mptc_->getPTC(ch)->precharge_sb_cnt;
    unsigned pre_pb_cnt = mptc_->getPTC(ch)->precharge_pb_cnt;
    unsigned pre_pb_dst_cnt = mptc_->getPTC(ch)->precharge_pb_dst_cnt;
    unsigned pre_ab_cnt = mptc_->getPTC(ch)->precharge_ab_cnt;
    unsigned read_cnt = mptc_->getPTC(ch)->read_cnt;
    unsigned write_cnt = mptc_->getPTC(ch)->write_cnt;
    unsigned read_p_cnt = mptc_->getPTC(ch)->read_p_cnt;
    unsigned write_p_cnt = mptc_->getPTC(ch)->write_p_cnt;
    unsigned mwrite_cnt = mptc_->getPTC(ch)->mwrite_cnt;
    unsigned mwrite_p_cnt = mptc_->getPTC(ch)->mwrite_p_cnt;
    unsigned timeout_cnt = mptc_->getPTC(ch)->dmc_timeout_cnt;
    unsigned fast_timeout_cnt = mptc_->getPTC(ch)->dmc_fast_timeout_cnt;
    unsigned slow_timeout_cnt = mptc_->getPTC(ch)->dmc_slow_timeout_cnt;
    unsigned rt_timeout_cnt = mptc_->getPTC(ch)->RtCmdCnt;
    unsigned perf_rd_timeout_cnt = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->rd_timeout_cnt;
    unsigned perf_wr_timeout_cnt = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->wr_timeout_cnt;
    unsigned dummy_timeout_cnt = mptc_->getPTC(ch)->dummy_timeout_cnt;
    unsigned dummy_hqos_cnt = mptc_->getPTC(ch)->dummy_hqos_cnt;
    unsigned row_hit_cnt = 0;
    unsigned row_miss_cnt = 0;
    unsigned rw_switch_cnt = mptc_->getPTC(ch)->rw_switch_cnt;
    unsigned rank_switch_cnt = mptc_->getPTC(ch)->rank_switch_cnt;
    unsigned r_rank_switch_cnt = mptc_->getPTC(ch)->r_rank_switch_cnt;
    unsigned w_rank_switch_cnt = mptc_->getPTC(ch)->w_rank_switch_cnt;
    unsigned sid_switch_cnt = mptc_->getPTC(ch)->sid_switch_cnt;
    unsigned rw_idle_cnt = mptc_->getPTC(ch)->rw_idle_cnt;
    unsigned refresh_pb_cnt = mptc_->getPTC(ch)->refresh_pb_cnt;
    unsigned refresh_ab_cnt = mptc_->getPTC(ch)->refresh_ab_cnt;
    unsigned r2w_switch_cnt = mptc_->getPTC(ch)->r2w_switch_cnt;
    unsigned w2r_switch_cnt = mptc_->getPTC(ch)->w2r_switch_cnt;
    unsigned phy_notlp_cnt = mptc_->getPTC(ch)->phy_notlp_cnt;
    unsigned phy_lp_cnt = mptc_->getPTC(ch)->phy_lp_cnt;
    unsigned ecc_read_cnt = mptc_->getPTC(ch)->ecc_read_cnt;
    unsigned ecc_write_cnt = mptc_->getPTC(ch)->ecc_write_cnt;
    unsigned merge_read_cnt = mptc_->getPTC(ch)->merge_read_cnt;
    unsigned fast_read_cnt = mptc_->getPTC(ch)->total_fast_rd_cnt;
    unsigned fast_act_cnt = mptc_->getPTC(ch)->total_fast_act_cnt;
    unsigned pde_cnt = 0, asrefe_cnt = 0, srpde_cnt = 0;
    unsigned pdx_cnt = 0, asrefx_cnt = 0, srpdx_cnt = 0;

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        pde_cnt += mptc_->getPTC(ch)->PdEnterCnt[rank];
        asrefe_cnt += mptc_->getPTC(ch)->AsrefEnterCnt[rank];
        srpde_cnt += mptc_->getPTC(ch)->SrpdEnterCnt[rank];
        pdx_cnt += mptc_->getPTC(ch)->PdExitCnt[rank];
        asrefx_cnt += mptc_->getPTC(ch)->AsrefExitCnt[rank];
        srpdx_cnt += mptc_->getPTC(ch)->SrpdExitCnt[rank];
    }

//    for (size_t bankidx = 0; bankidx < NUM_RANKS * NUM_BANKS; bankidx ++) {
//        perf2ptc_bank_rcnt[bankidx] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_rcnt[bankidx];
//        perf2ptc_bank_wcnt[bankidx] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_wcnt[bankidx];
//        perf2ptc_bank_cnt[bankidx] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_cnt[bankidx];
//    }
    

    if (POWER_EN) {
        STATE_PRINTN(ch, "-------------------- Power Event And Power Consumption --------------------------------\n");
        for (size_t que = 1; que <= TRANS_QUEUE_DEPTH; que ++) {
            STATE_PRINTN(ch, "PMU Que"<<setw(5)<<que<<":"<<setw(8)<<que_cnt[ch][que]<<" | ");
            if (que % 4 == 0) STATE_PRINTN(ch, endl);
        }
        STATE_PRINTN(ch, "PMU RdInc   :"<<setw(8)<<mptc_->getPTC(ch)->rd_inc_cnt<<" | ");
        STATE_PRINTN(ch, "PMU RdWrap  :"<<setw(8)<<mptc_->getPTC(ch)->rd_wrap_cnt<<" | ");
        STATE_PRINTN(ch, "PMU WrInc   :"<<setw(8)<<mptc_->getPTC(ch)->wr_inc_cnt<<" | ");
        STATE_PRINTN(ch, "PMU WrWrap  :"<<setw(8)<<mptc_->getPTC(ch)->wr_wrap_cnt<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU Rdata   :"<<setw(8)<<mptc_->getPTC(ch)->rdata_cnt<<" | ");
        STATE_PRINTN(ch, "PMU Wdata   :"<<setw(8)<<mptc_->getPTC(ch)->wdata_cnt<<" | ");
        STATE_PRINTN(ch, "PMU Act     :"<<setw(8)<<(act_cnt - pre_act_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Prep    :"<<setw(8)<<(pre_pb_cnt - pre_pre_pb_cnt[ch])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU Pres    :"<<setw(8)<<(pre_sb_cnt - pre_pre_sb_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Prea    :"<<setw(8)<<(pre_ab_cnt - pre_pre_ab_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU RdBl8   :"<<setw(8)<<(mptc_->getPTC(ch)->RdCntBl[BL8] - PreRdCntBl[BL8])<<" | ");
        STATE_PRINTN(ch, "PMU RdBl16  :"<<setw(8)<<(mptc_->getPTC(ch)->RdCntBl[BL16] - PreRdCntBl[BL16])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU RdBl24  :"<<setw(8)<<(mptc_->getPTC(ch)->RdCntBl[BL24] - PreRdCntBl[BL24])<<" | ");
        STATE_PRINTN(ch, "PMU RdBl32  :"<<setw(8)<<(mptc_->getPTC(ch)->RdCntBl[BL32] - PreRdCntBl[BL32])<<" | ");
        STATE_PRINTN(ch, "PMU RdBl48  :"<<setw(8)<<(mptc_->getPTC(ch)->RdCntBl[BL48] - PreRdCntBl[BL48])<<" | ");
        STATE_PRINTN(ch, "PMU RdBl64  :"<<setw(8)<<(mptc_->getPTC(ch)->RdCntBl[BL64] - PreRdCntBl[BL64])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU WrBl8   :"<<setw(8)<<(mptc_->getPTC(ch)->WrCntBl[BL8] - PreWrCntBl[BL8])<<" | ");
        STATE_PRINTN(ch, "PMU WrBl16  :"<<setw(8)<<(mptc_->getPTC(ch)->WrCntBl[BL16] - PreWrCntBl[BL16])<<" | ");
        STATE_PRINTN(ch, "PMU WrBl24  :"<<setw(8)<<(mptc_->getPTC(ch)->WrCntBl[BL24] - PreWrCntBl[BL24])<<" | ");
        STATE_PRINTN(ch, "PMU WrBl32  :"<<setw(8)<<(mptc_->getPTC(ch)->WrCntBl[BL32] - PreWrCntBl[BL32])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU WrBl48  :"<<setw(8)<<(mptc_->getPTC(ch)->WrCntBl[BL48] - PreWrCntBl[BL48])<<" | ");
        STATE_PRINTN(ch, "PMU WrBl64  :"<<setw(8)<<(mptc_->getPTC(ch)->WrCntBl[BL64] - PreWrCntBl[BL64])<<" | ");
        STATE_PRINTN(ch, "PMU Pbr     :"<<setw(8)<<(refresh_pb_cnt - pre_refresh_pb_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Abr     :"<<setw(8)<<(refresh_ab_cnt - pre_refresh_ab_cnt[ch])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU R2w     :"<<setw(8)<<(r2w_switch_cnt - pre_r2w_switch_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU W2r     :"<<setw(8)<<(w2r_switch_cnt - pre_w2r_switch_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Rnksw   :"<<setw(8)<<(rank_switch_cnt - pre_rank_switch_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Pde     :"<<setw(8)<<(pde_cnt - pre_pde_cnt[ch])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU Asrefe  :"<<setw(8)<<(asrefe_cnt - pre_asrefe_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Srpde   :"<<setw(8)<<(srpde_cnt - pre_srpde_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Pdx     :"<<setw(8)<<(pdx_cnt - pre_pdx_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Asrefx  :"<<setw(8)<<(asrefx_cnt - pre_asrefx_cnt[ch])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "PMU Srpdx   :"<<setw(8)<<(srpdx_cnt - pre_srpdx_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Idle    :"<<setw(8)<<(phy_notlp_cnt - pre_phy_notlp_cnt[ch])<<" | ");
        STATE_PRINTN(ch, "PMU Pdcc    :"<<setw(8)<<(phy_lp_cnt - pre_phy_lp_cnt[ch])<<" | ");
        STATE_PRINTN(ch, endl);
        STATE_PRINTN(ch, "Power Consumption :"<<fixed<<setw(26)<<(mptc_->getPTC(ch)->calc_power()-pre_power[ch])<<" | ");
        STATE_PRINTN(ch, "Power Consumption Total :"<<fixed<<setw(20)<<mptc_->getPTC(ch)->calc_power()<<endl);
        pre_power[ch] = mptc_->getPTC(ch)->calc_power();
    }

    if (DRAM_POWER_EN) {
        STATE_PRINTN(ch, "-------------------- Dram Power Consumption -------------------------------------------\n");
        double energy; string unit;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            auto power = mptc_->getRank(ch, rank)->DramPower[0];
            UnitConvert(&energy, &unit, power.act_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"ActEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.pre_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"PreEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.rd_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"RdEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.wr_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"WrEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.act_standby_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"ActIdleEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.pre_standby_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"PreIdleEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.refpb_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"RefpbEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.refab_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"RefabEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.asref_refab_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"SrefRefEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.asref_pre_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"SrefIdleEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.asref_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"SrefEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.act_pd_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"ActPdEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.idle_pd_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"IdlePdEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.srpd_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"SrPdEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD0"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd0Energy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.IddEnergy["IDD2N"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd2nEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD2P"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd2pEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD3N"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd3nEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.IddEnergy["IDD3P"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd3pEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD4W"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd4wEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD4R"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd4rEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.IddEnergy["IDD5"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd5Energy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD6"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd6Energy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.VddEnergy["VDD1"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Vdd1Energy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.VddEnergy["VDD2H"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Vdd2hEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.VddEnergy["VDD2L"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Vdd2lEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.VddEnergy["VDDQH"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"VddqhEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.VddEnergy["VDDQL"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"VddqlEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD0"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd0Average("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD2N"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd2nAverage("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD2P"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd2pAverage("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD3N"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd3nAverage("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD3P"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd3pAverage("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD4W"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd4wAverage("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD4R"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd4rAverage("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD5"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd5Average("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, endl);
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD6"]);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"Idd6Average("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"EngEffi(pJ/bit)");
            STATE_PRINTN(ch, " : "<<setw(8)<<power.energy_efficiency<<" | ");
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"AvgPower(mW)");
            STATE_PRINTN(ch, " : "<<setw(8)<<power.average_power<<" | ");
            STATE_PRINTN(ch, endl);
        }
        double total_energy = 0, total_power = 0;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            auto power = mptc_->getRank(ch, rank)->DramPower[0];
            UnitConvert(&energy, &unit, power.total_energy);
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<setw(18)<<"AllEnergy("+unit+")");
            STATE_PRINTN(ch, " : "<<setw(8)<<energy<<setw(3)<<" | ");
            total_energy += power.total_energy;
        }
        UnitConvert(&energy, &unit, total_energy);
        STATE_PRINTN(ch, setw(24)<<"TotalEnergy("+unit+")"<<" : "<<setw(8)<<energy<<setw(3)<<" | "<<endl);
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            total_power += mptc_->getRank(ch, rank)->DramPower[0].average_power;
        }
        STATE_PRINTN(ch, setw(24)<<"TotalAvgPower(mW)"<<" : "<<setw(8)<<total_power<<setw(3)<<" | "<<endl);
    }
    STATE_PRINTN(ch, "-------------------- Task Statistics (PTC Command Number) -----------------------------\n");
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN(ch, "Rank"<<rank<<" read      : "<<setw(8)<<
                (mptc_->getPTC(ch)->racc_rank_cnt[rank]-pre_racc_rank_cnt[ch][rank])<<" | ");
        STATE_PRINTN(ch, "Rank"<<rank<<" total read  : "<<setw(8)<<mptc_->getPTC(ch)->racc_rank_cnt[rank]<<" | ");
        if (rank == NUM_RANKS - 1) STATE_PRINTN(ch, endl);
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN(ch, "Rank"<<rank<<" write     : "<<setw(8)<<
                (mptc_->getPTC(ch)->wacc_rank_cnt[rank]-pre_wacc_rank_cnt[ch][rank])<<" | ");
        STATE_PRINTN(ch, "Rank"<<rank<<" total write : "<<setw(8)<<mptc_->getPTC(ch)->wacc_rank_cnt[rank]<<" | ");
        if (rank == NUM_RANKS - 1) STATE_PRINTN(ch, endl);
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN(ch, "Rank"<<rank<<" r+w       : "<<setw(8)<<
                (mptc_->getPTC(ch)->acc_rank_cnt[rank]-pre_acc_rank_cnt[ch][rank])<<" | ");
        STATE_PRINTN(ch, "Rank"<<rank<<" total r+w   : "<<setw(8)<<mptc_->getPTC(ch)->acc_rank_cnt[rank]<<" | ");
        if (rank == NUM_RANKS - 1) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "Read            : "<<setw(8)<<reads-pre_reads[ch]);
    STATE_PRINTN(ch, " | Total reads       : "<<setw(8)<<reads);
    STATE_PRINTN(ch, " | Write           : "<<setw(8)<<writes-pre_writes[ch]);
    STATE_PRINTN(ch, " | Total writes      : "<<setw(8)<<writes<<" |"<<endl);
    STATE_PRINTN(ch, "Total           : "<<setw(8)<<totals-pre_totals[ch]);
    STATE_PRINTN(ch, " | Total commands    : "<<setw(8)<<totals);
    STATE_PRINTN(ch, " | DMC             : "<<setw(8)<<access_cnt[ch]);
    STATE_PRINTN(ch, " | Total pre_act     : "<<setw(8)<<mptc_->getPTC(ch)->total_pre_act_cnt<<" |"<<endl);
    STATE_PRINTN(ch, "Total 32B Read  : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcRd32B);
    STATE_PRINTN(ch, " | Total 32B Write   : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcWr32B);
    STATE_PRINTN(ch, " | Total 64B Read  : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcRd64B);
    STATE_PRINTN(ch, " | Total 64B Write   : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcWr64B<<" |"<<endl);
    STATE_PRINTN(ch, "Total 128B Read : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcRd128B);
    STATE_PRINTN(ch, " | Total 128B Write  : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcWr128B);
    STATE_PRINTN(ch, " | Total 256B Read : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcRd256B);
    STATE_PRINTN(ch, " | Total 256B Write  : "<<setw(8)<<mptc_->getPTC(ch)->TotalDmcWr256B<<" |"<<endl);
    STATE_PRINTN(ch, "IECC            : "<<setw(8)<<mptc_->getPTC(ch)->total_iecc_cnt);
    STATE_PRINTN(ch, " | No IECC           : "<<setw(8)<<mptc_->getPTC(ch)->total_noiecc_cnt<<" |"<<endl);
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            unsigned b = rank * NUM_BANKS + bank;
            STATE_PRINTN(ch, "Bank"<<setw(2)<<b<<" ");
            STATE_PRINTN(ch, "read : "<<setw(6)<<mptc_->getPTC(ch)->racc_bank_cnt[b]<<" | ");
            STATE_PRINTN(ch, "write : "<<setw(6)<<mptc_->getPTC(ch)->wacc_bank_cnt[b]<<" | ");
            STATE_PRINTN(ch, "total : "<<setw(6)<<mptc_->getPTC(ch)->acc_bank_cnt[b]<<" | ");
        }
        STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "-------------------- Task Statistics (PERF Command Number) -----------------------------\n");
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            unsigned b = rank * NUM_BANKS + bank;
            STATE_PRINTN(ch, "Bank"<<setw(2)<<b<<" ");
            STATE_PRINTN(ch, "read : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bank_rcnt[b] - pre_perf_bank_rcnt[ch][b]<<" | ");
            STATE_PRINTN(ch, "write : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bank_wcnt[b] - pre_perf_bank_wcnt[ch][b]<<" | ");
        }
        STATE_PRINTN(ch, endl);
    }
    if (NUM_GROUPS > 0) {
        for (size_t group = 0; group < NUM_GROUPS; group ++) {
            for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<" Bg"<<setw(2)<<group<<" ");
                STATE_PRINTN(ch, "read : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bg_rcnt[rank][group] - pre_perf_bg_rcnt[ch][rank][group]<<" | ");
                STATE_PRINTN(ch, "write : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bg_wcnt[rank][group] - pre_perf_bg_wcnt[ch][rank][group]<<" | ");
            }
            STATE_PRINTN(ch, endl);
        }
    }
    STATE_PRINTN(ch, "-------------------- Perf2Ptc schedule (Counter) ---------------------------\n");
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            unsigned b = rank * NUM_BANKS + bank;
            STATE_PRINTN(ch, "Bank"<<setw(2)<<b<<" ");
            STATE_PRINTN(ch, "read : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_rcnt[b] - pre_perf2ptc_bank_rcnt[ch][b]<<" | ");
            STATE_PRINTN(ch, "write : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_wcnt[b] - pre_perf2ptc_bank_wcnt[ch][b]<<" | ");
//            STATE_PRINTN(ch, "total : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_cnt[b] - pre_perf2ptc_bank_cnt[b]<<" | ");
        }
        STATE_PRINTN(ch, endl);
    }
    if (NUM_GROUPS > 0) {
        for (size_t group = 0; group < NUM_GROUPS; group ++) {
            for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<" Bg"<<setw(2)<<group<<" ");
                STATE_PRINTN(ch, "read : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bg_rcnt[rank][group] - pre_perf2ptc_bg_rcnt[ch][rank][group]<<" | ");
                STATE_PRINTN(ch, "write : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bg_wcnt[rank][group] - pre_perf2ptc_bg_wcnt[ch][rank][group]<<" | ");
            }
            STATE_PRINTN(ch, endl);
        }
    }
    STATE_PRINTN(ch, "-------------------- Ptc schedule (Counter) ---------------------------\n");
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            unsigned b = rank * NUM_BANKS + bank;
            STATE_PRINTN(ch, "Bank"<<setw(2)<<b<<" ");
            STATE_PRINTN(ch, "read : "<<setw(6)<<mptc_->getPTC(ch)->ptc_r_bank_cnt[b] - pre_ptc_r_bank_cnt[ch][b]<<" | ");
            STATE_PRINTN(ch, "write : "<<setw(6)<<mptc_->getPTC(ch)->ptc_w_bank_cnt[b] - pre_ptc_w_bank_cnt[ch][b]<<" | ");
//            STATE_PRINTN(ch, "total : "<<setw(6)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_cnt[b] - pre_perf2ptc_bank_cnt[b]<<" | ");
        }
        STATE_PRINTN(ch, endl);
    }
    
    double sum = 0.0;
    double sumsq = 0.0;
    uint64_t min_cnt = uint64_t(-1);
    uint64_t max_cnt = 0;
    unsigned n = 0;
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            unsigned b = rank * NUM_BANKS + bank;
            uint64_t cnt = (mptc_->getPTC(ch)->ptc_r_bank_cnt[b] - pre_ptc_r_bank_cnt[ch][b]) +
                    (mptc_->getPTC(ch)->ptc_w_bank_cnt[b] - pre_ptc_w_bank_cnt[ch][b]);
            sum += double(cnt);
            sumsq += double(cnt) * double(cnt);
            if (cnt < min_cnt) min_cnt = cnt;
            if (cnt > max_cnt) max_cnt = cnt;
            n++;
        }
    }
    double mean = (n == 0) ? 0.0 : (sum / double(n));
    double var = (n == 0) ? 0.0 : (sumsq / double(n) - mean * mean);
    if (var < 0.0) var = 0.0;
    double stddev = sqrt(var);
    STATE_PRINTN(ch, "Ptc schedule bank cmd stddev : "<<fixed<<setprecision(2)<<setw(6)<<stddev);
    STATE_PRINTN(ch, " | mean : "<<fixed<<setprecision(2)<<setw(6)<<mean);
    STATE_PRINTN(ch, " | min/max : "<<min_cnt<<"/"<<max_cnt<<endl);
    
    if (NUM_GROUPS > 0) {
        for (size_t group = 0; group < NUM_GROUPS; group ++) {
            for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<" Bg"<<setw(2)<<group<<" ");
                STATE_PRINTN(ch, "read : "<<setw(6)<<mptc_->getPTC(ch)->ptc_r_bg_cnt[rank][group] - pre_ptc_r_bg_cnt[ch][rank][group]<<" | ");
                STATE_PRINTN(ch, "write : "<<setw(6)<<mptc_->getPTC(ch)->ptc_w_bg_cnt[rank][group] - pre_ptc_w_bg_cnt[ch][rank][group]<<" | ");
            }
            STATE_PRINTN(ch, endl);
        }
    }
    STATE_PRINTN(ch, "-------------------- Row Conflict Precharge Count (Counter) ---------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"RowConfPre");
        STATE_PRINTN(ch, " : "<<setw(6)<<mptc_->getPTC(ch)->rowconf_pre_cnt[bank]<<" | ");
        if (bank % 4 == 3) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "-------------------- Page Timeout Count (Counter) -------------------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"PageTimeout");
        STATE_PRINTN(ch, " : "<<setw(6)<<mptc_->getPTC(ch)->pageto_pre_cnt[bank]<<" | ");
        if (bank % 4 == 3) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "-------------------- Enhance Page Adapt Level Count (Counter) -------------------------\n");
    size = MAP_CONFIG["ENH_PAGE_ADPT_TIME"].size();
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN(ch, "Rank"<<i<<" "<<setw(13)<<"ENH_PAGE_ADAPT"<<" : ");
        for (size_t j = 0; j < size; j ++) {
            STATE_PRINTN(ch, "Level"<<j<<" : "<<setw(13)<<mptc_->getPTC(ch)->ehs_page_adapt_cnt[i][j]<<" | ");
        }
        STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "-------------------- Rhit Break Precharge Count (Counter) -----------------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"RhitBreakPre");
        STATE_PRINTN(ch, " : "<<setw(6)<<mptc_->getPTC(ch)->rhit_break_pre_cnt[bank]<<" | ");
        Total_rhit_break_pre_cnt[ch] += mptc_->getPTC(ch)->rhit_break_pre_cnt[bank]; 
        if (bank % 4 == 3) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "Total"<<setw(2)<<" "<<setw(13)<<"RhitBreakPre");
    STATE_PRINTN(ch, " : "<<setw(6)<<Total_rhit_break_pre_cnt[ch]<<" | ");
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "-------------------- Dummy Tout Precharge Count (Counter) -----------------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"DummyToutPre");
        STATE_PRINTN(ch, " : "<<setw(6)<<mptc_->getPTC(ch)->dummy_tout_pre_cnt[bank]<<" | ");
        Total_dummy_tout_pre_cnt[ch] += mptc_->getPTC(ch)->dummy_tout_pre_cnt[bank]; 
        if (bank % 4 == 3) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "Total"<<setw(2)<<" "<<setw(13)<<"DummyToutPre");
    STATE_PRINTN(ch, " : "<<setw(6)<<Total_dummy_tout_pre_cnt[ch]<<" | ");
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "-------------------- Func Precharge Count (Counter) -----------------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"FuncPrecharge");
        STATE_PRINTN(ch, " : "<<setw(6)<<mptc_->getPTC(ch)->func_pre_cnt[bank]<<" | ");
        Total_func_pre_cnt[ch] += mptc_->getPTC(ch)->func_pre_cnt[bank]; 
        if (bank % 4 == 3) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "Total"<<setw(2)<<" "<<setw(13)<<"FuncPrecharge");
    STATE_PRINTN(ch, " : "<<setw(6)<<Total_func_pre_cnt[ch]<<" | ");
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "-------------------- Func ABR Count (Counter) -----------------------------------------\n");     //todo: revise for e-mode
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        for (size_t sub_channel = 0; sub_channel < mptc_->getPTC(ch)->sc_num; sub_channel ++) {
            STATE_PRINTN(ch, "Rank"<<setw(2)<<rank<<" & Sc"<<setw(2)<<sub_channel<<setw(13)<<"FuncAbr"<<" : "<<setw(6));
            STATE_PRINTN(ch, (mptc_->getPTC(ch)->rank_refresh_cnt[rank][sub_channel] - pre_abr_cnt[ch][rank][sub_channel])<<" | ");
        }
        if (rank % 4 == 3 || rank == NUM_RANKS - 1) STATE_PRINTN(ch, endl);
    }
    STATE_PRINTN(ch, "-------------------- Func PBR Count (Counter) -----------------------------------------\n");
    if (ENH_PBR_EN) {
        for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
            STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"FuncPbr"<<" : "<<setw(6));
            STATE_PRINTN(ch, (mptc_->getPTC(ch)->perbank_refresh_cnt[bank] - pre_pbr_cnt[ch][bank])<<" | ");
            if (bank % 4 == 3) STATE_PRINTN(ch, endl);
        }
    
    } else {
        for (size_t bank = 0; bank < NUM_RANKS * mptc_->getPTC(ch)->pbr_bank_num * mptc_->getPTC(ch)->sc_num; bank ++) {
            STATE_PRINTN(ch, "Bank"<<setw(2)<<bank<<" "<<setw(13)<<"FuncPbr"<<" : "<<setw(6));
            STATE_PRINTN(ch, (mptc_->getPTC(ch)->perbank_refresh_cnt[bank] - pre_pbr_cnt[ch][bank])<<" | ");
            if (bank % 4 == 3) STATE_PRINTN(ch, endl);
        }
    }

    STATE_PRINTN(ch, "-------------------- Request Statistics (DDR Command Number) --------------------------\n");
    row_hit_cnt = ((read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt) > act_cnt) ?
            (read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt - act_cnt) : 0;
    row_miss_cnt = act_cnt;

    STATE_PRINTN(ch, setw(36)<<"Active cnt"<<" : "<<setw(12)<<act_cnt - pre_act_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total act cnt"<<" : "<<act_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Disturb active cnt"<<" : "<<setw(12)<<act_dst_cnt - pre_act_dst_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total disturb active cnt"<<" : "<<act_dst_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Bypass active cnt"<<" : "<<setw(12)<<byp_act_cnt);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total byp act cnt"<<" : "<<byp_act_cnt<<endl);

    float bypass_act_ratio =  ((float)byp_act_cnt / act_cnt);
    STATE_PRINTN(ch, setw(36)<<"Bypass active cnt ratio"<<" : "<<setw(12)<<bypass_act_ratio);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Bypass active cnt ratio"<<" : "<<bypass_act_ratio<<endl);
    STATE_PRINTN(ch, setw(36)<<"Precharge_sb cnt"<<" : "<<setw(12)<<pre_sb_cnt - pre_pre_sb_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total precharge_sb cnt"<<" : "<<pre_sb_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Precharge_pb cnt"<<" : "<<setw(12)<<pre_pb_cnt - pre_pre_pb_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total precharge_pb cnt"<<" : "<<pre_pb_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Disturb precharge_pb cnt"<<" : "<<setw(12)<<pre_pb_dst_cnt - pre_pre_pb_dst_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total disturb precharge_pb cnt"<<" : "<<pre_pb_dst_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Precharge_ab cnt"<<" : "<<setw(12)<<pre_ab_cnt - pre_pre_ab_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total precharge_ab cnt"<<" : "<<pre_ab_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Read cnt"<<" : "<<setw(12)<<read_cnt - pre_read_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total read cnt"<<" : "<<read_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Write"<<" : "<<setw(12)<<write_cnt - pre_write_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total write"<<" : "<<write_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Read with AP"<<" : "<<setw(12)<<read_p_cnt - pre_read_p_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total read with AP"<<" : "<<read_p_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Write with AP"<<" : "<<setw(12)<<write_p_cnt - pre_write_p_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total write with AP"<<" : "<<write_p_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Mask write"<<" : "<<setw(12)<<mwrite_cnt - pre_mwrite_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total mask write"<<" : "<<mwrite_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Mask write with AP"<<" : "<<setw(12)<<mwrite_p_cnt - pre_mwrite_p_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total mask write with AP"<<" : "<<mwrite_p_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Row hit"<<" : "<<setw(12)<<row_hit_cnt - pre_row_hit_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total row hit"<<" : "<<row_hit_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Row miss cnt"<<" : "<<setw(12)<<row_miss_cnt - pre_row_miss_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total row miss cnt"<<" : "<<row_miss_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"ecc read cnt"<<" : "<<setw(12)<<ecc_read_cnt - pre_ecc_read_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total ecc read cnt"<<" : "<<ecc_read_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"ecc write cnt"<<" : "<<setw(12)<<ecc_write_cnt - pre_ecc_write_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total ecc write cnt"<<" : "<<ecc_write_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"merge read cnt"<<" : "<<setw(12)<<merge_read_cnt - pre_merge_read_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total merge read cnt"<<" : "<<merge_read_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"fast read cnt"<<" : "<<setw(12)<<fast_read_cnt - pre_fast_read_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total fast read cnt"<<" : "<<fast_read_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"fast act cnt"<<" : "<<setw(12)<<fast_act_cnt - pre_fast_act_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total fast act cnt"<<" : "<<fast_act_cnt<<endl);

    unsigned total_cnt = row_hit_cnt + row_miss_cnt;
    float total_row_hit_ratio = (total_cnt == 0) ? 0 : ((float)row_hit_cnt / total_cnt);
    float total_row_miss_ratio = (total_cnt == 0) ? 0 : ((float)row_miss_cnt / total_cnt);

    unsigned cur_total_cnt = row_hit_cnt - pre_row_hit_cnt[ch] + row_miss_cnt - pre_row_miss_cnt[ch];
    float cur_total_row_hit_ratio = (cur_total_cnt == 0) ? 0 :
            ((float)(row_hit_cnt>pre_row_hit_cnt[ch]?(row_hit_cnt-pre_row_hit_cnt[ch]):0)/cur_total_cnt);
    float cur_total_row_miss_ratio = (cur_total_cnt == 0) ? 0 :
            ((float)(row_miss_cnt>pre_row_miss_cnt[ch]?(row_miss_cnt-pre_row_miss_cnt[ch]):0)/cur_total_cnt);

    STATE_PRINTN(ch, setw(36)<<"Row hit ratio"<<" : "<<setw(12)<<cur_total_row_hit_ratio);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total row hit ratio"<<" : "<<total_row_hit_ratio<<endl);
    STATE_PRINTN(ch, setw(36)<<"Row miss ratio"<<" : "<<setw(12)<<cur_total_row_miss_ratio);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total row miss ratio"<<" : "<<total_row_miss_ratio<<endl);
    STATE_PRINTN(ch, setw(36)<<"Timeout cnt"<<" : "<<setw(12)<<timeout_cnt - pre_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total timeout cnt"<<" : "<<timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Fast timeout cnt"<<" : "<<setw(12)<<fast_timeout_cnt - pre_fast_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total fast timeout cnt"<<" : "<<fast_timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Slow timeout cnt"<<" : "<<setw(12)<<slow_timeout_cnt - pre_slow_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total slow timeout cnt"<<" : "<<slow_timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Real Time Timeout cnt"<<" : "<<setw(12)<<rt_timeout_cnt - pre_rt_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total RT timeout cnt"<<" : "<<rt_timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Perf rd Timeout cnt"<<" : "<<setw(12)<<perf_rd_timeout_cnt - pre_perf_rd_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total Perf rd timeout cnt"<<" : "<<perf_rd_timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Perf wr Timeout cnt"<<" : "<<setw(12)<<perf_wr_timeout_cnt - pre_perf_wr_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total Perf wr timeout cnt"<<" : "<<perf_wr_timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Dummy Timeout cnt"<<" : "<<setw(12)<<dummy_timeout_cnt - pre_dummy_timeout_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total dummy timeout cnt"<<" : "<<dummy_timeout_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Dummy hqos cnt"<<" : "<<setw(12)<<dummy_hqos_cnt - pre_dummy_hqos_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total dummy hqos cnt"<<" : "<<dummy_hqos_cnt<<endl);

    STATE_PRINTN(ch, "-------------------- Confilct Statistics (DDR Command Number) -------------------------\n");
    STATE_PRINTN(ch, setw(36)<<"Address conflict"<<" : "<<setw(12)<<address_conf_cnt - pre_address_conf_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total address conf cnt"<<" : "<<address_conf_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Perf Addr conflict"<<" : "<<setw(12)<<perf_address_conf_cnt - pre_perf_address_conf_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total Perf addr conf cnt"<<" : "<<perf_address_conf_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Id conflict"<<" : "<<setw(12)<<id_conf_cnt - pre_id_conf_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total id conf cnt"<<" : "<<id_conf_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Bank conflict"<<" : "<<setw(12)<<ba_conf_cnt - pre_ba_conf_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total bank conf cnt"<<" : "<<ba_conf_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"All conflict"<<" : "<<setw(12)<<total_conf - pre_total_conf[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total all conf cnt"<<" : "<<total_conf<<endl);
    STATE_PRINTN(ch, setw(36)<<"RW switch cnt"<<" : "<<setw(12)<<rw_switch_cnt - pre_rw_switch_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total RW switch cnt"<<" : "<<rw_switch_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Rank switch cnt"<<" : "<<setw(12)<<rank_switch_cnt - pre_rank_switch_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total rank switch cnt"<<" : "<<rank_switch_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"R rank switch cnt"<<" : "<<setw(12)<<r_rank_switch_cnt - pre_r_rank_switch_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total rrank switch cnt"<<" : "<<r_rank_switch_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"W rank switch cnt"<<" : "<<setw(12)<<w_rank_switch_cnt - pre_w_rank_switch_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total wrank switch cnt"<<" : "<<w_rank_switch_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Sid switch cnt"<<" : "<<setw(12)<<sid_switch_cnt - pre_sid_switch_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total sid switch cnt"<<" : "<<sid_switch_cnt<<endl);
    STATE_PRINTN(ch, setw(36)<<"Rw idle cnt"<<" : "<<setw(12)<<rw_idle_cnt - pre_rw_idle_cnt[ch]);
    STATE_PRINTN(ch, " | "<<setw(36)<<"Total rw idle cnt"<<" : "<<rw_idle_cnt<<endl);
    if (rank_switch_cnt == 0) {
        STATE_PRINTN(ch, setw(36)<<"Average rank cnt"<<" : "<<setw(12)<<0);
    } else {
        STATE_PRINTN(ch, setw(36)<<"Average rank cnt"<<" : "<<setw(12)<<totals / rank_switch_cnt);
    }
    if (sid_switch_cnt == 0) {
        STATE_PRINTN(ch, " | "<<setw(36)<<"Average sid cnt"<<" : 0"<<endl);
    } else {
        STATE_PRINTN(ch, " | "<<setw(36)<<"Average sid cnt"<<" : "<<totals / sid_switch_cnt<<endl);
    }

    STATE_PRINTN(ch, "-------------------- Latency Statistics (ns less than) --------------------------------\n");
    size = mptc_->getPTC(ch)->lat_dly_cnt.size();
    for (size_t mid = 0; mid <= size; mid ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(6)<<"LatTh"<<" |");
    for (size_t i = 0; i < size; i ++) {
        STATE_PRINTN(ch, setw(6)<<mptc_->getPTC(ch)->lat_dly_step[i]<<" |");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(6)<<"LatNe"<<" |");
    for (size_t i = 0; i < size; i ++) {
        STATE_PRINTN(ch, setw(6)<<mptc_->getPTC(ch)->lat_dly_cnt[i]<<" |");
    }
    STATE_PRINTN(ch, endl);
    for (size_t mid = 0; mid <= size; mid ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);

    STATE_PRINTN(ch, "-------------------- Latency Statistics (ns) ------------------------------------------\n");
    STATE_PRINTN(ch, "max latency : "<<float(mptc_->getPTC(ch)->max_delay) * tDFI
            <<", task="<<mptc_->getPTC(ch)->max_delay_id<<", ");
    STATE_PRINTN(ch, "min latency : "<<float(mptc_->getPTC(ch)->min_delay) * tDFI
            <<", task="<<mptc_->getPTC(ch)->min_delay_id<<endl);
    STATE_PRINTN(ch, "high qos max latency : "<<float(mptc_->getPTC(ch)->highqos_max_delay) * tDFI
            <<", task="<<mptc_->getPTC(ch)->highqos_max_delay_id<<endl);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN(ch, "total ddrc average latency rank"<<rank<<" : "
                <<mptc_->getPTC(ch)->ddrc_av_lat_rank[rank] * tDFI<<endl);
    }
    if ((mptc_->getPTC(ch)->com_read_cnt - pre_com_read_cnt[ch]) > 0) {
        float avg_lat = float(mptc_->getPTC(ch)->total_latency - pre_total_latency[ch]) /
                (mptc_->getPTC(ch)->com_read_cnt - pre_com_read_cnt[ch]);
        STATE_PRINTN(ch, "int ddrc average latency : "<<avg_lat * tDFI<<endl);
    } else {
        STATE_PRINTN(ch, "int ddrc average latency : 0"<<endl);
    }
    STATE_PRINTN(ch, "total ddrc average latency : "<<mptc_->getPTC(ch)->ddrc_av_lat * tDFI<<endl);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN(ch, "total ddrc average high qos latency rank"<<rank<<" : "
                <<mptc_->getPTC(ch)->rank_ddrc_av_highqos_lat[rank] * tDFI<<endl);
    }
    STATE_PRINTN(ch, "total ddrc average high qos latency : "<<mptc_->getPTC(ch)->ddrc_av_highqos_lat * tDFI<<endl);
    STATE_PRINTN(ch, "high qos trigger read group swiching cnt : "<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_highqos_trig_grpsw_cnt<<endl);
    float cmd2dfi_ave_lat = (mptc_->getPTC(ch)->cmd_in2dfi_cnt == 0) ? 0 :
            (float(mptc_->getPTC(ch)->cmd_in2dfi_lat) / mptc_->getPTC(ch)->cmd_in2dfi_cnt);
    STATE_PRINTN(ch, "total cmd2dfi average latency : "<<cmd2dfi_ave_lat * tDFI<<endl);
    float cmd2dfi_ave_lat_int = ((mptc_->getPTC(ch)->cmd_in2dfi_cnt - pre_cmd_in2dfi_cnt[ch]) == 0) ? 0 :
            (float(mptc_->getPTC(ch)->cmd_in2dfi_lat - pre_cmd_in2dfi_lat[ch]) / (mptc_->getPTC(ch)->cmd_in2dfi_cnt - pre_cmd_in2dfi_cnt[ch]));
    STATE_PRINTN(ch, "int cmd2dfi average latency : "<<cmd2dfi_ave_lat_int * tDFI<<endl);
    float cmd_rdmet_cnt = (mptc_->getPTC(ch)->cmd_in2dfi_cnt == 0) ? 0 :
            (float(mptc_->getPTC(ch)->cmd_rdmet_cnt) / mptc_->getPTC(ch)->cmd_in2dfi_cnt);
    STATE_PRINTN(ch, "total read timing met average cnt : "<<cmd_rdmet_cnt<<endl);
    unsigned inhere_lat = tD_D+tCMD_PHY+tDAT_PHY+tCMD_RASC+tDAT_RASC+tCMD_ADAPT+RL+(BLEN/2/WCK2DFI_RATIO)+2;
    STATE_PRINTN(ch, "inhere latency : "<<float(inhere_lat) * tDFI<<endl);
    pre_total_latency[ch] = mptc_->getPTC(ch)->total_latency;
    pre_com_read_cnt[ch] = mptc_->getPTC(ch)->com_read_cnt;

    STATE_PRINTN(ch, "Pd Block Read cnt : "<<mptc_->getPTC(ch)->rd_met_pd_cnt<<", Average cnt : "
            <<(float(mptc_->getPTC(ch)->rd_met_pd_cnt)/mptc_->getPTC(ch)->totalReads)<<endl);
    STATE_PRINTN(ch, "Asref Block Read cnt : "<<mptc_->getPTC(ch)->rd_met_asref_cnt<<", Average cnt : "
            <<(float(mptc_->getPTC(ch)->rd_met_asref_cnt)/mptc_->getPTC(ch)->totalReads)<<endl);
    STATE_PRINTN(ch, "Cmd Met Pd cnt : "<<mptc_->getPTC(ch)->cmd_met_pd_cnt<<", Average cnt : "
            <<(float(mptc_->getPTC(ch)->cmd_met_pd_cnt)/mptc_->getPTC(ch)->totalTransactions)<<endl);
    STATE_PRINTN(ch, "Cmd Met Asref cnt : "<<mptc_->getPTC(ch)->cmd_met_asref_cnt<<", Average cnt : "
            <<(float(mptc_->getPTC(ch)->cmd_met_asref_cnt)/mptc_->getPTC(ch)->totalTransactions)<<endl);
    STATE_PRINTN(ch, "PBR Block Read cnt : "<<mptc_->getPTC(ch)->rd_met_pbr_cnt<<", Average cnt : "
            <<(float(mptc_->getPTC(ch)->rd_met_pbr_cnt)/mptc_->getPTC(ch)->totalReads)<<endl);
    STATE_PRINTN(ch, "ABR Block Read cnt : "<<mptc_->getPTC(ch)->rd_met_abr_cnt<<", Average cnt : "
            <<(float(mptc_->getPTC(ch)->rd_met_abr_cnt)/mptc_->getPTC(ch)->totalReads)<<endl);
    STATE_PRINTN(ch, "PBR Block Cmd cycle : "<<mptc_->getPTC(ch)->pbr_block_allcmd_cycle<<", PBR cycle : "<<mptc_->getPTC(ch)->pbr_cycle<<", ref_block_ratio : "
            <<(float(mptc_->getPTC(ch)->pbr_block_allcmd_cycle)/mptc_->getPTC(ch)->pbr_cycle)<<endl);
    float samerow_ratio = 100 * float(mptc_->getPTC(ch)->samerow_mask_rdcnt) / totals;
    STATE_PRINTN(ch, "Same row mask read cnt : "<<mptc_->getPTC(ch)->samerow_mask_rdcnt);
    STATE_PRINTN(ch, ", ratio : "<<samerow_ratio<<"%"<<endl);
    samerow_ratio = 100 * float(mptc_->getPTC(ch)->samerow_mask_wrcnt) / totals;
    STATE_PRINTN(ch, "Same row mask write cnt : "<<mptc_->getPTC(ch)->samerow_mask_wrcnt);
    STATE_PRINTN(ch, ", ratio : "<<samerow_ratio<<"%"<<endl);
    float page_exceed_ratio = (mptc_->getPTC(ch)->precharge_pb_cnt == 0) ? 0 :
            100 * mptc_->getPTC(ch)->page_exceed_cnt / mptc_->getPTC(ch)->precharge_pb_cnt;
    STATE_PRINTN(ch, "Precharge_pb cnt : "<<mptc_->getPTC(ch)->precharge_pb_cnt);
    STATE_PRINTN(ch, ", Page open time exceed cnt : "<<mptc_->getPTC(ch)->page_exceed_cnt);
    STATE_PRINTN(ch, ", ratio : "<<page_exceed_ratio<<"%"<<endl);
    if (TRFC_CC_EN) {
        float cc_ratio = 100 * double(mptc_->getPTC(ch)->abr_cc_cnt) / double(now());
        STATE_PRINTN(ch, "Abr close clock : "<<mptc_->getPTC(ch)->abr_cc_cnt);
        STATE_PRINTN(ch, ", ratio : "<<cc_ratio<<"%"<<endl);
        cc_ratio = 100 * double(mptc_->getPTC(ch)->pbr_cc_cnt) / double(now());
        STATE_PRINTN(ch, "Pbr close clock : "<<mptc_->getPTC(ch)->pbr_cc_cnt);
        STATE_PRINTN(ch, ", ratio : "<<cc_ratio<<"%"<<endl);
    }
    STATE_PRINTN(ch, "Negative R2W Rank Switch cnt : "<<mptc_->getPTC(ch)->neg_r2w_rank_sw<<endl);
    STATE_PRINTN(ch, "Long Gap R cnt : "<<mptc_->getPTC(ch)->r_lgap_cnt<<endl);
    STATE_PRINTN(ch, "Long Gap W cnt : "<<mptc_->getPTC(ch)->w_lgap_cnt<<endl);
    STATE_PRINTN(ch, "Long Gap R2W cnt : "<<mptc_->getPTC(ch)->r2w_lgap_cnt<<endl);
    STATE_PRINTN(ch, "Long Gap W2R cnt : "<<mptc_->getPTC(ch)->w2r_lgap_cnt<<endl);

#if 0
    for (size_t i = 0; i < 32; i ++) {
        STATE_PRINTN(ch, setw(10)<<(mptc_->getPTC(ch)->samerow_bit_rdcnt[i]+mptc_->getPTC(ch)->samerow_bit_wrcnt[i])<<endl);
    }
#endif
    uint32_t total_qos_cnt = 0;
    for (size_t i = 0; i < 8; i ++) {
        total_qos_cnt += mptc_->getPTC(ch)->qos_cnt[i];
    }
    STATE_PRINTN(ch, "-------------------- QOS Statistics (Total DMC Command Number/ns) ---------------------\n");
    STATE_PRINTN(ch, "qos "<<"|Count   "<<"|Ratio "<<"|Tout cnt "<<"|Tout fcnt "<<"|Tout scnt "<<"|Av lat "<<"|0~50      "
            <<"|50~100    "<<"|100~150   "<<"|150~200   "<<"|200~300   "<<"|300~500   "
            <<"|500~1000  "<<"|1000~2000 "<<"|2000~max  |"<<endl);
    for (size_t qos = 0; qos < 8; qos ++) {
        STATE_PRINTN(ch, "q"<<setw(3)<<qos<<"|"<<setw(8)<<mptc_->getPTC(ch)->qos_cnt[qos]<<"|");
        STATE_PRINTN(ch, setw(6)<<fixed<<setprecision(2)<<((total_qos_cnt == 0) ? 0 :
                    (float(mptc_->getPTC(ch)->qos_cnt[qos]) * 100 / total_qos_cnt))<<"|");
        STATE_PRINTN(ch, setw(9)<<mptc_->getPTC(ch)->qos_timeout_cnt[qos]<<"|");
        STATE_PRINTN(ch, setw(10)<<mptc_->getPTC(ch)->qos_fast_timeout_cnt[qos]<<"|");
        STATE_PRINTN(ch, setw(10)<<mptc_->getPTC(ch)->qos_slow_timeout_cnt[qos]<<"|");
        STATE_PRINTN(ch, setw(7)<<fixed<<setprecision(2)<<((mptc_->getPTC(ch)->qos_cnt[qos] == 0) ? 0 :
                    (float(mptc_->getPTC(ch)->qos_delay_cnt[qos]) * tDFI / mptc_->getPTC(ch)->qos_cnt[qos]))<<"|");
        for (size_t level = 0; level < 9; level ++) {
            STATE_PRINTN(ch, setw(10)<<mptc_->getPTC(ch)->qos_level_cnt[qos][level]<<"|");
        }
        STATE_PRINTN(ch, endl);
    }

    STATE_PRINTN(ch, "-------------------- MID Statistics (Command Number:Cmd2DQ Latency/ns) ----------------\n");
    for (size_t mid = 0; mid <= MidMax; mid ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "Mid    |");
    for (size_t mid = 0; mid < MidMax; mid ++) {
        STATE_PRINTN(ch, setw(7)<<mid<<"|");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "MidCnt |");
    for (size_t mid = 0; mid < MidMax; mid ++) {
        STATE_PRINTN(ch, setw(7)<<mptc_->getPTC(ch)->mid_cnt[mid]<<"|");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "MidLat |");
    for (size_t mid = 0; mid < MidMax; mid ++) {
        float mid_delay = float(mptc_->getPTC(ch)->mid_delay_cnt[mid]);
        float mid_cnt = float(mptc_->getPTC(ch)->mid_cnt[mid]);
        float mid_latency = (mid_cnt == 0) ? 0 : mid_delay * tDFI / mid_cnt;
        STATE_PRINTN(ch, setw(7)<<mid_latency<<"|");
    }
    STATE_PRINTN(ch, endl);
    for (size_t mid = 0; mid <= MidMax; mid ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);

    STATE_PRINTN(ch, "-------------------- OCC Statistics (Counter Number) ----------------------------------\n");
    STATE_PRINTN(ch, "occ 0 cnt : "<<setw(10)<<mptc_->getPTC(ch)->occ_1_cnt<<" | ");
    STATE_PRINTN(ch, "occ 1 cnt : "<<setw(10)<<mptc_->getPTC(ch)->occ_2_cnt<<" | ");
    STATE_PRINTN(ch, "occ 2 cnt : "<<setw(10)<<mptc_->getPTC(ch)->occ_3_cnt<<" | ");
    STATE_PRINTN(ch, "occ 3 cnt : "<<setw(10)<<mptc_->getPTC(ch)->occ_4_cnt<<endl);

#ifdef SYSARCH_PLATFORM
    STATE_PRINTN(ch, "-------------------- SQRT Statistics (Counter Number) ---------------------------------\n");
    STATE_PRINTN(ch, "DMC availability : "<<(mptc_->getPTC(ch)->avai_sqrt)<<endl);
#endif

    STATE_PRINTN(ch, "-------------------- PERF Statistics (Counter Number) ---------------------------------\n");
    STATE_PRINTN(ch, setw(15)<<"Gbuf read"<<" : "<<setw(10)<<(mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->read_cnt-pre_gbuf_reads[ch])<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Total Gbuf read"<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->read_cnt<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Gbuf write"<<" : "<<setw(10)<<(mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->write_cnt-pre_gbuf_writes[ch])<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Total Gbuf write"<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->write_cnt<<" | "<<endl);
    STATE_PRINTN(ch, setw(15)<<"Merge"<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->merge_cnt<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Forward"<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->forward_cnt<<" | ");
    STATE_PRINTN(ch, setw(15)<<"AddrPush"<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->push_cnt<<" | "<<endl);
    pre_gbuf_reads[ch] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->read_cnt;
    pre_gbuf_writes[ch] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->write_cnt;
    for (uint8_t i = 0; i < 6; i ++) {
        string level = "Gbuf lvl" + to_string(i);
        STATE_PRINTN(ch, setw(15)<<level<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->sch_level_cnt[i]-pre_sch_level_cnt[ch][i]<<" | ");
        STATE_PRINTN(ch, setw(15)<<"Total "+level<<" : "<<setw(10)<<mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->sch_level_cnt[i]<<" | "<<endl);
    }

    STATE_PRINTN(ch, "-------------------- PERF Queue Statistics (Percentage/Cycle) --------------------------\n");
    uint32_t perf_total = 0;
    size = perf_que_cnt[ch].size();

    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(7)<<"Qnum"<<"|");
    for (uint32_t index = 0; index < size; index++) {
        perf_total += perf_que_cnt[ch].at(index);
        STATE_PRINTN(ch, setw(7)<<index<<"|");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(7)<<"Per"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        float perf_cnt_dist_ratio = (float(perf_que_cnt[ch].at(index)) * 100) / perf_total;
        STATE_PRINTN(ch, setw(7)<<fixed<<setprecision(3)<<perf_cnt_dist_ratio<<"|");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(7)<<"Cycle"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        STATE_PRINTN(ch, setw(7)<<fixed<<setprecision(3)<<perf_que_cnt[ch].at(index)<<"|");
    }
    STATE_PRINTN(ch, endl);
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);

    for (uint32_t index = 0; index < size; index++) {
        perf_que_cnt[ch].at(index) = 0;
    }

    STATE_PRINTN(ch, "-------------------- DMC Pressure Statistics (Percentage/Cycle) -----------------------\n");
    float ratio = float(task_cnt[ch]) * 100 / STATE_TIME;
    STATE_PRINTN(ch, setw(15)<<"Cmd valid"<<" : "<<setw(10)<<task_cnt[ch]<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | ");
    ratio = float(total_task_cnt[ch]) * 100 / now();
    STATE_PRINTN(ch, setw(15)<<"Total cmd valid"<<" : "<<setw(10)<<total_task_cnt[ch]<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
    ratio = float(bp_cnt[ch]) * 100 / (bp_cnt[ch] + access_cnt[ch]);
    STATE_PRINTN(ch, setw(15)<<"DMC access"<<" : "<<setw(10)<<access_cnt[ch]<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Command bp"<<" : "<<setw(10)<<bp_cnt[ch]<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
    ratio = float(total_bp_cnt[ch]) * 100 / (total_bp_cnt[ch] + total_access_cnt[ch]);
    STATE_PRINTN(ch, setw(15)<<"Total access"<<" : "<<setw(10)<<total_access_cnt[ch]<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Total bp"<<" : "<<setw(10)<<total_bp_cnt[ch]<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Total bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);

    STATE_PRINTN(ch, "-------------------- DMC Queue Statistics (Percentage/Cycle) --------------------------\n");
    uint32_t total = 0;
    size = que_cnt[ch].size();

    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(7)<<"Qnum"<<"|");
    for (uint32_t index = 0; index < size; index++) {
        total += que_cnt[ch].at(index);
        STATE_PRINTN(ch, setw(7)<<index<<"|");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(7)<<"Per"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        float cnt_dist_ratio = (float(que_cnt[ch].at(index)) * 100) / total;
        STATE_PRINTN(ch, setw(7)<<fixed<<setprecision(3)<<cnt_dist_ratio<<"|");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(7)<<"Cycle"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        STATE_PRINTN(ch, setw(7)<<fixed<<setprecision(3)<<que_cnt[ch].at(index)<<"|");
    }
    STATE_PRINTN(ch, endl);
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);

    for (uint32_t index = 0; index < size; index++) {
        que_cnt[ch].at(index) = 0;
    }

    if(DEBUG_PDU){
        STATE_PRINTN(ch, "------------------------- IECC HIT RATE Statistics  -----------------------------\n");
        uint32_t try_cnt=mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->try_count;
        uint32_t hit_cnt=mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->rhit;
        STATE_PRINTN(ch, "IECC Try Hit count : "<< try_cnt<<endl);
        STATE_PRINTN(ch, "IECC Read Hit count : "<< hit_cnt);
        STATE_PRINTN(ch, ", ratio : "<<100*(float)hit_cnt/try_cnt<<"%"<<endl);
        hit_cnt=mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->whit;
        STATE_PRINTN(ch, "IECC Write Hit count : "<< hit_cnt);
        STATE_PRINTN(ch, ", ratio : "<<100*(float)hit_cnt/try_cnt<<"%"<<endl);
        STATE_PRINTN(ch, "IECC FULL Write count : "<< mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->wdata_full_counter<<endl);
        STATE_PRINTN(ch, "IECC MERGE Write count : "<< mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->merge_cnt<<endl);
        for (auto &bufs : mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->wr_ecc_buf) {
            int iecc_n_cnt=0;
            uint32_t iecc_n_shift=bufs.wr_ecc_pos;
            while(iecc_n_shift){
                iecc_n_cnt++;
                iecc_n_shift=iecc_n_shift&(iecc_n_shift-1);
            }
            mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->wpos[iecc_n_cnt]++;
        }
        STATE_PRINTN(ch, "wr_ecc_pos with n ones: "<< endl);
        for(int iecc_i=0;iecc_i<17;iecc_i++){
            STATE_PRINTN(ch, "    "<<iecc_i<<": "<< mras_->getIECC(ch / (NUM_CHANS / NUM_PFQS))->wpos[iecc_i]);
        }
    }
    task_cnt[ch] = 0;
    access_cnt[ch] = 0;
    bp_cnt[ch] = 0;

    //save the value of last time stataes
    pre_reads[ch] = reads;
    pre_writes[ch] = writes;
    pre_totals[ch] = totals;
    pre_address_conf_cnt[ch] = address_conf_cnt;
    pre_perf_address_conf_cnt[ch] = perf_address_conf_cnt;
    pre_id_conf_cnt[ch] = id_conf_cnt;
    pre_ba_conf_cnt[ch] = ba_conf_cnt;
    pre_total_conf[ch] = total_conf;
    pre_act_cnt[ch] = act_cnt;
    pre_act_dst_cnt[ch] = act_dst_cnt;
    pre_pre_sb_cnt[ch] = pre_sb_cnt;
    pre_pre_pb_cnt[ch] = pre_pb_cnt;
    pre_pre_pb_dst_cnt[ch] = pre_pb_dst_cnt;
    pre_pre_ab_cnt[ch] = pre_ab_cnt;
    pre_read_cnt[ch] = read_cnt;
    pre_write_cnt[ch] = write_cnt;
    pre_read_p_cnt[ch] = read_p_cnt;
    pre_write_p_cnt[ch] = write_p_cnt;
    pre_mwrite_cnt[ch] = mwrite_cnt;
    pre_mwrite_p_cnt[ch] = mwrite_p_cnt;
    pre_timeout_cnt[ch] = timeout_cnt;
    pre_fast_timeout_cnt[ch] = fast_timeout_cnt;
    pre_slow_timeout_cnt[ch] = slow_timeout_cnt;
    pre_rt_timeout_cnt[ch] = rt_timeout_cnt;
    pre_perf_rd_timeout_cnt[ch] = perf_rd_timeout_cnt;
    pre_perf_wr_timeout_cnt[ch] = perf_wr_timeout_cnt;
    pre_dummy_timeout_cnt[ch] = dummy_timeout_cnt;
    pre_dummy_hqos_cnt[ch] = dummy_hqos_cnt;
    pre_row_hit_cnt[ch] = row_hit_cnt;
    pre_row_miss_cnt[ch] = row_miss_cnt;
    pre_ecc_read_cnt[ch] = ecc_read_cnt;
    pre_ecc_write_cnt[ch] = ecc_write_cnt;
    pre_merge_read_cnt[ch] = merge_read_cnt;
    pre_fast_read_cnt[ch] = fast_read_cnt;
    pre_fast_act_cnt[ch] = fast_act_cnt;
    pre_rw_switch_cnt[ch] = rw_switch_cnt;
    pre_rank_switch_cnt[ch] = rank_switch_cnt;
    pre_r_rank_switch_cnt[ch] = r_rank_switch_cnt;
    pre_w_rank_switch_cnt[ch] = w_rank_switch_cnt;
    pre_sid_switch_cnt[ch] = sid_switch_cnt;
    pre_rw_idle_cnt[ch] = rw_idle_cnt;
    pre_refresh_pb_cnt[ch] = refresh_pb_cnt;
    pre_refresh_ab_cnt[ch] = refresh_ab_cnt;
    pre_r2w_switch_cnt[ch] = r2w_switch_cnt;
    pre_w2r_switch_cnt[ch] = w2r_switch_cnt;
    pre_phy_notlp_cnt[ch] = phy_notlp_cnt;
    pre_phy_lp_cnt[ch] = phy_lp_cnt;
    pre_acc_rank_cnt[ch] = mptc_->getPTC(ch)->acc_rank_cnt;
    pre_acc_bank_cnt[ch] = mptc_->getPTC(ch)->acc_bank_cnt;
    pre_racc_rank_cnt[ch] = mptc_->getPTC(ch)->racc_rank_cnt;
    pre_racc_bank_cnt[ch] = mptc_->getPTC(ch)->racc_bank_cnt;
    pre_wacc_rank_cnt[ch] = mptc_->getPTC(ch)->wacc_rank_cnt;
    pre_wacc_bank_cnt[ch] = mptc_->getPTC(ch)->wacc_bank_cnt;
    pre_pde_cnt[ch] = pde_cnt;
    pre_asrefe_cnt[ch] = asrefe_cnt;
    pre_srpde_cnt[ch] = srpde_cnt;
    pre_pdx_cnt[ch] = pdx_cnt;
    pre_asrefx_cnt[ch] = asrefx_cnt;
    pre_srpdx_cnt[ch] = srpdx_cnt;
    pre_abr_cnt[ch] = mptc_->getPTC(ch)->rank_refresh_cnt;
    pre_pbr_cnt[ch] = mptc_->getPTC(ch)->perbank_refresh_cnt;
    pre_cmd_in2dfi_cnt[ch] = mptc_->getPTC(ch)->cmd_in2dfi_cnt;
    pre_cmd_in2dfi_lat[ch] = mptc_->getPTC(ch)->cmd_in2dfi_lat;
//    pre_perf_bank_rcnt[ch] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bank_rcnt; 
//    pre_perf_bank_wcnt[ch] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bank_wcnt; 

    for (uint8_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        PreBankRowActCnt[i] = mptc_->getPTC(ch)->BankRowActCnt[i];
    }
    for (uint8_t i = 0; i < 7; i ++) {
        pre_sch_level_cnt[ch][i] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->sch_level_cnt[i];
    }
    for (uint8_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        pre_perf_bank_rcnt[ch][i] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bank_rcnt[i]; 
        pre_perf_bank_wcnt[ch][i] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bank_wcnt[i]; 
        pre_perf2ptc_bank_rcnt[ch][i] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_rcnt[i];
        pre_perf2ptc_bank_wcnt[ch][i] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_wcnt[i];
//        pre_perf2ptc_bank_cnt[i] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bank_cnt[i];
        pre_ptc_r_bank_cnt[ch][i] = mptc_->getPTC(ch)->ptc_r_bank_cnt[i];
        pre_ptc_w_bank_cnt[ch][i] = mptc_->getPTC(ch)->ptc_w_bank_cnt[i];
    }

    for (uint8_t i = 0; i < NUM_RANKS; i ++) {
        for (uint8_t j = 0; j < NUM_GROUPS; j ++) {
            pre_perf_bg_rcnt[ch][i][j] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bg_rcnt[i][j];
            pre_perf_bg_wcnt[ch][i][j] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf_bg_wcnt[i][j];
            pre_perf2ptc_bg_rcnt[ch][i][j] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bg_rcnt[i][j];
            pre_perf2ptc_bg_wcnt[ch][i][j] = mpfq_->getPFQ(ch / (NUM_CHANS / NUM_PFQS))->perf2ptc_bg_wcnt[i][j];
            pre_ptc_r_bg_cnt[ch][i][j] = mptc_->getPTC(ch)->ptc_r_bg_cnt[i][j];
            pre_ptc_w_bg_cnt[ch][i][j] = mptc_->getPTC(ch)->ptc_w_bg_cnt[i][j];
        
        }
    }


    //clear statistics
    STATE_PRINTN(ch, "-------------------- DMC USAGE Statistics (Percentage/Cycle) --------------------------\n");
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, setw(15)<<"Bank slot Max Usage"<<" : "<<setw(10)<<mptc_->getPTC(ch)->bs_max<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Bank slot Average Usage"<<" : "<<setw(10)<<((float)mptc_->getPTC(ch)->bs_total/mptc_->getPTC(ch)->bs_cnt)<<" | ");
    STATE_PRINTN(ch, setw(15)<<"Bank slot Backpress Level"<<" : "<<setw(10)<<BS_BP_LVL<<endl);
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN(ch, "--------");
    }
    STATE_PRINTN(ch, endl);
    STATE_PRINTN(ch, "========================================= END =========================================\n");
    STATE_PRINTN(ch, "\n");
}

/**
 * This function creates up to 3 output files:
 *     - The .log file if LOG_OUTPUT is set
 *     - the .vis file where csv data for each epoch will go
 *     - the .tmp file if verification output is enabled
 * The results directory is setup to be in PWD/TRACEFILENAME.[SIM_DESC]/DRAM_PARTNAME/PARAMS.vis
 * The environment variable SIM_DESC is also appended to output files/directories
 *
 * TODO: verification info needs to be generated per channel so it has to be
 * moved back to MemorySystem
 **/
}
