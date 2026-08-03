/* $$$!!Warning: Huawei key information asset. No spread without permission.$$$ */
/* CODEMARK:RKeR1B8WMAfemkt1tTDGp4eOEddgxKn4NOPmdw0w+6Q3n1pxgDEX+kGBiRV20e1NKuLwOh60qWwx
7DOUvTqsDpJdC/G6ahMCQuRlwWqc+IGKquH6vaaGAGe1zSmcLn5FMd2VBk0upEP5xKZPTVuBjKnw
SvZMzBtMrQ+w1lxbG5+EFWux51V2bvtZUTAAA+en/pM7ZB5Cy3u0JTs1VqxXwmxUmALsnN13rsZ6
elLFMxmcQvbA8P75QLAewnc3IkgyJMJQndhtFRm3uyY7ah2oFC9RtE/A70ZLkv78UbVpoXbRnfrm
RTADgWNHjhYSSnfK# */
/* $$$!!Warning: Deleting or modifying the preceding information is prohibited.$$$ */
/*
* Copyright @ Huawei Technologies Co., Ltd. 2019-2029. All rights reserved.
* Description: BankState.cpp
* Author     : l00434636
* Create     : 2020-10-27
*/

#include "BankState.h"

using namespace std;
using namespace DRAMSim;

//All banks start precharged
BankState:: BankState(ostream &DDRSim_log_): 
    DDRSim_log(DDRSim_log_),
    currentBankState(Idle),
    openRowAddress(0xFFFFFFFF),
    nextRead(0),
    nextReadAp(0),
    nextWrite(0),
    nextWriteAp(0),
    nextWriteRmw(0),
    nextWriteApRmw(0),
    nextWriteMask(0),
    nextWriteMaskAp(0),
    nextActivate1(0),
    nextActivate2(0),
    nextPrecharge(0),
    nextPerBankRefresh(0),
    nextAllBankRefresh(0),
    lastCommand(INVALID),
    stateChangeEn(false),
    stateChangeCountdown(0),
    rwIntlvCountdown(0),
    lastCmdType(DATA_READ),
    lastCmdPri(0),
    lastRow(0x80000000),
    lastCmdSource(0),
    lastPrechargeSource(0),
    lastMatgrp(0xFFFFFFFF),
    pageOpenTime(0),
    act_executing(false),
    fg_ref(false)
{}

void BankState::print() {
    PRINT(" == Bank State ");
    if (currentBankState == Idle) {
        PRINT("    State : Idle");
    }
    else if (currentBankState == RowActive) {
        PRINT("    State : Active");
    }
    else if (currentBankState == Refreshing) {
        PRINT("    State : Refreshing");
    }
    else if (currentBankState == PowerDown) {
        PRINT("    State : Power Down");
    }

    PRINT("    OpenRowAddress  : " << openRowAddress);
    PRINT("    nextRead        : " << nextRead);
    PRINT("   nextReadAp      : " << nextReadAp);
    PRINT("    nextWrite       : " << nextWrite);
    PRINT("    nextWriteAp     : " << nextWriteAp);
    PRINT("   nextWriteRmw    : " << nextWriteRmw);
    PRINT("    nextWriteApRmw  : " << nextWriteApRmw);
    PRINT("    nextActivate1   : " << nextActivate1);
    PRINT("    nextActivate2   : " << nextActivate2);
    PRINT("    nextPrecharge   : " << nextPrecharge);
    PRINT("    nextWriteMask   : " << nextWriteMask);
    PRINT("    nextWriteMaskAp : " << nextWriteMaskAp);
}
