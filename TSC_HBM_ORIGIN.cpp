void PTC::tsc_update(const BusPacket &bus_packet,bool hit) {
    //update each bank's state based on the command that was just popped out of the command queue
    //for readability's sake
    unsigned rank         = bus_packet.rank;
    unsigned bank         = bus_packet.bank;
    unsigned group        = bus_packet.group;
    unsigned sub_channel  = (bus_packet.bankIndex % NUM_BANKS) / sc_bank_num;
    unsigned sid          = bus_packet.sid;
    unsigned bank_start   = sub_channel * NUM_BANKS / sc_num;
    unsigned rw_intlv_cnt = 0;
    if (bus_packet.type >= WRITE_CMD && bus_packet.type <= READ_P_CMD) {
        bankStates[bus_packet.bankIndex].state->rwIntlvCountdown = BL_n_min[bus_packet.bl];
    }
    for (auto &state : bankStates) {
        if (state.state->rwIntlvCountdown > 0) rw_intlv_cnt ++;
    }
    unsigned trp_pb = bus_packet.fg_ref ? tRPfg : tRPpb;
    switch (bus_packet.type) {
        case READ_CMD :
        case READ_P_CMD :{     //todo: revise for e-mode
            for (auto &state : bankStates) {
                unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                if (state.rank == rank) { // same rank
                    if (state_channel == sub_channel) { // same rank, same subchannel
                        if (state.sid == sid) { // same rank, same sid
                            if (state.group == group) { // same rank, same sid, same bg
                                if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                    if (bus_packet.type == READ_P_CMD) { // same rank, same sid, same bg, same bank, read ap
                                        //fix :in order to prenvent rot-hit command to send a read or write request
                                        state.state->currentBankState = Precharging;
                                        if (IS_HBM2E || IS_HBM3) {
                                            state.state->nextActivate1 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP) + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                        }
                                        state.state->nextPerBankRefresh = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP) + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                        state.state->nextAllBankRefresh = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP) + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                        state.state->stateChangeEn = true;
                                        state.state->stateChangeCountdown = CalcTiming(true, bus_packet.bl, PCFG_TRTP) + CalcCmdCycle(rw_cycle, 1);
                                        } else { // same rank, same sid, same bg, same bank, read
                                        state.state->nextPrecharge = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP) + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                    }
                                    state.last_activerow = bus_packet.row;
                                }
                                if (!IS_DDR5) {
                                    state.state->nextRead   = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                                state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            } else { // same rank, same sid, diff bg
                                state.state->nextRead        = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S)      + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp      = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S)      + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            }
                            if (bus_packet.type == READ_P_CMD) {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDAPPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            } else {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            }
                        } else { // same rank, diff sid
                            state.state->nextRead      = max(now() + CalcTccd(false, bus_packet.bl, tCCD_R)      + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextWrite     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        }
                    } else {  // same rank, diff subchannel for lp6
                        if (state.group == group) {  // same group
                            state.state->nextWrite   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextRead    = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp  = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        } else {   // diff group
                            state.state->nextWrite   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextRead    = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp  = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        }
                        if (bus_packet.type == READ_P_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDAPPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead        = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead        = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, RL, 0);
            RankState[rank].wck_on       = true;
            send_wckfs[rank]             = false;
            break;
        }
        case WRITE_CMD :
        case WRITE_P_CMD :{      //todo: revise for e-mode
            for (auto &state : bankStates) {
                unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                if (state.rank == rank) { // same rank
                    if (state_channel == sub_channel) { // same rank, same subchannel
                        if (state.sid == sid) { // same rank, same sid
                            if (state.group == group) { // same rank, same sid, same bg
                                if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                    if (bus_packet.type == WRITE_P_CMD) { // same rank, same sid, same bg, same bank, write ap
                                        state.state->currentBankState = Precharging;
                                        if (IS_HBM2E || IS_HBM3) {
                                            state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)+ trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                        }
                                        state.state->stateChangeEn        = true;
                                        state.state->stateChangeCountdown = CalcTiming(false, bus_packet.bl, PCFG_TWR) + CalcCmdCycle(rw_cycle, 1);
                                        state.state->nextPerBankRefresh   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)+ trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                        state.state->nextAllBankRefresh   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)+ trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                    }
                                    state.state->nextPrecharge = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR) + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                    state.state->nextRead   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    state.state->nextWriteMask   = max(now() + CalcWrite2Mwrite(true, true, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(true, true, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                } else { // same rank, same sid, same bg, diff bank
                                    state.state->nextRead   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    state.state->nextWriteMask   = max(now() + CalcWrite2Mwrite(true, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(true, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                }
                                state.last_activerow = bus_packet.row;       //todo: revise for e-mode
                                if (!IS_DDR5) {
                                    state.state->nextWrite      = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                    state.state->nextWriteAp    = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                    state.state->nextWriteRmw   = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                }
                            } else { // same rank, same sid, diff bg
                                state.state->nextWrite       = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp     = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw    = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw  = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask   = max(now() + CalcWrite2Mwrite(false, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(false, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                state.state->nextRead   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            }
                            if (bus_packet.type == WRITE_CMD) {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            } else {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            }
                        } else { // same rank, diff sid
                            state.state->nextRead      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextWrite     = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteMask = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        }
                    } else {  // same rank, diff subchannel
                        if (state.group == group) {  // same group
                            state.state->nextRead    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            state.state->nextWrite   = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        } else {  // diff group
                            state.state->nextRead    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            state.state->nextWrite   = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        }
                        if (bus_packet.type == WRITE_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead        = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead        = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, WL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case WRITE_MASK_CMD :// mask write is always BL16, mask write not used for lpddr6   
        case WRITE_MASK_P_CMD :{
            for (auto &state : bankStates) {
                if (state.rank == rank) { // same rank
                    if (state.sid == sid) { // same rank, same sid
                        if (state.group == group) { // same rank, same sid, same bg
                            if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                if (bus_packet.type == WRITE_MASK_P_CMD) { // same rank, same sid, same bg, same bank, mask write ap
                                    state.state->currentBankState = Precharging;
                                    if (IS_HBM2E || IS_HBM3) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR) + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                    } 
                                    state.state->stateChangeEn = true;
                                    state.state->stateChangeCountdown = CalcTiming(false, bus_packet.bl, PCFG_TWR) + CalcCmdCycle(rw_cycle, 1);
                                }
                                state.state->nextPrecharge   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)   + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                state.state->nextWrite       = max(now() + CalcMwrite2Write(true, true, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp     = max(now() + CalcMwrite2Write(true, true, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw    = max(now() + CalcMwrite2Write(true, true, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw  = max(now() + CalcMwrite2Write(true, true, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask   = max(now() + CalcMwrite2Mwrite(true, true, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(true, true, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                state.last_activerow         = bus_packet.row;
                            } else { // same rank, same sid, same bg, diff bank
                                state.state->nextWrite       = max(now() + CalcMwrite2Write(true, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp     = max(now() + CalcMwrite2Write(true, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw    = max(now() + CalcMwrite2Write(true, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw  = max(now() + CalcMwrite2Write(true, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask   = max(now() + CalcMwrite2Mwrite(true, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(true, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            }
                            state.state->nextRead   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        } else { // same rank, same sid, diff bg
                            state.state->nextWrite       = max(now() + CalcMwrite2Write(false, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp     = max(now() + CalcMwrite2Write(false, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextWriteRmw    = max(now() + CalcMwrite2Write(false, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                            state.state->nextWriteApRmw  = max(now() + CalcMwrite2Write(false, false, bus_packet.bl)  + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                            state.state->nextWriteMask   = max(now() + CalcMwrite2Mwrite(false, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                            state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(false, false, bus_packet.bl) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            state.state->nextRead   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        }
                        if (bus_packet.type == WRITE_MASK_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD)   + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) + CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    } else { // same rank, diff sid
                        state.state->nextWrite     = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, tCCDMW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextRead      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead        = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead        = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp      = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite       = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp     = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw    = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw  = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask   = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, WL, 0);
            RankState[rank].wck_on       = true;
            send_wckfs[rank]             = false;
            break;
        }
        case ACTIVATE1_CMD:{         //todo: revise for e-mode
            if ((DMC_V580 && IS_LP5) || IS_LP6) {
                for (auto &state : bankStates) {
                    unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                    if (rank == state.rank) {   // same rank
                        if (sub_channel == state_channel) {   // same rank, same subchannel
                            if (state.group == group && state.bank == bank) {  // same rank, same bg, same bank
                                state.state->nextActivate1 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +      
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            } else {
                                state.state->nextActivate1 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                state.state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            }
                        } 
                    }
                }
            } else {
                bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            }
            break;
        }
        case ACTIVATE2_CMD:{         //todo: revisr for e-mode
            for (auto &state : bankStates) {
                unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                if (rank == state.rank) { // same rank
                    if (sub_channel == state_channel) {   //same rank. same subchannel
                        if (sid == state.sid) { // same rank, same sid
                            if (state.group == group) { // same rank, same sid, same bg
                                if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                    state.state->trc_met_time = now() + tRAS + trp_pb;
                                    state.state->currentBankState = RowActive;
                                    state.state->openRowAddress   = bus_packet.row;
                                    state.state->nextPrecharge    = max(now() + tRAS + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                                    if ((DMC_V580 && IS_LP5) || IS_LP6) {
                                        state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    } else {
                                        state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    }
                                    if (!DERATING_EN) {
                                    state.state->nextActivate2 = max(now() + tRAS + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                    } else {
                                    state.state->nextActivate2 = max(now() + tRAS + trp_pb + unsigned(ceil(3.75 / tDFI)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                    }
                                    state.state->nextRead        = max(now() + tRCD + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp      = max(now() + tRCD + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextReadAp);
                                    state.state->nextWrite       = max(now() + tRCD_WR + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWrite);
                                    state.state->nextWriteAp     = max(now() + tRCD_WR + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteAp);
                                    state.state->nextWriteRmw    = max(now() + tRCD_WR + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw  = max(now() + tRCD_WR + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteApRmw);
                                    state.state->nextWriteMask   = max(now() + tRCD + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + tRCD + CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                } else { // same rank, same sid, same bg, diff bank
                                    if ((DMC_V580 && IS_LP5) || IS_LP6) {
                                        state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    } else {
                                        state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    }
                                    state.state->nextActivate2 = max(now() + tRRD_L + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                    state.state->nextPerBankRefresh = max(now() + tRRD_L + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                    state.state->nextAllBankRefresh = max(now() + tRRD_L + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                }
                            } else { // same rank, same sid, diff bg
                                if ((DMC_V580 && IS_LP5) || IS_LP6) {
                                    state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                } else {
                                    state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                }
                                state.state->nextActivate2 = max(now() + tRRD_S + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                state.state->nextPerBankRefresh = max(now() + tRRD_S + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                state.state->nextAllBankRefresh = max(now() + tRRD_S + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                            }
                        } else { // same rank, diff sid
                            if (SID_LOOSE) {
                                state.state->nextActivate1 = max(now() + tRRD_Sdlr - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                state.state->nextActivate2 = max(now() + tRRD_Sdlr + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                state.state->nextPerBankRefresh = max(now() + tRRD_Sdlr + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            } else {
                                state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                state.state->nextActivate2 = max(now() + tRRD_L + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                state.state->nextPerBankRefresh = max(now() + tRRD_L + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            }
                            state.state->nextAllBankRefresh = max(now() + tRRD_L + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                        }
                    } 
                }
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            }
            break;
        }
        case PRECHARGE_SB_CMD:{
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bank_tmp].state->nextPrecharge);
            }
            if (IS_DDR5) {
                for (size_t i = 0; i < pbr_bg_num; i ++) {
                    uint32_t bankIndex = i * pbr_bank_num + bus_packet.bankIndex;
                    bankStates[bankIndex].state->currentBankState     = Precharging;
                    bankStates[bankIndex].state->stateChangeEn        = true;
                    bankStates[bankIndex].state->stateChangeCountdown = tRPab + CalcCmdCycle(cmd_cycle, 1);
                    bankStates[bankIndex].state->nextPerBankRefresh   = max(now() + tRPab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh   = max(now() + tRPab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    bankStates[bankIndex].state->nextActivate2        = max(now() + tRPab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                }
            }
            if (DMC_V596) funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            else // rtl not use tCMDPD
                funcState[rank].nextPde = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case PRECHARGE_PB_CMD:{        //todo: revise for e-mode
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                unsigned state_channel = (bank_tmp % NUM_BANKS) / sc_bank_num;
                if (sub_channel == state_channel) {    // same rank, same subchannel
                    bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bank_tmp].state->nextPrecharge);
                }
            }
            bankStates[bus_packet.bankIndex].state->currentBankState = Precharging;
            if (bus_packet.cmd_source == 1 || bus_packet.cmd_source == 2) {
//                trp_pb += 15; // For pagetimeout and func precharge command
                if (DMC_RATE <= 3200) {
                    trp_pb += 7;   // For pagetimeout and func precharge command, low frequency
                } else {
//                    trp_pb += 15;    // For pagetimeout and func precharge command 
                    trp_pb += 0;    // For pagetimeout and func precharge command 
                }
            }
            bankStates[bus_packet.bankIndex].state->stateChangeEn        = true;
            bankStates[bus_packet.bankIndex].state->stateChangeCountdown = trp_pb + CalcCmdCycle(cmd_cycle, 1);
            bankStates[bus_packet.bankIndex].state->nextPerBankRefresh   = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextPerBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextAllBankRefresh   = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextAllBankRefresh);
            if ((DMC_V580 && IS_LP5) || IS_LP6) {
                bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate1);
            } else {
                bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate1);
            }
            bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            if (DMC_V596) {
                if (IS_LP6) {    // tCMDPD + tnACU
                    funcState[rank].nextPde = max(now() + tCMDPD + tnACU + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
                } else {
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
                }
            } else { // rtl not use tCMDPD, trp_pb of lp6 include nACU
                funcState[rank].nextPde = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
            }
            break;
        }
        case PRECHARGE_AB_CMD:{       //todo: revise for e-mode
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                uint32_t bankIndex = rank * NUM_BANKS + i;
                unsigned state_channel = (bankIndex % NUM_BANKS) / sc_bank_num;
                if (sub_channel == state_channel) {       //same rank, same subchannel
                    bankStates[bankIndex].state->currentBankState     = Precharging;
                    bankStates[bankIndex].state->stateChangeEn        = true;
                    bankStates[bankIndex].state->stateChangeCountdown = tRPab + CalcCmdCycle(cmd_cycle, 1);
                    bankStates[bankIndex].state->nextPerBankRefresh   = max(now() + tRPab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh   = max(now() + tRPab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    if ((DMC_V580 && IS_LP5) || IS_LP6) {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + tRPab - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    } else {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + tRPab - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    }
                    bankStates[bankIndex].state->nextActivate2 = max(now() + tRPab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                }
            }
            if (DMC_V596) {
                if (IS_LP6) {     // tCMDPD + tnACU
                    funcState[rank].nextPde = max(now() + tCMDPD + tnACU + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                } else {
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                }
            } else { // rtl not use tCMDPD, tRPab of lp6 include nACU
                funcState[rank].nextPde = max(now() + tRPab + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            }
            break;
        }
        case REFRESH_CMD:{       //todo: revise for e-mode
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                uint32_t bankIndex     = rank * NUM_BANKS + i;
                unsigned state_channel = (bankIndex % NUM_BANKS) / sc_bank_num;
                if (sub_channel == state_channel) {        // same rank, same channel
                    if ((DMC_V580 && IS_LP5) || IS_LP6) {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + trfcab - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    } else {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + trfcab - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    }
                    bankStates[bankIndex].state->nextActivate2        = max(now() + trfcab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                    bankStates[bankIndex].state->nextPerBankRefresh   = max(now() + trfcab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh   = max(now() + trfcab + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    bankStates[bankIndex].state->currentBankState     = Refreshing;
                    bankStates[bankIndex].state->stateChangeEn        = true;
                    bankStates[bankIndex].state->stateChangeCountdown = trfcab + CalcCmdCycle(cmd_cycle, 1);
                }
            }
            funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case PER_BANK_REFRESH_CMD:{          //todo:revise for e-mode
            if (ENH_PBR_EN) {                          
                for (size_t i = 0; i < NUM_BANKS; i ++) {
                    uint32_t bankIndex = rank * NUM_BANKS + i;
                    if (bankIndex == (bus_packet.fst_bankIndex) || bankIndex == (bus_packet.lst_bankIndex)) {    // same rank, same bank group, same bank
//                        DEBUG(now()<<" fresh timing, bank="<<bankIndex<<" fst_bankIndex="<<bus_packet.fst_bankIndex<<" lst_bankIndex="<<bus_packet.lst_bankIndex);
                        // bankIndex check
                        if (bus_packet.fst_bankIndex == bus_packet.lst_bankIndex) {
                            ERROR(setw(10)<<now()<<" Not allowed same bank in a bank pair, task="<<bus_packet.task<<" bank="<<bankIndex
                                    <<" fst_bankIndex"<<bus_packet.fst_bankIndex<<" lst_bankIndex"<<bus_packet.lst_bankIndex);
                            assert(0);
                        } else if (bus_packet.fst_bankIndex > bus_packet.lst_bankIndex) {
                            if (((bus_packet.fst_bankIndex-bus_packet.lst_bankIndex)%pbr_sb_num) != 0) {
                                ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, task="<<bus_packet.task<<" bank="<<bankIndex
                                        <<" fst_bankIndex"<<bus_packet.fst_bankIndex<<" lst_bankIndex"<<bus_packet.lst_bankIndex);
                                assert(0);
                            }
                        } else if (bus_packet.lst_bankIndex > bus_packet.fst_bankIndex) {
                            if (((bus_packet.lst_bankIndex-bus_packet.fst_bankIndex)%pbr_sb_num) != 0) {
                                ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, task="<<bus_packet.task<<" bank="<<bankIndex
                                        <<" fst_bankIndex"<<bus_packet.fst_bankIndex<<" lst_bankIndex"<<bus_packet.lst_bankIndex);
                                assert(0);
                            }
                        }

                        bankStates[bankIndex].state->currentBankState     = Refreshing;
                        bankStates[bankIndex].state->stateChangeEn        = true;
                        bankStates[bankIndex].state->stateChangeCountdown = trfcpb + CalcCmdCycle(cmd_cycle, 1);
                        if ((DMC_V580 && IS_LP5) || IS_LP6) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                        } else {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                        }
                        bankStates[bankIndex].state->nextActivate2      = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    } else {
                        if (IS_LP6 && (refresh_cnt_pb[bus_packet.rank][sub_channel] == (NUM_BANKS/sc_num -2))) {           // enhanced dbr: refresh_cnt_pb = 2x PER_BANK_REFRESH_CMD
                            bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR_L + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        } else {
                            bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        }        
                        bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tPBR2PBR + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                        if (IS_LP5 || IS_LP6) {
                            if (DMC_V580 || IS_LP6) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                            }
                        } else if (IS_LP4) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_LP4 || IS_HBM2E || IS_HBM3) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_DDR5) {
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } 
                    }
                }
                if (DMC_V596) funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                else // rtl not use tCMDPD
                    funcState[rank].nextPde = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                break;
            } else {         
                for (size_t i = 0; i < pbr_bank_num; i ++) {
                    for (size_t j = 0; j < pbr_bg_num; j++) {
                        uint32_t bankIndex = rank * NUM_BANKS + i + j * pbr_bank_num + bank_start;
                        if (bankIndex == (bus_packet.bankIndex + j * pbr_bank_num)) {     // same bank with one of the bank pair in same subchannel
                            bankStates[bankIndex].state->currentBankState     = Refreshing;
                            bankStates[bankIndex].state->stateChangeEn        = true;
                            bankStates[bankIndex].state->stateChangeCountdown = trfcpb + CalcCmdCycle(cmd_cycle, 1);
                            if ((DMC_V580 && IS_LP5) || IS_LP6) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                            } else {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                            }
                            bankStates[bankIndex].state->nextActivate2      = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                            bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                            bankStates[bankIndex].state->trfcpb_met_time = now() + trfcpb;
                        } else {     //diff bank from one of bank pair in same subchannel
                            if (IS_LP6 && (refresh_cnt_pb[bus_packet.rank][sub_channel] == (pbr_bank_num -1))) {
                                bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR_L + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                            } else {
                                bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                            }        
                            bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tPBR2PBR + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                            if (IS_LP5 || IS_LP6) {
                                if (DMC_V580 || IS_LP6) {
                                    bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT - unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                                    bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                                } else {
                                    bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                                }
                            } else if (IS_LP4) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else if (IS_LP4 || IS_HBM2E || IS_HBM3) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else if (IS_DDR5) {
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT + CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } 
                        }
                    }
                }
                if (DMC_V596) funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                else // rtl not use tCMDPD
                    funcState[rank].nextPde = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                break;
            }
        }
        default : break;
    }
}