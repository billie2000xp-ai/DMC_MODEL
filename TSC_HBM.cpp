void PTC::tsc_update(const BusPacket &bus_packet,bool hit) {
    //update each bank's state based on the command that was just popped out of the command queue
    //for readability's sake
    unsigned rank = bus_packet.rank;
    unsigned bank = bus_packet.bank;
    unsigned group = bus_packet.group;
    unsigned sid = bus_packet.sid;
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
        case READ_P_CMD :{
            for (auto &state : bankStates) {
                if (state.rank == rank) { // same rank
                    if (state.sid == sid) { // same rank, same sid
                        if (state.group == group) { // same rank, same sid, same bg
                            if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                if (bus_packet.type == READ_P_CMD) { // same rank, same sid, same bg, same bank, read ap
                                    //fix :in order to prenvent rot-hit command to send a read or write request
                                    state.state->currentBankState = Precharging;
                                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2 || IS_HBM2E || IS_HBM3) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO * 2))
                                                + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                        state.state->nextActivate2 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    } else if (IS_LP4 || IS_LP5 || IS_GD2) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle),
                                                state.state->nextActivate1);
                                    } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                                        state.state->nextActivate2 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    }
                                    state.state->nextPerBankRefresh = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                            + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                    state.state->nextAllBankRefresh = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                            + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                    state.state->stateChangeEn = true;
                                    state.state->stateChangeCountdown = CalcTiming(true, bus_packet.bl, PCFG_TRTP) +
                                        CalcCmdCycle(rw_cycle, 1);
                                    if (IS_GD1 || IS_GD2 || IS_G3D) {
                                        for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                                            uint32_t bank_tmp = rank * NUM_BANKS + ba;
                                            unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                                            unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                                            bankStates[bank_tmp].state->nextReadAp = max(CalcCmdCycle(cmd_cycle, cmd_cycle)
                                                    + now() + tppd, bankStates[bank_tmp].state->nextReadAp);
                                        }
                                    }
                                } else { // same rank, same sid, same bg, same bank, read
                                    state.state->nextPrecharge = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                            + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                }
                                state.last_activerow = bus_packet.row;
                                if (IS_DDR5) {
                                    state.state->nextRead = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                            } else { // same rank, same sid, same bg, diff bank
                                if (IS_DDR5) {
                                    state.state->nextRead = max(now() + tCCD_M + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + tCCD_M + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                            }
                            if (!IS_DDR5) {
                                state.state->nextRead = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            }
                            state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                            state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                            state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                            state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                        } else { // same rank, same sid, diff bg
                            state.state->nextRead = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            if (IS_G3D) {
                                state.state->nextWrite = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextWriteMaskAp);
                            } else {
                                state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            }
                        }
                        funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDPD) +
                                CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                    } else { // same rank, diff sid
                        state.state->nextRead = max(now() + CalcTccd(false, bus_packet.bl, tCCD_R) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_R) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, RL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case WRITE_CMD :
        case WRITE_P_CMD :{
            for (auto &state : bankStates) {
                if (state.rank == rank) { // same rank
                    if (state.sid == sid) { // same rank, same sid
                        if (state.group == group) { // same rank, same sid, same bg
                            if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                if (bus_packet.type == WRITE_P_CMD) { // same rank, same sid, same bg, same bank, write ap
                                    state.state->currentBankState = Precharging;
                                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2 || IS_HBM2E || IS_HBM3) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO * 2)) +
                                                CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                        state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    } else if (IS_LP4 || IS_LP5 || IS_GD2) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle),
                                                state.state->nextActivate1);
                                    } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                                        state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    }
                                    state.state->stateChangeEn = true;
                                    state.state->stateChangeCountdown = CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                        CalcCmdCycle(rw_cycle, 1);
                                    state.state->nextPerBankRefresh = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                            + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                    state.state->nextAllBankRefresh = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                            + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                    if (IS_GD1 || IS_GD2 || IS_G3D) {
                                        for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                                            uint32_t bank_tmp = rank * NUM_BANKS + ba;
                                            unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                                            unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                                            bankStates[bank_tmp].state->nextWriteAp = max(CalcCmdCycle(cmd_cycle, cmd_cycle)
                                                    + now() + tppd, bankStates[bank_tmp].state->nextWriteAp);
                                        }
                                    }
                                }
                                state.state->nextPrecharge = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                        CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                if (IS_G3D) {
                                    state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_SB) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_SB) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                } else {
                                    state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                                state.state->nextWriteMask = max(now() + CalcWrite2Mwrite(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                if (IS_DDR5) {
                                    unsigned tccd_l_wr = (bus_packet.bl == BL16) ? tCCD_L_WR : (tCCD_L_WR+unsigned(8/WCK2DFI_RATIO));
                                    unsigned tccd_l_wr2 = (bus_packet.bl == BL16) ? tCCD_L_WR2 : (tCCD_L_WR2+unsigned(8/WCK2DFI_RATIO));
                                    state.state->nextWrite = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWrite);
                                    state.state->nextWriteAp = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteAp);
                                    state.state->nextWriteRmw = max(now() + tccd_l_wr + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + tccd_l_wr + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteApRmw);
                                }
                            } else { // same rank, same sid, same bg, diff bank
                                if (IS_GD1) {
                                    state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                } else if (IS_DDR5) {
                                    unsigned tccd_l_wr2 = (bus_packet.bl==BL16)?tCCD_L_WR2:(tCCD_L_WR2+unsigned(8/WCK2DFI_RATIO));
                                    state.state->nextWrite = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWrite);
                                    state.state->nextWriteAp = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteAp);
                                    state.state->nextWriteRmw = max(now() + tCCD_M_WR + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + tCCD_M_WR + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteApRmw);
                                } else {
                                    state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                                state.state->nextWriteMask = max(now() + CalcWrite2Mwrite(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            }
                            state.last_activerow = bus_packet.row;
                            if (!IS_DDR5) {
                                state.state->nextWrite = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                            }
                        } else { // same rank, same sid, diff bg
                            state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextWriteRmw = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                            state.state->nextWriteApRmw = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                            state.state->nextWriteMask = max(now() + CalcWrite2Mwrite(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                            state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            if (IS_G3D) {
                                state.state->nextRead = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextRead);
                                state.state->nextReadAp = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextReadAp);
                            } else {
                                state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            }
                        }
                        if (bus_packet.type == WRITE_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    } else { // same rank, diff sid
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);                                
                        state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);                                
                        state.state->nextWriteMask = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, WL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case WRITE_MASK_CMD :// mask write is always BL16
        case WRITE_MASK_P_CMD :{
            for (auto &state : bankStates) {
                if (state.rank == rank) { // same rank
                    if (state.sid == sid) { // same rank, same sid
                        if (state.group == group) { // same rank, same sid, same bg
                            if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                if (bus_packet.type == WRITE_MASK_P_CMD) { // same rank, same sid, same bg, same bank, mask write ap
                                    state.state->currentBankState = Precharging;
                                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2 || IS_HBM2E || IS_HBM3) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO * 2)) +
                                                CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                        state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    } else if (IS_LP4 || IS_LP5 || IS_GD2) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle),
                                                state.state->nextActivate1);
                                    } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                                        state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    }
                                    state.state->stateChangeEn = true;
                                    state.state->stateChangeCountdown = CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                        CalcCmdCycle(rw_cycle, 1);
                                    if (IS_GD1 || IS_GD2 || IS_G3D) {
                                        for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                                            uint32_t bank_tmp = rank * NUM_BANKS + ba;
                                            unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                                            unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                                            bankStates[bank_tmp].state->nextWriteMaskAp = max(CalcCmdCycle(cmd_cycle, cmd_cycle)
                                                    + now() + tppd , bankStates[bank_tmp].state->nextWriteMaskAp);
                                        }
                                    }
                                }
                                state.state->nextPrecharge = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                        CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                state.state->nextWrite = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcMwrite2Mwrite(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                state.last_activerow = bus_packet.row;
                            } else { // same rank, same sid, same bg, diff bank
                                state.state->nextWrite = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcMwrite2Mwrite(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            }
                            state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        } else { // same rank, same sid, diff bg
                            state.state->nextWrite = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextWriteRmw = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                            state.state->nextWriteApRmw = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                            state.state->nextWriteMask = max(now() + CalcMwrite2Mwrite(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                            state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            if (IS_G3D) {
                                state.state->nextRead = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextRead);
                                state.state->nextReadAp = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextReadAp);
                            } else {
                                state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            }
                        }
                        if (bus_packet.type == WRITE_MASK_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    } else { // same rank, diff sid
                        state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, tCCDMW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, WL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case ACTIVATE1_CMD:{
            if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                for (auto &state : bankStates) {
                    if (rank == state.rank) {
                        if (state.group == group && state.bank == bank) {
                            state.state->nextActivate1 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                        } else {
                            state.state->nextActivate1 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                        }
                    }
                }
            } else {
                bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO)) +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            }
            break;
        }
        case ACTIVATE2_CMD:{
            for (auto &state : bankStates) {
                if (rank == state.rank) { // same rank
                    if (sid == state.sid) { // same rank, same sid
                        if (state.group == group) { // same rank, same sid, same bg
                            if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                state.state->trc_met_time = now() + tRAS + trp_pb;
                                state.state->currentBankState = RowActive;
                                state.state->openRowAddress = bus_packet.row;
                                if (IS_GD2) {
                                    state.state->nextPrecharge = max(now() + tRAS + 4 +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                                } else {
                                    state.state->nextPrecharge = max(now() + tRAS +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                                }
                                if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                    state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO * 2))
                                            + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                } else {
                                    state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO)) +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                }
                                state.state->nextActivate2 = max(now() + tRAS + trp_pb +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                state.state->nextRead = max(now() + tRCD +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + tRCD +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextReadAp);
                                state.state->nextWrite = max(now() + tRCD_WR +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + tRCD_WR +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + tRCD_WR +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + tRCD_WR +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + tRCD +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + tRCD +
                                        CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteMaskAp);

                                // use tRAS + tRPpb instead of tRC
                                state.state->nextPerBankRefresh = max(now() + tRAS + tRPpb +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            } else { // same rank, same sid, same bg, diff bank
                                if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                    state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO * 2)) +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                } else {
                                    state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO)) +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                }
                                state.state->nextActivate2 = max(now() + tRRD_L +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                state.state->nextPerBankRefresh = max(now() + tRRD_L +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                state.state->nextAllBankRefresh = max(now() + tRRD_L +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                            }
                        } else { // same rank, same sid, diff bg
                            if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO * 2)) +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            } else {
                                state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO)) +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            }
                            state.state->nextActivate2 = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            state.state->nextPerBankRefresh = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            state.state->nextAllBankRefresh = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                        }
                    } else { // same rank, diff sid
                        if (SID_LOOSE) {
                            state.state->nextActivate1 = max(now() + tRRD_Sdlr - unsigned(ceil(OFREQ_RATIO)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRRD_Sdlr +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            state.state->nextPerBankRefresh = max(now() + tRRD_Sdlr +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                        } else {
                            state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            state.state->nextPerBankRefresh = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);  
                        }
                        state.state->nextAllBankRefresh = max(now() + tRRD_L +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
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
                bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bank_tmp].state->nextPrecharge);
            }
            if (IS_DDR5) {
                for (size_t i = 0; i < pbr_bg_num; i ++) {
                    uint32_t bankIndex = i * pbr_bank_num + bus_packet.bankIndex;
                    bankStates[bankIndex].state->currentBankState = Precharging;
                    bankStates[bankIndex].state->stateChangeEn = true;
                    bankStates[bankIndex].state->stateChangeCountdown = tRPab + CalcCmdCycle(cmd_cycle, 1);
                    bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    bankStates[bankIndex].state->nextActivate2 = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                }
            }
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            else // rtl not use tCMDPD
                funcState[rank].nextPde = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case PRECHARGE_PB_CMD:{
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bank_tmp].state->nextPrecharge);
            }
            bankStates[bus_packet.bankIndex].state->currentBankState = Precharging;
            if (bus_packet.cmd_source == 1 || bus_packet.cmd_source == 2) {
                trp_pb += 15; // For pagetimeout and func precharge command
            }
            bankStates[bus_packet.bankIndex].state->stateChangeEn = true;
            bankStates[bus_packet.bankIndex].state->stateChangeCountdown = trp_pb + CalcCmdCycle(cmd_cycle, 1);
            bankStates[bus_packet.bankIndex].state->nextPerBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextPerBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextAllBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextAllBankRefresh);
            if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb -
                        unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bus_packet.bankIndex].state->nextActivate1);
            } else {
                bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb -
                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bus_packet.bankIndex].state->nextActivate1);
            }
            bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
            else // rtl not use tCMDPD
                funcState[rank].nextPde = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
            break;
        }
        case PRECHARGE_AB_CMD:{
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                uint32_t bankIndex = rank * NUM_BANKS + i;
                bankStates[bankIndex].state->currentBankState = Precharging;
                bankStates[bankIndex].state->stateChangeEn = true;
                bankStates[bankIndex].state->stateChangeCountdown = tRPab + CalcCmdCycle(cmd_cycle, 1);
                bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tRPab +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tRPab +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                    bankStates[bankIndex].state->nextActivate1 = max(now() + tRPab - unsigned(ceil(OFREQ_RATIO * 2)) +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                } else {
                    bankStates[bankIndex].state->nextActivate1 = max(now() + tRPab - unsigned(ceil(OFREQ_RATIO)) +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                }
                bankStates[bankIndex].state->nextActivate2 = max(now() + tRPab +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
            }
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            else // rtl not use tCMDPD
                funcState[rank].nextPde = max(now() + tRPab + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case REFRESH_CMD:{
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                uint32_t bankIndex = rank * NUM_BANKS + i;
                if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                    bankStates[bankIndex].state->nextActivate1 = max(now() + trfcab - unsigned(ceil(OFREQ_RATIO * 2)) +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                } else {
                    bankStates[bankIndex].state->nextActivate1 = max(now() + trfcab - unsigned(ceil(OFREQ_RATIO)) +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                }
                bankStates[bankIndex].state->nextActivate2 = max(now() + trfcab +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcab +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcab +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                bankStates[bankIndex].state->currentBankState = Refreshing;
                bankStates[bankIndex].state->stateChangeEn = true;
                bankStates[bankIndex].state->stateChangeCountdown = trfcab + CalcCmdCycle(cmd_cycle, 1);
            }
            funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case PER_BANK_REFRESH_CMD:{
            for (size_t i = 0; i < pbr_bank_num; i ++) {
                for (size_t j = 0; j < pbr_bg_num; j++) {
                    uint32_t bankIndex = rank * NUM_BANKS + i + j * pbr_bank_num;
                    if (bankIndex == (bus_packet.bankIndex + j * pbr_bank_num)) {
                        bankStates[bankIndex].state->currentBankState = Refreshing;
                        bankStates[bankIndex].state->stateChangeEn = true;
                        bankStates[bankIndex].state->stateChangeCountdown = trfcpb + CalcCmdCycle(cmd_cycle, 1);
                        if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb -
                                    unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                        } else {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb -
                                    unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                        }
                        bankStates[bankIndex].state->nextActivate2 = max(now() + trfcpb +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcpb +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                        bankStates[bankIndex].state->trfcpb_met_time = now() + trfcpb;
                    } else {
                        bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR + PBR_STATE_CYCLE +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcpb + PBR_STATE_CYCLE +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                        if (IS_LP5 || IS_LP6 || IS_GD2) {
                            if (DMC_V580 || IS_LP6) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                        unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                            }
                        } else if (IS_LP4) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + tRRD_S -
                                    unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_LP4 || IS_HBM2E) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                    unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        }else if(IS_HBM3){
                            unsigned banks_per_sid = (NUM_BANKS / NUM_SIDS);
                            unsigned tmp_sid = bankIndex/banks_per_sid;
                            unsigned tpbr2act = (tmp_sid == sid) ? tPBR2ACT : tPBR2ACT_S;
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tpbr2act +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_DDR5) {
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_GD1 || IS_G3D) {
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        }
                    }
                }
            }
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            else // rtl not use tCMDPD
                funcState[rank].nextPde = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case ACTIVATE1_DST_CMD:{
            bankStates[bus_packet.bankIndex].state->currentBankState = Refreshing;
            for (auto &state : bankStates) {
                if (rank != state.rank) continue;
                state.state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
            }
            break;
        }
        case ACTIVATE2_DST_CMD:{
            for (auto &state : bankStates) {
                if (rank == state.rank) {
                    if (state.group == group) {
                        if (state.bank == bank) {
                            state.state->nextPrecharge = max(now() + tRAS +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                            state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO * 2))
                                    + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRAS + trp_pb +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                        } else {
                            state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO * 2)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            state.state->nextPerBankRefresh = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            state.state->nextAllBankRefresh = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                        }
                    } else {
                        state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO * 2)) +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                        state.state->nextActivate2 = max(now() + tRRD_S +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                        state.state->nextPerBankRefresh = max(now() + tRRD_S +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                        state.state->nextAllBankRefresh = max(now() + tRRD_S +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                    }
                }
                if (DMC_V596)
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            }
            break;
        }
        case PRECHARGE_PB_DST_CMD:{
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bank_tmp].state->nextPrecharge);
            }
            bankStates[bus_packet.bankIndex].state->currentBankState = Precharging;
            bankStates[bus_packet.bankIndex].state->stateChangeEn = true;
            bankStates[bus_packet.bankIndex].state->stateChangeCountdown = trp_pb + CalcCmdCycle(cmd_cycle, 1);
            bankStates[bus_packet.bankIndex].state->nextPerBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextPerBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextAllBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextAllBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb -
                    unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                    bankStates[bus_packet.bankIndex].state->nextActivate1);
            bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
            break;
        }
        default : break;
    }
}