
/* Generated by fzmparser version 2.7 --- DO NOT EDIT! */

/* Uses following resources: */
/* 5 events, 1+0 states, 0 inputs, 4 outputs, 5+0 match regs, 0+0 capture regs */

#include "sct_fsm.h"


void sct_fsm_init (void)
{
LPC_SCT1->CONFIG = (LPC_SCT1->CONFIG & ~0x00060001) | 0x00000001; /* UNIFIED */

/* MATCH/CAPTURE registers */

/* Unified counter - register side L is used and accessed as 32 bit value, reg H is not used */
LPC_SCT1->REGMODE_L = 0x00000000;         /* U: 5x MATCH, 0x CAPTURE, 11 unused */

LPC_SCT1->MATCH[0].U = init_val;             /* AHI */
LPC_SCT1->MATCHREL[0].U = init_val;
LPC_SCT1->MATCH[1].U = init_val;             /* ALI */
LPC_SCT1->MATCHREL[1].U = init_val;
LPC_SCT1->MATCH[2].U = init_val;             /* BHI */
LPC_SCT1->MATCHREL[2].U = init_val;
LPC_SCT1->MATCH[3].U = init_val;             /* BLI */
LPC_SCT1->MATCHREL[3].U = init_val;
LPC_SCT1->MATCH[4].U = freg;             /* MATCH_reset */
LPC_SCT1->MATCHREL[4].U = freg;

/* OUTPUT registers */
LPC_SCT1->OUT[0].SET = 0x00000001;        /* AHI */
LPC_SCT1->OUT[0].CLR = 0x00000002;
LPC_SCT1->OUT[1].SET = 0x00000004;        /* ALI */
LPC_SCT1->OUT[1].CLR = 0x00000001;
LPC_SCT1->OUT[5].SET = 0x00000008;        /* BHI */
LPC_SCT1->OUT[5].CLR = 0x00000001;
LPC_SCT1->OUT[2].SET = 0x00000001;        /* BLI */
LPC_SCT1->OUT[2].CLR = 0x00000010;
  /* Unused outputs must not be affected by any event */
LPC_SCT1->OUT[3].SET = 0;
LPC_SCT1->OUT[3].CLR = 0;
LPC_SCT1->OUT[4].SET = 0;
LPC_SCT1->OUT[4].CLR = 0;
LPC_SCT1->OUT[6].SET = 0;
LPC_SCT1->OUT[6].CLR = 0;
LPC_SCT1->OUT[7].SET = 0;
LPC_SCT1->OUT[7].CLR = 0;
LPC_SCT1->OUT[8].SET = 0;
LPC_SCT1->OUT[8].CLR = 0;
LPC_SCT1->OUT[9].SET = 0;
LPC_SCT1->OUT[9].CLR = 0;

/* Conflict resolution register */

/* EVENT registers */
LPC_SCT1->EVENT[0].CTRL = 0x00005004;     /* U: --> state U_ENTRY */
LPC_SCT1->EVENT[0].STATE = 0x00000001;
LPC_SCT1->EVENT[1].CTRL = 0x00005000;     /* U: --> state U_ENTRY */
LPC_SCT1->EVENT[1].STATE = 0x00000001;
LPC_SCT1->EVENT[2].CTRL = 0x00005001;     /* U: --> state U_ENTRY */
LPC_SCT1->EVENT[2].STATE = 0x00000001;
LPC_SCT1->EVENT[3].CTRL = 0x00005002;     /* U: --> state U_ENTRY */
LPC_SCT1->EVENT[3].STATE = 0x00000001;
LPC_SCT1->EVENT[4].CTRL = 0x00005003;     /* U: --> state U_ENTRY */
LPC_SCT1->EVENT[4].STATE = 0x00000001;
  /* Unused events must not have any effect */
LPC_SCT1->EVENT[5].STATE = 0;
LPC_SCT1->EVENT[6].STATE = 0;
LPC_SCT1->EVENT[7].STATE = 0;
LPC_SCT1->EVENT[8].STATE = 0;
LPC_SCT1->EVENT[9].STATE = 0;
LPC_SCT1->EVENT[10].STATE = 0;
LPC_SCT1->EVENT[11].STATE = 0;
LPC_SCT1->EVENT[12].STATE = 0;
LPC_SCT1->EVENT[13].STATE = 0;
LPC_SCT1->EVENT[14].STATE = 0;
LPC_SCT1->EVENT[15].STATE = 0;

/* STATE registers */
LPC_SCT1->STATE_L = 0;

/* state names assignment: */
  /* State U 0: U_ENTRY */

/* CORE registers */
LPC_SCT1->START_L = 0x00000000;
LPC_SCT1->STOP_L =  0x00000000;
LPC_SCT1->HALT_L =  0x00000000;
LPC_SCT1->LIMIT_L = 0x00000001;
LPC_SCT1->EVEN =    0x00000000;
LPC_SCT1->DMA0REQUEST = 0x00000000;
LPC_SCT1->DMA1REQUEST = 0x00000000;

}
