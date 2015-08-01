/*
 * @brief Blinky example using timers and sysTick
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "chip.h"    /* LPC1549JBD48 Peripheral Registers */
#include "initializer.h"
#include "sct_fsm.h"

#include <stdio.h>

#define i2ctest 1
#define motordrive 1
#define temp_sens

#define IAP
#ifdef IAP

#define IAP_REINVOKE_ISP_CMD 57
#define UART_ISP 	1
#define USB_ISP		2
#define CAN_ISP		3
/* IAP command variables */
static unsigned int command[5], result[4];

void endter_ISP(uint8_t mode){
	__disable_irq();

	command[0] = IAP_REINVOKE_ISP_CMD;						/* Prepare to write/erase command code */
	command[1] = mode;

	iap_entry(command, result);
}
#endif

#define CAN_test

#define CAN 0
#define CDC 1
void message_Action(uint16_t,uint16_t,uint16_t,uint16_t);

#ifdef CAN_test

#endif

#define velContoroll

typedef struct
{
  __IO uint32_t CNTL;				/* 0x000 */
  __IO uint32_t STAT;
  __I  uint32_t EC;
  __IO uint32_t BT;
  __I  uint32_t INT;
  __IO uint32_t TEST;
  __IO uint32_t BRPE;
       uint32_t RESERVED0;
  __IO uint32_t IF1_CMDREQ;			/* 0x020 */
  __IO uint32_t IF1_CMDMSK;
  __IO uint32_t IF1_MSK1;
  __IO uint32_t IF1_MSK2;
  __IO uint32_t IF1_ARB1;
  __IO uint32_t IF1_ARB2;
  __IO uint32_t IF1_MCTRL;
  __IO uint32_t IF1_DA1;
  __IO uint32_t IF1_DA2;
  __IO uint32_t IF1_DB1;
  __IO uint32_t IF1_DB2;
       uint32_t RESERVED1[13];
  __IO uint32_t IF2_CMDREQ;			/* 0x080 */
  __IO uint32_t IF2_CMDMSK;
  __IO uint32_t IF2_MSK1;
  __IO uint32_t IF2_MSK2;
  __IO uint32_t IF2_ARB1;
  __IO uint32_t IF2_ARB2;
  __IO uint32_t IF2_MCTRL;
  __IO uint32_t IF2_DA1;
  __IO uint32_t IF2_DA2;
  __IO uint32_t IF2_DB1;
  __IO uint32_t IF2_DB2;
       uint32_t RESERVED2[21];
  __I  uint32_t TXREQ1;				/* 0x100 */
  __I  uint32_t TXREQ2;
       uint32_t RESERVED3[6];
  __I  uint32_t ND1;				/* 0x120 */
  __I  uint32_t ND2;
       uint32_t RESERVED4[6];
  __I  uint32_t IR1;				/* 0x140 */
  __I  uint32_t IR2;
       uint32_t RESERVED5[6];
  __I  uint32_t MSGV1;				/* 0x160 */
  __I  uint32_t MSGV2;
       uint32_t RESERVED6[6];
  __IO uint32_t CLKDIV;				/* 0x180 */
} LPC_CAN_TypeDef;
/*@}*/ /* end of group LPC15xx_CAN */


/* Data structure for a CAN message */
typedef struct
{
    uint32_t	id;
    uint32_t 	dlc;
    uint32_t	data[4];
} message_object;

#define LPC_CAN               ((LPC_CAN_TypeDef *)LPC_C_CAN0_BASE)

#define CAN_STATUS_INTERRUPT      0x8000
#define DLC_MAX				8
/* bit field of IF command mask register */
#define	DATAB	(1 << 0)   /* 1 is transfer data byte 4-7 to message object, 0 is not */
#define	DATAA	(1 << 1)   /* 1 is transfer data byte 0-3 to message object, 0 is not */
#define	TREQ	(1 << 2)   /* 1 is set the TxRqst bit, 0 is not */
#define	INTPND	(1 << 3)
#define	CAN_CTRL	(1 << 4)   /* 1 is transfer the CTRL bit to the message object, 0 is not */
#define	ARB		(1 << 5)   /* 1 is transfer the ARB bits to the message object, 0 is not */
#define	MASK	(1 << 6)   /* 1 is transfer the MASK bit to the message object, 0 is not */
#define	WR		(1 << 7)   /* 0 is READ, 1 is WRITE */
#define RD      0x0000

/* bit field of IF mask 2 register */
#define	MASK_MXTD	(1 << 15)     /* 1 extended identifier bit is used in the RX filter unit, 0 is not */
#define	MASK_MDIR	(1 << 14)     /* 1 direction bit is used in the RX filter unit, 0 is not */

/* bit field of IF identifier 2 register */
#define	ID_MVAL		(1 << 15)     /* Message valid bit, 1 is valid in the MO handler, 0 is ignored */
#define	ID_MTD		(1 << 14)     /* 1 extended identifier bit is used in the RX filter unit, 0 is not */
#define	ID_DIR		(1 << 13)     /* 1 direction bit is used in the RX filter unit, 0 is not */


/* bit field of IF message control register */
#define	NEWD		(1 << 15)     /* 1 indicates new data is in the message buffer.  */
#define	MLST		(1 << 14)     /* 1 indicates a message loss. */
#define	INTP		(1 << 13)     /* 1 indicates message object is an interrupt source */
#define UMSK    	(1 << 12)     /* 1 is to use the mask for the receive filter mask. */
#define	TXIE		(1 << 11)     /* 1 is TX interrupt enabled */
#define	RXIE		(1 << 10)     /* 1 is RX interrupt enabled */
#define	ROEN		(1 << 9)      /* 1 is remote frame enabled */
#define TXRQ    	(1 << 8)      /* 1 is TxRqst enabled */
#define	EOB			(1 << 7)      /* End of buffer, always write to 1 */
#define	DLC			0x000F        /* bit mask for DLC */

#define ID_STD_MASK		0x07FF
#define ID_EXT_MASK		0x1FFFFFFF
#define DLC_MASK		0x0F

/* CAN Status register */
#define STAT_LEC		(0x7 << 0)
#define STAT_TXOK		(1 << 3)
#define STAT_RXOK		(1 << 4)
#define STAT_EPASS		(1 << 5)
#define STAT_EWARN		(1 << 6)
#define STAT_BOFF		(1 << 7)

/* CAN CTRL register */
#define CTRL_INIT		(1 << 0)
#define CTRL_IE			(1 << 1)
#define CTRL_SIE		(1 << 2)
#define CTRL_EIE		(1 << 3)
#define CTRL_DAR		(1 << 5)
#define CTRL_CCE		(1 << 6)
#define CTRL_TEST		(1 << 7)

/* CAN TEST register */
#define TEST_BASIC		(1 << 2)
#define TEST_SILENT		(1 << 3)
#define TEST_LBACK		(1 << 4)

/* bit field of IF command request n register */
#define IFCREQ_BUSY               0x8000

/* statistics of all the interrupts */
volatile uint32_t BOffCnt = 0;
volatile uint32_t EWarnCnt = 0;
volatile uint32_t EPassCnt = 0;

uint32_t CANRxDone;
message_object tx;
message_object rx;

void CAN_Message_Receive_Action(message_object msg);

#ifdef __cplusplus
extern "C" {
#endif
void CAN_IRQHandler(void)
{
 uint32_t can_stat, can_int;
 //Board_LED_Toggle(2);
 LPC_CAN->IF2_CMDMSK = RD|MASK|ARB|TREQ|DATAA|DATAB;
 LPC_CAN->IF2_CMDREQ = IFCREQ_BUSY;    /* Start message transfer */
 while(LPC_CAN->IF2_CMDREQ & IFCREQ_BUSY );	/* Check new data bit */
 can_int = LPC_CAN->INT;
 can_stat = LPC_CAN->STAT;
 if(can_int & CAN_STATUS_INTERRUPT)
 {
  if(can_stat & STAT_EWARN)
  {
   EWarnCnt++;
  }
  if(can_stat & STAT_BOFF)
  {
   BOffCnt++;
   return;
  }
 }
//message received
  if(can_stat & STAT_RXOK)
  {
   LPC_CAN->STAT &= ~STAT_RXOK;
   Board_LED_Toggle(3);
   rx.id = (LPC_CAN->IF2_ARB2 &0x1FFF) >> 2;
   rx.dlc = LPC_CAN->IF2_MCTRL & 0x000F;	// Get Msg Obj Data length
   rx.data[0] = LPC_CAN->IF2_DA1;
   rx.data[1] = LPC_CAN->IF2_DA2;
   rx.data[2] = LPC_CAN->IF2_DB1;
   rx.data[3] = LPC_CAN->IF2_DB2;

   CAN_Message_Receive_Action(rx);

   if(rx.id == 0x710)					//show received message 0x710
   {
    //Board_LED_Toggle(3);
   }
   CANRxDone = TRUE;
  }
  if(can_stat & STAT_TXOK)
  {
   LPC_CAN->STAT &= ~STAT_TXOK;
   Board_LED_Toggle(1);
  }
}

#ifdef __cplusplus
}
#endif

#define BITRATE100K36MHZ 0x00002AD7UL //CAN_BTR     -> BRP: 24, Quanta: 15, Seg1: 11, Seg2: 3, SJW: 3, Sample 80%
void CAN_Init( uint32_t CANBitClk )
{
  LPC_SYSCON->SYSAHBCLKCTRL[1]|= (1<<7);
  if ( !(LPC_CAN->CNTL & CTRL_INIT) )
  {
	LPC_CAN->CNTL |= CTRL_INIT;		/* Default state */
  }
  /* AHB clock is 72Mhz, the CAN clock is 1/2 AHB clock = 36Mhz */
  LPC_CAN->CLKDIV = 0x01;			/* Divided by 1 */
  /* Start configuring bit timing */
  LPC_CAN->CNTL |= CTRL_CCE;
  LPC_CAN->BT = CANBitClk;
  LPC_CAN->BRPE = 0x0000;
  /* Stop configuring bit timing */
  LPC_CAN->CNTL &= ~CTRL_CCE;
  LPC_CAN->CNTL |= (CTRL_DAR);			//disable retransmission
  LPC_CAN->CNTL |= (CTRL_TEST);			//enable test
  LPC_CAN->TEST |= (TEST_BASIC);		//BASIC mode
#if LOOPBACK_MODE
  LPC_CAN->TEST |= (TEST_LBACK);
#endif
  /* Initialization finishes, normal operation now. */
  LPC_CAN->CNTL &= ~CTRL_INIT;
  while ( LPC_CAN->CNTL & CTRL_INIT );
  NVIC_EnableIRQ(CAN_IRQn);
  /* By default, auto TX is enabled, enable all related interrupts */
  LPC_CAN->CNTL |= (CTRL_IE|CTRL_SIE|CTRL_EIE);
}

void CAN_Board_Init(){
	// CAN signal muxing LQFP48
	 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	 Chip_SWM_MovablePortPinAssign(SWM_CAN_TD1_O , 0, 18);
	 Chip_SWM_MovablePortPinAssign(SWM_CAN_RD1_I,  0, 13);
	 CAN_Init(BITRATE100K36MHZ);			//init CAN with CORECLOCK/2

	//set tx message
	 tx.id  = 0x720;
	 tx.dlc = 4;
	 tx.data[0] = 0x1122;
}

//send message
void CAN_Send(uint32_t *msg_ptr )
{
 uint32_t msg_id, tx_id, Length;
 msg_id = *msg_ptr++;
 tx_id = msg_id;
 Length = *msg_ptr++;
/* MsgVal: 1, Mtd: 0, Dir: 1, ID = 0x200 */
 LPC_CAN->IF1_ARB2 = ID_MVAL | ID_DIR | (tx_id << 2);
 LPC_CAN->IF1_ARB1 = 0x0000;
/* Mxtd: 0, Mdir: 1, Mask is 0x7FF */
 LPC_CAN->IF1_MSK2 = MASK_MDIR | (ID_STD_MASK << 2);
 LPC_CAN->IF1_MSK1 = 0x0000;
 LPC_CAN->IF1_MCTRL = UMSK|TXRQ|EOB|(Length & DLC_MASK);
 LPC_CAN->IF1_DA1 = *msg_ptr++;
 LPC_CAN->IF1_DA2 = *msg_ptr++;
 LPC_CAN->IF1_DB1 = *msg_ptr++;
 LPC_CAN->IF1_DB2 = *msg_ptr;
 LPC_CAN->IF1_CMDMSK = WR|MASK|ARB|CAN_CTRL|TREQ|DATAA|DATAB;
 LPC_CAN->IF1_CMDREQ = IFCREQ_BUSY;
}
#ifdef velContoroll

#define MaxInregVal 30000
int32_t Pgain,Igain,Dgain;
int32_t rockPgain,rockIgain,rockDgain,rockEnable,rockRecv;
uint32_t setRPMval;
int32_t roadVal;
uint32_t OUTsideVal,INsideVal,rate;

void setMotorRPM(uint32_t setRPM, uint32_t motorENC, int32_t P_gain, int32_t I_gain, int32_t D_gain);
void set_real_duty(uint8_t ch,uint32_t duty);
void setMotorRealDuty(uint32_t realDutyVal);
void servoRock(int enable,uint32_t setangle ,int32_t P_gain, int32_t I_gain, int32_t D_gain, int recv);
#endif

#define current_sens

#ifdef current_sens
#define BOARD_ADC_CH 1
static bool sequence0Complete, sequence1Complete, threshold1Crossed;
long CurrentSensNeutral = 0;
void CurrentSensInit();
void CurrentSensNeutralCalibration();
long GetCurrentRawValue();
float GetCurrentRealValue(long);

extern "C"{
	void ADC1A_IRQHandler(void);
	void ADC1_THCMP_IRQHandler(void);
}
#endif

#ifdef temp_sens

static volatile bool tempSequenceComplete;
static volatile int tempSampleIdx;

/* Temperature sample ADC read buffer - only index 9 is used for the valid
   temperature sample. */
#define TEMPSAMPLES 10
static uint32_t temp[TEMPSAMPLES];
static float firstTemp;
void temp_sens_init();
static void tempStartCycle(void);
static bool tempCycleComplete(void);
static uint32_t tempGetSample(void);
float GetTempSensRealValue(uint32_t);
#endif

#define QEI

#ifdef QEI
#include "sys_config.h"

#define PPR  256
#define Edges  4
#define QEI_LOAD 3000000
//#define QEI_MAPPOS 250000
#define QEI_MAPPOS 321422
uint32_t angle;
int angleStream = 0;

uint32_t GetRPM();
static void prvSetupHardware(void);

//Interruption du QEI
extern "C" {
	void QEI_IRQHandler()
	{
		//Board_LED_Set(0,Board_LED_Test(0));
		Board_LED_Toggle(0);
		LPC_QEI->CLR=1;

	};
}
#endif

#define usb_com 1
#define usb_read

#if usb_com
#include "app_usbd_cfg.h"
#include "cdc_vcom.h"
#include <string.h>
#include <stdlib.h>

static USBD_HANDLE_T g_hUsb;
static uint8_t vcom_rxBuff[256];
const  USBD_API_T *g_pUsbApi;

#include <stdarg.h>
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass);
void uprintf(const char *format, ...);
void Board_USB_init();

#define printf(...) uprintf(__VA_ARGS__)
/**
 * @brief	Handle interrupt from USB0
 * @return	Nothing
 */
extern "C" {

	void USB_IRQHandler(void)
	{
		USBD_API->hw->ISR(g_hUsb);
	}
}
#endif


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ1 (1000)	/* ticks per second */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#if i2ctest
/* I2CS transfer record for master and slave operations */
static I2CM_XFER_T  i2cmXferRec;

/* I2C clock is set to 1.8MHz */
#define I2C_CLK_DIVIDER     (40)

/* 100KHz I2C bit-rate - going too fast may prevent the salev from responding
   in time */
#define I2C_BITRATE         (100000)
/* Standard I2C mode */
#define I2C_MODE    (0)

/* Emulated EEPROM slave addresses */
uint16_t EEPROM1SLVADD = 0x20;

#if defined(BOARD_NXP_LPCXPRESSO_1549)
/** Our slave address and I2C information */
#define LPC_I2C_PORT         LPC_I2C0
#define LPC_I2C_INTHAND      I2C0_IRQHandler
#define LPC_IRQNUM           I2C0_IRQn
#endif

/* Emulated EEPROM device2 - size, buffer, and current address */
#define EMUEEPROMSIZE 512
static int addrbytes;

/* work buffers for this example */
uint8_t txWorkBuff[EMUEEPROMSIZE], rxWorkBuff[EMUEEPROMSIZE];

bool LEDstate = false;
uint8_t recvaddr,recvdata,i2cFlag=0;
uint8_t trandata,start = 0;
uint8_t LED1;
uint8_t servo=150;
#endif

/*****************************************************************************
 * SCT setting for motor drive
 *****************************************************************************/
#if motordrive

#define SCT_PWM            LPC_SCT1 /* Use SCT0 for PWM */
#define SCT_PWM_PIN_OUT    1        /* COUT1 Generate square wave */
#define SCT_PWM_OUT        1        /* Index of OUT PWM */
#define SCT_PWM_RATE   50        /* PWM frequency 50 Hz */

#define LED_STEP_CNT      20        /* Change LED duty cycle every 20ms */
#define OUT_STEP_CNT    1000        /* Change duty cycle every 1 second */

#define onBrake	0
#define onFree 	1

#define Aside	0
#define Bside	1

#define AHI_pin	9	//配線間違い　kicad上のHIP4081のライブラリにミスがあるので要修正 2015-02-28
#define ALI_pin	10
#define BHI_pin	14
#define BLI_pin 11
#define DIS_pin	12
#define ALL		0

#define SCTPINS 5
static const uint8_t sctpin[SCTPINS] = {AHI_pin, ALI_pin, BHI_pin, BLI_pin, DIS_pin};

uint8_t mode,dir,side,changeFlag,dis=0,SCT_freg,last_SCT_freg;
uint16_t duty;

void enable_driver(bool dis){
	Chip_GPIO_SetPinState(LPC_GPIO, 0, DIS_pin, !dis);
}

/*	SCT PWM output [Hz]		*/
void set_sct_fregencey(uint32_t freq){
	uint32_t rate;

	rate = Chip_Clock_GetSystemClockRate() / freq;;
	sct_fsm_reload_MATCH_reset(rate);
}

uint32_t get_sct_rate(){
	return LPC_SCT1->MATCHREL[4].U;
}

/* ch 	: output_pin number (ex.AHI_pin ,ALL)
 * duty : 0 ~ 100 value 					*/
void set_duty(uint8_t ch,uint16_t duty){
	uint32_t OUTsideVal,INsideVal,rate;

	rate = get_sct_rate();
	OUTsideVal = (uint32_t)(rate * duty / 100);
	INsideVal = rate - OUTsideVal;

	switch(ch){
		case AHI_pin:
			sct_fsm_reload_AHI(OUTsideVal);
			break;
		case ALI_pin:
			sct_fsm_reload_ALI(INsideVal);
			break;
		case BHI_pin:
			sct_fsm_reload_BHI(INsideVal);
			break;
		case BLI_pin:
			sct_fsm_reload_BLI(OUTsideVal);
			break;
		default:
			sct_fsm_reload_AHI(OUTsideVal);
			sct_fsm_reload_ALI(INsideVal);
			sct_fsm_reload_BHI(INsideVal);
			sct_fsm_reload_BLI(OUTsideVal);
	}
}

void drive_motor(uint8_t mode,uint8_t dir,uint16_t duty){
	if(mode == onBrake){
		if(dir == Aside){
			set_duty(AHI_pin, duty);
			set_duty(ALI_pin, (100 - (uint16_t)(duty*0.95)));
			set_duty(BHI_pin, 0);
			set_duty(BLI_pin, 100);
		}else{
			set_duty(AHI_pin, 0);
			set_duty(ALI_pin, 100);
			set_duty(BHI_pin, duty);
			set_duty(BLI_pin, (100 - (uint16_t)(duty*0.95)));
		}
	}else{
		if(dir == Aside){
			set_duty(AHI_pin, 100);
			set_duty(ALI_pin, 0);
			set_duty(BHI_pin, 0);
			set_duty(BLI_pin, duty);
		}else{
			set_duty(AHI_pin, 0);
			set_duty(ALI_pin, duty);
			set_duty(BHI_pin, 100);
			set_duty(BLI_pin, 0);
		}
	}
}

void SCT_init(){
	int idx;

	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(SCT_PWM);

	/* Use SCT0 pin */
	for (idx = 0; idx <  SCTPINS; idx++) {
		/* Set the GPIO as output with initial state off (high) */
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, sctpin[idx]);
	}
	sct_fsm_init();

	NVIC_EnableIRQ(SCT1_IRQn);
	LPC_SCT1->CTRL_U &= ~(1 << 2);

	enable_driver(false);
}
#endif
/*****************************************************************************
 * Private functions
 ****************************************************************************/
#if i2ctest
/* Initializes pin muxing for I2C interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_I2C_PinMux(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_1549)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
#else
	/* Configure your own I2C pin muxing here if needed */
#error "No I2C Pin Muxing defined for this example"
#endif
}

/* Setup I2C */
static void setupI2CMaster(void)
{
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(LPC_I2C_PORT);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C_PORT, I2C_CLK_DIVIDER);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C_PORT, I2C_BITRATE);

	/* Enable I2C master interface */
	Chip_I2CM_Enable(LPC_I2C_PORT);
}

/* Setup I2C */
static void setupI2CSlave(void)
{
	/* Some common I2C init was performed in setupI2CMaster(), so it doesn't
	   need to be done again for the slave setup. */

	/* Emulated EEPROM 1 is on slave index 1 */
	Chip_I2CS_SetSlaveAddr(LPC_I2C_PORT, 1, EEPROM1SLVADD);
	/* Enable Slave Address 1 */
	Chip_I2CS_EnableSlaveAddr(LPC_I2C_PORT, 1);

	/* Clear interrupt status and enable slave interrupts */
	Chip_I2CS_ClearStatus(LPC_I2C_PORT, I2C_STAT_SLVDESEL);
	Chip_I2C_EnableInt(LPC_I2C_PORT, I2C_INTENSET_SLVPENDING | I2C_INTENSET_SLVDESEL);

	/* Enable I2C slave interface */
	Chip_I2CS_Enable(LPC_I2C_PORT);
}

/* Function to wait for I2CM transfer completion */
static void WaitForI2cXferComplete(I2CM_XFER_T *xferRecPtr)
{
	/* Test for still transferring data */
	while (xferRecPtr->status == I2CM_STATUS_BUSY) {
		/* Sleep until next interrupt */
		__WFI();
	}
}

/* Function to setup and execute I2C transfer request */
static void SetupXferRecAndExecute(uint8_t devAddr,
								   uint8_t *txBuffPtr,
								   uint16_t txSize,
								   uint8_t *rxBuffPtr,
								   uint16_t rxSize)
{
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = devAddr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = txSize;
	i2cmXferRec.rxSz = rxSize;
	i2cmXferRec.txBuff = txBuffPtr;
	i2cmXferRec.rxBuff = rxBuffPtr;

	/* Wait for master to go pending - needed in mixed master/slave mode on single I2C bus */
	while (Chip_I2CM_IsMasterPending(LPC_I2C_PORT) == false) {}

	Chip_I2CM_Xfer(LPC_I2C_PORT, &i2cmXferRec);
	/* Enable Master Interrupts */
	Chip_I2C_EnableInt(LPC_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
	/* Disable all Interrupts */
	Chip_I2C_DisableInt(LPC_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);

	if (i2cmXferRec.status != I2CM_STATUS_OK) {
		DEBUGOUT("\r\nI2C error: %d\r\n", i2cmXferRec.status);
	}
}

/* Handler for slave start callback */
static void processSlaveTransferStart(uint8_t addr)
{
	addrbytes = 0;
}

/* Handler for slave send callback */
static uint8_t processSlaveTransferSend(uint8_t *data)
{
	/* Send data from emulated EEPROM */
	if(recvdata == 0x01) *data = LED1;
	else if (recvdata == 0x02)*data = servo;
	/*QEI test*/
#if 0
	else if (recvdata == 0x10)*data = LPC_QEI->STAT;
	else if (recvdata == 0x11)*data = LPC_QEI->CONF;
	else if (recvdata == 0x12)*data = LPC_QEI->POS;		//encoder position (maxsetting is MAXPOS)
	else if (recvdata == 0x13)*data = LPC_QEI->MAXPOS;
	else if (recvdata == 0x14)*data = LPC_QEI->CMPOS0;
	else if (recvdata == 0x15)*data = LPC_QEI->CMPOS1;
	else if (recvdata == 0x16)*data = LPC_QEI->CMPOS2;
	else if (recvdata == 0x17)*data = LPC_QEI->INXCNT;
	else if (recvdata == 0x18)*data = LPC_QEI->INXCMP0;
	else if (recvdata == 0x19)*data = LPC_QEI->LOAD;
	else if (recvdata == 0x1A)*data = LPC_QEI->TIME;
	else if (recvdata == 0x1B)*data = LPC_QEI->VEL;
	else if (recvdata == 0x1C)*data = LPC_QEI->CAP;		//encoder speed (pos/load)
	else if (recvdata == 0x1D)*data = LPC_QEI->VELCOMP;
	else if (recvdata == 0x1E)*data = LPC_QEI->FILTERPHA;
	else if (recvdata == 0x1F)*data = LPC_QEI->FILTERPHB;
	else if (recvdata == 0x20)*data = LPC_QEI->FILTERINX;
	else if (recvdata == 0x21)*data = LPC_QEI->WINDOW;
	else if (recvdata == 0x22)*data = LPC_QEI->INXCMP1;
	else if (recvdata == 0x23)*data = LPC_QEI->INXCMP2;
	else if (recvdata == 0x24)*data = LPC_QEI->INTSTAT;
	else if (recvdata == 0x25)*data = LPC_QEI->IE;
#else
	else if (recvdata == 0x10)*data = LPC_QEI->STAT;
	else if (recvdata == 0x12)*data = angle & 0xff;
	else if (recvdata == 0x13)*data = angle>>8;
#endif
	else if (recvdata == 0x30)*data = GetRPM()/4;

	else *data = 0x4d;


	return 0;
}

/* Handler for slave receive callback */
static uint8_t processSlaveTransferRecv(uint8_t data)
{
	static uint16_t addr;
	//static uint32_t duty;

	if (addrbytes == 0) {
		/* Address MSB bytes is in */
		addr = data;
	}
	else if (addrbytes == 1) {
		/* Address LSB bytes is in */
		if(addr == 0x01 ) LED1 = data;
		if(addr == 0x02 && data <= 180) servo = data+60;
		if(addr == 0x03 ) dis = data;
		if(addr == 0x04 ) mode = data;
		if(addr == 0x05 ) dir = data;
		if(addr == 0x06 ) duty = data;
		if(addr == 0x07 ) SCT_freg = data;
		/*	QEI  */
		if(addr == 0x10 ) LPC_QEI->CON = data;
		if(addr == 0x11 ) LPC_QEI->CONF = data;
		if(addr == 0x12 ) LPC_QEI->MAXPOS = data;
		if(addr == 0x13 ) LPC_QEI->CMPOS0 = data;
		if(addr == 0x14 ) LPC_QEI->CMPOS1 = data;
		if(addr == 0x15 ) LPC_QEI->CMPOS2 = data;
		if(addr == 0x16 ) LPC_QEI->INXCMP0 = data;
		if(addr == 0x17 ) LPC_QEI->LOAD = data;
		if(addr == 0x18 ) LPC_QEI->VELCOMP = data;
		if(addr == 0x19 ) LPC_QEI->FILTERPHA = data;
		if(addr == 0x1A ) LPC_QEI->FILTERPHB = data;
		if(addr == 0x1B ) LPC_QEI->FILTERINX = data;
		if(addr == 0x1C ) LPC_QEI->WINDOW = data;
		if(addr == 0x1D ) LPC_QEI->INXCMP1 = data;
		if(addr == 0x1E ) LPC_QEI->INXCMP2 = data;
		if(addr == 0x1F ) LPC_QEI->IEC = data;
		if(addr == 0x20 ) LPC_QEI->IES = data;
		if(addr == 0x21 ) LPC_QEI->CLR = data;
		if(addr == 0x22 ) LPC_QEI->SET = data;

		//Velocity contoroll
		if(addr == 0x30 ) Pgain = (Pgain & 0xff00) | data;
		if(addr == 0x31 ) Pgain = (Pgain & 0xff) | data << 8;
		if(addr == 0x32 ) Igain = (Igain & 0xff00) | data;
		if(addr == 0x33 ) Igain = (Igain & 0xff) | data << 8;
		if(addr == 0x34 ) setRPMval = (setRPMval & 0xff00) | data;
		if(addr == 0x35 ) setRPMval = (setRPMval & 0xff) | data << 8;


		//duty = (Chip_SCTPWM_GetTicksPerCycle(SCT_PWM)/2000*servo);
		//Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, duty);

		Board_LED_Set(2, LED1);
		changeFlag = 1;
	}
	else {
		/* Write data to emulated EEPROM */
	}
	addrbytes++;
	recvdata = data;

	return 0;
}

/* Handler for slave transfer complete callback */
static void processSlaveTransferDone(void)
{
	/* Nothing needs to be done here */
}

/* I2C slavecallback function list */
const static I2CS_XFER_T i2csCallBacks = {
	&processSlaveTransferStart,
	&processSlaveTransferSend,
	&processSlaveTransferRecv,
	&processSlaveTransferDone
};

extern "C" {
	void LPC_I2C_INTHAND(void)
	{
		uint32_t state = Chip_I2C_GetPendingInt(LPC_I2C_PORT);

		/* Error handling */
		if (state & (I2C_INTSTAT_MSTRARBLOSS | I2C_INTSTAT_MSTSTSTPERR)) {
			Chip_I2CM_ClearStatus(LPC_I2C_PORT, I2C_STAT_MSTRARBLOSS | I2C_STAT_MSTSTSTPERR);
		}

		/* Call I2CM ISR function with the I2C device and transfer rec */
		if (state & I2C_INTENSET_MSTPENDING) {
			Chip_I2CM_XferHandler(LPC_I2C_PORT, &i2cmXferRec);
		}

		/* I2C slave related interrupt */
		while (state & (I2C_INTENSET_SLVPENDING | I2C_INTENSET_SLVDESEL)) {
			Chip_I2CS_XferHandler(LPC_I2C_PORT, &i2csCallBacks);

			/* Update state */
			state = Chip_I2C_GetPendingInt(LPC_I2C_PORT);
		}
	}
}
#endif
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
#ifdef __cplusplus
extern "C" {
#endif
int i;

int USBstream = 0;

#ifdef current_sens
int currentSensTimer;
bool currentSensFirstFlag = true;
int ADseqCount;
uint32_t rawADvalue = 0;
long afterFiltterADvalue;
long tempValue;
#endif

#ifdef CAN_test
volatile uint32_t timer;				//timer
volatile uint32_t psec;					//parts of second
volatile uint32_t sec;
#endif

void SysTick_Handler(void)
{
	static int firstrockFlag = 0;
	static uint32_t setangle;

#ifdef current_sens
	//if(currentSensTimer == (int)TICKRATE_HZ1 / 200){
		if(ADseqCount == 10){
			ADseqCount = 0;
			afterFiltterADvalue = (long)rawADvalue/10;
			rawADvalue = 0;

			if(currentSensFirstFlag){
				CurrentSensNeutral = afterFiltterADvalue;
				currentSensFirstFlag = false;
			}
		}

		currentSensTimer = 0;
		rawADvalue += GetCurrentRawValue();
		Chip_ADC_StartSequencer(LPC_ADC1, ADC_SEQA_IDX);

		ADseqCount++;
	//}
#endif

	if(i == (int)TICKRATE_HZ1 / 5){
		i = 0;
#ifdef temp_sens
	static uint32_t count;
	static uint32_t rawSample;
	if (count >= (TICKRATE_HZ1 / 10)) {
			count = 0;
			/* Restart temperature cycle */
			tempStartCycle();
		}
	if (tempCycleComplete()) {
				rawSample = tempGetSample();
				tempValue = rawSample;
	}
	count++;
#endif

	Board_LED_Toggle(1);

	if(USBstream == 1){
		printf("STAT:%d POS:%d CAP:%d\t RPM:%d\t angle:%d TEMPnow:%3.2f (first:%3.2f) CRTr:%3.2f P:%d I:%d set:%d OUT:%d IN:%d road:%d\n",
							LPC_QEI->STAT,LPC_QEI->POS,LPC_QEI->CAP, GetRPM(),angle,
							GetTempSensRealValue(ADC_DR_RESULT(rawSample)),
							firstTemp,
							//GetCurrentRealValue(GetCurrentRawValue()),
							GetCurrentRealValue(afterFiltterADvalue),
							Pgain, Igain, setRPMval,
							OUTsideVal,INsideVal,roadVal
							);

	}else if(USBstream == 2){
		printf("CAN ID:%x\t DLC:%x\t [0]:%x\t [1]:%x\t [2]:%x\t [3]:%x\t\n",rx.id, rx.dlc,
					rx.data[0], rx.data[1], rx.data[2], rx.data[3]);
	}

	angle = (uint32_t)LPC_QEI->POS*360/LPC_QEI->MAXPOS;

	if (angleStream){
		tx.id  = EEPROM1SLVADD;
		tx.dlc = 8;
		tx.data[0] = (uint32_t)0;
		tx.data[1] = (uint32_t)0x23;
		tx.data[2] = (uint32_t)angle;
		tx.data[3] = (uint32_t)2;

		CAN_Send((uint32_t *)&tx);
	}

	//setMotorRPM(setRPMval, LPC_QEI->CAP, Pgain, Igain);
	}
#ifdef CAN_test
	 timer++;								//inc timer
	 psec++;								//inc parts of second
	 if(psec>=1000)							//1000ms
	 {
	  psec =0;								//reset
	  sec  =1;								//second flag
	 }


#endif
	if(setRPMval == 0 && rockEnable){
		/*
		if(firstrockFlag == 0){
			setangle = LPC_QEI->POS;
			firstrockFlag = 1;
		}*/
		 servoRock(1,setangle ,rockPgain,rockIgain, rockDgain , rockRecv);

	}else{
		setMotorRPM(setRPMval, LPC_QEI->CAP, Pgain, Igain, Dgain);
		setangle = LPC_QEI->POS;
		firstrockFlag = 0;
	}
	i++;
	currentSensTimer++;
}
#ifdef __cplusplus
}
#endif

#define MAXDIPS 4
static const uint8_t dippin[MAXDIPS] = {29, 28, 27, 26};
static const uint8_t dipports[MAXDIPS] = {0, 0, 0, 0};

void dipSwitchsetting(){
	int idx;

	for (idx = 0; idx <  MAXDIPS; idx++) {
		/* Set the GPIO as output with initial state off (high) */
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, dipports[idx], dippin[idx]);
	}
}

uint16_t i2cSetAddress(){
	int idx;
	int value[MAXDIPS];
	int res;

	for (idx = 0; idx <  MAXDIPS; idx++) {
		/* Set the GPIO as output with initial state off (high) */
		value[idx] = !Chip_GPIO_ReadPortBit(LPC_GPIO, dipports[idx],dippin[idx]);
	}

	res = 0x20 | (value[0] << 3) | (value[1] << 2)| (value[2] << 1)| (value[3]);

	EEPROM1SLVADD =res;
	return res;
}
/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	uint32_t sysTickRate;
	uint32_t j;
	uint32_t sleep;
#ifdef usb_read
	uint32_t prompt = 0, rdCnt = 0;
#endif

	SystemCoreClockUpdate();
	Board_Init();
	dipSwitchsetting();
	i2cSetAddress();

	for(sleep=0; sleep < EEPROM1SLVADD*10000 ;sleep++);

	/*
	 * SWM setting
	 */
	SwitchMatrix_Init();
	IOCON_Init();
	InputMux_Init();

#if motordrive
	SCT_init();

	SCT_freg = 10;
	last_SCT_freg = SCT_freg;
	set_sct_fregencey(SCT_freg * 1000);

	set_duty(AHI_pin,0);
	set_duty(ALI_pin,0);
	set_duty(BHI_pin,0);
	set_duty(BLI_pin,0);
#endif

#if usb_com
	Board_USB_init();
#endif

#ifdef QEI
	prvSetupHardware();
#endif

#ifdef current_sens
		CurrentSensInit();
#endif

	for(i=0;i<6;i++){
		Board_LED_Toggle(2);
		for(j = 0;j<1200000;j++);
	}
	Board_LED_Set(0, false);	//red
	Board_LED_Set(1, true);		//green
	//Board_LED_Set(2, true);		//blue
	/* The sysTick counter only has 24 bits of precision, so it will
	   overflow quickly with a fast core clock. You can alter the
	   sysTick divider to generate slower sysTick clock rates. */
	Chip_Clock_SetSysTickClockDiv(1);

	/* A SysTick divider is present that scales the sysTick rate down
	   from the core clock. Using the SystemCoreClock variable as a
	   rate reference for the SysTick_Config() function won't work,
	   so get the sysTick rate by calling Chip_Clock_GetSysTickClockRate() */
	sysTickRate = Chip_Clock_GetSysTickClockRate();

	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(sysTickRate / TICKRATE_HZ1);

#if i2ctest
	/* Setup I2C pin muxing */
		Init_I2C_PinMux();

		/* Setup I2C, master, and slave */
		setupI2CMaster();
		setupI2CSlave();

		/* Enable the interrupt for the I2C */
		NVIC_EnableIRQ(LPC_IRQNUM);
#endif

#ifdef temp_sens
		temp_sens_init();
#endif

#ifdef CAN_test
		CAN_Board_Init();
#endif

	/* LEDs toggle in interrupt handlers */
	while (1) {

#if 1
	if(last_SCT_freg != SCT_freg){
			set_sct_fregencey(SCT_freg*1000);
			last_SCT_freg = SCT_freg;
		}

	if(changeFlag){

		//drive_motor(mode,dir,duty);
		enable_driver(dis);
		changeFlag = 0;
	}
#endif

#ifdef usb_read
		uint16_t addr,reg,val,rw,res;
		/* If VCOM port is opened echo whatever we receive back to host. */
		rdCnt = vcom_bread(&vcom_rxBuff[0], 256);
		if (rdCnt) {
			//vcom_write(&g_rxBuff[0], rdCnt);

			if(vcom_rxBuff[0] == 'w'){ //write command
				sscanf((char *)vcom_rxBuff,"%c %x@%x %x",&rw,&addr,&reg,&val);
				if(addr == EEPROM1SLVADD){
					message_Action( reg, val, 1, CDC);
				}

#ifdef CAN_test
				if(addr != EEPROM1SLVADD){
					tx.id  = EEPROM1SLVADD;
					tx.dlc = 8;
					tx.data[0] = (uint32_t)addr;
					tx.data[1] = (uint32_t)reg;
					tx.data[2] = (uint32_t)val;
					tx.data[3] = (uint32_t)1;

					CAN_Send((uint32_t *)&tx);
				}
#endif
			}else{						//read command
				sscanf((char *)vcom_rxBuff,"%c %x@%x",&rw,&addr,&reg);
				if(addr == EEPROM1SLVADD){
					message_Action( reg, val, 0, CDC);
				}

#ifdef CAN_test
				if(addr != EEPROM1SLVADD){
					tx.id  = EEPROM1SLVADD;
					tx.dlc = 8;
					tx.data[0] = (uint32_t)addr;
					tx.data[1] = (uint32_t)reg;
					tx.data[2] = (uint32_t)0;
					tx.data[3] = (uint32_t)0;

					CAN_Send((uint32_t *)&tx);
				}
#endif
			}
#if 0
			if(vcom_rxBuff[0] == '1') Board_LED_Set(0, true);
			else Board_LED_Set(0, false);
#endif
		}
#endif
		__WFI();
	}

	return 0;
}

#ifdef temp_sens
/*********************************************************************
 * ADC setting functions
 * *******************************************************************/
/* This starts an ADC temperature sampling sequence using the recommended
   burst method. It bursts 10 temperature sensor samples via the ADC and
   then stops the ADC sequencer. The single sequence (non-burst) mode can
   also be used for ADC temperature sensor read. */
static void tempStartCycle(void)
{
	tempSampleIdx = 0;
	tempSequenceComplete = false;

	/* Enable burst mode */
	Chip_ADC_StartBurstSequencer(LPC_ADC0, ADC_SEQA_IDX);
}

/* Used to indicate when a temperature cycle is complete and the sample
   is ready */
static bool tempCycleComplete(void)
{
	return tempSequenceComplete;
}

/* Returns the last temperature sample only. Only valid if tempCycleComplete()
   returns true */
static uint32_t tempGetSample(void)
{
	tempSequenceComplete = false;
	return temp[TEMPSAMPLES - 1];
}

/**
 * @brief	Handle interrupt from ADC sequencer A
 * @return	Nothing
 */
extern "C" {
	void ADC0A_IRQHandler(void){
		uint32_t pending;

		/* Get pending interrupts */
		pending = Chip_ADC_GetFlags(LPC_ADC0);

		/* Sequence A completion interrupt */
		if (pending & ADC_FLAGS_SEQA_INT_MASK) {
			if (tempSampleIdx < TEMPSAMPLES) {
				/* Save sample */
				temp[tempSampleIdx] = Chip_ADC_GetDataReg(LPC_ADC0, 0);
				tempSampleIdx++;

				if (tempSampleIdx >= TEMPSAMPLES) {
					Chip_ADC_StopBurstSequencer(LPC_ADC0, ADC_SEQA_IDX);
					tempSequenceComplete = true;
				}
			}
		}

		/* Clear any pending interrupts */
		Chip_ADC_ClearFlags(LPC_ADC0, pending);
	}
}

float GetTempSensRealValue(uint32_t raw){
	return -(((((float)raw / (float) 0xfff) * 3300.0f) -577.3f) / 2.29f);
}

void temp_sens_init(){
	/* Setup ADC for 12-bit mode and normal power */
	Chip_ADC_Init(LPC_ADC0, 0);

	/* Setup ADC clock rate */
	Chip_ADC_SetClockRate(LPC_ADC0, 250000);

	/* For ADC0, select temperature sensor for channel 0 on ADC0 */
	Chip_ADC_SetADC0Input(LPC_ADC0, ADC_INSEL_TS);

	/* Setup a sequencer to do the following:
	   Perform ADC conversion of ADC channels 0 with EOS interrupt */
	Chip_ADC_SetupSequencer(LPC_ADC0, ADC_SEQA_IDX, (ADC_SEQ_CTRL_CHANSEL(0) |
													 ADC_SEQ_CTRL_MODE_EOS));

	/* Power up the internal temperature sensor - this also selects the
	    temperature sensor as the input for the ADC0 input */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_TS_PD);

	/* Use higher voltage trim */
	Chip_ADC_SetTrim(LPC_ADC0, ADC_TRIM_VRANGE_HIGHV);

	/* Need to do a calibration after initialization and trim */
	Chip_ADC_StartCalibration(LPC_ADC0);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC0))) {}

	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC0, Chip_ADC_GetFlags(LPC_ADC0));

	/* Enable ADC sequence A completion interrupt */
	Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE);

	/* Enable ADC NVIC interrupt */
	NVIC_EnableIRQ(ADC0_SEQA_IRQn);

	/* Enable sequencer */
	Chip_ADC_EnableSequencer(LPC_ADC0, ADC_SEQA_IDX);

	/* first sampling */
	/* Restart temperature cycle */
	tempStartCycle();
	while (!tempCycleComplete());

	firstTemp = GetTempSensRealValue(ADC_DR_RESULT(tempGetSample()));
}
#endif
/*********************************************************************
 * swm setting functions
 * *******************************************************************/

void InputMux_Init()
{
    LPC_INMUX->SCT0_INMUX[0] = 31;    /*  */
    LPC_INMUX->SCT0_INMUX[1] = 31;    /*  */
    LPC_INMUX->SCT0_INMUX[2] = 31;    /*  */
    LPC_INMUX->SCT0_INMUX[3] = 31;    /*  */
    LPC_INMUX->SCT0_INMUX[4] = 31;    /*  */
    LPC_INMUX->SCT0_INMUX[5] = 31;    /*  */
    LPC_INMUX->SCT0_INMUX[6] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[0] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[1] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[2] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[3] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[4] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[5] = 31;    /*  */
    LPC_INMUX->SCT1_INMUX[6] = 31;    /*  */
    LPC_INMUX->SCT2_INMUX[0] = 31;    /*  */
    LPC_INMUX->SCT2_INMUX[1] = 31;    /*  */
    LPC_INMUX->SCT2_INMUX[2] = 31;    /*  */
    LPC_INMUX->SCT3_INMUX[0] = 31;    /*  */
    LPC_INMUX->SCT3_INMUX[1] = 31;    /*  */
    LPC_INMUX->SCT3_INMUX[2] = 31;    /*  */
    LPC_INMUX->PINTSEL[0] = 127;    /*  */
    LPC_INMUX->PINTSEL[1] = 127;    /*  */
    LPC_INMUX->PINTSEL[2] = 127;    /*  */
    LPC_INMUX->PINTSEL[3] = 127;    /*  */
    LPC_INMUX->PINTSEL[4] = 127;    /*  */
    LPC_INMUX->PINTSEL[5] = 127;    /*  */
    LPC_INMUX->PINTSEL[6] = 127;    /*  */
    LPC_INMUX->PINTSEL[7] = 127;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[0] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[1] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[2] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[3] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[4] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[5] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[6] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[7] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[8] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[9] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[10] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[11] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[12] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[13] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[14] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[15] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[16] = 31;    /*  */
    LPC_INMUX->DMA_ITRIG_INMUX[17] = 31;    /*  */
    LPC_INMUX->DMA_INMUX[0] = 31;    /*  */
    LPC_INMUX->DMA_INMUX[1] = 31;    /*  */
    LPC_INMUX->DMA_INMUX[2] = 31;    /*  */
    LPC_INMUX->DMA_INMUX[3] = 31;    /*  */
    LPC_INMUX->FREQMEAS_REF = 15;    /*  */
    LPC_INMUX->FREQMEAS_TARGET = 15;    /*  */

}


void IOCON_Init() {
       /* Enable IOCON clock */
       Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

       /* Pin I/O Configuration */
       /*LPC_IOCON->PIO[0][0]= 0x90; */
       /*LPC_IOCON->PIO[0][1]= 0x90; */
       /*LPC_IOCON->PIO[0][2]= 0x90; */
       /*LPC_IOCON->PIO[0][3]= 0x90; */
       /*LPC_IOCON->PIO[0][4]= 0x90; */
       LPC_IOCON->PIO[0][5]= 0x80;
       LPC_IOCON->PIO[0][6]= 0x80;
       LPC_IOCON->PIO[0][7]= 0x80;
       /*LPC_IOCON->PIO[0][8]= 0x90; */
       LPC_IOCON->PIO[0][9]= 0x88;
       LPC_IOCON->PIO[0][10]= 0x88;
       LPC_IOCON->PIO[0][11]= 0x88;
       /*LPC_IOCON->PIO[0][12]= 0x90; */
       /*LPC_IOCON->PIO[0][13]= 0x90; */
       LPC_IOCON->PIO[0][14]= 0x88;
       /*LPC_IOCON->PIO[0][15]= 0x90; */
       /*LPC_IOCON->PIO[0][16]= 0x90; */
       /*LPC_IOCON->PIO[0][17]= 0x90; */
       /*LPC_IOCON->PIO[0][18]= 0x90; */
       /*LPC_IOCON->PIO[0][19]= 0x90; */
       /*LPC_IOCON->PIO[0][20]= 0x90; */
       /*LPC_IOCON->PIO[0][21]= 0x90; */
       /*LPC_IOCON->PIO[0][22]= 0x90; */
       /*LPC_IOCON->PIO[0][23]= 0x90; */
       /*LPC_IOCON->PIO[0][24]= 0x90; */
       /*LPC_IOCON->PIO[0][25]= 0x90; */
       /*LPC_IOCON->PIO[0][26]= 0x90; */
       /*LPC_IOCON->PIO[0][27]= 0x90; */
       /*LPC_IOCON->PIO[0][28]= 0x90; */
       /*LPC_IOCON->PIO[0][29]= 0x90; */

}

void SwitchMatrix_Init()
{
       /* Enable the clock to the Switch Matrix */
       Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

       /* Pin Assign 8 bit Configuration */
       /* CAN0_TD */
       /* CAN0_RD */
       LPC_SWM->PINASSIGN[6] = 0xff120dffUL;
       /* SCT1_OUT0 */
       /* SCT1_OUT1 */
       /* SCT1_OUT2 */
       LPC_SWM->PINASSIGN[8] = 0xff0b0a09UL;
       /* QEI_PHA */
       /* QEI_PHB */
       /* QEI_IDX */
       LPC_SWM->PINASSIGN[14] = 0x050607ffUL;

       /* Pin Assign 1 bit Configuration */
       /* ADC1_8 */
       LPC_SWM->PINENABLE[0] = 0xffefffffUL;
       /* I2C0_SDA */
       /* I2C0_SCL */
       /* SCT1_OUT5 */
       /* RESET */
       /* SWCLK */
       /* SWDIO */
       LPC_SWM->PINENABLE[1] = 0xff1fefe7UL;

}

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

#ifdef QEI
/*********************************************************************
 * QEI setting functions
 * *******************************************************************/
uint32_t GetRPM(){
	static uint32_t PCLK = Chip_Clock_GetSystemClockRate();
	//static uint32_t Load = PCLK/Edges;
	static uint32_t Load = LPC_QEI->LOAD;
	uint32_t Speed;

	Speed = LPC_QEI->CAP;

	//return (uint32_t)(PCLK * Speed * 60) / (Load * PPR * Edges);
	return (uint32_t)(Speed * 60) / (PPR * 4);
}

static void prvSetupHardware(void)
{

    /* LED0 is used for the link status, on = PHY cable detected */
    //Board_Init();

    //Définir la broche P1.23 comme point B du QEI

    Chip_GPIO_WriteDirBit(LPC_GPIO, 0x0, 0x5, false);
    //Chip_IOCON_PinMux(LPC_IOCON,0x1,0x14,  IOCON_MODE_PULLDOWN ,IOCON_FUNC1);
    Chip_GPIO_WriteDirBit(LPC_GPIO, 0x0, 0x6, false);
    //Chip_IOCON_PinMux(LPC_IOCON,0x1,0x17,  IOCON_MODE_PULLDOWN ,IOCON_FUNC1);
    Chip_GPIO_WriteDirBit(LPC_GPIO, 0x0, 0x7, false);
    //Chip_IOCON_PinMux(LPC_IOCON,0x1,0x18,  IOCON_MODE_PULLDOWN ,IOCON_FUNC1);

    //Activer l'horloge du QEI
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_QEI);
    //Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_QEI, SYSCTL_CLKDIV_4);
    Chip_SYSCTL_PeriphReset(RESET_QEI0);


    //Config du QEI en comptage simple sur la voie B
    LPC_QEI->CONF=4;

    //Définition du calcul de vitesse
    LPC_QEI->LOAD=(uint32_t)QEI_LOAD;
    LPC_QEI->MAXPOS=QEI_MAPPOS;

    //Filtrage du QEI
    LPC_QEI->FILTERPHB=10;

    //Reset du QEI
    LPC_QEI->CON=15;

    //Préparation de l'interruption de la vitesse
    //Autorisation de l'interruption uniquement sur overflow timer vitesse
    LPC_QEI->IES=1;

    NVIC_SetPriority((IRQn_Type) QEI_IRQn, 2);
    NVIC_EnableIRQ((IRQn_Type) QEI_IRQn);

}
#endif

/*********************************************************************
 * USB Visual COM port setting functions
 * *******************************************************************/
#if usb_com
/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
	USB_COMMON_DESCRIPTOR *pD;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
	uint32_t next_desc_adr;

	pD = (USB_COMMON_DESCRIPTOR *) pDesc;
	next_desc_adr = (uint32_t) pDesc;

	while (pD->bLength) {
		/* is it interface descriptor */
		if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

			pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
			/* did we find the right interface descriptor */
			if (pIntfDesc->bInterfaceClass == intfClass) {
				break;
			}
		}
		pIntfDesc = 0;
		next_desc_adr = (uint32_t) pD + pD->bLength;
		pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
	}

	return pIntfDesc;
}

#if 1
void uprintf(const char *format, ...){
	va_list ap;
	va_start(ap, format);
	char* allocatedBuffer;
	int size = vasprintf(&allocatedBuffer, format, ap);
	va_end(ap);

	//sprintf(&g_rxBuff[0],format,__format__ );
	vcom_write((uint8_t*)allocatedBuffer, size);

	free(allocatedBuffer);
}
#else
#include "debug.h"
#endif


#define mbed 0

void Board_USB_init(){
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret = LPC_OK;
	uint32_t prompt = 0, rdCnt = 0;
	uint32_t wait,waitTime;

	waitTime = (uint32_t)(Chip_Clock_GetSystemClockRate() /10 * 3);

	NVIC_DisableIRQ(USB0_IRQn);

	/* enable clocks */
	Chip_USB_Init();

    // to ensure that the USB host sees the device as
    // disconnected if the target CPU is reset.
	//for(wait=0; wait < waitTime; wait++);
#if 1
	/* initialize USBD ROM API pointer. */
	g_pUsbApi = (const USBD_API_T *) LPC_ROM_API->pUSBD;

	/* initialize call back structures */
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB0_BASE;
	/*	WORKAROUND for artf44835 ROM driver BUG:
		Code clearing STALL bits in endpoint reset routine corrupts memory area
		next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
		EP2_IN are used we need to specify 3 here. But as a workaround for this
		issue specify 4. So that extra EPs control structure acts as padding buffer
		to avoid data corruption. Corruption of padding memory doesn’t affect the
		stack/program behaviour.
	 */
	usb_param.max_num_ep = 3 + 1;
	usb_param.mem_base = USB_STACK_MEM_BASE;
	usb_param.mem_size = USB_STACK_MEM_SIZE;

	/* Set the USB descriptors */
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	/* Note, to pass USBCV test full-speed only devices should have both
	   descriptor arrays point to same location and device_qualifier set to 0.
	 */
	desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.device_qualifier = 0;

	/* USB Initialization */
	ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
	if (ret == LPC_OK) {

		/* Init VCOM interface */
		ret = vcom_init(g_hUsb, &desc, &usb_param);
		if (ret == LPC_OK) {
			/*  enable USB interrupts */
			NVIC_EnableIRQ(USB0_IRQn);
			/* now connect */
			USBD_API->hw->Connect(g_hUsb, 1);
		}
	}
#endif


#if 0
	/*** wait until when any key push  ***/
		while (!(vcom_bread(&g_rxBuff[0], 256)));
		vcom_write((uint8_t*)"Hello World!!\r\n", 15);
		/*************************************/
#endif
		printf("start\n");
}
#endif

/*********************************************************************
 * Current sensor setting functions
 * *******************************************************************/
#ifdef current_sens
void CurrentSensInit(){
	/* Setup ADC for 12-bit mode and normal power */
	Chip_ADC_Init(LPC_ADC1, 0);

	/* Setup for maximum ADC clock rate */
	Chip_ADC_SetClockRate(LPC_ADC1, ADC_MAX_SAMPLE_RATE);

	Chip_ADC_SetupSequencer(LPC_ADC1, ADC_SEQA_IDX, (ADC_SEQ_CTRL_CHANSEL(8) |
															 ADC_SEQ_CTRL_MODE_EOS));

#if defined(BOARD_NXP_LPCXPRESSO_1549)
	/* Use higher voltage trim for both ADCs */
	Chip_ADC_SetTrim(LPC_ADC1, ADC_TRIM_VRANGE_HIGHV);




#if 0
	/* For ADC1, seqeucner A will be used with threshold events.
	   It will be triggered manually by the sysTick interrupt and
	   only monitors the ADC1 input. */
	Chip_ADC_SetupSequencer(LPC_ADC1, ADC_SEQA_IDX,
							(ADC_SEQ_CTRL_CHANSEL(BOARD_ADC_CH) | ADC_SEQ_CTRL_MODE_EOS));

	/* Disables pullups/pulldowns and disable digital mode */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 8, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	/* Assign ADC1_1 to PIO0_9 via SWM (fixed pin) */
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC1_1);
#endif
#endif

	/* Need to do a calibration after initialization and trim */
	Chip_ADC_StartCalibration(LPC_ADC1);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC1))) {}
#if 0
	/* Setup threshold 0 low and high values to about 25% and 75% of max for
		 ADC1 only */
	Chip_ADC_SetThrLowValue(LPC_ADC1, 0, ((1 * 0xFFF) / 4));
	Chip_ADC_SetThrHighValue(LPC_ADC1, 0, ((3 * 0xFFF) / 4));
#endif

	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC1, Chip_ADC_GetFlags(LPC_ADC1));

#if 0
	/* Enable sequence A completion and threshold crossing interrupts for ADC1_1 */
	Chip_ADC_EnableInt(LPC_ADC1, ADC_INTEN_SEQA_ENABLE |
					   ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH));
#endif

#if 0
	/* Use threshold 0 for ADC channel and enable threshold interrupt mode for
	   channel as crossing */
	Chip_ADC_SelectTH0Channels(LPC_ADC1, ADC_THRSEL_CHAN_SEL_THR1(BOARD_ADC_CH));
	Chip_ADC_SetThresholdInt(LPC_ADC1, BOARD_ADC_CH, ADC_INTEN_THCMP_CROSSING);
#endif
	/* Enable related ADC NVIC interrupts */
	NVIC_EnableIRQ(ADC1_SEQA_IRQn);
	//NVIC_EnableIRQ(ADC1_THCMP);

	/* Enable sequencers */
	Chip_ADC_EnableSequencer(LPC_ADC1, ADC_SEQA_IDX);
}

void CurrentSensNeutralCalibration();

long GetCurrentRawValue(){
	return ADC_DR_RESULT( Chip_ADC_GetDataReg(LPC_ADC1, 8) );
}

float GetCurrentRealValue(long raw){
	return (float)(raw - CurrentSensNeutral)*3300/(4096*110);
}

extern "C"{
/**
 * @brief	Handle interrupt from ADC1 sequencer A
 * @return	Nothing
 */
void ADC1A_IRQHandler(void)
{
	uint32_t pending;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC1);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		sequence1Complete = true;
	}

	/* Clear Sequence A completion interrupt */
	Chip_ADC_ClearFlags(LPC_ADC1, ADC_FLAGS_SEQA_INT_MASK);
}

#if 0
/**
 * @brief	Handle threshold interrupt from ADC1
 * @return	Nothing
 */
void ADC1_THCMP_IRQHandler(void)
{
	uint32_t pending;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC1);

	/* Threshold crossing interrupt on ADC input channel */
	if (pending & ADC_FLAGS_THCMP_MASK(BOARD_ADC_CH)) {
		threshold1Crossed = true;
	}

	/* Clear threshold interrupt */
	Chip_ADC_ClearFlags(LPC_ADC1, ADC_FLAGS_THCMP_MASK(BOARD_ADC_CH));
}
#endif
}
#endif

/*********************************************************************
 * RPM contoroll setting functions
 * *******************************************************************/
#ifdef velContoroll
int32_t integVal;

void setMotorRPM(uint32_t setRPM, uint32_t motorENC, int32_t P_gain, int32_t I_gain, int32_t D_gain){
	static int32_t diff, lastRoadval;
	static int32_t pre_setRPM,pre_motorENC,d_diff;

	diff = (int32_t)((int32_t)(setRPM - motorENC)/100);
	d_diff = (setRPM - pre_setRPM) - (motorENC - pre_motorENC);

	roadVal = (int32_t)((diff * P_gain) + (integVal/100 * I_gain) + (d_diff * D_gain/ 100)) ;
	if (setRPM == 0) roadVal = 0;


	integVal += diff;
	lastRoadval = roadVal;

	if (integVal > 7200) integVal = 7200;
	else if (integVal < -7200) integVal = -7200;

	if (roadVal > 7200) roadVal= 7200;
	else if (roadVal < 0) roadVal = 0;

	pre_setRPM = setRPM;
	pre_motorENC = motorENC;

	setMotorRealDuty(roadVal);
}

void set_real_duty(uint8_t ch,uint32_t duty){
	//uint32_t OUTsideVal,INsideVal,rate;

	rate = get_sct_rate();
	OUTsideVal = (uint32_t)(duty);
	INsideVal = rate - OUTsideVal;

	if(OUTsideVal > rate) 	OUTsideVal = rate;
	if(INsideVal > rate)	INsideVal = rate;

	switch(ch){
		case AHI_pin:
			sct_fsm_reload_AHI(OUTsideVal);
			break;
		case ALI_pin:
			sct_fsm_reload_ALI(INsideVal);
			break;
		case BHI_pin:
			sct_fsm_reload_BHI(INsideVal);
			break;
		case BLI_pin:
			sct_fsm_reload_BLI(OUTsideVal);
			break;
		default:
			sct_fsm_reload_AHI(OUTsideVal);
			sct_fsm_reload_ALI(INsideVal);
			sct_fsm_reload_BHI(INsideVal);
			sct_fsm_reload_BLI(OUTsideVal);
	}
}

void setMotorRealDuty(uint32_t realDutyVal){
	//uint32_t OUTsideVal,INsideVal,rate;
	static uint32_t max =  get_sct_rate();
	if(mode == onBrake){
		if(dir == Aside){
			set_real_duty(AHI_pin, realDutyVal);
			set_real_duty(ALI_pin, (max - (uint16_t)(realDutyVal*0.95)));
			set_real_duty(BHI_pin, 0);
			set_real_duty(BLI_pin, max);
		}else{
			set_real_duty(AHI_pin, 0);
			set_real_duty(ALI_pin, max);
			set_real_duty(BHI_pin, realDutyVal);
			set_real_duty(BLI_pin, (max - (uint16_t)(realDutyVal*0.95)));
		}
	}else{
		if(dir == Aside){
			set_real_duty(AHI_pin, max);
			set_real_duty(ALI_pin, 0);
			set_real_duty(BHI_pin, 0);
			set_real_duty(BLI_pin, realDutyVal);
		}else{
			set_real_duty(AHI_pin, 0);
			set_real_duty(ALI_pin, realDutyVal);
			set_real_duty(BHI_pin, max);
			set_real_duty(BLI_pin, 0);
		}
	}
}
#endif

#ifdef CAN_test
void CAN_Message_Receive_Action(message_object msg){
	uint32_t addr,reg,val,rw,res;
	/*
	printf("CAN ID:%x\t DLC:%x\t [0]:%x\t [1]:%x\t [2]:%x\t [3]:%x\t\n",rx.id, rx.dlc,
	   					rx.data[0], rx.data[1], rx.data[2], rx.data[3]);
	*/
	addr = msg.data[0];
	reg  = msg.data[1];
	val  = msg.data[2];
	rw 	 = msg.data[3];

	if(addr == EEPROM1SLVADD){
		message_Action(reg, val, rw, CAN);
	}
}
#endif

void message_Action(uint16_t reg, uint16_t val, uint16_t rw, uint16_t CDCorCAN){
	uint32_t res;
	if(rw == 1){			//register write sequence

		if(reg == 0x01 ) LED1 = val;
		if(reg == 0x02 ) USBstream = val;
		if(reg == 0x03 ) dis = val;
		if(reg == 0x04 ) mode = val;
		if(reg == 0x05 ) dir = val;
		if(reg == 0x06 ) duty = val;
		if(reg == 0x07 ) SCT_freg = val;
		if(reg == 0x08 ) angleStream = val;

		//velocity Control
		if(reg == 0x10 ) Pgain = (uint32_t)val;
		if(reg == 0x11 ) Igain = (uint32_t)val;
		if(reg == 0x12 ) Dgain = (uint32_t)val;
		if(reg == 0x13 ) setRPMval = (uint32_t)val;

		if(reg == 0x15 ) rockEnable = (uint32_t)val;
		if(reg == 0x16 ) rockRecv = (uint32_t)val;
		if(reg == 0x17 ) rockPgain = (uint32_t)val;
		if(reg == 0x18 ) rockIgain = (uint32_t)val;
		if(reg == 0x19 ) rockDgain = (uint32_t)val;
		if(reg == 0x19 ) rockDgain = (uint32_t)val;

		//endter ISP
		if(reg == 0x44 ) endter_ISP((uint8_t)val);

		changeFlag = 1;
		Board_LED_Set(2, LED1);
	}else if(rw == 0) {	//register read sequence and send to host

		if(reg == 0x01 ) res = LED1;
		if(reg == 0x02 ) res = USBstream;
		if(reg == 0x03 ) res = dis;
		if(reg == 0x04 ) res = mode;
		if(reg == 0x05 ) res = dir;
		if(reg == 0x06 ) res = duty;
		if(reg == 0x07 ) res = SCT_freg;
		if(reg == 0x08 ) res = angleStream;

		//velocity Control
		if(reg == 0x10 ) res = Pgain;
		if(reg == 0x11 ) res = Igain;
		if(reg == 0x12 ) res = Dgain;
		if(reg == 0x13 ) res = setRPMval;

		if(reg == 0x20 ) res = LPC_QEI->STAT;
		if(reg == 0x21 ) res = LPC_QEI->POS;
		if(reg == 0x22 ) res = LPC_QEI->CAP;
		if(reg == 0x23 ) res = angle;

		if(reg == 0x30 ) res = (uint32_t)(GetTempSensRealValue(ADC_DR_RESULT(tempValue))*10);	//temp
		if(reg == 0x31 ) res = (int16_t)(GetCurrentRealValue(afterFiltterADvalue)*100);	//current

		if(CDCorCAN == CDC)printf("%x:%x:%d",EEPROM1SLVADD,reg,res);
		else if(CDCorCAN == CAN){
			tx.id  = EEPROM1SLVADD;
			tx.dlc = 8;
			tx.data[0] = (uint32_t)EEPROM1SLVADD;
			tx.data[1] = (uint32_t)reg;
			tx.data[2] = (uint32_t)res;
			tx.data[3] = (uint32_t)3;

			CAN_Send((uint32_t *)&tx);
		}
	}else if(rw == 2){
		printf("%x:%x:%d",EEPROM1SLVADD,reg,val);
	}
}

void servoRock(int enable,uint32_t setangle ,int32_t rP_gain, int32_t rI_gain, int32_t rD_gain, int recv){

	static int32_t diff, lastRoadval;
	static int32_t pre_setRPM,pre_motorENC,d_diff;
	static int32_t val;
	static uint32_t setRPMval;
	static int rv;


	diff = (int32_t)((int32_t)(setangle - LPC_QEI->POS)/10);

	if (diff < -16000){
		diff = setangle + (QEI_MAPPOS - LPC_QEI->POS);
	}else if (diff > 16000){
		diff = (LPC_QEI->POS* -1 ) + ( setangle - QEI_MAPPOS);
	}

	d_diff = (setangle - pre_setRPM) - (LPC_QEI->POS - pre_motorENC);

	val = (int32_t)((diff * rP_gain) + (integVal/10 * rI_gain) + (d_diff * rD_gain/ 10)) ;


	integVal += diff;
	lastRoadval = val;

	if (integVal > 7200) integVal = 7200;
	else if (integVal < -7200) integVal = -7200;

	if (val > 7200) val= 7200;
	//else if (val < 0) val = 0;

	pre_setRPM = setangle;
	pre_motorENC = LPC_QEI->POS;

	if(enable){
		if (val < 0){
			dir = recv;
			setRPMval = val*-1;
		}else{
			dir = (1 - recv);
			setRPMval = val;
		}

		setMotorRPM(setRPMval, LPC_QEI->CAP, Pgain, Igain, Dgain);
	}
}
