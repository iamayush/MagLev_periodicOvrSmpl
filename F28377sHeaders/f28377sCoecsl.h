/*
 * f28377sCoecsl.h
 *
 * equivalent of f28335 DSP/BIOS coecsl.h
 *
 * copied from f28377D_coecsl.h
 *
 *  Created on: Jul 14, 2015
 *      Author: fujinam2
 */

#ifndef F28377SCOECSL_H_
#define F28377SCOECSL_H_


/* SYS/BIOS includes */
#ifdef SYSBIOS
//#include <std.h>
//#include <mem.h>
//#include <sem.h>
//#include <que.h>
//#include <log.h>
//#include <sys.h>
//#include <tsk.h>
//#include <hwi.h>
//#include <swi.h>
//#include <rtdx.h>
//#include <clk.h>

/* non-SYS/BIOS includes */
#else
#define TRUE		1
#define FALSE		0
#endif


/* standard ANSI C includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

/* device includes */
//#include <DSP2833x_Device.h>
#include <F2837xS_device.h>
#include "F2837xS_Examples.h"

/* general constants */
#define PI			3.1415926535897932384626433832795
#define TWOPI		6.283185307179586476925286766559
#define HALFPI		1.5707963267948966192313216916398
#define GRAV		9.81


/* COECSL types and macros */
//typedef int bool;

//#define PLL_IMULT					0x14		//20
#define PLL_IMULT					0x28		//40

//using external clock oscillator chip.
//#ifndef F28377_CONTROL_CARD
//	#define OSCCLK_KHZ			20000L	//20 MHz
//#else
//    #ifdef F28377_CONTROL_CARD30
//        #define OSCCLK_KHZ          20000L	//20 MHz
//    #else
//	    #define OSCCLK_KHZ			20000L	//20 MHz
//    #endif
//#endif

#ifndef F28377_CONTROL_CARD
	#define OSCCLK_KHZ			10000L	//10 MHz
#else
    #ifdef F28377_CONTROL_CARD30
        #define OSCCLK_KHZ          10000L	//10 MHz
    #else
	    #define OSCCLK_KHZ			10000L	//10 MHz
    #endif
#endif




#if PLL_IMULT
#define SYSCLKOUT_KHZ	(OSCCLK_KHZ*PLL_IMULT/((ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV == 0) ? 1 : (ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV*2)))
#else
#define SYSCLKOUT_KHZ		(OSCCLK_KHZ)	//10MHz
#endif

/* high speed clock (input to event managers, adc) */
//#define HISPCLK_KHZ			(SYSCLKOUT_KHZ/((SysCtrlRegs.HISPCP.bit.HSPCLK == 0) ? 1 : (SysCtrlRegs.HISPCP.bit.HSPCLK*2)))
//#define HISPCLK_HZ			(HISPCLK_KHZ*1000L)

/* low speed clock (input to serial ports, etc.) */
//f_PLLSYSCLK = f_OSCCLK * (SYSPLLMULT.IMULT + SYSPLLMULT.FMULT) / SYSCLKDIVSEL.PLLSYSCLKDIV
//from technical reference manual,
//external clock = 20Mhz, so 20Mhz*(imult=20+ fmult=0) = 400Mz/(sysclkdivsel=2)
//= 200Mhz
//LSPCLKDIV is defualt /4 and is derived from SYSCLK (identical for each cpu to PLLSYSCLK)
//PLLSYSCLKDIV also default to /4
//IMULT = 7 bits, so 0-127, FMULT = 2 bits, so 0, .25, .5, .75,
//want IMULT = 20, FMULT = 0 if CLKDIV = 1, so then f_PLLSYSCLK = 10Mhz*20/1 = 200Mhz

//LSPCLKDIV shouldn't be changed, so then should still be default /4
//so 200Mhz/4 = 50Mhz
#define LSPCLK_KHZ (SYSCLKOUT_KHZ/((ClkCfgRegs.LOSPCP.bit.LSPCLKDIV == 0) ? 1 : (ClkCfgRegs.LOSPCP.bit.LSPCLKDIV*2)))

//#define LSPCLK_KHZ   50000
#define LSPCLK_HZ		(LSPCLK_KHZ*1000L)


/* NOTE!  You can think of these clock frequencies as "ticks per second" (HZ) or
   "ticks per millisecond (KHZ). */


/* COECSL function prototypes */
//void init_COECSL(void);
//#define nop() asm(" nop")


/* general structures/enumerations */
enum eQep_e {
	EQEP1,
	EQEP2,
	EQEP3
};
//enum gptimer_e {
//	GPTIMER1,
//	GPTIMER2,
//	GPTIMER3,
//	GPTIMER4
//};
//enum epwm {
//	EPWM1,
//	EPWM2,
//	EPWM3,
//	EPWM4,
//	EPWM5,
//	EPWM6,
//	EPWM3B,
//	EPWM4B
//};

/* COECSL includes */
#include <queue.h>
#include <buffer.h>
#include <smallprintf.h>

//#warn below include files should be added when functionality required

//#ifndef __LCD_H__
//	#include <lcd.h>
//#endif

//#include <RTDX_functions.h>
//#include <io.h>

#ifndef __F28377SSERIAL_H__
	#include <f28377sSerial.h>
#endif

//#ifndef __28335_ECAN_H__
//	#include <28335_ecan.h>
//#endif
//
//#include <28335_pwm.h>
//#include <28335_eQep.h>







#endif /* F28377SCOECSL_H_ */
