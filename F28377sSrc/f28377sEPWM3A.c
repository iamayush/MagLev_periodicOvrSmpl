/* 28335_EPWM3A.C:  F28377S DSP EPWM peripherals interface.
 *
 *
 */

#include <f28377sCoecsl.h>
#include <f28377sePWM3A.h>


// this function has already been called for you in the main() function.  
// It sets up PWM3A with a 20KHz carrier frequency PWM signal.  
void initEPwm3A(void)
{
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;		//set epwm3 to upcount mode
	EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
	EPwm3Regs.TBPRD = 2500; //set epwm3 counter  20KHz
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
	EPwm3Regs.TBCTR = 0x0000;                  // Clear counter

	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;		//clear when counter = compareA
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;			//set when timer is 0
}

// This function sets PWM3A duty cycle given the float value between -10 and 10.  
// Where 
// -10 equates to 0% duty cycle
//   0 equates to 50% duty cycle
//  10 equates to 100% duty cycle
//  so for example if you pass 5.0 to this function EPWM3A will be set to 75% duty cycle
//  Example code
//  float myu = 0;
//  float Kpgain = 4.5;
//  float error = 0;
//
//	myu = Kpgain*error;
//
//	setEPWM3A(myu);



void initEPwm8(void)
{
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      //set epwm3 to upcount mode
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm8Regs.TBPRD = 2500; //set epwm3 counter  20KHz
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                  // Clear counter

    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0

    EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;
}

void setEPWM3A(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm3Regs.CMPA.bit.CMPA = (int)pwmVal;

}




void setEPWM8A(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm8Regs.CMPA.bit.CMPA = (int)pwmVal;

}

void setEPWM8B(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm8Regs.CMPB.bit.CMPB = (int)pwmVal;
}
