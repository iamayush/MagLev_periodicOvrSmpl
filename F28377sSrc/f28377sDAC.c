/* 28335_EPWM3A.C:  F28377S DSP EPWM peripherals interface.
 *
 *
 */

#include <f28377sCoecsl.h>
#include <f28377sDAC.h>


// this function has already been called for you in the main() function.  
// It sets up Pin A0 and A1 as DAC outputs.  
void initDACs(void)
{
	EALLOW;
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;		//enable dacA output-->uses A0
	DacaRegs.DACCTL.bit.LOADMODE = 0;		//load on next sysclk
	DacaRegs.DACCTL.bit.DACREFSEL = 1;		//use ADC VREF as reference voltage

	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;		//enable dacB output-->uses A1
	DacbRegs.DACCTL.bit.LOADMODE = 0;		//load on next sysclk
	DacbRegs.DACCTL.bit.DACREFSEL = 1;		//use ADC VREF as reference voltage
	EDIS;
}

// This function sets DACA to the voltage between -10V and 10V passed to this function.
// If a larger value than 10V or -10V is passed it is saturated at 10V.    
//  Example code
//  float myu = 4.25;
//
//	setDAC1(myu);   // DAC1 will now output 4.25 Volts
void setDAC1(float dacouta0) {
	
	if (dacouta0 >  10) dacouta0 =  10;
	if (dacouta0 < -10) dacouta0 = -10;

	DacaRegs.DACVALS.bit.DACVALS = (int)(((-dacouta0+10.0)/(20.0/3.0))*4095.0/3.0);

}

// This function sets DACB to the voltage between -10V and 10V passed to this function.
// If a larger value than 10V or -10V is passed it is saturated at 10V.    
//  Example code
//  float myu = -4.25;
//
//	setDAC2(myu);   // DAC2 will now output -4.25 Volts
void setDAC2(float dacouta1) {
	
	if (dacouta1 >  10) dacouta1 =  10;
	if (dacouta1 < -10) dacouta1 = -10;

	DacbRegs.DACVALS.bit.DACVALS = (int)(((-dacouta1+10.0)/(20.0/3.0))*4095.0/3.0);
}


