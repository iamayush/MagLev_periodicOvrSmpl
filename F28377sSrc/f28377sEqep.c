/* 28335_EQEP.C:  F28335 DSP EQEP peripherals interface.
 *
 *
 * QEP (Quadrature Encoder Pulse) interface:
 *    1) initialize the QEP circuit with init_QEP()
 *    2) call QEP_read() to read the current encoder position in radians
 *
 * WRITTEN BY: Paul Miller <pamiller@uiuc.edu>
 * UPDATED: 01/26/2005
 */

#include <f28377sCoecsl.h>
#include <f28377seQep.h>

eqep_t eqep1;
eqep_t eqep3;

/* init_EQEP(): initialize quadrature encoder pulse QEP
 *
 * parameters:
 *    eqep   = address pointer to QEP structure
 *    eq    = EQEP 1 or 2
 *    cpr   = encoder's rated counts per revolution
 *    init_rad_coord = initial radian coordinate for 0 encoder count
 *
 */
void init_EQEP(eqep_t *eqep, enum eQep_e eq, int32 cpr, int pol, float init_rad_coord)
{

	eqep->eq = eq;
	eqep->cpr = 4*cpr;
	eqep->init_rad_coord = init_rad_coord;
	eqep->res = TWOPI/(eqep->cpr);
	eqep->pol = pol;

	// select event manager
	switch (eq) {

	case EQEP1:

		// setup QEP1 pins for input
		EALLOW;
			/* Disable internal pull-up for the selected output pins
	   	   	   for reduced power consumption */
			GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10 (EQEP1A)
			GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11 (EQEP1B)

			GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 2;   // Qual every 6 samples
			GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 2;   // Qual every 6 samples

			/* Configure EQEP-3 pins using GPIO regs*/
			// This specifies which of the possible GPIO pins will be EQEP3 functional pins.
			// Comment out other unwanted lines.
			GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 1;	// Configure GPIO10 as EQEP1A
			GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;    // Configure GPIO10 as EQEP1A
			GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EQEP1B
			GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;    // Configure GPIO11 as EQEP1B

		EDIS;

		EQep1Regs.QEPCTL.bit.QPEN = 0;  // make sure qep reset
		//EQep1Regs.QEPCTL.bit.SWI = 1;   // Initialize Position counter to zero
		EQep1Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
		EQep1Regs.QPOSCTL.all = 0x0;   // Disable eQep Position Compare
		EQep1Regs.QCAPCTL.all = 0x0;  // Disable eQep Capture
		EQep1Regs.QEINT.all = 0x0;  // Disable all eQep interrupts
		EQep1Regs.QPOSMAX = 0xFFFFFFFF;
		EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
		EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

		break;

	case EQEP3:

		// setup QEP3 pins for input
		EALLOW;

			/* Disable internal pull-up for the selected output pins
		   	   for reduced power consumption */
			GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1;    // Disable pull-up on GPIO62 (EQEP3A)
			GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1;    // Disable pull-up on GPIO63 (EQEP3B)


			GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 2;   // Qual every 6 samples
			GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 2;   // Qual every 6 samples

			/* Configure EQEP-3 pins using GPIO regs*/
			// This specifies which of the possible GPIO pins will be EQEP3 functional pins.
			// Comment out other unwanted lines.
			GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = 1;   // Configure GPIO62 as EQEP3A
			GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;    // Configure GPIO62 as EQEP3A
			GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 1;   // Configure GPIO63 as EQEP3B
			GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;    // Configure GPIO63 as EQEP3B

		EDIS;
	
		EQep3Regs.QEPCTL.bit.QPEN = 0;  // make sure qep reset
		//EQep3Regs.QEPCTL.bit.SWI = 1;   // Initialize Position counter to zero
		EQep3Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
		EQep3Regs.QPOSCTL.all = 0x0;   // Disable eQep Position Compare
		EQep3Regs.QCAPCTL.all = 0x0;  // Disable eQep Capture
		EQep3Regs.QEINT.all = 0x0;  // Disable all eQep interrupts
		EQep3Regs.QPOSMAX = 0xFFFFFFFF;
		EQep3Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
		EQep3Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

		break;

	}
}

/* EQEP_read(): read QEP encoder's position in radians
 *
 * parameters:
 *    eqep   = address pointer to an initialized QEP structure
 *
 * return: encoder position in radians
 */
float EQEP_read(eqep_t *eqep)
{
	int32 raw = 0;

	// read raw value
	switch (eqep->eq) {
	case EQEP1:
		raw = EQep1Regs.QPOSCNT;
		break;
	case EQEP3:
		raw = EQep3Regs.QPOSCNT;
		break;
	}

	if (raw >= QEP_MAXVALUE/2) raw -= QEP_MAXVALUE;

	return ((eqep->pol)*(eqep->res)*raw + eqep->init_rad_coord);
}

long EQEP_readraw(eqep_t *eqep)
{
	int32 raw = 0;

	// read raw value
	switch (eqep->eq) {
	case EQEP1:
		raw = EQep1Regs.QPOSCNT;
		break;
	case EQEP3:
		raw = EQep3Regs.QPOSCNT;
		break;
	}

	if (raw >= QEP_MAXVALUE/2) raw -= QEP_MAXVALUE;

	return (raw);
}

// This function is called for you in main().
// It initializes EQEP1 and EQEP3 on the 28377S.  EQEP2 pins are used for something else.
// EQEP1 is ENC1,  EQEP3 is ENC2
void init_EQEPs(void) {
	init_EQEP(&eqep1, EQEP1, 1000, -1, 0.0);
	init_EQEP(&eqep3, EQEP3, 1000, -1, PI);
	EQep1Regs.QPOSCNT = 0;
	EQep3Regs.QPOSCNT = 0;
}

// This function returns the radian value of Encoder 1.  Normally attached to the motor
//  Example code
//  float enc1 = 0;
//
//	enc1 = readEnc1();
//
float readEnc1(void) {
	return(EQEP_read(&eqep1));
}

// This function returns the radian value of Encoder 2.  Normally attached to the pendulum
//  Example code
//  float enc2 = 0;
//
//	enc2 = readEnc2();
//
float readEnc2(void) {
	return(EQEP_read(&eqep3));
}
