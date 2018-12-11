//###########################################################################
//
// FILE:   F2837xS_Spi.c
//
// TITLE:  F2837xS SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xS Support Library v140 $
// $Release Date: Fri Jan 16 14:01:54 CST 2015 $
// $Copyright: Copyright (C) 2014-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xS_device.h"     // F2837xS Headerfile Include File
#include "F2837xS_Examples.h"   // F2837xS Examples Include File

//---------------------------------------------------------------------------
// Example: InitSpiGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SPI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// For each SPI peripheral
// Only one GPIO pin should be enabled for SPISOMO operation.
// Only one GPIO pin should be enabled for SPISOMI operation.
// Only one GPIO pin should be enabled for SPICLK  operation.
// Only one GPIO pin should be enabled for SPISTE  operation.
// Comment out other unwanted lines.

void InitSpiGpio()
{
   InitSpiaGpio();
}

void InitSpiaGpio()
{

   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA) (using as regular gpio cs for now)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;  // Asynch input GPIO5 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;  // Asynch input GPIO3 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA) (using as regular gpio cs for now)

/* Configure SPI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
//  GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 2;  // Configure GPIO5 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
//  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;  // Configure GPIO3 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA (using as regular gpio cs for now)

    EDIS;
}


////////////////////////////////////////////////////////////////////////////
//Tori added for imu
void initSpibGpio(void)
{

	//MOSI: gpio60,
	//MISO: gpio64
	//clk: gpio58
	//not chip select: gpio59 (ste = slave transmit enable)

   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.

   GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;	//Enable pull-up on Gpio60 (SPIMOSIB)
   GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;	//Enable pull-up on gpio64 (spimisob)
   GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;	//Enable pull-up on gpio58 (spiclkb)
//   GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;	//Enable pull-up on Gpio59 (spisteb)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;	//Asynch input Gpio60 (spimosib)
   GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;	//Asynch input gpio64 (spimisob)
   GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;	//Asynch input gpio58 (spiclkb)
//   GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;	//Asynch input Gpio59 (spisteb)

/* Configure SPI-b pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 2;	//configure Gpio60 as spimosib
   GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 1;
   GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;	//configure gpio64 as spimisob
   GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;
   GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 2;	//configure gpio58 as spiclkb
   GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 1;

//   GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 6;	//configure gpio59 as spisteb

	EDIS;
}
////////////////////////////////////////////////////////////////////////////

//===========================================================================
// End of file.
//===========================================================================
