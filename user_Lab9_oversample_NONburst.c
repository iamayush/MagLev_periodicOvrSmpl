#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c28/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "f28377sCoecsl.h"
#include "f28377sEqep.h"
#include "f28377sSerial.h"
#include "f28377sEPWM3A.h"
#include "f28377sDAC.h"
#include "f28377sADC.h"
#include "lcd.h"

/***************************************************************************/
/***************************************************************************/
extern const Swi_Handle controlSWI; // swi for control input calc

void matlab_serialRX(serial_t *s, char data);
void simulink_serialRX(serial_t *s, char data);

#ifdef _FLASH
// These are defined by the linker (see device linker command file)
extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadSize;
extern unsigned int RamfuncsRunStart;
#endif

int adcb2result = 0;
long double ADCB2raw = 0;

#define NUM_SEND_QUEUES 120
#define MAX_SEND_LENGTH 1600
#define MAX_VAR_NUM 10

#pragma DATA_SECTION(matlabLock, ".my_vars")
float matlabLock = 0;
float matlabLockshadow = 0;

// UART 1 GLOBAL VARIABLES
int	UARTsensordatatimeouterror = 0;	// Initialize timeout error count
int	UARTtransmissionerror = 0;	        // Initialize transmission error count

int UARTbeginnewdata = 0;
int UARTdatacollect = 0;
char UARTMessageArray[101];
int UARTreceivelength = 0;

char Main_sendingarray = 0;  // Flag to Stop terminal prints when using matlab commands
//  Only way to clear this flag

union mem_add {
    float f;
    long i;
    char c[2];
}memloc;

union ptrmem_add {
    float* f;
    long* i;
    char c[2];
}ptrmemloc;

long* Main_address[MAX_VAR_NUM];
float Main_value[MAX_VAR_NUM];
char Main_SendArray[128];
char Main_SendArray2[128];
float Main_tempf=0;
int Main_i = 0;
int Main_j = 0;
int Main_memcount = 0;

char SIMU_databyte1 = 0;
int SIMU_Var1_fromSIMU_16bit = 0;
int SIMU_Var2_fromSIMU_16bit = 0;
int SIMU_Var3_fromSIMU_16bit = 0;
int SIMU_Var4_fromSIMU_16bit = 0;
int SIMU_Var5_fromSIMU_16bit = 0;
int SIMU_Var6_fromSIMU_16bit = 0;
int SIMU_Var7_fromSIMU_16bit = 0;
char SIMU_TXrawbytes[12];
long SIMU_Var1_toSIMU_32bit = 0;
long SIMU_Var2_toSIMU_32bit = 0;
int SIMU_Var1_toSIMU_16bit = 0;
int SIMU_Var2_toSIMU_16bit = 0;
int SIMU_beginnewdata = 0;
int SIMU_datacollect = 0;
int SIMU_Tranaction_Type = 0;
int SIMU_checkfirstcommandbyte = 0;

//Variables
long timer = 0; // keeps track of time in ms
char adc_count = 0; // keeps track of # times ADC is sampled
//reference signal generation
long double amp1=2; // reference high level in volts
long double amp2=1; // reference low level in volts
long double reference = 0; // stores reference value for each ms
long double voltSum = 0; // sum of all 16 ADC samples

// controls vars
long double u =0; // control effort
long double ADCvoltsb2 = 0; // Hall Effect sensors output in volts
long double vel_old = 0;
long double vel = 0; // derivative of sensor output
long double posn = 0; // sensor output
long double posn_old = 0;
long double error = 0; // error between desired posn and actual posn
long double error_old = 0;
long double integral = 0;
long double integral_old=0;

// PID Gains found thru MATLAB
long double Kp = 5.2924;
long double Kd = 0.1572;
long double Ki = 93.9734;

// Filter for slower trajectory
long double numfil[7]= {    9.4204523525420674e-13L,
                            5.6522714115252405e-12L,
                            1.4130678528813101e-11L,
                            1.8840904705084135e-11L,
                            1.4130678528813101e-11L,
                            5.6522714115252405e-12L,
                            9.4204523525420674e-13L};
long double denfil[7]= { 1.0000000000000000e+00L,
                         -5.8811881188118820e+00L,
                         1.4411822370355853e+01L,
                         -1.8835252998880922e+01L,
                         1.3846708268979292e+01L,
                         -5.4290064104116844e+00L,
                         8.8691688882963160e-01L};

long double r[7] = {0,0,0,0,0,0,0}; // stores reference values (square wave)
long double fr[7] = {0,0,0,0,0,0,0}; // stores filtered slower traj

// Generates Square wave
void ref(void){
    if(timer < 5000){
        reference = amp1;
    }
    else if((timer >= 5000)&&(timer < 10000)){
        reference = amp2;
    }
    else{
        reference = amp1;
        timer = 0;
    }
}

// Generates slower traj
void myfilter(void)
{
    r[6] = reference;
    fr[6]=-denfil[1]*fr[5]-denfil[2]*fr[4]-denfil[3]*fr[3]-denfil[4]*fr[2]-denfil[5]*fr[1]-denfil[6]*fr[0];
    fr[6]+=numfil[0]*r[6]+numfil[1]*r[5]+numfil[2]*r[4]+numfil[3]*r[3]+numfil[4]*r[2]+numfil[5]*r[1]+numfil[6]*r[0];

    int i;
    for(i =0; i<6; i++){
        fr[i]=fr[i+1];
        r[i]=r[i+1];
    }
}

//// Runs every 1 ms
//void myclockfunc(void)
//{
//    AdcbRegs.ADCSOCPRICTL.bit.RRPOINTER = 0x10;
//    AdcbRegs.ADCSOCFRC1.all |= 0xFFFF;  // now starting all 16 ADCB channels
//}

void ADChwifunc(void)
{
    adcb2result = AdcbResultRegs.ADCRESULT0;
    voltSum += (3.0*adcb2result/4095.0);
    adc_count++;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clearing adc flag

    if (1 == adc_count)
        ADCB2raw = voltSum;
    else if(16 == adc_count){
        // Post SWI controller()
        Swi_post(controlSWI);
    }
}

void controller(void){
    long double avg_volts = voltSum; // local copy of voltSum

    // Reset ADC cycle
    adc_count = 0;
    voltSum = 0;

    avg_volts /= 16.0; // calculating averaged ADC volts

    timer++;
    ref();
    myfilter();

    posn = avg_volts;
    vel = 0.6*vel_old + 400*(posn - posn_old); // derivative computed using approximation s = 500s/(s+500)
    error = fr[6] - posn;
    // error = reference - posn; // for square wave


    integral = integral_old + 0.001*(error + error_old)/2.0;
    u = Kp*error - Kd*vel + Ki*integral; // control law

    //saturation and anti-windup
    if((u < -10)){
        u = -10;
        integral = integral_old;
    }
    if((u > 10)){
        u = 10;
        integral = integral_old;
    }

    setEPWM8A(u); // sending control effort to PWM channel

    // storing old vals
    vel_old=vel;
    posn_old = posn;
    error_old = error;
    integral_old = integral;
}

void main(void)
{
    // Copy time critical code and Flash setup code to RAM
    // This includes InitFlash(), Flash API functions and any functions that are
    // assigned to ramfuncs section.
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the device .cmd file.
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif


    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
#ifdef _FLASH
    InitFlash_Bank0();
    InitFlash_Bank1();
#endif
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xS_SysCtrl.c file.
    InitSysCtrl();

    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xS_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    InitGpio();

    init_EQEPs();

    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);						//toggle led setup
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);		//toggle led setup
    GpioDataRegs.GPASET.bit.GPIO12 = 1;

    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);						//toggle led setup
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_PUSHPULL);		//toggle led setup
    GpioDataRegs.GPASET.bit.GPIO13 = 1;

    //LEDS
    //LED1  GPIO64
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO64 = 1;
    //LED2  GPIO91
    GPIO_SetupPinMux(91, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO91 = 1;
    //LED3  GPIO92
    GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO92 = 1;
    //LED4  GPIO99
    GPIO_SetupPinMux(99, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(99, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO99 = 1;

    //SWITCHES
    //SW1	GPIO41
    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(41, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
    //SW2	GPIO66
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
    //SW3	GPIO73
    GPIO_SetupPinMux(73, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(73, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
    //SW4	GPIO78
    GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(78, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);

    InitEPwm3Gpio();
    InitEPwm8Gpio();

    init_serial(&SerialA,115200,matlab_serialRX);

    init_serial(&SerialB,115200,simulink_serialRX);

    init_serial(&SerialC,19200,NULL); // For Text LCD

    init_lcd(90);

    //Configure the ADCs and power them up
    ConfigureADC();

    //Setup the ADCs for software conversions
    SetupADCSoftware();

    initEPwm3A();
    initEPwm8();
    setEPWM3A(0);
    setEPWM8A(0);
    initDACs();

    // ************ your Init code here  *******************


    UART_printfLine(1, "Digital Control");
    UART_printfLine(2, "of Dynamic Systems");

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_FREEZE; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0000; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 6; // divide by 64  50Mhz divide by 64 is 781250 Hz
    EPwm5Regs.TBPRD = 49;  // (1/16)ms sample actually 48.83 is (1/16)ms.
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; //unfreeze, and enter up count mode
    EDIS;


    // ************ End init code *************************

    // !!!!!!!!!!!!!!!!!  NO CODE below here in Main() !!!!!!!!!!!!!!!!!!!!!
    // !!!!!!!!!!!!!!!!!  All inits above !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // Disable CPU interrupts
    DINT;
    // Clear all CPU interrupt flags:
    IFR = 0x0000;
    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP8 | PIEACK_GROUP9);  // ACKnowledge any SCI interrupt requests

    BIOS_start();

}


/*********************/
//the code below is used to transmit data to MATLAB
void EchoSerialData(int memcount,char *buffer) {


    char sendmsg[256];
    int i;

    sendmsg[0] = 0x2A; // *
    sendmsg[2] = '0'; // 0
    for (i=3;i<(memcount*8+3);i++) {
        sendmsg[i] = buffer[i-3];
    }
    sendmsg[1] = i;
    serial_send(&SerialA, sendmsg, i);


}

void matlab_serialRX(serial_t *s, char data) {
    if (!UARTbeginnewdata) {// Only TRUE if have not yet begun a message
        if (42 == (unsigned char)data) {// Check for start char

            UARTdatacollect = 0;		// amount of data collected in message set to 0
            UARTbeginnewdata = 1;		// flag to indicate we are collecting a message
            Main_memcount = 0;
            Main_i = 0;
        }
    } else {	// Filling data
        if (0 == UARTdatacollect){
            UARTreceivelength = ((int)data)-1; // set receive length to value of char after start char
            UARTdatacollect++;
        }else if (UARTdatacollect < UARTreceivelength){
            UARTMessageArray[UARTdatacollect-1] = (char) data;
            // If sending out float value(s), save input memory locations and values at those addresses
            if (('0' == UARTMessageArray[0]) &&  (UARTdatacollect > 1)){

                if (Main_i == 0) {
                    ptrmemloc.c[1] = ((UARTMessageArray[UARTdatacollect-1] & 0xFF) << 8);
                }
                if (Main_i == 1) {
                    ptrmemloc.c[1] |= (UARTMessageArray[UARTdatacollect-1] & 0xFF);
                }
                if (Main_i == 2) {
                    ptrmemloc.c[0] = ((UARTMessageArray[UARTdatacollect-1] & 0xFF) << 8);
                }
                if (3 == Main_i){
                    ptrmemloc.c[0] |= (UARTMessageArray[UARTdatacollect-1] & 0xFF);

                    Main_address[Main_memcount]=ptrmemloc.i;
                    Main_value[Main_memcount]=*ptrmemloc.f;

                    Main_i = 0;
                    Main_memcount++;
                }else{
                    Main_i++;
                }
            }
            UARTdatacollect++;
        }
        if (UARTdatacollect == UARTreceivelength){  // If input receive length is reached
            UARTbeginnewdata = 0;	// Reset the flag
            UARTdatacollect = 0;	// Reset the number of chars collected

            // Case '0' : Sending data in endian format (big-endian address, big-endian value)
            if ('0' == UARTMessageArray[0]){
                for (Main_i = 0;Main_i<Main_memcount;Main_i++){
                    ptrmemloc.i=Main_address[Main_i];
                    Main_SendArray[0+8*Main_i]=((ptrmemloc.c[1]>>8)&0xFF);
                    Main_SendArray[1+8*Main_i]=ptrmemloc.c[1]&0xFF;
                    Main_SendArray[2+8*Main_i]=((ptrmemloc.c[0]>>8)&0xFF);
                    Main_SendArray[3+8*Main_i]=ptrmemloc.c[0]&0xFF;
                    memloc.f=Main_value[Main_i];
                    Main_SendArray[4+8*Main_i]=((memloc.c[1]>>8)&0xFF);
                    Main_SendArray[5+8*Main_i]=memloc.c[1]&0xFF;
                    Main_SendArray[6+8*Main_i]=((memloc.c[0]>>8)&0xFF);
                    Main_SendArray[7+8*Main_i]=memloc.c[0]&0xFF;
                }
                EchoSerialData(Main_memcount,Main_SendArray);	// Append header information to send data and transmit
                // Case '1' : Writing float value to memory address (big-endian received address / value)
            }else if ('1' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[5+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[6+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[7+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[8+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;


                }

                matlabLockshadow = matlabLock;
                // Case '2' : Sending array data in following format [char 1,char2,char3,...]
                // [*,3+input length of array,3 (code for array receiving in Matlab),...
                // 		array(0) chars in little-endian, ... , array(memcount) chars in little-endian]
            }else if ('2' == UARTMessageArray[0]){
                Main_sendingarray = 1;
                matlabLock = 1.0;
                matlabLockshadow = matlabLock;
                memloc.c[1] = NULL;
                memloc.c[0] = ((UARTMessageArray[5]&0xFF)<<8);
                memloc.c[0] |= (UARTMessageArray[6]&0xFF);
                Main_memcount = memloc.i;
                ptrmemloc.c[1] = ((UARTMessageArray[1]&0xFF)<<8);
                ptrmemloc.c[1] |= (UARTMessageArray[2]&0xFF);
                ptrmemloc.c[0] = ((UARTMessageArray[3]&0xFF)<<8);
                ptrmemloc.c[0] |= (UARTMessageArray[4]&0xFF);
                Main_SendArray[0]='*';
                Main_SendArray[1]=3+Main_memcount;
                Main_SendArray[2]='3';

                serial_send(&SerialA, Main_SendArray, 3);

                for (Main_i = 0; Main_i < Main_memcount;Main_i++){
                    Main_tempf = *ptrmemloc.f;
                    memloc.f = Main_tempf;
                    Main_SendArray2[0+Main_j*4] =  (memloc.c[0]&0xFF);
                    Main_SendArray2[1+Main_j*4] =  ((memloc.c[0]>>8)&0xFF);
                    Main_SendArray2[2+Main_j*4] =  (memloc.c[1]&0xFF);
                    Main_SendArray2[3+Main_j*4] =  ((memloc.c[1]>>8)&0xFF);
                    memloc.c[1] = ptrmemloc.c[1];
                    memloc.c[0] = ptrmemloc.c[0];
                    memloc.i+=2;  // was plus 4
                    ptrmemloc.c[1]=memloc.c[1];
                    ptrmemloc.c[0]=memloc.c[0];
                    Main_j++;
                    if (32 == Main_j){
                        memcpy(Main_SendArray,Main_SendArray2,128);
                        serial_send(&SerialA, Main_SendArray, 128);
                        Main_j = 0;
                    }
                }
                if (Main_j != 0){
                    serial_send(&SerialA, Main_SendArray2, (Main_memcount%32)*4);
                    Main_j = 0;
                }
                Main_sendingarray = 0;
                // Case '3' : Write float value to memory address (big-endian received address,
                //		little-endian received value)
            }else if ('3' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[8+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[7+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[6+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[5+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;

                }

                matlabLockshadow = matlabLock;
            }
        }
    }
}


extern eqep_t eqep1;
extern eqep_t eqep3;
void simulink_serialRX(serial_t *s, char data) {
    //	if (savenumbytes < 400) {  // Just for Debug
    //		savebytes[savenumbytes] = data;
    //		savenumbytes++;
    //	}
    if (!SIMU_beginnewdata) {// Only true if have not yet begun a message
        if (SIMU_checkfirstcommandbyte == 1) {
            if (0xFF == (unsigned char)data) {// Check for start 2 bytes command = 32767 becuase assuming command will stay between -10000 and 10000
                SIMU_checkfirstcommandbyte = 0;
            }
        } else {
            SIMU_checkfirstcommandbyte = 1;
            if (0x7F == (unsigned char)data) {// Check for start char

                SIMU_datacollect = 0;		// amount of data collected in message set to 0
                SIMU_beginnewdata = 1;		// flag to indicate we are collecting a message

                SIMU_Tranaction_Type = 2;

                // Coould Start ADC and then ADC interrupt will read ENCs also and then send
                // but that is for Simulink control
                // For Simulink data collection just send most current ADC and ENCs
                // Simulink Sample rate needs to be at least 500HZ but 200Hz probably better
                SIMU_Var1_toSIMU_32bit = 10000*posn;
                SIMU_Var2_toSIMU_32bit = 10000*ADCB2raw;
                //SIMU_Var2_toSIMU_32bit = 10000*fr[6];

                SIMU_Var1_toSIMU_16bit = 0;
                SIMU_Var2_toSIMU_16bit = 0;

                SIMU_TXrawbytes[3] = (char)((SIMU_Var1_toSIMU_32bit >> 24) & 0xFF);
                SIMU_TXrawbytes[2] = (char)((SIMU_Var1_toSIMU_32bit >> 16) & 0xFF);
                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_32bit >> 8) & 0xFF);
                SIMU_TXrawbytes[0] = (char)((SIMU_Var1_toSIMU_32bit) & 0xFF);

                SIMU_TXrawbytes[7] = (char)((SIMU_Var2_toSIMU_32bit >> 24) & 0xFF);
                SIMU_TXrawbytes[6] = (char)((SIMU_Var2_toSIMU_32bit >> 16) & 0xFF);
                SIMU_TXrawbytes[5] = (char)((SIMU_Var2_toSIMU_32bit >> 8) & 0xFF);
                SIMU_TXrawbytes[4] = (char)((SIMU_Var2_toSIMU_32bit) & 0xFF);

                SIMU_TXrawbytes[8] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[9] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[10] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[11] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);

                serial_send(&SerialB,SIMU_TXrawbytes,12);

            }
        }
    } else {	// Filling data
        if (SIMU_Tranaction_Type == 2) {
            if (SIMU_datacollect == 0){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 1){

                SIMU_Var1_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_datacollect++;
            } else if (SIMU_datacollect == 2){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 3){

                SIMU_Var2_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_datacollect++;
            } else if (SIMU_datacollect == 4){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 5){

                SIMU_Var3_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 6){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 7){

                SIMU_Var4_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 8){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 9){

                SIMU_Var5_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 10) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 11) {
                SIMU_Var6_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 12) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 13) {
                SIMU_Var7_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_beginnewdata = 0;	// Reset the flag
                SIMU_datacollect = 0;	// Reset the number of chars collected
                SIMU_Tranaction_Type = 0;
            }

        }
    }
}
