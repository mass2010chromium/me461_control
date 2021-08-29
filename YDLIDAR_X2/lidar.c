//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

// Only one of these can be uncommented at a time
//#define MATLAB
//#define SIMULINK

#ifdef MATLAB
void matlab_serialRX(serial_t *s, char data);
#endif
#ifdef SIMULINK
void simulink_serialRX(serial_t *s, char data);
#endif


#pragma DATA_SECTION(matlabLock, ".my_vars")
float matlabLock = 0;
float matlabLockshadow = 0;

#pragma DATA_SECTION(matlab_dist, ".my_arrs")
float matlab_dist[1000];

#pragma DATA_SECTION(matlab_angle, ".my_arrs")
float matlab_angle[1000];


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

void serialRXA(serial_t *s, char data);
void serialRXD(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t sampleSize = 0;


typedef struct data_point{
    double distance;
    uint32_t timestamp;
//    double startAngle;
//    double endAngle;
    double rawAngle;
    double cor_angle;
}data_pt;
data_pt pts[600];

typedef struct dis_time{
    double distance;
    uint32_t timestamp;
}dis_time;
dis_time pingpts[360];
dis_time pongpts[360];

uint32_t stamp45 = 0;
float dist45 = 0;

float anglesum = 0;
int16_t checkfullrevolution = 0;
int16_t pingpongflag = 1;
int16_t UseLIDARping = 0;
int16_t UseLIDARpong = 1;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;


#ifdef MATLAB
    init_serial(&SerialA,115200,matlab_serialRX);
#else
    #ifdef SIMULINK

        init_serial(&SerialA,115200,simulink_serialRX);
    #else
        init_serial(&SerialA,115200,serialRXA);
    #endif
#endif


//    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {

        if (UseLIDARping == 1) {
            UseLIDARping = 0;
            dist45 = pingpts[45].distance;
            stamp45 = pingpts[45].timestamp;
            UARTPrint = 1;
        }
        if (UseLIDARpong == 1) {
            UseLIDARpong = 0;
            dist45 = pongpts[45].distance;
            stamp45 = pongpts[45].timestamp;
            UARTPrint = 1;
        }

        if (UARTPrint == 1 ) {
#ifndef MATLAB
#ifndef SIMULINK
            serial_printf(&SerialA,"TimeStamp %ld Dist %.3f\r\n",stamp45,dist45);
#endif
#endif

            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
	
	
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	
	
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
	
	if ((CpuTimer2.InterruptCount % 50) == 0) {
		UARTPrint = 1;
	}
}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}
//----------------------next step: calculate the correction data---------------------------//
int16_t lidar_data[200];
int16_t lidar_count = 0;
int16_t state = 0; //read data
int16_t sample_size[100]; //sample size array
float start_angle1[100]; // start angle array
float start_angle2[100]; // for debug
float start_angle3[100];  //for debug
float end_angle1[100]; //end angle array
float angle_diff[100]; //array that keep track of angle diff

double end_angle;
double start_angle;
double cal_angle;

float angles[15][40];//record all the angles by calculating with the start/end angle
uint16_t angle_index = 0;
float anglescor[15][40]; //the corrected angles

float distance[15][40]; //distance array
//int16_t packetcount=-1; // count packet, since increase in the first time entering the loop, should set to -1
int16_t startangleLSB = 0; //lsb of start angle
int16_t endLSB = 0; //lsb of end angle
int16_t position = 0;//position of the
int16_t dis_state = 0;//state of distance(lsb or msb)
int16_t dis_count = 0; //count of distance number(should usually be 40)
uint16_t dLSB;//LSB of distance
uint16_t arrayIndex = 0; //index for filling the distance array
uint16_t anglearrayindex = 0; //index for filling the angle array
uint16_t sampleindex = 0;
float cor_end = 0; // make sure it is larger than the start angle
float delta_angle = 0; // cor_end - start
void serialRXD(serial_t *s, char data) {
    if(state == 0) { //check 0xaa
        if(data == 0xAA) {
            state = 1;
//            if (packetcount > 15) {
//                uint16_t i=0;
//                uint16_t j = 0;
//                for (i = 0; i < 15; i++) {
//                    for (j = 0; j < 40; j++) {
//                        matlab_angle[anglearrayindex] = angles[i][j];
//                        matlab_dist[anglearrayindex] = distance[i][j];
//                        anglearrayindex+=1;
//                    }
//                }
//                state = -1;
//            }
        } else {
            state = 0;
        }
    } else if (state == 1) {//check 0x55
        if(data == 0x55) {
            state = 2;
        } else {
            state = 0;
        }
    } else if (state == 2) {//check 0x0
        if (data == 0x0) {
            state = 3;
//            packetcount+=1;
        } else {
            state = 0;
        }
    } else if (state == 3){ //read sample size
//        if(packetcount < 100) {
//            sample_size[packetcount] = data;
            sampleSize = data;
//        }
        state = 4;
    } else if (state == 4) {//read starting angle lsb
        startangleLSB = data;
//        if(packetcount < 100) {
//            start_angle2[packetcount] = data;
//        }
        state = 5;
    } else if (state == 5) {//record starting angle
//        if(packetcount<10000) {
//            start_angle3[packetcount] = data;
            start_angle = ((((data<<8)| startangleLSB)>>1)&0x7FFF)/64.0;
//            start_angle1[packetcount] = start_angle;
//        }
        state = 6;
    } else if (state == 6) { //read end angle
//        if(packetcount<100) {
            endLSB = data;
//        }
        state = 7;
    } else if (state == 7) {//record end angle
        end_angle = ((((data<<8)| endLSB)>>1)&0x7FFF)/64.0;

        if (end_angle < start_angle) {
            cor_end = end_angle + 360;
        } else {
            cor_end = end_angle;
        }
        delta_angle = cor_end-start_angle;
//        if(packetcount<100) {
//            end_angle1[packetcount] = end_angle;
//            angle_diff[packetcount] = end_angle1[packetcount]-start_angle1[packetcount];
//        }

        state = 8;
    } else if (state == 8) {//record samples and ignore the check code
        if(position > 1) {
            if (dis_state == 0){
                dLSB = data;
                dis_state = 1;
            } else if (dis_state == 1){
//                if (packetcount < 15) {
                    float dist = (((uint16_t)data<<8 | dLSB))/4.0;
//                    distance[packetcount][dis_count%40] = dist;
                    pts[arrayIndex].distance = dist;
                    pts[arrayIndex].timestamp = numTimer0calls;
//                    pts[arrayIndex].startAngle = start_angle;
//                    pts[arrayIndex].endAngle = end_angle;

                    float raw_angle = (delta_angle)/(sampleSize - 1) * (sampleindex) +start_angle;
                    sampleindex = sampleindex+1;
                    if(sampleindex == sampleSize) {
                        sampleindex = 0;
                    }
                    pts[arrayIndex].rawAngle = raw_angle;
                    if(dist == 0){
                        cal_angle = raw_angle;
                    } else {
                        cal_angle = raw_angle + atan(21.8*(155.3-dist)/(155.3*dist))*57.296;
                    }
                    pts[arrayIndex].cor_angle = cal_angle;

                    if (pingpongflag == 1) {
                        pingpts[((int16_t)(cal_angle + 0.5))%360].distance = dist;
                        pingpts[((int16_t)(cal_angle + 0.5))%360].timestamp = numTimer0calls;
                    } else {
                        pongpts[((int16_t)(cal_angle + 0.5))%360].distance = dist;
                        pongpts[((int16_t)(cal_angle + 0.5))%360].timestamp = numTimer0calls;
                    }


                    dis_count += 1;
                    dis_state = 0;
                    if (arrayIndex < 599) {
                        arrayIndex += 1;
                    }

//                }
            }
        }
        position += 1;
        if(position >= sampleSize*2+2) { //the total number of bytes for check code(2) and sample data(80)

            if (checkfullrevolution == 0) {
                checkfullrevolution = 1;
                anglesum = delta_angle;
            } else {
                anglesum += delta_angle;
                if (anglesum > 360) {
                    checkfullrevolution = 0;
                    if (pingpongflag == 1){
                        UseLIDARping = 1;
                        pingpongflag = 0;
                    } else {
                        UseLIDARpong = 1;
                        pingpongflag = 1;
                    }
                }
            }
            position = 0;
            state = 0;//enter state 0 to read new packet
            GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
        }
        //-------------------------------generate the angles by the starting and ending angle-------------------------------------
//        for (angle_index = 0; angle_index < sample_size[packetcount]; angle_index++) {
//            if(sample_size[packetcount] == 0) {
//                angles[packetcount][angle_index] = 0;
//            } else {
//                float cal_angle = (fabs(end_angle1[packetcount] - start_angle1[packetcount]))/(sample_size[packetcount] - 1) * (angle_index) +start_angle1[packetcount];
//                angles[packetcount][angle_index] = cal_angle;
//            }
//        }
//        for (angle_index = 0; angle_index < sample_size[packetcount]; angle_index++) {
//            if (distance[packetcount][angle_index] == 0) {
//                anglescor[packetcount][angle_index] = angles[packetcount][angle_index];
//            } else {
//                anglescor[packetcount][angle_index] = angles[packetcount][angle_index] + atan(21.8*(155.3-distance[packetcount][angle_index])/(155.3*distance[packetcount][angle_index]))*57.296;
//            }
//        }
    }
}

#ifdef MATLAB
#define MAX_VAR_NUM 10

int UARTsensordatatimeouterror = 0; // Initialize timeout error count
int UARTtransmissionerror = 0;          // Initialize transmission error count

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

            UARTdatacollect = 0;        // amount of data collected in message set to 0
            UARTbeginnewdata = 1;       // flag to indicate we are collecting a message
            Main_memcount = 0;
            Main_i = 0;
        }
    } else {    // Filling data
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
            UARTbeginnewdata = 0;   // Reset the flag
            UARTdatacollect = 0;    // Reset the number of chars collected

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
                EchoSerialData(Main_memcount,Main_SendArray);   // Append header information to send data and transmit
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
                //      array(0) chars in little-endian, ... , array(memcount) chars in little-endian]
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
                //      little-endian received value)
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
#endif

#ifdef SIMULINK
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
long SIMU_Var3_toSIMU_32bit = 0;
long SIMU_Var4_toSIMU_32bit = 0;
long SIMU_Var5_toSIMU_32bit = 0;
long SIMU_Var6_toSIMU_32bit = 0;
long SIMU_Var7_toSIMU_32bit = 0;

long SIMU_Var1_toSIMU_16bit = 0;
long SIMU_Var2_toSIMU_16bit = 0;
long SIMU_Var3_toSIMU_16bit = 0;
long SIMU_Var4_toSIMU_16bit = 0;
long SIMU_Var5_toSIMU_16bit = 0;
long SIMU_Var6_toSIMU_16bit = 0;
long SIMU_Var7_toSIMU_16bit = 0;

int SIMU_beginnewdata = 0;
int SIMU_datacollect = 0;
int SIMU_Tranaction_Type = 0;
int SIMU_checkfirstcommandbyte = 0;

void simulink_serialRX(serial_t *s, char data) {
    if (!SIMU_beginnewdata) {// Only true if have not yet begun a message
        if (SIMU_checkfirstcommandbyte == 1) {
            if (0xFF == (unsigned char)data) {// Check for start 2 bytes command = 32767 becuase assuming command will stay between -10000 and 10000
                SIMU_checkfirstcommandbyte = 0;
            }
        } else {
            SIMU_checkfirstcommandbyte = 1;
            if (0x7F == (unsigned char)data) {// Check for start char

                SIMU_datacollect = 0;       // amount of data collected in message set to 0
                SIMU_beginnewdata = 1;      // flag to indicate we are collecting a message

                SIMU_Tranaction_Type = 2;

                // Coould Start ADC and then ADC interrupt will read ENCs also and then send
                // but that is for Simulink control
                // For Simulink data collection just send most current ADC and ENCs
                // Simulink Sample rate needs to be at least 500HZ but 200Hz probably better

//                SIMU_Var1_toSIMU_16bit = 1000*tilt_value;
//                SIMU_Var2_toSIMU_16bit = 1000*gyro_value;
//                SIMU_Var3_toSIMU_16bit = 1000*thetaR;
//                SIMU_Var4_toSIMU_16bit = 1000*vel_orig;
//                SIMU_Var5_toSIMU_16bit = 1000*u;
//                SIMU_Var6_toSIMU_16bit = 0;//1000*th_value;
//
//                SIMU_TXrawbytes[3] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[2] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[0] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
//
//                SIMU_TXrawbytes[7] = (char)((SIMU_Var4_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[6] = (char)(SIMU_Var4_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[5] = (char)((SIMU_Var3_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[4] = (char)(SIMU_Var3_toSIMU_16bit & 0xFF);
//
//                SIMU_TXrawbytes[11] = (char)((SIMU_Var6_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[10] = (char)(SIMU_Var6_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[9] = (char)((SIMU_Var5_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[8] = (char)(SIMU_Var5_toSIMU_16bit & 0xFF);


                SIMU_Var1_toSIMU_32bit = 10000*readEnc1();
                SIMU_Var2_toSIMU_32bit = 10000*readEnc2();
                SIMU_Var1_toSIMU_16bit = 0;//1000*tilt_value;
                SIMU_Var2_toSIMU_16bit = 0;//1000*gyro_value;
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

                serial_send(&SerialA,SIMU_TXrawbytes,12);

            }
        }
    } else {    // Filling data
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

                SIMU_beginnewdata = 0;  // Reset the flag
                SIMU_datacollect = 0;   // Reset the number of chars collected
                SIMU_Tranaction_Type = 0;
            }

        }
    }
}
#endif
