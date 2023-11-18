// RSLK Self Test via UART

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0=0;             // Timer A3 first edge, P10.4
uint32_t Done0=0;              // set each rising

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2=0;             // Timer A3 first edge, P8.2
uint32_t Done2=0;              // set each rising

volatile uint8_t bumpState;
volatile uint8_t status;
int volatile speed = 3000;
int bumpAct = 0;
//char message[20];

volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;
volatile uint32_t nr, nc, nl;

//Prepare Hardware for Testing
void RSLK_Reset(void){
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    //Own code starts here
    Motor_Init();
    IRSensor_Init();
    EnableInterrupts();
}

//samples data from various sensors, filters the data and sets a flag to indicate that new sensor data is available
void SensorRead_ISR(void)   //code from Lab4_ADCmain.c
{  // runs at 2000 Hz
    uint32_t raw17, raw12, raw16;
    P1OUT ^= 0x01;         // profile, used to toggle the state of the LSB of P1OUT (for profiling and debugging)
    P1OUT ^= 0x01;         // profile
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    nr = LPF_Calc(raw17);  // right is channel 17 P9.0
    nc = LPF_Calc2(raw12);  // center is channel 12, P4.1
    nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
    ADCflag = 1;           // semaphore
    P1OUT ^= 0x01;         // profile
}

//initializes the infrared sensors and prepares system to read data from the sensors. Also initializes UART communication
void IRSensor_Init(void)    //code from Lab4_ADCmain.c
{
    uint32_t raw12, raw16, raw17;
    int32_t n; uint32_t s;
    Clock_Init48MHz();  //SMCLK=12Mhz
    ADCflag = 0;
    s = 256; // replace with your choice
    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
    LPF_Init(raw17,s);     // P9.0/channel 17
    LPF_Init2(raw12,s);     // P4.1/channel 12
    LPF_Init3(raw16,s);     // P9.1/channel 16
    UART0_Init();          // initialize UART0 115,200 baud rate
    LaunchPad_Init();
    TimerA1_Init(&SensorRead_ISR,250);    // 2000 Hz sampling
}

volatile uint8_t CollisionData, CollisionFlag;

//called when collision is detected. Stops motor, processes the collision data (which bumpers are activated?).
void HandleCollision(uint8_t bumpSensor){
    Motor_Stop();
    CollisionData = bumpSensor;
    CollisionFlag = 1;
    P4->IFG &= ~0xED;   //clear interrupt flags in the Port 4 Interrupt Flag Register

    //Check individual bits and print messages
    for(bumpAct; bumpAct < 6; bumpAct++){
        int bit = (CollisionData >> bumpAct) &1;
        if(bit == 0){
            //char message[20];
            //sprintf(message, "Bumper %d activated\n", bumpAct);
            //UART0_OutString(message);
        }
    }
}

uint8_t ConvertCollisionData(uint8_t data){
    return data&0x3f;
}

//measure time intervals using Timer A3. Capturing the time when an edge occurs.
void PeriodMeasure0(uint16_t time){
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                    // setup for next
  Done0++;
}

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                    // setup for next
  Done2++;
}

// RSLK Self-Test
// Sample program of how the text based menu can be designed.
// Only one entry (RSLK_Reset) is coded in the switch case. Fill up with other menu entries required for Lab5 assessment.
// Init function to various peripherals are commented off.  For reference only. Not the complete list.


int main(void) {
  uint32_t cmd=0xDEAD, menu=0;

  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  //Motor_Stop();
  LaunchPad_Init();
  //Bump_Init();
  //Bumper_Init();
  BumpInt_Init(&HandleCollision);
  //IRSensor_Init();
  //Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF); //CR for \r and LF for \n (carriage return and new line)
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] Control Robot"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[7] Obstacle Avoidance"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      switch(cmd){
          case 0:   //RSLK Reset
              RSLK_Reset();
              menu =1;
              cmd=0xDEAD;
              break;

          case 1:   //Motor Test
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Selected: Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Control Instructions:"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[f] forward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[b] backward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[l] left-turn (right wheel only)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[r] right-turn (left wheel only)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

              uint32_t motorMode =EUSCIA0_InChar();
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              switch(motorMode){
                case 'f':
                    Motor_Forward(3000,3000);
                    Clock_Delay1ms(1000);
                    Motor_Stop();
                    break;
                case 'b':
                    Motor_Backward(3000,3000);
                    Clock_Delay1ms(1000);
                    Motor_Stop();
                    break;
                case 'l':
                    Motor_Left(3000,0);
                    Clock_Delay1ms(1000);
                    Motor_Stop();
                    break;
                case 'r':
                    Motor_Right(0,3000);
                    Clock_Delay1ms(1000);
                    Motor_Stop();
                    break;
                default:
                    break;
              }
              menu = 1;
              cmd=0xDEAD;
              break;

          case 2:   //IR Sensor Test
              //Code given in Lab4_ADCmain.c
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Selected: IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

              int32_t loopCounter;  //for inner loop: collects IR sensor data
              Clock_Init48MHz();

              EnableInterrupts();
              for(int i = 0; i<20; i++) //make 20 IR readings
              {
              for(loopCounter=0; loopCounter < 2000; loopCounter++){    //duration specified bz loopCounter, wait for ADCflag to become non-zero (indication that it has processed the data)
                    while(ADCflag == 0){};
                    ADCflag = 0;
                  }
              UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");      //LeftConvert converts the raw sensor data into distance measurements in mm
              UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");    //UART0_OutDec5 displays the IR sensor readings on the terminal
              UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");
              }
              menu = 1;
              cmd=0xDEAD;
              break;

          case 3:   //Bumper Test

              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Selected: Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Do you want the bump switches to cause an interrupt and the motor to stop after a bump switch has been activated?"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[y] yes (track one bump switch activation at a time)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[n] no (track several bump switch activations simultaneously)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              uint32_t volatile answer = EUSCIA0_InChar();
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              if(answer != 'y' && answer != 'n'){
                  EUSCIA0_OutString("Your selection must either be [y] or [n]!"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  break;
              }

              CollisionData = 0xED;
              CollisionFlag = 0;

               while(1){
                  Motor_Forward(3000,3000);
                  CollisionFlag = 0;
                  WaitForInterrupt();
                  if(CollisionFlag==1)
                  {
                      uint8_t tmp = CollisionData & 0x3F;
                      for(int i = 0; i<6; i++){
                          int digit = tmp & 1;
                          tmp=tmp>>1;
                          if(digit == 1){
                              UART0_OutString("1");
                          }else{
                              UART0_OutString("0");
                          }
                      }
                      //sprintf(message, "Bumper %d activated\n", bumpAct);
                      //UART0_OutString(message);


                      //UART0_OutUDec5(ConvertCollisionData(CollisionData));
                      int8_t bumpAbs = ConvertCollisionData(CollisionData);
                      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  if(answer == 'y'){
                      switch(bumpAbs){
                          case 31:
                              Clock_Delay1ms(100);
                              EUSCIA0_OutString("1st Bumper Activated! Object to the far left."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              break;
                          case 47:
                              Clock_Delay1ms(100);
                              EUSCIA0_OutString("2nd Bumper Activated! Object to the left."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              break;
                          case 55:
                              Clock_Delay1ms(100);
                              EUSCIA0_OutString("3rd Bumper Activated! Object slightly to the left."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              break;
                          case 59:
                              Clock_Delay1ms(100);
                              EUSCIA0_OutString("4th Bumper Activated! Object slightly to the right."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              break;
                          case 61:
                              Clock_Delay1ms(100);
                              EUSCIA0_OutString("5th Bumper Activated! Object to the right."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              break;
                          case 62:
                              Clock_Delay1ms(100);
                              EUSCIA0_OutString("6th Bumper Activated! Object to the far right."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              break;
                          default:
                              break;
                      }
                  }
                      if(answer == 'y'){
                          break;
                      }
                  }
              }
              menu = 1;
              cmd=0xDEAD;
              break;


          case 4:   //Reflectance Sensor Test
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Selected: Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              uint8_t refData = 0;  //used to store data read from the reflectance sensors
              for(int i = 0; i<30; i++){    //repeat for 30 readings
                  refData = Reflectance_Read(1000);
                  for(int i = 0; i<8;i++){          //binary format of refData (8 iterations for 8 bits)
                      EUSCIA0_OutUDec(refData%2);   //send 0 or 1 to the terminal (LSB of refData)
                      EUSCIA0_OutString("-");
                      refData = refData >> 1;
                  }
                  EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  Clock_Delay1ms(500);  //repeat reading every half second
              }
              menu = 1;
              cmd=0xDEAD;
              break;

          case 5:   //Tachometer Test
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Selected: Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

              uint32_t main_count=0;                                    //to keep track of the number of iterations of the loop
              TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);     //initializes Timer A3 capture module
                  Clock_Delay1ms(500);
                  Motor_Forward(3000,3000);
              while(LaunchPad_Input()==0){
                    WaitForInterrupt();
                    main_count++;                                       //increment for every iteration
                    if(main_count%50000){                                //
                        UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
                    }       //give the values to UART
                  }
                  Motor_Stop();

              //RSLK_Reset();
              menu = 1;
              cmd=0xDEAD;
              break;

          case 6:   //Advanced Topic1: Remote Control Robot
              //options: forward, backwards, left, right, stop, accelerate, decelerate, switch LED's on?
                RSLK_Reset();

                EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("Selected: Control Robot"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("Control Instructions:"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[w] forward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[x] backward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[a] left-turn (right wheel only)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[d] right-turn (left wheel only)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[s] stop motor"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[q] accelerate"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("[z] decelerate"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

                while(LaunchPad_Input()==0){
                    uint32_t motorModeC =EUSCIA0_InChar();
                    EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                    switch(motorModeC){
                      case 'w': //forwards
                          Motor_Forward(speed,speed);
                          //Clock_Delay1ms(1000);
                          break;
                      case 'a': //turn left
                          Motor_Left(speed,0);
                          //Clock_Delay1ms(1000);
                          break;
                      case 'd': //turn right
                          Motor_Right(0,speed);
                          //Clock_Delay1ms(1000);
                          break;
                      case 'x': //backwards
                          Motor_Backward(speed,speed);
                          //Clock_Delay1ms(1000);
                          break;
                      case 's': //stop
                          Motor_Stop();
                          break;
                      case 'q': //accelerate
                          speed *= 1.2;
                          break;
                      case 'z': //decelerate
                          speed *= 0.8;
                          break;
                  }

//                  default:
//                     break;
                }
                menu = 1;
                cmd=0xDEAD;
                break;

          case 7:           //Advanced Topic2: Obstacle Avoidance
                EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("Selected: Obstacle Avoidance"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

                //int32_t loopCounter;  //for inner loop: collects IR sensor data
                Clock_Init48MHz();

                while(LaunchPad_Input()==0){

                    Motor_Forward(speed,speed);

                    EnableInterrupts();
                    for(loopCounter; loopCounter < 2000; loopCounter++){    //duration specified bz loopCounter, wait for ADCflag to become non-zero (indication that it has processed the data)
                          while(ADCflag == 0){};
                          ADCflag = 0;
                        }
                    UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");      //LeftConvert converts the raw sensor data into distance measurements in mm
                    UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");    //UART0_OutDec5 displays the IR sensor readings on the terminal
                    UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");

                    if(LeftConvert(nl) < 80){
                        Motor_Stop();
                        Motor_Backward(speed,speed);
                        Clock_Delay1ms(250);
                        Motor_Stop();
                        Motor_Right(0,speed);
                        Clock_Delay1ms(500);
                    }
                    else if(CenterConvert(nc) < 80){
                        Motor_Stop();
                        Motor_Backward(speed,speed);
                        Clock_Delay1ms(250);
                        Motor_Stop();
                        Motor_Right(0,speed);
                        Clock_Delay1ms(250);
                    }
                    else if(RightConvert(nr) < 80){
                        Motor_Stop();
                        Motor_Backward(speed,speed);
                        Clock_Delay1ms(250);
                        Motor_Stop();
                        Motor_Left(0,speed);
                        Clock_Delay1ms(500);
                    }
                }

                menu = 1;
                cmd=0xDEAD;
                break;

              // ....
              // ....

          default:
              menu=1;
              break;
      }

      if(!menu)Clock_Delay1ms(3000);
      else{
          menu=0;
      }

      // ....
      // ....
  }
}

#if 0
//Sample program for using the UART related functions.
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}
#endif
