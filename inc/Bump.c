// Bump.c
// Runs on MSP432
// Provide low-level functions that interface bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 2, 2017

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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"

extern volatile uint8_t bumpState;
extern volatile uint8_t status;
void(*Port4Task)(uint8_t);

// Initialize Bump sensors
// Make six Port 4 pins inputs (SEL0, SEL1 und DIR muessen 0 sein)
// Activate interface pullup (REN && OUT muessen 1 sein)
// pins 7,6,5,3,2,0 -> 1110.1101 -> 0xED
void Bump_Init(void(*task)(uint8_t)){
    // write this as part of Lab 3
    // Initialise GPIO related registers.
    // Registers: SEL0, SEL1, DIR, REN, OUT.
    Port4Task = task;                 // global function pointer = user task

    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED;
    P4->REN |= 0xED;
    P4->OUT |= 0xED;
    P4->IES |= 0xED;                   // P1.4 and P1.1 are falling edge event
    P4->IFG &= ~0xED;                  // clear 0 - 7 (reduce possibility of extra interrupt)
    P4->IE |= 0xED;                    // arm interrupt on pin above
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000; // priority 2
    NVIC->ISER[1] = 0x00000040;        // enable interrupt 38 in NVIC (I/O port 4 interrupt = IRQ38)
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    // write this as part of Lab 3
    // Pack the 6 valid bits in the value read from the input data
    // register to occupy 6 lower order bits of the result variable.
    uint8_t bump0, bump1, bump2, bump3, bump4, bump5;
    uint8_t result;
    bump0 = (P4->IN)&0x01;        // for bump 0 (see if P4->IN has value 1 or 0 for bit 0 --> through bitwise AND) from pin 0 stay in pos 0
    bump1 = (((P4->IN)&0x04)>>1);        // for bump 1: from pin 2 to pos 1
    bump2 = (((P4->IN)&0x08)>>1);         // for bump 2: from pin 3 to pos 2
    bump3 = (((P4->IN)&0x20)>>2);        // for bump 3: from pin 5 to pos 3
    bump4 = (((P4->IN)&0x40)>>2);         // for bump 4: from pin 6 to pos 4
    bump5 = (((P4->IN)&0x80)>>2);        // for bump 5: from pin 7 to pos 5
    result = bump0 + bump1 + bump2 + bump3 + bump4 + bump5;   // assemble the 6 bumps we have gathered
    return (result);
}


void PORT4_IRQHandler(void){
    uint8_t status, result;
    status = P4 -> IV;

    if(status != 0x00){
        result = Bump_Read();
//        Port4Task(result);      // call the user-defined task with the bump switch
        bumpState = result;
    }
    P4->IFG &= 0x12;            // clear interrupt flags for bump switches


//    (*Port4Task)(Bump_Read());
    //P4->IFG=0x00;
}

