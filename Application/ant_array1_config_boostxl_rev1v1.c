/******************************************************************************

 @file       ant_array1_config_boostxl_rev1v1.c

 @brief This file contains the antenna array tables for
        Angle of Arrival feature.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2018-2018, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED 鈥淎S IS鈥� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_02_30_00_28
 Release Date: 2018-10-15 15:51:38
 *****************************************************************************/

#include "aoa/AOA.h"

// User defined nice-names for the pins
#define AOA_A1_SEL     AOA_PIN(IOID_27)
#define AOA_Ax_ANT1    AOA_PIN(IOID_28)
#define AOA_Ax_ANT2    AOA_PIN(IOID_29)
#define AOA_Ax_ANT3    AOA_PIN(IOID_30)

// NOTE: A1_ANT1 and A2_ANT1 is the same pin. Do not toggle if
//       switching between these. Or use the AOA_SWITCH_MASK macro.

AoA_Pattern antennaPattern_A1 = {
    .numPatterns = 32,
    .initialPattern = AOA_A1_SEL | AOA_Ax_ANT2,
    .toggles =
    {
     /**
      * changes:
      * 1\2\3 antennas' cycle
      * to
      * 1\2 antennas' cycle
      */
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2

     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2

     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2

     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2

     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
     AOA_A1_SEL | AOA_Ax_ANT1, // A1.1
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2

     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2 end at antenna 2
     AOA_A1_SEL | AOA_Ax_ANT2, // A1.2
    }
};

/*
 * @brief Overwrite set-pattern with toggle-pattern in place
 */
void BOOSTXL_AoA_AntennaPattern_A1_init()
{
    AoA_Pattern *pattern = &antennaPattern_A1;
    AOA_toggleMaker(pattern->toggles, pattern->initialPattern, pattern->numPatterns, pattern->toggles);
}

AoA_AntennaPair pair_A1[] =
{
//   {// v12
//    .a = 0,       // First antenna in pair
//    .b = 1,       // Second antenna in pair
//    .sign = 1,    // Sign for the result
//    .offset = 5,  // Measurement offset compensation
//    .gain = 1,    // Measurement gain compensation
//   },
   {// v23
    .a = 1,
    .b = 2,
    .sign = 1,
    .offset = 0,
    .gain = 1,
   },
   {// v13
    .a = 0,
    .b = 2,
    .sign = 1,
    .offset = 10,
    .gain = 0.50,
   },
};

AoA_AntennaConfig BOOSTXL_AoA_Config_ArrayA1 =
{
 .numAntennas = 3,
 .pattern = &antennaPattern_A1,
 .numPairs = sizeof(pair_A1) / sizeof(pair_A1[0]),
 .pairs = pair_A1,
};

uint32_t signalAmplitude_A1[sizeof(pair_A1) / sizeof(pair_A1[0])];
int16_t  pairAngle_A1[sizeof(pair_A1) / sizeof(pair_A1[0])];

AoA_AntennaResult BOOSTXL_AoA_Result_ArrayA1 =
{
 .signalStrength = (uint32_t *)&signalAmplitude_A1,
 .pairAngle = (int16_t *)&pairAngle_A1,
 .rssi = 0,
 .updated = false,
};
