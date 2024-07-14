/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*=====================================================================================
 * Description:
 *   The Modbus parameter structures used to define Modbus instances that
 *   can be addressed by Modbus protocol. Define these structures per your needs in
 *   your application. Below is just an example of possible parameters.
 *====================================================================================*/
#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

#include <stdint.h>



#pragma pack(push, 1)
typedef struct
{
    float holding_data0;
    float holding_data1;
    float holding_data2;
    float holding_data3;
    uint16_t test_regs[150];
    float holding_data4;
    float holding_data5;
    float holding_data6;
    float holding_data7;
} holding_reg_params_t;
#pragma pack(pop)

extern holding_reg_params_t holding_reg_params;

#endif // !defined(_DEVICE_PARAMS)