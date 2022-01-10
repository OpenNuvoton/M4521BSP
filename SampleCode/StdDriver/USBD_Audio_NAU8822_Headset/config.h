/******************************************************************************
 * @file     config.h
 * @version  V0.10
 * @brief
 *           Define the device setting.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/* Just select one */
#define MEDIAKEY    0
#define JOYSTICK    1

#if MEDIAKEY
    #define __HID__
    #define __MEDIAKEY__
#elif JOYSTICK
    #define __HID__
    #define __JOYSTICK__
#endif
