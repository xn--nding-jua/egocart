#ifndef USER_DUAL_MOTOR_H
#define USER_DUAL_MOTOR_H
//#############################################################################
// $Copyright:
// Copyright (C) 2017-2023 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//  Contains the user related definitions
//!

// **************************************************************************
// the includes
// modules
#include "userParams.h"
#include "user_common.h"
#include "user_motor_1.h"
#include "user_motor_2.h"
//!
//!
//! \defgroup USER USER
//!
//@{

// user defines
#define DRV8320_SPI // use TI DRV8320RS gatedriver
#define COMM_SCI // use SCI for communication
#define SCI_BAUDRATE 38400
#define MOTOR_MAX_SPEED 3500 // rpm
#define MOTOR_MAX_ACCEL 250 // rpm/second

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif // extern "C"
//@}  // ingroup
#endif // end of USER_DUAL_MOTOR_H definition
