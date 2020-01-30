/******************************************************************************

 @file  dbg.h

 @brief Provides data logging, using the on-chip RF core tracer

 Group: WCS, LPC
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2011-2019, Texas Instruments Incorporated
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
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
 
 
 *****************************************************************************/

/** \addtogroup module_dbg Debug Functionality (dbg)
 * \ingroup module_cc26_rfcore_fw
 *
 *
 *
 * \section section_dbg_responsibilities Responsibilities
 *
 * The tracer module implements three CPU debug channels that share the same serial interface.
 *
 * With debug IDs 0-1 reserved for special purposes, there can be 254 DBG_PRINTn() statements per 
 * debug channel. Having more DBG_PRINTn() statements will result in compilation error (undefined
 * symbol).
 *
 *
 * \section section_dbg_usage Usage
 *
 * \subsection section_dbg_enabling Enabling and Disabling
 * To use the debug functionality in a component, the following code must be added in file header:
 * \code
 * #define DBG_ENABLE
 * #include <dbgid.h>
 * \endcode
 *
 * The "dbgid.h" file is autogenerated by the Makefile and put in the src directory of the active
 * project. It has been generated on the basis of scanning all .c files and finding any usage of
 * the DBG_PRINTn macros.
 *
 * The debug functionality can be disabled altogether by defining \c DBG_GLOBAL_DISABLE at compilation.
 *
 *
 * @{
 */
#ifndef _DBG_H
#define _DBG_H

#include "rfctrc_regs.h"
//#include "error_handler.h"
#include <stdint.h>
#include "mac_radio.h"

//-------------------------------------------------------------------------------------------------------
/// \name Internal Functionality
//@{

#if !USE_TIRTOS_DBG
#if defined(DBG_ENABLE) && !defined(DBG_GLOBAL_DISABLE)
#define DBG_MACRO(x) if (!macRadioYielded) { x }
//#define DBG_MACRO(x) if (1) { x }
#else
#define DBG_MACRO(x)
#endif

// Functions used to implement DBG_PRINTn
extern void dbgPrintf(uint32_t x, uint32_t y, uint32_t z);
extern void dbgPrintf2(uint32_t x, uint32_t y);
extern void dbgPrintf0(uint32_t x);
#else
#include <xdc/runtime/Log.h>
#endif //!USE_TIRTOS_DBG

//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \anchor debugid_defs \name Reserved Debug IDs
//@{

#define DBGID_SYSTICK           1   ///< Debug ID reserved for system tick trace packets
#define DBGID_ENDSIM            2   ///< Debug ID reserved for end of simulation directive. Takes one parameter.


//@}

//-------------------------------------------------------------------------------------------------------

/// \anchor debug_preemp_levels \name Preemption Levels for DBG_PRINT()
//@{

#define DBGCH1      1   ///< Trace channel #1
#define DBGCH2      2   ///< Trace channel #2
#define DBGCH3      3   ///< Trace channel #3

#define DBGCPE      1   ///< Use trace channel #1 in CPE
#define DBGTOPSM    2   ///< Use trace channel #2 in TOPsm
#define DBGSYS      3   ///< Use trace channel #3 in system CPU

//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Available Operations
//@{

/// Helper macro
#define _XDBGFILESTR(x) #x
/// Helper macro
#define _XXDBG(x,y) DBGID_ ## x ## _ ## y
/// Helper macro
#define _XDBG(x,y) _XXDBG(x,y)

#if !USE_TIRTOS_DBG
/// Emulates printf() with no arguments, but sends an ID instead of the format string
#define DBG_PRINT0(pre, str) \
   DBG_MACRO( \
      dbgPrintf0(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 0); \
   )

/// Emulates printf() with 1 numeric argument (16-bit)
#define DBG_PRINT1(pre, str, a0) \
   DBG_MACRO( \
      dbgPrintf2(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 1, (((uint32_t)(a0))&0xFFFF)); \
   )

/// Emulates printf() with 2 numeric arguments (16-bit)
#define DBG_PRINT2(pre, str, a0, a1) \
   DBG_MACRO( \
      dbgPrintf2(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 2, ((((uint32_t)(a0))&0xFFFF) | (((uint32_t)(a1))&0xFFFF)<< 16)); \
   )

/// Emulates printf() with 3 numeric arguments (16-bit)
#define DBG_PRINT3(pre, str, a0, a1, a2) \
   DBG_MACRO( \
      dbgPrintf(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 3, (((uint32_t)(a0))&0xFFFF) | ((((uint32_t)(a1))&0xFFFF)<<16), ((uint32_t)(a2)&0xFFFF)); \
   )

/// Emulates printf() with 4 numeric arguments (16-bit)
#define DBG_PRINT4(pre, str, a0, a1, a2, a3) \
   DBG_MACRO( \
      dbgPrintf(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 4, (((uint32_t)(a0))&0xFFFF) | ((((uint32_t)(a1))&0xFFFF)<<16), (((uint32_t)(a2))&0xFFFF) | ((((uint32_t)(a3))&0xFFFF)<<16)); \
   )

/// Emulates printf() with 1 long numeric argument (32-bit)
#define DBG_PRINTL1(pre, str, a0) \
   DBG_MACRO( \
      dbgPrintf2(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 2, (((uint32_t)(a0)))); \
   )

/// Emulates printf() with 2 long numeric arguments (32-bit)
#define DBG_PRINTL2(pre, str, a0, a1) \
   DBG_MACRO( \
      dbgPrintf(((pre)<<16) | (_XDBG(_DBGFILE,__LINE__)<<8) | 4, ((uint32_t)(a0)), ((uint32_t)(a1))); \
   )


/// End of simulation macro
#define DBG_ENDSIM(pre, code) \
   DBG_MACRO( \
      dbgPrintf2(((pre)<<16) | ((DBGID_ENDSIM) << 8) | 1, (((uint32_t)(code))&0xFFFF)); \
   )
#else
/// Emulates printf() with no arguments, but sends an ID instead of the format string
#define DBG_PRINT0(pre, str) \
   (void) pre; \
   Log_info0(str);

/// Emulates printf() with 1 numeric argument (16-bit)
#define DBG_PRINT1(pre, str, a0) \
        (void) pre; \
        Log_info1(str, a0);

/// Emulates printf() with 2 numeric arguments (16-bit)
#define DBG_PRINT2(pre, str, a0, a1) \
        (void) pre; \
        Log_info2(str, a0, a1);

/// Emulates printf() with 3 numeric arguments (16-bit)
#define DBG_PRINT3(pre, str, a0, a1, a2) \
        (void) pre; \
        Log_info3(str, a0, a1, a2);

/// Emulates printf() with 4 numeric arguments (16-bit)
#define DBG_PRINT4(pre, str, a0, a1, a2, a3) \
        (void) pre; \
        Log_info4(str, a0, a1, a2, a3);

/// Emulates printf() with 1 long numeric argument (32-bit)
#define DBG_PRINTL1(pre, str, a0) \
        (void) pre; \
        Log_info1(str, a0);

/// Emulates printf() with 2 long numeric arguments (32-bit)
#define DBG_PRINTL2(pre, str, a0, a1) \
        (void) pre; \
        Log_info2(str, a0, a1);

#endif //#if !USE_TIRTOS_DBG

/// End of simulation with success
#define DBG_SUCCESS(pre) DBG_ENDSIM(pre, 0)

/// Verifies the given \a condition, and halts further code execution upon a negative result
#define DBG_ASSERT(pre, condition) \
   DBG_MACRO( \
      if (!(condition)) { \
         DBG_PRINT0(pre, #condition ); \
         FATAL_ERROR(); \
      } \
   )

/// Re-define ASSERT if trace is enabled
#if defined(DBG_ENABLE) && !defined(DBG_GLOBAL_DISABLE) && !defined(_NO_ASSERT)
#undef ASSERT
#define ASSERT(condition) DBG_ASSERT(DBGSYS, condition)
#endif

/// Used in the generated file dbgid.h to define preprocessor symbols
#define DBG_DEF(sym, id, pre, nargs, str, fname, lineno) \
   static const uint32_t sym = id;

//@}
//-------------------------------------------------------------------------------------------------------




#endif
//@}