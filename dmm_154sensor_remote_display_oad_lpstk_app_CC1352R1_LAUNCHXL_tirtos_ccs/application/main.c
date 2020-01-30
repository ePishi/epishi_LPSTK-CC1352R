/******************************************************************************

 @file  main.c

 @brief Main entry of the 15.4 & BLE remote display sample application.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"
#include "remote_display.h"

#include "macTask.h"
#include "sensor.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#include <inc/hw_ccfg.h>
#include <inc/hw_ccfg_simple_struct.h>

#ifdef NV_RESTORE
#include "macconfig.h"
#include "nvocmp.h"
#endif


#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
// BLE user defined configuration
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#endif // USE_DEFAULT_USER_CFG

#ifdef DMM_OAD
#include <ti/drivers/GPIO.h>
#include <profiles/oad/cc26xx/mark_switch_factory_img.h>
#endif

#include "mac_user_config.h"

/* Include DMM module */
#include <dmm/dmm_scheduler.h>
#include "ti_dmm_application_policy.h"
#include <dmm/dmm_priority_ble_154.h>

#if defined(RESET_ASSERT)
#include <driverlib/sys_ctrl.h>
#include "ssf.h"
#endif

#include "cui.h"
/*******************************************************************************
 * MACROS
 */
#if defined(FEATURE_SECURE_COMMISSIONING) || defined(FEATURE_NATIVE_OAD)
#error "Secure commissioning and Native OAD are not currently supported for DMM."
#endif
/*******************************************************************************
 * CONSTANTS
 */
/* Extended Address offset in FCFG (LSB..MSB) */
#define EXTADDR_OFFSET 0x2F0

#define MAC_APP_TASK_PRIORITY   1
#if defined(CC13X2R1_LAUNCHXL) || defined(CC26X2R1_LAUNCHXL) || defined(CC13X2P1_LAUNCHXL) || defined(CC13X2P_2_LAUNCHXL ) || defined(CC13X2P_4_LAUNCHXL)
#define MAC_APP_TASK_STACK_SIZE 1536
#else
#define MAC_APP_TASK_STACK_SIZE 900
#endif

#define MAIN_ASSERT_HWI_TIRTOS   4

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */
/* Used to check for a valid extended address */
static const uint8_t dummyExtAddr[] =
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/*******************************************************************************
 * GLOBAL VARIABLES
 */
/* MAC user defined configuration */
macUserCfg_t macUser0Cfg[] = MAC_USER_CFG;

Task_Struct macAppTask;        /* not static so you can see in ROV */
static uint8_t macAppTaskStack[MAC_APP_TASK_STACK_SIZE];

static uint8_t _macTaskId;

#ifdef NV_RESTORE
mac_Config_t Main_user1Cfg = { 0 };
#endif

/*******************************************************************************
 * EXTERNS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern ApiMac_sAddrExt_t ApiMac_extAddr;

/*!
 * @brief       Main task function
 *
 * @param       a0 -
 * @param       a1 -
 */
Void macAppTaskFxn(UArg a0, UArg a1)
{
#ifdef TIMAC_AGAMA_FPGA
    /* FPGA build disables POWER constraints */
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);

    IOCPortConfigureSet(IOID_20, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_18, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);
    // configure RF Core SMI Command Link
    IOCPortConfigureSet(IOID_22, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_21, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);
#endif

    /*
     * Copy the extended address from the CCFG area
     * Assumption: the memory in CCFG_IEEE_MAC_0 and CCFG_IEEE_MAC_1
     * is contiguous and LSB first.
     */
    memcpy(ApiMac_extAddr, (uint8_t *)&(__ccfg.CCFG_IEEE_MAC_0),
           (APIMAC_SADDR_EXT_LEN));

    /* Check to see if the CCFG IEEE is valid */
    if(memcmp(ApiMac_extAddr, dummyExtAddr, APIMAC_SADDR_EXT_LEN) == 0)
    {
        /* No, it isn't valid.  Get the Primary IEEE Address */
        memcpy(ApiMac_extAddr, (uint8_t *)(FCFG1_BASE + EXTADDR_OFFSET),
               (APIMAC_SADDR_EXT_LEN));
    }

#ifdef NV_RESTORE
    /* Setup the NV driver */
    NVOCMP_loadApiPtrs(&Main_user1Cfg.nvFps);

#if !defined(BLE_START) && defined(MAC_START)
    // The init is done by BLE. Leave it here for now in case we run 15.4 only.
    if(Main_user1Cfg.nvFps.initNV)
    {
        Main_user1Cfg.nvFps.initNV(NULL);
    }
#endif

#endif

    /* Initialize the application */
    Sensor_init(_macTaskId);

    /* Kick off application - Forever loop */
    while(1)
    {
        Sensor_process();
    }
}

/*!
 * @brief       TIRTOS HWI Handler.  The name of this function is set to
 *              M3Hwi.excHandlerFunc in app.cfg, you can disable this by
 *              setting it to null.
 *
 * @param       excStack - TIROS variable
 * @param       lr - TIROS variable
 */
xdc_Void Main_excHandler(UInt *excStack, UInt lr)
{
#if defined(RESET_ASSERT)
    Ssf_assertInd(MAIN_ASSERT_HWI_TIRTOS);

    /* Pull the plug and start over */
    SysCtrlSystemReset();
#else
    //spin here
    while(1);
#endif
}

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main()
{
  Task_Handle* pBleTaskHndl;
  Task_Handle* pMacTaskHndl;
  DMMPolicy_Params dmmPolicyParams;
  DMMSch_Params dmmSchedulerParams;
  Task_Params macAppTaskParams;

  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  Board_initGeneral();

#ifdef DMM_OAD
    /* If DMM_OAD is enabled, look for a left button
     *  press on reset. This indicates to revert to some
     *  factory image
     */
    if (!GPIO_read(CONFIG_GPIO_BTN1))
    {
        markSwitchFactoryImg();
    }
#endif /* DMM_OAD */

  // Enable iCache prefetching
  VIMSConfigure(VIMS_BASE, TRUE, TRUE);
  // Enable cache
  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

#if !defined( POWER_SAVING )
  /* Set constraints for Standby, powerdown and idle mode */
  // PowerCC26XX_SB_DISALLOW may be redundant
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif // POWER_SAVING

  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();

#ifndef POWER_MEAS
    /* Initialize UI for key and LED */
    CUI_params_t cuiParams;
    CUI_paramsInit(&cuiParams);

    // One-time initialization of the CUI
    CUI_init(&cuiParams);
#endif

#ifdef BLE_START
  /* Initialize ICall module */
  ICall_init();

  /* Start tasks of external images */
  ICall_createRemoteTasks();
  pBleTaskHndl = ICall_getRemoteTaskHandle(0);

  RemoteDisplay_createTask();
#endif

#ifdef MAC_START
  /* Initialize 15.4 sensor tasks */
  _macTaskId = macTaskInit(macUser0Cfg);
  pMacTaskHndl = macTaskGetTaskHndl();

  /* create 15.4 Sensor app task
   */
  Task_Params_init(&macAppTaskParams);
  macAppTaskParams.stack = macAppTaskStack;
  macAppTaskParams.stackSize = MAC_APP_TASK_STACK_SIZE;
  macAppTaskParams.priority = MAC_APP_TASK_PRIORITY;
  Task_construct(&macAppTask, macAppTaskFxn, &macAppTaskParams, NULL);
#endif

  /* initialize and open the DMM policy manager */
  DMMPolicy_init();
  DMMPolicy_Params_init(&dmmPolicyParams);
  dmmPolicyParams.numPolicyTableEntries = DMMPolicy_ApplicationPolicySize;
  dmmPolicyParams.policyTable = DMMPolicy_ApplicationPolicyTable;
  dmmPolicyParams.globalPriorityTable = globalPriorityTable_bleL154H;
  DMMPolicy_open(&dmmPolicyParams);

  /* initialize and open the DMM scheduler */
  DMMSch_init();
  DMMSch_Params_init(&dmmSchedulerParams);

  //Copy stack roles and index table
  memcpy(dmmSchedulerParams.stackRoles, DMMPolicy_ApplicationPolicyTable.stackRole, sizeof(DMMPolicy_StackRole) * DMMPOLICY_NUM_STACKS);
  dmmSchedulerParams.indexTable = DMMPolicy_ApplicationPolicyTable.indexTable;
  DMMSch_open(&dmmSchedulerParams);

  /* register clients with DMM scheduler */
  DMMSch_registerClient(pBleTaskHndl, DMMPolicy_StackRole_BlePeripheral);
  DMMSch_registerClient(pMacTaskHndl, DMMPolicy_StackRole_154Sensor);

  /* set the stacks in default states */
  DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BLE_IDLE);
  DMMPolicy_updateStackState(DMMPolicy_StackRole_154Sensor, DMMPOLICY_154_UNINIT);

  /* enable interrupts and start SYS/BIOS */
  BIOS_start();

  return 0;
}


/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
        CUI_assert("***ERROR*** >> OUT OF MEMORY!", false);
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
          CUI_assert("***ERROR*** >> INTERNAL FW ERROR!", false);
      }
      else
      {
          CUI_assert("***ERROR*** >> INTERNAL ERROR!", false);
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
        CUI_assert("***ERROR*** >> ICALL ABORT!", true);
      break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
        CUI_assert("***ERROR*** >> ICALL TIMEOUT!", true);
      break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
        CUI_assert("***ERROR*** >> WRONG API CALL!", true);
      break;

  default:
      CUI_assert("***ERROR*** >> DEFAULT SPINLOCK!", true);
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

/*******************************************************************************
 */
