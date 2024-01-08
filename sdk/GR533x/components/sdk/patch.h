#ifndef __PATCH_H_
#define __PATCH_H_

/**
 ****************************************************************************************
 *
 * @file patch.h
 *
 * @brief offer the interface for the patch function based on the FPB of the cortex arm-m4;
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
//enum
//{
//    BIT_OLD_FUN_NAME0,
//    BIT_OLD_FUN_NAME1,
//};

/*
 * MACRO DECLARATIONS
 ****************************************************************************************
 */
#define PATCH_ENABLE_FLAG(BIT) (1<<BIT)
//please add the macro for the different application(Support 16 patches);

//#define MANDATORY_PATCH       (PATCH_ENABLE_FLAG(BIT_OLD_FUN_NAME0) | PATCH_ENABLE_FLAG(BIT_OLD_FUN_NAME1))
#define MANDATORY_PATCH         0

#define OPTIMIZING_PATCH        0

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
  * @brief  The enable of the patch featurn based on the FPB of the Cortex ARM-m4.
  * @param  patch_flag    the flag used to control the function to be selected as the patch function,one bit for one function.
  *         This parameter can be a combiantion of the following values:
  *         @arg @ref (1<<BIT_LLD_LLCP_OPCODE_IS_INVALID ,    )
  *         @arg @ref (1<<BIT_LLD_TEST_ISR,                )
  *         please used the MACRO PATCH_ENABLE_FLAG(BIT) just like the MANDATORY_PATCH;
  *         and the different MACRO maybe defined for the different application;
  * @retval None
  */

extern void set_patch_flag(uint32_t patch_flag);

/**
  * @brief  Register the path function to the hardware patch.
  * @param  patch_index  the patch index.
  * @param  func_addr    the address of the patch function. 
  *
  * @retval None
  */
void fpb_register_patch_function(int patch_index, uint32_t func_addr);

#endif  // __PATCH_H_
