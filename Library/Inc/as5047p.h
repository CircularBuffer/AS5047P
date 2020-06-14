/************************************************************************************
*
*	File:		as5047p.h
*	Author:		Krzysztof Sawicki
*	Brief:		Source file for Generic AS5047P driver.
*
*	Description:
*	This is a generic (platform independent) AS5047P driver. Core language is C
*	but file 'as5047p_hal.c' is also compilable by C++ compiler.
*	Driver needs little effort to make it working. You need to go to file
*	'as5047p_hal.c/cpp' and implement 5 functions where you find this annotation:
*
* 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
*	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*
*	And that's it !
*
*	If you want to enable DEBUG mode, you can do it by defining AS5047P_USE_DEBUG
*	to 1 (in file 'as5047.h').
*
*	If you want to save some memory space, you can exclude error message texts
*	by defining AS5047P_SAVE_SPACE to 1 (in file 'as5047.h').It allows to save
*	approx. 11kB of space.
*
*************************************************************************************/

#ifndef AS5047P_H_
#define AS5047P_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* -------------------------------------------------------------------------------- */
/* -- Include SECTION                                                            -- */
/* -------------------------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------------- */
/* -- CONFIG SECTION                                                             -- */
/* -------------------------------------------------------------------------------- */
//--- It excludes an array of error message texts from compilation. It allows to save
//--- approx. 11kB of space.
#define AS5047P_SAVE_SPACE	0
//--- Activates DEBUG mode
#define AS5047P_USE_DEBUG	0

#define AS5047P_ERRMSG_COUNT 140
#define AS5047P_ERRMSG_MAX_LEN 80

/* -------------------------------------------------------------------------------- */
/* -- CONSTANTS SECTION             	                                         -- */
/* -------------------------------------------------------------------------------- */
#define AS5047P_OPT_ENABLED 		true
#define AS5047P_OPT_DISABLED 		false

#define AS5047P_ACCESS_WRITE 		false
#define AS5047P_ACCESS_READ 		true

#define AS5047P_FRAME_PARD		( 1 << 15)
#define AS5047P_FRAME_EF 		( 1 << 14)
#define AS5047P_FRAME_DATA		0x3FFF

#define AS5047P_ABIRES_100 	100
#define AS5047P_ABIRES_200 	200
#define AS5047P_ABIRES_400 	400
#define AS5047P_ABIRES_800 	800
#define AS5047P_ABIRES_1200 	1200
#define AS5047P_ABIRES_1600 	1600
#define AS5047P_ABIRES_2000 	2000
#define AS5047P_ABIRES_4000 	4000
#define AS5047P_ABIRES_1024 	1024
#define AS5047P_ABIRES_2048 	2048
#define AS5047P_ABIRES_4096 	4096

// --- Volatile registers
#define AS5047P_NOP          	0x0000
#define AS5047P_ERRFL        	0x0001
#define AS5047P_PROG        	0x0003
#define AS5047P_DIAAGC       	0x3FFC
#define AS5047P_MAG          	0x3FFD
#define AS5047P_ANGLEUNC     	0x3FFE
#define AS5047P_ANGLECOM     	0x3FFF

// --- Non-volatile registers
#define AS5047P_ZPOSM        	0x0016
#define AS5047P_ZPOSL        	0x0017
#define AS5047P_SETTINGS1    	0x0018
#define AS5047P_SETTINGS2    	0x0019

// --- Fields in registers
#define AS5047P_ERRFL_PARERR		( 1 << 2)
#define AS5047P_ERRFL_INVCOMM		( 1 << 1)
#define AS5047P_ERRFL_FRERR		( 1 << 0)
#define AS5047P_PROG_PROGVER		( 1 << 6)
#define AS5047P_PROG_PROGOTP		( 1 << 3)
#define AS5047P_PROG_OTPREF		( 1 << 2)
#define AS5047P_PROG_PROGEN		( 1 << 0)
#define AS5047P_DIAAGC_MAGL		( 1 << 11)
#define AS5047P_DIAAGC_MAGH		( 1 << 10)
#define AS5047P_DIAAGC_COF		( 1 << 9)
#define AS5047P_DIAAGC_LF		( 1 << 8)
#define AS5047P_DIAAGC_AGC		( 0x00FF << 0)
#define AS5047P_MAG_CMAG		( 0x3FFF << 0)
#define AS5047P_ANGLEUNC_CORDICANG	( 0x3FFF << 0)
#define AS5047P_ANGLECOM_DAECANG	( 0x3FFF << 0)
#define AS5047P_ZPOSM_ZPOSM		( 0x00FF << 0)
#define AS5047P_ZPOSL_COMP_H_ERR_EN	( 1 << 7)
#define AS5047P_ZPOSL_COMP_I_ERR_EN	( 1 << 6)
#define AS5047P_ZPOSL_ZPOSL		( 0x003F << 0)
#define AS5047P_SETTINGS1_BIT0		( 1 << 0)
#define AS5047P_SETTINGS1_NOISESET	( 1 << 1)
#define AS5047P_SETTINGS1_DIR		( 1 << 2)
#define AS5047P_SETTINGS1_UVW_ABI	( 1 << 3)
#define AS5047P_SETTINGS1_DAECDIS	( 1 << 4)
#define AS5047P_SETTINGS1_ABIBIN	( 1 << 5)
#define AS5047P_SETTINGS1_DATASEL	( 1 << 6)
#define AS5047P_SETTINGS1_PWMON		( 1 << 7)
#define AS5047P_SETTINGS2_UVWPP		( 0x0007 << 0)
#define AS5047P_SETTINGS2_HYS		( 0x0003 << 3)
#define AS5047P_SETTINGS2_ABIRES	( 0x0007 << 5)

/* -------------------------------------------------------------------------------- */
/* -- TYPES PROTOTYPES  	                                        	 -- */
/* -------------------------------------------------------------------------------- */
typedef struct{

  int16_t errorCode;
  char * msg;

}AS5047P_Error;

typedef struct{

  /* These are private members. Should not be modified directly */
  uint16_t id; // Used for CS signal selection in as5047p_hal.c .
  uint16_t buffRx;
  uint16_t buffTx;
 _Bool zeroPosCalibrated;
 _Bool initialized;
 AS5047P_Error error;

} AS5047P_Instance;

/************************************************************************************
 * If AS5047P_Result >= 0 it stores valid result.
 * If AS5047P_Result < 0 it stores error code.
 ************************************************************************************/
typedef int16_t AS5047P_Result;

/************************************************************************************
 * If AS5047P_ErrCode != 0 it stores error code.
 * If AS5047P_ErrCode == 0 no error.
 ************************************************************************************/
typedef int16_t AS5047P_ErrCode;

/* -------------------------------------------------------------------------------- */
/* -- FUNCTIONS PROTOTYPES  	                                        	 -- */
/* -------------------------------------------------------------------------------- */

AS5047P_Result AS5047P_ReadWriteRaw(AS5047P_Instance * instance, AS5047P_Result frameTx, _Bool rw);
AS5047P_Result AS5047P_ReadRegister(AS5047P_Instance * instance, uint16_t regAddr, _Bool devReachCheck );
AS5047P_ErrCode AS5047P_WriteRegister(AS5047P_Instance * instance, uint16_t regAddr, AS5047P_Result newRegContent, _Bool writeVerif, _Bool devReachCheck );
AS5047P_ErrCode AS5047P_Init(AS5047P_Instance * instance, uint16_t id);
AS5047P_ErrCode AS5047P_SetFactorySettings(AS5047P_Instance * instance);
_Bool AS5047P_ErrorPending(AS5047P_Instance * instance);
AS5047P_Error AS5047P_GetError(AS5047P_Instance * instance);
AS5047P_ErrCode AS5047P_ErrorAck(AS5047P_Instance * instance);
AS5047P_ErrCode AS5047P_SetFieldInRegister(AS5047P_Instance * instance, uint16_t regAddr, uint16_t fieldMask, uint16_t fieldVal);
AS5047P_Result AS5047P_ReadPosition(AS5047P_Instance * instance, _Bool extendedDiag);
AS5047P_ErrCode AS5047P_SetZeroPosition(AS5047P_Instance * instance);
AS5047P_ErrCode AS5047P_SetABIResolution(AS5047P_Instance * instance, uint16_t resolution);
AS5047P_ErrCode AS5047P_BurnOTP(AS5047P_Instance * instance, uint16_t yesImSure);
uint16_t AS5047P_GetID(AS5047P_Instance * instance);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_H_ */
