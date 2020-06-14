/************************************************************************************
*
*	File:		as5047p_hal.h
*	Author:		Krzysztof Sawicki
*	Brief:		Header file containing prototypes of low level functions
*			used by AS5047P driver.
*
*	Description:	---
*
*************************************************************************************/

#ifndef AS5047P_HAL_H_
#define AS5047P_HAL_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* -------------------------------------------------------------------------------- */
/* -- Include SECTION                                                            -- */
/* -------------------------------------------------------------------------------- */

#include <stdint.h>

/* -------------------------------------------------------------------------------- */
/* -- FUNCTIONS PROTOTYPES  	                                        	 -- */
/* -------------------------------------------------------------------------------- */

/************************************************************************************
*
* 	Function: 	AS5047P_HAL_Debug(id,errorCode,errMsg)
* 	Access:		public
* 	Inputs:
*		id: ID number uniquely identifies an SPI's slave. Relation between
*		slave's id and physical CS line is defined in AS5047P_SelectSPIAndGPIO.
*		errCode: Error code for which debug is to take place.
*		errMsg: Error message for 'errCode'
*
* 	Returns:
*		Nothing.
*
* 	Description:
* 	Function handles debug tasks for a given encoder.
*
************************************************************************************/
void AS5047P_HAL_Debug(uint16_t id,int16_t errorCode,char * errMsg);

/************************************************************************************
 *
 * 	Function: 	AS5047P_HAL_Delay_ms(delay)
 * 	Access:		public
 * 	Inputs:
 * 		ms:	Delay in ms.
 *
 * 	Returns:
 * 		Nothing.
 *
 * 	Description:
 * 	Function stalls program execution for 'delay' miliseconds.
 *
 ************************************************************************************/
void AS5047P_HAL_Delay_ms(uint32_t ms);

/************************************************************************************
 *
 * 	Function: 	AS5047P_HAL_Init(void)
 * 	Access:		public
 * 	Inputs:
 * 		idCS: Id of encoder's instance. Used to select correct CS line.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 * 	Function used to initialize whatever you want. It's called in AS5047P Init().
 *
 ************************************************************************************/
uint8_t AS5047P_HAL_Init(uint16_t id);

/************************************************************************************
 *
 * 	Function: 	AS5047P_HAL_SPI_Transaction(pTx,pRx)
 * 	Access:		public
 * 	Inputs:
 *		pTx: Pointer to two-byte field containing data to put on SPI MOSI line.
 *		pRx: Pointer to two-byte field to be filled with SPI MISO line data.
 *		idCS: Id of encoder's instance. Used to select correct CS line.
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 * 	Function is used for full-duplex SPI transactions. Function is of blocking
 * 	type.
 *
 ************************************************************************************/
uint8_t AS5047P_HAL_SPI_Transaction(uint16_t * pTx, uint16_t * pRx, uint16_t id);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_HAL_H_ */
