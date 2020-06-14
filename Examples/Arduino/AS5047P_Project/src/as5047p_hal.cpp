/************************************************************************************
*
*	File:		as5047p_hal.c
*	Author:		Krzysztof Sawicki
*	Brief:		Source file containing prototypes of low level functions
*			used by AS5047P driver. It utilizes hardware abstracted
*			layer idea.
*
*	Description:
*	To make AS5047P driver working, all functions with annotation:
*
* 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
*	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*
*	has to be implemented. They are mostly related to platform specific SPI
*	communication and CS signals handling.
*
*	If your CPU is fast (let's say 180-200Mhz and more, you also should take care
*	of annotations:
*
*	 	!!! !!! CAUTION CAUTION CAUTION  !!! !!!
*
*	 which relates to SPI timing.
*
*************************************************************************************/

#include "as5047p_hal.h"

/************************************************************************************
 * 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
 *	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *
 * 	Description:
 *	Here private members, #defines and #includes for handling GPIO for CS and SPI
 *	port should be kept.
 *
 ************************************************************************************/
#include "Arduino.h"
#include "SPI.h"

static uint16_t gpioPin; // GPIO pin number that is to be used for next SPI transaction

/************************************************************************************
 * 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
 *	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * 	Function: 	AS5047P_HAL_Delay_ms(ms)
 * 	Access:		public
 * 	Inputs:
 * 		ms:	Delay in ms.
 *
 * 	Returns:
 * 		Nothing.
 *
 * 	Description:
 * 	Function stalls program execution for 'ms' miliseconds.
 *
 ************************************************************************************/
void AS5047P_HAL_Delay_ms(uint32_t ms)
{
  delay(ms);
}

/************************************************************************************
 * 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
 *	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * 	Function: 	AS5047P_HAL_SPI_TxRx(pTx, pRx)
 * 	Access:		private
 * 	Inputs:
 *		pTx: Pointer to two-byte field containing data to put on SPI MOSI line.
 *		pRx: Pointer to two-byte field to be filled with SPI MISO line data.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 * 	Function is used for full-duplex SPI transactions. Function is of blocking
 * 	type.
 *
 * 	Communication parameters used by AS5047P are:
 *
 * 	- SPI MODE 1 (CPOL = 0, CPHA = 1)
 * 	- SCK max. 10Mhz (11.25Mhz worked also fine but not recommended)
 * 	- Data size: 16 bits
 * 	- Bits order: MSB first
 *
 ************************************************************************************/
static uint8_t AS5047P_HAL_SPI_TxRx(uint16_t * pTx, uint16_t * pRx)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  *pRx = SPI.transfer16(*pTx); //Send and receive
  SPI.endTransaction();
  
  return 0;
}

/************************************************************************************
 * 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
 *	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * 	Function: 	AS5047P_HAL_GPIO_Write(state)
 * 	Access:		private
 * 	Inputs:
 *		state: 0 for low output level,
 *		       1 for high output level.
 *
 * 	Returns:
 *		Nothing.
 *
 * 	Description:
 * 	Function allows to clear/set digital output.
 *
 ************************************************************************************/
static void AS5047P_HAL_GPIO_Write(uint8_t state)
{
   digitalWrite(gpioPin, state);
}

/************************************************************************************
 * 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	 	!!! !!! MUST BE IMPLEMENTED BY THE USER !!! !!!
 *	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * 	Function: 	AS5047P_SelectSPIAndGPIO(id)
 * 	Access:		private
 * 	Inputs:
 *		id: ID number uniquely identifies an SPI's slave.
 *
 * 	Returns:
 *		-1 : Slave with this id doesn't exist
 *		 0 : Success.
 *
 * 	Description:
 * 	Function defines relation between slave's id and physical port of CS line
 * 	as well as SPI periphery (SPI handler) if necessary.
 *
 ************************************************************************************/
static uint8_t AS5047P_SelectSPIAndGPIO(uint16_t id)
{
  //--- Chip select and SPI selection
  switch (id)
   {
     case 0:
       gpioPin = 9;
       break;

     case 1:
       gpioPin = 10;
       break;

       //...
       //... other cases
       //...

     //--- Slave with this id doesn't exist
     default:
       return -1;

   }

  return 0;
}

/************************************************************************************
 * 		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	 	!!! !!! CAN BE IMPLEMENTED BY THE USER  !!! !!!
 *	 	!!! !!! ONLY IF YOU WANT TO USE DEBUG	!!! !!!
 *	 	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
void AS5047P_HAL_Debug(uint16_t id,int16_t errorCode,char * errMsg)
{
  //Print error
	Serial.print("Encoder ID["); Serial.print( id ); Serial.print("]> Error "); Serial.print( errorCode ); Serial.print(" : "); Serial.println(errMsg);
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_SelectSlave(id)
 * 	Access:		private
 * 	Inputs:
 *		id: ID number uniquely identifies an SPI's slave. Relation between
 *		slave's id and physical CS line is defined in AS5047P_SelectSPIAndGPIO.
 *
 * 	Returns:
 *		-1 : Slave with this id doesn't exist
 *		 0 : Success.
 *
 * 	Description:
 * 	Function selects the slave by setting its CS line low.
 *
 ************************************************************************************/
static uint8_t AS5047P_SelectSlave(uint16_t id)
{
  //--- Select GPIO and SPI handler
  if ( AS5047P_SelectSPIAndGPIO (id) < 0 )
  {   //--- Slave with this id doesn't exist
      return -1;
  }
  //--- CS line set to low level (line occupied)
  AS5047P_HAL_GPIO_Write(0x00);

  /************************************************************************************
  *
  *	 	!!! !!! CAUTION CAUTION CAUTION  !!! !!!
  *
  * AS5047P needs a minimum delay of 350ns between FALLING edge of CS and RISING
  * edge of SCK signal. So fast CPUs (~180Mhz and more) may need a delay here.
  *
  ************************************************************************************/
  //delay_us(1);

  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_DeselectSlave(id)
 * 	Access:		private
 * 	Inputs:
 *		id: ID number uniquely identifies an SPI's slave. Relation between
 *		slave's id and physical CS line is defined in AS5047P_SelectSPIAndGPIO.
 *
 * 	Returns:
 *		-1 : Slave with this id doesn't exist
 *		 0 : Success.
 *
 * 	Description:
 * 	Function deselects the slave by setting its CS line high.
 *
 ************************************************************************************/
static uint8_t AS5047P_DeselectSlave(uint16_t id)
{
  //--- Select GPIO and SPI handler
  if ( AS5047P_SelectSPIAndGPIO (id) < 0 )
  {   //--- Slave with this id doesn't exist
      return -1;
  }
  //--- CS line set to high level (line released)
  AS5047P_HAL_GPIO_Write(0x01);

  /************************************************************************************
  *
  *	 	!!! !!! CAUTION CAUTION CAUTION  !!! !!!
  * AS5047P needs a minimum delay of 350ns between RISING and FALLING endge of CS.
  * So fast CPUs (~180Mhz and more) may need a delay here.
  *
  ************************************************************************************/
  //delay_us(1);

  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_HAL_Init(void)
 * 	Access:		public
 * 	Inputs:
 *		id: ID number uniquely identifies an SPI's slave. Relation between
 *		slave's id and physical CS line is defined in AS5047P_SelectSPIAndGPIO.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 * 	Function used to initialize low level hardware (CS lines, GPIO, etc).
 *
 ************************************************************************************/
uint8_t AS5047P_HAL_Init(uint16_t id)
{
  //--- Deselect SPI slave
  if ( AS5047P_DeselectSlave (id) < 0 )
  {   //--- Slave with this id doesn't exist
      return -1;
  }

  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_HAL_SPI_TxRx(pTx,pRx)
 * 	Access:		public
 * 	Inputs:
 *		pTx: Pointer to two-byte field containing data to put on SPI MOSI line.
 *		pRx: Pointer to two-byte field to be filled with SPI MISO line data.
 *		id: ID number uniquely identifies an SPI's slave. Relation between
 *		slave's id and physical CS line is defined in AS5047P_SelectSPIAndGPIO.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 * 	Function is used for full-duplex SPI transactions as well as slave selection.
 *
 ************************************************************************************/
uint8_t AS5047P_HAL_SPI_Transaction(uint16_t * pTx, uint16_t * pRx, uint16_t id)
{
  uint8_t err;

  //--- Select SPI slave
  if ( AS5047P_SelectSlave (id) < 0 )
  {   //--- Slave with this id doesn't exist
      return -1;
  }

  err = AS5047P_HAL_SPI_TxRx(pTx,pRx);

  /************************************************************************************
  *
  *	 	!!! !!! CAUTION CAUTION CAUTION  !!! !!!
  * AS5047P needs a minimum delay of 1/2*SPI_sck period between FALLING edge of last SCK
  * and RISING edge of CS signal. So fast CPUs may need a delay here.
  *
  ************************************************************************************/
  //delay_us(1);

  //--- Slave deselection
  AS5047P_DeselectSlave(id);

  if( err < 0)
  {
      return -1;
  }

  return 0;
}
