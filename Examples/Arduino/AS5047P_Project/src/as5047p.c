/************************************************************************************
*
*	File:		as5047p.c
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

#include "as5047p_hal.h"
#include "as5047p.h"

#if AS5047P_SAVE_SPACE == 1
//--- Variable used for error message texts when memory space needs to be saved deactivated.
static char * saveSpaceMsg = "No txt available. Save space mode is active.";
#endif //AS5047P_SAVE_SPACE


#if AS5047P_SAVE_SPACE == 0
/************************************************************************************
 *
 * 	Variable: 	char errMesage[][]
 * 	Access:		private
 *
 * 	Description:
 *	This variable stores all strings related to error messages.
 *	The row number is also the error code number.
 *	This is a complete list of all possible encoder's errors.
 *
 ************************************************************************************/
static char errMesage [AS5047P_ERRMSG_COUNT][AS5047P_ERRMSG_MAX_LEN] =
{
    "No error pending",	// 	[0]
    "Low level SPI access returned error",	// 	[1]
    "Framing error occurred in last Tx frame",	// 	[2]
    "Parity bit error occurred in Rx frame",	// 	[3]
    "",	// 	[4]
    "",	// 	[5]
    "",	// 	[6]
    "",	// 	[7]
    "",	// 	[8]
    "",	// 	[9]
    "Register with a given address doesn\'t exist",	// 	[10]
    "Device not accessible on SPI line",	// 	[11]
    "",	// 	[12]
    "",	// 	[13]
    "",	// 	[14]
    "",	// 	[15]
    "",	// 	[16]
    "",	// 	[17]
    "",	// 	[18]
    "",	// 	[19]
    "Register with a given address doesn\'t have write access or doesn\'t exist",	// 	[20]
    "Register content verification failed after write operation",	// 	[21]
    "Device not accessible on SPI line",	// 	[22]
    "",	// 	[23]
    "",	// 	[24]
    "",	// 	[25]
    "",	// 	[26]
    "",	// 	[27]
    "",	// 	[28]
    "",	// 	[29]
    "Low level hardware (SPI) init failed",	// 	[30]
    "Cant acknowledge the error in ERRFL register in init routine",	// 	[31]
    "",	// 	[32]
    "",	// 	[33]
    "",	// 	[34]
    "",	// 	[35]
    "",	// 	[36]
    "",	// 	[37]
    "",	// 	[38]
    "",	// 	[39]
    "Encoder not initialized while setting factory settings attempt",	// 	[40]
    "",	// 	[41]
    "",	// 	[42]
    "",	// 	[43]
    "",	// 	[44]
    "",	// 	[45]
    "",	// 	[46]
    "",	// 	[47]
    "",	// 	[48]
    "",	// 	[49]
    "Cant acknowledge the error in ERRFL register in reset routine",	// 	[50]
    "Encoder not initialized while reset attempt",	// 	[51]
    "",	// 	[52]
    "",	// 	[53]
    "",	// 	[54]
    "",	// 	[55]
    "",	// 	[56]
    "",	// 	[57]
    "",	// 	[58]
    "",	// 	[59]
    "Encoder not initialized while set field in register attempt",	// 	[60]
    "",	// 	[61]
    "",	// 	[62]
    "",	// 	[63]
    "",	// 	[64]
    "",	// 	[65]
    "",	// 	[66]
    "",	// 	[67]
    "",	// 	[68]
    "",	// 	[69]
    "Device not accessible on SPI line",	// 	[70]
    "Magnetic field strength too low",	// 	[71]
    "Magnetic field strength too high",	// 	[72]
    "CORDIC overflow",	// 	[73]
    "Magnet offset compensation error",	// 	[74]
    "Encoder not initialized while position read attempt",	// 	[75]
    "Encoder not calibrated (zero pos not set) while position read attempt",	// 	[76]
    "",	// 	[77]
    "",	// 	[78]
    "",	// 	[79]
    "Encoder not initialized while zero position set attempt",	// 	[80]
    "",	// 	[81]
    "",	// 	[82]
    "",	// 	[83]
    "",	// 	[84]
    "",	// 	[85]
    "",	// 	[86]
    "",	// 	[87]
    "",	// 	[88]
    "",	// 	[89]
    "Chosen ABI resolution is not correct",	// 	[90]
    "Encoder not initialized while ABI resolution set attempt",	// 	[91]
    "",	// 	[92]
    "",	// 	[93]
    "",	// 	[94]
    "",	// 	[95]
    "",	// 	[96]
    "",	// 	[97]
    "",	// 	[98]
    "",	// 	[99]
    "Timeout fired while waiting for OTP burn finish",	// 	[100]
    "Guardband test failed while OTP burn. Reprogramming not allowed",	// 	[101]
    "Encoder not initialized while OTP burn attempt",	// 	[102]
    "",	// 	[103]
    "",	// 	[104]
    "",	// 	[105]
    "",	// 	[106]
    "",	// 	[107]
    "",	// 	[108]
    "",	// 	[109]
    "",	// 	[110]
    "",	// 	[111]
    "",	// 	[112]
    "",	// 	[113]
    "",	// 	[114]
    "",	// 	[115]
    "",	// 	[116]
    "",	// 	[117]
    "",	// 	[118]
    "",	// 	[119]
    "",	// 	[120]
    "",	// 	[121]
    "",	// 	[122]
    "",	// 	[123]
    "",	// 	[124]
    "",	// 	[125]
    "",	// 	[126]
    "",	// 	[127]
    "",	// 	[128]
    "",	// 	[129]
    "",	// 	[130]
    "",	// 	[131]
    "",	// 	[132]
    "",	// 	[133]
    "",	// 	[134]
    "",	// 	[135]
    "",	// 	[136]
    "",	// 	[137]
    "",	// 	[138]
    "",	// 	[139]

};
#endif //AS5047P_SAVE_SPACE

/************************************************************************************
 *
 * 	Function: 	AS5047P_CalcParity(v)
 * 	Access:		private
 * 	Inputs:
 * 		v: 32 bits for which parity is to be calculated.
 *
 * 	Returns:
 * 		0: Calculated parity is even.
 * 		1: Calculated parity is odd.
 *
 * 	Description:
 *	Function calculates parity bit for a given input v.
 *
 *	Calculations according to:
 * 	http://www.graphics.stanford.edu/~seander/bithacks.html
 *
 *	Generates no error.
 *
 ************************************************************************************/
static uint8_t AS5047P_CalcParity(uint32_t v)
{
  v ^= v >> 1;
  v ^= v >> 2;
  v = (v & 0x11111111U) * 0x11111111U;
  return (v >> 28) & 1;
}

/************************************************************************************
 *
 * 	Function:  	AS5047P_CountTrailZeros(v)
 * 	Access:		private
 * 	Inputs:
 * 		v: 32 bits for which trailing zeros is to be calculated.
 *
 * 	Returns:
 * 		Number of trailing zeros.
 *
 * 	Description:
 *	Count the consecutive zero bits (trailing) on the right with modulus
 *	division and lookup table. According to:
 * 	http://www.graphics.stanford.edu/~seander/bithacks.html
 *
 *	Generates no error.
 *
 ************************************************************************************/
static uint8_t AS5047P_CountTrailZeros(uint32_t v)
{
  int Mod37BitPosition[] = // map a bit value mod 37 to its position
  {
    32, 0, 1, 26, 2, 23, 27, 0, 3, 16, 24, 30, 28, 11, 0, 13, 4,
    7, 17, 0, 25, 22, 31, 15, 29, 10, 12, 6, 0, 21, 14, 9, 5,
    20, 8, 19, 18
  };

  return Mod37BitPosition[(-v & v) % 37];

}

/************************************************************************************
 *
 * 	Function: 	AS5047P_SetBitsAndShiftAndMask(fieldMask,fieldVal)
 * 	Access:		private
 * 	Inputs:
 * 		fieldMask: 	Bit mask where fieldVal is to be set.
 * 		fieldVal:	Value to be sen on fieldMask bits.
 *
 * 	Returns:
 * 		Function returns fieldVal masked with fieldMask and shifted to the
 * 		left by number of trailing zeros in fieldMask.
 *
 * 	Description:
 *	See output description.
 *
 *	Generates no error.
 *
 ************************************************************************************/
static uint16_t AS5047P_SetBitsAndShiftAndMask(uint16_t fieldMask, uint16_t fieldVal)
{
  return (fieldVal << AS5047P_CountTrailZeros(fieldMask) ) & fieldMask;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_IsParityOk(frameRx)
 * 	Access:		private
 * 	Inputs:
 * 		frameRx:	16-bit raw frame received from AS5047P.
 *
 * 	Returns:
 * 		true:		Parity check passed.
 * 		false:		Parity check failed.
 *
 * 	Description:
 *	Function as an input received raw frame received through SPI from AS5047P,
 *	then the parity bit is calculated for bits [0-14] and result compared to
 *	parity bit received (bit15).
 *
 *	Generates no error.
 *
 ************************************************************************************/
static _Bool AS5047P_IsParityOk(uint16_t frameRx)
{
  uint32_t parityReceived;
  uint32_t parityCalculated;

  //--- Calculate parity for Rx frame
  parityReceived = (frameRx & AS5047P_FRAME_PARD) >> 15;
  parityCalculated = AS5047P_CalcParity( frameRx & (AS5047P_FRAME_DATA | AS5047P_FRAME_EF) );

  //--- Parity check
  if ( parityCalculated == parityReceived )
    return true;
  else
    return false;

}

/************************************************************************************
 *
 * 	Function: 	AS5047P_HandleError(instance,errCode)
 * 	Access:		private
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 * 		errCode:	Error code to be set in the encoder's structure.
 *
 * 	Returns:
 * 		Nothing.
 *
 * 	Description:
 *	Function sets error code into the encoder's structure and (if AS5047P_USE_DEBUG
 *	defined) calls AS5047P_HAL_Debug() to do extra debug-related tasks (printf).
 *
 *	Generates no error.
 *
 ************************************************************************************/
static void AS5047P_HandleError(AS5047P_Instance * instance, int16_t errCode)
{
  //--- Allow only setting errors (no clear possible)
    if( (instance->error.errorCode == 0) && (errCode != 0) )
    {
	instance->error.errorCode = errCode;


	if(errCode < AS5047P_ERRMSG_COUNT)
	{

#if AS5047P_SAVE_SPACE == 1
	    instance->error.msg = saveSpaceMsg;
#else
	    instance->error.msg = errMesage[errCode];
#endif //AS5047P_SAVE_SPACE

#if AS5047P_USE_DEBUG == 1
	    AS5047P_HAL_Debug(instance->id,instance->error.errorCode, instance->error.msg);
#endif //AS5047P_USE_DEBUG

	}
	else
	{
	    instance->error.msg = NULL;
	}
    }
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_ClearError(instance)
 * 	Access:		private
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 * 		Nothing.
 *
 * 	Description:
 *	Function clears error code in the encoder's structure.
 *
 *	Generates no error.
 *
 ************************************************************************************/
static void AS5047P_ClearError(AS5047P_Instance * instance)
{

  instance->error.errorCode = 0;

#if AS5047P_SAVE_SPACE == 1
  instance->error.msg = saveSpaceMsg;
#else
  instance->error.msg = errMesage[0];
#endif //AS5047P_SAVE_SPACE

#if AS5047P_USE_DEBUG == 1
  AS5047P_HAL_Debug(instance->id,instance->error.errorCode, instance->error.msg);
#endif //AS5047P_USE_DEBUG
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_ReadWriteRaw(instance,dataTx,rw)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 * 		dataTx:		Raw 14bits of data to be sent to AS5047P.
 * 		rw:		read/write modifier needed by AS5047P.
 *
 * 	Returns:
 *		Data received or -1 (execution with errors).
 *
 * 	Description:
 *	Function handles Rx/Tx transaction with AS5047P. It prepares 16bit Tx frame
 *	combining dataTx, rw and the parity bit. Then it utilizes AS5047P_HAL_SPI_Trans-
 *	action(), low level function for SPI transaction.
 *
 *	Generates error codes in range: [1-9]
 *
 ************************************************************************************/
AS5047P_Result AS5047P_ReadWriteRaw(AS5047P_Instance * instance, AS5047P_Result dataTx, _Bool rw)
{
  uint8_t lowLevHwErr;
  AS5047P_Result frameTx;

  //--- Set RW bit and calculate parity bit
  frameTx = dataTx & AS5047P_FRAME_DATA;
  frameTx |= rw << 14;
  frameTx |= AS5047P_CalcParity(frameTx) << 15;

  instance->buffTx = frameTx;
  lowLevHwErr = AS5047P_HAL_SPI_Transaction( &(instance->buffTx), &(instance->buffRx), instance->id);

  //--- Low level SPI access returned error
  if( lowLevHwErr != 0 )
  {
      AS5047P_HandleError(instance,1);
      return -1;
  }
  //--- Framing error occurred in last Tx frame
  if( (instance->buffRx) & AS5047P_FRAME_EF)
  {
      AS5047P_HandleError(instance,2);
      return -1;
  }

  //--- Parity bit error occurred in Rx frame
  if ( !AS5047P_IsParityOk(instance->buffRx) )
  {
      AS5047P_HandleError(instance,3);
      return -1;
  }

  return ( (instance->buffRx) & AS5047P_FRAME_DATA );

}

/************************************************************************************
 *
 * 	Function: 	AS5047P_ReadRegister(instance,regAddr,devReachCheck)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 * 		regAddr:	Register's address to be read.
 * 		devReachCheck:	Device reachability check activation.
 *
 * 	Returns:
 *		Register content or -1 (execution with errors).
 *
 * 	Description:
 *	Function reads content of register given by address in regAddr input.
 *	It needs 2 Rx/Tx SPI transaction when devReachCheck == false or 3 Rx/Tx
 *	transactions if devReachCheck == true.
 *	Input devReachCheck allows to check if AS5047P is reachable on SPI line by
 *	requesting DIAAGC register content which is always non-zero.
 *
 *	Generates error codes in range: [10-19]
 *
 ************************************************************************************/
AS5047P_Result AS5047P_ReadRegister(AS5047P_Instance * instance, uint16_t regAddr, _Bool devReachCheck )
{

  AS5047P_Result cmdResponse;
  AS5047P_Result tmpResponse;

  //--- Register with a given address doesn\'t exist
  if(!( (regAddr == AS5047P_NOP ) || (regAddr == AS5047P_ERRFL )  || (regAddr == AS5047P_PROG ) || (regAddr == AS5047P_DIAAGC ) || (regAddr == AS5047P_MAG )\
      || (regAddr == AS5047P_ANGLEUNC ) || (regAddr == AS5047P_ANGLECOM ) || (regAddr == AS5047P_ZPOSL ) || (regAddr == AS5047P_ZPOSM ) || (regAddr == AS5047P_SETTINGS1 )\
      || (regAddr == AS5047P_SETTINGS2 )) )
  {
      AS5047P_HandleError(instance,10);
      return -1;
  }

  /************************************************************************************
   *
   * STEP 1: Transmit COMMAND AND receive DUMMY RESPONSE.
   * 	     Ignore response and framing errors.
   *
   ************************************************************************************/

  AS5047P_ReadWriteRaw(instance, regAddr, AS5047P_ACCESS_READ);

  /************************************************************************************
   *
   * STEP 2: Transmit DIAAGC command AND receive COMMAND RESPONSE.
   * 	     DIAAGC register chosen because it's always non-zero value.
   * 	     Then check if no framing error occurred.
   * 	     Then check parity.
   *
   ************************************************************************************/

  cmdResponse = AS5047P_ReadWriteRaw(instance, AS5047P_DIAAGC , AS5047P_ACCESS_READ);

  //--- Framing error occurred in Tx or Rx frame
  if( cmdResponse < 0 )
  {

      return -1;
  }

  /************************************************************************************
   * STEP IS EXECUTED ONLY IF CHECK IS ENABLED (input devReachCheck) !
   * STEP 3: Transmit NOP command AND receive DIAAGC RESPONSE.
   * 	     Then check if no framing error occurred.
   * 	     Then check parity.
   * 	     Then check if response is non-zero. If this is not the case - as5047p or SPI
   * 	     is faulty.
   *
   ************************************************************************************/
  if( devReachCheck )
  {
    tmpResponse = AS5047P_ReadWriteRaw(instance, AS5047P_NOP , AS5047P_ACCESS_READ);

    //--- Framing error occurred in Tx or Rx frame
    if( tmpResponse < 0 )
    {

	return -1;
    }

    //--- Device not accessible on SPI line
    if( tmpResponse == 0 )
    {
	AS5047P_HandleError(instance,11);
	return -1;
    }
  }

  return cmdResponse;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_WriteRegister(instance,regAddr,newRegContent,
 * 					      writeVerif,devReachCheck)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 * 		regAddr:	Register's address to be written.
 * 		newRegContent:	The Content that is to be written do the register.
 * 		writeVerif:	Write verification activation.
 * 		devReachCheck:	Device reachability check activation.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function writes content given by newRegContent input to the register given
 *	by regAddr address.
 *	Normally, if writeVerif == false AND devReachCheck == false
 *	only 2 Rx/Tx SPI transactions are needed. writeVerif == true as well as
 *	devReachCheck == true each need one additional Rx/Tx SPI transaction.
 *
 *	Input devReachCheck allows to check if AS5047P is reachable on SPI line by
 *	requesting DIAAGC register content which is always non-zero.
 *
 *	Input writeVerif causes that additional read request is performed allowing
 *	content verification after write.
 *
 *	Generates error codes in range: [20-29]
 *
 ************************************************************************************/
AS5047P_ErrCode AS5047P_WriteRegister(AS5047P_Instance * instance, uint16_t regAddr, AS5047P_Result newRegContent, _Bool writeVerif, _Bool devReachCheck )
{
    AS5047P_Result tmpResponse;
    AS5047P_Result curRegContent;

    //--- Register with a given address doesn\'t have write access or doesn\'t exist
    if(!( (regAddr == AS5047P_PROG ) || (regAddr == AS5047P_ZPOSL )  || (regAddr == AS5047P_ZPOSM ) || (regAddr == AS5047P_SETTINGS1 ) || (regAddr == AS5047P_SETTINGS2 )) )
    {
	AS5047P_HandleError(instance,20);
	return -1;
    }

    /************************************************************************************
    *
    * STEP 1: Transmit COMMAND and receive DUMMY RESPONSE.
    * 	     Ignore response and EF flag.
    *
    *
    ************************************************************************************/

    AS5047P_ReadWriteRaw(instance, regAddr, AS5047P_ACCESS_WRITE);

    /************************************************************************************
    *
    * STEP 2: Transmit NEW CONTENT and receive PREVIOUS REGISTER CONTENT.
    * 	     Then check if no framing error occurred.
    *
    ************************************************************************************/

    tmpResponse = AS5047P_ReadWriteRaw(instance, newRegContent, AS5047P_ACCESS_WRITE);

    //--- Framing error occurred in Tx or Rx frame
    if( tmpResponse < 0 )
    {

	return -1;
    }

    /************************************************************************************
    * STEP IS EXECUTED ONLY IF CONTENT VERIFICATION IS REQUESTED (input writeVerif) !
    *
    * STEP 3: Transmit DIAAGC AND receive CURRENT REGISTER CONTENT.
    * 	     Then check if no framing error occurred.
    * 	     Then check parity.
    * 	     Then check if current content == user requested content
    *
    ************************************************************************************/
    if(writeVerif)
    {

      curRegContent = AS5047P_ReadWriteRaw(instance, AS5047P_DIAAGC , AS5047P_ACCESS_READ);

      //--- Framing error occurred in Tx or Rx frame
      if( curRegContent < 0 )
      {

	  return -1;
      }

      //--- For SETTINGS1 register bit0 is factory programmed to 1 (see datasheet)
      if(regAddr == AS5047P_SETTINGS1 )
      {
	  //--- Set Bit0 to 1 (factory preprogrammed to 1)
	  newRegContent = newRegContent | 0x0001;
      }

      //--- Register content verification failed after write operation
      if ( curRegContent != newRegContent )
      {
	  AS5047P_HandleError(instance,21);
	  return -1;
      }

    }

    /************************************************************************************
     * STEP IS EXECUTED ONLY IF CHECK IS ENABLED (input devReachCheck) !
     * STEP 4: Transmit NOP command AND receive DIAAGC RESPONSE.
     * 	     Then check if no framing error occurred.
     * 	     Then check parity.
     * 	     Then check if response is non-zero. If this is not the case - as5047p or SPI
     * 	     is faulty.
     *
     ************************************************************************************/
    if( devReachCheck )
    {
      tmpResponse = AS5047P_ReadWriteRaw(instance, AS5047P_NOP , AS5047P_ACCESS_READ);

      //--- Framing error occurred in Tx or Rx frame
      if( tmpResponse < 0 )
      {

	  return -1;
      }

      //--- Device not accessible on SPI line
      if( tmpResponse == 0 )
      {
	  AS5047P_HandleError(instance,22);
	  return -1;
      }
    }

    return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_Init(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *		id: ID number uniquely identifies an SPI's slave. Relation between
 *		slave's id and physical CS line is defined in file as5047p_hal.c .
 * 	Returns:
 *		-1 : Execution with errors or ERRFL errors flush not possible
 *		 0 : Success.
 *
 * 	Description:
 *	Function initializes low level hardware, instance's structure, and clears all
 *	pending errors in ERRFL register.
 *
 *	Generates error codes in range: [30-39]
 *
 ************************************************************************************/

AS5047P_ErrCode AS5047P_Init(AS5047P_Instance * instance, uint16_t id)
{
  AS5047P_Result readRegErrCode;
  AS5047P_Result halInitErrCode;

  instance->zeroPosCalibrated = false;
  instance->initialized = true;
  instance->id = id;

  /************************************************************************************
   *
   *	AS5047P needs 10ms after power up to recover
   *
   ***********************************************************************************/
  AS5047P_HAL_Delay_ms(10);

  //--- Init HAL functions (SPI, etc.)
  halInitErrCode = AS5047P_HAL_Init(id);

  //--- Low level hardware (SPI) init failed
  if( halInitErrCode != 0)
  {
      AS5047P_HandleError(instance,30);
      return -1;
  }

  //--- Flush error register
  AS5047P_ReadRegister(instance, AS5047P_ERRFL ,AS5047P_OPT_ENABLED); // Clears errors

  //--- Read error register value after clear. Must be zero.
  readRegErrCode = AS5047P_ReadRegister(instance, AS5047P_ERRFL ,AS5047P_OPT_ENABLED);

  //--- Error register read failed
  if( readRegErrCode < 0)
  {

      return -1;
  }

  //--- Cant acknowledge the error in ERRFL register in init routine
  if( readRegErrCode > 0)
  {
      AS5047P_HandleError(instance,31);
      return -1;
  }

  AS5047P_ClearError(instance);


  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_SetFactorySettings(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function sets SETTINGS1, SETTINGS2, ZPOSL, ZPOSM registers to their factory
 *	values (before OTP burn).
 *
 *	Generates error codes in range: [40-49]
 *
 ************************************************************************************/
AS5047P_ErrCode AS5047P_SetFactorySettings(AS5047P_Instance * instance)
{
    AS5047P_ErrCode errCode = 0;

    if(instance->initialized)
    {
	errCode = AS5047P_WriteRegister(instance, AS5047P_SETTINGS1, 0x0001, AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);
	errCode |= AS5047P_WriteRegister(instance, AS5047P_SETTINGS2, 0x000, AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);
	errCode |= AS5047P_WriteRegister(instance, AS5047P_ZPOSL, 0x000, AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);
	errCode |= AS5047P_WriteRegister(instance, AS5047P_ZPOSM, 0x000, AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);

        if( errCode != 0)
        {

  	  return -1;
        }

    }
    else
    {
        //--- Encoder not initialized while setting factory settings attempt
        AS5047P_HandleError(instance,40);
        return -1;
    }

    return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_ErrorPending(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		true : Error is pending
 *		false: No error.
 *
 * 	Description:
 *	Function returns true if error is pending (wainting for acknowledge) or false
 *	otherwise.
 *
 *	Generates no error.
 *
 ************************************************************************************/
_Bool AS5047P_ErrorPending(AS5047P_Instance * instance)
{
  return (instance->error).errorCode != 0 ? true : false;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_GetError(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		Error structure.
 *
 * 	Description:
 *	Function returns private member 'error'.
 *
 *	Generates no error.
 *
 ************************************************************************************/
AS5047P_Error AS5047P_GetError(AS5047P_Instance * instance)
{
  return (instance->error);
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_ErrorAck(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function acknowledges currently pending error.
 *
 *	Generates error codes in range: [50-59]
 *
 ************************************************************************************/
AS5047P_ErrCode AS5047P_ErrorAck(AS5047P_Instance * instance)
{
  AS5047P_Result readRegErrCode;

  if(instance->initialized)
  {
      if( AS5047P_ErrorPending(instance))
      {
	//--- Flush error register
	AS5047P_ReadRegister(instance, AS5047P_ERRFL ,AS5047P_OPT_ENABLED); // Clears errors
	readRegErrCode = AS5047P_ReadRegister(instance, AS5047P_ERRFL ,AS5047P_OPT_ENABLED); // Gets error register value after clear. Must be zero.

	//--- AS5047P's error register flush failed
	if( readRegErrCode < 0)
	{
	    return -1;
	}

	//--- Cant acknowledge the error in ERRFL register in reset routine
	if( readRegErrCode > 0)
	{
	    AS5047P_HandleError(instance,50);
	    return -1;
	}

	AS5047P_ClearError(instance);
      }

  }
  else
  {
      //--- Encoder not initialized while reset attempt
      AS5047P_HandleError(instance,51);
      return -1;
  }

  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_SetFieldInRegister(instance,regAddr,fieldMask,fieldVal)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function sets SINGLE field in a register given by regAddr address. Field that
 *	is to be set is given by its mask fieldMask, and value that is to be assigned
 *	is given by fieldVal.
 *
 *
 *	Generates error codes in range: [60-69]
 *
 ************************************************************************************/
AS5047P_ErrCode AS5047P_SetFieldInRegister(AS5047P_Instance * instance, uint16_t regAddr, uint16_t fieldMask, uint16_t fieldVal)
{
  AS5047P_Result regContent;
  AS5047P_ErrCode errCode = 0;

  if(instance->initialized)
  {
     /************************************************************************************
     *
     * STEP 1:  Read current register content - need to keep N/A bits unchanged.
     * 	 	Modify requested bits.
     *
     ************************************************************************************/

    regContent = AS5047P_ReadRegister(instance, regAddr,AS5047P_OPT_ENABLED);

    if(regContent < 0 )
    {

	return -1;
    }

    regContent &= (~fieldMask); // Clear field bits
    regContent |= AS5047P_SetBitsAndShiftAndMask(fieldMask, fieldVal); // Set relevant shifted and masked bits

    /************************************************************************************
    *
    * STEP 2:  Write new register content.
    *
    ************************************************************************************/

    errCode = AS5047P_WriteRegister(instance, regAddr,regContent,AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);

    if(errCode < 0 )
    {

	return -1;
    }

  }
  else
  {
      //--- Encoder not initialized while set field in register attempt
      AS5047P_HandleError(instance,60);
      return -1;
  }

  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_ReadPosition(instance,devReachCheck)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *		extendedDiag:	Extended diagnostics activation.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function reads current encoder's position from AS5047P_ANGLECOM register.
 *	Normally, if extendedDiag == false only 2 Rx/Tx SPI transactions are
 *	needed. if extendedDiag == true, 3 Rx/Tx SPI transactions are needed.
 *	Before requesting position, encoder must be calibrated (zero position set)
 *	calling AS5047P_SetZeroPosition() function.
 *	Input 'extendedDiag' allows to check if AS5047P is reachable on SPI line by
 *	requesting DIAAGC register content which is always non-zero and additional
 *	errors related to magnetic field are checked.
 *
 *	Generates error codes in range: [70-79]
 *
 ************************************************************************************/
AS5047P_Result AS5047P_ReadPosition(AS5047P_Instance * instance, _Bool extendedDiag)
{
  AS5047P_Result currPos;
  AS5047P_Result diagData;

  if(instance->initialized && instance->zeroPosCalibrated)
  {

      /************************************************************************************
       *
       * STEP 1: Transmit COMMAND AND receive DUMMY RESPONSE.
       * 	 Ignore response and framing errors.
       *
       ************************************************************************************/

      AS5047P_ReadWriteRaw(instance, AS5047P_ANGLECOM, AS5047P_ACCESS_READ);

      /************************************************************************************
       *
       * STEP 2: Transmit DIAAGC command AND receive COMMAND RESPONSE.
       * 	 DIAAGC register chosen because it's always non-zero value.
       * 	 Then check if no framing error occurred.
       * 	 Then check parity.
       *
       ************************************************************************************/

      currPos = AS5047P_ReadWriteRaw(instance, AS5047P_DIAAGC , AS5047P_ACCESS_READ);

      //--- Framing error occurred in Tx or Rx frame
      if( currPos < 0 )
      {
	  return -1;
      }

      if (extendedDiag)
      {
	/************************************************************************************
	 *
	 * STEP 3:  Transmit NOP command AND receive DIAAGC RESPONSE.
	 * 	    Then check if no framing error occurred.
	 * 	    Then check if response is non-zero. If this is not the case - as5047p or SPI
	 * 	    is faulty (device reachability check).
	 * 	    Check errors related to magnetic field
	 *
	 ************************************************************************************/

	diagData = AS5047P_ReadWriteRaw(instance, AS5047P_NOP , AS5047P_ACCESS_READ);

	//--- Framing error or parity error occured
	if( diagData < 0 )
	{
	    return -1;
	}

	//--- Device not accessible on SPI line
	if( diagData == 0 )
	{
	    AS5047P_HandleError(instance,70);
	    return -1;
	}

	//--- Magnetic field strength too low
	if( diagData & AS5047P_DIAAGC_MAGL )
	{
	  AS5047P_HandleError(instance,71);
	  return -1;
	}

	//--- Magnetic field strength too high
	if( diagData & AS5047P_DIAAGC_MAGH )
	{
	  AS5047P_HandleError(instance,72);
	  return -1;
	}

	//--- CORDIC overflow
	if( diagData & AS5047P_DIAAGC_COF )
	{
	  AS5047P_HandleError(instance,73);
	  return -1;
	}

	//--- Magnet offset compensation error
	if( ( diagData & AS5047P_DIAAGC_LF ) == 0 )
	{
	  AS5047P_HandleError(instance,74);
	  return -1;
	}
      }

  }
  else if( !instance->initialized )
  {
      //--- Encoder not initialized while position read attempt
      AS5047P_HandleError(instance,75);
      return -1;
  }
  else if( !instance->zeroPosCalibrated )
  {
      //--- Encoder not calibrated (zero pos not set) while position read attempt
      AS5047P_HandleError(instance,76);
      return -1;
  }

  return currPos;

}

/************************************************************************************
 *
 * 	Function: 	AS5047P_SetZeroPosition(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function clears current encoder's calibration (if any) from ZPOSL and
 *	ZPOSM registers, reads current (not calibrated) position and writes back
 *	calibration to ZPOSL and ZPOSM respectively. Other ZPOSL non-calibration
 *	related bits are left unchanged.
 *
 *	Generates error codes in range: [80-89]
 *
 ************************************************************************************/
AS5047P_ErrCode AS5047P_SetZeroPosition(AS5047P_Instance * instance)
{

  AS5047P_Result currPosition;
  AS5047P_Result currZPOSLContent;
  AS5047P_ErrCode errCode = 0;
  uint16_t newZPOSLContent;
  uint16_t newZPOSMContent;

  if(instance->initialized)
  {
      /************************************************************************************
      *
      * STEP 1:  Read current ZPOSL reg. content - need to keep comp_i_err_en and and
      * 	 comp_h_err_en bits unchanged.
      *
      ************************************************************************************/

     currZPOSLContent = AS5047P_ReadRegister(instance, AS5047P_ZPOSL ,AS5047P_OPT_ENABLED);

     if(currZPOSLContent < 0 )
     {

	return -1;
     }

     /************************************************************************************
     *
     * STEP 2:  Write 6 least significant bits of ZPOSL to default (0) value.
     * 	 	Keep comp_i_err_en and comp_h_err_en bits unchanged.
     * 	 	Write all bits of ZPOSM to default (0) value.
     *
     ************************************************************************************/

     //--- Keep comp_i_err_en and and comp_h_err_en bits unchanged. The rest set to 0
     errCode = AS5047P_WriteRegister(instance, AS5047P_ZPOSL , currZPOSLContent & (AS5047P_ZPOSL_COMP_I_ERR_EN | AS5047P_ZPOSL_COMP_H_ERR_EN),AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);
     errCode |= AS5047P_WriteRegister(instance, AS5047P_ZPOSM , 0x0000,AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);

     if(errCode != 0 )
     {

	return -1;
     }

     //--- This delay here is crucial before reading the position !!!!!!!!!!!!!!!!!!!!!!
     AS5047P_HAL_Delay_ms(1);

    /************************************************************************************
     *
     * STEP 3: 	Read current encoder position.
     *
     ************************************************************************************/

    currPosition = AS5047P_ReadRegister(instance, AS5047P_ANGLECOM ,AS5047P_OPT_ENABLED);

    //--- Position reading failed
    if(currPosition < 0 )
    {

      return -1;
    }

    /************************************************************************************
     *
     * STEP 4: 	Write 6 least significant bits of the current position to ZPOSL register.
     * 	 	Keep comp_i_err_en and comp_h_err_en unchanged.
     * 	 	Write 8 most significant bits of the current position to ZPOSM register.
     *
     ************************************************************************************/

    //--- Keep comp_i_err_en and and comp_h_err_en bits unchanged and take 6 (of 14) least significant bits of currPosition (14 bits data).
    newZPOSLContent = (currZPOSLContent & (AS5047P_ZPOSL_COMP_I_ERR_EN | AS5047P_ZPOSL_COMP_H_ERR_EN) ) | (currPosition & AS5047P_ZPOSL_ZPOSL);
    //--- Take 8 (of 14) most significant bits of currPosition (14 bits data).
    newZPOSMContent = (currPosition >> 6 ) & AS5047P_ZPOSM_ZPOSM;

    errCode = AS5047P_WriteRegister(instance, AS5047P_ZPOSL , newZPOSLContent,AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);
    errCode |= AS5047P_WriteRegister(instance, AS5047P_ZPOSM , newZPOSMContent,AS5047P_OPT_ENABLED,AS5047P_OPT_ENABLED);

    if(errCode != 0 )
    {

      return -1;
    }

    instance->zeroPosCalibrated = true;

    //--- This delay here is crucial before reading the position !!!!!!!!!!!!!!!!!!!!!!
    AS5047P_HAL_Delay_ms(1);

  }
  else
  {
      //--- Encoder not initialized while zero position set attempt
      AS5047P_HandleError(instance,80);
      return -1;
  }

  return 0;

}

/************************************************************************************
 *
 * 	Function: 	AS5047P_SetABIResolution(instance,resolution)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *		resolution:	Resolution to set. It accepts value of:
 *		100/200/400/800/1200/1600/2000/4000/1024/2048/4096 steps per revolution.
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function sets ABI resolution given by input 'resolution'. It operates on
 *	ABIBIN (SETTINGS1) and ABIRES (SETTINGS2) fields.
 *
 *	Generates error codes in range: [90-99]
 *
 ************************************************************************************/
AS5047P_ErrCode AS5047P_SetABIResolution(AS5047P_Instance * instance, uint16_t resolution)
{
  AS5047P_ErrCode errCode = 0;
  uint16_t abibin;
  uint16_t abires;

  //--- Check if resolution is on the list
  switch (resolution)
  {
    case AS5047P_ABIRES_100:
      abires = 7;
      break;
    case AS5047P_ABIRES_200:
      abires = 6;
      break;
    case AS5047P_ABIRES_400:
      abires = 5;
      break;
    case AS5047P_ABIRES_800:
      abires = 4;
      break;
    case AS5047P_ABIRES_1200:
      abires = 3;
      break;
    case AS5047P_ABIRES_1600:
      abires = 2;
      break;
    case AS5047P_ABIRES_2000:
      abires = 1;
      break;
    case AS5047P_ABIRES_4000:
      abires = 0;
      break;
    case AS5047P_ABIRES_1024:
      abires = 2;
      break;
    case AS5047P_ABIRES_2048:
      abires = 1;
      break;
    case AS5047P_ABIRES_4096:
      abires = 0;
      break;

      //--- Chosen ABI resolution is not correct
    default:
	AS5047P_HandleError(instance,90);
      return -1;

  }

  //--- ABIBIN = 1 only when resolution 1024 or 2048 or 4096 SPR needed. Otherwise = 0.
  abibin = (resolution % 100) ? 1 : 0;

  if(instance->initialized)
  {

    /************************************************************************************
    *
    * STEP 1:  Set ABIBIN bit.
    *
    ************************************************************************************/
    errCode = AS5047P_SetFieldInRegister(instance, AS5047P_SETTINGS1 , AS5047P_SETTINGS1_ABIBIN, abibin);

     if(errCode < 0 )
     {

	return -1;
     }

    /************************************************************************************
    *
    * STEP 2:  Set ABIRES bits.
    *
    ************************************************************************************/

    errCode = AS5047P_SetFieldInRegister(instance, AS5047P_SETTINGS2 , AS5047P_SETTINGS2_ABIRES, abires);

    if(errCode < 0 )
    {

	return -1;
    }
  }
  else
  {
      //--- Encoder not initialized while ABI resolution set attempt
      AS5047P_HandleError(instance,91);
      return -1;
  }

  return 0;

}

/************************************************************************************
 *
 * 	Function: 	AS5047P_BurnOTP(instance,yesImSure)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *		yesImSure:	Confirmation that OTP burn is to take place.
 *				Must be set to 666.
 *
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function writes permanently current content of four registers: SETTINGS1,
 *	SETTINGS2, ZPOSL, ZPOSM. When OTP burn process is finished, registers are
 *	filled with zeros and then refreshed with data written to non-volatile OTP
 *	memory. After that comparison takes place to check if data (old and new) match.
 *
 *	Generates error codes in range: [100-109]
 *
 ************************************************************************************/
/************************************************************************************
*				IMPORTANT!
*
* 	 Remember to cycle the power and then check content of non-volatile registers.
* 	 Only then you can be sure that OTP burn succeeded.
*
************************************************************************************/
AS5047P_ErrCode AS5047P_BurnOTP(AS5047P_Instance * instance, uint16_t yesImSure)
{
  AS5047P_ErrCode errCode = 0;
  AS5047P_Result response;
  AS5047P_Result oldZPOSM, oldZPOSL, oldSETTINGS1, oldSETTINGS2;
  AS5047P_Result newZPOSM, newZPOSL, newSETTINGS1, newSETTINGS2;
  uint16_t loopCntr = 0;

  if( yesImSure == 666 )
  {
    if(instance->initialized)
    {
	/************************************************************************************
	*
	* STEP 1: Read current contents of ZPOS, ZPOSL, SETTINGS1 and SETTINGS2 registers
	* 	for verification later.
	*
	************************************************************************************/
	oldZPOSM = AS5047P_ReadRegister(instance, AS5047P_ZPOSM ,AS5047P_OPT_ENABLED);
	oldZPOSL = AS5047P_ReadRegister(instance, AS5047P_ZPOSL ,AS5047P_OPT_ENABLED);
	oldSETTINGS1 = AS5047P_ReadRegister(instance, AS5047P_SETTINGS1 ,AS5047P_OPT_ENABLED);
	oldSETTINGS2 = AS5047P_ReadRegister(instance, AS5047P_SETTINGS2 ,AS5047P_OPT_ENABLED);

	//--- Registers reading failed
	if( (oldZPOSM < 0) || (oldZPOSL < 0) || (oldSETTINGS1 < 0) || (oldSETTINGS2 < 0) )
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 2: Set bit0 of SETTINGS1 to 0.
	* 	Datasheet:
	*  	For programming it's mandatory to clear bit0 (although it's read-only)
	*
	************************************************************************************/

	errCode = AS5047P_WriteRegister(instance, AS5047P_SETTINGS1 , (~AS5047P_SETTINGS1_BIT0) & oldSETTINGS1, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);

	if( errCode < 0)
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 3: Unlock OTParea for burning
	* 	(PROGEN=1)
	*
	************************************************************************************/

	errCode = AS5047P_WriteRegister(instance, AS5047P_PROG , AS5047P_PROG_PROGEN, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);

	if( errCode < 0)
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 4: Start OTP burning procedure
	* 	(PROGOTP=1)
	*
	************************************************************************************/

	errCode = AS5047P_WriteRegister(instance, AS5047P_PROG , AS5047P_PROG_PROGOTP, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);

	if( errCode < 0)
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 5: Wait for OTP burning procedure complete.
	* 	Reg(0x0003) == 0x01
	*
	************************************************************************************/
	do{

	    response = AS5047P_ReadRegister(instance, AS5047P_PROG , AS5047P_OPT_DISABLED);

	    if( response < 0)
	    {

		return -1;
	    }

	    //--- OTP burn finished
	    if (response == 0x0001)
	    {
		break;
	    }

	    //--- Wait for OTP burn finish
	    if (loopCntr < 2000)
	    {
		loopCntr++;
	    }
	    else //--- Timeout fired while waiting for OTP burn finish
	    {
		AS5047P_HandleError(instance,100);
		return -1;
	    }

	    AS5047P_HAL_Delay_ms(1);

	}while( 1 );

	/************************************************************************************
	*
	* STEP 6: Fill ZPOS, ZPOSL, SETTINGS1 and SETTINGS2 registers with zeros.
	*
	************************************************************************************/
	errCode = AS5047P_WriteRegister(instance, AS5047P_ZPOSM , 0x0000, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);
	errCode |= AS5047P_WriteRegister(instance, AS5047P_ZPOSL , 0x0000, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);
	errCode |= AS5047P_WriteRegister(instance, AS5047P_SETTINGS1 , 0x0000, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);
	errCode |= AS5047P_WriteRegister(instance, AS5047P_SETTINGS2 , 0x0000, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);

	//--- Registers writing failed
	if( errCode != 0 )
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 7: Set Guardband
	*
	************************************************************************************/

	errCode = AS5047P_WriteRegister(instance, AS5047P_PROG , AS5047P_PROG_PROGVER, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);

	if( errCode < 0)
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 8: Refresh non-volatile memory with OTP content
	*
	************************************************************************************/

	errCode = AS5047P_WriteRegister(instance, AS5047P_PROG , AS5047P_PROG_OTPREF, AS5047P_OPT_DISABLED, AS5047P_OPT_DISABLED);

	if( errCode < 0)
	{

	    return -1;
	}

	AS5047P_HAL_Delay_ms(1);

	/************************************************************************************
	*
	* STEP 9: Read current contents of ZPOS, ZPOSL, SETTINGS1 and SETTINGS2 registers
	*
	************************************************************************************/
	newZPOSM = AS5047P_ReadRegister(instance, AS5047P_ZPOSM , AS5047P_OPT_DISABLED);
	newZPOSL = AS5047P_ReadRegister(instance, AS5047P_ZPOSL , AS5047P_OPT_DISABLED);
	newSETTINGS1 = AS5047P_ReadRegister(instance, AS5047P_SETTINGS1 , AS5047P_OPT_DISABLED);
	newSETTINGS2 = AS5047P_ReadRegister(instance, AS5047P_SETTINGS2 , AS5047P_OPT_DISABLED);

	//--- Registers reading failed
	if( (newZPOSM < 0) || (newZPOSL < 0) || (newSETTINGS1 < 0) || (newSETTINGS2 < 0) )
	{

	    return -1;
	}

	/************************************************************************************
	*
	* STEP 10: Final comparison: old to new.
	* 	 Mask out bit0 in register SETTINGS1.
	*
	************************************************************************************/

	//--- Guardband test failed while OTP burn. Reprogramming not allowed
	if( (oldZPOSM != newZPOSM) || (oldZPOSL != newZPOSL) || ( ((~AS5047P_SETTINGS1_BIT0) & oldSETTINGS1) != ((~AS5047P_SETTINGS1_BIT0) & newSETTINGS1) ) || (oldSETTINGS2 != newSETTINGS2) )
	{
	    AS5047P_HandleError(instance,101);
	    return -1;
	}

    }
    else
    {
	//--- Encoder not initialized while OTP burn attempt
	AS5047P_HandleError(instance,102);
	return -1;
    }
  }

  return 0;
}

/************************************************************************************
 *
 * 	Function: 	AS5047P_GetID(instance)
 * 	Access:		public
 * 	Inputs:
 * 		instance:	Pointer to the encoder's instance.
 *
 * 	Returns:
 *		ID number of the instance given by 'instance' input.
 *
 * 	Description:
 *	Function return an ID number of the instance given by 'instance' input.
 *
 ************************************************************************************/
uint16_t AS5047P_GetID(AS5047P_Instance * instance)
{
  return instance->id;
}
