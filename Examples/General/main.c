/************************************************************************************
*
*	File:		main.c
*	Author:		Krzysztof Sawicki
*	Brief:		AS5047P driver - general usage example.
*
*	Description:
*	This code shows how to use AS5047P driver in general. It's non-compilable.
*
*************************************************************************************/

#include "as5047p.h"

  AS5047P_Instance encInstanceA = {0};
  AS5047P_Result encPositionA;

int main(void)
{
   //--- Initialize GPIO
   GPIO_Init();
   
   //--- Initialize SPI
   SPI_Init();
   
   //--- Initialize encoder and bind id number to the instance.
   AS5047P_Init(&encInstanceA, 0); // Bind encoder with id = 0

   //--- Set registers SETTINGS1, SETTINGS2, ZPOSL, ZPOSM to their factory defaults
   //--- (in this way ignoring OTP memory loaded values).
   AS5047P_SetFactorySettings(&encInstanceA);

   //--- Set current encoder position as new zero (AS5047P_ZPOSL, AS5047P_ZPOSM)
   AS5047P_SetZeroPosition(&encInstanceA);

   //--- Change ABI resolution to 4096 Steps per revolution (ABIRES, ABIBIN)
   AS5047P_SetABIResolution(&encInstanceA, AS5047P_ABIRES_4096);

   /***************************************************************************
    * The rest of configuration is done by working directly on registers' contents.
    ***************************************************************************/

   //--- Set field UVWPP of register AS5047P_SETTINGS2 to value 3 (meaning 4 pole pairs - datasheet).
   AS5047P_SetFieldInRegister(&encInstanceA, AS5047P_SETTINGS2 , AS5047P_SETTINGS2_UVWPP, 3);

   //--- Set fields COMP_I_ERR_EN and COMP_H_ERR_EN of register AS5047P_ZPOSL to 1
   //--- (meaning: enable the contribution of MAGH and MAGL to the error flag.
   AS5047P_SetFieldInRegister(&encInstanceA, AS5047P_ZPOSL , AS5047P_ZPOSL_COMP_I_ERR_EN, 1);
   AS5047P_SetFieldInRegister(&encInstanceA, AS5047P_ZPOSL , AS5047P_ZPOSL_COMP_H_ERR_EN, 1);

   //--- Set field PWMon of register AS5047P_SETTINGS1 to 1 (meaning: turn on PWM output).
   AS5047P_SetFieldInRegister(&encInstanceA, AS5047P_SETTINGS1 , AS5047P_SETTINGS1_PWMON, 1);

   //--- Set field HYS of register AS5047P_SETTINGS2 to 3 (meaning: ...).
   AS5047P_SetFieldInRegister(&encInstanceA, AS5047P_SETTINGS2 , AS5047P_SETTINGS2_HYS, 3);

   //--- Any of the above returned error.
   if ( AS5047P_ErrorPending(&encInstanceA) )
   {
       while(1)
       {}// Sorry Joe, no luck today.
   }

   //--- Burn OTP and verify. Current content of registers
   //--- is burnt to OTP memory. Second input is enable signal,
   //--- must be 666, otherwise OTP burn WILL NOT take place.
   AS5047P_BurnOTP(&encInstanceA, 000); // Replace '000' with '666' if you're really sure to burn OTP.

   while (1)
   {
     encPositionA = AS5047P_ReadPosition(&encInstanceA, AS5047P_OPT_ENABLED);

     if( !AS5047P_ErrorPending(&encInstanceA) )
     {
		//--- Print current position
		printf("Encoder ID[%d]> Position: %d\n",AS5047P_GetID(&encInstanceA), encPositionA);
     }
     else
     {
		//--- Print error message
		printf("Encoder ID[%d]> Error %d: %s\n",AS5047P_GetID(&encInstanceA),AS5047P_GetError(&encInstanceA).errorCode, AS5047P_GetError(&encInstanceA).msg);
     }

     //--- Acknowledge error if user button pressed
     if( UserButtonPressed )
     {
		AS5047P_ErrorAck(&encInstanceA);
     }

     //--- Delay 100ms (for printf)
     delay_ms(100);

   }

}

