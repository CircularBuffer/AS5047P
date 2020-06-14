#include "src/as5047p.h"
#include "SPI.h"

AS5047P_Instance encInstanceA;
AS5047P_Result encPositionA;

void setup() {
  // put your setup code here, to run once:

  // EncoderA CS pin
  pinMode(9,OUTPUT);  

  // SPI Init
  SPI.begin();
  
   //--- Initialize encoder and bind id number to the instance.
   AS5047P_Init(&encInstanceA, 0); // Bind encoder with id = 0
  //AS5047P_Init(&encInstanceB, 1); // Bind encoder with id = 1
  //AS5047P_Init(&encInstanceC, 2); // Bind encoder with id = 2 and so on...

   //--- Set registers SETTINGS1, SETTINGS2, ZPOSL, ZPOSM to their factory defaults
   //--- (in this way ignoring OTP memory loaded values).
   AS5047P_SetFactorySettings(&encInstanceA);

   //--- Set current encoder position as new zero (AS5047P_ZPOSL, AS5047P_ZPOSM)
   AS5047P_SetZeroPosition(&encInstanceA);  
}

void loop() {
  // put your main code here, to run repeatedly:

  encPositionA = AS5047P_ReadPosition(&encInstanceA, AS5047P_OPT_ENABLED);

  if( !AS5047P_ErrorPending(&encInstanceA) )
  {
    //--- Print current position
    Serial.print("Encoder ID["); Serial.print( AS5047P_GetID(&encInstanceA) ); Serial.print("]> Position: "); Serial.println( (uint16_t)encPositionA );
  }
  else
  {
    //--- Print error message
    Serial.print("Encoder ID["); Serial.print( AS5047P_GetID(&encInstanceA) ); Serial.print("]> Error "); Serial
    .print( AS5047P_GetError(&encInstanceA).errorCode ); Serial.print(" : "); Serial.println(AS5047P_GetError(&encInstanceA).msg);

    //--- Error acknowledgement
    AS5047P_ErrorAck(&encInstanceA);
  }

  //--- Delay 100ms (for print)
  delay(100);    
}
