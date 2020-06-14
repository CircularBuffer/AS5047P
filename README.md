# AS5047P driver
Generic driver for AMS AS5047P magnetic encoder written in C. Because the driver was meant to be portable (platform independent), some details related to SPI communication and CS (chip-select) signals must be implemented by the user itself. This implementation takes places in file 'as5047_hal.c/cpp' - 4 simple methods must be implemented - for more details check this file out. SPI and GPIO initialization is assumed to be done outside of the driver.
If you want to use AS5047P with **Arduino** or **Nucleo** - you can jump to Examples straight away, complete implementation is already done for them.

### Repository Structure
* **Library**: AS5047P generic driver library
* **Examples**: Driver implementations and examples
  * General: General example with most functions shown
  * Arduino: Driver implementation with an example for Arduino
  * Nucleo-F429ZI: Driver implementation with an example for Nucleo-F429ZI
  
## How to run library on Arduino
1. Import [example](https://github.com/CircularBuffer/AS5047P/tree/master/Example/Arduino) to your project
2. Adjust method AS5047P_SelectSPIAndGPIO() in 'as5047_hal.cpp' for your needs (CS pin selection)
3. Run
  
## How to run library on Nucleo (F429ZI)
1. Import [example](https://github.com/CircularBuffer/AS5047P/tree/master/Examples/Nucleo-F429ZI) to your project
2. Adjust method AS5047P_SelectSPIAndGPIO() in 'as5047_hal.c' for your needs (CS pin selection, SPI handler selection)
3. Run
  
## How to run library on the other platforms

To make the library working with your platform you need to:
1. Import [library](https://github.com/CircularBuffer/AS5047P/tree/master/Library) to your project
2. Configure GPIO for CS signals
3. Configure SPI with these parameters:
 * 	SPI MODE 1 (CPOL = 0, CPHA = 1)
 * 	SCK max. 10Mhz (11.25Mhz worked also fine but not recommended)
 * 	Data size: 16 bits
 * 	Bits order: MSB first
 * 	CS signals handled by software
4. Implement following methods from 'as5047_hal.c' file:
*  AS5047P_HAL_Delay_ms: used for generating delay in miliseconds
*  AS5047P_HAL_SPI_TxRx: used for full-duplex SPI transmission
*  AS5047P_HAL_GPIO_Write: used for setting/clearing CS pins
*  AS5047P_SelectSPIAndGPIO: used for selecting SPI periphery and CS signal (see examples for Arduino/Nucleo)
*  AS5047P_HAL_Debug: used for debug purposes. This is an option.

## AS5047P driver features
*  SPI communication monitoring (parity bit check)
*  SPI's slave reachibility test
*  Position value validation (magnetic field strength, CORDIC overflow, etc.) when AS5047P_ReadPosition() method is used.
*  Zero position calibration possible with AS5047P_SetZeroPosition() method.
*  Any register read/write possible with AS5047P_ReadRegister() & AS5047P_WriteRegister methods.
*  OTP burn possible

