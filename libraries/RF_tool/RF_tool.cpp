#include "RF_tool.h"


/*
*https://devzone.nordicsemi.com/question/5930/is-there-any-way-to-reset-nrf24l01/
*1)use power down mode (PWR_UP = 0) 
2)clear data ready flag and data sent flag in status register 
3)flush tx/rx buffer 
4)write status register as 0x0e;
*/
void resetRF(RF24& radio){
   // radio.flush_tx
 //void closeReadingPipe( uint8_t pipe ) ;

//  bool failureDetected; 
}


//http://gizmosnack.blogspot.co.at/2013/04/tutorial-nrf24l01-and-avr.html
