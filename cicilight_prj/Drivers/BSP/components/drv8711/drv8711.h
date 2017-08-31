#ifndef opbs_h
#define opbs_h

#include "registers.h"
#include "stdbool.h"


  // load register state from EEPROM, override drive and microstepping
  void begin (unsigned int torque, unsigned int gain, unsigned int microsteps) ;  

  void set_enable () ;
  
  void end () ;

  void clear_status () ;
  
  void get_status () ;
  
  unsigned int SPI_DRV8711_ReadWrite(unsigned char dataHi, unsigned char dataLo) ;
  //these are defined in registers.h

  void ReadAllRegisters () ;
  
  void WriteAllRegisters () ;
  
  



 

#endif