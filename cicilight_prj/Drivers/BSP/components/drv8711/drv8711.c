#include "spi.h"

#include "cmsis_os.h"
#include "drv8711.h"
#include "JJDK_ZK_GZ1.h"
#define APP_LOG_MODULE_NAME   "[drv8711]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#include "app_log.h"

extern SPI_HandleTypeDef hspi2;

 /*  
  byte _pin ;
  byte _amps ;
  byte _torque ;
  byte _gain ;
  byte _microstepreg ;
*/  

struct CTRL_Register 	G_CTRL_REG;
struct TORQUE_Register 	G_TORQUE_REG;
struct OFF_Register 	G_OFF_REG;
struct BLANK_Register	G_BLANK_REG;
struct DECAY_Register 	G_DECAY_REG;
struct STALL_Register 	G_STALL_REG;
struct DRIVE_Register 	G_DRIVE_REG;
struct STATUS_Register 	G_STATUS_REG;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drv8711_init (int pin)
{

 }

void begin (unsigned int torque, unsigned int gain, unsigned int microsteps) 
{
	
    // set the Initial default values
    // CTRL Register
	G_CTRL_REG.Address 	= 0x00;
	G_CTRL_REG.DTIME 	= 0x03;  //850ns
	switch (gain) {
    case 5:
        G_CTRL_REG.ISGAIN 	= 0x0;  //Gain of 10  
	  break;
    case 10:
        G_CTRL_REG.ISGAIN 	= 0x01;  //Gain of 10  
      break;
    case 20:
        G_CTRL_REG.ISGAIN 	= 0x02;  //Gain of 10  
      break;
    case 40:
        G_CTRL_REG.ISGAIN 	= 0x03;  //Gain of 10  
      break;
    }
	
	G_CTRL_REG.EXSTALL 	= 0x01;  //External Stall Detect
	
	switch (microsteps) {
    case 1:
        G_CTRL_REG.MODE 	= 0x0;  // 1/1 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
	  break;
    case 2:
        G_CTRL_REG.MODE 	= 0x01;  // 1/2 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    case 4:
        G_CTRL_REG.MODE 	= 0x02;  // 1/4 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    case 8:
        G_CTRL_REG.MODE 	= 0x03;  // 1/8 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    case 16:
        G_CTRL_REG.MODE 	= 0x04;  // 1/16 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
	case 32:
        G_CTRL_REG.MODE 	= 0x05;  // 1/32 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    case 64:
        G_CTRL_REG.MODE 	= 0x06;  // 1/64 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    case 128:
        G_CTRL_REG.MODE 	= 0x07;  // 1/128 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    case 256:
        G_CTRL_REG.MODE 	= 0x08;  // 1/256 Step   0x03=1/8, 0x08=1/256, 0x05=1/32
      break;
    }
	
	
	//G_CTRL_REG.MODE 	= 0x05;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
	G_CTRL_REG.RSTEP 	= 0x00;  //No Action
	G_CTRL_REG.RDIR 	= 0x00;  //Direction set by DIR Pin
	G_CTRL_REG.ENBL 	= 0x00;  //disenable motor
        //1000 11 01 0 0011 0 0 1

	/// TORQUE Register
	G_TORQUE_REG.Address = 0x01;
	G_TORQUE_REG.SIMPLTH = 0x01;  //100uS Back EMF Sample Threshold
	
	unsigned int torqueMapped = map(torque, 0, 100, 0, 255);
	G_TORQUE_REG.TORQUE  = torqueMapped;  // From the variable passed to the function
        //1000 0 000  01000110
        
	// OFF Register
	G_OFF_REG.Address 	= 0x02;
	G_OFF_REG.PWMMODE 	= 0x00;  //Internal Indexer
	G_OFF_REG.TOFF 		= 0x30;  //Default
        //1000 000 0 00110000

	// BLANK Register
	G_BLANK_REG.Address     = 0x03;
	G_BLANK_REG.ABT 	= 0x01;  //enable adaptive blanking time
	G_BLANK_REG.TBLANK 	= 0x80;  //no idea what this should be but the
        //1000 000 1 00001000            //user guide shows it set to this

	// DECAY Register.
	G_DECAY_REG.Address     = 0x04;
	G_DECAY_REG.DECMOD      = 0x01;  //mixed decay
	G_DECAY_REG.TDECAY      = 0x80;  //default
        //1000001100010000

	// STALL Register
	G_STALL_REG.Address     = 0x05;
	G_STALL_REG.VDIV 	= 0x02;  //Back EMF is divided by 8
	G_STALL_REG.SDCNT 	= 0x03;  //stalln asserted after 8 steps
	G_STALL_REG.SDTHR 	= 0x40;  //default
        //1000111101000000

	// DRIVE Register
	G_DRIVE_REG.Address     = 0x06;
	G_DRIVE_REG.IDRIVEP     = 0x00;  //High Side 50mA peak (source)
	G_DRIVE_REG.IDRIVEN     = 0x00;  //Low Side 100mA peak (sink)
	G_DRIVE_REG.TDRIVEP     = 0x01;  //High Side gate drive 500nS
	G_DRIVE_REG.TDRIVEN     = 0x01;  //Low Side Gate Drive 500nS
	G_DRIVE_REG.OCPDEG      = 0x03;  //OCP Deglitch Time 8uS
	G_DRIVE_REG.OCPTH       = 0x01;  //OCP Threshold 500mV
        //1000000001010101

	// STATUS Register
	G_STATUS_REG.Address = 0x07;
	G_STATUS_REG.STDLAT  = 0x00;
	G_STATUS_REG.STD     = 0x00;  
	G_STATUS_REG.UVLO    = 0x00;  
	G_STATUS_REG.BPDF    = 0x00;  
	G_STATUS_REG.APDF    = 0x00;
	G_STATUS_REG.BOCP    = 0x00;
	G_STATUS_REG.AOCP    = 0x00;
	G_STATUS_REG.OTS     = 0x00;
        
    
    WriteAllRegisters();
  
    //set_enable (true) ;
}


void set_enable ()
{
 // Set Enable
 G_CTRL_REG.ENBL 	= 0x01;  //enable motor
 WriteAllRegisters();
}

void end ()
{
  // Set Disable Bit
  G_CTRL_REG.ENBL 	= 0x00;  //disable motor
  WriteAllRegisters();

}

void clear_status ()
{
  // Clear Status
}

void get_status ()
{
 ReadAllRegisters();
}

void ReadAllRegisters()
{
    #define REGWRITE    0x00
    #define REGREAD     0x80
	
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read CTRL Register
    dataHi = REGREAD | (G_CTRL_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_CTRL_REG.DTIME        = ((readData >> 10) & 0x0003);
    G_CTRL_REG.ISGAIN       = ((readData >> 8) & 0x0003);
    G_CTRL_REG.EXSTALL      = ((readData >> 7) & 0x0001);
    G_CTRL_REG.MODE         = ((readData >> 3) & 0x000F);
    G_CTRL_REG.RSTEP        = ((readData >> 2) & 0x0001);
    G_CTRL_REG.RDIR         = ((readData >> 1) & 0x0001);
    G_CTRL_REG.ENBL         = ((readData >> 0) & 0x0001);
    
 
    APP_LOG_DEBUG("Control Register\r\n");
    APP_LOG_DEBUG("DTIME = %d\r\n",G_CTRL_REG.DTIME);
    APP_LOG_DEBUG("ISGAIN = %d\r\n",G_CTRL_REG.ISGAIN);
    APP_LOG_DEBUG("EXSTALL = %d\r\n",G_CTRL_REG.EXSTALL);
    APP_LOG_DEBUG("MODE = %d\r\n",G_CTRL_REG.MODE);
    APP_LOG_DEBUG("RSTEP = %d\r\n",G_CTRL_REG.RSTEP);
    APP_LOG_DEBUG("RDIR = %d\r\n",G_CTRL_REG.RDIR);
    APP_LOG_DEBUG("ENBL = %d\r\n",G_CTRL_REG.ENBL);

    // Read TORQUE Register
    dataHi = REGREAD | (G_TORQUE_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_TORQUE_REG.SIMPLTH    = ((readData >> 8) & 0x0007);
    G_TORQUE_REG.TORQUE     = ((readData >> 0) & 0x00FF);
    //APP_LOG_DEBUG("TORQUE = ");
    //APP_LOG_DEBUG(G_TORQUE_REG.TORQUE, HEX);

    // Read OFF Register
    dataHi = REGREAD | (G_OFF_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_OFF_REG.PWMMODE       = ((readData >> 8) & 0x0001);
    G_OFF_REG.TOFF          = ((readData >> 0) & 0x00FF);

    // Read BLANK Register
    dataHi = REGREAD | (G_BLANK_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_BLANK_REG.ABT         = ((readData >> 8) & 0x0001);
    G_BLANK_REG.TBLANK      = ((readData >> 0) & 0x00FF);

    // Read DECAY Register
    dataHi = REGREAD | (G_DECAY_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_DECAY_REG.DECMOD      = ((readData >> 8) & 0x0007);
    G_DECAY_REG.TDECAY      = ((readData >> 0) & 0x00FF);

    // Read STALL Register
    dataHi = REGREAD | (G_STALL_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_STALL_REG.VDIV        = ((readData >> 10) & 0x0003);
    G_STALL_REG.SDCNT       = ((readData >> 8) & 0x0003);
    G_STALL_REG.SDTHR       = ((readData >> 0) & 0x00FF);

    // Read DRIVE Register
    dataHi = REGREAD | (G_DRIVE_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_DRIVE_REG.IDRIVEP     = ((readData >> 10) & 0x0003);
    G_DRIVE_REG.IDRIVEN     = ((readData >> 8) & 0x0003);
    G_DRIVE_REG.TDRIVEP     = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.TDRIVEN     = ((readData >> 4) & 0x0003);
    G_DRIVE_REG.OCPDEG      = ((readData >> 2) & 0x0003);
    G_DRIVE_REG.OCPTH       = ((readData >> 0) & 0x0003);

    // Read STATUS Register
    dataHi = REGREAD | (G_STATUS_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);
    //APP_LOG_DEBUG(readData, BIN);
    G_STATUS_REG.STDLAT     = ((readData >> 7) & 0x0001);
    G_STATUS_REG.STD        = ((readData >> 6) & 0x0001);
    G_STATUS_REG.UVLO       = ((readData >> 5) & 0x0001);
    G_STATUS_REG.BPDF       = ((readData >> 4) & 0x0001);
    G_STATUS_REG.APDF       = ((readData >> 3) & 0x0001);
    G_STATUS_REG.BOCP       = ((readData >> 2) & 0x0001);
    G_STATUS_REG.AOCP       = ((readData >> 1) & 0x0001);
    G_STATUS_REG.OTS        = ((readData >> 0) & 0x0001);
}

void WriteAllRegisters()

    #define REGWRITE    0x00
    #define REGREAD     0x80
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;


    // Write TORQUE Register
    dataHi = REGWRITE | (G_TORQUE_REG.Address << 4) | (G_TORQUE_REG.SIMPLTH);
    dataLo = G_TORQUE_REG.TORQUE;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write OFF Register
    dataHi = REGWRITE | (G_OFF_REG.Address << 4) | (G_OFF_REG.PWMMODE);
    dataLo = G_OFF_REG.TOFF;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write BLANK Register
    dataHi = REGWRITE | (G_BLANK_REG.Address << 4) | (G_BLANK_REG.ABT);
    dataLo = G_BLANK_REG.TBLANK;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write DECAY Register
    dataHi = REGWRITE | (G_DECAY_REG.Address << 4) | (G_DECAY_REG.DECMOD);
    dataLo = G_DECAY_REG.TDECAY;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write STALL Register
    dataHi = REGWRITE | (G_STALL_REG.Address << 4) | (G_STALL_REG.VDIV << 2) | (G_STALL_REG.SDCNT);
    dataLo = G_STALL_REG.SDTHR;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write DRIVE Register
    dataHi = REGWRITE | (G_DRIVE_REG.Address << 4) | (G_DRIVE_REG.IDRIVEP << 2) | (G_DRIVE_REG.IDRIVEN);
    dataLo = (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write STATUS Register
    dataHi = REGWRITE | (G_STATUS_REG.Address << 4);
    dataLo = (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
    SPI_DRV8711_ReadWrite(dataHi, dataLo);
    
        // Write CTRL Register
    dataHi = REGWRITE | (G_CTRL_REG.Address << 4) | (G_CTRL_REG.DTIME << 2) | (G_CTRL_REG.ISGAIN);
    dataLo = (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
    SPI_DRV8711_ReadWrite(dataHi, dataLo);
}



unsigned int SPI_DRV8711_ReadWrite(unsigned char dataHi, unsigned char dataLo)
{
    unsigned int readData = 0;
    uint8_t tx[2],rx[2];
        
    tx[0]=dataHi;
    tx[1]=dataLo;

   HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_CS_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_CS_POS_Pin, BSP_COLUMN_STEP_MOTOR_CS_ENABLE_PIN_STATE);
   osDelay(1); 
   
   HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2,0xFFFF);
   
   readData=(uint16_t)rx[0]<<8|rx[1];

   HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_CS_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_CS_POS_Pin, BSP_COLUMN_STEP_MOTOR_CS_DISABLE_PIN_STATE);
  
  return readData;
}