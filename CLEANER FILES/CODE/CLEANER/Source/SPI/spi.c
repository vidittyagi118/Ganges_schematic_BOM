#include "spi.h"
#include "board.h"
#include "fsl_dspi.h"
#include "fsl_port.h"

// ************SPI*************

#define EXAMPLE_DSPI_MASTER_BASEADDR SPI0
#define DSPI_MASTER_CLK_SRC DSPI0_CLK_SRC
#define DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#define EXAMPLE_DSPI_MASTER_PCS_FOR_INIT (kDSPI_Pcs0 | kDSPI_Pcs1 | kDSPI_Pcs2 | kDSPI_Pcs3 | kDSPI_Pcs4)
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs1

//#define TRANSFER_SIZE 2U         /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */

#define MAX_TRANSFER_SIZE       10

/* DSPI user callback */
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);


dspi_master_config_t masterConfig;
dspi_transfer_t masterXfer;
 
dspi_master_handle_t g_m_handle;
volatile bool isTransferCompleted = false;

uint8_t masterTxData[MAX_TRANSFER_SIZE];
uint8_t masterRxData[MAX_TRANSFER_SIZE] = {0U};

void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        __NOP();
    }

    isTransferCompleted = true;
}

uint32_t GetPcsForSpi (uint8_t device)
{
  uint32_t pcs;
  switch(device)
  {
   case DRIVE_1:
     pcs = kDSPI_MasterPcs3;
    break;
   case DRIVE_2:
     pcs = kDSPI_MasterPcs2;
    break;
   case DRIVE_3:
     pcs = kDSPI_MasterPcs1;
    break;
   case FLS:
     pcs = kDSPI_MasterPcs4;
    break;
   case LCD:
     pcs = kDSPI_MasterPcs0;
    break;
   default:
    pcs = kDSPI_MasterPcs0;
    break;
  }
  return pcs;
}

bool SpiInit (void)
{

   /* Master config */
    masterConfig.whichCtar = kDSPI_Ctar1;
    masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame = 8U;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    masterConfig.whichPcs = EXAMPLE_DSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, DSPI_MASTER_CLK_FREQ);

    DSPI_MasterTransferCreateHandle(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, DSPI_MasterUserCallback, NULL);
    
    return true;
}


bool SpiTxData (uint8_t device, uint8_t *txdata, uint16_t txdatalen)
{
  if(txdatalen <= MAX_TRANSFER_SIZE)
  {
    for(uint16_t txindex =0; txindex < txdatalen; txindex++)
    {
        masterTxData[txindex] = txdata[txindex];
    }
    masterXfer.txData = &masterTxData[0];
    masterXfer.rxData = &masterRxData[0];
    masterXfer.dataSize = txdatalen;
    uint32_t pcs = GetPcsForSpi(device);
    masterXfer.configFlags = kDSPI_MasterCtar1 | pcs | kDSPI_MasterPcsContinuous;
    isTransferCompleted = false;
    DSPI_MasterTransferNonBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
    
    while (!isTransferCompleted)
    {
    }
    return true;
  } 
  return false;
}


bool SpiRxData (uint8_t device, uint8_t *rxdata, uint16_t rxdatalen)
{
  if(rxdatalen <= MAX_TRANSFER_SIZE)
  {
    for(uint16_t rxindex =0; rxindex < rxdatalen; rxindex++)
    {
        masterTxData[rxindex]=0;
        masterRxData[rxindex]=0;
    }

    masterXfer.txData = &masterTxData[0];
    masterXfer.rxData = &masterRxData[0];
    masterXfer.dataSize = rxdatalen;
    uint32_t pcs = GetPcsForSpi(device);
    masterXfer.configFlags = kDSPI_MasterCtar1 | pcs | kDSPI_MasterPcsContinuous;
    isTransferCompleted = false;
    DSPI_MasterTransferNonBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
    
    while (!isTransferCompleted)
    {
    }
    memcpy(rxdata, masterRxData, rxdatalen);
    return true;
  } 
  return false;
}


bool SpiConsecutiveTxRxData (uint8_t device, uint8_t *txdata, uint16_t txdatalen, uint8_t *rxdata, uint16_t rxdatalen)
{
  uint16_t totalTransferlen = txdatalen + rxdatalen;
  
  if(totalTransferlen <= MAX_TRANSFER_SIZE)
  {
    for(uint16_t txindex =0; txindex < txdatalen; txindex++)
    {
        masterTxData[txindex] = txdata[txindex];
    }
    masterXfer.txData = &masterTxData[0];
    masterXfer.rxData = &masterRxData[0];
    masterXfer.dataSize = totalTransferlen;
    uint32_t pcs = GetPcsForSpi(device);
    masterXfer.configFlags = kDSPI_MasterCtar1 | pcs | kDSPI_MasterPcsContinuous;
    isTransferCompleted = false;
    DSPI_MasterTransferNonBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
    
    while (!isTransferCompleted)
    {
    }
    memcpy(rxdata, &masterRxData[1], rxdatalen);
    return true;
  } 
  return false;
}

bool SpiTxRxData (uint8_t device, uint8_t *txdata, uint8_t *rxdata, uint16_t datalen)
{
  
  if(datalen <= MAX_TRANSFER_SIZE)
  {
    for(uint16_t txindex =0; txindex < datalen; txindex++)
    {
        masterTxData[txindex] = txdata[txindex];
        masterRxData[txindex]=0;
    }
    masterXfer.txData = &masterTxData[0];
    masterXfer.rxData = &masterRxData[0];
    masterXfer.dataSize = datalen;
    uint32_t pcs = GetPcsForSpi(device);
    masterXfer.configFlags = kDSPI_MasterCtar1 | pcs | kDSPI_MasterPcsContinuous;
    isTransferCompleted = false;
    DSPI_MasterTransferNonBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
    
    while (!isTransferCompleted)
    {
    }
    memcpy(rxdata, masterRxData, datalen);
    return true;
  } 
  return false;
}

void Direct_SPI_Init()
{
   gpio_pin_config_t SD_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(BOARD_SD_GPIO, BOARD_SD_CS_GPIO_PIN, &SD_config);
  GPIO_PinInit(BOARD_SD_GPIO, BOARD_SD_CLK_GPIO_PIN, &SD_config);
  GPIO_PinInit(BOARD_SD_GPIO, BOARD_SD_MOUT_GPIO_PIN, &SD_config);
  SD_M_IN_Init();
  
 /*  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs low
   
  char SPICount=0,SPIData;
 
  SPIData = 0x55;
   
  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address byte
  {
    if (SPIData & 0x80)                                          // Check for a 1
      GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN); //  SPI_MOSI = 1;                                             // and set the MOSI line appropriately
    else
       GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN); // SPI_MOSI = 0;
    
    GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN);   //clk high  //  SPI_CK = 1;                                                 // Toggle the clock line
    SDdelay1(100);
    GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN); //clk low //SPI_CK = 0;
    SPIData <<= 1;                                              // Rotate to get the next bit
    SDdelay1(100);
  
  } 
   
   GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs High
  */
}

static void EnableInterrupt (void)
{
 PORT_SetPinInterruptConfig(BOARD_SD_PORT, BOARD_SD_MIN_GPIO_PIN, kPORT_InterruptEitherEdge);
}

bool SD_M_IN_Init (void)
{
  gpio_pin_config_t Direct_SD_config = {
    kGPIO_DigitalInput, 0,
  };
  
  port_pin_config_t pull_up_Config = {
    kPORT_PullUp,
    kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainDisable,
    kPORT_HighDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister,     
  }; 
  
  GPIO_PinInit(BOARD_SD_GPIO, BOARD_SD_MIN_GPIO_PIN, &Direct_SD_config);
  PORT_SetPinConfig(BOARD_SD_PORT, BOARD_SD_MIN_GPIO_PIN, &pull_up_Config);
 // DisableInterrupt();
 // UpdateSense1State();
 // EnableInterrupt();
  return true;
}

static inline bool Read_SD_M_IN_State (void)
{
 return GPIO_PinRead(BOARD_SD_GPIO, BOARD_SD_MIN_GPIO_PIN);
}
  
void SDdelay1(unsigned int Timet)
{
  while(Timet--) __NOP();
}


void SD_Write_Byte(char SPIData)
{
  
  char SPICount=0;
  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN); //clk low //SPI_CK = 0;
    
  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs low
   
   
  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address byte
  {
    if (SPIData & 0x80)                                          // Check for a 1
      GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN); //  SPI_MOSI = 1;                                             // and set the MOSI line appropriately
    else
       GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN); // SPI_MOSI = 0;
    
    GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN);   //clk high  //  SPI_CK = 1;                                                 // Toggle the clock line
    SDdelay1(5);
    GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN); //clk low //SPI_CK = 0;
    SDdelay1(5);
    SPIData <<= 1;                                              // Rotate to get the next bit
  }  
  GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs High
  SDdelay1(5);
}

unsigned char SPICount;                                       // Counter used to clock out the data
unsigned char SPIData; 
unsigned char SPIData1;
unsigned char spidataread;
unsigned char spidataread1;
unsigned char SDbit;

unsigned char SD_READ_Byte (const unsigned char regAddr)
{
  
//  SPI_CS = 1;                                                   // Make sure we start with active-low CS high
  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN); // SPI_CK = 0;                                                   // and CK low
  SPIData = regAddr;                                            // Preload the data to be sent with Address and Data

  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs low SPI_CS = 0;          // Set active-low CS low to start the SPI cycle
 /*
  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address and Data
  {
    if (SPIData & 0x80)
     GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN); // SPI_MOSI = 1;
    else
     GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN); // SPI_MOSI = 0;
    
    GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN);   //clk high  //  SPI_CK = 1;                      // Toggle the clock line
    SDdelay1(100);
    GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN); //clk low //SPI_CK = 0;
    SDdelay1(100);
    SPIData <<= 1;
  }                                                             // and loop back to send the next bit
  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN);  //SPI_MOSI = 0;   
  
  // Reset the MOSI data line
  SDdelay1(10);
  GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs low SPI_CS = 1;  
   SDdelay1(200);
  GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN); //cs low SPI_CS = 0;  
  */
 
  SDdelay1(20);
  GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_MOUT_GPIO_PIN);  
  SPIData = 0;SPIData1 = 0;  spidataread =0;spidataread1 =0;
  SDbit =0;
  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock in the data to be read
  {
    SPIData <<=1; SPIData1 <<=1;                                               // Rotate the data
  
    GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN);   //SPI_CK = 1;                                     // Raise the clock to clock the data out of the MAX7456
    SDdelay1(5);    
    SPIData +=  Read_SD_M_IN_State();                           //SPI_MISO;                                        // Read the data bit
 
    GPIO_PortClear(BOARD_SD_GPIO, 1U << BOARD_SD_CLK_GPIO_PIN); //SPI_CK = 0;                           // Drop the clock ready for the next bit
    //SDdelay1(100);
   
     SPIData1 +=  Read_SD_M_IN_State();
  }                                                            // and loop back 
  GPIO_PortSet(BOARD_SD_GPIO, 1U << BOARD_SD_CS_GPIO_PIN);     //cs High SPI_CS = 1;                   // Raise CS
  SDdelay1(5);  
  spidataread = SPIData;
  spidataread1 = SPIData1;

  return ((unsigned char)SPIData);                              // Finally return the read data
}

/*
for (i=0; i<8; i++)
	{
		SPI_CLK = 1;		// clock high
		Fdata <<= 1;
		delay(10);
		SPI_CLK = 0;		// clock low

                //DelayMs(100);
		Fbit = (BYTE) SPI_MISO;	//read SPI data IN

		if(Fbit == 1)
			 Fdata |= 0x01;
		else Fdata |= 0x00;
	}
 
    SDbit =  Read_SD_M_IN_State();
    SDbit =  Read_SD_M_IN_State();
    if(SDbit == 1)
           SPIData1 |= 0x01;
    else
      SPIData1 |= 0x00;
*/

/*
 char MIN_pinn=0,pinsts;
void PINCHECK_MIN()
{ 
      
      MIN_pinn = GPIO_PinRead(BOARD_SD_GPIO, BOARD_SD_MIN_GPIO_PIN); //Read_SD_M_IN_State();
      SDdelay1(100); 
      if(MIN_pinn == 0)
      {
        pinsts = 0;
        SDdelay1(100); 
      }
      else
      {
        pinsts = 1;
        SDdelay1(100); 
      }
      if(MIN_pinn == 0)
      {
        pinsts = 0;
        SDdelay1(100); 
      }
      else
      {
        pinsts = 1;
        SDdelay1(100); 
      }
  
  
}*/


