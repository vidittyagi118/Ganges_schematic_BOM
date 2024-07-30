
#include "fsl_dspi.h"
#include "SDCard_Spi.h"

#define SDC_DSPI_BASEADDR SPI0
#define SDC_DSPI_CLK_SRC DSPI0_CLK_SRC
#define SDC_DSPI_IRQ SPI0_IRQn
#define INIT_DSPI_PCS (kDSPI_Pcs0|kDSPI_Pcs1|kDSPI_Pcs2|kDSPI_Pcs3)
#define SDC_DSPI_PCS kDSPI_Pcs0
#define SDC_DSPI_IRQHandler SPI0_IRQHandler

#define TRANSFER_SIZE 2U                //256U        /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 8000000U  //000U /*! Transfer baudrate - 500k */
#define MAX_TRANSFER_SIZE       10


uint8_t SDCSpiRxData[TRANSFER_SIZE] = {0U};
uint8_t SDCSpiTxData[TRANSFER_SIZE] = {0U};

volatile uint32_t SDCSpiTxCount;
volatile uint32_t SDCSpiRxCount;
volatile uint32_t SDCSpiCommand;
uint32_t SDCSpiFifoSize;

//dspi_master_handle_t g_m_handle;
//dspi_transfer_t masterXfer;
//volatile bool isTransferCompleted = false;
//uint8_t masterTxData[MAX_TRANSFER_SIZE];
//uint8_t masterRxData[MAX_TRANSFER_SIZE] = {0U};



volatile bool SDC_SpiTxComplete_Flag = true;
//void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
//{
//    if (status == kStatus_Success)
//    {
//        __NOP();
//    }
//
//    isTransferCompleted = true;
//}



void int_SDC_SPI (void)
{
    uint32_t srcClock_Hz;
//    uint32_t errorCount;
    dspi_master_config_t SDCSpiConfig;

    /* SDCSpi config */
    SDCSpiConfig.whichCtar = kDSPI_Ctar1;
    SDCSpiConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
    SDCSpiConfig.ctarConfig.bitsPerFrame = 8;
    SDCSpiConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;       //kDSPI_ClockPolarityActiveLow;       //kDSPI_ClockPolarityActiveHigh;
    SDCSpiConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;  //kDSPI_ClockPhaseSecondEdge
    SDCSpiConfig.ctarConfig.direction = kDSPI_MsbFirst;
    SDCSpiConfig.ctarConfig.pcsToSckDelayInNanoSec =   100000000U / TRANSFER_BAUDRATE;
    SDCSpiConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 10000000U / TRANSFER_BAUDRATE;
    SDCSpiConfig.ctarConfig.betweenTransferDelayInNanoSec = 10000000U / TRANSFER_BAUDRATE;

    SDCSpiConfig.whichPcs = INIT_DSPI_PCS;
    SDCSpiConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    SDCSpiConfig.enableContinuousSCK = false;
    SDCSpiConfig.enableRxFifoOverWrite = false;
    SDCSpiConfig.enableModifiedTimingFormat = false;
    SDCSpiConfig.samplePoint = kDSPI_SckToSin0Clock;    // kDSPI_SckToSin1Clock;

    srcClock_Hz = CLOCK_GetFreq(SDC_DSPI_CLK_SRC);
    DSPI_MasterInit(SDC_DSPI_BASEADDR, &SDCSpiConfig, srcClock_Hz);
//    DSPI_MasterTransferCreateHandle(SDC_DSPI_BASEADDR, &g_m_handle, DSPI_MasterUserCallback, NULL);
        EnableIRQ(SDC_DSPI_IRQ);
}


void SDC_DSPI_IRQHandler(void)
{
//    isTransferCompleted = true;
    if (SDCSpiRxCount < TRANSFER_SIZE)
    {
        while (DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_RxFifoDrainRequestFlag)
        {
            SDCSpiRxData[SDCSpiRxCount] = DSPI_ReadData(SDC_DSPI_BASEADDR);
            ++SDCSpiRxCount;

            DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestFlag);

            if (SDCSpiRxCount == TRANSFER_SIZE)
            {
                break;
            }
        }
    }

    if (SDCSpiTxCount < TRANSFER_SIZE)
    {
        while ((DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag) &&
               ((SDCSpiTxCount - SDCSpiRxCount) < SDCSpiFifoSize))
        {
            if (SDCSpiTxCount < TRANSFER_SIZE)
            {
                SDC_DSPI_BASEADDR->PUSHR = SDCSpiCommand | SDCSpiTxData[SDCSpiTxCount];
                ++SDCSpiTxCount;
            }
            else
            {
                break;
            }

            /* Try to clear the TFFF; if the TX FIFO is full this will clear */
            DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
        }
    }

    /* Check if we're done with this transfer.*/
    if ((SDCSpiTxCount == TRANSFER_SIZE) && (SDCSpiRxCount == TRANSFER_SIZE))
    {
        /* Complete the transfer and disable the interrupts */
        DSPI_DisableInterrupts(SDC_DSPI_BASEADDR,
                               kDSPI_RxFifoDrainRequestInterruptEnable | kDSPI_TxFifoFillRequestInterruptEnable);
        SDC_SpiTxComplete_Flag = true;
    }
}

void SDC_SPI_Tx (uint8_t * SDCSpiTxData , uint8_t * SDCSpiRxData, uint8_t SDC_SPI_TXData_Len)
{
  
   /* Start SDCSpi transfer*/
    dspi_command_data_config_t commandData;
    commandData.isPcsContinuous = false;
    commandData.whichCtar = kDSPI_Ctar1;
    commandData.whichPcs = kDSPI_MasterPcs3;
    commandData.isEndOfQueue = true;    //false;
    commandData.clearTransferCount = false;

    SDCSpiCommand = DSPI_MasterGetFormattedCommand(&commandData);

    SDCSpiFifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(SDC_DSPI_BASEADDR);
    SDCSpiTxCount = 0;
    SDCSpiRxCount = 0;

    DSPI_StopTransfer(SDC_DSPI_BASEADDR);
    DSPI_FlushFifo(SDC_DSPI_BASEADDR, true, true);
    DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_AllStatusFlag);
    DSPI_StartTransfer(SDC_DSPI_BASEADDR);

            SDC_SpiTxComplete_Flag = false;
    /*Fill up the SDCSpi Tx data*/
    while (DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag)
    {
        if (SDCSpiTxCount < SDC_SPI_TXData_Len)
        {
            DSPI_MasterWriteData(SDC_DSPI_BASEADDR, &commandData, SDCSpiTxData[SDCSpiTxCount]);
            ++SDCSpiTxCount;
        }
        else
        {
            break;
        }

        /* Try to clear the TFFF; if the TX FIFO is full this will clear */
        DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
    }

    /*Enable SDCSpi RX interrupt*/
    DSPI_EnableInterrupts(SDC_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestInterruptEnable);

    /* Wait slave received all data. */
  //  while (!isTransferCompleted)
  //  {
   // }  
}

//uint32_t GetPcsForSpi (uint8_t device)
//{
//  uint32_t pcs;
//  switch(device)
//  {
//   case DRIVE_1:
//     pcs = kDSPI_MasterPcs3;
//    break;
//   case DRIVE_2:
//     pcs = kDSPI_MasterPcs2;
//    break;
//   case DRIVE_3:
//     pcs = kDSPI_MasterPcs1;
//    break;
//   case FLS:
//     pcs = kDSPI_MasterPcs4;
//    break;
//   case SDC:
//     pcs = kDSPI_MasterPcs0;
//    break;
//   default:
//    pcs = kDSPI_MasterPcs0;
//    break;
//  }
//  return pcs;
//}

//bool SpiTxRxData (uint8_t device, uint8_t *txdata, uint8_t *rxdata, uint16_t datalen)
//{
//   /* Start SDCSpi transfer*/
//    dspi_command_data_config_t commandData;
//    commandData.isPcsContinuous = false;
//    commandData.whichCtar = kDSPI_Ctar1;
//    commandData.whichPcs = GetPcsForSpi(device);;
//    commandData.isEndOfQueue = true;    //false;
//    commandData.clearTransferCount = false;
//
//    SDCSpiCommand = DSPI_MasterGetFormattedCommand(&commandData);
//
//    SDCSpiFifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(SDC_DSPI_BASEADDR);
//    SDCSpiTxCount = 0;
//    SDCSpiRxCount = 0;
//
//    DSPI_StopTransfer(SDC_DSPI_BASEADDR);
//    DSPI_FlushFifo(SDC_DSPI_BASEADDR, true, true);
//    DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_AllStatusFlag);
//    DSPI_StartTransfer(SDC_DSPI_BASEADDR);
//
//            SDC_SpiTxComplete_Flag = false;
//    /*Fill up the SDCSpi Tx data*/
//    while (DSPI_GetStatusFlags(SDC_DSPI_BASEADDR) & kDSPI_TxFifoFillRequestFlag)
//    {
//        if (SDCSpiTxCount < datalen)
//        {
//            DSPI_MasterWriteData(SDC_DSPI_BASEADDR, &commandData, txdata[datalen]);
//            ++SDCSpiTxCount;
//        }
//        else
//        {
//            break;
//        }
//
//        /* Try to clear the TFFF; if the TX FIFO is full this will clear */
//        DSPI_ClearStatusFlags(SDC_DSPI_BASEADDR, kDSPI_TxFifoFillRequestFlag);
//    }
//
//    /*Enable SDCSpi RX interrupt*/
//    DSPI_EnableInterrupts(SDC_DSPI_BASEADDR, kDSPI_RxFifoDrainRequestInterruptEnable);
//
//    /* Wait slave received all data. */
//  //  while (!isTransferCompleted)
//  //  {
//   // }  
//}


