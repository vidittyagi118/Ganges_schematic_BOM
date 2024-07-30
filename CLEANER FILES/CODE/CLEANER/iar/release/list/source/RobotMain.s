///////////////////////////////////////////////////////////////////////////////
//
// IAR ANSI C/C++ Compiler V9.20.2.320/W64 for ARM        30/Dec/2021  15:31:58
// Copyright 1999-2021 IAR Systems AB.
//
//    Cpu mode     =  thumb
//    Endian       =  little
//    Source file  =
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\Source\RobotMain.c
//    Command line =
//        -f
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\release\obj\source\RobotMain.o.rsp
//        (F:\Ganges_Project\Venkat\SCR_30122021-AN\Source\RobotMain.c -D DEBUG
//        -D CPU_MK22FN1M0AVLQ12 -lC
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\release\list\source -lA
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\release\list\source
//        --diag_suppress Pa082,Pa050,Pe167 -o
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\release\obj\source
//        --debug --endian=little --cpu=Cortex-M4 -e --fpu=VFPv4_sp
//        --dlib_config "C:\Program Files\IAR Systems\Embedded Workbench
//        9.0\arm\inc\c\DLib_Config_Normal.h" -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../CMSIS/Include\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Source\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Startup\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Drivers\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Utilities/io\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Utilities/str\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Utilities/log\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar/../Utilities\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Board\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\LTC4015\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\DELAY\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\DEBUG\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\OTHER\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\ADC\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\SPI\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\I2C\ -I
//        "F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\TEMP
//        SENSORS\\" -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\DebugUart\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\ZigbeeUart\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\ZigbeeApiMode\
//        -I F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\ZigbeeAPILibrary\
//        -I F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotControl\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\MotorControl\Motor1\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\MotorControl\Motor1\Pwm1\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\MotorControl\Motor2\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\MotorControl\Motor2\Pwm2\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RotateSense\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotOperation\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\DefaultValues\
//        -I F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\EdgeSensor\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\PortInterrupt\
//        -I F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Relay\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\AutoMode\ -I
//        "F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\API
//        PROCESSING\\" -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\EEPROM\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotOperation\CycleMode\
//        -I F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RTC\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\MotorControl\BrushMotor\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\MotorControl\BrushMotor\BrushPwm\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotControl\BrushControl\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotOperation\AutoSchedule\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotOperation\FaultProcess\OverCurrentFault\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\RobotOperation\FaultProcess\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\OTHER\LPTimer\
//        -I F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\LED\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\LED\LED1\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\LED\LED2\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\LED\LED3\ -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\OTHER\WDTimer\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\OTHER\VBatREG\
//        -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\ConfigUart\
//        -I
//        "F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\ConfigUart\ConfigUart
//        API Process\\" -I
//        "F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Communication\ConfigUart\ConfigUart
//        API Process\JsonParser\\" -I
//        "F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\Event
//        Logger\\" -I
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\..\Source\SWITCH\ -Oh
//        --use_c++_inline) --dependencies=n
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\release\obj\source\RobotMain.o.d
//    Locale       =  C
//    List file    =
//        F:\Ganges_Project\Venkat\SCR_30122021-AN\iar\release\list\source\RobotMain.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__CPP_Runtime", "1"
        RTMODEL "__SystemLibrary", "DLib"
        RTMODEL "__dlib_file_descriptor", "0"
        RTMODEL "__dlib_full_locale_support", "0"
        RTMODEL "__dlib_version", "6"
        RTMODEL "__iar_require _Printf", ""
        AAPCS BASE,INTERWORK,VFP
        PRESERVE8
        REQUIRE8

        #define SHT_PROGBITS 0x1

        EXTERN ADCInit
        EXTERN BOARD_BootClockRUN
        EXTERN BOARD_InitPins
        EXTERN BatteryChargerInit
        EXTERN BatteryInfo
        EXTERN ChargerErrorHandler
        EXTERN CheckAndStoreQC
        EXTERN CheckMotorOverLoad
        EXTERN ConfigUartInit
        EXTERN ConfigUartReceive
        EXTERN ControlledCharging
        EXTERN DebugUartInit
        EXTERN DebugUartReceive
        EXTERN DefaultValuesInit
        EXTERN Direct_SPI_Init
        EXTERN DoInitialRelayOff
        EXTERN EEPROM_Init
        EXTERN EdgeSenseInit
        EXTERN EnablePortInterrupt
        EXTERN EventLogInit
        EXTERN FindAndUpdateImotOffsetValue
        EXTERN GetImot1Value
        EXTERN GetImot2Value
        EXTERN GetImot3Value
        EXTERN GetLPTimerMsTicks
        EXTERN GetRotateSense1Count
        EXTERN GetRotateSense2Count
        EXTERN GetRotateSenseCount
        EXTERN InitI2C
        EXTERN InitSwitch
        EXTERN InitialiseEepromVariables
        EXTERN IsEdgeSensor1Detected
        EXTERN IsEdgeSensor2Detected
        EXTERN LPTMR_Initialisation
        EXTERN LedInit
        EXTERN OperateLeds
        EXTERN ProcessReceivedJsonData
        EXTERN ProcessReceivedJsonData_config
        EXTERN RTCInit
        EXTERN RelayFSM
        EXTERN RelayInit
        EXTERN RestartCommTimer
        EXTERN RobotFSM
        EXTERN RobotInit
        EXTERN RobotOperate
        EXTERN RotateSenseInit
        EXTERN SDcardMount
        EXTERN SDcardOperation
        EXTERN Serial_Debug
        EXTERN SetFault
        EXTERN SetReceivedMACAddress
        EXTERN SetUartType
        EXTERN SwitchLed3Off
        EXTERN SwitchLed3On
        EXTERN TransmitWelcomeMessage
        EXTERN UpdateWDTimerValue
        EXTERN WDTimerInit
        EXTERN WDTimerRefresh
        EXTERN ZigbeePoll
        EXTERN ZigbeeUartInit
        EXTERN ZigbeeUartReceive
        EXTERN ftoa
        EXTERN snprintf
        EXTERN strlen

        PUBLIC DispDebugInfo
        PUBLIC SD_MOUNT_STATUS
        PUBLIC main
        
          CFI Names cfiNames0
          CFI StackFrame CFA R13 DATA
          CFI Resource R0:32, R1:32, R2:32, R3:32, R4:32, R5:32, R6:32, R7:32
          CFI Resource R8:32, R9:32, R10:32, R11:32, R12:32, R13:32, R14:32
          CFI Resource D0:64, D1:64, D2:64, D3:64, D4:64, D5:64, D6:64, D7:64
          CFI Resource D8:64, D9:64, D10:64, D11:64, D12:64, D13:64, D14:64
          CFI Resource D15:64
          CFI EndNames cfiNames0
        
          CFI Common cfiCommon0 Using cfiNames0
          CFI CodeAlign 2
          CFI DataAlign 4
          CFI ReturnAddress R14 CODE
          CFI CFA R13+0
          CFI R0 Undefined
          CFI R1 Undefined
          CFI R2 Undefined
          CFI R3 Undefined
          CFI R4 SameValue
          CFI R5 SameValue
          CFI R6 SameValue
          CFI R7 SameValue
          CFI R8 SameValue
          CFI R9 SameValue
          CFI R10 SameValue
          CFI R11 SameValue
          CFI R12 Undefined
          CFI R14 SameValue
          CFI D0 Undefined
          CFI D1 Undefined
          CFI D2 Undefined
          CFI D3 Undefined
          CFI D4 Undefined
          CFI D5 Undefined
          CFI D6 Undefined
          CFI D7 Undefined
          CFI D8 SameValue
          CFI D9 SameValue
          CFI D10 SameValue
          CFI D11 SameValue
          CFI D12 SameValue
          CFI D13 SameValue
          CFI D14 SameValue
          CFI D15 SameValue
          CFI EndCommon cfiCommon0
        
// F:\Ganges_Project\Venkat\SCR_30122021-AN\Source\RobotMain.c
//    1 #include "board.h"
//    2 #include "fsl_port.h"
//    3 #include "pin_mux.h"
//    4 #include "clock_config.h"
//    5 
//    6 #include "delay.h"
//    7 #include <stdio.h>
//    8 #include <string.h>
//    9 #include <stdlib.h>
//   10 #include "LTC4015_Main.h"
//   11 #include "adc.h"
//   12 #include "spi.h"
//   13 #include "i2c_init.h"
//   14 #include "PwmDriveSpi.h"
//   15 #include "WDTimer.h"
//   16 #include "RobotControl.h"       
//   17 #include "RotateSenseCommon.h"       
//   18 #include "EdgeSenseCommon.h"       
//   19 #include "ZigbeeUart.h"       
//   20 #include "DebugUart.h"  
//   21 #include "ConfigUart.h"      
//   22 #include "DefaultValues.h"       
//   23 #include "RobotOperation.h"       
//   24 #include "PortInterrupt.h"      
//   25 #include "Relay.h"               
//   26 #include "ZigbeeAPIMode.h"       
//   27 #include "MotCurrentFaultCommon.h"       
//   28 #include "CommCommon.h"  
//   29 #include "store_soc.h"     
//   30 #include "FaultProcess.h" 
//   31 #include "APIProcessing.h"
//   32 #include "ConfigAPIProcess.h"
//   33 #include "temp_sensors.h"
//   34 #include "eeprommain.h"
//   35 #include "Rtc.h"
//   36 #include "LPTimer.h"
//   37 #include "LedCommon.h"
//   38 #include "Serial_Debug.h"
//   39 #include "NumStrConversion.h"
//   40 #include "Led3.h"
//   41 #include "EventLogger.h"
//   42 #include "eepromStdtoHalDriver.h"
//   43 #include "switch.h"
//   44 
//   45 #include "SDFAT/ff.h"
//   46 #include "SDFAT/ffconf.h"
//   47 #include "SDFAT/diskio.h"
//   48 #include "SDFAT/integer.h"
//   49 #include "SDFAT/SDcardoperation.h"
//   50 
//   51 /*******************************************************************************
//   52 * Definitions
//   53 ******************************************************************************/
//   54 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        DATA
//   55 BYTE SD_MOUNT_STATUS;
SD_MOUNT_STATUS:
        DS8 1
//   56 void DispDebugInfo (void);
//   57 static void ProcessUartData(void);
//   58 
//   59 

        SECTION `.text`:CODE:NOROOT(2)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function main
        THUMB
//   60  void main(void)
//   61 {
main:
        PUSH     {LR}           
          CFI R14 Frame(CFA, -4)
          CFI CFA R13+4
        SUB      SP,SP,#+508    
          CFI CFA R13+512
//   62   
//   63   BOARD_InitPins();
          CFI FunCall BOARD_InitPins
        BL       BOARD_InitPins 
//   64   BOARD_BootClockRUN();
          CFI FunCall BOARD_BootClockRUN
        BL       BOARD_BootClockRUN
//   65   WDTimerInit(WDT_RESET_TIME_START);
        MOVW     R0,#+25000     
          CFI FunCall WDTimerInit
        BL       WDTimerInit    
//   66   LedInit();
          CFI FunCall LedInit
        BL       LedInit        
//   67   LPTMR_Initialisation();
          CFI FunCall LPTMR_Initialisation
        BL       LPTMR_Initialisation
//   68   DebugUartInit(); 
          CFI FunCall DebugUartInit
        BL       DebugUartInit  
//   69   ConfigUartInit();        
          CFI FunCall ConfigUartInit
        BL       ConfigUartInit 
//   70   TransmitWelcomeMessage(DEBUG_UART);
        MOVS     R0,#+1         
          CFI FunCall TransmitWelcomeMessage
        BL       TransmitWelcomeMessage
//   71   TransmitWelcomeMessage(CONFIG_UART); 
        MOVS     R0,#+2         
          CFI FunCall TransmitWelcomeMessage
        BL       TransmitWelcomeMessage
//   72   if(RTCInit() == false)
          CFI FunCall RTCInit
        BL       RTCInit        
        CBNZ.N   R0,??main_0    
//   73   {
//   74     SetFault(RTC_FAULT);
        MOVS     R0,#+8         
          CFI FunCall SetFault
        BL       SetFault       
//   75     Serial_Debug("\n RTC Initialisation ERROR *~*~*~*~*~*");
        LDR.N    R0,??DataTable3
        B.N      ??main_1       
//   76   }
//   77   else
//   78   {
//   79     Serial_Debug("\n RTC Initialisation OK");  
??main_0:
        ADR.N    R0,?_3         
??main_1:
          CFI FunCall Serial_Debug
        BL       Serial_Debug   
//   80   } 
//   81   
//   82   //SpiInit();
//   83   //Direct_SPI_Init();//direct pin inits
//   84   ADCInit();
          CFI FunCall ADCInit
        BL       ADCInit        
//   85   
//   86   WDTimerRefresh();
          CFI FunCall WDTimerRefresh
        BL       WDTimerRefresh 
//   87   
//   88   if(EEPROM_Init() == false)
          CFI FunCall EEPROM_Init
        BL       EEPROM_Init    
        CBNZ.N   R0,??main_2    
//   89   {
//   90     Serial_Debug("\n EEPROM Initialisation Failed");   
        ADR.N    R0,?_4         
          CFI FunCall Serial_Debug
        BL       Serial_Debug   
        B.N      ??main_3       
//   91   }
//   92   else
//   93   {
//   94     uint8_t status = (uint8_t)InitialiseEepromVariables();   
??main_2:
          CFI FunCall InitialiseEepromVariables
        BL       InitialiseEepromVariables
//   95     //SetLogEvent(EV_LOG_EEPROM, status);  
//   96   }
//   97 
//   98   EventLogInit(EEPROM);
??main_3:
        MOVS     R0,#+1         
          CFI FunCall EventLogInit
        BL       EventLogInit   
//   99   DefaultValuesInit();   
          CFI FunCall DefaultValuesInit
        BL       DefaultValuesInit
//  100   RelayInit();
          CFI FunCall RelayInit
        BL       RelayInit      
//  101   DoInitialRelayOff();  //07S
          CFI FunCall DoInitialRelayOff
        BL       DoInitialRelayOff
//  102   RobotInit();          
          CFI FunCall RobotInit
        BL       RobotInit      
//  103   RotateSenseInit();    
          CFI FunCall RotateSenseInit
        BL       RotateSenseInit
//  104   EdgeSenseInit();     
          CFI FunCall EdgeSenseInit
        BL       EdgeSenseInit  
//  105   EnablePortInterrupt();
          CFI FunCall EnablePortInterrupt
        BL       EnablePortInterrupt
//  106   BatteryChargerInit();
          CFI FunCall BatteryChargerInit
        BL       BatteryChargerInit
//  107   InitI2C();
          CFI FunCall InitI2C
        BL       InitI2C        
//  108   InitSwitch();
          CFI FunCall InitSwitch
        BL       InitSwitch     
//  109   //msdelay(2000);
//  110   
//  111   if(ZigbeeUartInit() == ERROR)
          CFI FunCall ZigbeeUartInit
        BL       ZigbeeUartInit 
        CMP      R0,#+1         
        ITT      EQ                
//  112   {
//  113     SetFault(ZIGBEE_FAULT);
        MOVEQ    R0,#+7         
          CFI FunCall SetFault
        BLEQ     SetFault       
//  114     //SetLogEvent(EV_LOG_ZB_UART, (uint8_t)EV_UART_ERROR);
//  115   }
//  116   else
//  117   {
//  118     //SetLogEvent(EV_LOG_ZB_UART, (uint8_t)EV_UART_STARTED); 
//  119   } 
//  120   WDTimerRefresh();
          CFI FunCall WDTimerRefresh
        BL       WDTimerRefresh 
//  121 #ifndef ZIGBEE_API_MODE_ENABLE
//  122   TransmitWelcomeMessage(ZIGBEE_UART);
//  123 #endif
//  124   FindAndUpdateImotOffsetValue();
          CFI FunCall FindAndUpdateImotOffsetValue
        BL       FindAndUpdateImotOffsetValue
//  125   UpdateWDTimerValue(WDT_RESET_TIME);
        MOVW     R0,#+5000      
          CFI FunCall UpdateWDTimerValue
        BL       UpdateWDTimerValue
//  126   RestartCommTimer();
          CFI FunCall RestartCommTimer
        BL       RestartCommTimer
//  127   
//  128     Direct_SPI_Init();
          CFI FunCall Direct_SPI_Init
        BL       Direct_SPI_Init
//  129     SD_MOUNT_STATUS = SDcardMount();
          CFI FunCall SDcardMount
        BL       SDcardMount    
        LDR.N    R1,??DataTable3_1
        STRB     R0,[R1, #+0]   
//  130   
//  131   
//  132   while (1)
//  133   {
//  134     WDTimerRefresh();
??main_4:
          CFI FunCall WDTimerRefresh
        BL       WDTimerRefresh 
//  135     OperateLeds();
          CFI FunCall OperateLeds
        BL       OperateLeds    
//  136     ChargerErrorHandler();
          CFI FunCall ChargerErrorHandler
        BL       ChargerErrorHandler
//  137     ProcessUartData();
        MOV      R0,#+4294967295
        STRD     R0,R0,[SP, #+0]
        MOV      R1,#+500       
        ADD      R0,SP,#+8      
          CFI FunCall DebugUartReceive
        BL       DebugUartReceive
        CBNZ.N   R0,??main_5    
          CFI FunCall SwitchLed3On
        BL       SwitchLed3On   
        MOVS     R0,#+1         
          CFI FunCall SetUartType
        BL       SetUartType    
        ADD      R0,SP,#+8      
          CFI FunCall strlen
        BL       strlen         
        UXTH     R1,R0          
        ADD      R0,SP,#+8      
          CFI FunCall ProcessReceivedJsonData
        BL       ProcessReceivedJsonData
          CFI FunCall SwitchLed3Off
        BL       SwitchLed3Off  
??main_5:
        MOV      R2,#+500       
        ADD      R1,SP,#+8      
        MOV      R0,SP          
          CFI FunCall ZigbeeUartReceive
        BL       ZigbeeUartReceive
        CBNZ.N   R0,??main_6    
          CFI FunCall RestartCommTimer
        BL       RestartCommTimer
          CFI FunCall SwitchLed3On
        BL       SwitchLed3On   
        MOVS     R0,#+0         
          CFI FunCall SetUartType
        BL       SetUartType    
        LDRD     R0,R1,[SP, #+0]
          CFI FunCall SetReceivedMACAddress
        BL       SetReceivedMACAddress
        ADD      R0,SP,#+8      
          CFI FunCall strlen
        BL       strlen         
        UXTH     R1,R0          
        ADD      R0,SP,#+8      
          CFI FunCall ProcessReceivedJsonData
        BL       ProcessReceivedJsonData
        MOVS     R0,#+0         
        MOVS     R1,#+0         
          CFI FunCall SetReceivedMACAddress
        BL       SetReceivedMACAddress
          CFI FunCall SwitchLed3Off
        BL       SwitchLed3Off  
??main_6:
        MOV      R1,#+500       
        ADD      R0,SP,#+8      
          CFI FunCall ConfigUartReceive
        BL       ConfigUartReceive
        CBNZ.N   R0,??main_7    
          CFI FunCall SwitchLed3On
        BL       SwitchLed3On   
        MOVS     R0,#+2         
          CFI FunCall SetUartType
        BL       SetUartType    
        ADD      R0,SP,#+8      
          CFI FunCall ProcessReceivedJsonData_config
        BL       ProcessReceivedJsonData_config
          CFI FunCall SwitchLed3Off
        BL       SwitchLed3Off  
          CFI FunCall RestartCommTimer
        BL       RestartCommTimer
//  138     RelayFSM();
??main_7:
          CFI FunCall RelayFSM
        BL       RelayFSM       
//  139     RobotOperate();
          CFI FunCall RobotOperate
        BL       RobotOperate   
//  140     RobotFSM();
          CFI FunCall RobotFSM
        BL       RobotFSM       
//  141     CheckMotorOverLoad();
          CFI FunCall CheckMotorOverLoad
        BL       CheckMotorOverLoad
//  142     ZigbeePoll();
          CFI FunCall ZigbeePoll
        BL       ZigbeePoll     
//  143     DispDebugInfo();
          CFI FunCall DispDebugInfo
        BL       DispDebugInfo  
//  144     CheckAndStoreQC();
          CFI FunCall CheckAndStoreQC
        BL       CheckAndStoreQC
//  145     ControlledCharging();
          CFI FunCall ControlledCharging
        BL       ControlledCharging
//  146     SDcardOperation();
          CFI FunCall SDcardOperation
        BL       SDcardOperation
        B.N      ??main_4       
//  147   }
//  148 }
          CFI EndBlock cfiBlock0
//  149 
//  150 static void ProcessUartData(void)
//  151 {
//  152   uint8_t rxDataBuffer[MAX_UART_RX_BUFFER_LENGTH];
//  153   uint64_t rxMacAddress = DEFAULT_RX_MAC_ADDR;
//  154   if(DebugUartReceive(rxDataBuffer, (sizeof(rxDataBuffer)/sizeof rxDataBuffer[0])) == SUCCESS)
//  155   {
//  156     SwitchLed3On();
//  157     SetUartType(DEBUG_UART);
//  158     ProcessReceivedJsonData(rxDataBuffer,strlen(rxDataBuffer));
//  159     SwitchLed3Off();
//  160   }
//  161   if(ZigbeeUartReceive(&rxMacAddress, rxDataBuffer, (sizeof(rxDataBuffer)/sizeof rxDataBuffer[0])) == SUCCESS)
//  162   {
//  163     RestartCommTimer();
//  164     SwitchLed3On();
//  165     SetUartType(ZIGBEE_UART);
//  166     SetReceivedMACAddress(rxMacAddress);
//  167 #ifdef ZIGBEE_JSON_MODE 
//  168     ProcessReceivedJsonData_config(rxDataBuffer);
//  169 #else
//  170     ProcessReceivedJsonData(rxDataBuffer,strlen(rxDataBuffer));
//  171 #endif
//  172     SetReceivedMACAddress(0);
//  173     SwitchLed3Off();
//  174   }
//  175   if(ConfigUartReceive(rxDataBuffer, (sizeof(rxDataBuffer)/sizeof rxDataBuffer[0])) == SUCCESS)
//  176   {
//  177     SwitchLed3On();
//  178     SetUartType(CONFIG_UART);
//  179     ProcessReceivedJsonData_config(rxDataBuffer);
//  180     SwitchLed3Off();
//  181     RestartCommTimer();
//  182   }
//  183 }
//  184 

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function DispDebugInfo
        THUMB
//  185 void DispDebugInfo (void)
//  186 {
DispDebugInfo:
        PUSH     {R4-R8,LR}     
          CFI R14 Frame(CFA, -4)
          CFI R8 Frame(CFA, -8)
          CFI R7 Frame(CFA, -12)
          CFI R6 Frame(CFA, -16)
          CFI R5 Frame(CFA, -20)
          CFI R4 Frame(CFA, -24)
          CFI CFA R13+24
        VPUSH    {D8-D9}        
          CFI D9 Frame(CFA, -32)
          CFI D8 Frame(CFA, -40)
          CFI CFA R13+40
        SUB      SP,SP,#+296    
          CFI CFA R13+336
//  187   char dispstring[200];
//  188   static uint32_t timeValue = 0;
//  189   uint32_t msTimeCount = GetLPTimerMsTicks();
          CFI FunCall GetLPTimerMsTicks
        BL       GetLPTimerMsTicks
//  190   if((msTimeCount - timeValue) > 2000)
        LDR.N    R1,??DataTable3_2
        LDR      R2,[R1, #+0]   
        SUBS     R2,R0,R2       
        MOVW     R3,#+2001      
        CMP      R2,R3          
        BCC.N    ??DispDebugInfo_0
//  191   {
//  192     timeValue = msTimeCount;
        STR      R0,[R1, #+0]   
//  193     
//  194     snprintf(dispstring, sizeof dispstring, "\nRotateSenseCount : %d, %d, %d, Edge Sensor : %d, %d",
//  195              GetRotateSenseCount(),GetRotateSense1Count(),GetRotateSense2Count(),
//  196              IsEdgeSensor1Detected(), IsEdgeSensor2Detected()); 
          CFI FunCall IsEdgeSensor2Detected
        BL       IsEdgeSensor2Detected
        MOV      R4,R0          
          CFI FunCall IsEdgeSensor1Detected
        BL       IsEdgeSensor1Detected
        MOV      R5,R0          
          CFI FunCall GetRotateSense2Count
        BL       GetRotateSense2Count
        MOV      R6,R0          
          CFI FunCall GetRotateSense1Count
        BL       GetRotateSense1Count
        MOV      R7,R0          
          CFI FunCall GetRotateSenseCount
        BL       GetRotateSenseCount
        LDR.W    R8,??DataTable3_3
        STR      R4,[SP, #+12]  
        MOV      R3,R0          
        STR      R5,[SP, #+8]   
        STR      R6,[SP, #+4]   
        STR      R7,[SP, #+0]   
        MOV      R2,R8          
        MOVS     R1,#+200       
        ADD      R0,SP,#+92     
          CFI FunCall snprintf
        BL       snprintf       
//  197     
//  198     Serial_Debug(dispstring);
        ADD      R0,SP,#+92     
          CFI FunCall Serial_Debug
        BL       Serial_Debug   
//  199     float Imot1 = GetImot1Value();
          CFI FunCall GetImot1Value
        BL       GetImot1Value  
        VMOV.F32 S16,S0         
//  200     float Imot2 = GetImot2Value();
          CFI FunCall GetImot2Value
        BL       GetImot2Value  
        VMOV.F32 S17,S0         
//  201     float Imot3 = GetImot3Value();
          CFI FunCall GetImot3Value
        BL       GetImot3Value  
        VMOV.F32 S18,S0         
//  202     char temp1Str[20], temp2Str[30], temp3Str[30];
//  203     ftoa(Imot1, temp1Str, 2);
        MOVS     R1,#+2         
        ADD      R0,SP,#+8      
        VMOV.F32 S0,S16         
          CFI FunCall ftoa
        BL       ftoa           
//  204     ftoa(Imot2, temp2Str, 2);
        MOVS     R1,#+2         
        ADD      R0,SP,#+60     
        VMOV.F32 S0,S17         
          CFI FunCall ftoa
        BL       ftoa           
//  205     ftoa(Imot3, temp3Str, 2);
        MOVS     R1,#+2         
        ADD      R0,SP,#+28     
        VMOV.F32 S0,S18         
          CFI FunCall ftoa
        BL       ftoa           
//  206     snprintf(dispstring, sizeof dispstring, "\nImot Values-> Imot_1 = %s, Imot_2 = %s, Imot_3 = %s",temp1Str,temp2Str, temp3Str); 
        ADD      R0,SP,#+28     
        ADD      R1,SP,#+60     
        STR      R0,[SP, #+4]   
        STR      R1,[SP, #+0]   
        ADD      R3,SP,#+8      
        ADD      R2,R8,#+56     
        MOVS     R1,#+200       
        ADD      R0,SP,#+92     
          CFI FunCall snprintf
        BL       snprintf       
//  207     Serial_Debug(dispstring);  
        ADD      R0,SP,#+92     
          CFI FunCall Serial_Debug
        BL       Serial_Debug   
//  208 //    Imot1 = GetTemperatureSensorData(TEMPSENSOR_1);
//  209 //    Imot2 = GetTemperatureSensorData(TEMPSENSOR_2);
//  210 //    ftoa(Imot1, temp1Str, 2);
//  211 //    ftoa(Imot2, temp2Str, 2);
//  212 //    snprintf(dispstring, sizeof dispstring, "\nTEMPERATURE-> TEMP_SENSE_1 = %s, TEMP_SENSE_2 = %s",temp1Str,temp2Str); 
//  213 //    Serial_Debug(dispstring); 
//  214     BatteryInfo();
          CFI FunCall BatteryInfo
        BL       BatteryInfo    
//  215   }
//  216 }
??DispDebugInfo_0:
        ADD      SP,SP,#+296    
          CFI CFA R13+40
        VPOP     {D8-D9}        
          CFI D8 SameValue
          CFI D9 SameValue
          CFI CFA R13+24
        POP      {R4-R8,PC}     
          CFI EndBlock cfiBlock1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable3:
        DATA32
        DC32     ?_0            

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable3_1:
        DATA32
        DC32     SD_MOUNT_STATUS

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable3_2:
        DATA32
        DC32     `DispDebugInfo::timeValue`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable3_3:
        DATA32
        DC32     ?_1            

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
?_3:
        DATA8
        DC8 "\012 RTC Initialisation OK"

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
?_4:
        DATA8
        DC8 "\012 EEPROM Initialisation Failed"
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
        DATA
`DispDebugInfo::timeValue`:
        DS8 4

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
?_0:
        DATA8
        DC8 "\012 RTC Initialisation ERROR *~*~*~*~*~*"
        DS8 1

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
?_1:
        DATA8
        DC8 0x0A, 0x52, 0x6F, 0x74, 0x61, 0x74, 0x65, 0x53
        DC8 0x65, 0x6E, 0x73, 0x65, 0x43, 0x6F, 0x75, 0x6E
        DC8 0x74, 0x20, 0x3A, 0x20, 0x25, 0x64, 0x2C, 0x20
        DC8 0x25, 0x64, 0x2C, 0x20, 0x25, 0x64, 0x2C, 0x20
        DC8 0x45, 0x64, 0x67, 0x65, 0x20, 0x53, 0x65, 0x6E
        DC8 0x73, 0x6F, 0x72, 0x20, 0x3A, 0x20, 0x25, 0x64
        DC8 0x2C, 0x20, 0x25, 0x64, 0
        DATA
        DS8 3
        DATA8
        DC8 0x0A, 0x49, 0x6D, 0x6F, 0x74, 0x20, 0x56, 0x61
        DC8 0x6C, 0x75, 0x65, 0x73, 0x2D, 0x3E, 0x20, 0x49
        DC8 0x6D, 0x6F, 0x74, 0x5F, 0x31, 0x20, 0x3D, 0x20
        DC8 0x25, 0x73, 0x2C, 0x20, 0x49, 0x6D, 0x6F, 0x74
        DC8 0x5F, 0x32, 0x20, 0x3D, 0x20, 0x25, 0x73, 0x2C
        DC8 0x20, 0x49, 0x6D, 0x6F, 0x74, 0x5F, 0x33, 0x20
        DC8 0x3D, 0x20, 0x25, 0x73, 0
        DATA
        DS8 3

        END
//  217 
//  218 
//  219 
//  220 
// 
//   5 bytes in section .bss
// 152 bytes in section .rodata
// 646 bytes in section .text
// 
// 646 bytes of CODE  memory
// 152 bytes of CONST memory
//   5 bytes of DATA  memory
//
//Errors: none
//Warnings: none
