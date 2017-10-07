/*******************************************************************************
*实现功能 ： 基于协议栈的按键实验
*实验平台 ： 秉火 Zigbee 开发板
*硬件连接 ： LED1 P1_0 
             LED2 P1_1
             LED3 P1_4
             BUT1 P0_4
             BUT2 P0_5
*******************************************************************************/

#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_adc.h"

#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

/* 设置地锁设备编号 */
#define Devic_Num 4 //0-99
/* 字符串操作 */
#include <string.h>
#include "stdio.h"

uint8 up_flag = 0;      //抬起标志
uint8 dowm_flag = 0;    //落下标志
uint8 up_over_flag = 0; //抬起限位标志

void Delayms(unsigned int xms) //i=xms 即延时 i 毫秒
{

  unsigned int i;
  
  for(i=xms;i>0;i--)
       MicroWait (1000);     // Wait 1ms
}

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
uint8 AD1_TaskID;
uint8 MY_TaskID;
uint8 Beep_Warn_TaskID;

devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );


/*********************************************************************
 * @fn      AD_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void AD_Init(uint8 task_id)
{
  AD1_TaskID = task_id;
  //AD 配置
  P0DIR |= 0x3F;
  HalAdcSetReference(HAL_ADC_REF_AVDD);
  //这里调试用 LED
  P2DIR |= 0x01;
  P2_0 = 0;
  
  //osal_start_timerEx(AD1_TaskID,0x01,500);

}

/* Motor IO Init */
#define Led_On (P1_3 = 1)
#define Led_Off (P1_3 = 0)
#define Beep_On (P1_2 = 1)
#define Beep_Off (P1_2 = 0)
#define Motor_A_H (P1_1 = 1)
#define Motor_A_L (P1_1 = 0)
#define Motor_B_H (P1_0 = 1)
#define Motor_B_L (P1_0 = 0)
#define Motor_Up {Motor_B_H;Motor_A_L;Led_On;Beep_On;}
#define Motor_Dowm {Motor_A_H;Motor_B_L;Led_On;Beep_On;}
#define Motor_Stop {Motor_A_L;Motor_B_L;Led_Off;Beep_Off;}//Delayms(100);Led_On;Beep_On;Delayms(100);Led_Off;Beep_Off;}
void Motor_IO_Init(void)
{
  P1DIR |= 0x0F;
  //Motor_A_L;
  //Motor_B_L;
  Motor_Stop;
  
  P1_2 = 0;
  //P1_3 = 0;
}
void MY_Test_Init(uint8 task_id)
{
  MY_TaskID = task_id;
    osal_start_timerEx(MY_TaskID,0x01,500);
}
void Beep_Warn_Init(uint8 task_id)
{
  Beep_Warn_TaskID = task_id;
}
 void Start_State(void)
{
 //初始状态 趴下
   Motor_A_H;
   Motor_B_L;
   dowm_flag = 1;
   up_flag = 0;
   up_over_flag = 0;
   osal_start_timerEx(AD1_TaskID,0x01,500);
}


void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  ///////////////// by Cavani //////////////////
  Motor_IO_Init();
  MT_UartInit ();    
  MT_UartRegisterTaskID(task_id);
  
  HalUARTWrite(0,"The test is KEY.\n", 17);
    
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif
  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;//0xFFFC

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}


uint8 K2_Base = 0,AD_D_H_Base = 0,AD_U_H_Base = 0;
uint16 AD_data,AD_data2;
int lend = 0,lend2 = 0;
int data_huancong[20],data_huancong2[20];
uint8 s_data[60],s_data2[60],s2_data[60],s2_data2[60];

uint16 MY_ProcessEvent( uint8 task_id, uint16 events )
{
  uint8 i;
  uint8 *str;
  uint8 *buf_send;
  buf_send = &s_data[1];//第0元素用于存储发送长度
  str = s_data2;
 //osal_start_timerEx(AD1_TaskID,0x01,500);
 P2_0 = ~P2_0;
  //AD 采集 电机状态
  AD_data = HalAdcRead(HAL_ADC_CHANNEL_6,HAL_ADC_RESOLUTION_14);

  
}

uint16 AD1_ProcessEvent( uint8 task_id, uint16 events )
{
  uint8 i;
  uint8 *str;
  uint8 *buf_send;
  buf_send = &s_data[1];//第0元素用于存储发送长度
  str = s_data2;
 //osal_start_timerEx(AD1_TaskID,0x01,500);
 // P2_0 = ~P2_0;
  //AD 采集  电机状态
  AD_data = HalAdcRead(HAL_ADC_CHANNEL_6,HAL_ADC_RESOLUTION_14);
 
#if 1
  data_huancong[0]=Devic_Num;//Device_number;
  data_huancong[1]=AD_data;
  data_huancong[2]=P1_5;
  data_huancong[3]=0;
  data_huancong[4]=0;
  data_huancong[5]=0;
  data_huancong[6]=0;
  data_huancong[7]=0;
  data_huancong[8]=0;
  data_huancong[9]=0;
  data_huancong[10]=0;
  data_huancong[11]=0;
   
  for(i=0;i<lend;i++)
  {
   s_data[i] = 0;
   s_data2[i] = 0;
  }
  for(i=0;i<12;i++)//data_huancong的元素个数次循环
  {
    if(i==1 || i==6)
      sprintf(str,"%X,",data_huancong[i]);
    else if(i==11)
      sprintf(str,"%d",data_huancong[i]);
    else
      sprintf(str,"%d,",data_huancong[i]);
    strcat(buf_send,str);//连接字符串
  }
  lend = (byte)osal_strlen( (void*)buf_send );
  s_data[0]=lend+1;
  
  
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       s_data[0],
                       s_data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
#endif
  
  if(dowm_flag == 1 && AD_data >= 200)
  {
     AD_D_H_Base++ ;
  }
  else if(up_flag == 1 && AD_data >=200)
  {
    AD_U_H_Base++ ;
  }
  else
  {
     AD_D_H_Base = 0;
     AD_U_H_Base = 0;
  }
  if(AD_D_H_Base >=2)
  {
    dowm_flag = 0;
    up_flag = 0;
    Motor_Stop;
    
  }
  
  if(AD_U_H_Base >=2)
  {
    dowm_flag = 1;
    Motor_Stop;
    Delayms(300);
    Motor_Dowm;
  }
  if(dowm_flag == 1 || up_flag == 1)
  osal_start_timerEx(AD1_TaskID,0x01,500);
  return 0;//(events ^ 0x04);
}

uint16 Beep_Warn_ProcessEvent( uint8 task_id, uint16 events )
{
  //如果up_over_flag=1，启动一次蜂鸣器任务，如果K2=0，蜂鸣器响，指示灯亮
  if(up_over_flag == 1)
  {
    osal_start_timerEx(Beep_Warn_TaskID,0x01,500);
    if(P1_5 == 0)
     // P2_0 = 0;
    {
      Beep_On;
      Led_On;
    }
    //如果K2=1关闭蜂鸣器和指示灯
    else //P2_0 = 1;
    {
      Beep_Off;
      Led_Off;
    }
  }
  return 0;//(events ^ 0x04);
}


uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          HalUARTWrite(0,"The pressed KEY is:", 19);
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            P2_0 = 1;
            // Start sending the periodic message in a regular interval.
            //osal_start_timerEx( SampleApp_TaskID,
               //               SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                   //           SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    SampleApp_SendPeriodicMessage();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}



void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_6 ) 
  { 
    //地锁收到up指令后将up_flag置1
    if(up_flag == 1)
    {
      //如果K2(P1_5)=1，电机停止，up_flag置0，up_over_flag=1,启动蜂鸣器任务
      Delayms(300);//i=xms 即延时 i 毫秒
      if(P1_5 == 1)
      {
        Motor_Stop;
        up_flag = 0;
        up_over_flag = 1;
        osal_start_timerEx(Beep_Warn_TaskID,0x01,500);
      }
    }
    //P1_5 = 1;
   // HalUARTWrite(0,"K1",2); //提示被按下的是KEY1 
   // HalUARTWrite(0,"\n",1); //
    //HalLedBlink( HAL_LED_1, 2, 50, 500 ); //LED1闪烁2次，每次为500ms，点亮时间为50%
  }
#if 0
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
#endif
}


void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
    uint16 flashTime;
    uint8 i;
    uint8 *str;
    uint8 *buf_send;
  switch ( pkt->clusterId)
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;
    case SAMPLEAPP_COM_CLUSTERID:     //如果是串口透传的信息
      if(((pkt->cmd.Data[1]-'0')*10+pkt->cmd.Data[2]-'0')==Devic_Num)
      {
        switch (pkt->cmd.Data[3]-'0')
        {
          case 7:
             Motor_Up;
             Beep_On;
             Led_On;
             up_flag = 1;
             dowm_flag = 0;
             K2_Base = 0;
             osal_start_timerEx(AD1_TaskID,0x01,500);
          break;
          
          case 8:
             Motor_Dowm;
             Beep_On;
             Led_On;
             dowm_flag = 1;
             up_flag = 0;
             up_over_flag = 0;
             osal_start_timerEx(AD1_TaskID,0x01,500);
          break; 
          
          case 9:
             Motor_Stop;
             Beep_Off;
             Led_Off;
             up_flag = 0;
             dowm_flag = 0;
             up_over_flag = 0;
          break; 
          
          case 6:
                  
            buf_send = &s2_data[1];//第0元素用于存储发送长度
            str = s2_data2;
            //AD 采集  电池电压
            AD_data2 = HalAdcRead(HAL_ADC_CHANNEL_7,HAL_ADC_RESOLUTION_14);
            AD_data2 = 0.02*AD_data2;
             //电量百分比转换 AD_data2*0.0227 即为相对 6.8V 的电量百分数
  #if 1
            data_huancong2[0]=AD_data2;//Device_number;
            data_huancong2[1]=P1_5;
            data_huancong2[2]=0;
            data_huancong2[3]=0;
            data_huancong2[4]=0;
            data_huancong2[5]=0;
            data_huancong2[6]=0;
            data_huancong2[7]=0;
            data_huancong2[8]=0;
            data_huancong2[9]=0;
            data_huancong2[10]=0;
            data_huancong2[11]=0;
             
            for(i=0;i<lend2;i++)
            {
             s2_data[i] = 0;
             s2_data2[i] = 0;
            }
            for(i=0;i<12;i++)//data_huancong的元素个数次循环
            {
              if(i==1 || i==6)
                sprintf(str,"%X,",data_huancong2[i]);
              else if(i==11)
                sprintf(str,"%d",data_huancong2[i]);
              else
                sprintf(str,"%d,",data_huancong2[i]);
              strcat(buf_send,str);//连接字符串
            }
            lend2 = (byte)osal_strlen( (void*)buf_send );
            s2_data[0]=lend2+1;
            
            
            if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                                 SAMPLEAPP_PERIODIC_CLUSTERID,
                                 s2_data[0],
                                 s2_data,
                                 &SampleApp_TransID,
                                 AF_DISCV_ROUTE,
                                 AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
                  
            {
            }
  #endif
          break;
        }
      }
      break;
    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  P2_0 = ~P2_0;
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/
