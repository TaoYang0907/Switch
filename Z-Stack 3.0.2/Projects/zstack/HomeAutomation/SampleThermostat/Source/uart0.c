#include "uart0.h"
#include "hal_uart.h"
#include "string.h"
#include "user_printf.h"
#include "OSAL.h"
#include "comdef.h"
#include "ZComDef.h"
#include "OnBoard.h"

#define NO_TASK_ID 0xFF

void Uart0_Handle( void );
void Uart0_Reset( void );

uint8 UART0_RX_BUFF[UART0_RX_BUFF_MAX];//接收缓存区
uint8 UART0_RX_STA = 0;                //接收状态标记
uint8 UART0_RX_LEN = 0;                //接收数据长度

static uint8 registeredUart0TaskID = NO_TASK_ID;

uint8 RegisterForUart0( uint8 task_id )
{
  if ( registeredUart0TaskID == NO_TASK_ID )
  {
    registeredUart0TaskID = task_id;
    return ( true );
  }
  else
    return ( false );
}

void Uart0_Init(uint8 baudRate)
{
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = baudRate;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = UART0_THRESHOLD;
  uartConfig.rx.maxBufSize        = UART0_RX_BUFF_MAX;
  uartConfig.tx.maxBufSize        = UART0_TX_BUFF_MAX;
  uartConfig.idleTimeout          = UART0_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = Uart0_Process;

  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
}

void Uart0_Process( uint8 port, uint8 event )
{
  UART0_RX_STA = 0;
  UART0_RX_LEN = 0;
  uint8 Res;
  (void)event;  // Intentionally unreferenced parameter

  while (Hal_UART_RxBufLen(port))
  {
    HalUARTRead (port, &Res, 1);//读取一个字节
    UART0_RX_BUFF[UART0_RX_STA&0x3F]=Res ;
    UART0_RX_STA++;

    if((UART0_RX_STA&0x80)==0)//接收未完成
    {
      if(UART0_RX_STA&0x40)//前一个接收到的数据结束符的“前一半”
      {
        if     (Res==0x0A)
        {
          UART0_RX_STA |= 0x80;//接收完成了
        }
        else if(Res!=0x0D)
        {
          UART0_RX_STA &= 0xBF;//不是结束符，则将前半个结束符标志置0
        }
      }
      else
      {
        if(Res==0x0D)
        {
          UART0_RX_STA |= 0x40;
        }
      }
    }

    if((UART0_RX_STA&0x80)==0x80)
    {
      UART0_RX_LEN = (UART0_RX_STA&0x3F)-2;
      Uart0_Handle();
      Uart0_Reset();
      break;
    }
  }
}

void Uart0_Handle( void )
{
//  if(strstr((const char*)UART0_RX_BUFF,"LED1_ON"))
//  {
//    printf("Router LED1_ON\n");
//    UART0_t *msgPtr;
//    msgPtr = (UART0_t *)osal_msg_allocate( sizeof(UART0_t) );
//    if ( msgPtr )
//    {
//      msgPtr->hdr.event = UART0_MESSAGE_LED1_ON;
//
//      osal_msg_send( registeredUart0TaskID, (uint8 *)msgPtr );
//    }
//  }
//  else if(strstr((const char*)UART0_RX_BUFF,"LED1_OFF"))
//  {
//    printf("Router LED1_OFF\n");
//    UART0_t *msgPtr;
//    msgPtr = (UART0_t *)osal_msg_allocate( sizeof(UART0_t) );
//    if ( msgPtr )
//    {
//      msgPtr->hdr.event = UART0_MESSAGE_LED1_OFF;
//
//      osal_msg_send( registeredUart0TaskID, (uint8 *)msgPtr );
//    }
//  }
}

void Uart0_Reset( void )
{
  memset(UART0_RX_BUFF,0,UART0_RX_BUFF_MAX);
  UART0_RX_STA = 0;
}