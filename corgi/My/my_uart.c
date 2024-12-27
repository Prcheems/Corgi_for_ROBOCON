#include "my_uart.h"
#include "usart.h"
#include "A1_motor.h"
#include "string.h"
#include "imu.h"
#include "handkey.h"
#include "vision.h"
#include "dog.h"
#include "Bluetooth.h"
#include "sbus.h"

#define PI 3.14159265f
USART_RECEIVETYPE Usart1Type1;
USART_RECEIVETYPE Usart2Type1;
USART_RECEIVETYPE Usart3Type1;
USART_RECEIVETYPE Usart4Type1;
USART_RECEIVETYPE Usart5Type1;
USART_RECEIVETYPE Usart6Type1;
USART_RECEIVETYPE Usart7Type1;
USART_RECEIVETYPE Usart8Type1;

extern A1_Motor_Struct motor[8];
static void Motor_DataProcess(uint8_t *pData, MOTOR_recv *recv); // 78×Ö½Ú

extern MOTOR_recv motor_recv;

void usart1_start_recv() // imu TTL-RS232
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Usart1Type1.RX_pData, 100);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void usart2_start_recv() // À¶ÑÀ
{
  Bluetooth_Recv_Start(&huart2, &hdma_usart2_rx);
}
void usart3_start_recv() // ÊÓ¾õ
{
  Vision_Recv_Start(&huart3, &hdma_usart3_rx);
}

void usart6_start_recv() // ÊÖ±ú
{
  SB_USART_Start(&huart6, &hdma_usart6_rx);
}

HAL_StatusTypeDef usart4_start_recv(void) // leg[0] motor[0] motor[1] TTL-RX485
{
  HAL_StatusTypeDef status;
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, Usart4Type1.RX_pData, RX_LEN1);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
  return status;
}
HAL_StatusTypeDef usart8_start_recv(void) // leg[1] motor[2] motor[3] TTL-RX485
{
  HAL_StatusTypeDef status;
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart8, Usart8Type1.RX_pData, RX_LEN1);
  __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
  return status;
}
HAL_StatusTypeDef usart5_start_recv(void) // leg[2] motor[4] motor[5] TTL-RX485
{
  HAL_StatusTypeDef status;
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Usart5Type1.RX_pData, RX_LEN1);
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
  return status;
}
HAL_StatusTypeDef usart7_start_recv(void) // leg[3] motor[6] motor[7] TTL-RX485
{
  HAL_StatusTypeDef status;
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart7, Usart7Type1.RX_pData, RX_LEN1);
  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  return status;
}

void usart_start_recv(void)
{
  // ÍÓÂÝÒÇ
  usart1_start_recv();

  // µç»ú
  usart4_start_recv(); // ÍÈ0
  usart8_start_recv(); // ÍÈ1
  usart5_start_recv(); // ÍÈ2
  usart7_start_recv(); // ÍÈ3

  // ÊÖ±ú
  usart6_start_recv();

  // À¶ÑÀ
  usart2_start_recv();

  // ÊÓ¾õ
  usart3_start_recv();

  motor[0].usart_flag = true;
  motor[1].usart_flag = true;
  motor[2].usart_flag = true;
  motor[3].usart_flag = true;
  motor[4].usart_flag = true;
  motor[5].usart_flag = true;
  motor[6].usart_flag = true;
  motor[7].usart_flag = true;
}

uint8_t cnti = 0;
void A1_All_Send(void)
{
  switch (cnti)
  {
  case 0:
  {
    if (motor[0].usart_flag == true)
      A1_Motor_Send_Data(&motor[0], &huart4);
    break;
  }
  case 1:
  {
    if (motor[2].usart_flag == true)
      A1_Motor_Send_Data(&motor[2], &huart8);
    break;
  }
  case 2:
  {
    if (motor[4].usart_flag == true)
      A1_Motor_Send_Data(&motor[4], &huart5);
    break;
  }
  case 3:
  {
    if (motor[6].usart_flag == true)
      A1_Motor_Send_Data(&motor[6], &huart7);
    break;
  }
  case 4:
  {
    if (motor[1].usart_flag == true)
      A1_Motor_Send_Data(&motor[1], &huart4);
    break;
  }
  case 5:
  {
    if (motor[3].usart_flag == true)
      A1_Motor_Send_Data(&motor[3], &huart8);
    break;
  }
  case 6:
  {
    if (motor[5].usart_flag == true)
      A1_Motor_Send_Data(&motor[5], &huart5);
    break;
  }
  case 7:
  {
    if (motor[7].usart_flag == true)
      A1_Motor_Send_Data(&motor[7], &huart7);
    break;
  }
  }
  cnti++;
  if (cnti > 7)
    cnti = 0;
}

void usart_reset(void)
{
  for (int i = 0; i <= 3; i++)
  {
    if ((motor[2 * i].last_measure.Pos == motor[2 * i].measure.Pos) && (motor[2 * i + 1].last_measure.Pos == motor[2 * i + 1].measure.Pos) &&
        (motor[2 * i].usart_flag == true) && (motor[2 * i + 1].usart_flag == true))
    {
      motor[2 * i].usart_cnt++;
      motor[2 * i + 1].usart_cnt++;
    }
    else
    {
      motor[2 * i].usart_cnt = 0;
      motor[2 * i + 1].usart_cnt = 0;
    }
  }

  for (int i = 0; i <= 7; i++)
  {
    if ((motor[i].usart_cnt >= 20) && (motor[i].usart_flag == true))
      motor[i].usart_flag = false;
  }

  if (motor[0].usart_flag == false)
  {
    MX_UART4_Init();
    usart4_start_recv();
    motor[0].usart_flag = true;
    motor[1].usart_flag = true;
    motor[0].usart_cnt = 0;
    motor[1].usart_cnt = 0;
  }
  if (motor[2].usart_flag == false)
  {
    MX_UART8_Init();
    usart8_start_recv();
    motor[2].usart_flag = true;
    motor[3].usart_flag = true;
    motor[2].usart_cnt = 0;
    motor[3].usart_cnt = 0;
  }
  if (motor[4].usart_flag == false)
  {
    MX_UART5_Init();
    usart5_start_recv();
    motor[4].usart_flag = true;
    motor[5].usart_flag = true;
    motor[4].usart_cnt = 0;
    motor[5].usart_cnt = 0;
  }
  if (motor[6].usart_flag == false)
  {
    MX_UART7_Init();
    usart7_start_recv();
    motor[6].usart_flag = true;
    motor[7].usart_flag = true;
    motor[6].usart_cnt = 0;
    motor[7].usart_cnt = 0;
  }

  for (int i = 0; i <= 7; i++)
  {
    motor[i].last_measure.Pos = motor[i].measure.Pos;
  }
}

uint16_t size;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if IS_USE_FREERTOS
  uint32_t status_value;
  status_value = taskENTER_CRITICAL_FROM_ISR(); // ÁÙ½ç¶Î´úÂë±£»¤
#endif

  size = Size;
  if (huart->Instance == USART1) // imu
  {
    HAL_UART_DMAStop(huart);
    imu_data_process(Usart1Type1.RX_pData);
    usart1_start_recv();
  }

  if (huart->Instance == USART2)
  {
    Bluetooth_Restart_IT(&huart2, &hdma_usart2_rx);
    Gamepad_stop_flag = 0;
  }

  if (huart->Instance == USART3)
  {
    Vision_Restart_IT(&huart3, &hdma_usart3_rx);
  }

  if (huart->Instance == USART6)
  {
    SBUS_Rcv_Prcs(RxTemp_2);
    SB_USART_Start(&huart6, &hdma_usart6_rx);
  }

  if (huart->Instance == UART4) // ÍÈ0
  {
    uint16_t i = 0;
    while (usart4_start_recv() != HAL_OK)
    {
      i++;
      if (i > 10000)
      {
        huart4.RxState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(&huart4);
        i = 0;
      }
    }

    HAL_UART_DMAStop(huart);
    Motor_DataProcess(Usart4Type1.RX_pData, &motor_recv);
    if (Usart4Type1.RX_pData[0] == 0xFE && Usart4Type1.RX_pData[1] == 0xEE)
    {
      uint8_t id = Usart4Type1.RX_pData[2];
      if (id == 0 || id == 1)
      {
        motor[id].measure.Pos = motor_recv.motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.1f;
        motor[id].measure.W = motor_recv.motor_recv_data.Mdata.W / 128.0f / 9.1f;
        motor[id].measure.T = motor_recv.motor_recv_data.Mdata.T / 256.0f * 9.1f;
      }
    }
    usart4_start_recv();
  }

  if (huart->Instance == UART8) // ÍÈ1
  {
    uint16_t i = 0;
    while (usart8_start_recv() != HAL_OK)
    {
      i++;
      if (i > 10000)
      {
        huart8.RxState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(&huart8);
        i = 0;
      }
    }

    HAL_UART_DMAStop(huart);
    Motor_DataProcess(Usart8Type1.RX_pData, &motor_recv);
    if (Usart8Type1.RX_pData[0] == 0xFE && Usart8Type1.RX_pData[1] == 0xEE)
    {
      uint8_t id = Usart8Type1.RX_pData[2];
      if (id == 0 || id == 1)
      {
        id += 2;
        motor[id].measure.Pos = motor_recv.motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.1f;
        motor[id].measure.W = motor_recv.motor_recv_data.Mdata.W / 128.0f / 9.1f;
        motor[id].measure.T = motor_recv.motor_recv_data.Mdata.T / 256.0f * 9.1f;
      }
    }
    usart8_start_recv();
  }

  if (huart->Instance == UART5) // ÍÈ2
  {
    uint16_t i = 0;
    while (usart5_start_recv() != HAL_OK)
    {
      i++;
      if (i > 10000)
      {
        huart5.RxState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(&huart5);
        i = 0;
      }
    }

    HAL_UART_DMAStop(huart);
    Motor_DataProcess(Usart5Type1.RX_pData, &motor_recv);
    if (Usart5Type1.RX_pData[0] == 0xFE && Usart5Type1.RX_pData[1] == 0xEE)
    {
      uint8_t id = Usart5Type1.RX_pData[2];
      if (id == 0 || id == 1)
      {
        id += 4;
        motor[id].measure.Pos = motor_recv.motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.1f;
        motor[id].measure.W = motor_recv.motor_recv_data.Mdata.W / 128.0f / 9.1f;
        motor[id].measure.T = motor_recv.motor_recv_data.Mdata.T / 256.0f * 9.1f;
      }
    }
    usart5_start_recv();
  }

  if (huart->Instance == UART7) // ÍÈ3
  {
    uint16_t i = 0;
    while (usart7_start_recv() != HAL_OK)
    {
      i++;
      if (i > 10000)
      {
        huart7.RxState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(&huart7);
        i = 0;
      }
    }

    HAL_UART_DMAStop(huart);
    Motor_DataProcess(Usart7Type1.RX_pData, &motor_recv);
    if (Usart7Type1.RX_pData[0] == 0xFE && Usart7Type1.RX_pData[1] == 0xEE)
    {
      uint8_t id = Usart7Type1.RX_pData[2];
      if (id == 0 || id == 1)
      {
        id += 6;
        motor[id].measure.Pos = motor_recv.motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.1f;
        motor[id].measure.W = motor_recv.motor_recv_data.Mdata.W / 128.0f / 9.1f;
        motor[id].measure.T = motor_recv.motor_recv_data.Mdata.T / 256.0f * 9.1f;
      }
    }
    usart7_start_recv();
  }

#if IS_USE_FREERTOS
  taskEXIT_CRITICAL_FROM_ISR(status_value);
#endif
}

int Gamepad_stop_flag = 0;
float get_float_from_4u8(unsigned char *p)
{
  float a;
  unsigned char *r;
  r = (unsigned char *)&a;
  *r = p[0];
  r++;
  *r = p[1];
  r++;
  *r = p[2];
  r++;
  *r = p[3];
  return (a);
}
static void Motor_DataProcess(uint8_t *pData, MOTOR_recv *recv) // 78×Ö½Ú
{

  if (pData[0] == 0xFE && pData[1] == 0xEE)
  {
    int id = 0;
    recv[id].motor_recv_data.head.start[0] = pData[0];
    recv[id].motor_recv_data.head.start[1] = pData[1];
    recv[id].motor_recv_data.head.motorID = pData[2];
    recv[id].motor_recv_data.head.reserved = pData[3];
    recv[id].motor_recv_data.Mdata.mode = pData[4];
    recv[id].motor_recv_data.Mdata.ReadBit = pData[5];
    recv[id].motor_recv_data.Mdata.Temp = pData[6];
    recv[id].motor_recv_data.Mdata.MError = pData[7];
    recv[id].motor_recv_data.Mdata.Read.u8[0] = pData[8];
    recv[id].motor_recv_data.Mdata.Read.u8[1] = pData[9];
    recv[id].motor_recv_data.Mdata.Read.u8[2] = pData[10];
    recv[id].motor_recv_data.Mdata.Read.u8[3] = pData[11];
    recv[id].motor_recv_data.Mdata.T = (pData[12] | pData[13] << 8);
    recv[id].motor_recv_data.Mdata.W = (pData[14] | pData[15] << 8);
    recv[id].motor_recv_data.Mdata.LW = get_float_from_4u8(&pData[16]);
    recv[id].motor_recv_data.Mdata.W2 = (pData[20] | pData[21] << 8);
    recv[id].motor_recv_data.Mdata.LW2 = get_float_from_4u8(&pData[22]);
    recv[id].motor_recv_data.Mdata.Acc = (pData[26] | pData[27] << 8);
    recv[id].motor_recv_data.Mdata.OutAcc = (pData[28] | pData[29] << 8);
    recv[id].motor_recv_data.Mdata.Pos = (pData[30] | pData[31] << 8 | pData[32] << 16 | pData[33] << 24);
    recv[id].motor_recv_data.Mdata.Pos2 = (pData[34] | pData[35] << 8 | pData[36] << 16 | pData[37] << 24);
    recv[id].motor_recv_data.Mdata.gyro[0] = (pData[38] | pData[39] << 8);
    recv[id].motor_recv_data.Mdata.gyro[1] = (pData[40] | pData[41] << 8);
    recv[id].motor_recv_data.Mdata.gyro[2] = (pData[42] | pData[43] << 8);
    recv[id].motor_recv_data.Mdata.acc[0] = (pData[44] | pData[45] << 8);
    recv[id].motor_recv_data.Mdata.acc[1] = (pData[46] | pData[47] << 8);
    recv[id].motor_recv_data.Mdata.acc[2] = (pData[48] | pData[49] << 8);
    recv[id].motor_recv_data.Mdata.Fgyro[0] = (pData[50] | pData[51] << 8);
    recv[id].motor_recv_data.Mdata.Fgyro[1] = (pData[52] | pData[53] << 8);
    recv[id].motor_recv_data.Mdata.Fgyro[2] = (pData[54] | pData[55] << 8);
    recv[id].motor_recv_data.Mdata.Facc[0] = (pData[56] | pData[57] << 8);
    recv[id].motor_recv_data.Mdata.Facc[1] = (pData[58] | pData[59] << 8);
    recv[id].motor_recv_data.Mdata.Facc[2] = (pData[60] | pData[61] << 8);
    recv[id].motor_recv_data.Mdata.Fmag[0] = (pData[62] | pData[63] << 8);
    recv[id].motor_recv_data.Mdata.Fmag[1] = (pData[64] | pData[65] << 8);
    recv[id].motor_recv_data.Mdata.Fmag[2] = (pData[66] | pData[67] << 8);
    recv[id].motor_recv_data.Mdata.Ftemp = pData[68];
    recv[id].motor_recv_data.Mdata.Force16 = (pData[69] | pData[70] << 8);
    recv[id].motor_recv_data.Mdata.Force8 = pData[71];
    recv[id].motor_recv_data.Mdata.FError = pData[72];
    recv[id].motor_recv_data.Mdata.Res[0] = pData[73];
    recv[id].motor_recv_data.CRCdata.u8[0] = pData[74];
    recv[id].motor_recv_data.CRCdata.u8[1] = pData[75];
    recv[id].motor_recv_data.CRCdata.u8[2] = pData[76];
    recv[id].motor_recv_data.CRCdata.u8[3] = pData[77];
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart1_start_recv();
  }
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart2_start_recv();
  }
  if (huart->Instance == USART3)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart3_start_recv();
  }
  if (huart->Instance == UART4)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart4_start_recv();
  }
  if (huart->Instance == UART5)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart5_start_recv();
  }
  if (huart->Instance == USART6)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart6_start_recv();
  }
  if (huart->Instance == UART7)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart7_start_recv();
  }
  if (huart->Instance == UART8)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart8_start_recv();
  }
}
