/*
 * task_UART.c
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */


static void UartTX(const uint8_t* cmdBuf, uint8_t cmdLen)
{
  EventBits_t eb = xEventGroupGetBits(uartEventGroupHandle);

  /* When TX is running wait for the end of the previous transfer */
  if (eb & Uart__DMA_TX_RUN) {
    /* Block until end of transmission - at max. 1 sec */
    eb = xEventGroupWaitBits(uartEventGroupHandle,
        Uart__DMA_TX_END,
        0,
        0, 1000 / portTICK_PERIOD_MS);
    if (!(eb & Uart__DMA_TX_RUN)) {
      /* Failed to send NMEA message */
      return;
    }
  }

  /* Copy to DMA TX buffer */
  memcpy(g_uartTxDmaBuf, cmdBuf, cmdLen);
  memset(g_uartTxDmaBuf + cmdLen, 0, sizeof(g_uartTxDmaBuf) - cmdLen);

  /* Re-set flags for TX */
  xEventGroupClearBits(uartEventGroupHandle, Uart__DMA_TX_END);
  xEventGroupSetBits(  uartEventGroupHandle, Uart__DMA_TX_RUN);

  /* Start transmission */
  if (HAL_OK != HAL_UART_Transmit_DMA(usartHuart1, g_usartGpsTxDmaBuf, cmdLen))
  {
    /* Drop packet */
  }
}

static void UartRX(void)
{
  const uint16_t dmaBufSize = sizeof(g_uartRxDmaBuf);

  /* Reset working indexes */
  g_uartRxDmaBufLast = g_uartRxDmaBufIdx = 0U;

  /* Start RX DMA */
  if (HAL_UART_Receive_DMA(usartHuart1, g_uartRxDmaBuf, dmaBufSize) != HAL_OK)
  {
    //Error_Handler();
  }

  /* Set RX running flag */
  xEventGroupSetBits(gpscomEventGroupHandle, Uart__DMA_RX_RUN);
}

static void uartServiceRX(void)
{
  taskDISABLE_INTERRUPTS();

  /* Interrupt locked block */
  {
    /* Find last written byte */
    g_uartRxDmaBufIdx = g_uartRxDmaBufLast + strnlen((char*)g_uartRxDmaBuf + g_uartRxDmaBufLast, sizeof(g_uartRxDmaBuf) - g_uartRxDmaBufLast);

    /* Slice DMA buffer data to the line buffer */
    while (g_uartRxDmaBufLast < g_uartRxDmaBufIdx) {
      char c;

      /* Byte copy */
      g_uartRxLineBuf[g_uartRxLineBufIdx++] = c = g_uartRxDmaBuf[g_uartRxDmaBufLast];
      g_uartRxDmaBuf[g_uartRxDmaBufLast++]  = 0;

      /* Process line buffer after line feed (LF) */
      if (c == '\n' || g_uartRxLineBufIdx == sizeof(g_uartRxLineBuf)) {
        uint8_t buf[ sizeof(g_uartRxLineBuf) ];
        uint8_t len;

        /* Create a work copy of the line buffer - DMA disabled block */
        {
          /* Make a work copy */
          len = g_uartRxLineBufIdx;
          memcpy(buf, g_uartRxLineBuf, len);

          /* Clear the line buffer */
          g_uartRxLineBufIdx = 0;
          memset(g_uartRxLineBuf, 0, sizeof(g_uartRxLineBuf));
        }

        /* Work on line buffer content */
        uartInterpreter(buf, len);
      }
    }

    /* Loop around */
    if (g_uartRxDmaBufLast >= sizeof(g_uartRxDmaBuf)) {
      g_uartRxDmaBufLast = 0U;
    }
  }

  taskENABLE_INTERRUPTS();
}


static void uartMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const uartCmds_t        cmd     = (uartCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgUart__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_uartStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Activation flag */
      s_uart_enable = 1U;

      /* Init module */
      uartInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Network_UART, 0U, MsgUart__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

#if 0
  case MsgUart__DeInitDo:
    {
      /* Init module */
      uartDeInit();

      /* Deactivate flag */
      s_uart_enable = 0U;
    }
    break;
#endif

  default: { }
  }  // switch (cmd)
}


/* Tasks */

void uartTaskInit(void)
{
  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_uartStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void uartTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2uart_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_UART, 1UL);                  // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    uartMsgProcess(msgLen, msgAry);
  }
}
