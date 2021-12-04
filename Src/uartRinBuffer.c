/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: 			 																																																											*	
*																																																																						*
*		File Name: uartRingBuffer.c																																																							*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification: 																																																													*
*																																																																						*	
*		Description:	Ring Buffer Uygulamalari																																																	*
*																																																																						*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*			Projede Drivers/STM32F3xx_HAL_Driver altinda UART,USART  icin kullanilacak .c dosyalarini  ekle:																			*
*				stm32f3xx_hal_uart.c																																																								*
*				stm32f3xx_hal_uart_ex.c																																																							*
*				stm32f3xx_hal_usart.c																																																								*
*				stm32f3xx_hal_usart_ex.c																																																						*
*																																																																						*
*				stm32f3xx_hal_conf.h icerisindeki 																																																	*
*				 #define HAL_UART_MODULE_ENABLED  																																																	*	 
* 			 #define HAL_USART_MODULE_ENABLED 																																																	*
*																																																																						*
*	 [2] 																																																																			* 
*																																																																						*
*    Bu uygulamada sayin ÇOSKUN TASDEMIR'in  egitimlerinde gerçeklestirmis oldugu ring buffer uygulamalarini																*
*    kendi uygulamalarima adapte edilecegim sekilde derlemeye çalistim.																																			*
*    Çoskun TASDEMIR github adresi : github.com/ctasdemir																																										*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "uartRingBuffer.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct UART_Buffer_Type{
	uint8_t buffer[BUFFER_SIZE];
	uint32_t head_pointer;
	uint32_t tail_pointer;
}UART_Buffer_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
volatile UART_Buffer_t UART_BufferRX;
volatile UART_Buffer_t UART_BufferTX;

/* Private function prototypes -----------------------------------------------*/
int8_t UART_is_buffer_empty(volatile UART_Buffer_t* buffer);

/* Private function-----------------------------------------------------------*/

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  
	/* Peripheral clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	PA2     ------> USART2_TX
	PA3     ------> USART2_RX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
	
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	/* 4- Enable UART Receive Data Register Not Empty */
  SET_BIT(USART2->CR1, USART_CR1_RXNEIE);

}

/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA 
  *         used for USART data transmission     
  */
void USART2_IRQHandler(void)
{  
  uint32_t isrflags = USART2->ISR;
  uint32_t control_reg1 = USART2->CR1;
	
	
    /* UART in mode Receiver */
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((control_reg1 & USART_CR1_RXNEIE) != RESET))
    {
		/* Read one byte from the receive data register */ 	
			
			UART_BufferRX.buffer[UART_BufferRX.head_pointer] = USART2->RDR;
			
			UART_BufferRX.head_pointer = UART_BufferRX.head_pointer + 1;
			
			if(UART_BufferRX.head_pointer == BUFFER_SIZE)
			{
				UART_BufferRX.head_pointer = 0;
			}
			
      return;
    }  


  /* UART in mode Transmitter */
  if(((isrflags & USART_ISR_TXE) != RESET) && ((control_reg1 & USART_CR1_TXEIE) != RESET))
  {    
		if(UART_BufferTX.head_pointer != UART_BufferTX.tail_pointer)
		{
			// Send one byte from Transmit buffer
			USART2->TDR = UART_BufferTX.buffer[UART_BufferTX.tail_pointer++];
			
			if(UART_BufferTX.tail_pointer == BUFFER_SIZE)
			{
				UART_BufferTX.tail_pointer = 0;
			}			
		}
		else
		{
		/* Disable the UART Transmit Data Register Empty Interrupt */
      CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);
		}			
		
    return;
  }
	
}

/**
  * @brief 	Bu fonksiyon girilen veriyi ring buffer'ina alarak UART Transmit Kesme fonksiyonunu baslatir
	*					Transmit kesmesi ile veri aktarimi saglanir.
  * @param 	isaretsiz 8 bitlik formatta gönderilmek istenilen veri giris parametresidir 
  * @retval yok
	* @note 	Alinan veri ring buffer tamponuna kaydedilir, Head göstergesi arttirilir, Tampon doluysa Head göstergesi basa döner.
  */
void UART_send_byte(uint8_t data)
{
	UART_BufferTX.buffer[UART_BufferTX.head_pointer++] = data;
	if(UART_BufferTX.head_pointer == BUFFER_SIZE)
	{
		UART_BufferTX.head_pointer = 0;
	}
  /* Enable the UART Transmit Data Register Empty Interrupt */
 SET_BIT(USART2->CR1, USART_CR1_TXEIE);
}

/**
  * @brief 	Bu fonksiyon islem yapilacak verinin olup olmadigini bildirir. Tail göstergesi Head göstergesine yetismisse islenmemis hazir veri yok demektir
  * @param 	Sorgulanmak istenilen Buffer giris parametresi verilir
  * @retval Sorgulanan buffer bos ise 1 bos degilse 0 degerini döndürür
	* @note 	yok
  */
int8_t UART_is_buffer_empty(volatile UART_Buffer_t* buffer)
{
	return (buffer->head_pointer == buffer->tail_pointer?1:0);
/*	
	if(buffer->head_pointer == buffer->tail_pointer)
	{
		return 1; // buffer is empty
	}
	else
	{
		 return 0;
	}
*/
}

/**
  * @brief 	Bu fonksiyon bufferda depolanan veriyi döndürür. 
  * @param 	yok
  * @retval Buffer'da deoplanan isaretsiz 8 bitlik veri, Buffer Bos ise 0 döndürülür
	* @note 	Veri isleme sokuldugu için Tail göstergesi arttirilir.
  */
uint8_t UART_read_byte(void)
{
	uint8_t character =  0; 
	
	if(UART_is_buffer_empty(&UART_BufferRX) == 1 )
	{
		character = 0;
	}
	else
	{
		character = UART_BufferRX.buffer[UART_BufferRX.tail_pointer++];
		
		if ( UART_BufferRX.tail_pointer == BUFFER_SIZE)
		{
			UART_BufferRX.tail_pointer = 0;
		}
	}	
	
	return character;	
}

/**
  * @brief 	Bu fonksiyon girilen dizi verisini ring buffer'ina alarak UART Transmit Kesme fonksiyonunu baslatir
	*					Transmit kesmesi ile veri aktarimi saglanir.
  * @param 	isaretsiz 8 bitlik formatta gönderilmek istenilen dizi verisi giris parametresidir
  * @param 	isaretsiz 32 bitlik formatta gönderilmek istenilen dizi verisinin uzunlugu giris parametresidir
	* @retval yok
	* @note 	yok
  */
void UART_send_byte_array(uint8_t* buffer, uint32_t size)
{
	int i;
	
	for(i=0;i<size;i++)
	{
		UART_send_byte(buffer[i]);
	}
}

/**
  * @brief 	Bu fonksiyon islem yapilmamis verinin olup olmadigini bildirir
  * @param 	yok
	* @retval Isleme sokulmamis veri varsa 0'dan büyük sayi döndürür, bütünün veriler islenmis ise 0
	* @note 	yok
  */
int UART_is_data_available(void)
{
	return UART_BufferRX.head_pointer - UART_BufferRX.tail_pointer;
}



/**
  * @brief 	Bu fonksiyon dizi degiskenin karakter uzunlugunu verir
  * @param 	Karakter uzunlugu bulunacak isaretsiz sekiz bit degerindeki dizi degisken 
  * @retval Girilen dizi degiskenin karakter uzunlugu
	* @note 	yok
  */
uint16_t StringLength(uint8_t *string)
{
	uint16_t stringLength=0;
	while(*(string+stringLength) != '\0')
	{
		stringLength++;
	}
	return stringLength;
}

/**
  * @brief 	Bu fonksiyon gelen verilerde aranacak bir dizinin olup olmadigini bildirir
  * @param 	Aranacak dizi degiskeni giris parametresi olarak verilir
  * @retval gelen veri içinde aranan dizi varsa 1 degerini döndürür 
	* @note 	yok
  */
uint8_t find_string_from_buffer(uint8_t *string)
{
	uint8_t stringMach;
	uint8_t stringLen=StringLength(string);

	while(*string != '\0')
	{
		while(UART_is_data_available())
		{
			if(*string == UART_read_byte() && *string != '\0')
			{
				stringMach++;
				string++;
			}
		}
	}
	
	if(stringMach == stringLen)
	{
			stringMach=1;
      return stringMach;
	}
	return stringMach;
}
