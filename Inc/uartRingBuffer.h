/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name:																																																														*	
*																																																																						*
*		File Name: 																																																															*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification Dates: 																																																										*
*																																																																						*	
*		Description:																																																														*
*																																																																						*
*																																																																						*	
*																																																																						*
* 																																																																					*
*		Notes:																																																																	*
*																																																																						*
*	 [1]																																																																			*
*	 																																																																					*
*	 [2]																																																										    							*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __uartRingBuffer
#define __uartRingBuffer

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void MX_USART2_UART_Init(void);
void UART_send_byte(uint8_t data);
uint8_t UART_read_byte(void);
void UART_send_byte_array(uint8_t* buffer, uint32_t size);
int UART_is_data_available(void);
uint16_t StringLength(uint8_t *string);
uint8_t find_string_from_buffer(uint8_t *string);

/* Private defines -----------------------------------------------------------*/
#define BUFFER_SIZE 6
#define BAUDRATE 9600

#endif
