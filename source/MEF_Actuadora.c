/*
 * MEF_Actuadora.c
 *
 *  Created on: 1 jun. 2022
 *      Author: Lucas
 */

#include "MEF_Actuadora.h"
#include "ringBuffer.h"
#include "Identificadores.h"
#include "SD2_board.h"
#include "uart_ringBuffer.h"
#include "mma8451.h"
#include "math.h"
#include "stdio.h"

#define SIZE_BUFFER 9

static void *pRb_Acciones;
static uint8_t trama[SIZE_BUFFER];
static int16_t acc = 0;

void MEF_Actuadora_init(void *pRbAcciones)
{
	pRb_Acciones = pRbAcciones;
}

void MEF_Actuadora_run(void)
{
	uint8_t datoTemp;
	static Estados_MEF_Actuadora estadoact = TRAMA_INIT;


	switch(estadoact)
	{
		case TRAMA_INIT:

			trama[0] = INICIO_TRAMA;
			trama[1] = HS_GRUPO;
			trama[2] = LS_GRUPO;
			estadoact = TRAMA_RESP;
			break;

		case TRAMA_RESP:
			if(!ringBuffer_isEmpty(pRb_Acciones))
			{
				ringBuffer_getData(pRb_Acciones, &datoTemp);	//Objetivo
				trama[3] = datoTemp;
				ringBuffer_getData(pRb_Acciones, &datoTemp);	//ID
				trama[4] = datoTemp;
				ringBuffer_getData(pRb_Acciones, &datoTemp);	//Accion
				trama[5] = datoTemp;

				estadoact = ACCION;
			}

			break;
		case ACCION:
			estadoact = RESET_TRAMA;

			if(trama[3] == OBJ_LED)
				estadoact = ACCION_LEDS;

			if(trama[3] == OBJ_SW)
				estadoact  = ACCION_SW;

			if(trama[3] == OBJ_ACC)
				estadoact = ACCION_ACC;

			break;

		case ACCION_LEDS:
			if(trama[4] == LED_ROJO_ID)
			{
				if(trama[5] == ACT_ENCENDER)
					board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
				if(trama[5] == ACT_APAGAR)
					board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
				if(trama[5] == ACT_TOGGLE)
					board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_TOGGLE);
			}
			if(trama[4] == LED_VERDE_ID)
			{
				if(trama[5] == ACT_ENCENDER)
					board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_ON);
				if(trama[5] == ACT_APAGAR)
					board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_OFF);
				if(trama[5] == ACT_TOGGLE)
					board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_TOGGLE);
			}

			trama[6] = FIN_TRAMA;
			uart_ringBuffer_envDatos(trama, sizeof(trama));

			estadoact = RESET_TRAMA;
			break;

		case ACCION_SW:
			if(trama[4] == SW1_ID)
			{
				if (board_getSw(BOARD_SW_ID_1))
				{
					trama[5] = EST_PRESIONADO;
				}
				else
				{
					trama[5] = EST_NOPRESIONADO;
				}
			}
			if(trama[4] == SW3_ID)
			{
				if (board_getSw(BOARD_SW_ID_3))
				{
					trama[5] = EST_PRESIONADO;
				}
				else
				{
					trama[5] = EST_NOPRESIONADO;
				}
			}

			trama[6] = FIN_TRAMA;
			uart_ringBuffer_envDatos(trama, sizeof(trama));
			estadoact = RESET_TRAMA;
			break;

		case ACCION_ACC:

			if(trama[4] == ACC_ID)
			{
				if(mma8451_isNewData())
				{
					acc = sqrt(pow(mma8451_getAcX(),2) + pow(mma8451_getAcY(),2) + pow(mma8451_getAcZ(),2));

					trama[5] = (int8_t) (acc/100);
					trama[6] = (int8_t) ((acc - 100*trama[5])/10);
					trama[7] = (int8_t) (acc - 10*trama[6] - 100*trama[5]);

					//El +=48 es para pasar de entero a ASCII

					trama[5] += 48;
					trama[6] += 48;
					trama[7] += 48;

					trama[8] = FIN_TRAMA;

					uart_ringBuffer_envDatos(trama, sizeof(trama));
					estadoact = RESET_TRAMA;
				}
			}

			break;

		case RESET_TRAMA:
			for(int8_t i = 0; i <= SIZE_BUFFER; i++)
			{
				trama[i] = 0;
			}
			estadoact = TRAMA_INIT;

			break;

	}

}



