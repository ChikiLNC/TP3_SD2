/*
 * MEF_Analizadora.c
 *
 *  Created on: 31 may. 2022
 *      Author: Lucas
 */
#include "MEF_Analizadora.h"
#include "MKL46Z4.h"
#include "uart_ringBuffer.h"

#define SIZE_BUFFER_RX 18

Paquete_de_Acciones paquete;
static void *pRb_Acciones;
static uint8_t buffer[SIZE_BUFFER_RX];

void MEF_Analizadora_init(void *pRbAcciones)
{
	pRb_Acciones = pRbAcciones;
}

void MEF_Analizadora_run(void)
{
	static Estados_MEF_Analizadora estado_analizadora = INICIO;

	switch(estado_analizadora)
	{
		case INICIO:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{
				if(buffer[0] == INICIO_TRAMA)
					estado_analizadora = GRUPOH;
			}

			break;

		case GRUPOH:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{
				if(buffer[0] == HS_GRUPO)
					estado_analizadora = GRUPOL;
				else
					estado_analizadora = INICIO;
			}
			break;

		case GRUPOL:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{
				if(buffer[0] == LS_GRUPO)
					estado_analizadora = ACCIONH;
				else
					estado_analizadora = INICIO;
			}
			break;

		case ACCIONH:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{
				if(buffer[0] == OBJ_LED || buffer[0] == OBJ_SW || buffer[0] == OBJ_ACC)
				{
					paquete.objetivo = buffer[0];
					estado_analizadora = ACCIONL;
				}
				else
				{
					estado_analizadora = INICIO;
				}
			}
			break;

		case ACCIONL:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{

				switch(paquete.objetivo)
				{
					case OBJ_LED:
						if(buffer[0] == LED_ROJO_ID || buffer[0] == LED_VERDE_ID)
						{
							paquete.Id_Objetivo = buffer[0];
							estado_analizadora = ACCIONLED;
						}
						else
						{
							estado_analizadora = INICIO;
						}
						break;

					case OBJ_SW:
						if(buffer[0] == SW1_ID || buffer[0] == SW3_ID)
						{
							paquete.Id_Objetivo = buffer[0];
							paquete.accion = ACT_LECTURA;
							estado_analizadora = FIN;
						}
						else
						{
							estado_analizadora = INICIO;
						}
						break;

					case OBJ_ACC:
						if(buffer[0] == ACC_ID)
						{
							paquete.Id_Objetivo = buffer[0];
							paquete.accion = ACT_LECTURA;
							estado_analizadora = FIN;
						}
						else
						{
							estado_analizadora = INICIO;
						}
						break;
				}
			}
			break;

		case ACCIONLED:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{
				if(buffer[0] == ACT_APAGAR || buffer[0] == ACT_ENCENDER || buffer[0] == ACT_TOGGLE)
				{
					paquete.accion = buffer[0];
					estado_analizadora = FIN;
				}
				else
				{
					estado_analizadora = INICIO;
				}
			}
				break;

		case FIN:
			if(uart_ringBuffer_recDatos(buffer, sizeof(buffer)))
			{
				estado_analizadora = INICIO;
				if(buffer[0] == FIN_TRAMA)
				{
					ringBuffer_putData(pRb_Acciones, paquete.objetivo);
					ringBuffer_putData(pRb_Acciones, paquete.Id_Objetivo);
					ringBuffer_putData(pRb_Acciones, paquete.accion);
				}
			}
			break;

	}
}
