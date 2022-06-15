/*
 * MEF_Actuadora.h
 *
 *  Created on: 1 jun. 2022
 *      Author: Lucas
 */

#ifndef MEF_ACTUADORA_H_
#define MEF_ACTUADORA_H_

#include "stdint.h"

typedef enum
{
	TRAMA_INIT = 0, TRAMA_RESP, ACCION, ACCION_LEDS, ACCION_SW, ACCION_ACC, RESET_TRAMA
}Estados_MEF_Actuadora;

void MEF_Actuadora_init(void *pRbAcciones);
void MEF_Actuadora_run(void);

#endif /* MEF_ACTUADORA_H_ */
