/*
 * MEF_Analizadora.h
 *
 *  Created on: 31 may. 2022
 *      Author: Lucas
 */

#ifndef MEF_ANALIZADORA_H_
#define MEF_ANALIZADORA_H_

#include "Identificadores.h"
#include "stdint.h"
#include "ringBuffer.h"

typedef enum
{
	INICIO = 0, GRUPOH, GRUPOL, ACCIONH, ACCIONL, ACCIONLED, FIN
}Estados_MEF_Analizadora;

typedef struct
{
	uint8_t objetivo;
	uint8_t Id_Objetivo;
	uint8_t accion;
}Paquete_de_Acciones;

void MEF_Analizadora_init(void *pRbAcciones);
void MEF_Analizadora_run(void);



#endif /* MEF_ANALIZADORA_H_ */
