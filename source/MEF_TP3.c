/*
 * MEF_TP3.c
 *
 *  Created on: 31 may. 2022
 *      Author: Lucas
 */
#include "MEF_TP3.h"
#include "MEF_Analizadora.h"
#include "MEF_Actuadora.h"


void MEF_TP3_init(void *pRingBuffer_Acciones)
{
	MEF_Analizadora_init(pRingBuffer_Acciones);
	MEF_Actuadora_init(pRingBuffer_Acciones);
}

void MEF_TP3_run()
{
	MEF_Analizadora_run();

	MEF_Actuadora_run();
}
