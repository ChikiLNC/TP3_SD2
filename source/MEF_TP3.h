/*
 * MEF_TP3.h
 *
 *  Created on: 31 may. 2022
 *      Author: Lucas
 */

#ifndef MEF_TP3_H_
#define MEF_TP3_H_

#include "ringBuffer.h"
#include "stdbool.h"

void MEF_TP3_init(void *pRingBufferAcciones);
void MEF_TP3_run(void);
void analizarDatos(void *pRb);

#endif /* MEF_TP3_H_ */
