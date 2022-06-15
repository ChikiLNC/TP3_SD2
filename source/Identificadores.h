/*
 * Identificadores.h
 *
 *  Created on: 31 may. 2022
 *      Author: Lucas
 */

#ifndef IDENTIFICADORES_H_
#define IDENTIFICADORES_H_

#define INICIO_TRAMA	0x3A 	//	Hex de :
#define FIN_TRAMA 		0x0A	//	Hex de LF
#define HS_GRUPO		48		//	ASCII 0
#define LS_GRUPO		57		//	ASCII 9

//Posibles objetivos a modificar/leer
#define OBJ_LED			48		//ASCII 0
#define OBJ_SW			49		//ASCII 1
#define OBJ_ACC			50		//ASCII 2

#define LED_ROJO_ID		49
#define LED_VERDE_ID	50
#define SW1_ID			49
#define SW3_ID			51		//ASCII 3
#define ACC_ID			48

#define ACT_ENCENDER 		69		//ASCII E
#define ACT_APAGAR 			65		//ASCII A
#define	ACT_TOGGLE  		84		//ASCII T
#define ACT_LECTURA  		76		//ASCII L
#define EST_NOPRESIONADO	78		//ASCII
#define EST_PRESIONADO		80



#endif /* IDENTIFICADORES_H_ */
