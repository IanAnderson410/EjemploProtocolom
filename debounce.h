
/*! \mainpage Debounce para los botones
 * \date 10/09/2023
 * \author Nombre
 * \section genDesc Descripcion general
 * El archivo contiene los elementos para realizar el filtrado de los botones
 *
 * \section desarrollos Observaciones generales
 * [Complete aqui con sus observaciones]
 *
 * \section changelog Registro de cambios
 *
 * |   Fecha    | Descripcion                                    |
 * |:----------:|:-----------------------------------------------|
 * | 10/09/2023 | Creacion del documento                         |
 *
 */

#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "myDelay.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/
/**
 * @brief DEfine los estados de la MEF del botón
 *        
 */
typedef enum{
    BUTTON_DOWN,
    BUTTON_UP,
    BUTTON_RISING,
    BUTTON_FALLING
}_eState;

/**
 * @brief DEfine el estado del botón. Pensado para configuración PULL_UP
 * 
 */
typedef enum{
    PRESSED,
    NOT_PRESSED,
    NO_EVENT
}_eEventInput;

/**
 * @brief Banderas para detectar los flancos con el Botón
 * 
 */
typedef enum{
    FALLINGFLAG,
    RISINGFLAG,
    NOFLAG,
}_eBtnFlag;

/**
 * @brief Estructura para el manejo de los botones
 * 
 */
typedef struct
{
    _eState         currentState;   //!< Estado del boton en la MEF de debounce
    _eEventInput    stateInput;     //!< Valor de la entrada digital del micro
    _eBtnFlag       flagDetected;   //!< Flag para detección de flanco 
    int32_t         timePressed;    //!< Toma el instante en que se presionó el botón 
    int32_t         timeDiff;       //!< Toma la diferencia entre el momento en que se presionó y el que se soltó
    uint8_t         mask;           //!< Mascara para filtrar el valor del BusIn
 }_sButton;

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/
 
/**
 * @brief Inicializa los botones
 * 
 * @param myButton Estructura de los botones
 * @param numButton Cantidad de botones
 */
void startButon(_sButton *myButton, uint8_t numButton);

/**
 * @brief MAquina de Estado de los botones
 * 
 * @param timedebounce Estructura de datos para el debounce
 * @param myButton Estruvtura de datos de los botones
 * @param statePulsadores Estado del array de botones
 */
void buttonTask(_delay_t *timedebounce, _sButton *myButton, uint8_t statePulsadores);
/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

/* END Global variables ------------------------------------------------------*/

#endif