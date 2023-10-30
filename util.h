/*! \mainpage Archivo para <gregar variables/definiciones/constantes etc a un programa>
 * \date 01/01/2023
 * \author Nombre
 * \section genDesc Descripcion general
 * [Complete aqui con su descripcion]
 *
 * \section desarrollos Observaciones generales
 * [Complete aqui con sus observaciones]
 *
 * \section changelog Registro de cambios
 *
 * |   Fecha    | Descripcion                                    |
 * |:----------:|:-----------------------------------------------|
 * |10/09/2023 | Creacion del documento                         |
 *
 */


#ifndef UTIL_H_
#define UTIL_H_

/* Includes ------------------------------------------------------------------*/
#include "myDelay.h"
#include <stdlib.h>
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/
/**
 * @brief Mapa de bits para declarar banderas
 * 
 */
typedef union{
    struct{
        uint8_t bit7 : 1;
        uint8_t bit6 : 1;
        uint8_t bit5 : 1;
        uint8_t bit4 : 1;
        uint8_t bit3 : 1;
        uint8_t bit2 : 1;
        uint8_t bit1 : 1;
        uint8_t bit0 : 1;
    }bits;
    uint8_t bytes;
}_uFlag;

/**
 * 
 * @brief Unión ara la descomposición/composición de números mayores a 1 byte
 * 
 */
typedef union{
    uint32_t    ui32;
    int32_t     i32;
    uint16_t    ui16[2];
    int16_t     i16[2];
    uint8_t     ui8[4];
    int8_t      i8[4];
}_uWord;
/**
 * @brief estructura para la recepción de datos por puerto serie
 * 
 */
typedef struct{
    uint8_t *buff;      /*!< Puntero para el buffer de recepción*/
    uint8_t indexR;     /*!< indice de lectura del buffer circular*/
    uint8_t indexW;     /*!< indice de escritura del buffer circular*/
    uint8_t indexData;  /*!< indice para identificar la posición del dato*/
    uint8_t mask;       /*!< máscara para controlar el tamaño del buffer*/
    uint8_t chk;        /*!< variable para calcular el checksum*/
    uint8_t nBytes;     /*!< variable para almacenar el número de bytes recibidos*/
    uint8_t header;     /*!< variable para mantener el estado dela MEF del protocolo*/
    uint8_t timeOut;    /*!< variable para resetear la MEF si no llegan más caracteres luego de cierto tiempo*/
    uint8_t isComannd;
}_sRx;
/**
 * @brief Estructura para la transmisión de datos por el puerto serie
 * 
 */
typedef struct{
    uint8_t *buff;      /*!< Puntero para el buffer de transmisión*/
    uint8_t indexR;     /*!<indice de lectura del buffer circular*/
    uint8_t indexW;     /*!<indice de escritura del buffer circular*/
    uint8_t mask;       /*!<máscara para controlar el tamaño del buffer*/
    uint8_t chk;        /*!< variable para calcular el checksum*/
}_sTx;

/**
 * @brief estructura para el manejo de sensores
 * 
 */
typedef struct{
    uint16_t    currentValue;
    uint16_t    maxValue;
    uint16_t    minValue;
    uint16_t    blackValue;
    uint16_t    whiteValue;
}_sSensor;

typedef struct{
    uint32_t    currentValue;
    uint32_t    intervalValue;
    int16_t    X2;
    int16_t    X1;
    int16_t    Y2;
    int16_t    Y1;
}_sServo;
/**
 * @brief Enumeración para la maquina de estados
 * que se encarga de decodificar el protocolo
 * de comunicación
 *  
 */
typedef enum{
    HEADER_U,
    HEADER_N,
    HEADER_E,
    HEADER_R,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eDecode;


/**
 * @brief Enumeración de los comandos del protocolo
 * 
 */
typedef enum{
    ALIVE = 0xF0,
    FIRMWARE= 0xF1,
    LEDSTATUS = 0x10,
    BUTTONSTATUS = 0x12,
    ANALOGSENSORS = 0xA0,
    SETBLACKCOLOR = 0xA6,
    SETWHITECOLOR = 0xA7,
    SETMOTOR = 0xA1,
    SERVOANGLE = 0xA2,
    CONFIGSERVO = 0xA5,
    SERVOFINISHMOVE = 0x0A,
    GETDISTANCE = 0xA3,
    GETSPEED = 0xA4,
    ACK = 0x0D,
    UNKNOWN = 0xFF
}_eCmd;


/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

/* END Global variables ------------------------------------------------------*/

#endif