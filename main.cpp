/*! \mainpage Ejercicio Titulo
 * \date 10/09/2023
 * \author Alejandro Rougier
 * \section Ejemplo comunicación USART
 * [Complete aqui con su descripcion]
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



/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "util.h"
#include "myDelay.h"
#include "debounce.h"
#include "config.h"
#include "wifi.h"

#include <stdint.h>
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/

/**
 *  @brief _sHeartBeat
 * estructura usada para el manejo del heartbeat.
*/
typedef struct{
    bool commingToPlay=false;
    uint16_t periodo;   /*!< El periodo que tarda en completar un ciclo completo el heartbeat segun su modo*/
    uint32_t timehb; 
}_sHeartBeat;
/**
 * @brief _eStatesOfButton is a enum for the four states in the button
*/
typedef enum{
    DOWN=0,
    RISSING,
    UP,
    FALL,
}_eStatesOfButton;
/**
 * @brief _sSw struct for the using of the button  
 *  */
typedef struct{
    uint8_t previousMeasurement=DOWN;
    uint8_t currentMeasurement=DOWN;  /*!< Estados posibles: presionado, no presionado*/
    uint32_t timePressed=0; /*!< storage the time while pressed the button */
    uint8_t state_b= DOWN;
}_sSW;


typedef enum{        
    MODE_IDDLE,
    MODE_1,         //Seguidor de linea MODO WAITING
    MODE_1_PLAY,    //Seguidor de linea MODO PLAY
    MODE_2,         //Esquivar obstaculo MODO WAITING
    MODE_2_PLAY,    //Esquivar obstaculo MODO PLAY
    MODE_3,         //Mantener distancia MODO WAITING
    MODE_3_PLAY,    //Mantener distancia MODO PLAY
}_eMode;
/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
#define RXBUFSIZE  256
#define TXBUFSIZE  256
#define DEBOUNCE    20
#define HEARBEATIME 100
#define GENERALTIME 10
#define NUMBUTTONS  4
#define DISTANCEINTERVAL    300

#define     FORWARD             2   // adelante
#define     BACKWARD            1   //atras
#define     STOP                0

#define     SERIE               0
#define     WIFI                1


#define RESETFLAGS      flags.bytes 
#define ISCOMAND        flags.bits.bit0
#define SERVOMOVING     flags.bits.bit1
#define SERVODIRECT     flags.bits.bit2
#define MEDIRDISTANCIA  flags.bits.bit3

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/

DigitalOut LED_HeartBit(PC_13); // 1 apagado, 0 prendido
DigitalIn SW0(PA_4);
RawSerial PC(PA_9,PA_10);//!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT
DigitalOut trigger(PB_13);//!< Salida para el trigger del sensor Ultrasonico 
InterruptIn hecho(PB_12); //!<pin de eco del sensor Ultrasonico definido como interrupción 
PwmOut  servo(PA_8);//!< Pin del Servo debe ser PWM para poder modular el ancho del pulso
AnalogIn irLeft(PA_0); //!<Sensor infrarrojo para detección de linea 
AnalogIn irCenter(PA_1);//!<Sensor infrarrojo para detección de linea 
AnalogIn irRight(PA_2);//!<Sensor infrarrojo para detección de linea 

InterruptIn speedLeft(PB_9);//!<Sensor de Horquilla para medir velocidad
InterruptIn speedRight(PB_8);//!<Sensor de Horquilla para medir velocidad

//Hardware puente H y motores 
/*
BusOut  leftIN(PB_14,PB_15);//!< Pines para determinara la dirección de giro del motor
BusOut  rightIN(PB_6,PB_7);//!< Pines para determinara la dirección de giro del motor
PwmOut  left_ENA(PB_1);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo
PwmOut  right_ENB(PB_0);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo
*/

BusOut  dirMLeft(PB_14,PB_15);//!< Pines para determinara la dirección de giro del motor

BusOut  dirMRight(PB_6,PB_7);//!< Pines para determinara la dirección de giro del motor

PwmOut  speedMLeft(PB_1);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo

PwmOut  speedMRight(PB_0);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo


/*AnalogIn ENA(PB_1);
AnalogIn ENB(PB_0);
DigitalIn IN_1(PB_15);
DigitalIn IN_2(PB_14);
DigitalIn IN_3(PB_7);
DigitalIn IN_4(PB_6);*/
/* END hardware configuration ------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief Hearbeat, indica el funcionamiento del sistema
 * 
 * @param timeHearbeat Variable para el intervalo de tiempo
 * @param mask Secuencia de encendido/apagado del led de Hearbeat
 */
void hearbeatTask(_delay_t *timeHearbeat, uint16_t mask);
/**
 * @brief Ejecuta las tareas del puerto serie Decodificación/trasnmisión
 * 
 * @param dataRx Estructura de datos de la recepción
 * @param dataTx Estructura de datos de la trasnmisión
 * @param source Identifica la fuente desde donde se enviaron los datos
 */
void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source);
/**
 * @brief Rutina oara medir distancia
 * 
 * @param medicionTime variable que contiene el intervalo de medición de distancia
 * @param triggerTime Variable que contiene el valor de tiempo del Trigger
 */
void distanceTask(_delay_t *medicionTime, int32_t *triggerTime);
/**
 * @brief Rutina para realizar la verificación de mocimiento del servo
 * en caso de que no se mueva envia la respuesta automática a la PC
 * 
 * @param servoTime almacena el tiempo del timer
 * @param intervalServo posee el tiempo del ointervalo para responder en función del ángulo a mover
 */
void servoTask(_delay_t *servoTime, uint32_t *intervalServo);
/**
 * @brief Rutina para medir la velocidad
 * 
 */
void speedTask();
/**
 * @brief Rutina para hacer la medición de los sensores IR
 * 
 */
void irSensorsTask();
/**
 * @brief Recepción de datos por el puerto serie
 * 
 */
void onRxData();
/**
 * @brief Pone el encabezado del protocolo, el ID y la cantidad de bytes a enviar
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param ID Identificación del comando que se envía
 * @param frameLength Longitud de la trama del comando
 * @return uint8_t devuelve el Checksum de los datos agregados al buffer de trasnmisión
 */
uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);
/**
 * @brief Agrega un byte al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param byte El elemento que se quiere agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putByteOnTx(_sTx    *dataTx, uint8_t byte);
/**
 * @brief Agrega un String al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param str String a agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putStrOntx(_sTx *dataTx, const char *str);

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);
/**
 * @brief Decodifica la trama recibida
 * @param dataRx Estructura para la recepción de datos
 */
void decodeHeader(_sRx *dataRx);
/**
 * @brief Decodifica el comando recibido en la transmisión y ejecuita las tareas asociadas a dicho comando
 * @param dataRx Estructura para la recepción de datos
 * @param dataTx Estructura para la trasnmisión de datos
 */
void decodeCommand(_sRx *dataRx, _sTx *dataTx);
/**
 * @brief Función del Sensor de Horquilla Izquierdo
 * cuenta los pulsos del sensor para medir velocidad luego
 */
void speedCountLeft(void);
/**
 * @brief Función del Sensor de Horquilla Derecho
 * cuenta los pulsos del sensor para medir velocidad luego
 */
void speedCountRight(void);
/**
 * @brief toma el valor inicial del Timer para medir distancia
 * una vez que se detecta el evento RISE del pin de eco
 */
void distanceInitMeasurement(void);
/**
 * @brief Completa la medición una vez que se recibió el pulso de regreso
 * en el pin de eco
 */
void distanceMeasurement(void);
/**
 * @brief Función para realizar la autoconexión de los datos de Wifi
 */
void autoConnectWifi();
/**
 * @brief envía de manera automática el alive
 */
void aliveAutoTask(_delay_t *aliveAutoTime);




/**
 * @brief HeartBit
 * Verifica que el programa se ejecute correctamente ejecutando un patron de encendido
 * y apagado con el LED del puerto PC_13, este patrón depende del modo en el que se 
 * encuentre.
*/
void HeartBit();
/**
 * @brief buttonTask
 * Maquina de Estado de los botones
 */
void ButtonTask();
void initMeasurement();
void finalMeasurement();
void do100ms();
void doTimeout();
/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

Timer   myTimer;
Timer   myTimerTrigger;  
Ticker  timerGral; //El ticker es periodico, cada tanto hace una interrupcion
Timeout triggerTimer; // El timeout es tipo one shot

_sHeartBeat hb; 
_sSW        button;
_uFlag      flags;
_sSensor    irSensor[3];
_sServo     miServo;
_uWord      myWord;
_delay_t    generalTime;

uint8_t     currentMode = MODE_IDDLE;

const char  firmware[] = "EX100923v01\n";

volatile    _sRx dataRx;
            _sTx dataTx; //not volatile

volatile    uint8_t buffRx[RXBUFSIZE];
            uint8_t buffTx[TXBUFSIZE]; //not volatile

uint8_t  globalIndex, index2;
uint32_t distancia;
uint32_t tiempoTotal;
uint16_t do100;

volatile uint32_t countLeftValue, countRightValue;

volatile int32_t  initialValue, finalValue, distanceValue;

uint32_t speedleftValue, speedRightValue;

wifiData myWifiData;

_sButton myButton[NUMBUTTONS];

_sTx wifiTx;

_sRx wifiRx;

uint8_t wifiBuffRx[RXBUFSIZE];

uint8_t wifiBuffTx[TXBUFSIZE];



int32_t motorSpeed, auxSpeed;


/* END Global variables ------------------------------------------------------*/


/* Function prototypes user code ----------------------------------------------*/
/**
 * @brief Instanciación de la clase Wifi, le paso como parametros el buffer de recepción, el indice de 
 * escritura para el buffer de recepción y el tamaño del buffer de recepción
 */
Wifi myWifi(wifiBuffRx, &wifiRx.indexW, RXBUFSIZE);








void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source){
    if(dataRx->isComannd){
        dataRx->isComannd=false;
        decodeCommand(dataRx,dataTx);
    }
    if(delayRead(&generalTime)){
        if(dataRx->header){
            dataRx->timeOut--;
        if(!dataRx->timeOut)
            dataRx->header = HEADER_U;
        }
    }
    if(dataRx->indexR!=dataRx->indexW){
        decodeHeader(dataRx);
       /* CODIGO A EFECTOS DE EVALUAR SI FUNCIONA LA RECEPCIÓN , SE DEBE DESCOMENTAR 
       Y COMENTAR LA LINEA decodeHeader(dataRx); 
       while (dataRx->indexR!=dataRx->indexW){
            dataTx->buff[dataTx->indexW++]=dataRx->buff[dataRx->indexR++];
            dataTx->indexW &= dataTx->mask;
            dataRx->indexR &= dataRx->mask;
        } */
    }
    if(dataTx->indexR!=dataTx->indexW){
        if(source){
             myWifi.writeWifiData(&dataTx->buff[dataTx->indexR++],1); 
             dataTx->indexR &=dataTx->mask; 
        }else{
            if(PC.writeable()){
                PC.putc(dataTx->buff[dataTx->indexR++]);
                dataTx->indexR &=dataTx->mask;
            }
        }
    }
}
void onRxData(){
    while(PC.readable()){
        dataRx.buff[dataRx.indexW++]=PC.getc();
        dataRx.indexW &= dataRx.mask;
    }
}
uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength){
    dataTx->chk = 0;
    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength+1;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= (frameLength+1);
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
    return  dataTx->chk;
}
uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte){
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}
uint8_t putStrOntx(_sTx *dataTx, const char *str){
    globalIndex=0;
    while(str[globalIndex]){
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk ;
}
uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos){
    uint8_t getByte;
    dataRx->indexData += iniPos;
    dataRx->indexData &=dataRx->mask;
    getByte = dataRx->buff[dataRx->indexData];
    dataRx->indexData += finalPos;
    dataRx->indexData &=dataRx->mask;
    return getByte;
}
void decodeHeader(_sRx *dataRx){
    uint8_t auxIndex=dataRx->indexW;
    while(dataRx->indexR != auxIndex){
        switch(dataRx->header)
        {
            case HEADER_U:
                if(dataRx->buff[dataRx->indexR] == 'U'){
                    dataRx->header = HEADER_N;
                    dataRx->timeOut = 5;
                }
            break;
            case HEADER_N:
                if(dataRx->buff[dataRx->indexR] == 'N'){
                    dataRx->header = HEADER_E;
                }else{
                    if(dataRx->buff[dataRx->indexR] != 'U'){
                        dataRx->header = HEADER_U;
                        dataRx->indexR--;
                    }
                }
            break;
            case HEADER_E:
                if(dataRx->buff[dataRx->indexR] == 'E'){
                    dataRx->header = HEADER_R;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case HEADER_R:
                if(dataRx->buff[dataRx->indexR] == 'R'){
                    dataRx->header = NBYTES;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case NBYTES:
                dataRx->nBytes=dataRx->buff[dataRx->indexR];
                dataRx->header = TOKEN;
            break;
            case TOKEN:
                if(dataRx->buff[dataRx->indexR] == ':'){
                    dataRx->header = PAYLOAD;
                    dataRx->indexData = dataRx->indexR+1;
                    dataRx->indexData &= dataRx->mask;
                    dataRx->chk = 0;
                    dataRx->chk ^= ('U' ^'N' ^'E' ^'R' ^dataRx->nBytes ^':') ;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case PAYLOAD:
                dataRx->nBytes--;
                if(dataRx->nBytes>0){
                   dataRx->chk ^= dataRx->buff[dataRx->indexR];
                }else{
                    dataRx->header = HEADER_U;
                    if(dataRx->buff[dataRx->indexR] == dataRx->chk)
                        dataRx->isComannd = true;
                }
            break;
            default:
                dataRx->header = HEADER_U;
            break;
        }
        dataRx->indexR++;
        dataRx->indexR &= dataRx->mask;
    }
}
void decodeCommand(_sRx *dataRx, _sTx *dataTx){
    //int32_t motorSpeed; 
    //int32_t auxSpeed;
    int8_t angleSource;
    uint32_t servoPrevio=miServo.currentValue;
    /*
    ALIVE = 0xF0,
    FIRMWARE= 0xF1,
    LEDSTATUS = 0x10,
    BUTTONSTATUS = 0x12,
    ANALOGSENSORS = 0xA0,
    SETBLACKCOLOR = 0xA6,
    SETWHITECOLOR = 0xA7,
    MOTORTEST = 0xA1,
    SERVOANGLE = 0xA2,
    CONFIGSERVO = 0xA5,
    SERVOFINISHMOVE = 0x0A,
    GETDISTANCE = 0xA3,
    GETSPEED = 0xA4,
    ACK = 0x0D,
    UNKNOWN = 0xFF
    */
    switch(dataRx->buff[dataRx->indexData]){
        case SETBLACKCOLOR:
            //putHeaderOnTx(dataTx, SETBLACKCOLOR, 2);
            //putByteOnTx(dataTx, setBlack);
            //putByteOnTx(dataTx, dataTx->chk);            
            break;
        case SETWHITECOLOR:
            //putHeaderOnTx(dataTx, SETWHITECOLOR, 2);
            //putByteOnTx(dataTx, setWhite);
            //putByteOnTx(dataTx, dataTx->chk);            
            break;
        case ALIVE:
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
            break;
        case FIRMWARE:
            putHeaderOnTx(dataTx, FIRMWARE, 12);
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;            
        case ANALOGSENSORS:
            myWord.ui16[0] =  irSensor[0].currentValue;
            putHeaderOnTx(dataTx, ANALOGSENSORS, 7);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            myWord.ui16[0] =  irSensor[1].currentValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            myWord.ui16[0] =  irSensor[2].currentValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] ); 
            putByteOnTx(dataTx, dataTx->chk);       
            break;
        case SETMOTOR:
            putHeaderOnTx(dataTx, SETMOTOR, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);            
    
   //         motorSpeed = myWord.i32;            
            
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            motorSpeed = myWord.i32;
            if(motorSpeed>=0)
                dirMLeft.write(FORWARD);
            else
                dirMLeft.write(BACKWARD);
            auxSpeed=(abs(motorSpeed))*250;
            speedMLeft.pulsewidth_us(auxSpeed);
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            motorSpeed = myWord.i32;
            if(motorSpeed>=0)
                dirMRight.write(FORWARD);
            else
                dirMRight.write(BACKWARD);
            auxSpeed=(abs(motorSpeed))*250;
            speedMRight.pulsewidth_us (auxSpeed);

        break;
        case SERVOANGLE:
            putHeaderOnTx(dataTx, SERVOANGLE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
            SERVOMOVING=true;
            angleSource = getByteFromRx(dataRx,1,0);            
           // if (angleSource<-180 || angleSource>180){
           //     break;
           // }else{
                miServo.currentValue = (((angleSource - miServo.Y1) * (miServo.X2-miServo.X1))/(miServo.Y2-miServo.Y1))+miServo.X1;
                //if(miServo.currentValue > (uint16_t)miServo.X2)
                  //  miServo.currentValue=miServo.X2;
                if(miServo.currentValue < (uint16_t)miServo.X1)
                    miServo.currentValue=miServo.X1;
                servo.pulsewidth_us(miServo.currentValue);
            //}
            if(miServo.currentValue>servoPrevio){
                miServo.intervalValue=(miServo.currentValue-servoPrevio);
            }else{
                miServo.intervalValue=(servoPrevio-miServo.currentValue);
            }
            miServo.intervalValue=(miServo.intervalValue*1000) /(miServo.X2-miServo.X1);
            if(miServo.intervalValue>1000)
                miServo.intervalValue=1000;
            if(miServo.intervalValue<50)
                miServo.intervalValue=50;
        break;
        case CONFIGSERVO:
        break;
        case GETDISTANCE:
           myWord.ui32 = tiempoTotal;
            //valor += 100;
            putHeaderOnTx(dataTx, GETDISTANCE, 5);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            putByteOnTx(dataTx, dataTx->chk);  
        break;
        case GETSPEED:
            myWord.ui32 = speedleftValue;
            putHeaderOnTx(dataTx, GETSPEED, 9);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            myWord.ui32 = speedRightValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            putByteOnTx(dataTx, dataTx->chk);           
        break;        
        default:
            putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
            putByteOnTx(dataTx,UNKNOWN );
            putByteOnTx(dataTx, dataTx->chk);
            break;   

    }
}
void distanceTask(_delay_t *medicionTime, int32_t *triggerTime){
    if(delayRead(medicionTime)){
        MEDIRDISTANCIA=true; 
        trigger.write(false);
    }
        /******************** MEDICIÓN DE DISTANCIA NON-BLOCKING ********************/
    if (MEDIRDISTANCIA == true){
        trigger.write(true);

    }
}
void servoTask(_delay_t *servoTime, uint32_t*intervalServo){
     /******************** RESPUESTA AUTOMATICA DEL SERVO ********************/
    if(SERVOMOVING){
        if(delayRead(servoTime)){
            SERVOMOVING=false;
            putHeaderOnTx(&dataTx,SERVOANGLE,2);
            putByteOnTx(&dataTx,SERVOFINISHMOVE);
            putByteOnTx(&dataTx, dataTx.chk);
        }
    }
}
void speedTask(){
    static int32_t timeSpeed=0;
    #define INTERVAL 1000
    if ((myTimer.read_ms()-timeSpeed)>=INTERVAL){
            timeSpeed=myTimer.read_ms();       
            speedleftValue = countLeftValue;
            countLeftValue=0;
            speedRightValue=countRightValue;
            countRightValue=0;
    } 
}
void irSensorsTask(){
    static int32_t timeSensors=0;
    static uint8_t index=0;
    #define INTERVALO 30
    if ((myTimer.read_ms()-timeSensors)>=INTERVALO){
            timeSensors=myTimer.read_ms(); 
            switch(index){
                case 0:
                    irSensor[index].currentValue=irLeft.read_u16();
                break;
                case 1:
                    irSensor[index].currentValue=irCenter.read_u16();
                break;
                case 2:
                    irSensor[index].currentValue=irRight.read_u16();
                break;
                default:
                    index=255;
            }
            index++;
            index &=(-(index<=2));
    } 

}
void speedCountLeft(void){
    countLeftValue++;
}
void speedCountRight(void){
    countRightValue++;
}
/**********************************AUTO CONNECT WIF*********************/
void autoConnectWifi(){
    #ifdef AUTOCONNECTWIFI
        memcpy(&myWifiData.cwmode, dataCwmode, sizeof(myWifiData.cwmode));
        memcpy(&myWifiData.cwdhcp,dataCwdhcp, sizeof(myWifiData.cwdhcp) );
        memcpy(&myWifiData.cwjap,dataCwjap, sizeof(myWifiData.cwjap) );
        memcpy(&myWifiData.cipmux,dataCipmux, sizeof(myWifiData.cipmux) );
        memcpy(&myWifiData.cipstart,dataCipstart, sizeof(myWifiData.cipstart) );
        memcpy(&myWifiData.cipmode,dataCipmode, sizeof(myWifiData.cipmode) );
        memcpy(&myWifiData.cipsend,dataCipsend, sizeof(myWifiData.cipsend) );
        myWifi.configWifi(&myWifiData);
    #endif
}
void aliveAutoTask(_delay_t *aliveAutoTime){
    if(myWifi.isWifiReady()){
        if(delayRead(aliveAutoTime))
        {
            putHeaderOnTx(&wifiTx, ALIVE, 2);
            putByteOnTx(&wifiTx, ACK );
            putByteOnTx(&wifiTx, wifiTx.chk);
        }
    }
}
/**********************************AUTO CONNECT WIF*********************/



void HeartBit(){    
    LED_HeartBit = 1;
    hb.timehb++;
    if(hb.commingToPlay == false){  
        hb.periodo = 30;        
    }else{
        if(hb.commingToPlay == true)    hb.periodo = 10;            
        }
    switch(currentMode){
    case MODE_IDDLE:                
        if((hb.timehb % 3 ) == 1 )  LED_HeartBit = 0;            
        break;
    case MODE_1:         
        if(hb.timehb == 1 ) LED_HeartBit = 0;    
        break;    
    case MODE_2:            
        if(hb.timehb == 1  || hb.timehb == 4)   LED_HeartBit = 0;            
        break;
    case MODE_3:            
        if(hb.timehb == 1 || hb.timehb == 4  || hb.timehb == 8) LED_HeartBit = 0;        
        break;  
    case MODE_1_PLAY:
        hb.periodo = 30;        
        if(hb.timehb <= 12 || hb.timehb == 16 ) LED_HeartBit = 0;        
        break;
    case MODE_2_PLAY:
        hb.periodo = 30;        
        if(hb.timehb <= 12 || hb.timehb == 16 || hb.timehb == 19) LED_HeartBit = 0;        
        break;      
    case MODE_3_PLAY:
        hb.periodo = 30;        
        if(hb.timehb <= 12 || hb.timehb == 16 || hb.timehb == 19  || hb.timehb == 23) LED_HeartBit = 0;        
        break;
    }
    if(hb.timehb >= hb.periodo) hb.timehb = 0 ;
}
void ButtonTask(){    
        button.currentMeasurement = SW0.read();
        switch (button.state_b){
        case DOWN:            
            //================ [ CHOOSE THE MODE ] =========================            
            hb.commingToPlay = false;
            //if(button.timePressed >= 3 && button.timePressed <= 25 ){
            if(button.timePressed >= 25 && button.timePressed <= 75 ){    
                button.timePressed = 0;
                switch(currentMode){
                    case MODE_IDDLE:
                        currentMode=MODE_1;
                        break;
                    case MODE_1:
                        currentMode=MODE_2;
                        break;
                    case MODE_2:    
                        currentMode=MODE_3;
                        break;
                    case MODE_3:
                        currentMode=MODE_IDDLE;
                        break;
                }
                }else{
                //if(button.timePressed > 25 && button.timePressed <= 75 && currentMode!= MODE_IDDLE){ // entra cuando el periodo de tiempo va desde 1000 ms a 3000 ms                  
                if(button.timePressed > 75 && button.timePressed <=140  && currentMode!= MODE_IDDLE){ // entra cuando el periodo de tiempo va desde 1000 ms a 3000 ms                  
                    button.timePressed = 0;
                    currentMode ++ ;                      
                    }
                }
            button.timePressed = 0;
            //================ [ CHOOSE THE MODE ] =========================
            if(button.currentMeasurement == 1 )   button.state_b = RISSING;            
            break;                
        case RISSING:
            switch (button.currentMeasurement){
                case 0:
                    button.state_b = DOWN; 
                    break;
                case 1:
                    button.state_b = UP; 
                    break;                
                }                            
            break;
        case UP:            
            button.timePressed++;            
            if(button.timePressed > 25 ){
                hb.commingToPlay = true;                
                switch(currentMode){
                case MODE_1_PLAY:
                    currentMode = MODE_1;       
                    break;
                case MODE_2_PLAY:
                    currentMode = MODE_2;
                    break;
                case MODE_3_PLAY:
                    currentMode = MODE_3;
                    break;
            }
            }            
            if(button.currentMeasurement == 0)   button.state_b = FALL;
            break;
        case FALL:
            switch (button.currentMeasurement){
                case 0:
                    button.state_b = DOWN; 
                    break;
                case 1:
                    button.state_b = UP; 
                    break;                
                }            
            break;            
        }                
}
void initMeasurement(){
    //myTimer.start();
    myTimerTrigger.start();
}
void finalMeasurement(){
    tiempoTotal = myTimerTrigger.read_us(); // us microsegundos; ms milisegundos
    distancia = tiempoTotal/58;
    myTimerTrigger.stop();
}
void do100ms(){
    do100++;
    trigger.write(1); // El trigger es igual a 1
    triggerTimer.attach_us(&doTimeout, 10);
}
void doTimeout(){ // reseteamos 
    myTimerTrigger.reset(); // apagamos la pata del trigger
    trigger.write(0); // Escribimos en la salida un 0
}
/* END Function prototypes user code ------------------------------------------*/
int main(){
    dataRx.buff = (uint8_t *)buffRx;
    dataRx.indexR = 0;
    dataRx.indexW = 0;
    dataRx.header = HEADER_U;
    dataRx.mask = RXBUFSIZE - 1;

    dataTx.buff = buffTx;
    dataTx.indexR = 0;
    dataTx.indexW = 0;
    dataTx.mask = TXBUFSIZE -1;

    wifiRx.buff = wifiBuffRx;
    wifiRx.indexR = 0;
    wifiRx.indexW = 0;
    wifiRx.header = HEADER_U;
    wifiRx.mask = RXBUFSIZE - 1;

    wifiTx.buff = wifiBuffTx;
    wifiTx.indexR = 0;
    wifiTx.indexW = 0;
    wifiTx.mask = TXBUFSIZE -1;

    RESETFLAGS = 0;

/* Local variables -----------------------------------------------------------*/        
    _delay_t    medicionTime;
    _delay_t    servoTime;
    _delay_t    aliveAutoTime;
    //int32_t    triggerTime;    
    uint32_t timeHeartBit=0;
    uint32_t timeGral=0;    

   /* SERVO DEFUALT VALUES*/
    /*
    miServo.X2=2000;
    miServo.X1=1000;
    miServo.Y2=90;
    miServo.Y1=-90;
    */
   /* SERVO IAN VALUES >:) */ 
    miServo.X2=2500;
    miServo.X1=500;
    miServo.Y2=181;
    miServo.Y1=1;
    miServo.intervalValue=1000;
    /* FIN SERVO DEFAULT VALUES*/
/* END Local variables -------------------------------------------------------*/


/* User code -----------------------------------------------------------------*/
//    myTimerTrigger
    PC.baud(115200);
  //  speedMLeft.period_ms(25);
//    speedMRight.period_ms(25);
    servo.period_ms(20);
    trigger.write(false);
   /********** attach de interrupciones ********/
    PC.attach(&onRxData, SerialBase::IrqType::RxIrq);
    speedLeft.rise(&speedCountLeft);
    speedRight.rise(&speedCountRight);
   // timerGral.attach_us(&do100ms, 10000); // el attach esta en segundos,
    hecho.rise(&initMeasurement);
    hecho.fall(&finalMeasurement);
     /********** FIN - attach de interrupciones ********/
    
    servo.pulsewidth_us(1500);
    //speedMLeft.pulsewidth_us(0);
    //speedMRight.pulsewidth_us(0);    
    delayConfig(&generalTime,GENERALTIME);
    delayConfig(&medicionTime,DISTANCEINTERVAL);
    delayConfig(&servoTime,miServo.intervalValue);
    delayConfig(&aliveAutoTime, ALIVEAUTOINTERVAL);
    //delayConfig(&timeGlobal, GENERALTIME );    
    myWifi.initTask();
    autoConnectWifi();
    currentMode = MODE_IDDLE;
    myTimer.start();                        
    while(1){        
        if(myTimer.read_ms() - timeGral >= 40){
            timeGral = myTimer.read_ms();            
            ButtonTask();         

            do100++;
            trigger.write(1); // El trigger es igual a 1
            triggerTimer.attach_us(&doTimeout, 10);
            }
        if((myTimer.read_ms() - timeHeartBit) >= 100){  //100 ms 
            timeHeartBit = myTimer.read_ms();
            HeartBit();
            }
        myWifi.taskWifi();    
        serialTask((_sRx *)&dataRx,&dataTx, SERIE);
        irSensorsTask();
        servoTask(&servoTime,&miServo.intervalValue);
        aliveAutoTask(&aliveAutoTime);                            
    }
/* END User code -------------------------------------------------------------*/
}