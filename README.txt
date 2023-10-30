CARPETA QT : "EjemploProtocolo" 
Copiar en la pc, abrir Qt y abrir como proyecto.
CAPETA MBED: "MBED"
Los archivos dentro de esta capeta se deben copiar dentro
de la carpeta de proyecto base configurada en su Pc. 
El "main.cpp" original debe reemplazarse por este.

Cambiar en el archivo "config.h" en la línea: 

28 const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"FCAL\",\"fcalconcordia.06-2019\"\r\n";

FCAL (Cambiarlo por el nombre(SSID) de la red wifi donde se quieren conectar)

fcalconcordia.06-2019 (Por el PASS de la red wifi donde se quieren conectar)

en la linea :

35 const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"192.168.1.14\",30010,30001,0\r\n";

192.168.1.14 (cambiar por la IP de la máquina a donde quieren que se conecte el ESP-01)
