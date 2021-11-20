  #pragma once
// October 13 2021

 


//CODIGOS DE ERROR
unsigned int mensajes = 0;

//Variables las maquinas de estado principales
unsigned int Edo_maq_ABRIR = 0;
unsigned int Edo_maq_CERRAR = 0;
unsigned int Edo_maq_HOME = 0;
boolean Secuencia_ABRIR = false;
boolean Secuencia_CERRAR = false;
boolean Secuencia_HOME = false;

//boolean Secuencia_cargar_auto = false;
//boolean hab_Secuencia_cargar_auto = false;


//Umbrales para permisivos
//unsigned int Temp_bateria_int;
unsigned int Temperatura_int = 0;
unsigned int Humedad_int = 0;
unsigned int vel_viento_int;
unsigned int Voltaje_bateria_int;


//Parametros condiciones ambientales
int Temperatura_OK = 35;
int Humedad_OK = 75;
int vel_viento_OK = 10; //10m/s
int Voltaje_bateria_drone_OK = 10; //10V equivale a un 50% de carga en la pila del mavic 2
int Temperatura_bateria_drone_MAX = 40; //16V equivale a un 95% de carga en la pila del mavic 2


//Variables condiciones ambientales
float Temperature;
float Humidity;
float vel_viento;

//Variables bateria
//float Temp_bateria_Celcius;
float Voltaje_bateria_drone;


//Parametros de tiempo configurables rutina de cierre
unsigned int tiempo_enfriado = 1800;  //1hr
unsigned int tiempo_carga = 10800;  //2hrs
unsigned int tiempo_video = 180;  //3 mins


// -------------  Definicion de los parametros Ethernet de default ------------------

byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x51, 0x06 };  // esta MAC address es la que va a tener el PLC. Puede ser cualquiera! pero es fija, no se puede cambiar
byte ip[] = { 192, 168, 1, 119 };     //IP del PLC estacion 1
byte gateway[] = { 192, 168, 0, 254 };
byte subnet[] = { 255, 255, 255, 0 };

//byte new_IP[] = { 192, 168, 0, 1 };     //IP desde KEPWARE
//byte new_MSK[] = { 255, 255, 255, 0 };     //IP_MSK desde KEPWARE
//byte new_gateway[] = { 192, 168, 0, 254 };     //IP_MSK desde KEPWARE




//Variables del DOMO
long max_posicion_Domo = 75000; //[centecimas de grado. 36000 representa 360 grados]
int posicion_Domo = 0;

//Variables del PAD
int posicion_PAD = 0;
int min_posicion_PAD = 0; //[centecimas de grado]
int max_posicion_PAD = 7000; //[centecimas de grado]
int vel_PAD = 10000; //[-2000 .. 2000] ~[-32 Amps - 32 Amps]

// bits de control/monitoreo general

boolean sistema_habilitado = false;
boolean Permisivo_despegue = false;
boolean C_Ambientales_OK = false;
//boolean Warning_Temp_bateria = false;
boolean Warning_Voltaje_bateria = false;
//boolean habilita_Warning_Temperatura_bateria = false; //no jala la temperatura
boolean habilita_Warning_Voltaje_bateria = true;

boolean  DRONE_ON = false;
boolean Ventilador_ON = false;
boolean Alarma_sistema = false;

//variables de  ciclos de operacion
//unsigned ciclos_prueba = 0;
unsigned int Ciclos = 0;  //Cuenta los ciclos de operacion del drone-puerto. Se usa int porque se escribe en EEPROM
//unsigned Max_ciclos_PRUEBA = 5;


// **************** variables de Entradas digitales **********************



//entradas  digitales virtuales creadas por software

boolean Voltaje_Bateria_bajo = false;
//boolean Temperatura_bateria_alta = false;
boolean Warning_bateria = false;

boolean Domo_Abierto = false;
boolean Domo_Cerrado = false;

boolean PAD_opened = false;
boolean PAD_closed = false;

boolean base_cerrada = false;
boolean base_abierta = false;
boolean home_base = false;

//Comandos
boolean PE = false;

boolean Auto_Manual = true;  //por deault esta en control automatico de secuencia
boolean Remoto_Local = true; //por default esta en modo Remoto, es decir; desde kepware via ethernet
boolean CHARGE_ON_Manual = false;
boolean reset_dronepuerto = false;


boolean Ventilador_ON_Manual = false;


boolean ABRIR_M = false;
boolean CERRAR_M = false;

boolean Energizar_PEINE_Manual = false;

//Comandos de mantenimiento
boolean actualizar_IP = false;
boolean actualizar_Parametros = false;

boolean PAD_open_M = false;
boolean PAD_close_M = false;

//boolean reconocer_alarmas = false;
//boolean borrar_alarmas = false;
//boolean borrar_todo_alarmas = false;


