/*

	 programa PLC Drone-puerto DD-21
	 Desarrollado por: Sergio Montufar DroneDomo Inc

	 

	 //19 Noviembre 2021  v 1.2
	 

	*/

	// Libraries

#include "DroneDomoTest.h" 


// Include Industrial Shields libraries
#include <RS485.h>
//#include <RS232.h>
//#include <Ethernet2.h>
#include <EEPROM.h>

//#include <Wire.h>
#include "AM2315.h"
#include "PLC_Timers.h" //definiciones de variables de los timers
//#include "Mudbus.h"
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <DS18B20.h>
#include <Servo.h>


#include "IO_serie_001.h"  //Mapeo E/S



//#define vel_PAD 16800 MAXIMO//[ centecimas de grado por rev]
#define vel_Domo_close 80000
#define vel_Domo_open 40000


#define min_posicion_Domo 0
#define max_torque_Domo 2000 //[-2000 .. 2000] ~ [-32 Amps - 32 Amps]

#define TOL1 1000  //equivale a 5 grados

#define TOL2 2000  //equivale a 10 grados

char PLC_version[] = "Drone-puerto DD-11   PLC program V1.2  -----  19 Noviembre 2021  -----  ";
int version_firmware = 12;







// *************************************************************************************************************************************************************************
// ************************************************************************************************************************  PARAMETROS DE CALIBRACION  ********************
// *************************************************************************************************************************************************************************

//Constantes del servo de transmisor






//Constantes de los timers para falla en movimientos (segundos)
unsigned int Timeout_ABRIR_Domo = 20;
unsigned int Timeout_CERRAR_Domo = 20;
unsigned int Timeout_ABRIR_PAD = 10;
unsigned int Timeout_CERRAR_PAD = 10;

const unsigned MAXTIMERS = 22;  //MAXTIMERS timers TIPO plc de la aplicacion

uint8_t angleKp = 100;
uint8_t angleKi = 80;
uint8_t speedKp = 50;
uint8_t speedKi = 20;
uint8_t torqueKp = 100;
uint8_t torqueKi = 100;



//Parametros condiciones ambientales




unsigned long previousMillis = 0;
unsigned int T_Count_temp = 0;

//Parametros de tiempo configurables



unsigned int Umbral_alto_Temp_bateria_Drone_ON = 500;
unsigned int Umbral2_alto_Temp_bateria_Drone_ON = 200;
unsigned int Umbral_bajo_Temp_bateria = 150;



unsigned long currentMillis_BATERIA;
unsigned long previousMillis_BATERIA;
unsigned long limiteMillis_BATERIA = 15000;



uint8_t comando_motor[20];
uint16_t status_motor[30];
uint16_t encoder;
uint16_t encoder_original;
uint16_t encoder_offset;
uint16_t encoder2;
uint16_t encoder_original2;
uint16_t encoder_offset2;
uint16_t encoder1;
uint16_t encoder_original1;
uint16_t encoder_offset1;


uint16_t encoderLSB;
uint16_t encoderMSB;
uint16_t encoder_originalLSB;
uint16_t encoder_originalMSB;
uint16_t encoder_offsetLSB;
uint16_t encoder_offsetMSB;


byte index = 0;
uint8_t MOTOR = 1;

// -------------  Definicion de los parametros Ethernet de default ------------------





// Timers tipo PLC
volatile unsigned int contador_timer0 = 0;
volatile PLC_Timer timer[MAXTIMERS];  // 20 timers  tipo PLC para la aplicacion


//mailbox de protocolo Modbus RTU
//Mudbus Mb; //Se declara la variable Mb del tipo "Mudbus"



// ***************** salidas digitales ************************

//**variables de control de motores de pasos  **




boolean PAD_cerrado_primera_vez = false;
boolean Domo_Cerrado_primera_vez = false;
boolean CHARGE_ON_primera_vez = false;





boolean Cargador_ON = false;
boolean Cargador_ON_activo = false;





boolean Ventilador_ON_Auto = false;
boolean Ventilador_ON_Carga_Auto = false;


boolean Comando_abrir_ejes_pad_enviado = false;
boolean Comando_ABRIR_Domo_enviado = false;
boolean Comando_CERRAR_Domo_enviado = false;
boolean Comando_cerrar_ejes_pad_enviado = false;
boolean Comando_DRONE_POWER_enviado = false;
boolean Comando_DRONE_CHARGE_enviado = false;

boolean Comando_motor_stop_enviado = false;
boolean Comando_motor_operate_enviado = false;





// **************** variables de Entradas digitales **********************




//entradas  digitales virtuales creadas por comparacion de posicion

boolean PAD_closed2 = false;




boolean PAD_closed_PREV = false;



// **************** Entradas Analogicas **********************




float Voltaje_bateria_drone_raw = 0;





// Variables generales


unsigned int Error_base = 0;


int posicion_Domo2 = 0;
int reset_Domo_fase1 = 80;
int reset_Domo_fase2 = 680;

int reset_limite_fase1 = 65;
//offset = 30 en Obispo. Offset = 15 en San Vicente
int reset_offset_fase2 = 15; 

int posicion_Domo2_anterior = 0;

int posicion_PAD2 = 0;

//variables maquinas de Estado






unsigned int Edo_maq_lee_bateria = 0;


unsigned int Edo_maq_Reset_HOME_PAD = 0;



//booleanas

boolean Debug = false;
boolean ABRIR = false;
boolean CERRAR = false;

boolean ABRIR_A = false;
boolean CERRAR_A = false;
boolean ABRIR_L = false;
boolean CERRAR_L = false;
boolean cmd_ABRIR_Domo = false;
boolean cmd_CERRAR_Domo = false;


boolean PAD_close = false;
boolean PAD_close_A = false;
boolean PAD_close_L = false;

boolean PAD_open = false;
boolean PAD_open_A = false;
boolean PAD_open_L = false;


boolean cmd_PAD_close = false;
boolean cmd_PAD_open = false;




boolean Secuencia_ABRIR2 = false;
boolean Secuencia_CERRAR2 = false;

boolean Secuencia_HOME2 = false;
boolean Secuencia_PRUEBA = false;

boolean bandera_TX_auto = true;

boolean  PAD_ON = false;
boolean CHARGE_ON = false;

boolean CHARGE_ON_Auto = false;
boolean CHARGE_ON_Carga_Auto = false;
boolean Auto_Manual_ON = false;
boolean Auto_Manual_ant = false;
boolean Remoto_Local_ON = false;
boolean Remoto_Local_ant = false;
boolean Manual_ON = false;
boolean Manual_ant = false;


boolean sistema_habilitado_ON = false;
boolean sistema_habilitado_ant = false;
boolean sistema_deshabilitado_ON = false;
boolean sistema_deshabilitado_ant = false;




boolean ABRIR_Domo_falla = false;
boolean CERRAR_Domo_falla = false;

boolean ABRIR_PAD_falla = false;
boolean CERRAR_PAD_falla = false;

//*****************************************************************************************************************************//
// ******************************************  IMPLEMENTACION DE TIMERS PLC  ****************************************************
//*****************************************************************************************************************************//





void configura_arduino_timers() {

	noInterrupts();           // disable all interrupts

	// initialize timer0
	// Timer0 configuration for interrupt every 1 mSeg
	OCR0A = 0xFA;  //OCRx - Output Compare Register Timer 0FA = 250 ->compare match register = 16MHz/64/1000Hz = 250

	TIMSK0 |= _BV(OCIE0A);  //TIMSKx - Timer/Counter Interrupt Mask Register. To enable/disable timer interrupts.


	/*
	// initialize timer1  Se usa para control de velocidad de Puertas y centrador Y

	//timer1 configuracion para  interrupciones cada 0.16 mSeg
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1  = 0;
	OCR1A = 5;               //OCRx - Output Compare Register Timer 1  ->compare match register = 16MHz/256/6250Hz = 10
	TCCR1B |= (1 << WGM12);   // CTC mode
	TCCR1B |= (1 << CS12);    // 256 prescaler
	TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

	*/

	interrupts();     // enable all interrupts

}


void inicia_timers() {
	unsigned i;

	for (i = 0; i < MAXTIMERS; i++) {   //MAXTIMERS timers TIPO plc de la aplicacion
		timer[i].PV = 0;
		timer[i].EN = false;
		timer[i].OUT = false;
		timer[i].PREV = false;
		// duracion default del timer 2 X 1 = 2 Seg
		timer[i].PSET = 2; // cuaquier otra aplicacion cmbiar este PRESET

	}

	//habilitacion de timers astables fijos para la aplicacion

	//timer[0]   Sensor de Temperatura_inty Humedad_intcada  5 minutos (300 Seg)

	timer[0].AST = true; //Timer 0 se configura astable para que oscile todo el tiempo
	timer[0].PSET = 300; //Timer[0] duracion  300 Seg  300 X 1 = 300Seg = 5 mins
	timer[0].EN = true;  //habilita todo el tiempo el timer [0]  para que empiece a contar

}

// implemetacion de filtros para entradas digitales. Se basa en un Timer_ON con durecion de 200 mSeg

boolean Timer_filtro_ON(boolean input, unsigned int timer_num) {


	//timer[timer_num].AST =false;   //por default es false
	//timer[timer_num].PSET = 2; //duracion del filtro por default = 2 X 0.1 = 200 mSeg

	timer[timer_num].EN = input; //  la entrada habilita al timer. si es  false el timer se apaga !
	return timer[timer_num].OUT;
}

// *******   Timer 0 Interrupt entra cada milissegundo  - 1 mSeg-  ********

SIGNAL(TIMER0_COMPA_vect)
{

	unsigned i;


	contador_timer0++;

	if (contador_timer0 >= 980) {     //duracion de los timers de aplicacion cada 1 segundos -1000 mS-
		contador_timer0 = 0;



		for (i = 0; i < MAXTIMERS; i++) {  //MAXTIMERS timers TIPO plc de la aplicacion

			if (timer[i].EN) {
				timer[i].PV++;


				if (timer[i].PV >= timer[i].PSET) {
					timer[i].OUT = true;    //se activa la salida del timer y se queda activa mientras este activo timer[i].EN

					if (timer[i].AST)//oscilador astable
						timer[i].PV = 0;

					//deteccion FLANCO de subida
					if (timer[i].OUT && !timer[i].PREV) {
						timer[i].POS = true;     //esto dura solo un ciclo del timer! o sea 100 mSeg!
						timer[i].PREV = timer[i].POS;
					}
					else
						timer[i].POS = false;


				}
				else { //todavia no llega al PRESET time
					timer[i].OUT = false; //quizas redundante mantenerlo en cero hasta que no llega a la cuenta
					timer[i].POS = false; //necesario para timers astables
					timer[i].PREV = false;
				}


			}
			else { //se apago la entrada del timer
				timer[i].OUT = false; //la salida se apaga en cuanto se apaga la entrada
				timer[i].POS = false;
				timer[i].PREV = false;
			}

		}

	}
}






void setup_parametros() {

	Serial.println("setup parametros");

	int eeAddress = 50;   //primera localidad de EEPROM. Aqui esta la bandera " direccion_configurada"
	byte parametros_configurados = 0;


	eeAddress = 50;
	EEPROM.get(eeAddress, Ciclos);

	eeAddress = 20;
	parametros_configurados = EEPROM.read(eeAddress);

	Serial.println("parametros_configurados antes de leerlos del EEPROM");
	Serial.print("parametros_configurados ");
	Serial.println(parametros_configurados);
	Serial.print("Temperatura_OK ");
	Serial.println(Temperatura_OK);
	Serial.print("Humedad_OK ");
	Serial.println(Humedad_OK);
	Serial.print("vel_viento_OK ");
	Serial.println(vel_viento_OK);
	Serial.print("max_posicion_PAD ");
	Serial.println(max_posicion_PAD);
	Serial.print("vel_PAD ");
	Serial.println(vel_PAD);

	Serial.print("tiempo Enfriado  ");
	Serial.println(tiempo_enfriado);
	Serial.print("tiempo Carga  ");
	Serial.println(tiempo_carga);
	Serial.print("tiempo descarga Video  ");
	Serial.println(tiempo_video);






	if (parametros_configurados != 165) { //los parametros NO estan configurados, primera vez, se toman los de default


		parametros_configurados = 165; //la activo con el codigo 10100101B  por tener muchos unos y ceros en cierta posicion 



	  //escribe los parametros en el EEPROM
		eeAddress = 20;
		EEPROM.write(eeAddress, parametros_configurados);

		eeAddress = 21;
		EEPROM.put(eeAddress, Temperatura_OK);
		eeAddress += 2;
		EEPROM.put(eeAddress, Humedad_OK);
		eeAddress += 2;
		EEPROM.put(eeAddress, vel_viento_OK);
		eeAddress += 2;
		EEPROM.put(eeAddress, max_posicion_PAD);
		eeAddress += 2;
		EEPROM.put(eeAddress, vel_PAD);
		eeAddress += 2;
		EEPROM.put(eeAddress, tiempo_enfriado);
		eeAddress += 2;
		EEPROM.put(eeAddress, tiempo_carga);
		eeAddress += 2;
		EEPROM.put(eeAddress, tiempo_video);
		eeAddress += 2;




	}
	else { // los parametros ya estan configurados:  direccion_configurada ==165, obtiene la direccion del EEPROM

	//lee los parametros ethernet del EEPROM

		eeAddress = 21;
		EEPROM.get(eeAddress, Temperatura_OK);
		eeAddress += 2;
		EEPROM.get(eeAddress, Humedad_OK);
		eeAddress += 2;
		EEPROM.get(eeAddress, vel_viento_OK);
		eeAddress += 2;
		EEPROM.get(eeAddress, max_posicion_PAD);
		eeAddress += 2;
		EEPROM.get(eeAddress, vel_PAD);
		eeAddress += 2;
		EEPROM.get(eeAddress, tiempo_enfriado);
		eeAddress += 2;
		EEPROM.get(eeAddress, tiempo_carga);
		eeAddress += 2;
		EEPROM.get(eeAddress, tiempo_video);
		eeAddress += 2;




	}

	Serial.println("Parametros leidos de EEPROM");
	Serial.print("parametros_configurados ");
	Serial.println(parametros_configurados);
	Serial.print("Temperatura_OK ");
	Serial.println(Temperatura_OK);
	Serial.print("Humedad_OK ");
	Serial.println(Humedad_OK);
	Serial.print("vel_viento_OK ");
	Serial.println(vel_viento_OK);
	Serial.print("max_posicion_PAD ");
	Serial.println(max_posicion_PAD);
	Serial.print("vel_PAD ");
	Serial.println(vel_PAD);


	Serial.print("tiempo Enfriado  ");
	Serial.println(tiempo_enfriado);
	Serial.print("tiempo Carga  ");
	Serial.println(tiempo_carga);
	Serial.print("tiempo descarga Video  ");
	Serial.println(tiempo_video);




}


void new_parametros() {
	int eeAddress = 20;   //primera localidad de EEPROM. Aqui esta la bandera " parametros_configurados"
	byte parametros_configurados = 165; //la activo con el codigo 10100101B  por tener muchos unos y ceros en cierta posicion

	/*
	//Constantes de los timers para falla en movimientos (decimas de segundo) // si no se modifican desde el HMI, escribe lo mismo que tenia
	//*Mb.R[42] = Timeout_ABRIR_Domo;
	//*Mb.R[43] = Timeout_CERRAR_Domo;
	//*Mb.R[44] = servo1_position0;
	//*Mb.R[45] = servo1_position1;
	////*Mb.R[46] = Timeout_ABRIR_PAD;
	////*Mb.R[47] = Timeout_CERRAR_PAD;
	//*Mb.R[46] = Primer_Pulso_ON_OFF;
	//*Mb.R[47] = Segundo_Pulso_ON_OFF;
*/
	if (actualizar_Parametros) {


		Serial.println("********* new_parametros  escribe estos parametros en el EEPROM************");
		Serial.print("parametros_configurados ");
		Serial.println(parametros_configurados);
		Serial.print("Temperatura_OK ");
		Serial.println(Temperatura_OK);
		Serial.print("Humedad_OK ");
		Serial.println(Humedad_OK);
		Serial.print("vel_viento_OK ");
		Serial.println(vel_viento_OK);
		Serial.print("max_posicion_PAD ");
		Serial.println(max_posicion_PAD);
		Serial.print("vel_PAD ");
		Serial.println(vel_PAD);

		Serial.print("tiempo Enfriado  ");
		Serial.println(tiempo_enfriado);
		Serial.print("tiempo Carga  ");
		Serial.println(tiempo_carga);
		Serial.print("tiempo descarga Video  ");
		Serial.println(tiempo_video);



		Serial.println("******************************************************");


		//escribe los parametros en el EEPROM
		eeAddress = 20;
		EEPROM.write(eeAddress, parametros_configurados);

		eeAddress = 21;
		EEPROM.put(eeAddress, Temperatura_OK);
		eeAddress += 2;
		EEPROM.put(eeAddress, Humedad_OK);
		eeAddress += 2;
		EEPROM.put(eeAddress, vel_viento_OK);
		eeAddress += 2;
		EEPROM.put(eeAddress, max_posicion_PAD);
		eeAddress += 2;
		EEPROM.put(eeAddress, vel_PAD);
		eeAddress += 2;
		EEPROM.put(eeAddress, tiempo_enfriado);
		eeAddress += 2;
		EEPROM.put(eeAddress, tiempo_carga);
		eeAddress += 2;
		EEPROM.put(eeAddress, tiempo_video);
		eeAddress += 2;




		//lee los parametros ethernet del EEPROM
		eeAddress = 20;
		EEPROM.get(eeAddress, parametros_configurados);

		eeAddress = 21;
		EEPROM.get(eeAddress, Temperatura_OK);
		eeAddress += 2;
		EEPROM.get(eeAddress, Humedad_OK);
		eeAddress += 2;
		EEPROM.get(eeAddress, vel_viento_OK);
		eeAddress += 2;
		EEPROM.get(eeAddress, max_posicion_PAD);
		eeAddress += 2;
		EEPROM.get(eeAddress, vel_PAD);
		eeAddress += 2;
		EEPROM.get(eeAddress, tiempo_enfriado);
		eeAddress += 2;
		EEPROM.get(eeAddress, tiempo_carga);
		eeAddress += 2;
		EEPROM.get(eeAddress, tiempo_video);
		eeAddress += 2;




		Serial.println("****Valores de Parametros despues de leer en EEPROM****");
		Serial.print("parametros_configurados ");
		Serial.println(parametros_configurados);
		Serial.print("Temperatura_OK ");
		Serial.println(Temperatura_OK);
		Serial.print("Humedad_OK ");
		Serial.println(Humedad_OK);
		Serial.print("vel_viento_OK ");
		Serial.println(vel_viento_OK);
		Serial.print("max_posicion_PAD ");
		Serial.println(max_posicion_PAD);
		Serial.print("vel_PAD ");
		Serial.println(vel_PAD);

		Serial.print("tiempo Enfriado  ");
		Serial.println(tiempo_enfriado);
		Serial.print("tiempo Carga  ");
		Serial.println(tiempo_carga);
		Serial.print("tiempo descarga Video  ");
		Serial.println(tiempo_video);




		Serial.println("******************************************************");


		actualizar_Parametros = false;
		////*Mb.C[40] = false; //necesario
	}
}

//------------------------------------------------------------------------------------------------------------------------------------------ MODBUS Routines
/*
void lee_modbus() {
	Mb.Run(); //Update the values of Mb.R and Mb.C every loop cycle


	//-------------------------------
	// lectura y escritura de registros desde kepware
	//-------------------------------
	//lectura  registros desde kepware
	unsigned long MSB_posicion_Domo = 0;
	unsigned long MSB_posicion_PAD = 0;


	//*Mb.R[1] = mensajes;
	//*Mb.R[2] = Edo_maq_ABRIR;
	//*Mb.R[3] = Edo_maq_CERRAR;
	//*Mb.R[4] = Edo_maq_HOME;

	//*Mb.R[12] = Temperatura_int; //Viene multiplicada por 100, Aqui se hace el cast a integer porque este MODBUS no maneja floats!
	//*Mb.R[13] = Humedad_int; //Viene multiplicada por 100, Aqui se hace el cast a integer porque este MODBUS no maneja floats!
	//*Mb.R[14] = vel_viento_int; //Viene multiplicada por 100, Aqui se hace el cast a integer porque este MODBUS no maneja floats!
	//*Mb.R[15] = Voltaje_bateria_int;  //Viene multiplicada por 100, Aqui se hace el cast a integer porque este MODBUS no maneja floats!

	////*Mb.R[18] = (int)(posicion_Domo*100.0 / max_posicion_Domo * 1.0); // (max_posicion_Domo / 100) * 100; // / 75000 equivale a 180 grados en el engrane secundario;
	////*Mb.R[20] = (int)(posicion_PAD*100.0 / max_posicion_PAD  *1.0); // (max_posicion_PAD / 100) * 100; // 83000


	//*Mb.R[18] = (int)(posicion_Domo * 1.0 / 750 * 100); // (max_posicion_Domo / 100) * 100; // / 75000 equivale a 180 grados en el engrane secundario;
	//*Mb.R[20] = (int)(posicion_PAD * 1.0 / 70 * 100); // (max_posicion_PAD / 100) * 100; // 83000



	//*Mb.R[22] = Temp_bateria_int;  //Viene multiplicada por 100, Aqui se hace el cast a integer porque este MODBUS no maneja floats!  


	//*Mb.R[24] = ciclos_prueba;

	//*Mb.R[50] = Ciclos;  //Cuenta los ciclos de operacion del drone-puerto
	//*Mb.R[51] = version_firmware;
	//-------------------------------
	// escritura registros  desde kepware
	//-------------------------------
	Temperatura_OK = //*Mb.R[9];
	Humedad_OK = //*Mb.R[10];
	vel_viento_OK = //*Mb.R[11];
	Voltaje_bateria_drone_OK = //*Mb.R[16];

	Temperatura_bateria_drone_MAX = //*Mb.R[17];

	Max_ciclos_PRUEBA = //*Mb.R[23];



	tiempo_carga = //*Mb.R[27];
	tiempo_video = //*Mb.R[28];
	tiempo_enfriado = //*Mb.R[26];

	new_IP[0] = //*Mb.R[30];
	new_IP[1] = //*Mb.R[31];
	new_IP[2] = //*Mb.R[32];
	new_IP[3] = //*Mb.R[33];
	new_MSK[0] = //*Mb.R[34];
	new_MSK[1] = //*Mb.R[35];
	new_MSK[2] = //*Mb.R[36];
	new_MSK[3] = //*Mb.R[37];
	new_gateway[0] = //*Mb.R[38];
	new_gateway[1] = //*Mb.R[39];
	new_gateway[2] = //*Mb.R[40];
	new_gateway[3] = //*Mb.R[41];

	//Constantes de los timers para falla en movimientos (segundos)
	Timeout_ABRIR_Domo = //*Mb.R[42];
	Timeout_CERRAR_Domo = //*Mb.R[43];
	servo1_position0 = //*Mb.R[44];
	servo1_position1 = //*Mb.R[45];
	//Timeout_ABRIR_PAD = //*Mb.R[46];
	//Timeout_CERRAR_PAD = //*Mb.R[47];
	Primer_Pulso_ON_OFF = //*Mb.R[46];
	Segundo_Pulso_ON_OFF = //*Mb.R[47];
	max_posicion_PAD = //*Mb.R[48];
	vel_PAD = //*Mb.R[49];




	//-------------------------------
	//lectura  bits desde kepware
	//-------------------------------
	//*Mb.C[1] = Permisivo_despegue;

	//*Mb.C[2] = Domo_Abierto;
	//*Mb.C[3] = Domo_Cerrado;

	//*Mb.C[4] = Voltaje_Bateria_bajo;
	//*Mb.C[5] = Temperatura_bateria_alta;

	//*Mb.C[6] = DRONE_ON;
	//*Mb.C[7] = base_abierta;
	//*Mb.C[8] = sistema_habilitado;
	//*Mb.C[9] = base_cerrada;

	//*Mb.C[23] = PAD_opened;
	//*Mb.C[24] = Warning_bateria;
	//*Mb.C[31] = C_Ambientales_OK;
	//*Mb.C[34] = PAD_closed;
	////*Mb.C[35] = libre ;

	//*Mb.C[36] = LIBRE
	//*Mb.C[17] = Retro_edo_TX;

	////*Mb.C[45] = habilita_Warning_Temperatura_bateria;
	////*Mb.C[46] = habilita_Warning_Voltaje_bateria;

	//-------------------------------
	// escritura bits  desde kepware
	//-------------------------------
	Auto_Manual = //*Mb.C[21];
	Remoto_Local = //*Mb.C[22];
	CHARGE_ON_Manual = //*Mb.C[18];

	if (//*Mb.C[10]) {
		habilita_sistema();
	}
	else {
		deshabilita_sistema();
	}


	home_base = //*Mb.C[33];
	reset_dronepuerto = //*Mb.C[42];

	PE = //*Mb.C[47]; //28 junio 2021 lo cambie de direccion para deshabilitarlo del HMI que parece esta mandndo paros de Emergencia..
	bandera_TX_auto = //*Mb.C[41];
	Debug = //*Mb.C[43];
	Secuencia_Encender_DRONE_Manual = //*Mb.C[12];
	Secuencia_Encender_TX_Manual = //*Mb.C[37];
	actualizar_IP = //*Mb.C[38];
	Ventilador_ON_Manual = //*Mb.C[39];
	actualizar_Parametros = //*Mb.C[40];

	inicia_pruebas_A = //*Mb.C[44];
	habilita_pruebas = //*Mb.C[32];

	habilita_Warning_Temperatura_bateria = //*Mb.C[45];
	habilita_Warning_Voltaje_bateria = //*Mb.C[46];


	//Comandos manuales de abrir y cerrar desde kepware
	if (//*Mb.C[13] && //*Mb.C[14]) { //PROTECCION No se pueden tener los dos comandos a la vez desde MODBUS
		//*Mb.C[13] = false;
		//*Mb.C[14] = false;
	}
	if (//*Mb.C[13]) { //Abrir Domo
		ABRIR_M = true;
		CERRAR_M = false;
	}
	if (//*Mb.C[14]) { //Cerrar Domo
		ABRIR_M = false;
		CERRAR_M = true;;
	}


	//bits de control de secuencias de abrir-subir y cerrar-bajar
	if (//*Mb.C[19] && //*Mb.C[20]) { //PROTECCION No se pueden tener los dos comandos a la vez desde MODBUS
		//*Mb.C[19] = false;
		//*Mb.C[20] = false;
	}
	if (//*Mb.C[19]) { //Secuencia_Abrir Domo
		Secuencia_ABRIR = true;
		Secuencia_CERRAR = false;
	}
	if (//*Mb.C[20]) { //Secuencia_Cerrar Domo
		Secuencia_ABRIR = false;
		Secuencia_CERRAR = true;
	}


	if (//*Mb.C[15])  //RESET de energia de CPU sistema de computo
		reset_Computo = false;
	else
		reset_Computo = true;



	if (//*Mb.C[16]) 	  //Energizar Peine
		Energizar_PEINE_Manual = true;
	else
		Energizar_PEINE_Manual = false;


	//digitalWrite(reset_Khadas_output, reset_Computo);
	//digitalWrite(Energizar_PEINE_output, Energizar_PEINE);




	//Comandos manuales de PAD_close y PAD_open desde kepware
	if (//*Mb.C[25] && //*Mb.C[26]) { //PROTECCION No se pueden tener los dos comandos a la vez desde MODBUS
		//*Mb.C[25] = false;
		//*Mb.C[26] = false;
	}
	if (//*Mb.C[25]) { //PAD_close
		PAD_close_M = true;
		PAD_open_M = false;
	}
	if (//*Mb.C[26]) { //PAD_open
		PAD_close_M = false;
		PAD_open_M = true;
	}



	//Control de la Secuencia de carga en Automatico
	Secuencia_cargar_auto = //*Mb.C[29];  //Secuencia_Cargar

	hab_Secuencia_cargar_auto = //*Mb.C[30]; ////habilitar Secuencia_cargar





}
*/

void reset_comandos() {

	//comandos remotos desde Modbus

	//CHARGE_ON
	CHARGE_ON_Manual = false;
	//*Mb.C[18] = CHARGE_ON_Manual;

	CHARGE_ON_Auto = false;


	//PAD front && back manual
	//*Mb.C[25] = false;
	//*Mb.C[26] = false;

	//PAD front && back manual
	//*Mb.C[27] = false; //libre
	//*Mb.C[28] = false;  //libre

	//Secuencias cargar en Automatico
	//Secuencia_cargar_auto = false; // comentado para que funcione aun con el sistema deshabilitado
	////*Mb.C[29] = false;
	//hab_Secuencia_cargar_auto = false;  //comentado para que se quede trabajando 
	////*Mb.C[30] = false;

		//bandera de encender en Automatico e transmisor por default en 1
	bandera_TX_auto = true;
	//*Mb.C[41] = bandera_TX_auto;


	//Abrir y cerrar manual 
	//*Mb.C[13] = false;
	//*Mb.C[14] = false;

	//Secuencias Abrir y cerrar Automatico
	//*Mb.C[19] = false;
	//*Mb.C[20] = false;

	//Ventilador_ON
	//*Mb.C[39] = false;
	Ventilador_ON_Auto = false;
	Ventilador_ON_Manual = false;
	//actualizar_IP
	////*Mb.C[38] = false;

	//actualizar_Parametros
	////*Mb.C[40] = false;


	// Comandos locales

	ABRIR = false;
	CERRAR = false;


	ABRIR_L = false;
	CERRAR_L = false;


	ABRIR_A = false;
	CERRAR_A = false;


	ABRIR_M = false;
	CERRAR_M = false;




	PAD_open = false;
	PAD_close = false;
	PAD_close_L = false;
	PAD_open_L = false;
	PAD_close_A = false;
	PAD_open_A = false;
	PAD_close_M = false;
	PAD_open_M = false;


	PAD_closed_PREV = false;
	//PAD_cerrado_primera_vez = false;



  //rompe los enclavamientos por si se quedaron activos a la mitad de un movimiento
	cmd_ABRIR_Domo = false;
	cmd_CERRAR_Domo = false;



	cmd_PAD_close = false;
	cmd_PAD_open = false;


	//comandos de las maquinas de estado (automatico)


	ABRIR_A = false;
	CERRAR_A = false;


	PAD_close_A = false;
	PAD_open_A = false;


	//comandos digitales
	Comando_abrir_ejes_pad_enviado = false;
	Comando_cerrar_ejes_pad_enviado = false;
	Comando_ABRIR_Domo_enviado = false;
	Comando_CERRAR_Domo_enviado = false;


	//Comando_motor_stop_enviado = false;
	//Comando_motor_operate_enviado = false;

	  //inicia_maquinas_de_estados();  NO ponerlo AQUI

	//reset de fallas por timeout

	ABRIR_Domo_falla = false;
	CERRAR_Domo_falla = false;
	ABRIR_PAD_falla = false;
	CERRAR_PAD_falla = false;



	Cargador_ON = false;//regresa

	Energizar_PEINE_Manual = false;

	//reset equipo de computo
	/*
	reset_Computo = false;
	//*Mb.C[15] = false;


	  */

	//RESET TIMERS DE FALLA
	ABRIR_Domo_falla = false;
	timer[7].EN = false;
	timer[7].PV = 0;

	CERRAR_Domo_falla = false;
	timer[8].EN = false;
	timer[8].PV = 0;

	ABRIR_PAD_falla = false;
	timer[12].EN = false;
	timer[12].PV = 0;

	CERRAR_PAD_falla = false;
	timer[13].EN = false;
	timer[13].PV = 0;

}

//*******************************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************  INICIA I2C ROUTINES  FOR TEMPERATURE AND Humidity SENSOR ***** ////
//*******************************************************************************************************************************************************************************************************//
#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
//#define dataPin  20  //NO PUEDO USAR ESTOS PORQUE EL BUS I2C los usa ethernet
//#define clockPin 21

#define dataPin  3
#define clockPin 2
SHT1x sht1x(dataPin, clockPin);
float temp_c;
float temp_f;


void read_TH_sensor() { //este procedimiento se llama cada 30 Seg y actualiza las variables de Temperatura_inty humedad


 /**
 * ReadSHT1xValues
 *
 * Read temperature and Humidity values from an SHT1x-series (SHT10,
 * SHT11, SHT15) sensor.
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au>
 * www.practicalarduino.com
 */







 // Read values from the sensor
	temp_c = sht1x.readTemperatureC();
	temp_f = sht1x.readTemperatureF();
	Humidity = sht1x.readHumidity();

	// Print the values to the serial port
	//Serial.print("Temperature: ");
	//Serial.print(temp_c, DEC);
	//Serial.print("C / ");
	//Serial.print(temp_f, DEC);
	//Serial.print("F. Humidity: ");
	//Serial.print(Humidity);
	//Serial.println("%");



	//Humidity /= 10; //se deja como entero porque modbus no maneja floats!
	//Serial.print("Humedad_int= "); Serial.println(Humidity);

	Temperature = temp_c;
	//Temperature /= 10;  //se deja como entero porque modbus no maneja floats!
	//Serial.print("Temperatura_int= "); Serial.println(Temperature);



}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//********************************* PROGRAMA COMBINACIONAL SIMULACION DE LADDER *******************************************//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void habilita_sistema() //Esta rutina se llama CONTINUAMENTE, depende solo del bit de Modbus //*Mb.C[10]==1 que viene de kepware!
{

	sistema_habilitado = true;
	//*Mb.C[8] = sistema_habilitado; // lo hago aqui tambien por velocidad del monitoreo






}

void deshabilita_sistema()  //Esta rutina se llama CONTINUAMENTE, depende solo del bit de Modbus //*Mb.C[10]==0 que viene de kepware!
{

	sistema_habilitado = false;
	//*Mb.C[8] = sistema_habilitado; // lo hago aqui tambien por velocidad del monitoreo




	//apaga los comandos de secuencias

	inicia_maquinas_de_estados(); // MUY  IMPORTANTE PARA QUE AL DESHABILITAR EL SISTEMA DE REINICIEN LAS MAQUINAS DE ESTADOS

	Secuencia_ABRIR = false;
	Secuencia_CERRAR = false;

	Secuencia_HOME = false;
	//Secuencia_PRUEBA = false; // LO TENGO QUE QUITAR PARA QUE NO MATE A LA MAQUINA DE PRUEBA!!

	//Secuencia_Encender_DRONE = false;  // esta maquina puede correr aun estando el sistema deshabilitado
	//Secuencia_Encender_TX = false;  // esta maquina puede correr aun estando el sistema deshabilitado


	reset_comandos();


}










/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************** funcionamiento de PLC ***************************************************//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void lee_entradas_PLC() {



	//Entradas digitales fisicas



	 //Actualiza variables booleanas de estado de los ejes de la base 

	base_cerrada = Domo_Cerrado && PAD_closed;
	base_abierta = Domo_Abierto && PAD_opened;

	//Entradas digitales virtuales




  //Entradas Analogicas

	  // read analog value from wind speed  sensor:
	vel_viento = analogRead(wind_sensor_input);
	vel_viento = vel_viento / 100 - 0.4;// Cambio de escala a m/Seg. la escala maxima es 32.5m/Seg  
	vel_viento = vel_viento / 1.6 * 60;
	vel_viento = vel_viento * 0.52 + 1.147;







	//Actualiza valores a variables  de globales de posicion de la base 

	if ((posicion_Domo) >= (max_posicion_Domo - TOL1) / 100) {
		Domo_Cerrado = true;
		Domo_Abierto = false;
	}


	else if ((posicion_Domo) <= (min_posicion_Domo + TOL1) / 100) {
		Domo_Abierto = true;
		Domo_Cerrado = false;
	}
	else
	{
		Domo_Abierto = false;
		Domo_Cerrado = false;
	}




	if ((posicion_PAD * 100) >= (max_posicion_PAD - TOL2)) {
		PAD_closed = true;
		PAD_opened = false;
	}

	else  if ((posicion_PAD * 100) <= (min_posicion_PAD + TOL2)) {

		PAD_opened = true;
		PAD_closed = false;

	}
	else {
		PAD_closed = false;
		PAD_opened = false;

	}


	//deteccion FLANCO de subida de PAD cerrado
	if (PAD_closed && !PAD_cerrado_primera_vez) {

		PAD_cerrado_primera_vez = true;     //esto dura solo un ciclo de cerrado
		Serial.println("PAD Cerrado");

		//lee_bateria();

		timer[18].AST = false; //Timer18  NO  astable 
		timer[18].PSET = 6; //Timer[18] duracion  2 Seg  
		timer[18].EN = true;

	}
	if (PAD_opened) {
		Cargador_ON = false;//regresa   //se desergiza el cargador para que no haya energia en los contactos del PAD mientras esta abierto
		PAD_cerrado_primera_vez = false;
	}


	if (PAD_closed) {

		if (timer[18].OUT) {
			if (timer[18].EN == true) {
				Serial.println("Drone estable");
				lee_bateria();
				display_bateria();
			}
				
			timer[18].EN = false;
			timer[18].PV = 0;


		}
	}



	//deteccion FLANCO de subida de Domo_Cerrado
	if (Domo_Cerrado && !Domo_Cerrado_primera_vez) {

		Domo_Cerrado_primera_vez = true;     //esto dura solo un ciclo de cerrado
		Serial.println("Domo_Cerrado");

		timer[32].AST = false; //Timer18  NO  astable 
		timer[32].PSET = 3; //Timer[32] duracion  3 Seg  
		timer[32].EN = true;

	}
	if (Domo_Abierto) {

		Domo_Cerrado_primera_vez = false;
	}







	//deteccion FLANCO de subida de CHARGE_ON
	if (CHARGE_ON && !CHARGE_ON_primera_vez) {
		CHARGE_ON_primera_vez = true;     //esto se activa solo la primera vez
		Serial.println("Carga ON");
		lee_bateria();
		
		//Edo_maq_PEINE_ON = 0; //  NO HACERLO!, causa un loop al mandar CHARGE_ON si la maquina esta corriendo en ese momento
		Cargador_ON = true; //lo va a matar el edo 2 de  maquina de lee_BATERIA


	}

	if (!CHARGE_ON) {
		CHARGE_ON_primera_vez = false;
		Cargador_ON = false;
	}
		

}



void escribe_salidas() {




	if (PE && sistema_habilitado) {

		Comando_motor_operate_enviado = false;
		if (!Comando_motor_stop_enviado) {
			Serial.println("Paro de EMERGENCIA Activo");
			Comando_motor_stop(1);
			delay(15);
			Comando_motor_stop(2);
			Serial.println("Comando_motor_stop_enviado"); //Revisar
			Comando_motor_stop_enviado = true;
		}

	}
	else {
	
		Comando_motor_stop_enviado = false;
		if (!Comando_motor_operate_enviado) {
			Serial.println("Paro de EMERGENCIA Desactivo");
			Comando_motor_operate(1);
			delay(15);
			Comando_motor_operate(2);
			Serial.println("Comando_motor_operate_enviado"); //Revisar
			Comando_motor_operate_enviado = true;
		}



	}




	digitalWrite(Ventilador_ON_output, Ventilador_ON); //ahorita control manual exclusivo desde panel de control
	

	//digitalWrite(CHARGE_ON_output, CHARGE_ON); //ahorita control manual exclusivo desde panel de control
	digitalWrite(CHARGE_ON_output, Cargador_ON); // //regresa


	

}


void Reset_maquinas_DJI() {


	//*Mb.C[12] = Secuencia_Encender_DRONE_Manual = false;; //necesario



	Edo_maq_lee_bateria = 0;

}

void procesa() {






	// escritura de parametros dede kepware
	if (actualizar_Parametros) {
		new_parametros();
		actualizar_Parametros = false;
		//*Mb.C[40] = false; //necesario
	}


	if (home_base) {
		condini();
		inicia_maquinas_de_estados();
		Reset_maquinas_DJI();
		//reset_comandos();  //lo hace Condini

		Secuencia_HOME = true;
		home_base = false;
		//*Mb.C[33] = false; //Necesario para que Kepware no force el valor 
	}

	if (reset_dronepuerto) {
		Reset_maquinas_DJI();
		reset_comandos();
		// inicia_maquinas_de_estados();//causa caos

		 //reset de fallas por timeout

		ABRIR_Domo_falla = false;
		CERRAR_Domo_falla = false;
		ABRIR_PAD_falla = false;
		CERRAR_PAD_falla = false;



		reset_dronepuerto = false;
		//*Mb.C[42] = false; //Necesario para que Kepware no force el valor 
	}


	// lectura del sensor de Temperatura_inty Humedad_intcada 5 mins
	if (timer[0].POS) {
		timer[0].POS = false;  //POS activo solo un ciclo del scan ppal
		read_TH_sensor();
		Serial.println("---------------------------------------------- ");
		Serial.print("Temperatura = ");
		Serial.print(Temperature);
		Serial.print("  Humedad = ");
		Serial.println(Humidity);
		//despliegue de voltaje y temperatura de la bateria
		//lee_bateria(); // no leer aqui porque si esta cargando lee el voltaje del cargador!
		Serial.print("Voltaje_bateria_drone = ");
		Serial.println(Voltaje_bateria_drone);
		Serial.println("---------------------------------------------- ");
	}


	// ********   Deteccion de FLANCO de subida y bajada  *******

	if (Auto_Manual != Auto_Manual_ant) { //Subida de Auto_manual
		Auto_Manual_ON = true;
		Auto_Manual_ant = Auto_Manual;
	}
	else //esta en manual
		Auto_Manual_ON = false;

	if (!Auto_Manual != Manual_ant) { //Bajada de Auto_Manual
		Manual_ON = true;
		Manual_ant = !Auto_Manual;
	}
	else //esta en manual
		Manual_ON = false;



	if (Remoto_Local != Remoto_Local_ant) {
		Remoto_Local_ON = true;
		Remoto_Local_ant = Remoto_Local;
	}
	else
		Remoto_Local_ON = false;


	if (!sistema_habilitado != sistema_deshabilitado_ant) { //Bajada de sistema_habilitado
		if (!sistema_habilitado) {
			sistema_deshabilitado_ON = true;
			Serial.println("************ Sistema Des-Habilitado ************* ");
		}
		sistema_deshabilitado_ant = !sistema_habilitado;
	}
	else //esta en manual
		sistema_deshabilitado_ON = false;


	if (sistema_habilitado != sistema_habilitado_ant) { //Bajada de sistema_habilitado
		if (sistema_habilitado) {
			sistema_habilitado_ON = true;
			Serial.println("************ Sistema Habilitado ************* ");
		}
		sistema_habilitado_ant = sistema_habilitado;
	}
	else //esta en manual
		sistema_habilitado_ON = false;


	//Reset de comandos al cambiar a Auto/Manual o Remoto/Local
	if (Auto_Manual_ON || Remoto_Local_ON || Manual_ON || sistema_deshabilitado_ON || sistema_habilitado_ON) {

		inicia_maquinas_de_estados();
		Reset_maquinas_DJI();

		reset_comandos();




	}



	//manejo del permisivo de despegue y bit de condiciones ambientales


	//cast a integer, proteccion de valores negativos y PONER multiplicador a las variables de referencia
	if (Temperature > 0)
		Temperatura_int = (int)(Temperature * 100);    // va MULTIPLICADO POR 100
	else
		Temperatura_int = 0;

	if (Humidity > 0)
		Humedad_int = (int)(Humidity * 100);           //va MULTIPLICADO POR 100
	else
		Humedad_int = 0;





	if (vel_viento > 0)
		vel_viento_int = (int)(vel_viento * 100);      //vel_viento esta MULTIPLICADO POR 100
	else
		vel_viento_int = 0;







	//Alarma de Voltaje bateria bajo
	if (Voltaje_bateria_int / 100 < (float)Voltaje_bateria_drone_OK)
		Voltaje_Bateria_bajo = true;
	else
		Voltaje_Bateria_bajo = false;

	//Alarma de Temperatura bateria Alta

	/*
	if (Temp_bateria_Celcius > Temperatura_bateria_drone_MAX && Temp_bateria_Celcius < Umbral_alto_Temp_bateria_Drone_ON)
		Temperatura_bateria_alta = true;
	else
		Temperatura_bateria_alta = false;


	//Alarma de Warning de condiciones de bateria 

	if (Warning_Temp_bateria || Warning_Voltaje_bateria)
		Warning_bateria = true;
	else
		Warning_bateria = false;


	if (habilita_Warning_Temperatura_bateria)
		Warning_Temp_bateria = Temperatura_bateria_alta;
	else
		Warning_Temp_bateria = false;
*/

	if (habilita_Warning_Voltaje_bateria)
		Warning_Voltaje_bateria = Voltaje_Bateria_bajo;
	else
		Warning_Voltaje_bateria = false;

	//if( Temperatura_int<= Temperatura_OK && Humedad_int<= Humedad_OK && vel_viento <= vel_viento_OK*100 && !Voltaje_Bateria_bajo)
	if (Temperature <= Temperatura_OK && Humidity <= Humedad_OK && vel_viento <= vel_viento_OK) //se quita el voltaje  porque no se puede medir el voltaje de la bateria, ademas de que las barras ya se abrieron.

		C_Ambientales_OK = true;
	else
		C_Ambientales_OK = false;

	if (Debug) { //Revisar
		C_Ambientales_OK = true;
		Warning_Voltaje_bateria = false;
		//*Warning_Temp_bateria = false;
	}

	//if (base_abierta && C_Ambientales_OK && !Warning_Voltaje_bateria && !Warning_Temp_bateria)
		if (base_abierta && C_Ambientales_OK && !Warning_Voltaje_bateria)
		Permisivo_despegue = true;
	else
		Permisivo_despegue = false;








	//*******************************************************************************************************************************************************************************************
	// **************************************************************************************************************************************************  CONTROL DE BAJO NIVEL   *********************
	//*******************************************************************************************************************************************************************************************

	//MANEJO DE MENSAJES


	//mensajes = 1 "Condiciones Ambientales no validas"
	//mensajes = 2 "La base NO esta cerrada. Busca el home"
	//mensajes = 3 "La base NO esta abierta. Se procede a abrirla"
	//mensajes = 4 "No se puede ejecutar comando. Sistema NO habilitado"


	/*
	if (Secuencia_ABRIR && !C_Ambientales_OK )
	  mensajes = 1;

	else if (Secuencia_ABRIR && !base_cerrada )
	  mensajes = 2;


	else if (Secuencia_CERRAR && !base_abierta )
	  mensajes = 3;

	else if ((Secuencia_CERRAR || Secuencia_ABRIR || Secuencia_HOME ) && !sistema_habilitado)
	  mensajes = 4;

	else if (Secuencia_ABRIR && Edo_maq_ABRIR >0)
	  mensajes = 5;

	else if (Secuencia_CERRAR && Edo_maq_CERRAR > 0)
	  mensajes = 6;

	else if (Secuencia_HOME && Edo_maq_HOME > 0)
	  mensajes = 7;
	*/
	if (ABRIR_Domo_falla)
		mensajes = 70;
	else if (CERRAR_Domo_falla)
		mensajes = 71;

	else if (ABRIR_PAD_falla)
		mensajes = 74;
	else if (CERRAR_PAD_falla)
		mensajes = 75;
	/*  ver maquina de estados de pruebas
	else if (Inicia  Rutina de Pruebas)
	mensajes = 76;
	else if (Termina Rutina de Pruebas)
	mensajes = 77;

	else
	mensajes = 0;
	  */
	  // *******************************************************************************************************************************************************BORRAR*****
	  // ************************************************* CONTROL BAJO NIVEL************************************************************************************BORRAR*****
	  // *******************************************************************************************************************************************************BORRAR*****


	

	  // +++ CONTROL DE Domo  +++


	  ///control de senales en manual o automatico. se abren o cierran las dos puertas con la misma senal

	  //ABRIR Domo

	if (((Remoto_Local && Auto_Manual && ABRIR_A) || (Remoto_Local && !Auto_Manual && ABRIR_M) || (!Remoto_Local && !Auto_Manual && ABRIR_L)) && !CERRAR)

		ABRIR = true;
	else
		ABRIR = false;


	//GENERACION DE comados al motor

		//if ((ABRIR || cmd_ABRIR_Domo) && !CERRAR && sistema_habilitado && !Domo_Abierto && !PE && !ABRIR_Domo_falla) {
	if ((ABRIR || cmd_ABRIR_Domo) && !CERRAR && sistema_habilitado && !Domo_Abierto && !PE) {

		if (!Comando_ABRIR_Domo_enviado) {
			Comando_ABRIR_Domo();
			Serial.println("Abriendo Domo"); //Revisar
			Comando_ABRIR_Domo_enviado = true;
		}
		cmd_ABRIR_Domo = true;
	}
	else
		cmd_ABRIR_Domo = false;





	//FALLA POR TIMEOUT
	if (cmd_ABRIR_Domo && !ABRIR_Domo_falla) {//timer[7]  
		//ABRIR_Domo_falla = false;

		timer[7].PSET = (unsigned long)Timeout_ABRIR_Domo;
		timer[7].EN = true;


		if (timer[7].OUT) {
			timer[7].EN = false;
			timer[7].PV = 0;
			ABRIR_Domo_falla = true;
			Serial.print("ABRIR_Domo_falla = ");
			Serial.println(ABRIR_Domo_falla);

		}
	}


	//RESET DE COMANDOS DE ABRIR  

	if (Domo_Abierto) {

		ABRIR = false;
		ABRIR_L = false;
		ABRIR_A = false;
		ABRIR_M = false;
		//*Mb.C[13] = ABRIR_M;
		ABRIR_Domo_falla = false;
		timer[7].EN = false;
		timer[7].PV = 0;
		Comando_ABRIR_Domo_enviado = false;




	}

	//CERRAR Domo

	if (((Remoto_Local && Auto_Manual && CERRAR_A) || (Remoto_Local && !Auto_Manual && CERRAR_M) || (!Remoto_Local && !Auto_Manual && CERRAR_L)) && !ABRIR)

		CERRAR = true;
	else
		CERRAR = false;

		//if ((CERRAR || cmd_CERRAR_Domo) && !ABRIR && sistema_habilitado && !Domo_Cerrado  && !PE && !CERRAR_Domo_falla) {
	if ((CERRAR || cmd_CERRAR_Domo) && !ABRIR && sistema_habilitado && !Domo_Cerrado && !PE) {

		if (!Comando_CERRAR_Domo_enviado) {
			Comando_CERRAR_Domo();
			Serial.println("Cerrando Domo"); //Revisar
			Comando_CERRAR_Domo_enviado = true;
		}
		cmd_CERRAR_Domo = true;
	}
	else
		cmd_CERRAR_Domo = false;




	//FALLA POR TIMEOUT
	if (cmd_CERRAR_Domo && !CERRAR_Domo_falla) {//timer[8]  
		//CERRAR_Domo_falla = false;

		timer[8].PSET = (unsigned long)Timeout_CERRAR_Domo;
		timer[8].EN = true;


		if (timer[8].OUT) {
			timer[8].EN = false;
			timer[8].PV = 0;
			CERRAR_Domo_falla = true;
			Serial.print("CERRAR_Domo_falla = ");
			Serial.println(CERRAR_Domo_falla);

		}
	}



	//RESET DE COMANDOS DE CERRAR

	if (Domo_Cerrado) {

		CERRAR = false;
		CERRAR_L = false;
		CERRAR_A = false;
		CERRAR_M = false;
		//*Mb.C[14] = CERRAR_M;
		CERRAR_Domo_falla = false;
		timer[8].EN = false;
		timer[8].PV = 0;
		Comando_CERRAR_Domo_enviado = false;


	}

	// *******************************************************************************************************************************************************BORRAR*****
	// +++ CONTROL DEL PAD  +++

	// *******************************************************************************************************************************************************BORRAR*****





	///control de senales en manual o automatico

	if (((Remoto_Local && Auto_Manual && PAD_open_A) || (Remoto_Local && !Auto_Manual && PAD_open_M) || (!Remoto_Local && !Auto_Manual && PAD_open_L)) && !PAD_close)
		PAD_open = true;
	else
		PAD_open = false;


	if (((Remoto_Local && Auto_Manual && PAD_close_A) || (Remoto_Local && !Auto_Manual && PAD_close_M) || (!Remoto_Local && !Auto_Manual && PAD_close_L)) && !PAD_open)
		PAD_close = true;
	else
		PAD_close = false;






	//GENERACION DE COMANDO cmd_PAD_open 

	  //if (PAD_open && !PAD_close && sistema_habilitado && !PAD_opened  &&  !PE) {  //SIN ENCLAVAMIENTO! para poder pararlo antes de llegar al sw de final de carrrera
	if (PAD_open && sistema_habilitado && !PE) {  //SIN ENCLAVAMIENTO! para poder pararlo antes de llegar al sw de final de carrrera

		if (!Comando_abrir_ejes_pad_enviado) {
			Comando_abrir_ejes_pad();
			Comando_abrir_ejes_pad_enviado = true;
		}
		cmd_PAD_open = true;
	}
	else {
		//Comando_abrir_ejes_pad_enviado = false;
		cmd_PAD_open = false;
	}

	//FALLA POR TIMEOUT
	if (cmd_PAD_open && !ABRIR_PAD_falla) {//timer[7]  
		//ABRIR_PAD_falla = false;

		timer[12].PSET = (unsigned long)Timeout_ABRIR_PAD;
		timer[12].EN = true;


		if (timer[12].OUT) {
			timer[12].EN = false;
			timer[12].PV = 0;
			ABRIR_PAD_falla = true;
			Serial.print("ABRIR_PAD_falla = ");
			Serial.println(ABRIR_PAD_falla);

		}
	}


	//GENERACION DE COMANDO cmd_PAD_close 

	 //if (PAD_close && !PAD_open && sistema_habilitado && !PAD_closed  &&  !PE) {    //SIN ENCLAVAMIENTO! para poder pararlo antes de llegar al sw de final de carrrera
	if (PAD_close && sistema_habilitado && !PE) {    //SIN ENCLAVAMIENTO! para poder pararlo antes de llegar al sw de final de carrrera

		if (!Comando_cerrar_ejes_pad_enviado) {
			Comando_cerrar_ejes_pad();
			Comando_cerrar_ejes_pad_enviado = true;
		}
		cmd_PAD_close = true;
	}
	else {
		//Comando_cerrar_ejes_pad_enviado = false;
		cmd_PAD_close = false;
	}

	//FALLA POR TIMEOUT
	if (cmd_PAD_close && !CERRAR_PAD_falla) {//timer[7]  
		//CERRAR_PAD_falla = false;

		timer[13].PSET = (unsigned long)Timeout_CERRAR_PAD;
		timer[13].EN = true;


		if (timer[13].OUT) {
			timer[13].EN = false;
			timer[13].PV = 0;
			CERRAR_PAD_falla = true;
			Serial.print("CERRAR_PAD_falla = ");
			Serial.println(CERRAR_PAD_falla);
		}
	}


	//RESET COMANDOS PAD
	if (PAD_opened) { //LOS EJES YA ABRIERON

		PAD_open = false;
		PAD_open_L = false;
		PAD_open_A = false;
		PAD_open_M = false;
		//*Mb.C[26] = PAD_open_M;
		ABRIR_PAD_falla = false;
		timer[12].EN = false;
		timer[12].PV = 0;
		Comando_abrir_ejes_pad_enviado = false;
	}
	if (PAD_closed) {  //LOS EJES YA CERRARON

		PAD_close = false;
		PAD_close_L = false;
		PAD_close_A = false;
		PAD_close_M = false;
		//*Mb.C[25] = PAD_close_M;
		CERRAR_PAD_falla = false;
		timer[13].EN = false;
		timer[13].PV = 0;
		Comando_cerrar_ejes_pad_enviado = false;
	}

	// *******************************************************************************************************************************************************BORRAR*****
	// *******************************************************************************************************************************************************BORRAR*****
	// *******************************************************************************************************************************************************BORRAR*****



	//INTER-LOCKS   de proteccion
	//NOTA:  El Eje Y debe moverse igual o  mas rapido que el cierre de Puertas!!!

	if (Remoto_Local) { // en Local no funcionan los interlocks

	}
	else  //esta en local
	{
		ABRIR_L = ABRIR_M;
		CERRAR_L = CERRAR_M;




		PAD_open_L = PAD_open_M;
		PAD_close_L = PAD_close_M;

	}



}

byte turno = 1;

void actualiza_posiciones() { //Los tengo que multiplexear porque sino el motor los ignora
	if (turno == 1) {
		Comando_read_multi_loop_Angle(1);
		turno = 2;
	}
	else if (turno = 2) {
		Comando_read_multi_loop_Angle(2);
		turno = 1;
	}


}


void ladder() {

	lee_entradas_PLC();
	procesa();
	escribe_salidas();

}

//**********************************************************************************************************************************************************************************************
// ***********************************************************************************************************************  MAQUINAS DE ESTADO  ************************************************
//**********************************************************************************************************************************************************************************************

void inicia_maquinas_de_estados() {
	Edo_maq_ABRIR = 0;
	Edo_maq_CERRAR = 0;
	Edo_maq_HOME = 0;

	   
	//Edo_maq_ENCENDER_TX = 0;  //muy importante para que corran en manual sin estar el sistema habilitado

}



void preproceso() { // se llama antes de las maquinas de estados



	if (PAD_closed) {//Solo se puede mandar CHARGE_ON si el pad esta cerrado!!

		if (Auto_Manual) {
			if (CHARGE_ON_Auto || CHARGE_ON_Carga_Auto) {
				CHARGE_ON = true;		// enciende CHARGE
				//*Mb.C[18] = CHARGE_ON;
			}
			else {
				CHARGE_ON = false;		// apaga CHARGE
				//*Mb.C[18] = CHARGE_ON;
			}
		}
		else {
			if (CHARGE_ON_Manual) {
				CHARGE_ON = true;		// enciende CHARGE
				//*Mb.C[18] = CHARGE_ON;
			}
			else {
				CHARGE_ON = false;		// apaga CHARGE
				//*Mb.C[18] = CHARGE_ON;
			}

		}

	}
	else {
		CHARGE_ON = false;		// apaga CHARGE
		CHARGE_ON_Manual = false;
		//*Mb.C[18] = CHARGE_ON;
	}


	if (Auto_Manual) {
		if (Ventilador_ON_Auto || Ventilador_ON_Carga_Auto) {
			Ventilador_ON = true;		// enciende ventilador
			//*Mb.C[39] = Ventilador_ON;
		}
		else {
			Ventilador_ON = false;		// apaga ventilador
			//*Mb.C[39] = Ventilador_ON;
		}
	}
	else {
		if (Ventilador_ON_Manual) {
			Ventilador_ON = true;		// enciende ventilador
			//*Mb.C[39] = Ventilador_ON;
		}
		else {
			Ventilador_ON = false;		// apaga ventilador
			//*Mb.C[39] = Ventilador_ON;
		}

	}

	   
	

}







void maquinas_de_estados() {


	preproceso();




	//--------------------------------------------------------------------------  Secuencia de HOME  en Automatico  -----------------------


	if (Edo_maq_HOME == 0) {  //no hace nada, espera el arranque de la maquina

		if ((Secuencia_HOME || Secuencia_HOME2) && !base_cerrada)
			Edo_maq_HOME = 1;
	}

	if (Edo_maq_HOME == 1) {

		//SE HABILITA AUTOMATICAMENTE EL SISTEMA!!!  Y YA NO LO DESHABILITO  NUNCA !!!!!
		//*Mb.C[10] = true; //Necesario para que Kepware no force el valor
		habilita_sistema();

		PAD_close_A = true; //Cierra el PAD

		if (PAD_closed) {// 
			Edo_maq_HOME = 2;
			Serial.print(" --------------------------Edo_maq_HOME =  ");
			Serial.println(Edo_maq_HOME);
		}
	}

	if (Edo_maq_HOME == 2) {  //Cierra el Domo
		PAD_close_A = false;
		CERRAR_A = true;
		if (Domo_Cerrado) {
			Edo_maq_HOME = 3;
			Serial.print(" --------------------------Edo_maq_HOME =  ");
			Serial.println(Edo_maq_HOME);
		}
			
	}

	if (Edo_maq_HOME == 3) {  //Cierra Puertas
		CERRAR_A = false;


		if (Domo_Cerrado && PAD_closed) {

		
			Edo_maq_HOME = 4;
			Serial.print(" --------------------------Edo_maq_HOME =  ");
			Serial.println(Edo_maq_HOME);
		}
	}

	if (Edo_maq_HOME == 4) {

		Secuencia_HOME = false;
		Secuencia_HOME2 = false;
		Serial.println("--------------------------Fin de Secuencia de HOME ");
		Serial.println("------------------------------------------------- ");

		/* // correccion 11 Junio 2020
						  //SE DESHABILITA AUTOMATICAMENTE EL SISTEMA!!!
			  //*Mb.C[10] = false; //Necesario para que Kepware no force el valor
			  deshabilita_sistema ();
		*/

		if ((true))
			Edo_maq_HOME = 0;
	}






	//  --------------------------------------------------------------------   Secuencia de ABRIR en Automatico -------------------


	if (Edo_maq_ABRIR == 0) {  //No hace nada, espera el arranque de la maquina

//reset de timers de la secuencia
	//Usa los timers:
	//timer[9]
	//timer[6]
	
		timer[9].EN = false;
		timer[9].PV = 0;
		timer[6].EN = false;
		timer[6].PV = 0;

	 //if(Secuencia_ABRIR && C_Ambientales_OK && !Voltaje_Bateria_bajo &&  base_cerrada && Auto_Manual && Edo_maq_CERRAR == 0 && Edo_maq_HOME == 0 && !base_abierta) { //si ya esta abierta, se ignora el comando
		 //if (Secuencia_ABRIR && C_Ambientales_OK  && base_cerrada && Auto_Manual && Edo_maq_CERRAR == 0 && Edo_maq_HOME == 0 && !base_abierta) { //si ya esta abierta, se ignora el comando
			 //if ((Secuencia_ABRIR || Secuencia_ABRIR2) && C_Ambientales_OK && base_cerrada && Auto_Manual && Edo_maq_CERRAR == 0 && Edo_maq_HOME == 0 && !base_abierta) { //si ya esta abierta, se ignora el comando
		//if ((Secuencia_ABRIR || Secuencia_ABRIR2) && C_Ambientales_OK && !Warning_bateria && base_cerrada && Auto_Manual && Edo_maq_CERRAR == 0 && Edo_maq_HOME == 0 && !base_abierta) { //si ya esta abierta, se ignora el comando
		if ((Secuencia_ABRIR || Secuencia_ABRIR2) && base_cerrada && Auto_Manual && Edo_maq_CERRAR == 0 && Edo_maq_HOME == 0 && !base_abierta) { //si ya esta abierta, se ignora el comando

			if (bandera_TX_auto) {
				//Secuencia_Encender_TX_Auto = true;  //se activa  la secuencia de encendido del TX
				Edo_maq_ABRIR = 13;
				Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
				Serial.print(Edo_maq_ABRIR);
				Serial.println("  Encendiendo Transmisor  ");
			}
			else {
				Edo_maq_ABRIR = 1;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.print(Edo_maq_ABRIR);
			Serial.println("  Abrir Domo  ");
			}
			


		}
		//if (Secuencia_ABRIR && C_Ambientales_OK && !Warning_bateria && Auto_Manual && !base_cerrada) {  //delay y se resetea el comando de abrir
		if (Secuencia_ABRIR && Auto_Manual && !base_cerrada) {  //delay y se resetea el comando de abrir
			Edo_maq_ABRIR = 11;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.println(Edo_maq_ABRIR);
		}

		if (Secuencia_ABRIR && base_abierta)  //si ya esta abierta (Domo+abierto && PAD_Opened), se ignora el comando y se resetea el comando de abrir
			Secuencia_ABRIR = false;
	}


	if (Edo_maq_ABRIR == 13) {  //se queda esperando que acabe la secuencia d encendido de transmisor 
		if (true) { //Esta senal se apaga al final de la secuencia de encendido/apagado de TX
			Edo_maq_ABRIR = 1;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.println(Edo_maq_ABRIR);
		}
	}


	if (Edo_maq_ABRIR == 11) { //solo un delay pero no activa la secuenci de abrir porque la base esta abierta
		//timer[9]   
		timer[9].AST = false; //Timer 10  NO  astable 
		timer[9].PSET = 5;
		timer[9].EN = true;


		if (timer[9].OUT) {
			timer[9].EN = false;
			timer[9].PV = 0;

			Secuencia_ABRIR = false; //se resetea el comando de abrir
			//*Mb.C[19] = false; //Necesario para que Kepware no force el valor

			Secuencia_ABRIR2 = false;
			Edo_maq_ABRIR = 0;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.println(Edo_maq_ABRIR);


		}

	}






	//Aqui empieza propiamente la secuencia de ABRIR.  La base esta cerrada

	if (Edo_maq_ABRIR == 1) {  // Abrir Puertas




	   //SE HABILITA AUTOMATICAMENTE EL SISTEMA!!!  Y YA NO LO DESHABILITO  NUNCA !!!!!
		//*Mb.C[10] = true; //Necesario para que Kepware no force el valor
		habilita_sistema();

		ABRIR_A = true;        //Abre las puertas


		if (Domo_Abierto) { //a la mitad de puertas abiertas  ya se sale para empezar a encender DRONE
			Edo_maq_ABRIR = 2;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.println(Edo_maq_ABRIR);
		}
	}

	if (Edo_maq_ABRIR == 2) {   //continua abriendo las puertas y Cerrando PAD (si estaba abierto)


		if (Domo_Abierto && PAD_closed) {//se queda esperando a base abierta =   Puertas_abiertas
			Edo_maq_ABRIR = 3;

			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.print(Edo_maq_ABRIR);
			Serial.println("  Encendiendo Drone  ");
		}
	}

	if (Edo_maq_ABRIR == 3) {  //dispara la maquina de encender DRONE
		ABRIR_A = false;


		//Secuencia_Encender_DRONE_Auto = true; //dispara la maquina de encender DRONE
		   ////*Mb.C[12] = Secuencia_Encender_DRONE; //necesario


			 //timer[6]   delay de 1 Seg)
		timer[6].AST = false; //Timer 6  NO  astable
		timer[6].PSET = 1; //Timer[6] duracion  1 Seg
		timer[6].EN = true;

		if (timer[6].OUT) {
			timer[6].EN = false;
			timer[6].PV = 0;
			Edo_maq_ABRIR = 40;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.println(Edo_maq_ABRIR);

		}


	}

	if (Edo_maq_ABRIR == 40) { //espera a que termine la maquina de encender/apagar DRONE

		if (true) {
		Edo_maq_ABRIR = 4;
		Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
		Serial.print(Edo_maq_ABRIR);
		Serial.println("  Abrir PAD  ");
		}
			

	}



	if (Edo_maq_ABRIR == 4) { //Abre los ejes

		PAD_open_A = true;   //abre centrador  



		if (PAD_opened) {//Se queda esperando a que abra los ejes
			Edo_maq_ABRIR = 5;
			Serial.print("--------------------------------------------Edo_maq_ABRIR =  ");
			Serial.println(Edo_maq_ABRIR);
		}
	}

	if (Edo_maq_ABRIR == 5) { //Fin de secuencia. 

		Permisivo_despegue = true;
		PAD_open_A = false;

		Secuencia_ABRIR = false;
		//*Mb.C[19] = false; //Necesario para que Kepware no force el valor

		Secuencia_ABRIR2 = false;


		if (true) {
			Ciclos++;  //incremente el numero de ciclos
			int eeAddress = 50;
			EEPROM.put(eeAddress, Ciclos); //lo escribe al EEPROM

			Edo_maq_ABRIR = 0;
			Serial.println("------------------------------------------- Fin de Secuencia ABRIR ");
			Serial.println("------------------------------------------------------------------- ");
		}


	}



	// -----------------------------------------------------------------------------------------------------------------  Secuencia de CERRAR en Automatico  ------------------------------


	if (Edo_maq_CERRAR == 0) {  //no hace nada, espera el arranque de la maquina

		//reset de timers de la secuencia
	//Usa los timers:
	//timer[1]
	//timer[2]
	//timer[10]
		timer[1].EN = false;
		timer[1].PV = 0;
		timer[1].PSET = (unsigned long)tiempo_enfriado; //timer[1] duracion  configurable

		timer[2].EN = false;
		timer[2].PV = 0;
		timer[2].PSET = (unsigned long)tiempo_carga; //timer[2] duracion  configurable

		timer[10].EN = false;
		timer[10].PV = 0;
		timer[10].PSET = (unsigned long)tiempo_video + 6; //timer[10] duracion  configurable + 6 segs de delay extras


		if (Secuencia_CERRAR && Auto_Manual && !base_abierta && Edo_maq_ABRIR == 0 && Edo_maq_HOME == 0 && !base_cerrada) {

		Edo_maq_CERRAR = 1; //se va a checar puertas abiertas, aunque normalmente deberian estar abiertas!
		Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
		Serial.println(Edo_maq_CERRAR);
		}

		  //if(Secuencia_CERRAR && Auto_Manual && base_abierta && Edo_maq_ABRIR == 0 && Edo_maq_HOME == 0 && ! base_cerrada)  //Si esta cerrada se ignora el comando
		if ((Secuencia_CERRAR || Secuencia_CERRAR2) && Auto_Manual && base_abierta && Edo_maq_ABRIR == 0 && Edo_maq_HOME == 0 && !base_cerrada) { //Si esta cerrada se ignora el comando

			Edo_maq_CERRAR = 4; //se va a directo a mover eje Y para centrar el drone!
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.println(Edo_maq_CERRAR);
		}
	

		if (Secuencia_CERRAR && base_cerrada)  //si ya esta cerrada, se ignora el comando y se resetea el comando de cerrar
			Secuencia_CERRAR = false;
	}


	//asegura que este abierta!!

	if (Edo_maq_CERRAR == 1) { //Abre Puertas

		   //SE HABILITA AUTOMATICAMENTE EL SISTEMA!!! 
		//*Mb.C[10] = true; //Necesario para que Kepware no force el valor
		habilita_sistema();

		ABRIR_A = true;

		if (Domo_Abierto) {      //solo checa Domo, no le importa donde este el elevador!    
			Edo_maq_CERRAR = 2;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.println(Edo_maq_CERRAR);
		}
	}






	if (Edo_maq_CERRAR == 2) { //Abre Eje X, Abre eje Y y Sube elevador



		ABRIR_A = false;
		PAD_open_A = true;        //Asegura que el centrador eje X este abierto





		if (PAD_opened) {
	
			Edo_maq_CERRAR = 3;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.println(Edo_maq_CERRAR);
		}

	}

	if (Edo_maq_CERRAR == 3) {

		PAD_open_A = false;

		if (true)
			Edo_maq_CERRAR = 4;
		Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
		Serial.print(Edo_maq_CERRAR);
		Serial.println("  Cerrar PAD  ");

	}


	//Aqui empieza propiamente la secuencia de CERRAR, la base esta Abierta!

	if (Edo_maq_CERRAR == 4) { //Cierra los ejes

		// SE QUITA    Encender_Drone = false;//Se asefura que este en cero la  se;al al micro que enciende / apaga el drone POR SI LAS DUDAS!!  //REVISAR



		PAD_close_A = true;
		if (PAD_closed) {     // Cierra ejes para centrar y sujetar  el drone
			Edo_maq_CERRAR = 7;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.println(Edo_maq_CERRAR);
		}
	}



	if (Edo_maq_CERRAR == 7) {  //nada

		PAD_close_A = false;

		Edo_maq_CERRAR = 8;
		Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
		Serial.print(Edo_maq_CERRAR);
		Serial.println("  Cerrar Domo  ");
	}

	if (Edo_maq_CERRAR == 8) {  //Cierra domo
		Cargador_ON = false;//regresa   //se desergiza el cargador para que no haya energia en los contactos del PAD mientras esta en la secuencia de cierre
		CERRAR_A = true;

		if (base_cerrada) {
			Edo_maq_CERRAR = 9;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");

			Serial.print(Edo_maq_CERRAR);
			Serial.println("  Descargando video  ");
		}
			
	}

	if (Edo_maq_CERRAR == 9) { // delay para bajar video
		//Serial.println("Edo_maq_CERRAR = 9");
		Cargador_ON = false;//regresa   //se desergiza el cargador para que no haya energia en los contactos del PAD mientras esta en la secuencia de cierre

		CERRAR_A = false;

		//timer[10]   
		timer[10].AST = false; //Timer 10  NO  astable 
		timer[10].PSET = (unsigned long)tiempo_video + 6; //timer[10] duracion  configurable + 6 segs de delay extras

		timer[10].EN = true;


		if (timer[10].OUT) {
			
			timer[10].EN = false;
			timer[10].PV = 0;
			Edo_maq_CERRAR = 10;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.print(Edo_maq_CERRAR);
			Serial.println("  Apagando Drone  ");


		}

	}

	if (Edo_maq_CERRAR == 10) { //apagar Drone
		//Serial.println("Edo_maq_CERRAR = 10");



  //APAGA EL DRONE
		//Secuencia_Encender_DRONE_Auto = true; //dispara la maquina de encender/apagar DRONE
		   ////*Mb.C[12] = Secuencia_Encender_DRONE; //necesario



		if (true)
			Edo_maq_CERRAR = 40;
		Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
		Serial.println(Edo_maq_CERRAR);

	}

	if (Edo_maq_CERRAR == 40) { //espera a que termine la maquina de encender DRONE

		if (true) {
			Edo_maq_CERRAR = 18;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.println(Edo_maq_CERRAR);
		}
			

	}

	if (Edo_maq_CERRAR == 18) { //se activa la secuencia de apagado  del TX

		Cargador_ON = false;//regresa   //se desergiza el cargador para que no haya energia en los contactos del PAD mientras esta en la secuencia de cierre


		if (bandera_TX_auto) {

			//Secuencia_Encender_TX_Auto = true;
			Edo_maq_CERRAR = 19;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.print(Edo_maq_CERRAR);
			Serial.println("  Apagando Transmisor  ");
		}
		else {
			Edo_maq_CERRAR = 20;

			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.print(Edo_maq_CERRAR);
			Serial.println("  Enfriamiento de Drone  ");
		}
			


	}

	if (Edo_maq_CERRAR == 19) {  //se queda esperando que acabe la secuencia d encendido de transmisor 
		if (true) {//Esta senal se apaga al final de la secuencia de encendido/apagado de TX
			Edo_maq_CERRAR = 20;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");

			Serial.print(Edo_maq_CERRAR);
			Serial.println("  Enfriamiento de Drone  ");
		}
	}

	if (Edo_maq_CERRAR == 20) { // Inicia tiempo de enfriado del drone  //ya aqui el drone ya esta apagado..

		Cargador_ON = false;//regresa   //se desergiza el cargador para que no haya energia en los contactos del PAD ni 5 V en el cable blanco  y no haya carga lenta mientras enfria


	
		Ventilador_ON_Auto = true;
		Comando_DRONE_CHARGE_enviado = false;

		////*Mb.C[10] = false; //Necesario para que Kepware no force el valor
		//deshabilita_sistema();  NO LLAMARLO AQUI porque reseea las maquinas de estado!

	 //timer[1]   
		timer[1].AST = false; //Timer 10  NO  astable 
		timer[1].PSET = (unsigned long)tiempo_enfriado; //timer[1] duracion  configurable
		timer[1].EN = true;


		if (timer[1].OUT) {
			
			timer[1].EN = false;
			timer[1].PV = 0;
			Edo_maq_CERRAR = 21;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");

			Serial.print(Edo_maq_CERRAR);
			Serial.println("  Carga de Drone  ");
		}

	}



	if (Edo_maq_CERRAR == 21) { //Inicia tiempo de carga   Tiempo de carga del drone

   // esto sirve para la llamada externa del ciclo de carga desde el HMI a cierta ahora SE ASUME QUE EL SISTEMA ESTA E AUTOMATICO
				  //SE HABILITA AUTOMATICAMENTE EL SISTEMA!!!  
		//*Mb.C[10] = true; //Necesario para que Kepware no force el valor
		habilita_sistema();
		Ventilador_ON_Auto = true;

		Cargador_ON = true;//regresa   //se energiza el cargador para que  haya energia en los contactos del PAD y 5 V en el cable blanco  mientras carga
		CHARGE_ON_Auto = true;

		mensajes = 78;   //manda codigo de inicio de carga al HMI




	 //timer[2]   
		timer[2].AST = false; //Timer 11  NO  astable 
		timer[2].PSET = (unsigned long)tiempo_carga; //timer[2] duracion  configurable
		timer[2].EN = true;


		if (timer[2].OUT) {
			
			timer[2].EN = false;
			timer[2].PV = 0;
			Comando_DRONE_CHARGE_enviado = false;
			Edo_maq_CERRAR = 22;
			Serial.print("--------------------------------------------Edo_maq_CERRAR =  ");
			Serial.println(Edo_maq_CERRAR);

		}

	}


	if (Edo_maq_CERRAR == 22) {  //Fin de secuencia de CERRAR

		//Serial.println("Edo_maq_CERRAR = 22");
	   //vuelvo a mandar el comando de DRONE_CHARGE
		CHARGE_ON_Auto = false;
		mensajes = 79;   //mada codigo de fin de carga al HMI

		Secuencia_CERRAR = false;
		//*Mb.C[20] = false; //Necesario para que Kepware no force el valor

		Secuencia_CERRAR2 = false;

		//*Mb.C[10] = false; //Necesario para que Kepware no force el valor
		deshabilita_sistema(); //Se apagan comandos y se reinician las maquinas de estado


		Ventilador_ON_Auto = false; //apaga ventilador

		Serial.println("-------------------------------------------------Fin de Secuencia de CERRAR ");
		Serial.println("-------------------------------------------------------------------------------------------------- ");

		if (true)
			Edo_maq_CERRAR = 0;
	}







	// ----------------------------------------------------------  Secuencia de leer voltaje y temperatura de bateria  ------------------------------



	// ----- aqui inicia la maquina de leer bateria -----

	if (Edo_maq_lee_bateria == 0) {

		//reset de timers de la secuencia
//Usa los timers:
		//timer[19]
		timer[19].EN = false;
		timer[19].PV = 0;


		//if (PAD_closed && Edo_maq_ENCENDER_DRONE == 0 && Edo_maq_PEINE_ON < 2) {//se activa cuando se cierra el pad y no se estan manipulando las senales de control de la tarjeta de carga
			if (PAD_closed && Cargador_ON && CHARGE_ON ) {//se activa cuando se cierra el pad y no se estan manipulando las senales de control de la tarjeta de carga

			Edo_maq_lee_bateria = 1;
			//Serial.print("Edo_maq_lee_bateria =  ");
			//Serial.println(Edo_maq_lee_bateria);
			}
			if (PAD_opened || (PAD_closed && !CHARGE_ON && !Cargador_ON)) {//cuando esta abierto o cerrdo sin carga
							//se leen los valores de voltaje y temp de la bateria
				
				Edo_maq_lee_bateria = 4;
				previousMillis_BATERIA = millis();  //necesario para el siguiente delay
				//Serial.print("Edo_maq_lee_bateria =  ");
				//Serial.println(Edo_maq_lee_bateria);
			}
	}


	if (Edo_maq_lee_bateria == 1) { //delay inicial antes de leer bateria


		//timer[19]   tiempo (60 Seg)
		timer[19].AST = false; //Timer 1  NO  astable
		timer[19].PSET = 5 * 60; //Timer[19] duracion  5 X 60 = 5 mins
		timer[19].EN = true;


		if (timer[19].OUT) {
			timer[19].EN = false;
			timer[19].PV = 0;

			Edo_maq_lee_bateria = 2;
			 //Serial.print("Edo_maq_lee_bateria =  ");
			 //Serial.println(Edo_maq_lee_bateria);
			previousMillis_BATERIA = millis();  //necesario para el siguiente delay
		}

		if (PAD_opened || (PAD_closed && !CHARGE_ON && !Cargador_ON)) {//lo saco del timer

			Edo_maq_lee_bateria = 0;

		}


	}


	if (Edo_maq_lee_bateria == 2) { // quita energia por 10 segs
		if (Cargador_ON) {
			Cargador_ON_activo = true;
			Cargador_ON = false;

		}


		currentMillis_BATERIA = millis();
		if (currentMillis_BATERIA - previousMillis_BATERIA >= limiteMillis_BATERIA) { //15 segs
			Edo_maq_lee_bateria = 3;
			
			//se leen los valores de voltaje y temp de la bateria
			if (!Cargador_ON)
				lee_bateria(); //Cargador_ON y va = 0 en esta llamada
			//Serial.print("Edo_maq_lee_bateria =  ");
			//Serial.println(Edo_maq_lee_bateria);
			previousMillis_BATERIA = millis();  //necesario para el siguiente delay
		}
	}


	if (Edo_maq_lee_bateria == 3) { //delay antes de volver a energizar la carga

			if (Cargador_ON) {
				Cargador_ON_activo = true;
				Cargador_ON = false;

			}
		currentMillis_BATERIA = millis();
		if (currentMillis_BATERIA - previousMillis_BATERIA >= 5000) { //5 segs
			Edo_maq_lee_bateria = 5;



			//Serial.print("Edo_maq_lee_bateria =  ");
			//Serial.println(Edo_maq_lee_bateria);
		}
	}


	if (Edo_maq_lee_bateria == 4) { //delay antes de volver a leer


		currentMillis_BATERIA = millis();
		if (currentMillis_BATERIA - previousMillis_BATERIA >= 5000) { //5 segs
			Edo_maq_lee_bateria = 5;
			if (!Cargador_ON)
				lee_bateria(); //Cargador_ON ya va = 0 en esta llamada


			//Serial.print("Edo_maq_lee_bateria =  ");
			//Serial.println(Edo_maq_lee_bateria);
		}
	}



	if (Edo_maq_lee_bateria == 5) {

		//Cargador_ON = false;

		if (PAD_closed) {

			if (Cargador_ON_activo) {
				Cargador_ON_activo = false;

				 Cargador_ON = true;

			}


		}




		Edo_maq_lee_bateria = 0;
		 //Serial.print("Edo_maq_lee_bateria =  ");
		// Serial.println(Edo_maq_lee_bateria);



	}


}

void display_bateria() {

	//Serial.println("");
	//Serial.println("---------------- lee bateria ------------------------------ ");
	//Serial.print("CHARGE_ON: ");
	//Serial.println(CHARGE_ON);
	//Serial.print("Encender_Drone: ");
	//Serial.println(Encender_Drone);
	//Serial.print("Cargador_ON: ");
	//Serial.println(Cargador_ON);

	Serial.println("");
	Serial.println("---------------------------------------------- ");
	Serial.print("Voltaje_bateria_drone = ");
	Serial.println(Voltaje_bateria_drone);
	//Serial.print("Voltaje_bateria_drone_raw = ");
	//Serial.println(Voltaje_bateria_drone_raw);



}

void lee_bateria() {
	// Manejo de Voltaje de bateria
	Voltaje_bateria_drone_raw = analogRead(voltaje_bateria_input);
	Voltaje_bateria_drone = Voltaje_bateria_drone_raw * 2.0 * 1.0355;



	if (Voltaje_bateria_drone / 100 > 20.0) //truco
		Voltaje_bateria_drone = Voltaje_bateria_drone_raw * 2.0 * 1.0355;

	//asignacion a la variable a desplegar en HMI
	Voltaje_bateria_int = (int)(Voltaje_bateria_drone * 1.0); // Voltaje_bateria_drone esta  MULTIPLICADO POR 100


	

	if (PAD_closed  && CHARGE_ON){ //los valores se mandan al pto serie solo si esta cerrado el PAD y cargando..

		display_bateria();

	}

}
////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t conv_2_int32(uint64_t Angle_long) {

	return (int32_t)Angle_long / 100 / 6;
}

void decodifica_status_motor() {



	unsigned i;
	int64_t Angle_long;
	int32_t Angle;


	if (status_motor[0] == 0x3E) {

		if (status_motor[1] == 0x30) { //Read PID parameters command

			Serial.println("Read PID parameters command response: ");
			for (i = 0; i <= 11; i++) {

				Serial.print(status_motor[i], HEX);
			}
			Serial.println();
			Serial.print("motor: "); Serial.println(status_motor[2]);

			//  parametros PID
			angleKp = status_motor[5];
			angleKi = status_motor[6];
			speedKp = status_motor[7];
			speedKi = status_motor[8];
			torqueKp = status_motor[9];
			torqueKi = status_motor[10];

			Serial.print("angleKp ");
			Serial.println(angleKp);
			Serial.print("angleKi ");
			Serial.println(angleKi);

			Serial.print("speedKp ");
			Serial.println(speedKp);
			Serial.print("speedKi ");
			Serial.println(speedKi);
/*
			Serial.print("torqueKp ");
			Serial.println(torqueKp);
			Serial.print("torqueKi ");
			Serial.println(torqueKi);
*/
		}


		if (status_motor[1] == 0x90) { //Read encoder command

			Serial.println("Read encoder command response: ");
			for (i = 0; i <= 11; i++) {

				Serial.print(status_motor[i], HEX);
			}
			Serial.println();
			Serial.print("motor: "); Serial.println(status_motor[2]);

			//  ENCODER POSITON = ORIGINAL ENCODER + OFFSET
			encoderMSB = status_motor[6];
			encoderLSB = status_motor[5];
			encoder = (encoderMSB << 8 | encoderLSB);
			//Serial.print("encoder: ");
			//Serial.print("MSB: ");
			//Serial.print(encoderMSB, HEX);
			//Serial.print(" LSB: ");
			//Serial.print(encoderLSB, HEX);
			Serial.print("   encoder decimal= : ");
			Serial.println(encoder, DEC);

			// ORIGINAL ENCODER POSITON
			encoder_originalMSB = status_motor[8];
			encoder_originalLSB = status_motor[7];
			encoder_original = (encoder_originalMSB << 8 | encoder_originalLSB);
			//Serial.print("encoder ORIGINAL POSITION = : ");
			//Serial.print("MSB: ");
			//Serial.print(encoder_originalMSB, HEX);
			//Serial.print(" LSB: ");
			//Serial.print(encoder_originalLSB, HEX);
			Serial.print("   encoder ORIGINAL POSITION decimal= : ");
			Serial.println(encoder_original, DEC);

			// OFFSET ENCODER POSITON
			encoder_offsetMSB = status_motor[10];
			encoder_offsetLSB = status_motor[9];
			encoder_offset = (encoder_offsetMSB << 8 | encoder_offsetLSB);
			//Serial.print("encoder OFFSET POSITION = : ");
			//Serial.print("MSB: ");
			//Serial.print(encoder_offsetMSB, HEX);
			//Serial.print(" LSB: ");
			//Serial.print(encoder_offsetLSB, HEX);
			Serial.print("  encoder OFFSET  DECIMAL= : ");
			Serial.println(encoder_offset, DEC);

			switch (status_motor[2]) {
			case 1:
				encoder1 = encoder;
				encoder_original1 = encoder_original;
				encoder_offset1 = encoder_offset;
				Serial.print("Encoder 1: ");
				Serial.println(encoder1, DEC);

				break;

			case 2:

				encoder2 = encoder;
				encoder_original2 = encoder_original;
				encoder_offset2 = encoder_offset;
				Serial.print("Encoder 2: ");
				Serial.println(encoder2, DEC);
				break;

			}
		}


		if (status_motor[1] == 0x92) { //Read multi-loop Angle command

			//Serial.print("Read multi_loop Angle RESPONSE ");




			//Get the 64 bit  Multi turn Angle
			Angle_long = 0;
			for (i = 12; i >= 5; i--) {
				Angle_long = (Angle_long << 8) | status_motor[i];
			}
			Angle = conv_2_int32(Angle_long);

			switch (status_motor[2]) {
			case 1:

				posicion_Domo = abs(Angle); // Saco valor absoluto porque los valores son negativos
				posicion_Domo2 = Angle;
				if (Debug) {
					Serial.print("Angle_Domo: ");
					Serial.println(Angle, DEC);
				}

				break;

			case 2:
				posicion_PAD = abs(Angle); // Saco valor absoluto porque los valores son negativos
				posicion_PAD2 = Angle;
				if (Debug) {
					Serial.print("Angle_PAD: ");
					Serial.println(Angle, DEC);
				}
				break;
			}

		}

	}


	
}


/*
void lee_status_motor() {

	byte dato;
	unsigned int i;

	//char dato_str[4] = { 0 };

	if (RS485.available()) {
		dato = RS485.read();

		   // Hexadecimal representation
		Serial.print("HEX: ");
		Serial.print(dato, HEX);


		if (dato == 0x3E){// dato = 0x3E inicio de status array
			index = 0;
			status_motor[index] = dato;
			index++;

		}
		else{

			status_motor[index] = dato;
			index++;
			//Deteccion de fin de respuesta de comando
			if (index >= 11) {

				Serial.print(" Comando recibido:  ");
				for (i = 0; i <= 11; i++){

				  Serial.print(status_motor[i],HEX);  // Print received byte in Hexadecimal representation
				}
				Serial.println();

				decodifica_status_motor();
				index = 0;
				//for (i = 0; i <= 7; i++) //limpia status array
				//	status_motor[i] = 0x00;
			}

		}

	}
}
*/









/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*** Lee Comando desde la terminal ***//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void lee_comando() {
	int result = 0;
	int incomingboolean = 0;         // incoming serial boolean
	char comando = ' ';
	int i = 0;


	// if we get a valid boolean, read boolean
	if (Serial.available() > 0) {
		// get incoming boolean:
		incomingboolean = Serial.read();

		Serial.print("Comando: ");
		comando = (char)incomingboolean;
		Serial.println(comando);




		if (comando == 'A') {
			Auto_Manual = true;
			//*Mb.C[21] = true; //Necesario para que Kepware no force el valor    
			Serial.println("Modo Automatico");
		}

		if (comando == 'a') {
			actualizar_Parametros = true;
			Serial.println("Comando actualizar_Parametros al EEPROM");
		}


		if (comando == 'B') { //Back PAD  Abrir PAD
			Serial.println("PAD_open");
			PAD_open_L = true;
			PAD_close_L = false;

			Comando_abrir_ejes_pad();
			Serial.println("abrir_ejes_pad");
		}

		if (comando == 'b') {  //abrir seguro domo //aqui
			Serial.println("seguro_domo_open");

		}

		if (comando == 'C') {
			Serial.println("Cerrar Domo");
			CERRAR_L = true;
			ABRIR_L = false;

			Comando_CERRAR_Domo();
			//Torque_control(2, vel_PAD);

		}
		if (comando == 'c'){
			Serial.println("Secuencia de CERRAR en Automatico ");
			Secuencia_CERRAR2 = true;
		}


		if (comando == 'D') {
			Debug = true;
			Serial.println("Debug habilitado");
		}
		if (comando == 'd') {
			Debug = false;
			Serial.println("Debug Deshabilitado");

		}
		/*
		if (comando == 'E') {
			Serial.println("---------------------------------------------- ");
			Serial.print("Edo_maq_ABRIR = ");
			Serial.println(Edo_maq_ABRIR);
			Serial.print("Edo_maq_CERRAR = ");
			Serial.println(Edo_maq_CERRAR);



			Serial.print("Edo_maq_HOME = ");
			Serial.println(Edo_maq_HOME);



			Serial.print("Edo_maq_lee_bateria = ");
			Serial.println(Edo_maq_lee_bateria);


			Serial.print("Secuencia_ABRIR = ");
			Serial.println(Secuencia_ABRIR);

			Serial.print("Secuencia_CERRAR = ");
			Serial.println(Secuencia_CERRAR);



			Serial.println("---------------------------------------------- ");




		}
*/
		if (comando == 'F') {  //Front PAD  Cerrar PAD
			Serial.println("PAD_close");
			PAD_open_L = false;
			PAD_close_L = true;

			Comando_cerrar_ejes_pad();
			Serial.println("cerrar_ejes_pad");
		}
/*
		if (comando == 'f') {  //cerrar seguro domo //aqui

		}

		if (comando == 'G') { //////////////////  DEBUG     ///////////////////

			//Serial.println("");
			//Serial.println("---SENSORES DE POSICION VIRTUALES----- ");

			Serial.print("Domo_ABIERTO= ");
			Serial.println(Domo_Abierto, BIN);

			Serial.print("Domo_CERRADO = ");
			Serial.println(Domo_Cerrado, BIN);

			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.print("posicion_Domo = ");
			Serial.println(posicion_Domo);

			Serial.print("posicion_PAD  = ");
			Serial.println(posicion_PAD);
			Serial.println("");

			Serial.print("  PAD_closed = ");
			Serial.println(PAD_closed, BIN);

			Serial.print("  PAD_opened = ");
			Serial.println(PAD_opened, BIN);
		}
*/
/*	
			Serial.println("");
			Serial.println("---------------------------------------------- ");

			Serial.println("---SENSORES ANALOGICOS----- ");
			Serial.println("");
			Serial.print("Temperatura_int= ");
			Serial.println(Temperature);
			Serial.print("  Humedad_int= ");
			Serial.println(Humidity);
			Serial.print("  vel_viento = ");
			Serial.println(vel_viento / 100);
			Serial.print("  vel_viento_int = ");
			Serial.println(vel_viento_int);
			Serial.print("  Voltaje_bateria_drone = ");
			Serial.println(Voltaje_bateria_drone);
			Serial.print("  Voltaje_bateria_int = ");
			Serial.println(Voltaje_bateria_int);
			Serial.print("Temp_bateria: ");
			Serial.print(Temp_bateria_Celcius);
			Serial.println(" C");


			Serial.print("  Voltaje_bateria_drone_OK = ");
			Serial.println(Voltaje_bateria_drone_OK);
			Serial.print("  Temperatura_bateria_drone_MAX = ");
			Serial.println(Temperatura_bateria_drone_MAX);



			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.println("");
			Serial.print("  Warning_bateria = ");
			Serial.println(Warning_bateria);
			Serial.print("  Warning_Temp_bateria = ");
			Serial.println(Warning_Temp_bateria);
			Serial.print("  Warning_Voltaje_bateria = ");
			Serial.println(Warning_Voltaje_bateria);
			Serial.print("  Temperatura_bateria_alta = ");
			Serial.println(Temperatura_bateria_alta);
			Serial.print("  Voltaje_Bateria_bajo = ");
			Serial.println(Voltaje_Bateria_bajo);

			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.println("");
			Serial.print("  habilita_Warning_Temperatura_bateria = ");
			Serial.println(habilita_Warning_Temperatura_bateria);
			Serial.print("  habilita_Warning_Voltaje_bateria = ");
			Serial.println(habilita_Warning_Voltaje_bateria);



		}
*/
/*
		if (comando == 'g') { //////////////////  DEBUG     ///////////////////


			Serial.print("sistema_habilitado = ");
			Serial.println(sistema_habilitado);
			Serial.print("Auto_Manual ");
			Serial.println(Auto_Manual);
			Serial.print("Remoto_Local ");
			Serial.println(Remoto_Local);
			Serial.print("PE = ");
			Serial.println(PE);
			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.print("Auto_Manual_ON ");
			Serial.println(Auto_Manual_ON);
			Serial.print("Remoto_Local_ON ");
			Serial.println(Remoto_Local_ON);
			Serial.println("");
			Serial.println("---------------------------------------------- ");
}

			Serial.print("ABRIR = ");
			Serial.println(ABRIR);
			Serial.print(" CERRAR = ");
			Serial.println(CERRAR);

			Serial.print("ABRIR_A = ");
			Serial.println(ABRIR_A);
			Serial.print(" CERRAR_A = ");
			Serial.println(CERRAR_A);

			Serial.print("ABRIR_M = ");
			Serial.println(ABRIR_M);
			Serial.print(" CERRAR_M = ");
			Serial.println(CERRAR_M);

			Serial.print("ABRIR_L = ");
			Serial.println(ABRIR_L);
			Serial.print(" CERRAR_L = ");
			Serial.println(CERRAR_L);

			Serial.println("");
			Serial.println("---------------------------------------------- ");

			Serial.print("cmd_ABRIR_Domo = ");
			Serial.println(cmd_ABRIR_Domo);
			Serial.print(" cmd_CERRAR_Domo = ");
			Serial.println(cmd_CERRAR_Domo);

			Serial.println("");
			Serial.println("---------------------------------------------- ");

			Serial.print("cmd_PAD_close = ");
			Serial.println(cmd_PAD_close);
			Serial.print(" cmd_PAD_open = ");
			Serial.println(cmd_PAD_open);

			Serial.print("PAD_close_M = ");
			Serial.println(PAD_close_M);
			Serial.print(" PAD_open_M = ");
			Serial.println(PAD_open_M);

			Serial.print("PAD_close_A = ");
			Serial.println(PAD_close_A);
			Serial.print(" PAD_open_A = ");
			Serial.println(PAD_open_A);

			Serial.print(" CERRAR_PAD_falla = ");
			Serial.println(CERRAR_PAD_falla);
			Serial.print(" ABRIR_PAD_falla = ");
			Serial.println(ABRIR_PAD_falla);


			Serial.print("hab_Secuencia_cargar_auto: ");
			Serial.println(hab_Secuencia_cargar_auto);
			Serial.print("Secuencia_cargar_auto: ");
			Serial.println(Secuencia_cargar_auto);

			Serial.println("---------------------------------------------- ");
			Serial.print("Codigo de Mensajes ");
			Serial.println(mensajes);


		}
*/
		if (comando == 'H') {

			//*Mb.C[10] = true; //Necesario para que Kepware no force el valor
			habilita_sistema();
			Serial.println("Sistema Habilitado ");
		}

		if (comando == 'h') {
			//*Mb.C[10] = false; //Necesario para que Kepware no force el valor
			deshabilita_sistema();
			Serial.println("Sistema Deshabilitado");
		}
/*
		if (comando == 'I') {
 
		}


		if (comando == 'J') {

		}
		if (comando == 'j') { 

	
		}
*/
		if (comando == 'K') {

			CHARGE_ON_Manual = true;
			//*Mb.C[18] = CHARGE_ON_Manual;
		}

		if (comando == 'k') {

			CHARGE_ON_Manual = false;
			//*Mb.C[18] = CHARGE_ON_Manual;
		}

		if (comando == 'L') {

			//*Mb.C[22] = false; //Necesario para que Kepware no force el valor
			Remoto_Local = false;
			Serial.println("Sistema en modo LOCAL ");
		}
		if (comando == 'l') {
			Serial.println("Secuencia de HOME en Automatico ");
			Secuencia_HOME2 = true;
		}
		




		if (comando == 'M') {

			Auto_Manual = false;
			//*Mb.C[21] = false; //Necesario para que Kepware no force el valor
			Serial.println("Modo Manual");
		}
/*
		if (comando == 'm') { //////////////////  DEBUG     ///////////////////


			
			Serial.print("Temperatura_OK ");
			Serial.println(Temperatura_OK);
			Serial.print("Humedad_OK ");
			Serial.println(Humedad_OK);
			Serial.print("vel_viento_OK ");
			Serial.println(vel_viento_OK);
			Serial.print("max_posicion_PAD ");
			Serial.println(max_posicion_PAD);
			Serial.print("vel_PAD ");
			Serial.println(vel_PAD);

			Serial.println("----------------------------------------------");

			Serial.print("tiempo Enfriado  ");
			Serial.println(tiempo_enfriado);
			Serial.print("tiempo Enfriado PSET ");
			Serial.println(timer[1].PSET);
			Serial.print("tiempo Enfriado PV ");
			Serial.println(timer[1].PV);

			Serial.print("tiempo Carga  ");
			Serial.println(tiempo_carga);
			Serial.print("tiempo Carga PSET ");
			Serial.println(timer[2].PSET);
			Serial.print("tiempo Carga PV ");
			Serial.println(timer[2].PV);

			Serial.print("tiempo descarga Video  ");
			Serial.println(tiempo_video);
			Serial.print("tiempo descarga Video PSET ");
			Serial.println(timer[10].PSET);
			Serial.print("tiempo descarga Video PV ");
			Serial.println(timer[10].PV);
		}
*/
		if (comando == 'N') {
			Serial.println("Forzar Estado de Enfriamiento de Secuencia CERRAR ");
			if (Auto_Manual && PAD_closed && Edo_maq_CERRAR == 0 && Edo_maq_ABRIR == 0 && Edo_maq_HOME == 0) {
				Edo_maq_CERRAR = 20;
			}
		}
		if (comando == 'n') {
			Serial.println("Forzar Estado de Carga de Secuencia CERRAR ");
			if (Auto_Manual && PAD_closed && Edo_maq_CERRAR == 0 && Edo_maq_ABRIR == 0 && Edo_maq_HOME == 0) {
				Edo_maq_CERRAR = 21;
			}
		}

		if (comando == 'O') {
			Serial.println("Abrir Domo");
			ABRIR_L = true;
			CERRAR_L = false;
			//Torque_control(1, max_torque_Domo);
			Comando_ABRIR_Domo();


		}

		if (comando == 'o') {
			Serial.println("Secuencia de ABRIR en Automatico ");
			Secuencia_ABRIR2 = true;
		}


		if (comando == 'P') {
			Serial.println("Paro de EMERGENCIA Activo");

			PE = true;
			//*Mb.C[47] = true; //Necesario para que Kepware no force el valor 


		}

		if (comando == 'p') {
			Serial.println("Paro de EMERGENCIA Desactivo");
			PE = false;
			//*Mb.C[47] = false; //Necesario para que Kepware no force el valor 

		}

		if (comando == 'V') {
			actualiza_posiciones();
			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.print("posicion_Domo = ");
			Serial.println(posicion_Domo);

			Serial.print("posicion_PAD  = ");
			Serial.println(posicion_PAD);

			Serial.print("  PAD_closed = ");
			Serial.println(PAD_closed, BIN);
			Serial.print("max_posicion_PAD  = ");
			Serial.println(max_posicion_PAD);
			Serial.println();

			Serial.print("  PAD_opened = ");
			Serial.println(PAD_opened, BIN);
			Serial.print("min_posicion_PAD  = ");
			Serial.println(min_posicion_PAD);

			Serial.print("vel_PAD ");
			Serial.println(vel_PAD);
			Serial.print("Ciclos ");
			Serial.println(Ciclos);
		}

		if (comando == 'q') {


		}


		if (comando == 'R') {

			//*Mb.C[22] = true; //Necesario para que Kepware no force el valor
			Remoto_Local = true;
			Serial.println("Sistema en modo REMOTO ");
		}

		if (comando == 'r') { //reset de ciclos en EEPROM
			Ciclos = 0;
			int eeAddress = 50;
			EEPROM.put(eeAddress, Ciclos); //lo escribe al EEPROM

		}



		if (comando == 'S') {
			Serial.println("Carga ON Manual");
			CHARGE_ON_Manual = true;
			//*Mb.C[18] = CHARGE_ON_Manual;
		}
		if (comando == 's') {
			Serial.println("Carga Off Manual");
			CHARGE_ON_Manual = false;
			//*Mb.C[18] = CHARGE_ON_Manual;
		}


		if (comando == 'T') {
			Serial.println("Leer Sensor de Temp y Humedad");
			read_TH_sensor();
			Serial.print("Temperatura = ");
			Serial.print(Temperature);
			Serial.print("  Humedad = ");
			Serial.println(Humidity);

		}
		if (comando == 't') {





		}



/*
		if (comando == 'U') {
			Write_PID_parameter_to_RAM(MOTOR, angleKp, angleKi, speedKp, speedKi, torqueKp, torqueKi);
			Serial.println();
			Serial.print("Parametros mandados a RAM motor ");
			Serial.println(MOTOR);
			Serial.println();

			Serial.print("angleKp ");
			Serial.println(angleKp);
			Serial.print("angleKi ");
			Serial.println(angleKi);

			Serial.print("speedKp ");
			Serial.println(speedKp);
			Serial.print("speedKi ");
			Serial.println(speedKi);

			Serial.print("torqueKp ");
			Serial.println(torqueKp);
			Serial.print("torqueKi ");
			Serial.println(torqueKi);
		}

		if (comando == 'u') {
			Write_PID_parameter_to_ROM(MOTOR, angleKp, angleKi, speedKp, speedKi, torqueKp, torqueKi);
			Serial.println();
			Serial.print("Parametros mandados a ROM del motor ");
			Serial.println(MOTOR);
			Serial.println();

			Serial.print("angleKp ");
			Serial.println(angleKp);
			Serial.print("angleKi ");
			Serial.println(angleKi);

			Serial.print("speedKp ");
			Serial.println(speedKp);
			Serial.print("speedKi ");
			Serial.println(speedKi);

			Serial.print("torqueKp ");
			Serial.println(torqueKp);
			Serial.print("torqueKi ");
			Serial.println(torqueKi);

		}
*/

		if (comando == 'v') {
			Serial.println(PLC_version);
			Serial.println("---------------------------------------------- ");


		}

	/*
		if (comando == 'Q') {


		
			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.print("CHARGE_ON: ");
			Serial.println(CHARGE_ON);

			Serial.print("Cargador_ON: ");
			Serial.println(Cargador_ON);

			//lee_bateria();

			Serial.println("");
			Serial.println("---------------------------------------------- ");
			Serial.print("Voltaje_bateria_drone = ");
			Serial.println(Voltaje_bateria_drone);
			Serial.print("Voltaje_bateria_drone_raw = ");
			Serial.println(Voltaje_bateria_drone_raw);

			Serial.println("");
			Serial.println("---------------------------------------------- ");

			Serial.print("posicion_Domo = ");
			Serial.println(posicion_Domo2);

			Serial.print("posicion_PAD  = ");
			Serial.println(posicion_PAD2);
			Serial.println("");



		}

	*/
		if (comando == 'W') {

			Serial.print("  vel_viento = ");
			Serial.println(vel_viento / 100);
			Serial.print("  vel_viento_int = ");
			Serial.println(vel_viento_int);
		}


/*
		if (comando == 'w') {



			Serial.println();
			Serial.print("Parametros de default en RAM del PLC ");
			angleKp = 100;
			angleKi = 80;
			speedKp = 50;
			speedKi = 20;
			torqueKp = 100;
			torqueKi = 100;
			Serial.println();

			Serial.print("angleKp ");
			Serial.println(angleKp);
			Serial.print("angleKi ");
			Serial.println(angleKi);

			Serial.print("speedKp ");
			Serial.println(speedKp);
			Serial.print("speedKi ");
			Serial.println(speedKi);

			Serial.print("torqueKp ");
			Serial.println(torqueKp);
			Serial.print("torqueKi ");
			Serial.println(torqueKi);

		}
*/
/*

		if (comando == 'X') {//TOGGLE


		}

		if (comando == 'x') {
	
		}

		if (comando == 'Y') {

		}
*/		
		if (comando == 'y') {

			int eeAddress = 50;   //primera localidad de EEPROM. Aqui esta la bandera " direccion_configurada"
			byte parametros_configurados = 0;

			// parametros_configurados = 165; //la activo con el codigo 10100101B  por tener muchos unos y ceros en cierta posicion 
			parametros_configurados = 0; //la activo con el codigo 10100101B  por tener muchos unos y ceros en cierta posicion 

		  //escribe los parametros en el EEPROM
			eeAddress = 20;
			EEPROM.write(eeAddress, parametros_configurados);
		}

		/*
		
		if (comando == 'Z') {

			Serial.println();
			Serial.println("Parametros en RAM del PLC ");
			Serial.print("angleKp ");
			Serial.println(angleKp);
			Serial.print("angleKi ");
			Serial.println(angleKi);

			Serial.print("speedKp ");
			Serial.println(speedKp);
			Serial.print("speedKi ");
			Serial.println(speedKi);

			Serial.print("torqueKp ");
			Serial.println(torqueKp);
			Serial.print("torqueKi ");
			Serial.println(torqueKi);

		}
*/

		if (comando == 'z') {

			Comando_read_PID_parameters(MOTOR);
		}

		if (comando == '0') {
			Comando_motor_shutdown(MOTOR);
		}

		if (comando == '.') {
			Comando_motor_operate(MOTOR);
		}

		if (comando == ',') {
			Comando_motor_stop(MOTOR);
		}



		if (comando == '1') {
			MOTOR = 1;
			Serial.print("MOTOR  = ");
			Serial.println(MOTOR);
		}

		if (comando == '2') {
			MOTOR = 2;
			Serial.print("MOTOR  = ");
			Serial.println(MOTOR);

		}
		if (comando == '3') {
			//Comando_reset_error_flag(MOTOR);
			//Comando_reset_zero_position_to_ROM(MOTOR);
			Comando_reset_zero_encoder(MOTOR);
		}

		if (comando == '4') {
			Comando_reset_zero_position_to_ROM(MOTOR);
			//Comando_reset_zero_encoder(MOTOR);

		}

		if (comando == '5') {

			}

		if (comando == '6') {


		}

		if (comando == '7') {
			Comando_read_encoder(MOTOR);
			delay(500);
			Comando_read_multi_loop_Angle(MOTOR);
		}
		if (comando == '8') {

			motor_incremental_position_speed_control(MOTOR, 7000, vel_PAD, 0x01);
		}



		if (comando == '9') {

			motor_incremental_position_speed_control(MOTOR, 7000, vel_PAD, 0x00);

		}

		if (comando == '(') {
			vel_PAD = vel_PAD + 500;
			//*Mb.R[49] = vel_PAD; // necesario
			Serial.print("vel_PAD ");
			Serial.println(vel_PAD);


		}
		if (comando == ')') {
			vel_PAD = vel_PAD - 500;
			//*Mb.R[49] = vel_PAD; // necesario
			Serial.print("vel_PAD ");
			Serial.println(vel_PAD);


		}


	}

}



//*****************************************************************************************************************************//
// ****************************************************  SETUP  ****************************************************************
//*****************************************************************************************************************************//




void condini() {


	CHARGE_ON_Carga_Auto = false; //no lo puedo poner en reset comandos

	reset_comandos();



	//Por Default esta en control automatico de secuencia
	Auto_Manual = true;
	//*Mb.C[21] = true;  //necesario la primera vez para que kepware no escriba cero al inicio

	//Por Default esta en control Remoto
	Remoto_Local = true; //por default esta en modo Remoto, es decir; desde kepware via ethernet
	//*Mb.C[22] = true;   //necesario la primera vez para que kepware no escriba cero al inicio


	//*Mb.C[32] = habilita_pruebas;




	//*Mb.C[41] = bandera_TX_auto;

	//*Mb.R[9] = Temperatura_OK;
	//*Mb.R[10] = Humedad_OK;
	//*Mb.R[11] = vel_viento_OK;
	//*Mb.R[16] = Voltaje_bateria_drone_OK;
	//*Mb.R[23] = Max_ciclos_PRUEBA;


	//*Mb.R[27] = tiempo_carga;
	//*Mb.R[28] = tiempo_video;
	//*Mb.R[26] = tiempo_enfriado;

	//Constantes de los timers para falla en movimientos (decimas de segundo)
	//*Mb.R[42] = Timeout_ABRIR_Domo;
	//*Mb.R[43] = Timeout_CERRAR_Domo;
	//*Mb.R[44] = servo1_position0;
	//*Mb.R[45] = servo1_position1;
	////*Mb.R[46] = Timeout_ABRIR_PAD;
   ////*Mb.R[47] = Timeout_CERRAR_PAD;
	//*Mb.R[46] = Primer_Pulso_ON_OFF;
	//*Mb.R[47] = Segundo_Pulso_ON_OFF;


	//*Mb.R[48] = max_posicion_PAD;  // limita la posicio maxima de cerrado del PAD
	//*Mb.R[49] = vel_PAD; // limita el torque naximo del motor del PAD

  //necesario la primera vez para que kepware no escriba cero al inicio. ya cuando condini se llama, ya los leyo del EEPROM!!

	//*Mb.R[30] = ip[3];
	//*Mb.R[31] = ip[2];
	//*Mb.R[32] = ip[1];
	//*Mb.R[33] = ip[0];
	//*Mb.R[34] = subnet[3];
	//*Mb.R[35] = subnet[2];
	//*Mb.R[36] = subnet[1];
	//*Mb.R[37] = subnet[0];
	//*Mb.R[38] = gateway[3];
	//*Mb.R[39] = gateway[2];
	//*Mb.R[40] = gateway[1];
	//*Mb.R[41] = gateway[0];

	//*Mb.R[17] = Temperatura_bateria_drone_MAX;

	deshabilita_sistema();


	//Condini inicia las maquinas de estados
	inicia_maquinas_de_estados();



	//checa las condiciones de la base. 
	lee_entradas_PLC();  //esto se usa solo al arrancar el sistema


   //set de las senales de warning de bateria
	//*habilita_Warning_Temperatura_bateria = false; //no jala la temperatura
	//*Mb.C[45] = habilita_Warning_Temperatura_bateria; //necesario

	habilita_Warning_Voltaje_bateria = true;
	//*Mb.C[46] = habilita_Warning_Voltaje_bateria; //necesario



	lee_bateria();

}


uint8_t Checksumcrc(uint8_t* aData, uint8_t StartIndex, uint8_t DataLenght)
{
	uint8_t crc = 0;
	uint8_t i = 0;
	for (i = StartIndex; i < (StartIndex + DataLenght); i++)
	{
		crc += aData[i];
	}
	return crc;
}

//Set point  Loop de Torque.. [-2000 ... 2000]


//Comando Write_PID_parameter_to_RAM 
void Write_PID_parameter_to_RAM(uint8_t ID_motor, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t torqueKp, uint8_t torqueKi) {

	unsigned i;

	Serial.println("Comando Write_PID_parameter_to_RAM motor: ");
	Serial.println(MOTOR);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x31;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x06;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = (uint8_t)(angleKp);
	comando_motor[6] = (uint8_t)(angleKi);
	comando_motor[7] = (uint8_t)(speedKp);
	comando_motor[8] = (uint8_t)(speedKi);
	comando_motor[9] = (uint8_t)(torqueKp);
	comando_motor[10] = (uint8_t)(torqueKi);
	comando_motor[11] = Checksumcrc(comando_motor, 5, 0X06);

	for (i = 0; i < 12; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);

	}

	/*
	Serial.print("Command sent  ");
	for (i = 0; i < 18; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
		}
	Serial.println();
*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}


//Comando Write_PID_parameter_to_ROM 
void Write_PID_parameter_to_ROM(uint8_t ID_motor, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t torqueKp, uint8_t torqueKi) {

	unsigned i;

	Serial.println("Comando Write_PID_parameter_to_ROM motor: ");
	Serial.println(MOTOR);


	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x32;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x06;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = (uint8_t)(angleKp);
	comando_motor[6] = (uint8_t)(angleKi);
	comando_motor[7] = (uint8_t)(speedKp);
	comando_motor[8] = (uint8_t)(speedKi);
	comando_motor[9] = (uint8_t)(torqueKp);
	comando_motor[10] = (uint8_t)(torqueKi);
	comando_motor[11] = Checksumcrc(comando_motor, 5, 0X06);

	for (i = 0; i < 12; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);

	}

	/*
	Serial.print("Command sent  ");
	for (i = 0; i < 18; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
		}
	Serial.println();
*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}


void Torque_control(uint8_t ID_motor, uint16_t iqControl) {





	unsigned i;


	Serial.print("Comando Torque control ");

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0xA1;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 2;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = *(uint8_t*)(&iqControl);
	comando_motor[6] = *((uint8_t*)(&iqControl) + 1);
	comando_motor[7] = Checksumcrc(comando_motor, 5, 2);

	for (i = 0; i < 8; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);

	}
	/*
	Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i < 8; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}

//Set point  Loop de angulo y velocidad..  Angulo en centesimas de grado y velocidad en centecimas de grados/rev.
void motor_incremental_position_speed_control(uint8_t ID_motor, int32_t angleControl, uint32_t maxSpeed, uint8_t direction) {





	unsigned i;

	angleControl = angleControl * 6; // NECESARIO PORQUE EL REDUCTOR DEL MOTOR TIEE UNA RELACION DE 6.

	if (direction == 0)
		angleControl = angleControl * -1;

	Serial.println("motor_incremental_position_speed_control ");

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0xA8;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x08;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = *(uint8_t*)(&angleControl);
	comando_motor[6] = *((uint8_t*)(&angleControl) + 1);
	comando_motor[7] = *((uint8_t*)(&angleControl) + 2);
	comando_motor[8] = *((uint8_t*)(&angleControl) + 3);
	comando_motor[9] = *(uint8_t*)(&maxSpeed);
	comando_motor[10] = *((uint8_t*)(&maxSpeed) + 1);
	comando_motor[11] = *((uint8_t*)(&maxSpeed) + 2);
	comando_motor[12] = *((uint8_t*)(&maxSpeed) + 3);
	comando_motor[13] = Checksumcrc(comando_motor, 5, 8);

	for (i = 0; i < 14; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);

	}

	/*
	Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i < 14; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
		}
	Serial.println();
*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}



//Set point  Loop de angulo y velocidad..  Angulo en centesimas de grado y velocidad en centecimas de grados/rev.
void motor_single_position_speed_control(uint8_t ID_motor, uint16_t angleControl, uint32_t maxSpeed, uint8_t direction) {





	unsigned i;

	angleControl = angleControl * 6; // NECESARIO PORQUE EL REDUCTOR DEL MOTOR TIEE UNA RELACION DE 6.

	Serial.println("Comando angle_speed_control ");

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0xA6;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x08;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = direction;
	comando_motor[6] = *(uint8_t*)(&angleControl);
	comando_motor[7] = *((uint8_t*)(&angleControl) + 1);
	comando_motor[8] = 0x00; //null

	comando_motor[9] = *(uint8_t*)(&maxSpeed);
	comando_motor[10] = *((uint8_t*)(&maxSpeed) + 1);
	comando_motor[11] = *((uint8_t*)(&maxSpeed) + 2);
	comando_motor[12] = *((uint8_t*)(&maxSpeed) + 3);
	comando_motor[13] = Checksumcrc(comando_motor, 5, 8);

	for (i = 0; i < 14; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);

	}

	/*
	Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i < 14; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
		}
	Serial.println();
*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}



//Set point  Loop de angulo y velocidad..  Angulo en centesimas de grado y velocidad en centecimas de grados/rev.
void motor_multi_position_speed_control(uint8_t ID_motor, uint64_t angleControl, uint32_t maxSpeed) {




	unsigned i;
	angleControl = angleControl * 6; // NECESARIO PORQUE EL REDUTOR DEL MOTOR TIEE UNA RELACION DE 6.

	Serial.println("Comando angle_speed_control ");

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0xA4;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x0C;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = *(uint8_t*)(&angleControl);
	comando_motor[6] = *((uint8_t*)(&angleControl) + 1);
	comando_motor[7] = *((uint8_t*)(&angleControl) + 2);
	comando_motor[8] = *((uint8_t*)(&angleControl) + 3);
	comando_motor[9] = *((uint8_t*)(&angleControl) + 4);
	comando_motor[10] = *((uint8_t*)(&angleControl) + 5);
	comando_motor[11] = *((uint8_t*)(&angleControl) + 6);
	comando_motor[12] = *((uint8_t*)(&angleControl) + 7);
	comando_motor[13] = *(uint8_t*)(&maxSpeed);
	comando_motor[14] = *((uint8_t*)(&maxSpeed) + 1);
	comando_motor[15] = *((uint8_t*)(&maxSpeed) + 2);
	comando_motor[16] = *((uint8_t*)(&maxSpeed) + 3);
	comando_motor[17] = Checksumcrc(comando_motor, 5, 0X0C);

	for (i = 0; i < 18; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);

	}

	/*
	Serial.print("Command sent  ");
	for (i = 0; i < 18; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
		}
	Serial.println();
*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}


void Comando_reset_error_flag(uint8_t ID_motor) {
	unsigned i = 0;

	//Serial.print("Comand _reset_error_flag");
	//Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x9B;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}

	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}


void motor_multi_position_control(uint8_t ID_motor, uint64_t angleControl) {



	unsigned i;
	angleControl = angleControl * 6; // NECESARIO PORQUE EL REDUTOR DEL MOTOR TIEE UNA RELACION DE 6.

	Serial.println("Comando angle_control ");

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0xA3;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x08;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	comando_motor[5] = *(uint8_t*)(&angleControl);
	comando_motor[6] = *((uint8_t*)(&angleControl) + 1);
	comando_motor[7] = *((uint8_t*)(&angleControl) + 2);
	comando_motor[8] = *((uint8_t*)(&angleControl) + 3);
	comando_motor[9] = *((uint8_t*)(&angleControl) + 4);
	comando_motor[10] = *((uint8_t*)(&angleControl) + 5);
	comando_motor[11] = *((uint8_t*)(&angleControl) + 6);
	comando_motor[12] = *((uint8_t*)(&angleControl) + 7);

	comando_motor[13] = Checksumcrc(comando_motor, 5, 8);




	for (i = 0; i < 14; i++) {

		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}
	/*
	Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i < 14; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	*/
	RS485.flush();
	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}

void Comando_reset_zero_position_to_ROM(uint8_t ID_motor) {
	unsigned i = 0;

	Serial.print(" Comand reset_zero_position_to_ROM");
	Serial.print("MOTOR  = ");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x19;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}

	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	*/
	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}

void Comando_reset_zero_encoder(uint8_t ID_motor) {
	unsigned i = 0;

	Serial.println("Comand reset zero encoder");
	Serial.print("MOTOR  = ");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x91;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x02;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);
	//comando_motor[5] = encoder_offsetLSB;
	//comando_motor[6] = encoder_offsetMSB;
	//comando_motor[5] = encoderLSB;
	//comando_motor[6] = encoderMSB;
	comando_motor[5] = 0x0;
	comando_motor[6] = 0x0;
	comando_motor[7] = Checksumcrc(comando_motor, 5, 2);

	for (i = 0; i <= 7; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
		//Serial.print(comando_motor[i], HEX);
	}

	Serial.print("Command sent:  ");
	for (i = 0; i <= 7; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();

	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}

void Comando_motor_shutdown(uint8_t ID_motor) {
	unsigned i = 0;

	Serial.println("Comando Motor shutdown ");
	Serial.print("MOTOR  = ");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x80;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}

	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
*/
	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}

void Comando_motor_stop(uint8_t ID_motor) {
	unsigned i = 0;

	Serial.print("Comando Motor stop ");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x81;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}

	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
*/
	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}

void Comando_motor_operate(uint8_t ID_motor) {
	unsigned i = 0;

	Serial.println("Comando Motor operate ");
	Serial.print("MOTOR  = ");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x88;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}

	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
*/
	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido
}

void Comando_read_encoder(uint8_t ID_motor) {
	unsigned i = 0;
	Serial.println("Comando read encoder");
	Serial.print("MOTOR  = ");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x90;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}
	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	*/
	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}

void Comando_read_multi_loop_Angle(uint8_t ID_motor) {
	unsigned i = 0;
	if (Debug) {
		Serial.println("Comando Read multi-loop Angle");
		Serial.print("MOTOR  = ");
		Serial.println(ID_motor);
	}
	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x92;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}

	//RS485.write(comando_motor, 5);

	if (Debug) {
		Serial.print("COMANDO ENVIADO  ");
		for (i = 0; i <= 4; i++) {
			Serial.print(comando_motor[i], HEX);
			Serial.print(" ");
		}
		Serial.println();

	}

	delay(10); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}


void Comando_read_PID_parameters(uint8_t ID_motor) {
	unsigned i = 0;
	Serial.print("Comando_read_PID_parameters motor:");
	Serial.println(ID_motor);

	comando_motor[0] = 0x3E;
	comando_motor[1] = 0x30;
	comando_motor[2] = ID_motor;
	comando_motor[3] = 0x00;
	comando_motor[4] = Checksumcrc(comando_motor, 0, 4);

	for (i = 0; i <= 4; i++) {
		// send it to the RS-485 port
		RS485.write(comando_motor[i]);
	}
	/*Serial.print("COMANDO ENVIADO  ");
	for (i = 0; i <= 4; i++) {
		Serial.print(comando_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	*/
	RS485.flush();
	delay(150); //HYPER NECESARIO!!!, si no el motor pierde los comandos al enviarlos tan rapido

}


//Este evento se activa cada que llega un byte
void RS485Event() {
	unsigned i, index;
	byte rx;






	index = 0;
	while (RS485.available()) {

		// Print received byte
		rx = RS485.read();

		status_motor[index] = rx;
		index++;

		// Hexadecimal representation
		//Serial.print("HEX: ");
		//Serial.println(rx, HEX);

		//RS485.write("HEX: ");
		//RS485.write(rx);

	}

	/*Serial.print("STRING RECIBIDO  ");
	for (i = 0; i < index; i++) {
		Serial.print(status_motor[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	*/
	decodifica_status_motor();

}


void Comando_cerrar_ejes_pad() {


	//max_posicion_PAD = 9000;
	Serial.print("Comando cerrar_ejes_pad   SP Posicion: ");
	Serial.print(max_posicion_PAD);
	Serial.print("   SP Velocidad: ");
	Serial.println(vel_PAD);
	Comando_reset_error_flag(2); //reset fallas en motor 2
	delay(15);
	motor_multi_position_speed_control(2, max_posicion_PAD, vel_PAD);




	
}







void Comando_abrir_ejes_pad() {



	//min_posicion_PAD = 0;
	Serial.print("Comando abrir_ejes_pad  SP Posicion ");
	Serial.print(min_posicion_PAD);
	Serial.print("   SP Velocidad: ");
	Serial.println(vel_PAD);
	Comando_reset_error_flag(2); //reset fallas en motor 2
	delay(15);
	motor_multi_position_speed_control(2, min_posicion_PAD, vel_PAD);



	Cargador_ON = false;//regresa   //se desergiza el cargador para que no haya energia en los contactos del PAD mientras esta abierto

}


void Comando_ABRIR_Domo() {

	unsigned i;
	Serial.print("Comando ABRIR Domo   Posicion");
	Serial.println(min_posicion_Domo);
	motor_multi_position_speed_control(1, min_posicion_Domo, vel_Domo_open);
}

void Comando_CERRAR_Domo() {

	unsigned i;
	Serial.print("Comando CERRAR Domo  Posicion");
	Serial.println(max_posicion_Domo);
	motor_multi_position_speed_control(1, max_posicion_Domo, vel_Domo_close);
}








void setup() {

	configura_arduino_timers();
	inicia_timers();

	// initialize the serial port:
	Serial.begin(115200);
	//Al arrancar manda la version del programa al puerto serial
	Serial.println(PLC_version);

	// Begin RS485 port
	//RS232.begin(115200);

	// Begin RS485 port
	RS485.begin(115200);


	//setup_ethernet();
	//setup_parametros(); //tiene que ir antes de condini()!



	read_TH_sensor();
	condini();

	interrupts();     // enable all interrupts

	//************************************API Rest Init **************************************************
	configureRestAPI();
	//AutoEPExpo(); // Exposes the Sequence Functions to the Rest API 
	
	//ControlEPExpo(); // Exposes the Control Functions to the Rest API
	MonitorEPExpo(); // Exposes the Monitor Endpoints to the Rest API

	 // Start watchdog
	//wdt_enable(WDTO_4S);

	Serial.println("Ready...\n");


	 // Start watchdog
	//wdt_enable(WDTO_4S);

	/*
	 * CONNECTION TO REST API SHOULD TAKE 8-10 SECONDS
	 */

	 //************************************API Rest Init **************************************************




	//Secuencia_Reset_Zero = true;




}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// **** MAIN PROGRAM  ****
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long currentMillis;

void loop()
{
	
	currentMillis = millis();
	if (currentMillis - previousMillis >= 250) { // Leo las poiciones cada 250mSeg
		previousMillis = currentMillis;
		actualiza_posiciones();



	}

	// ***********      Lee comandos desde el puerto Serial  ********
	lee_comando();
	//lee_status_motor();

	// ***********      Comunicacion con MODBUS  ********
	//lee_modbus();


	// ***********      Ejecuta programa combinacional   LLAMA A LA LOGICA DE ESCALERA  ********
	ladder();

	// ***********   Control secuencial    LLAMA A LAS MAQUINAS DE ESTADOS    *************

	maquinas_de_estados();






// ***************************** API Rest functionality ****************************


		// listen for incoming clients
		EthernetClient client = server.available();
		rest.handle(client);

		//wdt_reset();
		// ***************************** API Rest functionality ****************************




}
