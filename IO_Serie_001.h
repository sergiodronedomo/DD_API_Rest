

//Donepuerto: Serie = 007 Malinalco


// ***************** MAPEO salidas digitales ************************


const int Ventilador_ON_output = Q0_0;		//Cable blanco grueso ventilador
const int  reset_Khadas_output = Q0_1;		//cable Azul delgado  Reset de la computadora NUC
const int  seguro_domo = Q0_2;				// Seguro del DOMO
const int  Drone_ON_OFF_output = Q0_3;		// Cable Verde encendido del Drone
const int  Energizar_PEINE_output = Q0_4;	//cable blanco con azul  Manda energia al PEINE 
const int    libre05 = Q0_5;				// ventilador 
const int	CHARGE_ON_output = Q0_6;		//  cable azul grueso Carga DRONE
const int	SERVO_output = Q0_7;			// Cable naranja PWM desde el PLC hacia el cto del servo para prender y apagar el Transmisor  



// **************** MAPEO Entradas digitales **********************


const int libre1_input = I0_0;      //S1  pin 22 Arduino
const int libre2_input = I0_1;      //S2  pin 23 Arduino
const int Retro_edo_TX_input = I0_2;    //Transmisor_STATUS
const int libre3_input = I0_3;      //
const int libre4_input = I0_4;
const int libre5_input = I0_5;
const int libre6_input = I0_6;
const int libre7_input = I0_7;		// 
const int status_fuente48V = I0_8;   	// 
const int status_fuente24V = I0_9;


// **************** MAPEO Entradas Analogicas **********************
const int Temp_bateria_input = I0_10;
const int voltaje_bateria_input = I0_11;
const int wind_sensor_input = I0_12;

