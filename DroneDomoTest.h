
// Libraries
#include <SPI.h>
#include "Ethernet.h"
#include "aREST.h"
#include <avr/wdt.h>
#include "Communications.h"



// IP address in case DHCP fails
IPAddress IP(ip[0], ip[1], ip[2], ip[3]); // IP Address to Connect w/ REST API

// Ethernet server
EthernetServer server(80);

// Create aREST instance
aREST rest = aREST();

char* device_id = "DronePort"; // ID to identify the device on the cloud (should be at least 6 characters long)


//---------------------------------------------------------------------------------------------------------
//Function prototypes to be exposed to the API

//int Enable_Disable(String command);


//int Auto_Man(String command) ;

int ResetAlarms(String command);

//int AcknoledgeAlarms(String command);




//int ClearAlarms(String command);

//int ClearAllAlarms(String command);

int ChargeDrone_ON_OFF(String command);

int Ventilation_ON_OFF(String command);


int OpenDome(String command);

int CloseDome(String command);

int ClosePAD(String command);

int OpenPAD(String command);

//int IP_address(String command);

int OpenDomeSequence(String command);

int CloseDomeSequence(String command);

int HomeSequence(String command);
//---------------------------------------------------------------------------------------------------------

// Function Pointer prototype which takes in a String and returns an int
typedef int (*Automatic)(String);
typedef int (*Control)(String);

//---------------------------------------------------------------------------------------------------------

String trimCmnd(String command) { // Trims a command that is of class String and removes whitespaces and the "/" when POSTING
	int cmnd_length = command.length();
	String charTarget = " ";

	int search_index = command.indexOf("/");

	command.remove(search_index);
	command.trim();

	return command;
}

bool str_to_bool(String command) { // Converts a command that is of class String into a bool
	bool boolEq;

	command = trimCmnd(command); // Trims the command so it can be used in the if statements

	if ((command.substring(0) == "true") || (command == "1")) {
		boolEq = true;
	}
	else if ((command.substring(0) == "false") || (command == "0")) {
		boolEq = false;
	}
	else {
		Serial.println("[ERROR]: \"" + command + "\" is not equivalent to a boolean");
	}

	return boolEq;
}

//---------------------------------------------------------------------------------------------------------

// Array of 'EndPoint' function pointers
Automatic endpoints[3] = { &OpenDomeSequence, &CloseDomeSequence, &HomeSequence};

Control c_endpoints[7] = {  &ResetAlarms, &ChargeDrone_ON_OFF, &Ventilation_ON_OFF,
						&OpenDome, &CloseDome, &ClosePAD, &OpenPAD };


char* auto_instances[] = { "ODS", "CDS", "HS" };

char* cntrl_instances[] = {  "RA", " CDR", "VENT", "OD", "CD", "CP", "OP" };

String expMessage[] = { "Open Dome Sequence", "Close Dome Sequence", "Home Sequence",
						"ResetAlarms", "ChargeDrone_ON_OFF", "Ventilation_ON_OFF",
						"OpenDome", "CloseDome", "ClosePAD", "OpenPAD" };

// Array of String instances for the Monitor Endpoint Exposure to the Rest API
char* monInstances[] = { "NumberOfCycles", "DomeOpenStatus", "DomeCloseStatus",
						 "PadOpenStatus", "PadCloseStatus", "DronePortOpenStatus",
						 "DronePortClosedStatus", "CurrentHomeStatus",
						 "CurrentTake_offPermisive", "CurrentWeatherPermisive",
						 "BatteryAlarm", "SystemAlarm", "EmergencyStopStatus",
						 "WeatherWindConditions", "WeatherTemperatureConditions",
						 "WeatherHumidityConditions",
						 "BatteryVoltageConditions", "AlarmMessage" };

#define NUM_AUTO_EP     3
#define NUM_CONTROL_EP  7

//---------------------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------------------







void configureRestAPI() {
	// Init variables and expose them to REST API

	// Give name & ID to the device
	rest.set_id(device_id);
	rest.set_name("ethernet");

	// Start the Ethernet connection and the server
	if (Ethernet.begin(mac) == 0) {
		Serial.println("Failed to configure Ethernet using DHCP...");
		// no point in carrying on, so do nothing forevermore:
		// try to congifure using IP address instead of DHCP:
		Ethernet.begin(mac, IP);
	}

	server.begin();
	Serial.print("Server IP is at ");
	Serial.println(Ethernet.localIP());

	Serial.println("\nRest API Configured & Connected");
	/*
	 * CONNECTION TO REST API SHOULD TAKE 8-10 SECONDS
	 */
}

void AutoEPExpo() {
	Serial.println("\nInitiated Function Exposure to Rest API...");

	for (int i = 0; i < NUM_AUTO_EP; i++) {
		rest.function(auto_instances[i], endpoints[i]);  // Exposes the functions to the REST API
		Serial.println(expMessage[i] + " Exposed to Rest API...");
	}
	Serial.println("Automatic Endpoint Exposure to Rest API completed...\n");
}

void ControlEPExpo() {
	Serial.println("\nInitiated Control Endpoint Exposure to Rest API...");

	for (int i = 0; i < NUM_CONTROL_EP; i++) {
		rest.function(cntrl_instances[i], c_endpoints[i]); // Exposes the functions to the REST API
		Serial.println(expMessage[i + 3] + " Exposed to Rest API...");
	}
	Serial.println("Control Endpoints Exposure to Rest API completed...\n");
}

void MonitorEPExpo() {
	Serial.println("\nInitiated Monitor Endpoint Exposure to Rest API...");

	rest.variable(monInstances[0], &Ciclos);
	rest.variable(monInstances[1], &Domo_Abierto);
	rest.variable(monInstances[2], &Domo_Cerrado);
	rest.variable(monInstances[3], &PAD_opened);
	rest.variable(monInstances[4], &PAD_closed);
	rest.variable(monInstances[5], &base_abierta);
	rest.variable(monInstances[6], &base_cerrada);
	rest.variable(monInstances[7], &home_base);
	rest.variable(monInstances[8], &Permisivo_despegue);
	rest.variable(monInstances[9], &C_Ambientales_OK);
	rest.variable(monInstances[10], &Warning_bateria);
	rest.variable(monInstances[11], &Alarma_sistema);
	rest.variable(monInstances[12], &PE);
	rest.variable(monInstances[13], &vel_viento);
	rest.variable(monInstances[14], &Temperature);
	rest.variable(monInstances[15], &Humidity);
	//rest.variable(monInstances[16], &Temp_bateria_Celcius);
	rest.variable(monInstances[16], &Voltaje_bateria_drone);
	//rest.variable(monInstances[18], &Edo_maq_ABRIR);
	//rest.variable(monInstances[19], &Edo_maq_CERRAR);
	//rest.variable(monInstances[20], &Edo_maq_HOME);
	rest.variable(monInstances[17], &mensajes);

	Serial.println("Monitor Endpoints Exposure to Rest API completed...\n");
}

int Enable_Disable(String command) {
	sistema_habilitado = str_to_bool(command);
	String IO_mode = (sistema_habilitado == true ? "Enabled" : "Disabled");
	Serial.println("System " + IO_mode + "\n");
	return 1;
}

int Auto_Man(String command) {
	Auto_Manual = str_to_bool(command);
	String IO_mode = (Auto_Man == true ? "Auto" : "Manual");
	Serial.println(IO_mode + "Mode On\n");
	return 1;
}

int ResetAlarms(String command) {
	reset_dronepuerto = str_to_bool(command);
	if (reset_dronepuerto) { Serial.println("Alarms Reset\n"); }
	return 1;
}
/*
int AcknoledgeAlarms(String command) {
	reconocer_alarmas = str_to_bool(command);
	Serial.println("Acknowledge Alarms\n");
	return 1;
}



int ClearAlarms(String command) {
	borrar_alarmas = str_to_bool(command);
	if (borrar_alarmas) { Serial.println("Clear Alarms\n"); }
	return 1;
}

int ClearAllAlarms(String command) {
	borrar_todo_alarmas = str_to_bool(command);
	if (borrar_todo_alarmas) { Serial.println("Clear All Alarms \n"); }
	return 1;
}
*/

int ChargeDrone_ON_OFF(String command) {
	CHARGE_ON_Manual = str_to_bool(command);
	String IO_mode = (CHARGE_ON_Manual == true ? "On" : "Off");
	Serial.println("Charge Mode " + IO_mode + "\n");
	return 1;
}

int Ventilation_ON_OFF(String command) {
	Ventilador_ON_Manual = str_to_bool(command);
	String IO_mode = (Ventilador_ON_Manual == true ? "On" : "Off");
	Serial.println("Ventilation " + IO_mode + "\n");
	return 1;
}

int OpenDome(String command) {
	ABRIR_M = str_to_bool(command);
	if (ABRIR_M) { Serial.println("Open Dome\n"); }
	return 1;
}

int CloseDome(String command) {
	CERRAR_M = str_to_bool(command);
	if (CERRAR_M) { Serial.println("Close Dome\n"); }
	return 1;
}

int ClosePAD(String command) {
	PAD_close_M = str_to_bool(command);
	if (PAD_close_M) { Serial.println("Drone Locked\n"); }
	return 1;
}

int OpenPAD(String command) {
	PAD_open_M = str_to_bool(command);
	if (PAD_open_M) { Serial.println("Drone Unlocked\n"); }
	return 1;
}

//int IP_address(String command){
//  byte IP_new = 
//  return 1;
//}

int OpenDomeSequence(String command) {

	Serial.println("\nOpen Dome Sequence  Started\n");

	return 1;
}

int CloseDomeSequence(String command) {

	Serial.println("Close Dome Sequence Started");

	return 1;
}

int HomeSequence(String command) {

	Serial.println("Home Sequence  Started");

	return 1;
}





//byte str_to_byte(String command){
//  byte new_byte[4];
//
//  command = trimCmnd(command);
//  
//  String data_array[] = { command.substring(0,2), command.substring(4,6),
//                          command.substring(8), command.substring(10,12) };
//                         
//  for(int digit = 0; digit < 4; digit++){
//    new_byte[digit] = byte(data_array[digit].toInt());
//  }
//  
//  return new_byte;
//}
