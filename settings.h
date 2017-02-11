//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "p03mmr"
#define DEVICE_TYPE "ESPRFID"
#define DEVICE_ID "RFIDESP779432"
#define TOKEN "hY5OOupZk*U1yMl1G8"
//-------- Customise the above values --------

//-------- Customise these values-----------
//---------Blurmix Topics---------------------

const char publishTopic[] = "iot-2/evt/status/fmt/json";
const char responseTopic[] ="iotdm-1/response";
const char manageTopic[] = "iotdevice-1/mgmt/manage";
const char updateTopic[] = "iotdm-1/device/update";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/reboot";

//-----------Variables de Configuracion del Servicio de NTP
//-------- Configuracion de parametros de servicio remots de hora (NTP Servers:)

IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const char* ntpServerName = "time.nist.gov";
unsigned int localPort = 2390;  // local port to listen for UDP packets
const int timeZone = -6;  // Eastern central Time (USA)

//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long publishInterval = 20*1000UL; //Variable configurable remotamente sobre el interbalo de publicacion
unsigned long UPDATESENDTIME = 15*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos

//-------- Variables de ERROR EN ENVIO de paquetes de MQTT ANTES DE REINICIO
#define FAILTRESHOLD 150

