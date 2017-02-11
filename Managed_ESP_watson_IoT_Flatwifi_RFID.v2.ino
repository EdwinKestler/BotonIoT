#include <Arduino.h>

// Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>                                              //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>                                             //https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>                                              //https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
//----------------------------------------------------------------------librerias de TIEMPO NTP
#include <TimeLib.h>                                                  //TimeTracking
#include <WiFiUdp.h>                                                  //UDP packet handling for NTP request
//----------------------------------------------------------------------Librerias de manejo de setup de redes 
#include <ESP8266WebServer.h>                                         //Libreira de html para ESP8266
#include <DNSServer.h>                                                //Libreria de DNS para resolucion de Nombres
#include <WiFiManager.h>                                              //https://github.com/tzapu/WiFiManager
//----------------------------------------------------------------------Librerias de Codigo de Lectora RFID
#include <SoftwareSerial.h>                                           //Libreria de SoftwareSerial para recibir data del sensor
#include "settings.h"                                                 //Libreria local que contiene valores configurables de conexion
//----------------------------------------------------------------------Poner el Pin de ADC en modo de sensar el voltaje da la bateria
ADC_MODE(ADC_VCC);                                                    //Se opne el pin A0 en modo de Lectura interna 1.8V
//----------------------------------------------------------------------Variables de verificacion de fallas de capa de conexion con servicio
int failed, sent, published;                                          //Variables de conteo de envios 
//----------------------------------------------------------------------Los Pines para la conexion del modulo de RFID TTL son (14,12); //Rx, TX Arduino --> Tx, Rx En RDM6300
SoftwareSerial swSer(14, 13, false, 256);                             //Rx, TX Arduino --> Tx, Rx En RDM6300
//----------------------------------------------------------------------Inicio de cliente UDP
WiFiUDP Udp;                                                          //Cliente UDP para WIFI
//----------------------------------------------------------------------Codigo para estblecer el protocolo de tiempo en red NTP
const int NTP_PACKET_SIZE = 48;                                       //NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                                   //Buffer to hold incoming & outgoing packets
boolean NTP = false;                                                  //Bandera que establece el estado inicial del valor de NTP
//----------------------------------------------------------------------Variables del servicio de envio de datos MQTT
char server[] = "iot.flatbox.io";       //EL ORG es la organizacion configurada para el servicio de Bluemix
//char authMethod[] = "use-token-auth";                                 //Tipo de Autenticacion para el servicio de Bluemix (la calve es unica por cada nodo)
//char token[] = TOKEN;                                                 //Variable donde se almacena el Token provisto por el servicio (ver Settings.h)
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;             //Variable de Identificacion de Cliente para servicio de MQTT Bluemix 
//----------------------------------------------------------------------Declaracion de Variables Globales (procuar que sean las minimas requeridas.)
int FWVERSION = 1;                                                    //Variable configurable remotamente sobre la vesion e firmware
int SleepState = 0;
unsigned long lastPublishMillis;                                      //Variable para llevar conteo del tiempo desde la ultima publicacion 
String ISO8601;                                                       //Variable para almacenar la marca del timepo (timestamp) de acuerdo al formtao ISO8601
//----------------------------------------------------------------------definir Parametros de Lector de RFID
int readVal = 0;                                                      // individual character read from serial
int counter = -1;                                                     // counter to keep position in the buffer
char tagId[12];                                                       // final tag ID converted to a string
String OldTagRead;                                                    //VAriable para guardar la ultima tag leida y evitar lecturas consecutivas
unsigned int readData[12];                                            //Variable para el alamcenamiento de la lectura del TAG (12 DIGITOS)
//----------------------------------------------------------------------Variables Para casignacion de pines para los led RGB
                                                                      //NOTA PARA EL MODULO ESP8266 NO se podra asignar el PIN 16 debido a que este resete el modulo
int verde = 13;                                                       //Asignacion de PIN 13 Para el color Verde del LeD RGB
int rojo = 15;                                                        //Asignacion de PIN 15 Para el color Verde del LeD RGB
int azul = 2;                                                         //Asignacion de PIN 2 Para el color Verde del LeD RGB (ping por defecto para ESP TOY)
//----------------------------------------------------------------------Variables Propias del CORE ESP8266 Para la administracion del Modulo
String NodeID = String(ESP.getChipId());                              //Variable Global que contiene la identidad del nodo (ChipID) o numero unico
//----------------------------------------------------------------------Funcion remota para administrar las actulizaciones remotas de las variables configurables desde IBMbluemix
void handleUpdate(byte* payload) {                                    //La Funcion recibe lo que obtenga Payload de la Funcion Callback que vigila el Topico de subcripcion (Subscribe TOPIC)
  StaticJsonBuffer<1536> jsonBuffer;                                  //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payload);          //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
    }                                                                 //se cierra el condicional
  Serial.println(F("handleUpdate payload:"));                         //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
  Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
  JsonObject& d = root["d"];                                          //Se define el objeto "d" como  la raiz del mensaje JSON
  JsonArray& fields = d["fields"];                                    //se define el arreglo "fields" del JSON
  for(JsonArray::iterator it=fields.begin();                          //se daclara una rutina para buscar campos dentro del arreglo 
      it!=fields.end();                                               //si no se encuentra lo que se busca se termina la busqueda
      ++it) {                                                         //se busca el siguiente campo
        JsonObject& field = *it;                                      //se asigna lo que tenga el iterador de campos field
        const char* fieldName = field["field"];                       //se crea l avariable nombre de campo
        if (strcmp (fieldName, "metadata") == 0) {                    //Se confirma valida si el campo contiene "metadata"
          JsonObject& fieldValue = field["value"];                    //Se asigna el valor de campo a el objeto de JSON
          if (fieldValue.containsKey("publishInterval")) {            //Si el Valor del campo contiene la LLave "publishInterval"
            publishInterval = fieldValue["publishInterval"];          //asignar ese valor a la variable global "publishInterval"
            Serial.print(F("publishInterval:"));                      //se imprime un mensaje con ka variable que acaba de modificarse remotamente
            Serial.println(publishInterval);                          //se imprime el nuevo valor de la variable actualizada
          }
        }
        if (strcmp (fieldName, "deviceInfo") == 0){                   //Se confirma valida si el campo contiene "deviceInfo"                  
          JsonObject& fieldValue = field["value"];                    //Se asigna el valor de campo a el objeto de JSON
          if (fieldValue.containsKey("fwVersion")) {                  //Si el Valor del campo contiene la LLave "fwVersion"
            FWVERSION = fieldValue["fwVersion"];                      //asignar ese valor a la variable global "FWVERSION"
            Serial.print(F("fwVersion:"));                            //se imprime un mensaje con ka variable que acaba de modificarse remotamente
            Serial.println(FWVERSION);                                //se imprime el nuevo valor de la variable actualizada
          }
        }
      }
}

//----------------------------------------------------------------------Funcion remota para mandar a dormir el esp despues de enviar un RFID
void handleResponse (byte* payloadrsp) {
  StaticJsonBuffer<200> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payloadrsp);       //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
  }                                                                   //se cierra el condicional
  
  Serial.println(F("handleResponse payload:"));                       //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
  Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
  
  JsonObject& d = root["d"];                                          //Se define el objeto "d" como  la raiz del mensaje JSON
  JsonArray& fields = d["fields"];                                    //se define el arreglo "fields" del JSON
  for(JsonArray::iterator it=fields.begin();                          //se daclara una rutina para buscar campos dentro del arreglo 
      it!=fields.end();                                               //si no se encuentra lo que se busca se termina la busqueda
      ++it) {                                                         //se busca el siguiente campo
        JsonObject& field = *it;                                      //se asigna lo que tenga el iterador de campos field
        const char* fieldName = field["field"];                       //se crea l avariable nombre de campo
        if (strcmp (fieldName, "metadata") == 0) {                    //Se confirma valida si el campo contiene "metadata"
          JsonObject& fieldValue = field["value"];                    //Se asigna el valor de campo a el objeto de JSON
          if (fieldValue.containsKey("SleepState")) {                      //Si el Valor del campo contiene la LLave "SleepState"
            SleepState = fieldValue["SleepState"];                    //asignar ese valor a la variable global "SleepState"
            Serial.print(F("SleepState:"));                                 //se imprime un mensaje con ka variable que acaba de modificarse remotamente
            Serial.println(SleepState);                                    //se imprime el nuevo valor de la variable actualizada
            if (SleepState == 0){
              Serial.println("Device sent to sleep");
              delay(100);
              ESP.deepSleep(0);
            }
            if (SleepState == 1){
              Serial.println("NO AUTH");
              digitalWrite(verde, LOW);
              digitalWrite(rojo, HIGH);
              digitalWrite(azul, LOW);
              delay(200);
              digitalWrite(verde, LOW);
              digitalWrite(rojo, LOW);
              digitalWrite(azul, LOW);
              delay(500);
            }
            if (SleepState == 2){
              Serial.println("AUTH");
              digitalWrite(verde, HIGH);
              digitalWrite(rojo, LOW);
              digitalWrite(azul, LOW);
              delay(500); 
              digitalWrite(verde, LOW);
              digitalWrite(rojo, LOW);
              digitalWrite(azul, LOW);
              delay(500);
            }
          }
        }
      }
}

//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength){//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
  Serial.print(F("callback invoked for topic: "));                    //Imprimir un mensaje señalando sobre que topico se recibio un mensaje
  Serial.println(topic);                                              //Imprimir el Topico
  
  if (strcmp (responseTopic, topic) == 0) {                            //verificar si el topico conicide con el Topico responseTopic[] definido en el archivo settings.h local
    handleResponse(payload);
    //return; // just print of response for now                         //Hacer algo si conicide (o en este caso hacer nada)
  }
  
  if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    ESP.restart();                                                    //Emitir comando de reinicio para ESP8266
  }
  
  if (strcmp (updateTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico updateTopic[] definido en el archivo settings.h local
    handleUpdate(payload);                                            //enviar a la funcion handleUpdate el contenido del mensaje para su parseo.
  } 
}
//----------------------------------------------------------------------definicion de Cliente WIFI para ESP8266 y cliente de publicacion y subcripcion
WiFiClient wifiClient;                                                //Se establece el Cliente Wifi
PubSubClient client(server, 1883, callback, wifiClient);              //se establece el Cliente para el servicio MQTT
//----------------------------------------------------------------------Funcion de Conexion a Servicio de MQTT
void mqttConnect() {
 if (!!!client.connected()) {                                         //Verificar si el cliente se encunetra conectado al servicio
  Serial.print(F("Reconnecting MQTT client to "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
  Serial.println(server);                                             //Imprimir la direccion del servidor a donde se esta intentado conectar 
  while (!!!client.connect(clientId)) {            //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    digitalWrite(verde, HIGH);
    digitalWrite(rojo, HIGH);
    delay(500);                                                       //esperar 500 milisegundos entre cada intento de conexion al servicio de  MQTT
    digitalWrite(verde, LOW);
    digitalWrite(rojo, LOW);
  }
  Serial.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}
//----------------------------------------------------------------------Funcion encargada de subscribir el nodo a los servicio de administracion remota y de notificar los para metros configurables al mismo
void initManagedDevice() {
  if (client.subscribe("iotdm-1/response")) {                         //Subscribir el nodo al servicio de mensajeria de respuesta
    Serial.println(F("subscribe to responses OK"));                   //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to responses FAILED"));               //Si no se logra la subcripcion imprimir un mensaje de error
  }
  
  if (client.subscribe(rebootTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to reboot FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  
  if (client.subscribe("iotdm-1/device/update")) {                    //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to update OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to update FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
  }

  StaticJsonBuffer<1536> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["publishInterval"] = publishInterval;
  JsonObject& supports = d.createNestedObject("supports");
  supports["deviceActions"] = true;  
  //JsonObject& deviceInfo = d.createNestedObject("deviceInfo");
  //deviceInfo["fwVersion"] = FWVERSION;
  char buff[1536];
  root.printTo(buff, sizeof(buff));
  Serial.println(F("publishing device metadata:"));
  Serial.println(buff);
  
  if (client.publish(manageTopic, buff)) {
    Serial.println(F("device Publish ok"));
  }else {
    Serial.println(F("device Publish failed:"));
  }
}

//--------  send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


//-------- Funcion para obtener el paquee de TP y procesasr la fecha hora desde el servidor de NTP
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      NTP = true;
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}

//-------- Fucnion para la apertura y conexion de paquetes de UDP para el servicio de  NTP
void udpConnect() {
  Serial.println(F("Starting UDP"));
  Udp.begin(localPort);
  Serial.print(F("Local port: "));
  Serial.println(Udp.localPort());
  Serial.println(F("waiting for sync"));
   setSyncProvider(getNtpTime);
}

//--------  anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  Serial.println(F("empezando"));
  if (!  wifiManager.autoConnect("flatwifi")) {
    if (!wifiManager.startConfigPortal("FlatWifi")) {
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
}

void flashblue(){
  digitalWrite(azul, HIGH);
  delay(500);
  digitalWrite(azul, LOW);
}

//------------------------------- setup  ---------------------------------------------//
//-------- Funcion Principal de inicializacion de rutina en modulo 
void setup() {
  pinMode(rojo, OUTPUT);
  pinMode(verde, OUTPUT);
  pinMode(azul, OUTPUT);
  digitalWrite(rojo, HIGH);
  digitalWrite(verde, LOW);
  digitalWrite(azul, HIGH);
  Serial.begin(115200);// iniciamos el puerto de comunicaiones en pines 0 y 1 para el envio de mensajes de depuracion y error
  Serial.println(F("initializing RFID WIFI READER Setup"));
  swSer.begin(9600);  //inciamos el puerto serial por software para la lectora RFID (puertos 12 y13) a 9600bps.
  
  //--------  Funcion de Conexion a Wifi
  while (WiFi.status() != WL_CONNECTED) {//conectamos al wifi si no hay la rutina iniciara una pagina web de configuracion en la direccion 192.168.4.1 
    wifimanager();
    delay(1000);
  }
  Serial.print(F("nWiFi connected, IP address: "));
  Serial.println(WiFi.localIP());
  Serial.println();//dejamos una linea en blanco en la terminal 
  //una vez contados al Wifi nos aseguramos tener la hora correcta simepre
  Serial.println(F("Connected to WiFi, Sync NTP time")); //mensaje de depuracion para saber que se intentara obtner la hora
  while (NTP == false) {
    flashblue();
    udpConnect ();//iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
    delay(500);
  }
  Serial.println(F("Time Sync, Connecting to mqtt sevrer"));
  mqttConnect();//Conectamos al servicio de Mqtt con las credenciales provistas en el archivo "settings.h"
  Serial.println(F("Mqtt Connection Done!, sending Device Data"));
  initManagedDevice();//inciamos la administracion remota desde Bluemix
  Serial.println(F("Finalizing Setup")); //enviamos un mensaje de depuracion 
  digitalWrite(rojo, LOW);
  digitalWrite(verde, LOW);
  digitalWrite(azul, LOW);  
  delay(100);
}

//-------- funcion que procesa como desplegar y transmitir la hora de acuerdo al formato del ISO8601
void ISO8601TimeStampDisplay(){
  // digital clock display of the time
  ISO8601 = String (year(), DEC);
  ISO8601 += "-";
  ISO8601 += month();
  ISO8601 += "-";
  ISO8601 += day();
  ISO8601 +="T";
  ISO8601 += hour();
  ISO8601 += ":";
  ISO8601 += minute();
  ISO8601 += ":";
  ISO8601 += second();
  ISO8601 +="-06:00";
 }

time_t prevDisplay = 0; // Cuando fue actualizada la hora del reloj

//-------- Funcion de verificacion de hora y formato de la misma
void checkTime () {
   if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      ISO8601TimeStampDisplay();  
    }
  }
}

//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//

boolean publishData(String IDModulo, String tagread, String Tstamp) {
  digitalWrite(azul, LOW);
  if (OldTagRead != tagread){
    OldTagRead = tagread;
    StaticJsonBuffer<250> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& data = d.createNestedObject("data");
    data["chipID"] = IDModulo;
    data["tag"] = tagread;
    data["tstamp"] = Tstamp;
    char Mqttdata[250];
    root.printTo(Mqttdata, sizeof(Mqttdata));
    Serial.println(F("publishing device metadata:")); 
    Serial.println(Mqttdata);
    sent ++;
    if (client.publish(publishTopic, Mqttdata)){
      Serial.println(F("Publish OK"));
      digitalWrite(verde, HIGH);
      delay(500);
      digitalWrite(verde, LOW);
      published ++;
      failed = 0; 
      }else {
        Serial.println(F("Publish FAILED"));
        digitalWrite(verde, HIGH);
        digitalWrite(rojo, HIGH);
        delay(500);
        digitalWrite(verde, LOW);
        digitalWrite(rojo, LOW);
        failed ++;
        OldTagRead = "";
      }
  }else{
    Serial.println("Este es una lectura consecutiva");
  }
    digitalWrite(verde, LOW);
    digitalWrite(rojo, LOW);
    digitalWrite(azul, LOW);
}

//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishManageData (String Sid0, int env, int fail){
  float vdd = ESP.getVcc()/1000 ;
  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["device_name"] = Sid0;
  metadata["Bateria"] = vdd;
  metadata["enviados"] = env;
  metadata["fallidos"] = fail;
  char buff[1024];
  root.printTo(buff, sizeof(buff));
  Serial.println("publishing device metadata:");
  Serial.println(buff);
  if (client.publish(manageTopic, buff)) {
    Serial.println("Manage Publish ok");
     published ++;
     failed = 0; 
  }else {
    Serial.print(F("Manage Publish failed:"));
    failed ++;
  }
}

// this function clears the rest of data on the serial, to prevent multiple scans
void clearSerial() {
   while (Serial.read() >= 0) {
    ; // do nothing
  }
  Serial.flush();
  while (swSer.read() >= 0) {
    ; // do nothing
  }
  swSer.flush();
  //asm volatile ("  jmp 0");
}

// convert the int values read from serial to ASCII chars
void parseTag() {
  int i;
  for (i = 0; i < 12; ++i) {
    tagId[i] = readData[i];
  }
  tagId[12] = 0;
}

void ParseTag(){
  if (swSer.available() > 0) {
    // read the incoming byte:
    readVal = swSer.read();
    digitalWrite(azul, LOW);    
    switch (readVal) {
      case 2:
      counter = 0; // start reading
      break;
      case 3:
      // process the tag we just read
      parseTag();
      checkTime();
      publishData(NodeID, tagId, ISO8601);
      // clear serial to prevent multiple reads
      clearSerial();
      // reset reading state
      counter = -1;
      break;
      default: 
      // save valuee
      readData[counter] = readVal;
      // increment counter
      ++counter;
      break;
    }
  }
}

//-------- Funcion de Ciclo
void loop() {
  digitalWrite(azul, HIGH);
  
  ParseTag();
 
  // VERIFICAMOS CUANTAS VECES NO SE HAN ENVIOADO PAQUETES (ERRORES)
   if (failed >= FAILTRESHOLD){
    failed =0;
    published =0;
    sent=0;    
    ESP.reset();
  }
  
  //si ha pasado el tiempo establecido entonces publicar los datos
  if(millis() - lastPublishMillis > UPDATESENDTIME) {
    String Msg = String ("MSGfailed" + failed); 
    Serial.println(Msg);
    publishManageData(NodeID, published, failed);
    UPDATESENDTIME = 30*60*1000UL; 
    lastPublishMillis = millis(); //Actulizar la ultima hora de envio
  }
  
  //verificar que el cliente de Conexion al servicio se encuentre conectado
  if (!client.loop()) {
    mqttConnect();// si el clinete al servicio MQTT se desconecto volver a conectarlo.
  }
}
