
/*
 * date:8/12/2016
 * editor:Edwin Kestler
 * added: String FirmwareVersion= "V0.01";                                        //read in chage history
 * added file: "change_history" in sketch directory
 * replaced: digitalWrites whit color functions in: void handleResponse (byte* payloadrsp) function:
 * from:
             * if (SleepState == 1){
                          digitalWrite(verde, HIGH);
                          digitalWrite(rojo, LOW);
                          digitalWrite(azul, LOW);              
                        }
 * to:
             *  *  if (SleepState == 1){
                          Green();              
                        }
 * 
 * Test && Compile: OK.
 * removed: digitalWrite(azul, LOW);   in void ParseTag() function.
 * from:
             * void ParseTag(){
              if (swSer.available() > 0) {
                  // read the incoming byte:
                  readVal = swSer.read();
                  digitalWrite(azul, LOW);    
 *to:
                 *void ParseTag(){
                  if (swSer.available() > 0) {
                      // read the incoming byte:
                      readVal = swSer.read();
                      switch (readVal) {
 *added: comments for functions Loop(),NormalReset()...etc
             *  //--------------------------------------------------------------------------Funcion de Reinicio automatico cada 24h (prevenir buffer overflow.!!!------------------------------------------------------------------------------
 * 
 * Test: Compile:ok Upload:OK
 * 
 * changed: cambio de valores predeterminados de servidor de mqtt server. a los valores del ip del servidor ciam (internet por intranet)
             * de: char server[] = "iotarduinodaygt.flatbox.io";       //EL ORG es la organizacion configurada para el servicio de Bluemix
             * por char server[] = "192.168.7.103";       //EL ORG es la organizacion configurada para el servicio de Bluemix
             * 
 * Test: Compile:ok,Upload:ok,test: changed flows mqtt in node red service, slow server, needed reset to activate changes in server. after that (ok)
 * 
 * changed: added topic to witch priunted in serial output:   Serial.println(F("publishing device data to manageTopic:"));
 *            Serial.println(F("publishing device data to manageTopic:"));
 *   
 *  Test:Compile: ok Upload:ok
 *  
 *  Changed: all timers for one long timer of second and every other for a multiplier. in settings.h
 *            //Variables de Reloj para espera y envio de paquetes de MQTT
              unsigned long UInterval     = 1000UL; //Variable configurable remotamente sobre el interbalo de publicacion
              //unsigned long publishInterval     = 60*1000UL; //Variable configurable remotamente sobre el interbalo de publicacion
              //unsigned long warningInterval     = 1000UL; //Variable configurable remotamente sobre el interbalo de publicacion
              //unsigned long UPDATESENDTIME      = 60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos
              //unsigned long NormalResetTIME     = 60*60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos
               changed every timer for a multiplier of a single second timer in fuctions time dependant:
               void updateDeviceInfo(){
                  if(millis() - lastUPDATEMillis > 60 * UInterval) {
                    lastUPDATEMillis = millis(); //Actulizar la ultima hora de envio
 *   
 * added: total de mensajes enviados en funcion de datos de manejo de dispositivo: void publishRF_ID_Manejo (String IDModulo,String MSG,float vValue,int RSSIV, int env, int fail,String Tstamp){

            *   data["enviados"] = sent;

 * added: se añadio el contador de envios para todas las funciones de envio de datos.
           *root.printTo(buff, sizeof(buff));
            Serial.println(F("publishing device manageTopic metadata:"));
            Serial.println(buff);
            sent++;
            if (client.publish(manageTopic, buff)) {
              Serial.println(F("device Publish ok"));
            }else {
              Serial.println(F("device Publish failed:"));
            }
 *added: se agrego  la identificacion de eventos de boton y eventos de lectura, para poder diferenciar el total de eventos y hacer cuadre de los mismos.
          *int IdEventoB= 0;
          int IdEventoT=0;
 *added: se agrego a cada evento la funcion para el envio del identificador de eventos.
           *boolean publishRF_ID_Lectura(String IDModulo, String Tstamp, String tagread) {
            if (OldTagRead != tagread){
              OldTagRead = tagread;
              IdEventoT ++;
              String IDEventoT = String (NodeID + IdEventoT);
              StaticJsonBuffer<250> jsonBuffer;
              JsonObject& root = jsonBuffer.createObject();
              JsonObject& d = root.createNestedObject("d");
              JsonArray& tagArray = d.createNestedArray("tagdata");
              JsonObject& data = tagArray.createNestedObject();
              data["ChipID"] = IDModulo;
              data["IDeventoTag"]= IDEventoT;
 *date:10/12/2016
 * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.02";                                        //read in chage history
 * added: Desplegar el CHIPID al inicio del script SETUP para poder configurar este en el client ID del servidor MQTT OJO
 * Esto debido a que dos clientes de MQTT NO pueden tener el mismo clientID
 * 
               *Serial.println(F("")); 
                Serial.print(F("initializing RFID WIFI READER Setup, CHIPID:"));
                Serial.println(NodeID);
               *
 *Compile:OK, Test OK
 *
 *date:12/12/2016
 * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.03";          
 * Changed From JSON ARRAY to JSONOBJECT in data format for simplified use in backend server:
 * for all Fucntions that send data to MS-SQL SERVER
 * 
 * void publishRF_Boton(String IDModulo, String BEventID, String Tstamp) 
 * boolean publishRF_ID_Lectura(String IDModulo, String Tstamp, String tagread) {
 * void publishRF_ID_Manejo (String IDModulo,String MSG,float vValue,int RSSIV, int env, int fail,String Tstamp){
 * 
 * in the fallowing form:
 * 
               *  void publishRF_Boton(String IDModulo, String BEventID, String Tstamp) {
                StaticJsonBuffer<500> jsonBuffer;
                JsonObject& root = jsonBuffer.createObject();
                JsonObject& d = root.createNestedObject("d");
                JsonObject& botondata = d.createNestedObject("botondata");
                botondata["ChipID"] = IDModulo;
                botondata["IDEventoBoton"] = BEventID;
                botondata["Tstamp"] = Tstamp;
                char MqttBotondata[500];
                root.printTo(MqttBotondata, sizeof(MqttBotondata));
                Serial.println(F("publishing device publishTopic metadata:")); 
                Serial.println(MqttBotondata);
                
 *Compile: OK, Test: OK
 *date:13/12/2016
 * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.04";          
 * Changed Warning for Reconnect Function.
 * 
                 *void reconnect() {
                  // Loop until we're reconnected
                  while (!client.connected()) {
                    Serial.print(F("Attempting MQTT connection..."));
                    if (client.connect(clientId)) {
                      Serial.println(F("connected"));
                      } else {
                      flashWhite();
 *Compile: OK; test OK;
 *
 **date:13/12/2016
 * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.04";          
 * Changed Warning for Reconnect Function, warning disabled the button led, warning from white to flashpurple for continous disconection.
 * 
               * void botonCIAM(){
                lecturaBoton = digitalRead(BotonCiam);
                digitalWrite(azul, EstadoBoton);
                if (lecturaBoton == 1 && UltimoEstadoBoton == 0 && millis() - RetardoHora > RetardoCambio){
 *
                 *void reconnect() {
                  // Loop until we're reconnected
                  while (!client.connected()) {
                    Serial.print(F("Attempting MQTT connection..."));
                    flashWhite();
                    BEEP();
                    if (client.connect(clientId)) {
                      Serial.println(F("connected"));
                      } else {
                      flashPurple();
                      BEEP();
                      Serial.print(F("failed, rc="));
                      Serial.print(client.state());
                      Serial.println(F(" try again in 3 seconds"));
                      // Wait 3 seconds before retrying
                      delay(3000);
                    }
                  }
                }
                *date:13/12/2016
                *
 * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.05";          
 * date:13/12/2016
 * Changed: sequito lo de reenvio de codigo por falta de red, se cambio el para metro de falta de wifi a -77Dbm
 * se actulizo la tabla de colores.
 * Changed: SleepState por DeviceState:
                 * if (fieldValue.containsKey("State")) {                      //Si el Valor del campo contiene la LLave "DeviceState"
                            DeviceState = fieldValue["State"];                    //asignar ese valor a la variable global "DeviceState"
                            Serial.print(F("DeviceState:"));                                 //se imprime un mensaje con ka variable que acaba de modificarse remotamente
                            Serial.println(DeviceState);                                    //se imprime el nuevo valor de la variable actualizada
                            if (DeviceState == 0){
                              Serial.println(F("lights off"));
                              EstadoBoton = 0;
                              lightsOff();
                              delay(100);
                              //ESP.deepSleep(0);
                            }
                            if (DeviceState == 1){
                             Green();              
                            }
                            if (DeviceState == 2){
                             Red();
                            }
                            if (DeviceState == 3){
                              Blue();
                            }
                            if (DeviceState == 4){
                              White();
                            }
                             if (DeviceState == 5){
                              Purple();
                            }
                             if (DeviceState == 7){
                              flashBlue();
                            }
                             if (DeviceState == 8){
                              flashRed();
                            }
                             if (DeviceState == 9){
                              flashGreen();
                            }
                             if (DeviceState == 10){
                              flashPurple();
                            }
                             if (DeviceState == 11){
                              flashWhite();
                            }
                          }
                        }
* * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.06";          
 * date:19/01/2017
 * Changed: se modifico el parametro de tiempo de envio a cada 5 minutos en lugar de acada minuto para comprobar tiempo de vida de bateria:
                 *  if(millis() - lastUPDATEMillis > 5 * 60 * UInterval) {
                      lastUPDATEMillis = millis(); //Actulizar la ultima hora de envio
                      String msg = ("on");
                      float VBat = Bateria();
                      int WifiSignal = WiFi.RSSI();
                      checkTime();    
                      publishRF_ID_Manejo(NodeID,msg, VBat, WifiSignal, published, failed, ISO8601);        //publishRF_ID_Manejo (String IDModulo,String MSG,float vValue, int fail,String Tstamp)
                      if (WiFi.RSSI() < -75){
 * 
* * * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V0.07";          
 * date:19/01/2017
 * Changed: sse agrego la rutina de light sleep ´mopdem sleep para ahorrar bateria:
                                           * extern "C" {
                            #include "user_interface.h"
                          }

   in setup
                          //wifi_set_sleep_type(NONE_SLEEP_T);
                          wifi_set_sleep_type(MODEM_SLEEP_T);
                          wifi_set_sleep_type(LIGHT_SLEEP_T);
 * 
*  * * editor:Edwin Kestler
 * Changed: String FirmwareVersion= "V1.0";          
 * date:07/02/2017
 * Changed: se cambio el numero de version a 1.0 debido a que entra en produccion
 * se camio la IP del NTP y del MQTT ala IP provista por Ernesto para la puesta en produccion.
 * Changed: also changed the response MQTT feed to include the Chipid to response 
 *      in Settings.h
 *      from:
 *      const char responseTopic[] ="iotdm-1/response";
 *       to:
 *          const char responseTopic[] ="iotdm-1/response/13697730";
 * 
*/
