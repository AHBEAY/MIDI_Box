#include <BLEDevice.h> //////////////////////////////////////////////////////////////////////////////
#include <BLEServer.h> //////////////////necessary librarries////////////////////////////////////////
#include <BLEUtils.h> ///////////////////for MIDI_BLE device///////////////////////////////////////
#include <BLE2902.h>  ////////////////////////////////////////////////////////////////////////////
#include<BLEscan.h> ////////////////////////////////////////////////////////////////////////////

#define MIDI_SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700" //BLE_MIDI device service_UUID we want to create
#define MIDI_CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3" //BLE_MIDI characteresric_UUID we want to create

// The remote service we wish to connect to.
static BLEUUID serviceUUID("03b80e5a-ede8-4b33-a751-6ce34ec4c700");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("7772e5db-3868-4112-a1a9-f2669d106bf3");

volatile byte byte1, byte2, byte3, byteExc, MIDI_Values[]= {}; // for saving the status, Data1, Data2 values
volatile int SizeOfMidi_msg= 0;// Size of the serial MIDI packet were going to send over BLE

///////////////////////////////////////////////////////////////////// this function permits to treat all kinds of MIDI-Messages(1-3 Bytes)
///////////////////////Receive_SerialMIDI/////////////////////////////  except the System-Exclusive-Messages
void Receive_SerialMIDI() {

  byte1 = Serial2.read(); //Status byte
  if(byte1 != 0b11110000){ //all kind of messages except System-Exclusive messages
    switch (Serial2.available()) { //Serial.available gives the number of bytes in the Serial-UART Port + Buffer
      case 2: { // 3bytes message
         
          byte2 = Serial2.read(); //Data1 byte
          byte3 = Serial2.read(); //Data2 byte
          byte Status = byte1; //Status byte 
          byte Data1 = byte2; //Data1 byte
          byte Data2 = byte3; //Data2 byte
        
          if ( ((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7) && ((byte3 xor 0b10000000) >> 7)) { // verify MIDI-message (1-------, 0-------, 0-------)
           
            MIDI_Values[0] = Status; // This array will be used for the BLE-send
            MIDI_Values[1] = Data1;  // the two bytes of MIDI-BLE will be added afterwards
            MIDI_Values[2] = Data2;  
            SizeOfMidi_msg= 3;       // length of the midi nessage (how many bytes)
          }
        break;
      }
      case 1: { // 2bytes message
          byte byte2 = Serial2.read(); //Data1 byte
          byte Status = byte1; //Status byte 
          byte Data1 = byte2; //Data1 byte
        
          if (((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7)) { // verify MIDI-message (1-------, 0-------)
            MIDI_Values[0] = Status;  // This array will be used afterwards for the BLE-send
            MIDI_Values[1] = Data1;   //the two bytes of MIDI-BLE will be added afterwards
            SizeOfMidi_msg= 2;        // length of the midi nessage (how many bytes)
          }
        break;
      }
      case 0: { // 1byte message
        byte Status= byte1;
        
        if ((byte1 & 0b10000000) >> 7) { // verify MIDI-message (1-------)
          MIDI_Values[0] = Status; // This array will be used afterwards for the BLE-send
          SizeOfMidi_msg= 1;        // length of the nidi nessage (how many bytes)
        }
        break;
      }
      default: //In this case the MIDI sent MIDI message (from the MIDI-Instrument is wrong) do nothing 
        break;
    }
  }
  else { //else systeme exclusive messages
    MIDI_Values[0]= byte1; // append the first Byte of MIDI-Exclusive-message ti MIDI_Value array
    SysExc(); // break this message eith the fonction SysExc
  }
}
/////////////////////////////////////////////////////////////
///////fonction to treart the serial Exclusive message///////
void SysExc(){
  int i= 1; // i= 0 for the first byte= 0b11110000
  do { // repeat this loop until we reach the final byte of system exlusive message
    byteExc= Serial2.read(); // read the serial value
    MIDI_Values[i]= byteExc;  // save it in the array
  } while (byteExc != 0b11110111); // last message= 0b11110111
  SizeOfMidi_msg= i; //size of MIDI system exclusive message
}
/////////////////////////////////////////////////////////////
///////fonction to treart the serial Exclusive message///////

//////////////////////////////////////////////////////////////////////
///////////////////////Receive_SerialMIDI/////////////////////////////


//////////////////////////////////////////////////////////////////////
//////////////////////////Server/////////////////////////////////////
BLEServer* pServer = NULL; //This pointer is responsable for the Server fonctions
BLECharacteristic* pCharacteristic = NULL; //This pointer is responsable for the all events which have relationship with the characteristics of the server
bool deviceConnected = false;

//////////////////////////////////////////////////////////
//////MIDI_BLE_Server connect/disconnect callbacks//////// 
class MyServerCallbacks: public BLEServerCallbacks { 
    void onConnect(BLEServer* pServer) { //Callback fonction when the server is connected to a new device
      deviceConnected = true;
      BLEDevice::startAdvertising(); // start advertising when connected
    };

    void onDisconnect(BLEServer* pServer) { //Callback fonction when the server is disconnected from a client
      deviceConnected = false;
    }
};
//////MIDI_BLE_Server connect/disconnect callbacks////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
///////////////////onWrite////////////////////////////////
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) { //If a connected client wrote MIDI-message on the carachteristic, this function will take place
      std::string MIDI_data = pCharacteristic->getValue(); //read the wrote value in a char
      int length= MIDI_data.length(); // length of the wrote BLE-MIDI message

      if ((length >= 3) & ((MIDI_data[0] & 0b10000000) >> 7) && ((MIDI_data[1] & 0b10000000) >> 7)) { //length of MIDI-BLE message needs to be greater as 3 (MIDI message + 2bytes), byte1 10------, byte2 1------- )
        byte byte1= MIDI_data[2]; // Status Byte
        
        if (byte1 != 0b11110000){ // all kind of message except system exclusive messages
          
        switch(length-2){ // size of the actual MIDI message sent over BLE = size of the BLE-packet - 2
        case 3:{
          
          byte byte2= MIDI_data[3]; // Data1 Byte
          byte byte3= MIDI_data[4]; // Data2 Byte
          byte Status= byte1; // Status Byte
          byte Data1= byte2;  // Data1 Byte
          byte Data2= byte3;  //0Data2 Byte
      
          if (((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7) && ((byte3 xor 0b10000000) >> 7)){ // verify MIDI-message (1-------, 0-------, 0-------)
            //Serial write to send the message from the ESP32 to MIDI-instrument 
            Serial2.write(Status); // Status Byte
            Serial2.write(Data1);  // Data1 Byte
            Serial2.write(Data2);  // Data2 Byte
          }
          break;}
        
        case 2:{
          byte byte2= MIDI_data[3]; // Data1 Byte
          byte Status= byte1; // Status Byte
          byte Data1= byte2;  // Data1 Byte
      
          if (((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7)){ // verify MIDI-message (1-------, 0-------)
            //Serial write to send the message from the ESP32 to MIDI-instrument
            Serial2.write(Status);
            Serial2.write(Data1);
          }
          break;}
        
        case 1:{
          byte Status= byte1; // Status Byte
         
          if (((byte1 & 0b10000000) >> 7)){
            //Serial write to send the message from the ESP32 to MIDI-instrument
            Serial2.write(Status);
          }
          break;}

        default:
          break;
          }
        }
      
    else { //BLE-Server when receives a sys-exc-message serial write until we reach the stop byte
      Serial2.write(byte1);
      int i= 3; // i=0 headerbyte, i=1 timestampbyte, i= 2 first byte of MIDI system exlusive message
      
      do{  // read each byte of the MIDI exclusive message  until we reach the last byte 0b11110111
        Serial2.write(MIDI_data[i]); //Serial write of this byte 
        i++; //increment i to move to the next byte
      }while(MIDI_data[i] != 0b11110111);// last byte of systeme exclusive message
        } 
      }
    } 
};
///////////////////onWrite////////////////////////////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
///////////////////Server-definition//////////////////////
void Server() {
  BLEDevice::init("MIDI_BOX"); // Name of the BLE_Device
  
  pServer = BLEDevice::createServer(); // create the server from this BLE::device
  pServer->setCallbacks(new MyServerCallbacks()); //callback for connect/disconnect

  BLEService *pService = pServer->createService(MIDI_SERVICE_UUID); //assocoiate this defined service to the server MIDI-BLE

  pCharacteristic = pService->createCharacteristic( //assocoiate this defined Characteristic to the server MIDI-BLE
                      MIDI_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   | //Server: reads when Client writes
                      BLECharacteristic::PROPERTY_WRITE  | //Client: write
                      BLECharacteristic::PROPERTY_NOTIFY | //Server: notify Clients with a new value
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );
  pCharacteristic->addDescriptor(new BLE2902()); // descriptor for the Client Characteristic Configuration which has a UUID of 0x2902.
  pCharacteristic->setCallbacks(new MyCallbacks()); //onWrite callback
  
  pService->start(); //start the BLE-service
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(MIDI_SERVICE_UUID);
  pAdvertising->setScanResponse(true); // set to true to make it possible: making connections wtih the BLE_BOX
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
};
///////////////////Server-definition//////////////////////
/////////////////////////////////////////////////////////

//////////////////////////////////
//////send_MIDI_BLE_Server////////
void Send_BLEServer() { //fonction to send a serial MIDI packet over BLE
  uint8_t packet[SizeOfMidi_msg+2]= {}; //this packet will be sent from the MIDI-server over BLE (size+2 for header- and timestamp bytes)

  auto t = millis(); //timee of sending the BLE-message
  uint8_t headerByte = (1 << 7) | ((t >> 7) & ((1 << 6) - 1)); //header byte 1 0 - - - - - - 
  uint8_t timestampByte = (1 << 7) | (t & ((1 << 7) - 1)); //timestmp 1 - - - - - - -
  
  packet[0]= headerByte; // headerByte
  packet[1]= timestampByte; // timestampByte
  
  for (int i= 2; i<SizeOfMidi_msg+2; i++){ // fill the other bytes with the actual MIDI-values
    packet[i]= MIDI_Values[i-2];
  }
  pCharacteristic->setValue((uint8_t*)&packet, SizeOfMidi_msg+2); // set the packet to be sent
  pCharacteristic->notify(); //notify clients there is new value to be read
}
//////////////////////////////////
//////send_MIDI_BLE_Server////////
//////////////////////////////////////////////////////////////////////
//////////////////////////Server/////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//////////////////////////Client/////////////////////////////////////
static boolean doConnect = false; // when a server is found by the client and this server has the service and characteristic we are lookinf for, this variable will be set true 
static boolean connected = false; // when the client is connected to a server, this variable will bet set true
static boolean doScan = false; // to scan the existing BLE-devices
static BLERemoteCharacteristic* pRemoteCharacteristic; //This pointer is responsable for the BLE-reading and BLE-writing fonctions
static BLEAdvertisedDevice* myDevice; // when scanning this ponter will show the found device

//////////////////////////////////////////////////////////
///////////////////onNotify///////////////////////////////
static void notifyCallback( 
  BLERemoteCharacteristic* pBLERemoteCharacteristic, //to read the value from the Server
  uint8_t* pData, //the MIDI-BLE packet
  size_t length,  // size of the packet
  bool isNotify) { // when the server notified a new value, this bool value will be set true
    
   if ((length >= 3) & ((pData[0] & 0b10000000) >> 7) && ((pData[1] & 0b10000000) >> 7)){ //length of MIDI-BLE message needs to be greater as 3 (MIDI message + 2bytes), byte1 10------, byte2 1------- 
     byte byte1= pData[2]; // Status byte
     
     if (byte1 != 0b11110000){
      
      switch(length-2){// size of the actual MIDI message sent over BLE = size of the BLE-packet - 2
        case 3:{
          
          byte byte2= pData[3]; // Data1
          byte byte3= pData[4]; // Data2
          byte Status= byte1; // Status byte
          byte Data1= byte2; // Data1
          byte Data2= byte3; // Data2
      
          if (((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7) && ((byte3 xor 0b10000000) >> 7)){// verify MIDI-message (1-------, 0-------, 0-------)
            //Serial write to send the message from the ESP32 to MIDI-instrument
            Serial2.write(Status);
            Serial2.write(Data1);
            Serial2.write(Data2);
          }
          break;}
        
        case 2:{
          byte byte2= pData[3]; // Data1
          byte Status= byte1; // Status byte
          byte Data1= byte2; // Data1
      
          if (((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7)){// verify MIDI-message (1-------, 0-------)
            //Serial write to send the message from the ESP32 to MIDI-instrument            
            Serial2.write(Status);
            Serial2.write(Data1);
          }
          break;}
        
        case 1:{
          byte Status= byte1;   // Status byte
         
          if (((byte1 & 0b10000000) >> 7) && ((byte2 xor 0b10000000) >> 7) && ((byte3 xor 0b10000000) >> 7)){// verify MIDI-message (1-------)
            //Serial write to send the message from the ESP32 to MIDI-instrument
            Serial2.write(Status);
          }
          break;}

        default:
          break;
      }
    }
    else{// send system exclusive message
      Serial2.write(byte1); // write the first byte of the system exclusive message to MIDI-Instrument serial 
      int i= 3; // // i=0 headerbyte, i=1 timestampbyte, i= 2 first byte of MIDI system exlusive message
     
      do{ // read each byte of the MIDI exclusive message  until we reach the last byte 0b11110111
        
        Serial2.write(pData[i]); //Serial write of this byte
        i++; //increment i to move to the next byte
      }while(byteExc != 0b11110111); // last byte of systeme exclusive message
    }
  }
  
}
///////////////////onNotify///////////////////////////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//////MIDI_BLE_Client connect/disconnect callbacks////////
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) { //do nothing on connecting
  }

  void onDisconnect(BLEClient* pclient) { //Callback fonction when the client is disconnected from a server
    connected = false;
  }
};
//////MIDI_BLE_Client connect/disconnect callbacks////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//////MIDI_BLE_Client connect/disconnect callbacks////////
bool connectToServer() {
    myDevice->getAddress().toString().c_str(); 
    
    BLEClient*  pClient  = BLEDevice::createClient(); // create the BLE-Client

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      serviceUUID.toString().c_str();
      pClient->disconnect();
      return false;
    }

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      charUUID.toString().c_str();
      pClient->disconnect();
      return false;
    }

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
    }

    //callback fonction for the notify events
    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true; //shen connected set to true
    return true;
}
 
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

/////////////////////////////////////////////////////////
///////////////////Client-definition/////////////////////
void Client(){
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}
///////////////////Client-definition/////////////////////
/////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////
///////////////////send BLE-Client///////////////////////
void Send_BLEClient(){
  uint8_t packet[SizeOfMidi_msg+2]= {}; //this packet will be sent from the MIDI-server over BLE (size+2 for header- and timestamp bytes)

  auto t = millis(); //timee of sending the BLE-message
  uint8_t headerByte = (1 << 7) | ((t >> 7) & ((1 << 6) - 1)); //header byte 1 0 - - - - - - 
  uint8_t timestampByte = (1 << 7) | (t & ((1 << 7) - 1)); //timestmp 1 - - - - - - -
  
  packet[0]= headerByte;    // headerByte
  packet[1]= timestampByte; // timestampByte
  
  for (int i= 2; i<SizeOfMidi_msg+2; i++){ // fill the other bytes with the actual MIDI-values
    packet[i]= MIDI_Values[i-2];
  }
  pRemoteCharacteristic->writeValue((uint8_t *)&packet, SizeOfMidi_msg+2); // write the value to be recognized from the Server
}
///////////////////send BLE-Client///////////////////////
/////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////Client/////////////////////////////////////


//////////////////////////////////////////////////////////////////////
/////////////////////////////Setup////////////////////////////////////
void setup() {
  pinMode(14, INPUT);
  
  pinMode(27, OUTPUT); //Red LED to indicate that the system works
  digitalWrite(27, 1); // Turn it on
 
  pinMode(25, OUTPUT); //Blue LED to indicate that server mode is selected
  pinMode(26, OUTPUT); //Green LED to indicate that client mode is selected
  Serial2.begin(31250); // MIDI baudrate for reading and writing
  Serial.begin(115200);
}
//////////////////////////////////////////////////////////////////////
/////////////////////////////Setup////////////////////////////////////

//////////////////////////////////////////////////////////////////////
/////////////////////////////Loop/////////////////////////////////////
void loop(){ 
  
  if (true ){                   // BLE-Server 
    Server();                                     // define server
    digitalWrite(25, 1); digitalWrite(26, 0);    // turn on the server LED and turn off the Client LED 
    
     while(true){               // repeat this loop when the server is stay defined
        if (Serial2.available()>=1){              // new MIDI-message
          Serial.print("JJJJJJJJ");
          Receive_SerialMIDI();                   // break down the message 
          Send_BLEServer();                       // send it over BLE
        }
     }
      BLEServer* pServer = NULL;                  // when the Loop is left, reset all the server variable
      BLECharacteristic* pCharacteristic = NULL;  // when the Loop is left, reset all the server variable
      deviceConnected = false;                    // when the Loop is left, reset all the server variable
    }
    
   if (digitalRead(14) == LOW){                   // BLE-Client
    Client();                                     // define Client
    if (doConnect == true) {                      // A server with the service and characteristic found. Connect to it              
      if (connectToServer()) { // here the connected boolean of the client will be set true

      } else {
      }
    doConnect = false;
  }
     digitalWrite(25, 0); digitalWrite(26, 1);    // turn off the server LED and turn pn the Client LED
     while((digitalRead(14)== LOW) && (connected)){ // repeat this loop when the server is stay defined
        
        if (Serial2.available()>=1){                // new MIDI-message
          Receive_SerialMIDI();                     // break down the message
          Send_BLEClient();                         // send it over BLE
          }
      }
      // when we leave the loop that that the client is no more connected ti the server do a scan another time
      if(doScan){
        BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect.
        }
    }
  }
/////////////////////////////Loop///////////////////////////////////// 
//////////////////////////////////////////////////////////////////////
