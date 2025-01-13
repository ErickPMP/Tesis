#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>
#include <SimplyAtomic.h>
//FALTA IDENTIFICAR LA MAC DEL ESP32 DEL CUBESAT
Ticker Timer1;
unsigned long timeold;
int ACTIVADOR = 4;
int CONTADOR_ACTIVADOR = 0;
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = { 0xEC, 0x62, 0x60, 0x9E, 0x33, 0xE4 };  // MODIFICAR POR EL DEL CUBESAT
//uint8_t broadcastAddress[] = { 0xE4, 0x65, 0xB8, 0x4C, 0x8C, 0x60 }; 

// Define variables to store incoming readings
float incomingRPM;
float incomingPWM;
float incomingRPM2;
float incomingRPM3;
float incomingPWM3;
float incomingOutD;
float incominganguloConvertido;
float incomingOut_B;
float incomingOut_IT1;
float incomingOut_IT2;
float incomingOut_MR;
float incomingSPo;
float incomingDutyCicleInverso2;
float incominge_Ori;
float incomingSPo_Ori;
float incomingACTIVADOR;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
  float RPM;
  float PWM;
  float RPM2;
  float RPM3;
  float PWM3;
  float OutD;
  float SPo;
  float DutyCicleInverso2;
  float e_Ori;
  float ACTIVADOR;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;
struct_message send_Data;
esp_now_peer_info_t peerInfo;

typedef union {
  float number;
  uint8_t bytes[4];
} valor;

valor RPM_ML;
valor PWM_ML;
valor RPM2_ML;
valor RPM3_ML;
valor PWM3_ML;
valor OutD_ML;
valor anguloConvertido_ML;
valor Out_B_ML;
valor Out_IT1_ML;
valor Out_IT2_ML;
valor Out_MR_ML;
valor SPo_ML;
valor DutyCicleInverso2_ML;
valor e_Ori_ML;
valor SPo_Ori_ML;
valor ACTIVADOR_ML;
valor ACTIVADOR_SIMULINK_ML;
float value;
// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingRPM = incomingReadings.RPM;
  incomingPWM = incomingReadings.PWM;
  incomingRPM2 = incomingReadings.RPM2;
  incomingPWM3 = incomingReadings.PWM3;
  incomingSPo = incomingReadings.SPo;
  incomingDutyCicleInverso2 = incomingReadings.DutyCicleInverso2;
  incomingACTIVADOR = incomingReadings.ACTIVADOR;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){success = "Delivery Success :)";}
  else{success = "Delivery Fail :(";}
  Serial.println(">>>>>");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  //Timer1.once_ms(1, onTimer1);
  timeold = 0;
}

void loop() {
  if(Serial.available() > 0){value = recepcion();  }
  RPM_ML.number = incomingRPM;
  PWM_ML.number = incomingPWM;
  RPM2_ML.number = incomingRPM2;
  RPM3_ML.number = incomingRPM3;
  PWM3_ML.number = incomingPWM3;
  OutD_ML.number = incomingOutD;
  SPo_ML.number = incomingSPo;
  DutyCicleInverso2_ML.number = incomingDutyCicleInverso2;
  e_Ori_ML.number = incominge_Ori;
  ACTIVADOR_ML.number = incomingACTIVADOR;
  ACTIVADOR_SIMULINK_ML.number = value;
  ATOMIC(){
    if (millis()-timeold >= 100){
      Serial.write('V');
      for (int i = 0; i < 4; i++) { Serial.write(RPM_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(PWM_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(RPM2_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(PWM3_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(SPo_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(DutyCicleInverso2_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(ACTIVADOR_ML.bytes[i]); }
      for(int i=0; i<4; i++){Serial.write(ACTIVADOR_SIMULINK_ML.bytes[i]);}
      Serial.write('\n');
      timeold = millis();
      if (value == 4 && coso == 0){
        send_Data.ACTIVADOR = value;
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
        if (result == ESP_OK) {Serial.println("Sent with success");}
        else {Serial.println("Error sending the data");}
        coso = 1;
      }
      if (value == 5 && coso == 0){
        
      }
    }
  }
}

//Recibir Flotante
float recepcion(){
  int i;
  valor buf;
  for(i=0; i<4; i++)
    buf.bytes[i] = Serial.read();  
  return buf.number;
}
