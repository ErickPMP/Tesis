#include <Arduino.h>
#include <Ticker.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <esp_now.h> //ENVIO DE DATOS
#include <WiFi.h>     //ENVIO DE DATOS
#include <SimplyAtomic.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

const char* ssid = "W-UCSP";
const char* password = "";

AsyncWebServer server(80);
unsigned long timeold;

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

Ticker Timer1;  //ENVIO DE DATOS A MATLAB Y CALCULO DE RPM
Ticker Timer2;  //FUNCIONES DE TRANSFERENCIA DEL MRAC Y EL CONTROLADOR
Ticker Timer3;  //LECTURAS DE LA BRUJULA
Ticker Timer4;  //FILTRO BRUJULA
Ticker Timer5; //ENVIO DE DATOS ESP_NOW
Ticker Timer6; // INICIALIZACIÓN DEL CONTROLADOR DESPUÉS DE 15 SEGUNDOS
//**************************ESP_NOW
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x9C, 0x1B, 0xFC}; // MAC DEL ESP32 QUE RECIBE
String success; // VARIABLE PARA GUARDAR SI EL MENSAJE ENVIADO FUE EXITOSO
typedef struct struct_message { // ESTRUCTURA PARA ENVIAR LOS DATOS (MODIFICAR)
  float RPM;
  float PWM;
  float RPM2;
  float RPM3;
  float PWM3;
  float OutD;
  float anguloConvertido;
  float Out_B;
  float Out_IT1;
  float Out_IT2;
  float Out_MR;
  float SPo;
  float DutyCicleInverso2;
  float e_Ori;
  float SPo_Ori;
  float ACTIVADOR;
} struct_message;
struct_message ESP_SEND;  // ESTRUCTURA DEL MENSAJE POR DONDE SE ENVIARAN TODOS LOS DATOS
struct_message incomingReadings;
esp_now_peer_info_t peerInfo;
//**********Brujula********
float u_pos,e_p,Geo_fix;
QMC5883LCompass compass;
double anguloConvertido;
int x,y,z;
float angulo;
//*************************

const int D23 = 23;           // PWM
const int D14 = 14;           // PWM_MOTOR2
const int D25 = 25;           // PWM_MOTOR3
const int D4 = 4;             // SIGNO CW CCW
const int D16 = 16;            // SIGNO CW CCW_ MOTOR2
const int D26 = 26;            // SIGNO CW CCW_ MOTOR3

const int interruptPinA = 18, interruptPinB = 19;   // ENCODER MOTOR1
const int interruptPinA2 = 27, interruptPinB2 = 13; // ENCODER MOTOR2
const int interruptPinA3 = 32, interruptPinB3 = 33; // ENCODER MOTOR3
const int frecuencia = 25000;  // FRECUENCIA PWM 25 KHZ
const int canal = 0;      //0-15 16canales
const int canal2 = 1;
const int canal3 = 2;
const int resolucion = 8; //0-255
bool PinA, PinB;
bool PinA2, PinB2;
bool PinA3, PinB3;
volatile long EncoderCount = 0;
volatile long EncoderCount2 = 0;
volatile long EncoderCount3 = 0;
float t, s, ms, w;
float RPM, PWM;   //MOTOR1
float RPM2, PWM2; //MOTOR2
float RPM3, PWM3; //MOTOR3
float SPo,OutD,Out_B; //MOTOR1
float SPo2,OutD2; //MOTOR2
float SPo3,OutD3; //MOTOR3
float DutyCicleInverso2=0.0, u=0.0, u_abs=0.0; // SEÑALES DE CONTROL MOTOR2
//CONTROL PID (ORIENTACIÓN)
float Kp_Ori = 1500, Ki_Ori = 60.0, Kd_Ori = 1000.0;    //BUENOS VALORES, SE SINTONIZO CON RAMPAS RAPIDAS
//float Kp_Ori = 1100, Ki_Ori = 200.0, Kd_Ori = 100.0;
float u_Ori = 0.0, e_Ori = 0.0, upO, uiO, udO, e_1O, ui_1O, u_absO, SPo_Ori;
//CONTROL MRAC (VELOCIDAD)
float Gamma1 = 3.65E-6, Gamma2 = -9.42E-7; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 2000 RPM CADA 15 SEG ALTO, BAJO 5 SEG
//float Gamma1 = 5E-5, Gamma2 = -1.13E-5; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 100 RPM CADA 20 SEG ALTO, BAJO 5 SEG ,ADAPTACIÓN LENTA
//float Gamma1 = 0.0004, Gamma2 = -8.9E-5; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 100 RPM CADA 20 SEG ALTO, BAJO 5 SEG ,ADAPTACIÓN MEDIA
//float Gamma1 = 0.0006, Gamma2 = -0.000112; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 100 RPM CADA 20 SEG ALTO, BAJO 5 SEG ,ADAPTACIÓN MEDIA-ALTA (MUCHAS SUPOSICIONES FALLA)
float eM;
// Modelo matematico
float M1D , M2D, rkD, rk1D, rk2D;
float b0D = 0.0;
float b1D = 0.0244327134682811;
float b2D = 0.0175428282633313;
float a0D = 1.0;
float a1D = -1.36573514881393;
float a2D = 0.367479877670365;
// Filtro de orden 1 para brújula FRECUENCIA DE CORTE 0.5 HZ
float M1_B , M2_b, rk_B, rk1_B, rk2_B;
float b0_B = 0.0;
float b1_B = 0.000499875020830730;
float b2_B = 0.0;
float a0_B = 1.0;
float a1_B = -0.999500124979169;
float a2_B = 0.0;
//  MODELO DE REFERENCIA
float rk_MR = 0.0, rk1_MR = 0.0, Out_MR = 0.0;
float b0_MR = 0.0;
float b1_MR = 0.00249687760253988;
float a0_MR = 1.0;
float a1_MR = -0.997503122397460;
// TETHA 1 Y TETHA 2
float rk_T1 = 0.0, rk1_T1 = 0.0, Out_T1 = 0.0;
float rk_T2 = 0.0, rk1_T2 = 0.0, Out_T2 = 0.0;
float dTetha1 = 0.0, dTetha2 = 0.0; //DERIVADA DE TETHA1 Y TETHA2
// INTEGRADOR
float rk_IT1 = 0.0, rk1_IT1 = 0.0, Out_IT1 = 0.0;
float rk_IT2 = 0.0, rk1_IT2 = 0.0, Out_IT2 = 0.0;
float b0 = 0.0;
float b1 = 0.00100000000000000;
float a0 = 1.0;
float a1 = -1.0;
//********INICIALIZADOR
int INICIADOR = 0;
//*********************
float Pos_Inicial = 0.0;
int Iniciador_rampa = 0;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {// AVISO DE ENVIO EXITOSO ESP_NOW
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){success = "Delivery Success :)";}
  else{success = "Delivery Fail :(";}
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingACTIVADOR = incomingReadings.ACTIVADOR;
}

void ICACHE_RAM_ATTR onTimer1() {
  ms = ms + 1;
  if (ms == 1000.00) {ms = 0.0; s = s + 1;}
  t    = s + ms / 1000.00;
}
void ICACHE_RAM_ATTR onTimer2() { // MODIFICAR A TS = 1ms
  if (incomingACTIVADOR == 5){
    //  CONTROL DE ORIENTACIÓN
    e_Ori = SPo_Ori - (Out_B * M_PI / 180); // Out_B es anguloConvertido  filtrado
    //**************
    if (e_Ori > M_PI){e_Ori = e_Ori - 2 * M_PI;}
    if (e_Ori < -M_PI) {e_Ori = e_Ori + 2 * M_PI;}
    //**************
    //Control Proporcional Orientación
    upO = Kp_Ori * e_Ori;
    if (upO > 3000.0 * M_PI/30.0) {upO = 3000.0 * M_PI/30.0;}
    else if (upO < -3000.0 * M_PI/30.0) {upO = -3000.0 * M_PI/30.0;}
    else {upO = upO;}
    //upO = constrain (upO, -3000.0, 3000.0);
    //Control Integral Orientación
    uiO = e_Ori * 0.001 + ui_1O;
    ui_1O = uiO;
    uiO = Ki_Ori * uiO;
    if (uiO > 3000.0 * M_PI/30.0) {uiO = 3000.0 * M_PI/30.0;}
    else if (uiO < -3000.0 * M_PI/30.0) {uiO = -3000.0 * M_PI/30.0;}
    else {uiO = uiO;}
    //uiO = constrain(uiO, -3000.0, 3000.0);
    //Control Derivativo Orientación
    udO = (e_Ori - e_1O) / 0.001;
    e_1O = e_Ori;
    //udO = constrain(udO, -3000.0, 3000.0);
    udO = Kd_Ori * udO;
    if (udO > 3000.0 * M_PI/30.0) {udO = 3000.0 * M_PI/30.0;}
    else if (udO < -3000.0 * M_PI/30.0) {udO = -3000.0 * M_PI/30.0;}
    else {udO = udO;}
    u_Ori = upO + uiO + udO;
    if (u_Ori > 3000.0 * M_PI/30.0) {SPo = 3000.0 * M_PI/30.0;}
    else if (u_Ori < -3000.0 * M_PI/30.0) {SPo = -3000.0 * M_PI/30.0;}
    else {SPo = u_Ori;}
    //SPo = constrain(u_Ori, -3000.0, 3000.0);
    //******************************
    //  MODELO MATEMÁTICO
    OutD = rkD*b0D + rk1D*b1D + rk2D*b2D;
    rkD = (abs(DutyCicleInverso2 - 255) * 12.14 / 255) - rk1D*a1D - rk2D*a2D;
    rk2D = rk1D;
    rk1D = rkD;
    // MODELO DE REFERENCIA
    Out_MR = rk_MR * b0_MR + rk1_MR * b1_MR;
    rk_MR = SPo - rk1_MR * a1_MR;
    rk1_MR = rk_MR;
    //TETHA1
    Out_T1 = rk_T1 * b0_MR + rk1_T1 * b1_MR;
    rk_T1 = SPo - rk1_T1 * a1_MR;
    rk1_T1 = rk_T1;
    // INTEGRADOR DE TETHA1
    Out_IT1 = rk_IT1 * b0 + rk1_IT1 * b1;
    rk_IT1 = dTetha1 - rk1_IT1 * a1;
    rk1_IT1 = rk_IT1;
    //TETHA2
    Out_T2 = rk_T2 * b0_MR + rk1_T2 * b1_MR;
    rk_T2 = RPM2 - rk1_T2 * a1_MR;
    rk1_T2 = rk_T2;
    // INTEGRADOR DE TETHA2
    Out_IT2 = rk_IT2 * b0 + rk1_IT2 * b1;
    rk_IT2 = dTetha2 - rk1_IT2 * a1;
    rk1_IT2 = rk_IT2;
    // LIMITES DE LOS INTEGRADORES (NO SE DEBE USAR CONSTRAIN GENERA PROBLEMAS)
    /*
    if (Out_IT1 < 0.0) {Out_IT1 = 0.0;}
    else {Out_IT1 = Out_IT1;}
    if (Out_IT2 > 0.0) {Out_IT2 = 0.0;}
    else {Out_IT2 = Out_IT2;}
    */
    //CONTROL DE VELOCIDAD CON DOS PARÁMETROS TETHA
    eM = RPM2 - Out_MR;                   // ERROR DE SALIDA CON EL MODELO DE REFERENCIA
    dTetha1 = -1 * Out_T1 * eM * Gamma1; //ANTES DE ENTRAR AL INTEGRADOR DE TETHA1
    dTetha2 = -1 * Out_T2 * eM * Gamma2; //ANTES DE ENTRAR AL INTEGRADOR DE TETHA2
    u = (Out_IT1 + 6.5e-1) * SPo - (Out_IT2 - 5.0e-2) * RPM2; // LEY DE CONTROL MRAC CON DOS PARÁMETROS
    //u = (Out_IT1) * SPo - (Out_IT2) * RPM2; 
    if (u >= 0){digitalWrite(D16, HIGH);}
    else {digitalWrite(D16, LOW);}
    u_abs = abs(u);
    u_abs = constrain(u_abs, 0, 249);
    DutyCicleInverso2 = 249 - u_abs;
  }
  else if (incomingACTIVADOR == 4){
    e_Ori = 0.0;
    upO = 0.0;
    uiO = 0.0;
    udO = 0.0;
    u_Ori = 0.0;
    SPo = 0.0;
    OutD = 0.0;
    Out_MR = 0.0;
    Out_T1 = 0.0;
    Out_IT1 = 0.0;
    Out_T2 = 0.0;
    Out_IT2 = 0.0;
    eM = 0.0;
    dTetha1 = 0.0;
    dTetha2 = 0.0;
    u = 0.0;
    u_abs =0.0;
    DutyCicleInverso2 = 255;

    e_1O = 0.0;
    ui_1O = 0.0;
  }
}

void ICACHE_RAM_ATTR onTimer3() { //LECTURAS DE LA BRUJULA TS = 40 ms
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  angulo = atan2(y, z);
  anguloConvertido = angulo*(180/M_PI);
  //if(anguloConvertido<0) anguloConvertido=anguloConvertido+360;
}

void ICACHE_RAM_ATTR onTimer4() { //FILTRO DE LA BRUJULA A 0.5 HZ CON TS = 1 ms
  Out_B = rk_B * b0_B + rk1_B * b1_B + rk2_B * b2_B;
  rk_B = anguloConvertido - rk1_B * a1_B - rk2_B * a2_B;
  rk2_B = rk1_B;
  rk1_B = rk_B;
}
/*
void ICACHE_RAM_ATTR onTimer5() { //ENVIO DE DATA AL OTRO ESP32  TS = 70 ms
  ESP_SEND.RPM2 = RPM2;
  ESP_SEND.anguloConvertido = anguloConvertido;
  ESP_SEND.Out_B = Out_B;
  ESP_SEND.Out_IT1 = Out_IT1;
  ESP_SEND.Out_IT2 = Out_IT2;
  ESP_SEND.Out_MR = Out_MR;
  ESP_SEND.SPo = SPo;
  ESP_SEND.DutyCicleInverso2 = DutyCicleInverso2;
  ESP_SEND.SPo_Ori = SPo_Ori;
  ESP_SEND.ACTIVADOR = incomingACTIVADOR;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ESP_SEND, sizeof(ESP_SEND)); // SE LLENA EN EL VOID LOOP
  if (result == ESP_OK) {Serial.println("Sent with success");}
  else {Serial.println("Error sending the data");}
}
*/
void ICACHE_RAM_ATTR onTimer6() {
  INICIADOR = 1;
}

void ICACHE_RAM_ATTR ISR_EncoderA2() {
  PinB2 = digitalRead(interruptPinB2);
  PinA2 = digitalRead(interruptPinA2);

  if (PinB2 == LOW) {
    if (PinA2 == HIGH) {
      EncoderCount2++;
    }
    else {
      EncoderCount2--;
    }
  }

  else {
    if (PinA2 == HIGH) {
      EncoderCount2--;
    }
    else {
      EncoderCount2++;
    }
  }
}

void ICACHE_RAM_ATTR ISR_EncoderB2() {
  bool PinA2 = digitalRead(interruptPinA2);
  bool PinB2 = digitalRead(interruptPinB2);

  if (PinA2 == LOW) {
    if (PinB2 == HIGH) {
      EncoderCount2--;
    }
    else {
      EncoderCount2++;
    }
  }

  else {
    if (PinB2 == HIGH) {
      EncoderCount2++;
    }
    else {
      EncoderCount2--;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  compass.init();
  compass.setCalibration(-1083, 918, -1013, 921, -555, 1582);
  // ESP_NOW************
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {Serial.println("Error initializing ESP-NOW");return;}
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;     
  if (esp_now_add_peer(&peerInfo) != ESP_OK){Serial.println("Failed to add peer");return;}
  esp_now_register_recv_cb(OnDataRecv);
  //*******************
  pinMode(D4, OUTPUT);
  pinMode(D16, OUTPUT);
  pinMode(D26, OUTPUT);
  ledcSetup(canal,frecuencia,resolucion);
  ledcSetup(canal2,frecuencia,resolucion);
  ledcSetup(canal3,frecuencia,resolucion);
  ledcAttachPin(D23,canal);
  ledcAttachPin(D14,canal2);
  ledcAttachPin(D25,canal3);
  Timer1.attach_ms(1, onTimer1);  //ENVIO DE DATOS A MATLAB Y CALCULO DE RPM
  Timer2.attach_ms(1, onTimer2);  //FUNCIONES DE TRANSFERENCIA DEL MRAC Y EL CONTROLADOR 
  Timer3.attach_ms(40, onTimer3); //LECTURAS DE LA BRUJULA
  Timer4.attach_ms(1, onTimer4);  //FILTRO BRUJULA
//  Timer5.attach_ms(110, onTimer5); //ENVIO DE DATA AL OTRO ESP32 (ESP_NOW)
  Timer6.once(15, onTimer6);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(interruptPinA2, INPUT_PULLUP);
  pinMode(interruptPinB2, INPUT_PULLUP);
  pinMode(interruptPinA3, INPUT_PULLUP);
  pinMode(interruptPinB3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), ISR_EncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB2), ISR_EncoderB2, CHANGE);

  //************ELEGANTOTA***********
  WiFi.begin(ssid, password);
  Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hola MRAC.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
  //*********************************
}

void loop() {
  digitalWrite(D4, HIGH);
  //digitalWrite(D16, HIGH);
  digitalWrite(D26, HIGH);
  //ledcWrite(canal,PWM);
  ledcWrite(canal2,DutyCicleInverso2);
  //ledcWrite(canal3,PWM3);
  if (millis() - timeold >= 100){
    ATOMIC(){
    //RPM = EncoderCount * 1.483 * M_PI/30;// * 1.71;
    RPM2 = EncoderCount2 * 1.483 * M_PI/30;
    //RPM3 = EncoderCount3 * 1.483 * M_PI/30;
    EncoderCount2 = 0.0;
    timeold = millis();

    ESP_SEND.RPM2 = RPM2;
    ESP_SEND.anguloConvertido = anguloConvertido;
    ESP_SEND.Out_B = Out_B;
    ESP_SEND.Out_IT1 = Out_IT1;
    ESP_SEND.Out_IT2 = Out_IT2;
    ESP_SEND.Out_MR = Out_MR;
    ESP_SEND.SPo = SPo;
    ESP_SEND.DutyCicleInverso2 = DutyCicleInverso2;
    ESP_SEND.SPo_Ori = SPo_Ori;
    ESP_SEND.ACTIVADOR = incomingACTIVADOR;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ESP_SEND, sizeof(ESP_SEND)); // SE LLENA EN EL VOID LOOP
    if (result == ESP_OK) {Serial.println("Sent with success");}
    else {Serial.println("Error sending the data");}
    }
  }

  if (incomingACTIVADOR == 4){
    /*
    if(t<2.0) SPo_Ori = 40.0 * M_PI / 180.0;
    if(t>=2.0) {t = 0.0; ms=0,s=0;}
    */
    SPo_Ori = Out_B * M_PI / 180;
    Iniciador_rampa = 0;
  }
  if (incomingACTIVADOR == 5){
    /*
    // SEAÑAL RAMPA Y PLANOS CADA 20 SEGUNDOS
    if(t<20.0) SPo_Ori = -40.0 * M_PI / 180.0;
    if((t>=20.0)&&(t<40.0)) SPo_Ori = ((40.0+40.0)* M_PI / 180.0 /(40.0-20.0))*(t-20.0) - 40.0 * M_PI / 180.0;
    if((t>=40.0)&&(t<60.0)) SPo_Ori = 40 * M_PI / 180.0;
    if((t>=60.0)&&(t<80.0)) SPo_Ori = ((-40.0-40.0)* M_PI / 180.0 /(80.0-60.0))*(t-60.0) + 40.0 * M_PI / 180.0;
    if(t>=80.0) {t = 0.0; ms=0,s=0;}
    */
    if(Iniciador_rampa == 0){
      Pos_Inicial = Out_B;
      Iniciador_rampa = Iniciador_rampa + 1;
      t = 0.0; ms=0,s=0;
    }
    if(Iniciador_rampa == 1){
      if(t<30.0) SPo_Ori = ((-40.0-Pos_Inicial)* M_PI / 180.0 /(30.0-0.0))*(t-0.0) + Pos_Inicial * M_PI / 180.0;
      if(t>=30.0) {t = 0.0; ms=0,s=0;Iniciador_rampa = Iniciador_rampa + 1;}
    }
    if(Iniciador_rampa == 2){
      // +90° -180° +270° -180°
      if(t<60.0) SPo_Ori = -40.0 * M_PI / 180.0;
      if((t>=60.0)&&(t<90.0)) SPo_Ori = ((50.0+40.0)* M_PI / 180.0 /(90.0-60.0))*(t-60.0) - 40.0 * M_PI / 180.0;
      if((t>=90.0)&&(t<150.0)) SPo_Ori = 50 * M_PI / 180.0;
      if((t>=150.0)&&(t<210.0)) SPo_Ori = ((-130.0-50.0)* M_PI / 180.0 /(210.0-150.0))*(t-150.0) + 50.0 * M_PI / 180.0;
      if((t>=210.0)&&(t<270.0)) SPo_Ori = -130.0 * M_PI / 180.0;
      if((t>=270.0)&&(t<360.0)) SPo_Ori = ((140.0+130.0)* M_PI / 180.0 /(360.0-270.0))*(t-270.0) - 130.0 * M_PI / 180.0;
      if((t>=360.0)&&(t<420.0)) SPo_Ori = 140.0 * M_PI / 180.0;
      if((t>=420.0)&&(t<480.0)) SPo_Ori = ((-40.0-140.0)* M_PI / 180.0 /(480.0-420.0))*(t-420.0) + 140.0 * M_PI / 180.0;
      if(t>=480.0) {t = 0.0; ms=0,s=0;}
    }
  }   
}
