#include <Ticker.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <esp_now.h> //ENVIO DE DATOS
#include <WiFi.h>     //ENVIO DE DATOS
#include <SimplyAtomic.h>
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
Ticker Timer6; // INICIALIZACIÓN DEL CONTROLADOR DESPUES DE 15 SEGUNDOS
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
  float Out_IKp;
  float Out_IKi;
  float Out_IKd;
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
//float Kp_Ori = 1500.0, Ki_Ori = 10.0, Kd_Ori = 1000.0;
float Kp_Ori = 1500.0, Ki_Ori = 100.0, Kd_Ori = 1000.0;
float u_Ori = 0.0, e_Ori = 0.0, upO, uiO, udO, e_1O, ui_1O, u_absO, SPo_Ori;
//CONTROL MRAC PID (VELOCIDAD)
float Gamma_Kp = -0.000105, Gamma_Ki = -0.00020, Gamma_Kd = 0.0000114;
//float Gamma1 = 3.65E-6, Gamma2 = -9.42E-7; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 2000 RPM CADA 15 SEG ALTO, BAJO 5 SEG
//float Gamma1 = 5E-5, Gamma2 = -1.13E-5; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 100 RPM CADA 20 SEG ALTO, BAJO 5 SEG ,ADAPTACIÓN LENTA
//float Gamma1 = 0.0004, Gamma2 = -8.9E-5; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 100 RPM CADA 20 SEG ALTO, BAJO 5 SEG ,ADAPTACIÓN MEDIA
//float Gamma1 = 0.0006, Gamma2 = -0.000112; // OBTENIDO CON SIMULINK CON SEÑAL DE PRUEBA A 100 RPM CADA 20 SEG ALTO, BAJO 5 SEG ,ADAPTACIÓN MEDIA-ALTA (MUCHAS SUPOSICIONES FALLA)
float eM,eC;
// Modelo matematico
float rkD, rk1D, rk2D;
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
//  MODELO DE REFERENCIA (MODIFICAR VALORES)
float rk_MR = 0.0, rk1_MR = 0.0, rk2_MR = 0.0, Out_MR = 0.0;
float b0_MR = 0.0;
float b1_MR = 0.000801915452637996;
float b2_MR = -0.000793432793956761;
float a0_MR = 1.0;
float a1_MR = -1.99417499916524;
float a2_MR = 0.994183481823922;
// KP, KI Y KD (MODIFICAR VALORES)
float rk_Kp = 0.0, rk1_Kp = 0.0, rk2_Kp = 0.0, Out_Kp = 0.0;
float rk_Ki = 0.0, rk1_Ki = 0.0, rk2_Ki = 0.0, Out_Ki = 0.0;
float rk_Kd = 0.0, rk1_Kd = 0.0, rk2_Kd = 0.0, Out_Kd = 0.0;
float dKp = 0.0, dKi = 0.0, dKd = 0.0; //DERIVADA DE TETHA1 Y TETHA2
float b0_Kp = 0.0, b1_Kp = 0.000997087499582620, b2_Kp = -0.000997087499582620;
float b0_Ki = 0.0, b1_Ki = 4.99028812602188e-07, b2_Ki = 4.98059393868356e-07;
float b0_Kd = 0.999999999999999, b1_Kd = -1.99999575454703, b2_Kd = 0.999995754547028;
// INTEGRADOR (MODIFICAR VALORES)
float rk_IKp = 0.0, rk1_IKp = 0.0, Out_IKp = 0.0;
float rk_IKi = 0.0, rk1_IKi = 0.0, Out_IKi = 0.0;
float rk_IKd = 0.0, rk1_IKd = 0.0, Out_IKd = 0.0;
float rk_IeC = 0.0, rk1_IeC = 0.0, Out_IeC = 0.0;
float b0 = 0.0;
float b1 = 0.00100000000000000;
float a0 = 1.0;
float a1 = -1.0;
// DERIVADOR DEL ERROR COMUN
float DeC = 0.0, DeC_1 = 0.0;
//***********INICIALIZADOR
int INICIADOR = 0;
//************************
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
  t = s + ms / 1000.00;
}
void ICACHE_RAM_ATTR onTimer2() { // MODIFICAR A TS = 1ms
  if(incomingACTIVADOR == 5){
    // CONTROL DE ORIENTACIÓN
    e_Ori = SPo_Ori - (Out_B * M_PI / 180); // Out_B es anguloConvertido  filtrado
    //***************
    if (e_Ori > M_PI){e_Ori = e_Ori - 2 * M_PI;}
    if (e_Ori < -M_PI) {e_Ori = e_Ori + 2 * M_PI;}
    //**************
    //Control Proporcional Orientación
    upO = Kp_Ori * e_Ori;
    if(upO > 3000.0 * M_PI/30.0){upO = 3000.0 * M_PI/30.0;}
    else if (upO < -3000.0 * M_PI/30.0){upO = -3000.0 * M_PI/30.0;}
    else {upO = upO;}
    //upO = constrain (upO, -3000.0, 3000.0);
    //Control Integral Orientación
    uiO = e_Ori * 0.001 + ui_1O;
    ui_1O = uiO;
    uiO = Ki_Ori * uiO;
    if(uiO > 3000.0 * M_PI/30.0){uiO = 3000.0 * M_PI/30.0;}
    else if (uiO < -3000.0 * M_PI/30.0){uiO = -3000.0 * M_PI/30.0;}
    else {uiO = uiO;}
    //uiO = constrain(uiO, -3000.0, 3000.0);
    //Control Derivativo Orientación
    udO = (e_Ori - e_1O) / 0.001;
    e_1O = e_Ori;
    //udO = constrain(udO, -3000.0, 3000.0);
    udO = Kd_Ori * udO;
    if(udO > 3000.0 * M_PI/30.0){udO = 3000.0 * M_PI/30.0;}
    else if (udO < -3000.0 * M_PI/30.0){udO = -3000.0 * M_PI/30.0;}
    else {udO = udO;}
    u_Ori = upO + uiO + udO;
    if(u_Ori > 3000.0 * M_PI/30.0){SPo = 3000.0 * M_PI/30.0;}
    else if (u_Ori < -3000.0 * M_PI/30.0){SPo = -3000.0 * M_PI/30.0;}
    else {SPo = u_Ori;}
    //SPo = constrain(u_Ori, -3000.0, 3000.0);
    //***************
    //  MODELO MATEMÁTICO
    OutD = rkD * b0D + rk1D * b1D + rk2D * b2D;
    rkD = (abs(DutyCicleInverso2 - 255) * 12.14 / 255) - rk1D * a1D - rk2D * a2D;
    rk2D = rk1D;
    rk1D = rkD;
    // MODELO DE REFERENCIA (MODIFICADO)
    Out_MR = rk_MR * b0_MR + rk1_MR * b1_MR + rk2_MR * b2_MR;
    rk_MR = SPo - rk1_MR * a1_MR - rk2_MR * a2_MR;
    rk2_MR = rk1_MR;
    rk1_MR = rk_MR;
    //GAMMA_KP (ANTES TETHA1) (MODIFICADO) (REVISAR LA ENTRADA SPO)
    Out_Kp = rk_Kp * b0_Kp + rk1_Kp * b1_Kp + rk2_Kp * b2_Kp;
    rk_Kp = eM - rk1_Kp * a1_MR - rk2_Kp * a2_MR;
    rk2_Kp = rk1_Kp;
    rk1_Kp = rk_Kp;
    // INTEGRADOR DE KP (REVISAR LA ENTRADA)
    Out_IKp = rk_IKp * b0 + rk1_IKp * b1;
    rk_IKp = dKp - rk1_IKp * a1;
    rk1_IKp = rk_IKp;
    //GAMMA_KI (REVISAR LA ENTRADA)
    Out_Ki = rk_Ki * b0_Ki + rk1_Ki * b1_Ki + rk2_Ki * b2_Ki;
    rk_Ki = eM - rk1_Ki * a1_MR - rk2_Ki * a2_MR;
    rk2_Ki = rk1_Ki;
    rk1_Ki = rk_Ki;
    // INTEGRADOR DE KI (REVISAR LA ENTRADA)
    Out_IKi = rk_IKi * b0 + rk1_IKi * b1;
    rk_IKi = dKi - rk1_IKi * a1;
    rk1_IKi = rk_IKi;
    // GAMMA_KD (REVISAR LA ENTRADA)
    Out_Kd = rk_Kd * b0_Kd + rk1_Kd * b1_Kd + rk2_Kd * b2_Kd;
    rk_Kd = RPM2 - rk1_Kd * a1_MR - rk2_Kd * a2_MR;
    rk2_Kd = rk1_Kd;
    rk1_Kd = rk_Kd;
    // INTEGRADOR DE KD (REVISAR LA ENTRADA)
    Out_IKd = rk_IKd * b0 + rk1_IKd * b1;
    rk_IKd = dKd - rk1_IKd * a1;
    rk1_IKd = rk_IKd;
    // INTEGRADOR DEL ERROR COMUN
    Out_IeC = rk_IeC * b0 + rk1_IeC * b1;
    rk_IeC = eC - rk1_IeC * a1;
    rk1_IeC = rk_IeC;
    // DERIVADOR DEL ERROR COMUN
    DeC = (RPM2 - DeC_1) / 0.001;
    DeC_1 = RPM2;
    // LIMITES DE LOS INTEGRADORES (NO SE DEBE USAR CONSTRAIN GENERA PROBLEMAS)
    /*
    if (Out_IKp < 0.0) {Out_IKp = 0.0;}
    else {Out_IKp = Out_IKp;}
    if (Out_IKi < 0.0) {Out_IKi = 0.0;}
    else {Out_IKi = Out_IKi;}
    */
    if (Out_IKd > 0.0) {Out_IKd = 0.0;}
    else {Out_IKd = Out_IKd;}
    //CONTROL DE VELOCIDAD CON DOS PARÁMETROS TETHA
    eC = SPo - RPM2;                    // ERROR DE SALIDA COMUN ORIENTACIÓN - SETPOINT
    eM = RPM2 - Out_MR;                 // ERROR DE SALIDA CON EL MODELO DE REFERENCIA
    dKp = -1 * Out_Kp * eM * Gamma_Kp;  //ANTES DE ENTRAR AL INTEGRADOR DE KP
    dKi = -1 * Out_Ki * eM * Gamma_Ki;  //ANTES DE ENTRAR AL INTEGRADOR DE KI
    dKd = Out_Kd * eM * Gamma_Kd;       //ANTES DE ENTRAR AL INTEGRADOR DE KD
    u = (Out_IKp + 0.13+0.22) * eC + (Out_IKi + 1.0+0.3) * Out_IeC - (Out_IKd) * DeC; // LEY DE CONTROL MRACPARÁMETROS PID
    if (u >= 0){digitalWrite(D16, HIGH);}
    else {digitalWrite(D16, LOW);}
    u_abs = abs(u);
    u_abs = constrain(u_abs, 0, 255);
    DutyCicleInverso2 = 255 - u_abs;
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
    Out_Kp = 0.0;
    Out_IKp = 0.0;
    Out_Ki = 0.0;
    Out_IKi = 0.0;
    Out_Kd = 0.0;
    Out_IKd = 0.0;
    Out_IeC = 0.0;
    DeC = 0.0;
    eC = 0.0;
    eM = 0.0;
    dKp = 0.0;
    dKi = 0.0;
    dKd = 0.0;
    u = 0.0;
    u_abs = 0.0;
    DutyCicleInverso2 = 255;
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

void ICACHE_RAM_ATTR onTimer5() { //ENVIO DE DATA AL OTRO ESP32  TS = 70 ms
  ESP_SEND.RPM2 = RPM2;
  ESP_SEND.anguloConvertido = anguloConvertido;
  ESP_SEND.Out_B = Out_B;
  ESP_SEND.Out_IKp = Out_IKp;
  ESP_SEND.Out_IKi = Out_IKi;
  ESP_SEND.Out_IKd = Out_IKd;
  ESP_SEND.Out_MR = Out_MR;
  ESP_SEND.SPo = SPo;
  ESP_SEND.DutyCicleInverso2 = DutyCicleInverso2;
  ESP_SEND.SPo_Ori = SPo_Ori;
  ESP_SEND.ACTIVADOR = incomingACTIVADOR;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ESP_SEND, sizeof(ESP_SEND)); // SE LLENA EN EL VOID LOOP
  if (result == ESP_OK) {Serial.println("Sent with success");}
  else {Serial.println("Error sending the data");}
}

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
  Timer5.attach_ms(110, onTimer5); //ENVIO DE DATA AL OTRO ESP32 (ESP_NOW)
  Timer6.once(15,onTimer6);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(interruptPinA2, INPUT_PULLUP);
  pinMode(interruptPinB2, INPUT_PULLUP);
  pinMode(interruptPinA3, INPUT_PULLUP);
  pinMode(interruptPinB3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), ISR_EncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB2), ISR_EncoderB2, CHANGE);
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
