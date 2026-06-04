// =====================================================
// ESP32-S3 + IBT-2 tipo BTS7960 + 2 Motores DC + 2 Servos
// Control por WEBSOCKET (Punto de Acceso)
// =====================================================

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ---------------- Configuración Wi-Fi ----------------
const char* ssid = "ESP32-Control-Web";
const char* password = ""; 

// ----------------- Servos Simétricos -------------------
const int pinServoIzq = 12; // Servo lado Izquierdo
const int pinServoDer = 48; // Servo lado Derecho

Servo servoIzq;
Servo servoDer;

// Estado global para la telemetría
int servoEstadoReal = 0; 

// Servidor WebSocket en el puerto 80
WebSocketsServer webSocket = WebSocketsServer(80);

// ---------------- Variables del PID ----------------
float cv = 0; float cv1 = 0;
float error = 0; float error1 = 0; float error2 = 0;
float kp = 1.0; float ki = 10.0; float kd = 0.01;
float Tm = 0.1; 

// ---------------- Encoder ----------------
volatile long contador = 0;
const int pinEncoderA = 19;   
const int pinEncoderB = 20;   

// ---------------- BTS7960 / IBT-2 PINES ASIGNADOS ----------------
// Motor M1
const int LPWM_M1 = 13;          
const int RPWM_M1 = 14; 

// Motor M2
const int LPWM_M2 = 47; 
const int RPWM_M2 = 21; 

// ---------------- Datos del motor ----------------
const int PPR = 17; const int ratio = 108;  
const float EDGES_PER_PULSE = 2; 
const float PULSOS_POR_VUELTA = PPR * ratio * EDGES_PER_PULSE; 
const float RPM_MAXIMA = 150;

// ---------------- Tiempo ----------------
unsigned long previousMillis = 0;
const long interval = 100;   

// ---------------- Variables de control ----------------
float sp = 0; float pv = 0;                
int direccionMotor = 0;

void IRAM_ATTR interrupcionEncoder() {
  bool A = digitalRead(pinEncoderA);
  bool B = digitalRead(pinEncoderB);
  if (A == B) contador++; else contador--;
}

// ---------------- Manejador de Eventos WebSocket ----------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    
    // --- CONTROL DEL MOTOR ---
    if (msg.startsWith("M:")) {
      float norm = msg.substring(2).toFloat(); 
      if (abs(norm) <= 0.05) { 
        direccionMotor = 0; sp = 0;
        cv = 0; cv1 = 0; error = 0; error1 = 0; error2 = 0;
        analogWrite(LPWM_M1, 0); 
        analogWrite(RPWM_M1, 0);
        analogWrite(LPWM_M2, 0); 
        analogWrite(RPWM_M2, 0);
      } 
      else if (norm > 0.05) { 
        direccionMotor = 1; sp = norm * RPM_MAXIMA; 
      } 
      else if (norm < -0.05) { 
        direccionMotor = -1; sp = abs(norm) * RPM_MAXIMA; 
      }
    }
    
    // --- CONTROL DE AMBOS SERVOS SIMÉTRICOS (ON/OFF) ---
    else if (msg.startsWith("S:")) {
      String comando = msg.substring(2);
      
      if (comando == "ABRIR") {
        servoEstadoReal = 57; 
        servoIzq.write(57);  
        servoDer.write(0);  
        
        Serial.println("[SERVOS] Comando: ABRIR -> Izq: 57° | Der: 0°");
      } 
      else if (comando == "CERRAR") {
        servoEstadoReal = 0;  
        servoIzq.write(0); 
        servoDer.write(57);  
        
        Serial.println("[SERVOS] Comando: CERRAR -> Izq: 0° | Der: 57°");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  pinMode(pinEncoderA, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
  
  // Salidas para los dos controladores de motor
  pinMode(LPWM_M1, OUTPUT); 
  pinMode(RPWM_M1, OUTPUT);
  pinMode(LPWM_M2, OUTPUT); 
  pinMode(RPWM_M2, OUTPUT);

  // Inicializar ambos Servos de manera limpia
  servoIzq.attach(pinServoIzq);
  servoDer.attach(pinServoDer);
  
  // Posición inicial: AMBOS CERRADOS al encender el robot
  servoIzq.write(0);
  servoDer.write(57);

  // Configuración de resoluciones PWM a 8 bits en los 4 canales necesarios del ESP32-S3
  analogWriteResolution(LPWM_M1, 8); 
  analogWriteResolution(RPWM_M1, 8); 
  analogWriteResolution(LPWM_M2, 8); 
  analogWriteResolution(RPWM_M2, 8); 

  // Forzar apagado total inicial
  analogWrite(LPWM_M1, 0); 
  analogWrite(RPWM_M1, 0);
  analogWrite(LPWM_M2, 0); 
  analogWrite(RPWM_M2, 0);
  
  attachInterrupt(digitalPinToInterrupt(pinEncoderA), interrupcionEncoder, CHANGE);
  Serial.println("Sistema de Tracción + Mecanismos Dobles Listo.");
}

void loop() {
  webSocket.loop();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    noInterrupts();
    long pulsos = contador;
    contador = 0;
    interrupts();

    pv = (abs(pulsos) * 10.0 * 60.0) / PULSOS_POR_VUELTA;

    if (direccionMotor == 0 || sp == 0) {
      analogWrite(LPWM_M1, 0); analogWrite(RPWM_M1, 0);
      analogWrite(LPWM_M2, 0); analogWrite(RPWM_M2, 0);
      cv = 0; cv1 = 0; error = 0; error1 = 0; error2 = 0;
    }
    else {
      error = sp - pv;
      cv = cv1 + (kp + kd / Tm) * error + (-kp + ki * Tm - 2 * kd / Tm) * error1 + (kd / Tm) * error2;
      cv1 = cv; error2 = error1; error1 = error;
      if (cv > 500) cv = 500; if (cv < 0) cv = 0;
      int pwm_final = (cv * 255.0) / 500.0;

      // Sincronización en paralelo de ambos puentes H según comando del Joystick
      if (direccionMotor == 1) { 
        analogWrite(RPWM_M1, pwm_final); analogWrite(LPWM_M1, 0); 
        analogWrite(RPWM_M2, pwm_final); analogWrite(LPWM_M2, 0); 
      }
      else if (direccionMotor == -1) { 
        analogWrite(RPWM_M1, 0); analogWrite(LPWM_M1, pwm_final); 
        analogWrite(RPWM_M2, 0); analogWrite(LPWM_M2, pwm_final); 
      }
    }

    // --- TELEMETRÍA (Envía RPM y el estado global de los servos) ---
    String jsonTelemetry = "{\"pv\":" + String(pv, 0) + ", \"servo\":" + String(servoEstadoReal) + "}";
    webSocket.broadcastTXT(jsonTelemetry);
  }
}