/*
 * ============================================================================
 * PROYECTO: POWERGuard - Prueba de Hardware y Adquisición de FP Verdadero
 * EQUIPO: PowerGuard 5 (Diana Carballo, Emma Palacios, Eduardo Figueroa, Juan Pablo Ramírez)
 * INSTITUCIÓN: Instituto Kriete de Ingeniería y Ciencias - Circuitos Eléctricos
 * MICROCONTROLADOR: Arduino Nano (ATmega328P, 5V, ADC 10 bits)
 * ============================================================================
 * CONEXIONES VERIFICADAS SEGÚN ESQUEMÁTICO:
 *  - A0: Sensor de Tensión (Transformador 120/12V -> Divisor 100k/10k + VB 2.5V)
 *  - A1: Sensor de Corriente (ACS712-30A, sensibilidad 66 mV/A, centro 2.5V)
 *  - D4: Paso 1 -> LED ROJO  + MOC3063 (U3) + BTA16 (TR1) -> C1 = 15 uF (81.4 VAR)
 *  - D5: Paso 2 -> LED AZUL  + MOC3063 (U4) + BTA16 (TR2) -> C2 = 30 uF (162.9 VAR)
 *  - D6: Paso 3 -> LED VERDE + MOC3063 (U5) + BTA16 (TR3) -> C3 = 60 uF (325.7 VAR)
 * ============================================================================
 * CONDICIÓN EXPERIMENTAL DE BANCO (LABORATORIO REAL):
 *  - Carga: Motor WEG W01 0.5 HP (115V AC, 60 Hz) en VACÍO CONTINUO.
 *  - FP sin compensar: 0.18 - 0.20 (P ≈ 120-140 W, Q ≈ 600-650 VAR inductivo puro).
 *  - Picos de arranque (Inrush): 10 A a 15 A (soportados linealmente por ACS712-30A).
 *  - El banco de 105 uF compensa hasta 570 VAR, elevando el FP a > 0.95 y
 *    reduciendo la corriente eficaz de red de ~5.5 A a ~1.2 A (>75% de alivio).
 * ============================================================================
 */

#include <math.h>

// --- DEFINICIÓN DE PINES ---
const uint8_t PIN_V_ADC    = A0; // Entrada tensión aislada
const uint8_t PIN_I_ADC    = A1; // Entrada corriente ACS712-30A
const uint8_t PIN_LED_ROJO = 4;  // D4 -> C1 = 15 uF
const uint8_t PIN_LED_AZUL = 5;  // D5 -> C2 = 30 uF
const uint8_t PIN_LED_VERD = 6;  // D6 -> C3 = 60 uF

const uint8_t STEP_PINS[3] = {PIN_LED_ROJO, PIN_LED_AZUL, PIN_LED_VERD};
const float   STEP_UF[3]   = {15.0, 30.0, 60.0};

// --- PARÁMETROS DE MUESTREO (32 muestras/ciclo a 60 Hz) ---
const float    FREQ_RED         = 60.0;
const uint16_t MUESTRAS_CICLO   = 32;
const uint16_t CICLOS_VENTANA   = 10;
const uint16_t N_TOTAL          = MUESTRAS_CICLO * CICLOS_VENTANA; // 320 muestras
const uint8_t  SHIFT_90_GRADOS  = MUESTRAS_CICLO / 4;              // 8 muestras (N/4)
const uint32_t PERIODO_MUESTRA_US = 521; // 1 / (60 Hz * 32) = 520.83 us

// --- CONSTANTES DE CALIBRACIÓN HARDWARE ---
// Relación teórica T1 (120V/12V = 10) * Divisor (100k+10k)/10k = 11 -> Factor = 110.0
// Ajustar CALIB_V midiendo un foco incandescente con multímetro
float CALIB_V = 110.0; 

// ACS712-30A: 66 mV/A -> 1 / 0.066 = 15.1515 A/V
float CALIB_I = 15.1515; 

// --- TEMPORIZADORES DE SEGURIDAD (Sección 8 de la Propuesta) ---
const uint32_t TIEMPO_BLOQUEO_DESCARGA_MS = 30000UL; // 30 segundos (tau = 6s para 60uF con R=100k)
bool stepState[3] = {false, false, false};
uint32_t lastOffTime[3] = {0, 0, 0};

// Offsets de tierra virtual VB (~512 cuentas = 2.50 V)
float offsetV = 512.0;
float offsetI = 512.0;
bool jsonMode = true; // Por defecto habilitado para comunicación con Dashboard

void calibrarOffsets() {
  long sumV = 0, sumI = 0;
  for (int k = 0; k < 400; k++) {
    sumV += analogRead(PIN_V_ADC);
    sumI += analogRead(PIN_I_ADC);
    delayMicroseconds(500);
  }
  offsetV = sumV / 400.0;
  offsetI = sumI / 400.0;
}

void setStep(uint8_t idx, bool state, bool bypassLock = false) {
  if (idx > 2) return;
  if (state && !stepState[idx]) {
    // Verificar bloqueo de 30s antes de reconectar un capacitor
    uint32_t elapsed = millis() - lastOffTime[idx];
    if (!bypassLock && lastOffTime[idx] != 0 && elapsed < TIEMPO_BLOQUEO_DESCARGA_MS) {
      if (!jsonMode) {
        Serial.print(F("[BLOQUEO SEGURIDAD] Paso "));
        Serial.print(idx + 1);
        Serial.print(F(" descargando. Faltan "));
        Serial.print((TIEMPO_BLOQUEO_DESCARGA_MS - elapsed) / 1000.0, 1);
        Serial.println(F(" s"));
      }
      return;
    }
    stepState[idx] = true;
    digitalWrite(STEP_PINS[idx], HIGH);
  } else if (!state && stepState[idx]) {
    stepState[idx] = false;
    digitalWrite(STEP_PINS[idx], LOW);
    lastOffTime[idx] = millis();
  }
}

void apagarTodo() {
  for (uint8_t i = 0; i < 3; i++) setStep(i, false, true);
}

void testSecuenciaLEDs() {
  if (!jsonMode) Serial.println(F("\n--- INICIANDO TEST DE LEDS Y OPTOTRIACS (D4, D5, D6) ---"));
  apagarTodo();
  const char* nombres[3] = {"LED ROJO (D4 - C1 15uF)", "LED AZUL (D5 - C2 30uF)", "LED VERDE (D6 - C3 60uF)"};
  for (uint8_t i = 0; i < 3; i++) {
    if (!jsonMode) { Serial.print(F("Encendiendo: ")); Serial.println(nombres[i]); }
    digitalWrite(STEP_PINS[i], HIGH);
    delay(700);
    digitalWrite(STEP_PINS[i], LOW);
    delay(250);
  }
  if (!jsonMode) Serial.println(F("Encendiendo los 3 LEDs simultaneamente..."));
  for (uint8_t i = 0; i < 3; i++) digitalWrite(STEP_PINS[i], HIGH);
  delay(800);
  for (uint8_t i = 0; i < 3; i++) digitalWrite(STEP_PINS[i], LOW);
  if (!jsonMode) Serial.println(F("--- TEST FINALIZADO ---\n"));
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW); // Arranque seguro en LOW
  }

  delay(300);
  calibrarOffsets();

  // Encabezado diagnóstico inicial
  Serial.println(F("=========================================================="));
  Serial.println(F(" POWERGuard 5 - Diagnostico de Hardware y Medicion FP"));
  Serial.println(F(" Instituto Kriete de Ingenieria y Ciencias - Circuitos"));
  Serial.println(F(" Comandos: [t] Test LEDs | [1/2/3] Toggle Paso | [0] OFF"));
  Serial.println(F("           [c] Calibrar VB 2.5V | [j] Modo JSON Dashboard"));
  Serial.println(F("=========================================================="));
  Serial.print(F("Offsets iniciales -> VB_A0: ")); Serial.print(offsetV * (5.0 / 1023.0), 3);
  Serial.print(F(" V | VB_A1: ")); Serial.print(offsetI * (5.0 / 1023.0), 3); Serial.println(F(" V\n"));
}

void loop() {
  // 1. Leer comandos del Monitor Serie o Dashboard Web Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') testSecuenciaLEDs();
    else if (cmd == '1') setStep(0, !stepState[0]);
    else if (cmd == '2') setStep(1, !stepState[1]);
    else if (cmd == '3') setStep(2, !stepState[2]);
    else if (cmd == '0') apagarTodo();
    else if (cmd == 'c' || cmd == 'C') { calibrarOffsets(); }
    else if (cmd == 'j' || cmd == 'J') { jsonMode = !jsonMode; }
  }

  // 2. Adquisición de 320 muestras (10 ciclos a 60 Hz)
  float vBuffer[SHIFT_90_GRADOS] = {0}; // Buffer circular para v[k - N/4]
  uint8_t bufIdx = 0;

  double sumV2 = 0.0;
  double sumI2 = 0.0;
  double sumP  = 0.0;
  double sumQ  = 0.0;

  uint32_t tNext = micros();

  for (uint16_t k = 0; k < N_TOTAL; k++) {
    while ((int32_t)(micros() - tNext) < 0) {
      // Espera precisa para fs = 1920 Hz (520.83 us por muestra)
    }
    tNext += PERIODO_MUESTRA_US;

    int rawV = analogRead(PIN_V_ADC);
    int rawI = analogRead(PIN_I_ADC);

    // Convertir cuentas ADC (0-1023) a Voltios y Amperios instantáneos sin componente DC
    float vInst = ((rawV - offsetV) * (5.0 / 1023.0)) * CALIB_V;
    float iInst = ((rawI - offsetI) * (5.0 / 1023.0)) * CALIB_I;

    // Recuperar v[k - N/4] (desfasada 90 grados) para potencia reactiva con signo
    float vDelayed = vBuffer[bufIdx];
    vBuffer[bufIdx] = vInst;
    bufIdx = (bufIdx + 1) % SHIFT_90_GRADOS;

    sumV2 += (double)vInst * vInst;
    sumI2 += (double)iInst * iInst;
    sumP  += (double)vInst * iInst;
    if (k >= SHIFT_90_GRADOS) {
      sumQ += (double)vDelayed * iInst;
    }
  }

  // 3. Cálculos según Ecuaciones de la Sección 4
  float vRms = sqrt(sumV2 / N_TOTAL);
  float iRms = sqrt(sumI2 / N_TOTAL);
  float pAct = sumP / N_TOTAL;
  float qRea = sumQ / (N_TOTAL - SHIFT_90_GRADOS);
  float sApp = vRms * iRms;

  float fp = (sApp > 1.0) ? (fabs(pAct) / sApp) : 1.0;
  if (fp > 1.0) fp = 1.0;

  float phiDeg = acos(fp) * (180.0 / PI);
  bool esCapacitivo = (qRea < -5.0); // Q < 0 indica adelanto (sobrecompensación)

  // Cálculo de capacitancia requerida para meta FP2 = 0.97 (tan(14.07°) = 0.2506)
  float qcReq = 0.0;
  float cReq_uF = 0.0;
  if (!esCapacitivo && fp < 0.95 && pAct > 10.0) {
    float phi1Rad = acos(fp);
    qcReq = pAct * (tan(phi1Rad) - 0.2506);
    float omega = 2.0 * PI * FREQ_RED;
    if (vRms > 50.0) {
      cReq_uF = (qcReq / (omega * vRms * vRms)) * 1e6;
    }
  }

  // Calcular tiempo restante de bloqueo de descarga (segundos)
  float remLock[3] = {0, 0, 0};
  for (uint8_t i = 0; i < 3; i++) {
    if (!stepState[i] && lastOffTime[i] != 0) {
      uint32_t el = millis() - lastOffTime[i];
      if (el < TIEMPO_BLOQUEO_DESCARGA_MS) {
        remLock[i] = (TIEMPO_BLOQUEO_DESCARGA_MS - el) / 1000.0;
      }
    }
  }

  // 4. Salida por Puerto Serie
  if (jsonMode) {
    Serial.print(F("{\"v\":")); Serial.print(vRms, 2);
    Serial.print(F(",\"i\":")); Serial.print(iRms, 2);
    Serial.print(F(",\"p\":")); Serial.print(pAct, 1);
    Serial.print(F(",\"q\":")); Serial.print(qRea, 1);
    Serial.print(F(",\"s\":")); Serial.print(sApp, 1);
    Serial.print(F(",\"fp\":")); Serial.print(fp, 3);
    Serial.print(F(",\"phi\":")); Serial.print(phiDeg, 1);
    Serial.print(F(",\"cap\":")); Serial.print(esCapacitivo ? 1 : 0);
    Serial.print(F(",\"qc_req\":")); Serial.print(qcReq, 1);
    Serial.print(F(",\"c_req\":")); Serial.print(cReq_uF, 1);
    Serial.print(F(",\"leds\":["));
    Serial.print(stepState[0] ? 1 : 0); Serial.print(',');
    Serial.print(stepState[1] ? 1 : 0); Serial.print(',');
    Serial.print(stepState[2] ? 1 : 0); Serial.print(F("],\"lock\":["));
    Serial.print(remLock[0], 1); Serial.print(',');
    Serial.print(remLock[1], 1); Serial.print(',');
    Serial.print(remLock[2], 1); Serial.println(F("]}"));
  } else {
    Serial.print(F("Vrms: ")); Serial.print(vRms, 1); Serial.print(F(" V | "));
    Serial.print(F("Irms: ")); Serial.print(iRms, 2); Serial.print(F(" A | "));
    Serial.print(F("P: ")); Serial.print(pAct, 1); Serial.print(F(" W | "));
    Serial.print(F("Q: ")); Serial.print(qRea, 1); Serial.print(F(" VAR | "));
    Serial.print(F("FP: ")); Serial.print(fp, 3);
    Serial.print(esCapacitivo ? F(" (ADELANTO) | ") : F(" (ATRASO)   | "));
    Serial.print(F("C_req: ")); Serial.print(cReq_uF, 1); Serial.print(F(" uF | LEDs [R,A,V]: ["));
    Serial.print(stepState[0]); Serial.print(',');
    Serial.print(stepState[1]); Serial.print(',');
    Serial.print(stepState[2]); Serial.println(F("]"));
  }

  delay(330); // Completar ventana de actualización cada ~500 ms
}

