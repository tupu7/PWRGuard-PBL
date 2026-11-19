/**
 * ============================================================================
 * POWERGuard 5 — Dashboard de Telemetría y Control de Factor de Potencia
 * Instituto Kriete de Ingeniería y Ciencias · Circuitos Eléctricos
 * Equipo: Diana Carballo, Emma Palacios, Eduardo Figueroa, Juan Pablo Ramírez
 * Microcontrolador: Arduino Nano (ATmega328P, 5V, ADC 10 bits)
 * ============================================================================
 */

(() => {
  'use strict';

  // --- CONSTANTES DEL SISTEMA Y HARDWARE ---
  const FREQ_RED = 60.0;
  const OMEGA = 2.0 * Math.PI * FREQ_RED; // 376.9911 rad/s
  const LOCK_TIME_MS = 30000; // 30 segundos de descarga de seguridad (5 tau para 60uF con R=100k)
  const TAU_RC = [1.5, 3.0, 6.0]; // Tau para 15uF, 30uF y 60uF con R=100kΩ (s)
  const STEP_UF = [15.0, 30.0, 60.0];
  const STEP_VAR_120V = [81.43, 162.86, 325.72]; // Inyección nominal a 120V

  // Carga base del Motor de Inducción WEG W01 (0.5 HP, 115/120V, 60Hz, 8.24A nom)
  // CONDICIÓN EXPERIMENTAL: Motor siempre en vacío con FP máx sin compensar ≈ 0.20
  // y picos de corriente transitorios de arranque entre 10 y 15 A.
  const MOTOR_PRESETS = {
    empty: { p: 130.0, q: 640.0, v: 120.0, label: 'En Vacío (Lab Real: FP ≈ 0.20 · 130W / 640 VAR)' },
    mid:   { p: 350.0, q: 650.0, v: 120.0, label: 'Carga Media Teórica (350W / 650 VAR · FP 0.47)' },
    full:  { p: 588.0, q: 744.0, v: 120.0, label: 'Plena Carga Teórica (588W / 744 VAR · FP 0.62)' },
    off:   { p: 0.0,   q: 0.0,   v: 120.0, label: 'Motor Apagado (0W / 0 VAR)' }
  };

  // --- ESTADO GLOBAL DE LA APLICACIÓN ---
  const state = {
    // Modo de operación: 'sim' (Simulación) o 'serial' (Hardware Real USB)
    mode: 'sim',
    isAutoControl: true,
    motorPresetKey: 'empty', // Por defecto: la condición real de laboratorio en vacío

    // Parámetros de la simulación
    simMotor: { ...MOTOR_PRESETS.empty },
    
    // Simulación de transitorio de arranque (Pico 10 - 15 A)
    peakCurrentRecorded: 13.8, // Valor máximo registrado (Max-Hold)
    inrushMultiplier: 1.0,
    inrushDecayTimer: null,

    // Estado de los 3 pasos de capacitores [Paso 1 (15uF), Paso 2 (30uF), Paso 3 (60uF)]
    stepState: [false, false, false],
    lastOffTime: [0, 0, 0], // Timestamp de cuando se apagó cada paso para el timer de 30s

    // Medidas instantáneas adquiridas (vía USB o calculadas en simulación)
    telemetry: {
      vRms: 120.0,
      iRms: 5.44,
      pAct: 130.0,
      qRea: 640.0,
      sApp: 653.1,
      fp: 0.199,
      phiDeg: 78.5,
      deltaTMs: 3.63,
      isCapacitive: false,
      qcReq: 574.0,
      cReqUF: 105.7,
      qInjected: 0.0,
      cTotalUF: 0.0,
      iReductionPct: 0.0,
      fpTeorico: 0.199,
      errorPct: 0.0
    },

    // Temporizador de histéresis automática
    lastAutoActionTime: 0,
    autoActionDelayMs: 4500, // Tiempo mínimo entre maniobras automáticas

    // Buffer histórico para tabla y exportación CSV
    telemetryHistory: [],
    maxHistorySize: 100,

    // Web Serial API handles
    serial: {
      port: null,
      reader: null,
      writer: null,
      keepReading: false,
      textDecoder: null,
      buffer: ''
    },

    // Estado del osciloscopio
    scope: {
      paused: false,
      timebaseMsDiv: 2.0, // ms por división
      vScale: 40.0, // Voltios por división
      iScale: 2.0,  // Amperios por división
      timeSec: 0
    }
  };

  // --- REFERENCIAS A ELEMENTOS DEL DOM ---
  const dom = {
    // Header & Modes
    modeBadge: document.getElementById('modeBadge'),
    modeText: document.getElementById('modeText'),
    toggleAutoBtn: document.getElementById('toggleAutoBtn'),
    autoIndicatorDot: document.getElementById('autoIndicatorDot'),
    autoBtnText: document.getElementById('autoBtnText'),
    btnConnectSerial: document.getElementById('btnConnectSerial'),
    serialBtnLabel: document.getElementById('serialBtnLabel'),

    // Motor Presets & Inrush Simulator
    btnLoadEmpty: document.getElementById('btnLoadEmpty'),
    btnLoadMid: document.getElementById('btnLoadMid'),
    btnLoadFull: document.getElementById('btnLoadFull'),
    btnLoadOff: document.getElementById('btnLoadOff'),
    btnSimInrush: document.getElementById('btnSimInrush'),

    // Quick Command Buttons
    btnSendTest: document.getElementById('btnSendTest'),
    btnSendCalib: document.getElementById('btnSendCalib'),
    btnSendOff: document.getElementById('btnSendOff'),
    btnExportCSV: document.getElementById('btnExportCSV'),

    // KPI Cards
    valVrms: document.getElementById('valVrms'),
    valIrms: document.getElementById('valIrms'),
    valIPeak: document.getElementById('valIPeak'),
    btnResetPeak: document.getElementById('btnResetPeak'),
    valIReduction: document.getElementById('valIReduction'),
    valP: document.getElementById('valP'),
    valPLoadPct: document.getElementById('valPLoadPct'),
    valQ: document.getElementById('valQ'),
    badgeQState: document.getElementById('badgeQState'),
    valQInjected: document.getElementById('valQInjected'),
    valS: document.getElementById('valS'),
    valDeltaT: document.getElementById('valDeltaT'),
    valFP: document.getElementById('valFP'),
    valPhiDeg: document.getElementById('valPhiDeg'),
    fpStatusBadge: document.getElementById('fpStatusBadge'),
    fpGaugeProgress: document.getElementById('fpGaugeProgress'),
    fpZoneLabel: document.getElementById('fpZoneLabel'),

    // Bank Summary Badges
    badgeCapTotal: document.getElementById('badgeCapTotal'),
    badgeBinaryCode: document.getElementById('badgeBinaryCode'),

    // 3 LEDs Hardware Panel
    bezelLed1: document.getElementById('bezelLed1'),
    coreLed1: document.getElementById('coreLed1'),
    badgeStateLed1: document.getElementById('badgeStateLed1'),
    lockTimerText1: document.getElementById('lockTimerText1'),
    lockProgressBar1: document.getElementById('lockProgressBar1'),
    btnToggleStep1: document.getElementById('btnToggleStep1'),

    bezelLed2: document.getElementById('bezelLed2'),
    coreLed2: document.getElementById('coreLed2'),
    badgeStateLed2: document.getElementById('badgeStateLed2'),
    lockTimerText2: document.getElementById('lockTimerText2'),
    lockProgressBar2: document.getElementById('lockProgressBar2'),
    btnToggleStep2: document.getElementById('btnToggleStep2'),

    bezelLed3: document.getElementById('bezelLed3'),
    coreLed3: document.getElementById('coreLed3'),
    badgeStateLed3: document.getElementById('badgeStateLed3'),
    lockTimerText3: document.getElementById('lockTimerText3'),
    lockProgressBar3: document.getElementById('lockProgressBar3'),
    btnToggleStep3: document.getElementById('btnToggleStep3'),

    // Oscilloscope & Triangle
    scopeCanvas: document.getElementById('scopeCanvas'),
    btnPauseScope: document.getElementById('btnPauseScope'),
    hudVpp: document.getElementById('hudVpp'),
    hudIpp: document.getElementById('hudIpp'),
    hudPhi: document.getElementById('hudPhi'),
    hudDt: document.getElementById('hudDt'),
    triangleCanvas: document.getElementById('triangleCanvas'),
    triS1: document.getElementById('triS1'),
    triQL: document.getElementById('triQL'),
    triS2: document.getElementById('triS2'),
    triQnet: document.getElementById('triQnet'),
    triFP: document.getElementById('triFP'),

    // Truth Table
    truthTableBody: document.getElementById('truthTableBody'),

    // Math Formulas
    calcVrmsStep: document.getElementById('calcVrmsStep'),
    calcIrmsStep: document.getElementById('calcIrmsStep'),
    calcPVal: document.getElementById('calcPVal'),
    calcSVal: document.getElementById('calcSVal'),
    calcFPVal: document.getElementById('calcFPVal'),
    calcQSignBadge: document.getElementById('calcQSignBadge'),
    calcPhiVal: document.getElementById('calcPhiVal'),
    calcDtVal: document.getElementById('calcDtVal'),
    calcQcReqStep: document.getElementById('calcQcReqStep'),
    calcCReqVal: document.getElementById('calcCReqVal'),
    valFPTeo: document.getElementById('valFPTeo'),
    badgeErrorPct: document.getElementById('badgeErrorPct'),

    // Telemetry Table & Serial Terminal
    telemetryTableBody: document.getElementById('telemetryTableBody'),
    recordCountBadge: document.getElementById('recordCountBadge'),
    btnClearTelemetry: document.getElementById('btnClearTelemetry'),
    terminalOutput: document.getElementById('terminalOutput'),
    btnClearTerminal: document.getElementById('btnClearTerminal'),
    termInput: document.getElementById('termInput'),
    termSendBtn: document.getElementById('termSendBtn'),

    // Modal
    modalDocs: document.getElementById('modalDocs'),
    btnOpenDocs: document.getElementById('btnOpenDocs'),
    btnCloseDocs: document.getElementById('btnCloseDocs'),
    btnModalCloseAction: document.getElementById('btnModalCloseAction')
  };

  // ==========================================================================
  // CÁLCULOS MATEMÁTICOS DE INGENIERÍA ELÉCTRICA
  // ==========================================================================

  /**
   * Calcula el estado eléctrico completo para una carga dada y combinación de capacitores activa.
   */
  function computeElectricalState(vRms, pMotor, qMotor, stepStates) {
    // 1. Capacitancia total conectada y reactiva aportada
    let cTotalUF = 0.0;
    if (stepStates[0]) cTotalUF += STEP_UF[0];
    if (stepStates[1]) cTotalUF += STEP_UF[1];
    if (stepStates[2]) cTotalUF += STEP_UF[2];

    const cTotalFarads = cTotalUF * 1e-6;
    const qInjected = OMEGA * cTotalFarads * (vRms * vRms); // Inyección Qc = omega * C * V^2

    // 2. Potencia reactiva neta que ve la red (Q_net = Q_motor - Q_capacitores)
    const qNet = qMotor - qInjected;
    const isCapacitive = (qNet < -5.0); // Margen de histéresis

    // 3. Potencia aparente neta resultante: S = sqrt(P^2 + Q^2)
    const sApp = Math.sqrt(pMotor * pMotor + qNet * qNet);

    // 4. Corriente eficaz de línea total: I_rms = S / V_rms
    let iRms = (vRms > 10.0) ? (sApp / vRms) : 0.0;

    // Aplicar transitorio de arranque (pico de 10 - 15 A)
    if (state.inrushMultiplier > 1.0) {
      iRms *= state.inrushMultiplier;
    }

    // Actualizar registro Max-Hold de pico de corriente
    if (iRms > state.peakCurrentRecorded) {
      state.peakCurrentRecorded = iRms;
    }

    // Corriente sin compensar en estado estacionario (para cálculo de % de reducción)
    const sUncompensated = Math.sqrt(pMotor * pMotor + qMotor * qMotor);
    const iUncomp = (vRms > 10.0) ? (sUncompensated / vRms) : 0.0;
    const iSteadyState = (vRms > 10.0) ? (sApp / vRms) : 0.0;
    const iReductionPct = (iUncomp > 0.1) ? Math.max(0, ((iUncomp - iSteadyState) / iUncomp) * 100) : 0.0;

    // 5. Factor de Potencia medido verdadero: FP = P / S
    let fp = (sApp > 1.0) ? (Math.abs(pMotor) / sApp) : 1.0;
    if (fp > 1.0) fp = 1.0;

    // 6. Ángulo de desfase phi y tiempo de cruce por cero delta t
    const phiRad = Math.acos(fp);
    const phiDeg = phiRad * (180.0 / Math.PI);
    const deltaTMs = (phiDeg / (360.0 * FREQ_RED)) * 1000.0;

    // 7. Dimensionamiento de reactiva y capacitancia requerida para meta FP2 = 0.97
    // tan(phi_2) = tan(arccos(0.97)) = 0.2506
    let qcReq = 0.0;
    let cReqUF = 0.0;
    if (!isCapacitive && fp < 0.95 && pMotor > 10.0) {
      const tanPhi1 = Math.tan(phiRad);
      qcReq = pMotor * (tanPhi1 - 0.2506);
      if (qcReq > 0 && vRms > 50.0) {
        cReqUF = (qcReq / (OMEGA * vRms * vRms)) * 1e6;
      }
    }

    // 8. FP Teórico predicho por el modelo ideal para validación de error
    const fpTeorico = (sApp > 0.1) ? (pMotor / sApp) : 1.0;
    // Error experimental entre modelo y simulación con ruido
    const errorPct = Math.abs(fp - fpTeorico) / (fpTeorico || 1) * 100;

    return {
      vRms,
      iRms,
      pAct: pMotor,
      qRea: qNet,
      sApp,
      fp,
      phiDeg,
      deltaTMs,
      isCapacitive,
      qcReq,
      cReqUF,
      qInjected,
      cTotalUF,
      iReductionPct,
      fpTeorico,
      errorPct
    };
  }

  /**
   * Simula un transitorio real de corriente de arranque con pico entre 10 y 15 A.
   */
  function triggerInrushPeak() {
    if (state.simMotor.p < 10.0 && state.simMotor.q < 10.0) {
      logTerminal('[INRUSH] El motor está apagado. Selecciona un estado de marcha (En Vacío o Carga) para arrancar.');
      return;
    }
    // Generar un pico realista entre 11.0 y 14.7 A
    const targetPeak = 11.0 + Math.random() * 3.7;
    const baseI = Math.max(0.5, (state.telemetry.sApp || 653) / (state.telemetry.vRms || 120));
    state.inrushMultiplier = targetPeak / baseI;

    if (targetPeak > state.peakCurrentRecorded) {
      state.peakCurrentRecorded = targetPeak;
    }

    logTerminal(`[INRUSH] ¡Transitorio de arranque simulado! Pico de corriente: ${targetPeak.toFixed(2)} A (Medido por ACS712-30A, rango ±30A).`);

    if (dom.valIPeak) {
      dom.valIPeak.textContent = `${state.peakCurrentRecorded.toFixed(1)} A`;
    }

    if (state.inrushDecayTimer) clearInterval(state.inrushDecayTimer);
    const startTime = Date.now();
    const durationMs = 2000;

    state.inrushDecayTimer = setInterval(() => {
      const elapsed = Date.now() - startTime;
      if (elapsed >= durationMs) {
        state.inrushMultiplier = 1.0;
        clearInterval(state.inrushDecayTimer);
        state.inrushDecayTimer = null;
      } else {
        const progress = elapsed / durationMs;
        state.inrushMultiplier = 1.0 + ((targetPeak / baseI) - 1.0) * Math.exp(-progress * 4.5);
      }
    }, 50);
  }

  /**
   * Reinicia el indicador Max-Hold de corriente pico.
   */
  function resetPeakCurrent() {
    state.peakCurrentRecorded = state.telemetry.iRms || 5.44;
    if (dom.valIPeak) {
      dom.valIPeak.textContent = `${state.peakCurrentRecorded.toFixed(1)} A`;
    }
    logTerminal(`[PICO MÁX] Indicador Max-Hold reiniciado a ${state.peakCurrentRecorded.toFixed(2)} A.`);
  }

  // ==========================================================================
  // SEGURIDAD: CONTROL DE BLOQUEO DE DESCARGA RC (30 SEGUNDOS)
  // ==========================================================================

  /**
   * Verifica si un paso de capacitor puede reconectarse respetando el bloqueo de 30s.
   */
  function isStepLocked(idx) {
    if (idx < 0 || idx > 2) return false;
    if (state.stepState[idx]) return false; // Ya está encendido
    if (state.lastOffTime[idx] === 0) return false;

    const elapsed = Date.now() - state.lastOffTime[idx];
    return elapsed < LOCK_TIME_MS;
  }

  /**
   * Obtiene los segundos restantes de descarga para un paso dado.
   */
  function getLockRemainingSec(idx) {
    if (idx < 0 || idx > 2) return 0;
    if (state.stepState[idx] || state.lastOffTime[idx] === 0) return 0;

    const elapsed = Date.now() - state.lastOffTime[idx];
    if (elapsed >= LOCK_TIME_MS) return 0;
    return (LOCK_TIME_MS - elapsed) / 1000.0;
  }

  /**
   * Calcula el voltaje remanente estimado en el capacitor según la curva de descarga RC:
   * V(t) = V_peak * exp(-t / tau), donde V_peak = sqrt(2) * 120V = 169.7V
   */
  function getEstimatedDischargeVoltage(idx) {
    const remSec = getLockRemainingSec(idx);
    if (remSec <= 0) return 0.0;
    const elapsedSec = (LOCK_TIME_MS / 1000.0) - remSec;
    const tau = TAU_RC[idx];
    const vPeak = Math.SQRT2 * (state.telemetry.vRms || 120.0);
    const vRem = vPeak * Math.exp(-elapsedSec / tau);
    return Math.max(0.0, vRem);
  }

  /**
   * Enciende o apaga un paso físico/simulado respetando bloqueos.
   */
  function setStep(idx, newState, bypassLock = false) {
    if (idx < 0 || idx > 2) return false;

    // Intentar encender
    if (newState && !state.stepState[idx]) {
      if (!bypassLock && isStepLocked(idx)) {
        const rem = getLockRemainingSec(idx).toFixed(1);
        logTerminal(`[BLOQUEO DE SEGURIDAD] Paso ${idx + 1} (${STEP_UF[idx]} µF) descargando. Restan ${rem} s (τ = ${TAU_RC[idx]}s). Reconexión inhibida.`);
        return false;
      }
      state.stepState[idx] = true;
      logTerminal(`[BANCO] Paso ${idx + 1} (LED ${['ROJO','AZUL','VERDE'][idx]}, ${STEP_UF[idx]} µF) -> CONECTADO`);
      if (state.mode === 'serial') {
        sendSerialCommand(String(idx + 1));
      }
      return true;
    }
    // Apagar
    else if (!newState && state.stepState[idx]) {
      state.stepState[idx] = false;
      state.lastOffTime[idx] = Date.now();
      logTerminal(`[BANCO] Paso ${idx + 1} (LED ${['ROJO','AZUL','VERDE'][idx]}, ${STEP_UF[idx]} µF) -> DESCONECTADO (Iniciando temporizador RC 30s)`);
      if (state.mode === 'serial') {
        sendSerialCommand(String(idx + 1));
      }
      return true;
    }
    return false;
  }

  function turnOffAllSteps(bypassLock = true) {
    logTerminal(`[BANCO] Desconexión de emergencia / Apagar todos los pasos [0]`);
    for (let i = 0; i < 3; i++) {
      if (state.stepState[i]) {
        state.stepState[i] = false;
        state.lastOffTime[i] = Date.now();
      }
    }
    if (state.mode === 'serial') {
      sendSerialCommand('0');
    }
  }

  // ==========================================================================
  // LAZO DE CONTROL AUTOMÁTICO (HISTÉRESIS 0.95 - 0.99)
  // ==========================================================================
  function runAutoControlLoop() {
    if (!state.isAutoControl) return;

    const now = Date.now();
    if (now - state.lastAutoActionTime < state.autoActionDelayMs) {
      return; // Esperar ventana mínima entre maniobras para estabilidad de red
    }

    const { pAct, qRea, fp, isCapacitive, vRms } = state.telemetry;

    // 1. Regla de seguridad: si el motor está apagado o I < 0.5A, apagar capacitores para evitar sobretensión capacitiva
    if (pAct < 20.0 || state.telemetry.iRms < 0.5) {
      const anyOn = state.stepState.some(Boolean);
      if (anyOn) {
        logTerminal(`[AUTO] Motor en reposo o corriente mínima. Desconectando banco por seguridad.`);
        turnOffAllSteps(true);
        state.lastAutoActionTime = now;
      }
      return;
    }

    // 2. Si ya está en la banda óptima [0.95 - 0.99] y no es capacitivo, mantener estado
    if (!isCapacitive && fp >= 0.95 && fp <= 0.995) {
      return;
    }

    // 3. Si hay sobrecompensación capacitiva (Q < -5 VAR o FP < 0.95 adelantado), reducir capacitancia
    if (isCapacitive || qRea < -5.0) {
      // Intentar apagar primero el paso más grande disponible o el ajuste fino
      for (let i = 2; i >= 0; i--) {
        if (state.stepState[i]) {
          logTerminal(`[AUTO] Sobrecompensación detectada (Q = ${qRea.toFixed(1)} VAR). Apagando Paso ${i + 1} (${STEP_UF[i]} µF).`);
          setStep(i, false);
          state.lastAutoActionTime = now;
          return;
        }
      }
    }

    // 4. Si el FP es deficiente (< 0.95 inductivo), buscar la mejor combinación binaria de las 8 posibles
    // que acerque el FP a 0.97 sin provocar sobrecompensación capacitiva y cuyos pasos apagados NO estén bloqueados por descarga
    if (fp < 0.95) {
      let bestCombination = null;
      let minFpDistance = 999.0;

      // Evaluar las 8 combinaciones posibles (000 a 111)
      for (let code = 1; code < 8; code++) {
        const candidateSteps = [
          Boolean(code & 1), // Paso 1 (15 uF)
          Boolean(code & 2), // Paso 2 (30 uF)
          Boolean(code & 4)  // Paso 3 (60 uF)
        ];

        // Verificar si la combinación requiere encender un paso que está en bloqueo de 30s
        let hasLockedStep = false;
        for (let i = 0; i < 3; i++) {
          if (candidateSteps[i] && !state.stepState[i] && isStepLocked(i)) {
            hasLockedStep = true;
            break;
          }
        }
        if (hasLockedStep) continue; // No se puede usar esta combinación por ahora

        // Calcular el resultado predicho para esta combinación
        const pred = computeElectricalState(vRms, pAct, state.simMotor.q, candidateSteps);

        // Descartar si provoca sobrecompensación severa
        if (pred.qRea < -20.0) continue;

        // Distancia a la meta FP = 0.97
        const distance = Math.abs(pred.fp - 0.970);
        if (distance < minFpDistance) {
          minFpDistance = distance;
          bestCombination = candidateSteps;
        }
      }

      // Aplicar la mejor combinación encontrada
      if (bestCombination) {
        let changed = false;
        for (let i = 0; i < 3; i++) {
          if (bestCombination[i] !== state.stepState[i]) {
            setStep(i, bestCombination[i]);
            changed = true;
          }
        }
        if (changed) {
          logTerminal(`[AUTO] Algoritmo de optimización seleccionó combinación óptima [${bestCombination.map(b => b ? 1 : 0).join(', ')}]. Meta FP = 0.97.`);
          state.lastAutoActionTime = now;
        }
      }
    }
  }

  // ==========================================================================
  // MOTOR DE SIMULACIÓN (MODO SIN ARDUINO)
  // ==========================================================================
  function runSimulationStep() {
    if (state.mode !== 'sim') return;

    // Agregar ligera fluctuación estocástica de laboratorio (±0.4 V, ±0.03 A, ±2 W)
    const noiseV = (Math.random() - 0.5) * 0.6;
    const noiseP = (Math.random() - 0.5) * 3.0;
    const noiseQ = (Math.random() - 0.5) * 4.0;

    const vSim = Math.max(0, state.simMotor.v + noiseV);
    const pSim = Math.max(0, state.simMotor.p + (state.simMotor.p > 10 ? noiseP : 0));
    const qSim = Math.max(0, state.simMotor.q + (state.simMotor.q > 10 ? noiseQ : 0));

    // Computar parámetros de la red
    const result = computeElectricalState(vSim, pSim, qSim, state.stepState);
    state.telemetry = result;

    // Ejecutar lógica de control automático si está habilitada
    runAutoControlLoop();

    // Actualizar interfaz y registrar en buffer histórico
    updateUI();
    recordTelemetryHistory();
  }

  // ==========================================================================
  // COMUNICACIÓN CON ARDUINO NANO (WEB SERIAL API)
  // ==========================================================================

  /**
   * Intenta conectar con el puerto serie USB del Arduino Nano a 115200 baudios.
   */
  async function connectSerial() {
    if (!('serial' in navigator)) {
      alert('La Web Serial API no está soportada en este navegador.\nPor favor, abre esta aplicación en Google Chrome, Microsoft Edge, Opera o Brave para conectar el Arduino Nano.');
      return;
    }

    try {
      // Si ya está conectado, desconectar
      if (state.serial.port) {
        await disconnectSerial();
        return;
      }

      logTerminal('[SERIAL] Solicitando selección de puerto serie USB...');
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: 'none' });

      state.serial.port = port;
      state.mode = 'serial';
      state.serial.keepReading = true;

      // Actualizar botón y badges
      dom.serialBtnLabel.textContent = 'Desconectar Arduino';
      dom.btnConnectSerial.classList.replace('from-sky-600', 'from-emerald-600');
      dom.btnConnectSerial.classList.replace('to-blue-600', 'to-teal-600');
      dom.modeBadge.classList.replace('text-emerald-400', 'text-sky-400');
      dom.modeBadge.classList.replace('border-emerald-500/30', 'border-sky-500/30');
      dom.modeBadge.classList.replace('bg-emerald-500/10', 'bg-sky-500/10');
      dom.modeText.textContent = 'ARDUINO NANO CONECTADO';

      logTerminal('[SERIAL] Conexión establecida a 115200 baudios.');
      logTerminal('[SERIAL] Enviando comando \'j\' para habilitar telemetría JSON...');

      // Crear writer
      const encoder = new TextEncoderStream();
      const outputDone = encoder.readable.pipeTo(port.writable);
      state.serial.writer = encoder.writable.getWriter();

      // Enviar 'j' para asegurar modo JSON
      await state.serial.writer.write('j\n');

      // Iniciar bucle de lectura
      readSerialLoop(port);

    } catch (err) {
      logTerminal(`[ERROR SERIAL] No se pudo conectar: ${err.message}`);
      await disconnectSerial();
    }
  }

  async function disconnectSerial() {
    state.serial.keepReading = false;
    try {
      if (state.serial.reader) {
        await state.serial.reader.cancel();
        state.serial.reader = null;
      }
      if (state.serial.writer) {
        await state.serial.writer.close();
        state.serial.writer = null;
      }
      if (state.serial.port) {
        await state.serial.port.close();
        state.serial.port = null;
      }
    } catch (e) {
      console.warn('Cierre de puerto:', e);
    }

    state.mode = 'sim';
    dom.serialBtnLabel.textContent = 'Conectar Arduino Nano (USB)';
    dom.btnConnectSerial.classList.replace('from-emerald-600', 'from-sky-600');
    dom.btnConnectSerial.classList.replace('to-teal-600', 'to-blue-600');
    dom.modeBadge.classList.replace('text-sky-400', 'text-emerald-400');
    dom.modeBadge.classList.replace('border-sky-500/30', 'border-emerald-500/30');
    dom.modeBadge.classList.replace('bg-sky-500/10', 'bg-emerald-500/10');
    dom.modeText.textContent = 'SIMULACIÓN ACTIVA';
    logTerminal('[SERIAL] Desconectado. Regresando a modo simulación local.');
  }

  async function readSerialLoop(port) {
    state.serial.textDecoder = new TextDecoder();
    state.serial.buffer = '';

    while (port.readable && state.serial.keepReading) {
      try {
        const reader = port.readable.getReader();
        state.serial.reader = reader;

        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) {
            const chunk = state.serial.textDecoder.decode(value);
            state.serial.buffer += chunk;

            // Procesar líneas delimitadas por \n
            let lineEndIdx;
            while ((lineEndIdx = state.serial.buffer.indexOf('\n')) >= 0) {
              const line = state.serial.buffer.slice(0, lineEndIdx).trim();
              state.serial.buffer = state.serial.buffer.slice(lineEndIdx + 1);
              if (line.length > 0) {
                handleSerialLine(line);
              }
            }
          }
        }
      } catch (err) {
        if (state.serial.keepReading) {
          logTerminal(`[ERROR LECTURA SERIAL] ${err.message}`);
        }
        break;
      } finally {
        if (state.serial.reader) {
          state.serial.reader.releaseLock();
          state.serial.reader = null;
        }
      }
    }
  }

  /**
   * Procesa cada línea recibida del Arduino Nano (JSON de telemetría o texto de depuración).
   */
  function handleSerialLine(line) {
    // Si es JSON del firmware: {"v":...,"i":...,"p":...,"q":...,"s":...,"fp":...,"phi":...,"cap":...,"qc_req":...,"c_req":...,"leds":[...],"lock":[...]}
    if (line.startsWith('{') && line.endsWith('}')) {
      try {
        const data = JSON.parse(line);
        
        // Mapear telemetría
        state.telemetry.vRms = data.v || 0.0;
        state.telemetry.iRms = data.i || 0.0;
        state.telemetry.pAct = data.p || 0.0;
        state.telemetry.qRea = data.q || 0.0;
        state.telemetry.sApp = data.s || 0.0;
        state.telemetry.fp = data.fp || 1.0;
        state.telemetry.phiDeg = data.phi || 0.0;
        state.telemetry.deltaTMs = (state.telemetry.phiDeg / (360.0 * FREQ_RED)) * 1000.0;
        state.telemetry.isCapacitive = (data.cap === 1);
        state.telemetry.qcReq = data.qc_req || 0.0;
        state.telemetry.cReqUF = data.c_req || 0.0;

        // Actualizar estados de LEDs según el hardware
        if (Array.isArray(data.leds) && data.leds.length === 3) {
          state.stepState = data.leds.map(val => Boolean(val));
        }

        // Tiempos de descarga
        if (Array.isArray(data.lock) && data.lock.length === 3) {
          for (let i = 0; i < 3; i++) {
            if (data.lock[i] > 0) {
              state.lastOffTime[i] = Date.now() - (LOCK_TIME_MS - (data.lock[i] * 1000));
            }
          }
        }

        // Cálculos complementarios
        let cTot = 0;
        if (state.stepState[0]) cTot += STEP_UF[0];
        if (state.stepState[1]) cTot += STEP_UF[1];
        if (state.stepState[2]) cTot += STEP_UF[2];
        state.telemetry.cTotalUF = cTot;
        state.telemetry.qInjected = OMEGA * (cTot * 1e-6) * Math.pow(state.telemetry.vRms, 2);

        // Modelo teórico para error
        const qMotorEst = state.telemetry.qRea + state.telemetry.qInjected;
        const sTeo = Math.sqrt(Math.pow(state.telemetry.pAct, 2) + Math.pow(state.telemetry.qRea, 2));
        state.telemetry.fpTeorico = (sTeo > 0.1) ? (state.telemetry.pAct / sTeo) : 1.0;
        state.telemetry.errorPct = Math.abs(state.telemetry.fp - state.telemetry.fpTeorico) / (state.telemetry.fpTeorico || 1) * 100;

        // Reducción de corriente
        const sSinComp = Math.sqrt(Math.pow(state.telemetry.pAct, 2) + Math.pow(qMotorEst, 2));
        const iSinComp = (state.telemetry.vRms > 10) ? (sSinComp / state.telemetry.vRms) : 0;
        state.telemetry.iReductionPct = (iSinComp > 0.1) ? Math.max(0, ((iSinComp - state.telemetry.iRms) / iSinComp) * 100) : 0;

        updateUI();
        recordTelemetryHistory();

      } catch (err) {
        logTerminal(`[JSON PARSE ERROR] ${line}`);
      }
    } else {
      // Línea de texto normal (banners, calibración, etc.)
      logTerminal(line);
    }
  }

  /**
   * Envía un comando por el puerto serie USB al Arduino Nano.
   */
  async function sendSerialCommand(cmd) {
    if (state.serial.writer) {
      try {
        await state.serial.writer.write(cmd + '\n');
        logTerminal(`[TX >> ARDUINO] Comando '${cmd}' enviado.`);
      } catch (err) {
        logTerminal(`[TX ERROR] Fallo al enviar '${cmd}': ${err.message}`);
      }
    } else {
      logTerminal(`[TX SIM] Comando '${cmd}' procesado localmente.`);
    }
  }

  // ==========================================================================
  // ACTUALIZACIÓN DE LA INTERFAZ GRÁFICA (DOM)
  // ==========================================================================
  function updateUI() {
    const t = state.telemetry;

    // 1. Tarjetas KPI
    dom.valVrms.textContent = t.vRms.toFixed(1);
    dom.valIrms.textContent = t.iRms.toFixed(2);
    if (dom.valIPeak) {
      dom.valIPeak.textContent = `${state.peakCurrentRecorded.toFixed(1)} A`;
    }
    dom.valIReduction.textContent = `-${t.iReductionPct.toFixed(1)}%`;
    dom.valIReduction.className = (t.iReductionPct > 5.0) ? 'text-emerald-400 font-mono font-bold' : 'text-slate-400 font-mono';

    dom.valP.textContent = t.pAct.toFixed(1);
    if (state.motorPresetKey === 'empty') {
      dom.valPLoadPct.textContent = 'En Vacío (Lab)';
      dom.valPLoadPct.className = 'text-emerald-400 font-mono';
    } else {
      const pNominalMotor = 588.0;
      const loadPct = (pNominalMotor > 0) ? Math.min(150, Math.round((t.pAct / pNominalMotor) * 100)) : 0;
      dom.valPLoadPct.textContent = `${loadPct}% Carga`;
      dom.valPLoadPct.className = 'text-slate-300 font-mono';
    }

    dom.valQ.textContent = Math.abs(t.qRea).toFixed(1);
    dom.valQInjected.textContent = `${t.qInjected.toFixed(1)} VAR`;

    if (t.isCapacitive) {
      dom.badgeQState.textContent = 'ADELANTO';
      dom.badgeQState.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono bg-blue-500/20 text-blue-400 border border-blue-500/30';
    } else {
      dom.badgeQState.textContent = 'ATRASO';
      dom.badgeQState.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono bg-amber-500/20 text-amber-400 border border-amber-500/30';
    }

    dom.valS.textContent = t.sApp.toFixed(1);
    dom.valDeltaT.textContent = `${t.deltaTMs.toFixed(2)} ms`;

    // 2. Medidor de Factor de Potencia (FP) & Gauge
    dom.valFP.textContent = t.fp.toFixed(3);
    dom.valPhiDeg.textContent = t.phiDeg.toFixed(1);

    // Ajustar color y badge según el estado del FP
    if (t.isCapacitive) {
      dom.fpStatusBadge.textContent = 'SOBRECOMPENSADO';
      dom.fpStatusBadge.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono font-semibold bg-purple-500/20 text-purple-400 border border-purple-500/30';
      dom.fpGaugeProgress.className = 'text-purple-500 transition-all duration-500 ease-out';
      dom.fpZoneLabel.textContent = 'Riesgo Resonancia';
      dom.fpZoneLabel.className = 'text-purple-400 font-medium';
    } else if (t.fp >= 0.95 && t.fp <= 0.99) {
      dom.fpStatusBadge.textContent = 'ÓPTIMO (OBJETIVO)';
      dom.fpStatusBadge.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono font-semibold bg-emerald-500/20 text-emerald-400 border border-emerald-500/30';
      dom.fpGaugeProgress.className = 'text-emerald-400 transition-all duration-500 ease-out';
      dom.fpZoneLabel.textContent = 'Dentro de Norma CFE';
      dom.fpZoneLabel.className = 'text-emerald-400 font-semibold';
    } else if (t.fp >= 0.85) {
      dom.fpStatusBadge.textContent = 'ACEPTABLE';
      dom.fpStatusBadge.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono font-semibold bg-yellow-500/20 text-yellow-400 border border-yellow-500/30';
      dom.fpGaugeProgress.className = 'text-yellow-400 transition-all duration-500 ease-out';
      dom.fpZoneLabel.textContent = 'Banda Intermedia';
      dom.fpZoneLabel.className = 'text-yellow-400 font-medium';
    } else if (t.fp <= 0.22) {
      dom.fpStatusBadge.textContent = 'VACÍO (FP ≤ 0.20)';
      dom.fpStatusBadge.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono font-semibold bg-rose-500/20 text-rose-400 border border-rose-500/30';
      dom.fpGaugeProgress.className = 'text-rose-500 transition-all duration-500 ease-out';
      dom.fpZoneLabel.textContent = 'Vacío Lab (Máx 0.20)';
      dom.fpZoneLabel.className = 'text-rose-400 font-medium';
    } else {
      dom.fpStatusBadge.textContent = 'DEFICIENTE';
      dom.fpStatusBadge.className = 'px-1.5 py-0.5 rounded text-[10px] font-mono font-semibold bg-rose-500/20 text-rose-400 border border-rose-500/30';
      dom.fpGaugeProgress.className = 'text-rose-500 transition-all duration-500 ease-out';
      dom.fpZoneLabel.textContent = 'Bajo Límite CFE';
      dom.fpZoneLabel.className = 'text-rose-400 font-medium';
    }

    // Actualizar arco SVG del gauge (escala de 0.20 a 1.00 -> 0% a 100%)
    const pct = Math.max(0, Math.min(100, Math.round(t.fp * 100)));
    dom.fpGaugeProgress.setAttribute('stroke-dasharray', `${pct}, 100`);

    // 3. Resumen del Banco
    dom.badgeCapTotal.textContent = `${t.cTotalUF.toFixed(1)} µF`;
    const binStr = `[${state.stepState[0] ? 1 : 0}, ${state.stepState[1] ? 1 : 0}, ${state.stepState[2] ? 1 : 0}]`;
    const stepDec = (state.stepState[0] ? 1 : 0) + (state.stepState[1] ? 2 : 0) + (state.stepState[2] ? 4 : 0);
    dom.badgeBinaryCode.textContent = `${binStr} (Paso ${stepDec}/7)`;

    // 4. Actualizar los 3 Canales de LEDs Físicos (Rojo, Azul, Verde)
    updateLEDChannel(0, dom.bezelLed1, dom.coreLed1, dom.badgeStateLed1, dom.lockTimerText1, dom.lockProgressBar1, dom.btnToggleStep1, 'red');
    updateLEDChannel(1, dom.bezelLed2, dom.coreLed2, dom.badgeStateLed2, dom.lockTimerText2, dom.lockProgressBar2, dom.btnToggleStep2, 'blue');
    updateLEDChannel(2, dom.bezelLed3, dom.coreLed3, dom.badgeStateLed3, dom.lockTimerText3, dom.lockProgressBar3, dom.btnToggleStep3, 'green');

    // 5. Actualizar Memoria de Cálculo
    updateMathFormulas();

    // 6. Actualizar Tabla de Verdad
    updateTruthTable(stepDec);
  }

  /**
   * Actualiza el aspecto óptico, barra RC de descarga y botón de un canal LED individual.
   */
  function updateLEDChannel(idx, bezelEl, coreEl, badgeEl, lockTextEl, lockProgressEl, btnEl, colorName) {
    const isOn = state.stepState[idx];
    const remSec = getLockRemainingSec(idx);
    const isLocked = (remSec > 0);

    // Apariencia del diodo LED 3D
    if (isOn) {
      bezelEl.className = `led-bezel led-${colorName}-active`;
      coreEl.className = `led-core led-${colorName} led-on`;
      badgeEl.textContent = 'ON (1)';
      badgeEl.className = `px-2 py-0.5 rounded text-[11px] font-mono font-bold bg-${colorName === 'red' ? 'rose' : colorName === 'blue' ? 'sky' : 'emerald'}-500/20 text-${colorName === 'red' ? 'rose' : colorName === 'blue' ? 'sky' : 'emerald'}-400 border border-${colorName === 'red' ? 'rose' : colorName === 'blue' ? 'sky' : 'emerald'}-500/40`;
    } else {
      bezelEl.className = 'led-bezel';
      coreEl.className = `led-core led-${colorName} led-off`;
      badgeEl.textContent = 'OFF (0)';
      badgeEl.className = 'px-2 py-0.5 rounded text-[11px] font-mono font-bold bg-slate-800 text-slate-400 border border-slate-700';
    }

    // Barra de descarga RC y tiempo remanente
    if (isLocked) {
      const vRem = getEstimatedDischargeVoltage(idx);
      lockTextEl.textContent = `Descargando: ${remSec.toFixed(1)}s (~${vRem.toFixed(1)}V)`;
      lockTextEl.className = 'font-mono text-amber-400 text-xs font-semibold';
      const pct = (remSec / (LOCK_TIME_MS / 1000.0)) * 100;
      lockProgressEl.style.width = `${pct}%`;
    } else {
      lockTextEl.textContent = 'Descargado / Listo (0.0 V)';
      lockTextEl.className = 'font-mono text-emerald-400 text-xs font-semibold';
      lockProgressEl.style.width = '0%';
    }

    // Botón de control manual
    btnEl.disabled = (state.isAutoControl || isLocked);
    if (state.isAutoControl) {
      btnEl.title = 'Inhibido: El control automático está activo.';
    } else if (isLocked) {
      btnEl.title = `Inhibido por seguridad: Restan ${remSec.toFixed(1)} s de descarga RC.`;
    } else {
      btnEl.title = 'Clic para conmutar paso.';
    }
  }

  /**
   * Resuelve y muestra paso a paso las fórmulas de la rúbrica con los números actuales.
   */
  function updateMathFormulas() {
    const t = state.telemetry;

    // Bloque 1
    dom.calcVrmsStep.textContent = `= √( (1/320) · ${(t.vRms * t.vRms * 320).toFixed(0)} ) = ${t.vRms.toFixed(2)} V`;
    dom.calcIrmsStep.textContent = `= √( (1/320) · ${(t.iRms * t.iRms * 320).toFixed(1)} ) = ${t.iRms.toFixed(2)} A`;
    dom.calcPVal.textContent = `${t.pAct.toFixed(1)} W`;
    dom.calcSVal.textContent = `${t.sApp.toFixed(1)} VA`;
    dom.calcFPVal.textContent = `${t.fp.toFixed(3)}`;

    // Bloque 2
    if (t.isCapacitive) {
      dom.calcQSignBadge.textContent = `${t.qRea.toFixed(1)} VAR (ADELANTO / SOBRECOMPENSADO)`;
      dom.calcQSignBadge.className = 'px-2 py-0.5 rounded text-[11px] font-bold bg-purple-500/20 text-purple-300 border border-purple-500/30';
    } else {
      dom.calcQSignBadge.textContent = `+${t.qRea.toFixed(1)} VAR (ATRASO / INDUCTIVO)`;
      dom.calcQSignBadge.className = 'px-2 py-0.5 rounded text-[11px] font-bold bg-amber-500/20 text-amber-300 border border-amber-500/30';
    }
    dom.calcPhiVal.textContent = `${t.phiDeg.toFixed(1)}°`;
    dom.calcDtVal.textContent = `${t.deltaTMs.toFixed(2)} ms`;

    // Bloque 3
    const phi1Rad = Math.acos(Math.min(1.0, t.fp));
    const tanPhi1 = Math.tan(phi1Rad);
    dom.calcQcReqStep.textContent = `Q_C = ${t.pAct.toFixed(1)} · (${tanPhi1.toFixed(3)} - 0.251) = ${t.qcReq.toFixed(1)} VAR`;
    dom.calcCReqVal.textContent = `${t.cReqUF.toFixed(1)} µF`;
    dom.valFPTeo.textContent = `${t.fpTeorico.toFixed(3)}`;

    const err = t.errorPct;
    dom.badgeErrorPct.textContent = `Error: ${err.toFixed(2)}% (${err < 2.0 ? 'Excelente < 2%' : 'Aceptable < 5%'})`;
  }

  /**
   * Genera y actualiza la tabla de las 8 combinaciones de capacitores (000 a 111).
   */
  function updateTruthTable(activeStepDec) {
    if (!dom.truthTableBody) return;

    let html = '';
    const v = state.telemetry.vRms || 120.0;
    const p = state.telemetry.pAct || 588.0;
    const qMotor = (state.mode === 'sim') ? state.simMotor.q : (state.telemetry.qRea + state.telemetry.qInjected);

    for (let i = 0; i < 8; i++) {
      const b0 = (i & 1) ? 1 : 0; // Paso 1 (15 uF)
      const b1 = (i & 2) ? 1 : 0; // Paso 2 (30 uF)
      const b2 = (i & 4) ? 1 : 0; // Paso 3 (60 uF)

      const cTot = (b0 * 15.0) + (b1 * 30.0) + (b2 * 60.0);
      const qInj = OMEGA * (cTot * 1e-6) * (v * v);
      const qRes = qMotor - qInj;
      const sRes = Math.sqrt(p * p + qRes * qRes);
      let fpEst = (sRes > 0.1) ? (p / sRes) : 1.0;
      if (fpEst > 1.0) fpEst = 1.0;

      const isActive = (i === activeStepDec);
      const rowClass = isActive ? 'active-row font-bold text-white' : 'hover:bg-slate-900/40';

      const isCapacitiveRes = (qRes < -5.0);
      const fpColorClass = isCapacitiveRes ? 'text-purple-400' : (fpEst >= 0.95 ? 'text-emerald-400 font-bold' : 'text-slate-400');

      html += `
        <tr class="${rowClass} transition">
          <td class="py-2 px-3 flex items-center gap-2">
            <span class="w-2 h-2 rounded-full ${isActive ? 'bg-sky-400 animate-ping' : 'bg-slate-700'}"></span>
            <span>Paso ${i} [${b0}, ${b1}, ${b2}]</span>
          </td>
          <td class="py-2 px-3 text-red-400">${b0 ? 'ON (15 µF)' : '0'}</td>
          <td class="py-2 px-3 text-blue-400">${b1 ? 'ON (30 µF)' : '0'}</td>
          <td class="py-2 px-3 text-emerald-400">${b2 ? 'ON (60 µF)' : '0'}</td>
          <td class="py-2 px-3 font-semibold text-slate-200">${cTot.toFixed(0)} µF</td>
          <td class="py-2 px-3 text-sky-400">${qInj.toFixed(1)} VAR</td>
          <td class="py-2 px-3 ${fpColorClass}">
            ${fpEst.toFixed(3)} ${isCapacitiveRes ? '(Capacitivo)' : (fpEst >= 0.95 ? '★ Meta' : '')}
          </td>
          <td class="py-2 px-3 text-center">
            ${isActive ? '<span class="px-2 py-0.5 rounded text-[10px] bg-emerald-500/20 text-emerald-400 border border-emerald-500/40">ACTIVA</span>' : '<span class="text-slate-600">—</span>'}
          </td>
          <td class="py-2 px-3 text-right">
            <button onclick="window.selectCombination(${i})" class="px-2 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 text-[10px] transition disabled:opacity-30 disabled:cursor-not-allowed" ${state.isAutoControl ? 'disabled' : ''}>
              Activar
            </button>
          </td>
        </tr>
      `;
    }
    dom.truthTableBody.innerHTML = html;
  }

  // Hacer accesible la función de selección para los botones en la tabla
  window.selectCombination = function(code) {
    if (state.isAutoControl) {
      alert('Desactiva el Control Automático primero para seleccionar pasos manualmente.');
      return;
    }
    const b0 = Boolean(code & 1);
    const b1 = Boolean(code & 2);
    const b2 = Boolean(code & 4);

    setStep(0, b0);
    setStep(1, b1);
    setStep(2, b2);
  };

  // ==========================================================================
  // OSCILOSCOPIO DIGITAL DE 2 CANALES (CANVAS 60 FPS)
  // ==========================================================================
  function setupOscilloscope() {
    const canvas = dom.scopeCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);
    }
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    // Ciclo de animación
    let lastAnimTime = performance.now();

    function renderScope(now) {
      requestAnimationFrame(renderScope);
      const dt = (now - lastAnimTime) / 1000.0;
      lastAnimTime = now;

      if (!state.scope.paused) {
        state.scope.timeSec += dt;
      }

      const rect = canvas.getBoundingClientRect();
      const width = rect.width;
      const height = rect.height;

      // 1. Limpiar pantalla de fósforo CRT
      ctx.fillStyle = '#060a12';
      ctx.fillRect(0, 0, width, height);

      // 2. Dibujar retícula (Grid) clásica de osciloscopio
      const divX = 10; // 10 divisiones horizontales
      const divY = 8;  // 8 divisiones verticales
      const stepX = width / divX;
      const stepY = height / divY;

      ctx.strokeStyle = '#152033';
      ctx.lineWidth = 1;

      // Líneas menores
      ctx.beginPath();
      for (let x = 0; x <= width; x += stepX) {
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
      }
      for (let y = 0; y <= height; y += stepY) {
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
      }
      ctx.stroke();

      // Ejes centrales con marcas de graduación (sub-ticks)
      const midX = width / 2;
      const midY = height / 2;
      ctx.strokeStyle = '#22324e';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, midY); ctx.lineTo(width, midY);
      ctx.moveTo(midX, 0); ctx.lineTo(midX, height);
      ctx.stroke();

      // Pequeñas subdivisiones en los ejes centrales
      ctx.strokeStyle = '#32486d';
      ctx.beginPath();
      for (let x = 0; x <= width; x += stepX / 5) {
        ctx.moveTo(x, midY - 3); ctx.lineTo(x, midY + 3);
      }
      for (let y = 0; y <= height; y += stepY / 5) {
        ctx.moveTo(midX - 3, y); ctx.lineTo(midX + 3, y);
      }
      ctx.stroke();

      // 3. Parámetros de señal instantáneos
      const { vRms, iRms, phiDeg, deltaTMs } = state.telemetry;
      const vPeak = Math.SQRT2 * vRms;
      const iPeak = Math.SQRT2 * iRms;
      const phiRad = phiDeg * (Math.PI / 180.0) * (state.telemetry.isCapacitive ? -1 : 1);

      // Escalas de visualización en píxeles
      // Calibrar para que 170V pico ocupe ~3 divisiones verticales
      const vPixelsPerVolt = (height / 2) / 200.0;
      // Escala inteligente de corriente: si hay transitorio de arranque (picos 10-15A RMS -> hasta 21A pico),
      // auto-ajusta la ganancia para que el pico quepa holgadamente dentro del 82% del espacio vertical sin saturar
      const maxExpectedIPeak = Math.max(12.0, iPeak * 1.15);
      const iPixelsPerAmp = ((height / 2) * 0.82) / maxExpectedIPeak;

      // Mapear tiempo en X: 2 ms por división
      const totalTimeSpanMs = divX * state.scope.timebaseMsDiv; // e.g. 20 ms (1.2 ciclos de 60Hz)
      const msPerPixel = totalTimeSpanMs / width;

      // 4. Dibujar Canal 1: Tensión v(t) [Amarillo Neón #facc15]
      ctx.shadowBlur = 8;
      ctx.shadowColor = '#facc15';
      ctx.strokeStyle = '#facc15';
      ctx.lineWidth = 2.0;
      ctx.beginPath();

      const timeOffset = state.scope.timeSec;

      for (let px = 0; px < width; px++) {
        const tMs = px * msPerPixel;
        const tSec = (tMs / 1000.0) + (timeOffset * 0.05); // Desplazamiento visual suave
        const angle = OMEGA * tSec;
        const vInst = vPeak * Math.sin(angle);
        const py = midY - (vInst * vPixelsPerVolt);

        if (px === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      // 5. Dibujar Canal 2: Corriente i(t) [Cian Eléctrico #06b6d4]
      ctx.shadowBlur = 8;
      ctx.shadowColor = '#06b6d4';
      ctx.strokeStyle = '#06b6d4';
      ctx.lineWidth = 2.0;
      ctx.beginPath();

      for (let px = 0; px < width; px++) {
        const tMs = px * msPerPixel;
        const tSec = (tMs / 1000.0) + (timeOffset * 0.05);
        // Cuando hay desfase inductivo, i(t) retrasa a v(t) -> sin(omega*t - phi)
        const angle = OMEGA * tSec - phiRad;
        const iInst = iPeak * Math.sin(angle);
        const py = midY - (iInst * iPixelsPerAmp);

        if (px === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      // Desactivar sombras
      ctx.shadowBlur = 0;

      // 6. Actualizar mediciones HUD en pantalla
      dom.hudVpp.textContent = `${(vPeak * 2).toFixed(1)} Vp-p`;
      const isSurge = (iRms >= 9.5);
      dom.hudIpp.textContent = `${(iPeak * 2).toFixed(1)} Ap-p (${iRms.toFixed(2)} Arms) ${isSurge ? '⚡ [PICO INRUSH]' : ''}`;
      dom.hudPhi.textContent = `${phiDeg.toFixed(1)}° ${state.telemetry.isCapacitive ? '(Adelanto)' : '(Atraso)'}`;
      dom.hudDt.textContent = `${deltaTMs.toFixed(2)} ms`;
    }

    requestAnimationFrame(renderScope);
  }

  // ==========================================================================
  // TRIÁNGULO DE POTENCIAS & FASOR EN TIEMPO REAL (CANVAS)
  // ==========================================================================
  function setupPowerTriangle() {
    const canvas = dom.triangleCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);
    }
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    function renderTriangle() {
      requestAnimationFrame(renderTriangle);

      const rect = canvas.getBoundingClientRect();
      const width = rect.width;
      const height = rect.height;

      ctx.fillStyle = '#070d18';
      ctx.fillRect(0, 0, width, height);

      // Origen del plano complejo de potencias
      const originX = width * 0.15;
      const originY = height * 0.65;

      const pVal = state.telemetry.pAct || 588.0;
      const qVal = state.telemetry.qRea || 744.0;
      const qMotor = (state.mode === 'sim') ? state.simMotor.q : (state.telemetry.qRea + state.telemetry.qInjected);
      const qInjected = state.telemetry.qInjected || 0.0;

      // Escalas de dibujo (máximo ~1000 W/VAR)
      const maxVal = 1050.0;
      const scaleX = (width * 0.70) / maxVal;
      const scaleY = (height * 0.50) / maxVal;

      const pPx = pVal * scaleX;
      const qMotorPx = qMotor * scaleY;
      const qNetPx = qVal * scaleY;
      const qInjPx = qInjected * scaleY;

      // 1. Ejes coordenados P (Real) y Q (Imaginario)
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(originX - 20, originY); ctx.lineTo(width - 20, originY); // Eje P
      ctx.moveTo(originX, 20); ctx.lineTo(originX, height - 20); // Eje Q
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.font = '10px JetBrains Mono';
      ctx.fillText('+P (kW / W)', width - 75, originY - 8);
      ctx.fillText('+Q (Inductivo)', originX + 8, 30);
      ctx.fillText('-Q (Capacitivo)', originX + 8, height - 25);

      // 2. Vector Potencia Activa P (Eje horizontal verde esmeralda)
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + pPx, originY);
      ctx.stroke();
      drawArrowHead(ctx, originX, originY, originX + pPx, originY, '#10b981');

      ctx.fillStyle = '#34d399';
      ctx.fillText(`P = ${pVal.toFixed(0)} W`, originX + (pPx / 2) - 25, originY + 16);

      // 3. Vector Reactiva Motor Inicial Q_L (Línea punteada naranja hacia arriba)
      ctx.strokeStyle = '#f59e0b';
      ctx.setLineDash([4, 4]);
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(originX + pPx, originY);
      ctx.lineTo(originX + pPx, originY - qMotorPx);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillStyle = '#fbbf24';
      ctx.fillText(`Q_motor = ${qMotor.toFixed(0)} VAR`, originX + pPx + 8, originY - (qMotorPx / 2));

      // 4. Vector de Compensación Capacitiva Q_C (Flecha azul hacia abajo restando)
      if (qInjPx > 5) {
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(originX + pPx + 15, originY - qMotorPx);
        ctx.lineTo(originX + pPx + 15, originY - qMotorPx + qInjPx);
        ctx.stroke();
        drawArrowHead(ctx, originX + pPx + 15, originY - qMotorPx, originX + pPx + 15, originY - qMotorPx + qInjPx, '#38bdf8');

        ctx.fillStyle = '#38bdf8';
        ctx.fillText(`-Q_C = ${qInjected.toFixed(0)} VAR`, originX + pPx + 22, originY - qMotorPx + (qInjPx / 2));
      }

      // 5. Vector Reactiva Neta Q_neta (Sólido ámbar/púrpura)
      ctx.strokeStyle = state.telemetry.isCapacitive ? '#c084fc' : '#eab308';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(originX + pPx, originY);
      ctx.lineTo(originX + pPx, originY - qNetPx);
      ctx.stroke();

      // 6. Fasor Aparente Original S_1 (Línea punteada)
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.4)';
      ctx.setLineDash([3, 3]);
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + pPx, originY - qMotorPx);
      ctx.stroke();
      ctx.setLineDash([]);

      // 7. Fasor Aparente Resultante S_2 (Línea sólida esmeralda/cian destacada)
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + pPx, originY - qNetPx);
      ctx.stroke();
      drawArrowHead(ctx, originX, originY, originX + pPx, originY - qNetPx, '#38bdf8');

      // Arco del ángulo phi resultante
      const phiRad = state.telemetry.phiDeg * (Math.PI / 180.0);
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(originX, originY, 35, 0, -phiRad, true);
      ctx.stroke();

      ctx.fillStyle = '#38bdf8';
      ctx.fillText(`φ = ${state.telemetry.phiDeg.toFixed(1)}°`, originX + 42, originY - 10);

      // Actualizar tarjeta inferior del triángulo
      const s1 = Math.sqrt(pVal * pVal + qMotor * qMotor);
      dom.triS1.textContent = `${s1.toFixed(0)}`;
      dom.triQL.textContent = `${qMotor.toFixed(0)}`;
      dom.triS2.textContent = `${state.telemetry.sApp.toFixed(0)}`;
      dom.triQnet.textContent = `${state.telemetry.qRea.toFixed(0)}`;
      dom.triFP.textContent = `FP ${state.telemetry.fp.toFixed(3)}`;
    }

    function drawArrowHead(ctx, fromX, fromY, toX, toY, color) {
      const headLength = 10;
      const dx = toX - fromX;
      const dy = toY - fromY;
      const angle = Math.atan2(dy, dx);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(toX, toY);
      ctx.lineTo(toX - headLength * Math.cos(angle - Math.PI / 6), toY - headLength * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(toX - headLength * Math.cos(angle + Math.PI / 6), toY - headLength * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fill();
    }

    requestAnimationFrame(renderTriangle);
  }

  // ==========================================================================
  // REGISTRO DE TELEMETRÍA Y EXPORTACIÓN CSV
  // ==========================================================================
  function recordTelemetryHistory() {
    const t = state.telemetry;
    const now = new Date();
    const timeStr = now.toTimeString().split(' ')[0] + '.' + String(Math.floor(now.getMilliseconds() / 100));

    const entry = {
      timestamp: timeStr,
      isoDate: now.toISOString(),
      vRms: t.vRms,
      iRms: t.iRms,
      pAct: t.pAct,
      qRea: t.qRea,
      sApp: t.sApp,
      fp: t.fp,
      cTotalUF: t.cTotalUF,
      steps: [state.stepState[0] ? 1 : 0, state.stepState[1] ? 1 : 0, state.stepState[2] ? 1 : 0],
      isCap: t.isCapacitive ? 1 : 0,
      errorPct: t.errorPct
    };

    state.telemetryHistory.unshift(entry);
    if (state.telemetryHistory.length > state.maxHistorySize) {
      state.telemetryHistory.pop();
    }

    dom.recordCountBadge.textContent = `${state.telemetryHistory.length} registros`;

    // Renderizar las últimas 8 muestras en la tabla
    let html = '';
    const displaySamples = state.telemetryHistory.slice(0, 8);
    for (const s of displaySamples) {
      html += `
        <tr class="hover:bg-slate-900/60">
          <td class="py-1.5 px-2 text-slate-400">${s.timestamp}</td>
          <td class="py-1.5 px-2 text-yellow-400">${s.vRms.toFixed(1)}</td>
          <td class="py-1.5 px-2 text-cyan-400">${s.iRms.toFixed(2)}</td>
          <td class="py-1.5 px-2 text-slate-200">${s.pAct.toFixed(1)}</td>
          <td class="py-1.5 px-2 text-amber-400">${s.qRea.toFixed(1)}</td>
          <td class="py-1.5 px-2 ${s.fp >= 0.95 ? 'text-emerald-400 font-bold' : 'text-slate-300'}">${s.fp.toFixed(3)}</td>
          <td class="py-1.5 px-2 text-sky-400">${s.cTotalUF.toFixed(0)}</td>
          <td class="py-1.5 px-2 font-mono text-slate-400">[${s.steps.join(',')}]</td>
        </tr>
      `;
    }
    dom.telemetryTableBody.innerHTML = html;
  }

  function exportCSV() {
    if (state.telemetryHistory.length === 0) {
      alert('No hay registros de telemetría en el buffer para exportar.');
      return;
    }

    const headers = [
      'iso_timestamp',
      'v_rms_v',
      'i_rms_a',
      'potencia_activa_p_w',
      'potencia_reactiva_q_var',
      'potencia_aparente_s_va',
      'factor_de_potencia_fp',
      'capacitancia_total_uf',
      'step1_rojo_15uf',
      'step2_azul_30uf',
      'step3_verde_60uf',
      'es_capacitivo',
      'error_experimental_pct'
    ];

    let csvContent = headers.join(',') + '\n';

    // Orden cronológico (antiguo a reciente)
    const reversedHistory = [...state.telemetryHistory].reverse();
    for (const r of reversedHistory) {
      const row = [
        r.isoDate,
        r.vRms.toFixed(2),
        r.iRms.toFixed(3),
        r.pAct.toFixed(2),
        r.qRea.toFixed(2),
        r.sApp.toFixed(2),
        r.fp.toFixed(4),
        r.cTotalUF.toFixed(1),
        r.steps[0],
        r.steps[1],
        r.steps[2],
        r.isCap,
        r.errorPct.toFixed(2)
      ];
      csvContent += row.join(',') + '\n';
    }

    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    const timestampStr = new Date().toISOString().replace(/[:.]/g, '-');
    link.setAttribute('href', url);
    link.setAttribute('download', `POWERGuard5_Mediciones_${timestampStr}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);

    logTerminal(`[CSV] Archivo descargado con ${reversedHistory.length} lecturas.`);
  }

  // ==========================================================================
  // CONSOLA TERMINAL SERIE
  // ==========================================================================
  function logTerminal(msg) {
    if (!dom.terminalOutput) return;
    const time = new Date().toTimeString().split(' ')[0];
    dom.terminalOutput.textContent += `[${time}] ${msg}\n`;
    dom.terminalOutput.scrollTop = dom.terminalOutput.scrollHeight;
  }

  // ==========================================================================
  // INICIALIZACIÓN Y EVENT LISTENERS
  // ==========================================================================
  function initEventListeners() {
    // 1. Selector Auto / Manual
    dom.toggleAutoBtn.addEventListener('click', () => {
      state.isAutoControl = !state.isAutoControl;
      if (state.isAutoControl) {
        dom.autoIndicatorDot.className = 'w-2 h-2 rounded-full bg-sky-400 animate-pulse';
        dom.autoBtnText.textContent = 'Control Automático (0.95 - 0.99)';
        dom.toggleAutoBtn.classList.replace('border-amber-700', 'border-slate-700');
        logTerminal('[MODO] Control Automático Activado (Banda de Histéresis: 0.95 a 0.99).');
      } else {
        dom.autoIndicatorDot.className = 'w-2 h-2 rounded-full bg-amber-400';
        dom.autoBtnText.textContent = 'Control Manual Habilitado';
        dom.toggleAutoBtn.classList.replace('border-slate-700', 'border-amber-700');
        logTerminal('[MODO] Control Manual Habilitado (Pulsadores físicos y botones de paso activos).');
      }
      updateUI();
    });

    // 2. Conectar Arduino USB
    dom.btnConnectSerial.addEventListener('click', () => {
      connectSerial();
    });

    // 3. Botones de Cargas del Motor WEG & Transitorio Inrush
    function setMotorPreset(key) {
      const prevKey = state.motorPresetKey;
      state.motorPresetKey = key;
      state.simMotor = { ...MOTOR_PRESETS[key] };

      [dom.btnLoadEmpty, dom.btnLoadMid, dom.btnLoadFull, dom.btnLoadOff].forEach(btn => {
        if (!btn) return;
        btn.classList.remove('bg-slate-800', 'text-slate-200', 'font-medium', 'bg-rose-900/40', 'bg-emerald-950/60', 'text-emerald-300', 'border-emerald-500/40');
        btn.classList.add('text-slate-400');
      });

      if (key === 'empty') {
        dom.btnLoadEmpty.classList.add('bg-emerald-950/60', 'text-emerald-300', 'border-emerald-500/40', 'font-medium');
      } else if (key === 'mid') {
        dom.btnLoadMid.classList.add('bg-slate-800', 'text-slate-200', 'font-medium');
      } else if (key === 'full') {
        dom.btnLoadFull.classList.add('bg-slate-800', 'text-slate-200', 'font-medium');
      } else if (key === 'off') {
        dom.btnLoadOff.classList.add('bg-rose-900/40', 'text-rose-200', 'font-medium');
      }

      logTerminal(`[CARGA MOTOR] Seleccionado preset: ${MOTOR_PRESETS[key].label}`);

      // Si se pasa de OFF a cualquier marcha, disparar automáticamente el pico de arranque (10-15A)
      if (prevKey === 'off' && key !== 'off') {
        triggerInrushPeak();
      }
    }

    if (dom.btnLoadEmpty) dom.btnLoadEmpty.addEventListener('click', () => setMotorPreset('empty'));
    if (dom.btnLoadMid) dom.btnLoadMid.addEventListener('click', () => setMotorPreset('mid'));
    if (dom.btnLoadFull) dom.btnLoadFull.addEventListener('click', () => setMotorPreset('full'));
    if (dom.btnLoadOff) dom.btnLoadOff.addEventListener('click', () => setMotorPreset('off'));

    // Botón para simular pico de arranque (10-15 A)
    if (dom.btnSimInrush) {
      dom.btnSimInrush.addEventListener('click', () => {
        triggerInrushPeak();
      });
    }

    // Botón para reiniciar indicador Max-Hold de corriente
    if (dom.btnResetPeak) {
      dom.btnResetPeak.addEventListener('click', () => {
        resetPeakCurrent();
      });
    }

    // 4. Botones rápidos de hardware
    dom.btnSendTest.addEventListener('click', () => {
      logTerminal('[COMANDO] Ejecutando test secuencial de LEDs (D4, D5, D6)...');
      sendSerialCommand('t');
      // En modo simulación emular la secuencia
      if (state.mode === 'sim') {
        simulateLedTestSequence();
      }
    });

    dom.btnSendCalib.addEventListener('click', () => {
      logTerminal('[COMANDO] Calibración de referencias virtuales VB 2.5V...');
      sendSerialCommand('c');
    });

    dom.btnSendOff.addEventListener('click', () => {
      turnOffAllSteps(true);
    });

    dom.btnExportCSV.addEventListener('click', exportCSV);

    // 5. Botones individuales de los 3 pasos LED
    dom.btnToggleStep1.addEventListener('click', () => setStep(0, !state.stepState[0]));
    dom.btnToggleStep2.addEventListener('click', () => setStep(1, !state.stepState[1]));
    dom.btnToggleStep3.addEventListener('click', () => setStep(2, !state.stepState[2]));

    // 6. Controles de osciloscopio
    dom.btnPauseScope.addEventListener('click', () => {
      state.scope.paused = !state.scope.paused;
      dom.btnPauseScope.textContent = state.scope.paused ? 'Reanudar' : 'Pausa';
      dom.btnPauseScope.className = state.scope.paused ? 'px-2 py-0.5 rounded bg-emerald-800 text-emerald-200 text-[11px]' : 'px-2 py-0.5 rounded bg-slate-800 text-slate-300 text-[11px]';
    });

    // 7. Acciones de telemetría y terminal
    dom.btnClearTelemetry.addEventListener('click', () => {
      state.telemetryHistory = [];
      recordTelemetryHistory();
    });

    dom.btnClearTerminal.addEventListener('click', () => {
      dom.terminalOutput.textContent = '';
    });

    dom.termSendBtn.addEventListener('click', () => {
      const cmd = dom.termInput.value.trim();
      if (cmd) {
        sendSerialCommand(cmd);
        dom.termInput.value = '';
      }
    });

    dom.termInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        const cmd = dom.termInput.value.trim();
        if (cmd) {
          sendSerialCommand(cmd);
          dom.termInput.value = '';
        }
      }
    });

    // 8. Modal de Ficha Técnica
    function toggleModal(show) {
      if (show) {
        dom.modalDocs.classList.remove('hidden');
        dom.modalDocs.classList.add('flex');
      } else {
        dom.modalDocs.classList.remove('flex');
        dom.modalDocs.classList.add('hidden');
      }
    }
    dom.btnOpenDocs.addEventListener('click', () => toggleModal(true));
    dom.btnCloseDocs.addEventListener('click', () => toggleModal(false));
    dom.btnModalCloseAction.addEventListener('click', () => toggleModal(false));
    dom.modalDocs.addEventListener('click', (e) => {
      if (e.target === dom.modalDocs) toggleModal(false);
    });

    // Atajos de teclado para laboratorio
    window.addEventListener('keydown', (e) => {
      if (document.activeElement === dom.termInput) return;
      if (e.key === 't' || e.key === 'T') dom.btnSendTest.click();
      else if (e.key === '1') dom.btnToggleStep1.click();
      else if (e.key === '2') dom.btnToggleStep2.click();
      else if (e.key === '3') dom.btnToggleStep3.click();
      else if (e.key === '0') dom.btnSendOff.click();
      else if (e.key === 'c' || e.key === 'C') dom.btnSendCalib.click();
      else if (e.key === 'i' || e.key === 'I') triggerInrushPeak();
    });
  }

  /**
   * Emula el comando serie 't' en modo simulación (destello de LEDs secuencial).
   */
  function simulateLedTestSequence() {
    state.stepState = [false, false, false];
    updateUI();
    setTimeout(() => { state.stepState = [true, false, false]; updateUI(); }, 200);
    setTimeout(() => { state.stepState = [false, true, false]; updateUI(); }, 800);
    setTimeout(() => { state.stepState = [false, false, true]; updateUI(); }, 1400);
    setTimeout(() => { state.stepState = [true, true, true]; updateUI(); }, 2000);
    setTimeout(() => { state.stepState = [false, false, false]; updateUI(); }, 2800);
  }

  // ==========================================================================
  // ARRANQUE DE LA APLICACIÓN
  // ==========================================================================
  function start() {
    initEventListeners();
    setupOscilloscope();
    setupPowerTriangle();

    logTerminal('==================================================================');
    logTerminal(' POWERGuard 5 — Dashboard de Telemetría y Compensación de FP');
    logTerminal(' Instituto Kriete de Ingeniería y Ciencias · Circuitos Eléctricos');
    logTerminal(' CONDICIÓN REAL DE LABORATORIO:');
    logTerminal(' • Motor WEG W01 en VACÍO CONTINUO');
    logTerminal(' • FP máximo sin compensar: 0.18 - 0.20 (Inductivo puro)');
    logTerminal(' • Picos de arranque transitorios: 10 - 15 A');
    logTerminal(' • Sensor Hall ACS712-30A (Escala ±30A lineal sin saturación)');
    logTerminal(' • Banco de capacitores 1:2:4 (15, 30, 60 uF -> 105 uF max)');
    logTerminal('==================================================================');
    logTerminal('[INICIO] Sistema listo en Modo Simulación (Vacío continuo).');
    logTerminal('[INICIO] Conecta el Arduino Nano por USB para adquirir datos reales.');

    // Bucle periódico de adquisición/simulación cada 500 ms (coincidente con ventana de 10 ciclos del ATmega328P)
    setInterval(() => {
      if (state.mode === 'sim') {
        runSimulationStep();
      } else {
        // En modo serie, refrescar timers de bloqueo RC cada 500ms
        updateUI();
      }
    }, 500);

    // Ejecución inicial inmediata
    runSimulationStep();
  }

  // Ejecutar cuando el DOM esté listo
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', start);
  } else {
    start();
  }

})();

