/**
 * ============================================================================
 * POWERGuard 5 — SCADA Industrial Telemetry & Control Engine
 * Instituto Kriete de Ingeniería y Ciencias · Circuitos Eléctricos
 * Equipo: Diana Carballo, Emma Palacios, Eduardo Figueroa, Juan Pablo Ramírez
 * Microcontrolador: Arduino Nano (ATmega328P, 5V, ADC 10 bits)
 * ============================================================================
 */

(() => {
  'use strict';

  // --- CONSTANTES DEL SISTEMA Y HARDWARE ELÉCTRICO ---
  const FREQ_RED = 60.0;
  const OMEGA = 2.0 * Math.PI * FREQ_RED; // 376.991118 rad/s
  const LOCK_TIME_MS = 30000; // 30 segundos de descarga de seguridad (5 tau para 60uF con R=100k)
  const TAU_RC = [1.5, 3.0, 6.0]; // Tau para 15uF, 30uF y 60uF con R=100kΩ
  const STEP_UF = [15.0, 30.0, 60.0];

  // PARÁMETROS FÍSICOS RIGUROSOS DEL MOTOR WEG W01 (0.5 HP / 0.37 kW)
  // En Vacío: P = 130 W, Q = 600 VAR -> FP_uncomp = 0.212 | Con 105 uF (Qc=570 VAR): Qnet = 30 VAR -> FP_comp = 0.974
  // Carga Media: P = 350 W, Q = 650 VAR -> FP_uncomp = 0.474 | Con 105 uF (Qc=570 VAR): Qnet = 80 VAR -> FP_comp = 0.975
  // Plena Carga: P = 588 W, Q = 744 VAR -> FP_uncomp = 0.620 | Con 105 uF (Qc=570 VAR): Qnet = 174 VAR -> FP_comp = 0.959
  const MOTOR_PRESETS = {
    empty: { p: 130.0, q: 600.0, v: 120.0, label: 'En Vacío (Lab Real: FP ≈ 0.212 · 130 W / 600 VAR)' },
    mid:   { p: 350.0, q: 650.0, v: 120.0, label: 'Carga Media (350 W / 650 VAR · FP ≈ 0.474)' },
    full:  { p: 588.0, q: 744.0, v: 120.0, label: 'Plena Carga (588 W / 744 VAR · FP ≈ 0.620)' },
    off:   { p: 0.0,   q: 0.0,   v: 120.0, label: 'Motor Apagado (0 W / 0 VAR)' }
  };

  // --- ESTADO GLOBAL DE LA APLICACIÓN ---
  const state = {
    mode: 'sim', // 'sim' o 'serial'
    isAutoControl: true,
    motorPresetKey: 'empty', // Condición real de prueba en vacío
    presentationMode: false,

    // Carga de simulación activa
    simMotor: { ...MOTOR_PRESETS.empty },

    // Transitorio de arranque (pico 10 - 15 A)
    peakCurrentRecorded: 13.8,
    inrushMultiplier: 1.0,
    inrushDecayTimer: null,

    // Estado físico del banco de capacitores [D4 (15uF), D5 (30uF), D6 (60uF)]
    // Inicia con los capacitores conectados para alcanzar el FP óptimo en vacío de 0.974
    stepState: [true, true, true],
    lastOffTime: [0, 0, 0],

    // Telemetría instantánea (Fuente única de verdad física)
    telemetry: {
      vRms: 120.0,
      iRms: 1.11,
      pAct: 130.0,
      qRea: 30.0,
      sApp: 133.4,
      fp: 0.974,
      phiDeg: 13.0,
      deltaTMs: 0.60,
      isCapacitive: false,
      qcReq: 0.0,
      cReqUF: 0.0,
      qInjected: 570.0,
      cTotalUF: 105.0,
      iReductionPct: 78.3,
      fpTeorico: 0.974,
      errorPct: 0.0
    },

    // Lazo automático de histéresis
    lastAutoActionTime: 0,
    autoActionDelayMs: 4500,

    // Sistema centralizado de alarmas
    alarms: [],
    alarmCounterId: 1,

    // Registro histórico para tabla y CSV
    telemetryHistory: [],
    maxHistorySize: 100,

    // Manejador Web Serial
    serial: {
      port: null,
      reader: null,
      writer: null,
      keepReading: false,
      textDecoder: null,
      buffer: ''
    },

    // Osciloscopio
    scope: {
      paused: false,
      timebaseMsDiv: 2.0,
      timeSec: 0
    }
  };

  // --- REFERENCIAS A ELEMENTOS DEL DOM ---
  const dom = {
    // Header & Tabs
    tabBtnMonitor: document.getElementById('tabBtnMonitor'),
    tabBtnEngineering: document.getElementById('tabBtnEngineering'),
    tabBtnHistory: document.getElementById('tabBtnHistory'),
    tabMonitor: document.getElementById('tabMonitor'),
    tabEngineering: document.getElementById('tabEngineering'),
    tabHistory: document.getElementById('tabHistory'),

    modeBadge: document.getElementById('modeBadge'),
    modeText: document.getElementById('modeText'),
    toggleAutoBtn: document.getElementById('toggleAutoBtn'),
    autoIndicatorDot: document.getElementById('autoIndicatorDot'),
    autoBtnText: document.getElementById('autoBtnText'),
    btnConnectSerial: document.getElementById('btnConnectSerial'),
    serialBtnLabel: document.getElementById('serialBtnLabel'),
    btnOpenAlarms: document.getElementById('btnOpenAlarms'),
    alarmCountHeaderBadge: document.getElementById('alarmCountHeaderBadge'),
    btnTogglePresentation: document.getElementById('btnTogglePresentation'),
    btnOpenDocs: document.getElementById('btnOpenDocs'),

    // Motor Controls
    btnLoadEmpty: document.getElementById('btnLoadEmpty'),
    btnLoadMid: document.getElementById('btnLoadMid'),
    btnLoadFull: document.getElementById('btnLoadFull'),
    btnLoadOff: document.getElementById('btnLoadOff'),
    btnSimInrush: document.getElementById('btnSimInrush'),
    btnSendTest: document.getElementById('btnSendTest'),
    btnSendCalib: document.getElementById('btnSendCalib'),
    btnSendOff: document.getElementById('btnSendOff'),

    // Motor Visual Card
    cardMotorWeg: document.getElementById('cardMotorWeg'),
    motorStatusDot: document.getElementById('motorStatusDot'),
    badgeMotorState: document.getElementById('badgeMotorState'),
    motorRunningIndicator: document.getElementById('motorRunningIndicator'),
    motorLoadText: document.getElementById('motorLoadText'),
    motorPText: document.getElementById('motorPText'),

    // Protagonist FP Meter
    fpGaugeCanvas: document.getElementById('fpGaugeCanvas'),
    valFP: document.getElementById('valFP'),
    valPhiDeg: document.getElementById('valPhiDeg'),
    fpStatusBadge: document.getElementById('fpStatusBadge'),
    valFpRegime: document.getElementById('valFpRegime'),
    valDeltaT: document.getElementById('valDeltaT'),
    fpConditionSummary: document.getElementById('fpConditionSummary'),

    // Bank 1:2:4
    badgeCapTotal: document.getElementById('badgeCapTotal'),
    ledCore1: document.getElementById('ledCore1'),
    ledCore2: document.getElementById('ledCore2'),
    ledCore3: document.getElementById('ledCore3'),
    ledStateText1: document.getElementById('ledStateText1'),
    ledStateText2: document.getElementById('ledStateText2'),
    ledStateText3: document.getElementById('ledStateText3'),
    btnStepToggle1: document.getElementById('btnStepToggle1'),
    btnStepToggle2: document.getElementById('btnStepToggle2'),
    btnStepToggle3: document.getElementById('btnStepToggle3'),
    lockStatusSummary: document.getElementById('lockStatusSummary'),
    lockBarFill: document.getElementById('lockBarFill'),

    // KPIs
    valVrms: document.getElementById('valVrms'),
    valIrms: document.getElementById('valIrms'),
    valIReduction: document.getElementById('valIReduction'),
    valIPeak: document.getElementById('valIPeak'),
    btnResetPeak: document.getElementById('btnResetPeak'),
    valP: document.getElementById('valP'),
    valPLoadPct: document.getElementById('valPLoadPct'),
    valQ: document.getElementById('valQ'),
    badgeQState: document.getElementById('badgeQState'),
    valQMotorText: document.getElementById('valQMotorText'),
    valQInjectedText: document.getElementById('valQInjectedText'),

    // Visualizations
    scopeCanvas: document.getElementById('scopeCanvas'),
    btnPauseScope: document.getElementById('btnPauseScope'),
    hudVpp: document.getElementById('hudVpp'),
    hudIpp: document.getElementById('hudIpp'),
    hudDt: document.getElementById('hudDt'),
    hudPhi: document.getElementById('hudPhi'),
    triangleCanvas: document.getElementById('triangleCanvas'),
    triS1: document.getElementById('triS1'),
    triS2: document.getElementById('triS2'),

    // Bottom Bar
    badgeCritCount: document.getElementById('badgeCritCount'),
    badgeWarnCount: document.getElementById('badgeWarnCount'),
    btnFooterOpenAlarms: document.getElementById('btnFooterOpenAlarms'),
    btnQuickExportCSV: document.getElementById('btnQuickExportCSV'),

    // Engineering Tab
    truthTableBody: document.getElementById('truthTableBody'),
    mathMemoVrms: document.getElementById('mathMemoVrms'),
    mathMemoIrms: document.getElementById('mathMemoIrms'),
    mathMemoP: document.getElementById('mathMemoP'),
    mathMemoS: document.getElementById('mathMemoS'),
    mathMemoFP: document.getElementById('mathMemoFP'),
    mathMemoQ: document.getElementById('mathMemoQ'),
    mathMemoPhi: document.getElementById('mathMemoPhi'),
    mathMemoDt: document.getElementById('mathMemoDt'),
    mathMemoQcReq: document.getElementById('mathMemoQcReq'),
    mathMemoCReq: document.getElementById('mathMemoCReq'),
    mathMemoFPTeo: document.getElementById('mathMemoFPTeo'),
    mathMemoErrorBadge: document.getElementById('mathMemoErrorBadge'),

    // History Tab
    telemetryTableBody: document.getElementById('telemetryTableBody'),
    recordCountBadge: document.getElementById('recordCountBadge'),
    btnClearTelemetry: document.getElementById('btnClearTelemetry'),
    btnExportCSVTab: document.getElementById('btnExportCSVTab'),
    terminalOutput: document.getElementById('terminalOutput'),
    btnClearTerminal: document.getElementById('btnClearTerminal'),
    termInput: document.getElementById('termInput'),
    termSendBtn: document.getElementById('termSendBtn'),

    // Alarms Drawer
    alarmDrawer: document.getElementById('alarmDrawer'),
    alarmBackdrop: document.getElementById('alarmBackdrop'),
    btnCloseAlarms: document.getElementById('btnCloseAlarms'),
    btnAckAllAlarms: document.getElementById('btnAckAllAlarms'),
    alarmFeedContainer: document.getElementById('alarmFeedContainer'),

    // Docs Modal
    modalDocs: document.getElementById('modalDocs'),
    btnCloseDocs: document.getElementById('btnCloseDocs'),
    btnModalCloseAction: document.getElementById('btnModalCloseAction')
  };

  // ==========================================================================
  // FUENTE ÚNICA DE VERDAD: CÁLCULOS FÍSICOS RIGUROSOS
  // ==========================================================================

  /**
   * Resuelve el estado eléctrico completo respetando las leyes de circuitos de CA:
   * S = sqrt(P^2 + Q^2), FP = P / S, phi = atan2(Q, P), Qc = 2*pi*f*C*V^2.
   */
  function computeElectricalState(vRms, pMotor, qMotor, stepStates) {
    // 1. Si el motor está apagado
    if (pMotor <= 0.1 && qMotor <= 0.1) {
      return {
        vRms,
        iRms: 0.0,
        pAct: 0.0,
        qRea: 0.0,
        sApp: 0.0,
        fp: 1.0,
        phiDeg: 0.0,
        deltaTMs: 0.0,
        isCapacitive: false,
        isMotorOff: true,
        qcReq: 0.0,
        cReqUF: 0.0,
        qInjected: 0.0,
        cTotalUF: 0.0,
        iReductionPct: 0.0,
        fpTeorico: 1.0,
        errorPct: 0.0
      };
    }

    // 2. Capacitancia total conectada del banco 1:2:4
    let cTotalUF = 0.0;
    if (stepStates[0]) cTotalUF += STEP_UF[0];
    if (stepStates[1]) cTotalUF += STEP_UF[1];
    if (stepStates[2]) cTotalUF += STEP_UF[2];

    const cTotalFarads = cTotalUF * 1e-6;
    // Inyección reactiva capacitiva: Qc = omega * C * V^2
    const qInjected = OMEGA * cTotalFarads * (vRms * vRms);

    // 3. Reactiva neta vista por la red: Q_neta = Q_motor - Q_capacitores
    const qNet = qMotor - qInjected;
    const isCapacitive = (qNet < -3.0); // Adelanto cuando Q < 0

    // 4. Potencia aparente neta resultante: S = sqrt(P^2 + Q^2)
    const sApp = Math.sqrt(pMotor * pMotor + qNet * qNet);

    // 5. Corriente eficaz de línea total: I_rms = S / V_rms
    let iRms = (vRms > 10.0) ? (sApp / vRms) : 0.0;

    // Aplicar transitorio de arranque inrush (10 - 15 A)
    if (state.inrushMultiplier > 1.0) {
      iRms *= state.inrushMultiplier;
    }

    // Registrar pico máximo (Max-Hold)
    if (iRms > state.peakCurrentRecorded) {
      state.peakCurrentRecorded = iRms;
    }

    // Corriente sin compensar en estado estacionario (para porcentaje de alivio)
    const sUncomp = Math.sqrt(pMotor * pMotor + qMotor * qMotor);
    const iUncomp = (vRms > 10.0) ? (sUncomp / vRms) : 0.0;
    const iSteadyState = (vRms > 10.0) ? (sApp / vRms) : 0.0;
    const iReductionPct = (iUncomp > 0.1) ? Math.max(0, ((iUncomp - iSteadyState) / iUncomp) * 100) : 0.0;

    // 6. Factor de Potencia verdadero: FP = P / S
    let fp = (sApp > 0.1) ? (Math.abs(pMotor) / sApp) : 1.0;
    if (fp > 1.0) fp = 1.0;

    // 7. Ángulo de desfase phi = atan2(Q, P) y tiempo delta t
    const phiRad = Math.atan2(Math.abs(qNet), pMotor);
    const phiDeg = phiRad * (180.0 / Math.PI);
    const deltaTMs = (phiDeg / (360.0 * FREQ_RED)) * 1000.0;

    // 8. Dimensionamiento para meta FP2 = 0.97 (tan 14.07° = 0.2506)
    let qcReq = 0.0;
    let cReqUF = 0.0;
    if (!isCapacitive && fp < 0.95 && pMotor > 10.0) {
      const tanPhi1 = Math.tan(phiRad);
      qcReq = pMotor * (tanPhi1 - 0.2506);
      if (qcReq > 0 && vRms > 50.0) {
        cReqUF = (qcReq / (OMEGA * vRms * vRms)) * 1e6;
      }
    }

    // 9. Modelo teórico ideal para validación de error experimental
    const fpTeorico = (sApp > 0.1) ? (pMotor / sApp) : 1.0;
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
      isMotorOff: false,
      qcReq,
      cReqUF,
      qInjected,
      cTotalUF,
      iReductionPct,
      fpTeorico,
      errorPct
    };
  }

  // ==========================================================================
  // SEGURIDAD: CONTROL DE BLOQUEO DE DESCARGA RC (30 SEGUNDOS)
  // ==========================================================================

  function isStepLocked(idx) {
    if (idx < 0 || idx > 2) return false;
    if (state.stepState[idx]) return false;
    if (state.lastOffTime[idx] === 0) return false;
    const elapsed = Date.now() - state.lastOffTime[idx];
    return elapsed < LOCK_TIME_MS;
  }

  function getLockRemainingSec(idx) {
    if (idx < 0 || idx > 2) return 0;
    if (state.stepState[idx] || state.lastOffTime[idx] === 0) return 0;
    const elapsed = Date.now() - state.lastOffTime[idx];
    if (elapsed >= LOCK_TIME_MS) return 0;
    return (LOCK_TIME_MS - elapsed) / 1000.0;
  }

  function getEstimatedDischargeVoltage(idx) {
    const remSec = getLockRemainingSec(idx);
    if (remSec <= 0) return 0.0;
    const elapsedSec = (LOCK_TIME_MS / 1000.0) - remSec;
    const tau = TAU_RC[idx];
    const vPeak = Math.SQRT2 * (state.telemetry.vRms || 120.0);
    return Math.max(0.0, vPeak * Math.exp(-elapsedSec / tau));
  }

  function setStep(idx, newState, bypassLock = false) {
    if (idx < 0 || idx > 2) return false;

    // Intentar encender
    if (newState && !state.stepState[idx]) {
      if (!bypassLock && isStepLocked(idx)) {
        const rem = getLockRemainingSec(idx).toFixed(1);
        addAlarm('ADVERTENCIA', `Paso ${idx + 1} (${STEP_UF[idx]} µF) en descarga RC (${rem} s restantes). Reconexión bloqueada por seguridad.`, 'Esperar ciclo completo de 30s de enfriamiento');
        logTerminal(`[SEGURIDAD] Paso ${idx + 1} (${STEP_UF[idx]} µF) descargando. Restan ${rem} s.`);
        return false;
      }
      state.stepState[idx] = true;
      logTerminal(`[BANCO] Paso ${idx + 1} (D${idx + 4} · ${STEP_UF[idx]} µF) -> CONECTADO`);
      addAlarm('INFORMATIVA', `Paso ${idx + 1} (${STEP_UF[idx]} µF) conectado al bus AC.`, 'Monitorear respuesta del factor de potencia');
      if (state.mode === 'serial') sendSerialCommand(String(idx + 1));
      return true;
    }
    // Desconectar
    else if (!newState && state.stepState[idx]) {
      state.stepState[idx] = false;
      state.lastOffTime[idx] = Date.now();
      logTerminal(`[BANCO] Paso ${idx + 1} (D${idx + 4} · ${STEP_UF[idx]} µF) -> DESCONECTADO (Bloqueo RC 30s activo)`);
      addAlarm('INFORMATIVA', `Paso ${idx + 1} (${STEP_UF[idx]} µF) desconectado. Iniciando temporizador RC de 30s.`, 'Seguridad armada');
      if (state.mode === 'serial') sendSerialCommand(String(idx + 1));
      return true;
    }
    return false;
  }

  function turnOffAllSteps(bypassLock = true) {
    logTerminal('[BANCO] Desconexión de emergencia de todos los pasos [0]');
    for (let i = 0; i < 3; i++) {
      if (state.stepState[i]) {
        state.stepState[i] = false;
        state.lastOffTime[i] = Date.now();
      }
    }
    addAlarm('INFORMATIVA', 'Desconexión de emergencia ejecutada: Todos los capacitores apagados.', 'Verificar estado del motor');
    if (state.mode === 'serial') sendSerialCommand('0');
  }

  // ==========================================================================
  // LAZO DE CONTROL AUTOMÁTICO (HISTÉRESIS 0.95 - 0.99)
  // ==========================================================================
  function runAutoControlLoop() {
    if (!state.isAutoControl) return;

    const now = Date.now();
    if (now - state.lastAutoActionTime < state.autoActionDelayMs) return;

    const { pAct, qRea, fp, isCapacitive, isMotorOff, vRms } = state.telemetry;

    // Si el motor está en reposo, desconectar capacitores para evitar sobretensión
    if (isMotorOff || pAct < 10.0) {
      if (state.stepState.some(Boolean)) {
        logTerminal('[AUTO] Motor en reposo. Desconectando capacitores por seguridad.');
        turnOffAllSteps(true);
        state.lastAutoActionTime = now;
      }
      return;
    }

    // Si ya está en la banda óptima [0.95 - 0.995] y no es capacitivo, no conmutar
    if (!isCapacitive && fp >= 0.950 && fp <= 0.995) return;

    // Si hay sobrecompensación capacitiva (Q < -5 VAR), apagar pasos
    if (isCapacitive || qRea < -5.0) {
      for (let i = 2; i >= 0; i--) {
        if (state.stepState[i]) {
          logTerminal(`[AUTO] Sobrecompensación detectada (Q = ${qRea.toFixed(1)} VAR). Apagando Paso ${i + 1}.`);
          setStep(i, false);
          state.lastAutoActionTime = now;
          return;
        }
      }
    }

    // Si el FP es deficiente (< 0.95 inductivo), buscar la combinación binaria óptima (001 a 111)
    if (fp < 0.95) {
      let bestCombination = null;
      let minFpDistance = 999.0;

      for (let code = 1; code < 8; code++) {
        const candidate = [Boolean(code & 1), Boolean(code & 2), Boolean(code & 4)];

        // Omitir si requiere un paso en bloqueo de 30s
        let locked = false;
        for (let i = 0; i < 3; i++) {
          if (candidate[i] && !state.stepState[i] && isStepLocked(i)) {
            locked = true;
            break;
          }
        }
        if (locked) continue;

        // Evaluar resultado
        const pred = computeElectricalState(vRms, pAct, state.simMotor.q, candidate);
        if (pred.qRea < -15.0) continue; // Evitar sobrecompensar

        const distance = Math.abs(pred.fp - 0.974);
        if (distance < minFpDistance) {
          minFpDistance = distance;
          bestCombination = candidate;
        }
      }

      if (bestCombination) {
        let changed = false;
        for (let i = 0; i < 3; i++) {
          if (bestCombination[i] !== state.stepState[i]) {
            setStep(i, bestCombination[i]);
            changed = true;
          }
        }
        if (changed) {
          logTerminal(`[AUTO] Combinación óptima aplicada [${bestCombination.map(b => b ? 1 : 0).join(', ')}]. Meta FP = 0.974.`);
          addAlarm('INFORMATIVA', `Maniobra automática: Banco configurado en [${bestCombination.map(b => b ? 1 : 0).join(', ')}]`, 'Factor de potencia optimizado');
          state.lastAutoActionTime = now;
        }
      }
    }
  }

  // ==========================================================================
  // SISTEMA CENTRALIZADO DE ALARMAS INTELIGENTES
  // ==========================================================================

  function addAlarm(severity, message, suggestion = '') {
    const time = new Date().toTimeString().split(' ')[0];
    const alarm = {
      id: state.alarmCounterId++,
      severity, // 'CRITICA', 'ADVERTENCIA', 'INFORMATIVA'
      message,
      suggestion,
      timestamp: time,
      acknowledged: false
    };

    state.alarms.unshift(alarm);
    if (state.alarms.length > 50) state.alarms.pop();

    updateAlarmFeed();
  }

  function checkSupervisoryAlarms() {
    const t = state.telemetry;

    // Alarma de Sobrecorriente fuera de arranque
    if (t.iRms > 12.0 && state.inrushMultiplier <= 1.05) {
      if (!state.alarms.some(a => !a.acknowledged && a.message.includes('Sobrecorriente'))) {
        addAlarm('CRITICA', `Sobrecorriente detectada: ${t.iRms.toFixed(2)} A excede límite seguro.`, 'Verificar atascamiento mecánico del motor');
      }
    }

    // Alarma de Factor de Potencia Bajo
    if (!t.isMotorOff && t.fp < 0.950 && !t.isCapacitive) {
      if (!state.alarms.some(a => !a.acknowledged && a.message.includes('FP por debajo'))) {
        addAlarm('ADVERTENCIA', `FP deficiente: ${t.fp.toFixed(3)} bajo el límite CFE (0.950).`, 'Conectar pasos del banco 1:2:4 o habilitar modo AUTO');
      }
    }

    // Alarma de Sobrecompensación (Riesgo Resonancia)
    if (t.isCapacitive && t.qRea < -5.0) {
      if (!state.alarms.some(a => !a.acknowledged && a.message.includes('Sobrecompensación'))) {
        addAlarm('ADVERTENCIA', `Sobrecompensación capacitiva: Q = ${t.qRea.toFixed(1)} VAR (Adelanto).`, 'Desconectar pasos del banco para mitigar sobretensiones');
      }
    }
  }

  function updateAlarmFeed() {
    if (!dom.alarmFeedContainer) return;

    let critCount = 0;
    let warnCount = 0;

    let html = '';
    for (const a of state.alarms) {
      if (!a.acknowledged) {
        if (a.severity === 'CRITICA') critCount++;
        if (a.severity === 'ADVERTENCIA') warnCount++;
      }

      let colorBorder = 'border-slate-800';
      let badgeStyle = 'badge-neutral';
      if (a.severity === 'CRITICA') {
        colorBorder = 'border-rose-500/40 bg-rose-950/20';
        badgeStyle = 'badge-critico';
      } else if (a.severity === 'ADVERTENCIA') {
        colorBorder = 'border-amber-500/40 bg-amber-950/20';
        badgeStyle = 'badge-subcompensado';
      } else {
        colorBorder = 'border-cyan-500/40 bg-cyan-950/20';
        badgeStyle = 'badge-optimo';
      }

      html += `
        <div class="p-3 rounded-lg border ${colorBorder} space-y-1.5 transition">
          <div class="flex items-center justify-between text-[11px] font-mono">
            <span class="px-1.5 py-0.2 rounded font-bold ${badgeStyle}">${a.severity}</span>
            <span class="text-slate-500">${a.timestamp}</span>
          </div>
          <p class="text-xs text-slate-200 leading-snug">${a.message}</p>
          ${a.suggestion ? `<p class="text-[11px] text-cyan-400 font-mono">⚡ Acción: ${a.suggestion}</p>` : ''}
        </div>
      `;
    }

    if (state.alarms.length === 0) {
      html = '<div class="text-center py-8 text-xs text-slate-500 font-mono">Sin alarmas activas en el registro.</div>';
    }

    dom.alarmFeedContainer.innerHTML = html;

    // Badges en header y footer
    if (dom.alarmCountHeaderBadge) {
      dom.alarmCountHeaderBadge.textContent = String(critCount + warnCount);
      dom.alarmCountHeaderBadge.className = (critCount > 0)
        ? 'text-[10px] font-mono font-bold bg-rose-500/20 text-rose-400 border border-rose-500/40 px-1.5 py-0.2 rounded-full animate-pulse'
        : (warnCount > 0)
          ? 'text-[10px] font-mono font-bold bg-amber-500/20 text-amber-400 border border-amber-500/40 px-1.5 py-0.2 rounded-full'
          : 'text-[10px] font-mono font-bold bg-scada-800 text-slate-400 border border-scada-700 px-1.5 py-0.2 rounded-full';
    }

    if (dom.badgeCritCount) dom.badgeCritCount.textContent = `${critCount} críticas`;
    if (dom.badgeWarnCount) dom.badgeWarnCount.textContent = `${warnCount} advertencias`;
  }

  // ==========================================================================
  // TRANSITORIO DE ARRANQUE (PICO 10 - 15 A)
  // ==========================================================================

  function triggerInrushPeak() {
    if (state.simMotor.p <= 0.1 && state.simMotor.q <= 0.1) {
      logTerminal('[INRUSH] El motor está apagado. Selecciona un estado de marcha primero.');
      return;
    }

    // Pico realista de 11.5 A a 14.8 A
    const targetPeak = 11.5 + Math.random() * 3.3;
    const baseI = Math.max(0.5, (state.telemetry.sApp || 133.4) / (state.telemetry.vRms || 120.0));
    state.inrushMultiplier = targetPeak / baseI;

    if (targetPeak > state.peakCurrentRecorded) {
      state.peakCurrentRecorded = targetPeak;
    }

    logTerminal(`[INRUSH] Transitorio de arranque: Pico de corriente ${targetPeak.toFixed(2)} A (Sensado por ACS712-30A sin saturar).`);
    addAlarm('INFORMATIVA', `Pico transitorio de arranque: ${targetPeak.toFixed(2)} A registrado por el sensor ACS712-30A.`, 'Comportamiento inductivo normal de aceleración');

    if (state.inrushDecayTimer) clearInterval(state.inrushDecayTimer);
    const startTime = Date.now();
    const durationMs = 1800;

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

  function resetPeakCurrent() {
    state.peakCurrentRecorded = state.telemetry.iRms || 1.11;
    logTerminal(`[MAX-HOLD] Indicador de pico reiniciado a ${state.peakCurrentRecorded.toFixed(2)} A.`);
  }

  // ==========================================================================
  // SIMULADOR LOCAL
  // ==========================================================================
  function runSimulationStep() {
    if (state.mode !== 'sim') return;

    // Ruido suave de laboratorio
    const noiseV = (Math.random() - 0.5) * 0.4;
    const noiseP = (state.simMotor.p > 10) ? (Math.random() - 0.5) * 1.5 : 0;
    const noiseQ = (state.simMotor.q > 10) ? (Math.random() - 0.5) * 2.0 : 0;

    const vSim = Math.max(0, state.simMotor.v + noiseV);
    const pSim = Math.max(0, state.simMotor.p + noiseP);
    const qSim = Math.max(0, state.simMotor.q + noiseQ);

    state.telemetry = computeElectricalState(vSim, pSim, qSim, state.stepState);

    runAutoControlLoop();
    checkSupervisoryAlarms();
    updateUI();
    recordTelemetryHistory();
  }

  // ==========================================================================
  // WEB SERIAL API (ARDUINO NANO ATMEGA328P A 115200 BAUD)
  // ==========================================================================

  async function connectSerial() {
    if (!('serial' in navigator)) {
      alert('La Web Serial API no está soportada en este navegador.\nUsa Google Chrome, Microsoft Edge o Brave.');
      return;
    }

    try {
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

      dom.serialBtnLabel.textContent = 'Desconectar';
      dom.btnConnectSerial.classList.replace('bg-blue-600', 'bg-emerald-600');
      dom.modeBadge.classList.replace('text-emerald-400', 'text-cyan-400');
      dom.modeBadge.classList.replace('border-emerald-500/30', 'border-cyan-500/30');
      dom.modeBadge.classList.replace('bg-emerald-500/10', 'bg-cyan-500/10');
      dom.modeText.textContent = 'ARDUINO NANO';

      logTerminal('[SERIAL] Conectado a 115200 baudios.');
      logTerminal('[SERIAL] Enviando comando \'j\' para activar telemetría JSON...');
      addAlarm('INFORMATIVA', 'Arduino Nano conectado por USB. Adquisición por hardware activa.', 'Monitorear flujo JSON');

      const encoder = new TextEncoderStream();
      encoder.readable.pipeTo(port.writable);
      state.serial.writer = encoder.writable.getWriter();
      await state.serial.writer.write('j\n');

      readSerialLoop(port);

    } catch (err) {
      logTerminal(`[ERROR SERIAL] Fallo al conectar: ${err.message}`);
      await disconnectSerial();
    }
  }

  async function disconnectSerial() {
    state.serial.keepReading = false;
    try {
      if (state.serial.reader) { await state.serial.reader.cancel(); state.serial.reader = null; }
      if (state.serial.writer) { await state.serial.writer.close(); state.serial.writer = null; }
      if (state.serial.port) { await state.serial.port.close(); state.serial.port = null; }
    } catch (e) {
      console.warn('Cierre de puerto:', e);
    }

    state.mode = 'sim';
    dom.serialBtnLabel.textContent = 'Conectar USB';
    dom.btnConnectSerial.classList.replace('bg-emerald-600', 'bg-blue-600');
    dom.modeBadge.classList.replace('text-cyan-400', 'text-emerald-400');
    dom.modeBadge.classList.replace('border-cyan-500/30', 'border-emerald-500/30');
    dom.modeBadge.classList.replace('bg-cyan-500/10', 'bg-emerald-500/10');
    dom.modeText.textContent = 'SIMULACIÓN';
    logTerminal('[SERIAL] Desconectado. Regresando a modo simulación local.');
    addAlarm('INFORMATIVA', 'Arduino Nano desconectado. Sistema en modo simulación.', 'Reconectar cuando sea necesario');
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

            let lineEndIdx;
            while ((lineEndIdx = state.serial.buffer.indexOf('\n')) >= 0) {
              const line = state.serial.buffer.slice(0, lineEndIdx).trim();
              state.serial.buffer = state.serial.buffer.slice(lineEndIdx + 1);
              if (line.length > 0) handleSerialLine(line);
            }
          }
        }
      } catch (err) {
        if (state.serial.keepReading) logTerminal(`[ERROR LECTURA SERIAL] ${err.message}`);
        break;
      } finally {
        if (state.serial.reader) {
          state.serial.reader.releaseLock();
          state.serial.reader = null;
        }
      }
    }
  }

  function handleSerialLine(line) {
    if (line.startsWith('{') && line.endsWith('}')) {
      try {
        const data = JSON.parse(line);
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

        if (Array.isArray(data.leds) && data.leds.length === 3) {
          state.stepState = data.leds.map(Boolean);
        }

        if (Array.isArray(data.lock) && data.lock.length === 3) {
          for (let i = 0; i < 3; i++) {
            if (data.lock[i] > 0) {
              state.lastOffTime[i] = Date.now() - (LOCK_TIME_MS - (data.lock[i] * 1000));
            }
          }
        }

        let cTot = 0;
        if (state.stepState[0]) cTot += STEP_UF[0];
        if (state.stepState[1]) cTot += STEP_UF[1];
        if (state.stepState[2]) cTot += STEP_UF[2];
        state.telemetry.cTotalUF = cTot;
        state.telemetry.qInjected = OMEGA * (cTot * 1e-6) * Math.pow(state.telemetry.vRms, 2);

        const sTeo = Math.sqrt(Math.pow(state.telemetry.pAct, 2) + Math.pow(state.telemetry.qRea, 2));
        state.telemetry.fpTeorico = (sTeo > 0.1) ? (state.telemetry.pAct / sTeo) : 1.0;
        state.telemetry.errorPct = Math.abs(state.telemetry.fp - state.telemetry.fpTeorico) / (state.telemetry.fpTeorico || 1) * 100;

        const qMotorEst = state.telemetry.qRea + state.telemetry.qInjected;
        const sSinComp = Math.sqrt(Math.pow(state.telemetry.pAct, 2) + Math.pow(qMotorEst, 2));
        const iSinComp = (state.telemetry.vRms > 10) ? (sSinComp / state.telemetry.vRms) : 0;
        state.telemetry.iReductionPct = (iSinComp > 0.1) ? Math.max(0, ((iSinComp - state.telemetry.iRms) / iSinComp) * 100) : 0;

        updateUI();
        recordTelemetryHistory();
      } catch (e) {
        logTerminal(`[JSON ERROR] ${line}`);
      }
    } else {
      logTerminal(line);
    }
  }

  async function sendSerialCommand(cmd) {
    if (state.serial.writer) {
      try {
        await state.serial.writer.write(cmd + '\n');
        logTerminal(`[TX >> ARDUINO] '${cmd}'`);
      } catch (err) {
        logTerminal(`[TX ERROR] Fallo: ${err.message}`);
      }
    } else {
      logTerminal(`[TX SIM] '${cmd}'`);
    }
  }

  // ==========================================================================
  // ACTUALIZACIÓN DE INTERFAZ SCADA (DOM)
  // ==========================================================================
  function updateUI() {
    const t = state.telemetry;

    // 1. MEDIDOR PROTAGONISTA DE FACTOR DE POTENCIA
    dom.valFP.textContent = t.isMotorOff ? '0.000' : t.fp.toFixed(3);
    dom.valPhiDeg.textContent = t.isMotorOff ? '0.0' : t.phiDeg.toFixed(1);
    dom.valDeltaT.textContent = t.isMotorOff ? '0.00 ms' : `${t.deltaTMs.toFixed(2)} ms`;

    if (t.isMotorOff) {
      dom.fpStatusBadge.textContent = 'MOTOR APAGADO';
      dom.fpStatusBadge.className = 'px-2.5 py-0.5 rounded text-[11px] font-mono font-bold badge-neutral';
      dom.valFpRegime.textContent = 'Sin Carga Activa';
      dom.valFpRegime.className = 'text-slate-400 font-bold';
      dom.fpConditionSummary.textContent = 'En Reposo Seguro';
      dom.fpConditionSummary.className = 'text-slate-400 font-medium';
    } else if (t.isCapacitive) {
      dom.fpStatusBadge.textContent = 'SOBRECOMPENSADO · ADELANTO';
      dom.fpStatusBadge.className = 'px-2.5 py-0.5 rounded text-[11px] font-mono font-bold badge-sobrecompensado';
      dom.valFpRegime.textContent = 'ADELANTO (Capacitivo)';
      dom.valFpRegime.className = 'text-purple-400 font-bold';
      dom.fpConditionSummary.textContent = 'Riesgo de Resonancia';
      dom.fpConditionSummary.className = 'text-purple-400 font-medium';
    } else if (t.fp >= 0.950 && t.fp <= 0.990) {
      dom.fpStatusBadge.textContent = 'ÓPTIMO EN RANGO';
      dom.fpStatusBadge.className = 'px-2.5 py-0.5 rounded text-[11px] font-mono font-bold badge-optimo';
      dom.valFpRegime.textContent = 'ATRASO (Inductivo)';
      dom.valFpRegime.className = 'text-emerald-400 font-bold';
      dom.fpConditionSummary.textContent = 'Cumple Norma CFE (0.95–0.99)';
      dom.fpConditionSummary.className = 'text-emerald-400 font-medium';
    } else if (t.fp > 0.990) {
      dom.fpStatusBadge.textContent = 'UNITARIO / AL LÍMITE';
      dom.fpStatusBadge.className = 'px-2.5 py-0.5 rounded text-[11px] font-mono font-bold badge-optimo';
      dom.valFpRegime.textContent = 'ATRASO Mínimo';
      dom.valFpRegime.className = 'text-emerald-300 font-bold';
      dom.fpConditionSummary.textContent = 'Excelente Compensación';
      dom.fpConditionSummary.className = 'text-emerald-400 font-medium';
    } else {
      dom.fpStatusBadge.textContent = 'SUBCOMPENSADO · ATRASO';
      dom.fpStatusBadge.className = 'px-2.5 py-0.5 rounded text-[11px] font-mono font-bold badge-critico';
      dom.valFpRegime.textContent = 'ATRASO (Inductivo Puro)';
      dom.valFpRegime.className = 'text-amber-400 font-bold';
      dom.fpConditionSummary.textContent = (t.fp <= 0.25) ? 'Vacío sin Compensar (FP ≈ 0.21)' : 'Bajo Límite CFE (< 0.95)';
      dom.fpConditionSummary.className = 'text-amber-400 font-medium';
    }

    renderFpGauge(t.isMotorOff ? 0 : t.fp, t.isCapacitive);

    // 2. PANEL VISUAL DEL MOTOR WEG W01
    if (t.isMotorOff) {
      dom.cardMotorWeg.classList.remove('motor-running-active');
      dom.motorStatusDot.className = 'w-2 h-2 rounded-full bg-slate-500';
      dom.badgeMotorState.textContent = 'APAGADO';
      dom.badgeMotorState.className = 'px-2 py-0.5 rounded text-[10px] font-mono font-bold bg-slate-800 text-slate-400 border border-slate-700';
      dom.motorRunningIndicator.classList.add('hidden');
      dom.motorLoadText.textContent = 'En Reposo';
      dom.motorPText.textContent = '0 W';
    } else {
      dom.cardMotorWeg.classList.add('motor-running-active');
      dom.motorStatusDot.className = 'w-2 h-2 rounded-full bg-emerald-400 animate-pulse';
      dom.badgeMotorState.textContent = 'ENCENDIDO';
      dom.badgeMotorState.className = 'px-2 py-0.5 rounded text-[10px] font-mono font-bold bg-emerald-500/15 text-emerald-400 border border-emerald-500/30';
      dom.motorRunningIndicator.classList.remove('hidden');

      if (state.motorPresetKey === 'empty') dom.motorLoadText.textContent = 'En Vacío (Lab)';
      else if (state.motorPresetKey === 'mid') dom.motorLoadText.textContent = 'Carga Media';
      else dom.motorLoadText.textContent = 'Plena Carga';

      dom.motorPText.textContent = `${t.pAct.toFixed(0)} W`;
    }

    // 3. BANCO DE CAPACITORES 1:2:4
    dom.badgeCapTotal.textContent = `${t.cTotalUF.toFixed(0)} µF`;

    // Step 1
    dom.ledCore1.className = state.stepState[0] ? 'scada-led-core led-red-on' : 'scada-led-core led-red-off';
    dom.ledStateText1.textContent = state.stepState[0] ? 'ON' : 'OFF';
    dom.ledStateText1.className = state.stepState[0] ? 'text-[11px] font-mono font-bold text-emerald-400' : 'text-[11px] font-mono font-bold text-slate-500';
    dom.btnStepToggle1.disabled = state.isAutoControl || isStepLocked(0);

    // Step 2
    dom.ledCore2.className = state.stepState[1] ? 'scada-led-core led-blue-on' : 'scada-led-core led-blue-off';
    dom.ledStateText2.textContent = state.stepState[1] ? 'ON' : 'OFF';
    dom.ledStateText2.className = state.stepState[1] ? 'text-[11px] font-mono font-bold text-emerald-400' : 'text-[11px] font-mono font-bold text-slate-500';
    dom.btnStepToggle2.disabled = state.isAutoControl || isStepLocked(1);

    // Step 3
    dom.ledCore3.className = state.stepState[2] ? 'scada-led-core led-green-on' : 'scada-led-core led-green-off';
    dom.ledStateText3.textContent = state.stepState[2] ? 'ON' : 'OFF';
    dom.ledStateText3.className = state.stepState[2] ? 'text-[11px] font-mono font-bold text-emerald-400' : 'text-[11px] font-mono font-bold text-slate-500';
    dom.btnStepToggle3.disabled = state.isAutoControl || isStepLocked(2);

    // Bloqueo de Descarga RC
    let maxLockRem = Math.max(getLockRemainingSec(0), getLockRemainingSec(1), getLockRemainingSec(2));
    if (maxLockRem > 0) {
      dom.lockStatusSummary.textContent = `DESCARGANDO ${maxLockRem.toFixed(1)} s`;
      dom.lockStatusSummary.className = 'text-amber-400 font-semibold';
      const pct = (maxLockRem / (LOCK_TIME_MS / 1000.0)) * 100;
      dom.lockBarFill.style.width = `${pct}%`;
    } else {
      dom.lockStatusSummary.textContent = 'DISPONIBLE (0.0 V)';
      dom.lockStatusSummary.className = 'text-emerald-400 font-semibold';
      dom.lockBarFill.style.width = '0%';
    }

    // 4. 4 TARJETAS KPI
    dom.valVrms.textContent = t.vRms.toFixed(1);
    dom.valIrms.textContent = t.iRms.toFixed(2);
    dom.valIPeak.textContent = `${state.peakCurrentRecorded.toFixed(1)} A`;
    dom.valIReduction.textContent = `-${t.iReductionPct.toFixed(1)}%`;
    dom.valIReduction.className = (t.iReductionPct > 5)
      ? 'text-[11px] font-mono px-1.5 py-0.5 rounded bg-emerald-500/15 text-emerald-400 border border-emerald-500/30'
      : 'text-[11px] font-mono px-1.5 py-0.5 rounded bg-scada-800 text-slate-400 border border-scada-700';

    dom.valP.textContent = t.pAct.toFixed(1);
    if (state.motorPresetKey === 'empty') dom.valPLoadPct.textContent = 'En Vacío (Lab)';
    else if (state.motorPresetKey === 'mid') dom.valPLoadPct.textContent = '59% Carga';
    else if (state.motorPresetKey === 'full') dom.valPLoadPct.textContent = '100% Carga';
    else dom.valPLoadPct.textContent = 'Reposo 0%';

    dom.valQ.textContent = Math.abs(t.qRea).toFixed(1);
    const qMotorBase = (state.mode === 'sim') ? state.simMotor.q : (t.qRea + t.qInjected);
    dom.valQMotorText.textContent = qMotorBase.toFixed(0);
    dom.valQInjectedText.textContent = `-${t.qInjected.toFixed(0)}`;

    if (t.isCapacitive) {
      dom.badgeQState.textContent = 'ADELANTO';
      dom.badgeQState.className = 'text-[10px] font-mono px-1.5 py-0.2 rounded bg-purple-500/15 text-purple-400 border border-purple-500/30';
    } else {
      dom.badgeQState.textContent = 'ATRASO';
      dom.badgeQState.className = 'text-[10px] font-mono px-1.5 py-0.2 rounded bg-amber-500/15 text-amber-400 border border-amber-500/30';
    }

    // 5. MEMORIA DE CÁLCULO (PESTAÑA INGENIERÍA)
    updateMathMemo();

    // 6. TABLA DE VERDAD 1:2:4
    const stepDec = (state.stepState[0] ? 1 : 0) + (state.stepState[1] ? 2 : 0) + (state.stepState[2] ? 4 : 0);
    updateTruthTable(stepDec);
  }

  // ==========================================================================
  // INSTRUMENTO PRINCIPAL: GAUGE SEMICIRCULAR DE FACTOR DE POTENCIA
  // ==========================================================================
  function renderFpGauge(fpVal, isCapacitive) {
    const canvas = dom.fpGaugeCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    ctx.clearRect(0, 0, width, height);

    const cx = width / 2;
    const cy = height - 12;
    const radius = Math.min(cx, cy) - 14;
    const lineWidth = 10;

    // 1. Arco de Fondo (Track)
    ctx.beginPath();
    ctx.arc(cx, cy, radius, Math.PI, 0, false);
    ctx.strokeStyle = '#18181c';
    ctx.lineWidth = lineWidth;
    ctx.lineCap = 'round';
    ctx.stroke();

    // 2. Zonas de Color Semicirculares (Escala de 0.00 a 1.00)
    // Zona Roja (< 0.85): 0 a 85%
    const arcStart = Math.PI;
    const arcEnd = 0;
    const totalAngle = Math.PI;

    function getAngle(val) {
      const clamped = Math.max(0.0, Math.min(1.0, val));
      return Math.PI + clamped * totalAngle;
    }

    // Zona Roja (< 0.85)
    ctx.beginPath();
    ctx.arc(cx, cy, radius, getAngle(0.0), getAngle(0.85), false);
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = lineWidth;
    ctx.lineCap = 'butt';
    ctx.stroke();

    // Zona Amarilla (0.85 - 0.95)
    ctx.beginPath();
    ctx.arc(cx, cy, radius, getAngle(0.85), getAngle(0.95), false);
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = lineWidth;
    ctx.stroke();

    // Zona Verde Óptima [0.95 - 0.99] (Con brillo)
    ctx.beginPath();
    ctx.arc(cx, cy, radius, getAngle(0.95), getAngle(0.99), false);
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = lineWidth;
    ctx.stroke();

    // Zona Púrpura (> 0.99 o Capacitiva)
    ctx.beginPath();
    ctx.arc(cx, cy, radius, getAngle(0.99), getAngle(1.00), false);
    ctx.strokeStyle = isCapacitive ? '#c084fc' : '#22d3ee';
    ctx.lineWidth = lineWidth;
    ctx.stroke();

    // 3. Ticks y Etiquetas Clave (0.2, 0.5, 0.85, 0.95, 1.0)
    const tickValues = [0.2, 0.5, 0.85, 0.95, 1.0];
    ctx.fillStyle = '#71717a';
    ctx.font = '8.5px JetBrains Mono';
    ctx.textAlign = 'center';

    for (const tv of tickValues) {
      const a = getAngle(tv);
      const txInner = cx + (radius - 12) * Math.cos(a);
      const tyInner = cy + (radius - 12) * Math.sin(a);
      const txOuter = cx + (radius - 4) * Math.cos(a);
      const tyOuter = cy + (radius - 4) * Math.sin(a);

      ctx.beginPath();
      ctx.moveTo(txInner, tyInner);
      ctx.lineTo(txOuter, tyOuter);
      ctx.strokeStyle = '#3f3f46';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      const labelX = cx + (radius - 22) * Math.cos(a);
      const labelY = cy + (radius - 22) * Math.sin(a) + 3;
      ctx.fillText(tv.toFixed(2), labelX, labelY);
    }

    // 4. Aguja Indicadora (Needle)
    const needleAngle = getAngle(fpVal);
    const needleLength = radius - 8;
    const nx = cx + needleLength * Math.cos(needleAngle);
    const ny = cy + needleLength * Math.sin(needleAngle);

    // Color de la aguja según estado
    const needleColor = isCapacitive
      ? '#c084fc'
      : (fpVal >= 0.95 && fpVal <= 0.99)
        ? '#22c55e'
        : (fpVal >= 0.85)
          ? '#fbbf24'
          : '#ef4444';

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(nx, ny);
    ctx.strokeStyle = needleColor;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Pivote central
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fillStyle = needleColor;
    ctx.fill();
    ctx.strokeStyle = '#09090b';
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // ==========================================================================
  // MEMORIA DE CÁLCULO EN VIVO & TABLA DE VERDAD
  // ==========================================================================

  function updateMathMemo() {
    const t = state.telemetry;
    dom.mathMemoVrms.textContent = `= √( (1/320) · ${(t.vRms * t.vRms * 320).toFixed(0)} ) = ${t.vRms.toFixed(2)} V`;
    dom.mathMemoIrms.textContent = `= √( (1/320) · ${(t.iRms * t.iRms * 320).toFixed(1)} ) = ${t.iRms.toFixed(2)} A`;
    dom.mathMemoP.textContent = `${t.pAct.toFixed(1)} W`;
    dom.mathMemoS.textContent = `${t.sApp.toFixed(1)} VA`;
    dom.mathMemoFP.textContent = `${t.fp.toFixed(3)}`;

    dom.mathMemoQ.textContent = `${t.qRea >= 0 ? '+' : ''}${t.qRea.toFixed(1)} VAR (${t.isCapacitive ? 'ADELANTO' : 'ATRASO'})`;
    dom.mathMemoPhi.textContent = `${t.phiDeg.toFixed(1)}°`;
    dom.mathMemoDt.textContent = `${t.deltaTMs.toFixed(2)} ms`;

    dom.mathMemoQcReq.textContent = `Q_C = ${t.qcReq.toFixed(1)} VAR (Meta FP=0.97)`;
    dom.mathMemoCReq.textContent = `${t.cReqUF.toFixed(1)} µF`;
    dom.mathMemoFPTeo.textContent = `${t.fpTeorico.toFixed(3)}`;
    dom.mathMemoErrorBadge.textContent = `Error: ${t.errorPct.toFixed(2)}%`;
  }

  function updateTruthTable(activeStepDec) {
    if (!dom.truthTableBody) return;

    let html = '';
    const v = state.telemetry.vRms || 120.0;
    const p = state.telemetry.pAct || 130.0;
    const qMotor = (state.mode === 'sim') ? state.simMotor.q : (state.telemetry.qRea + state.telemetry.qInjected);

    for (let i = 0; i < 8; i++) {
      const b0 = (i & 1) ? 1 : 0;
      const b1 = (i & 2) ? 1 : 0;
      const b2 = (i & 4) ? 1 : 0;

      const cTot = (b0 * 15.0) + (b1 * 30.0) + (b2 * 60.0);
      const qInj = OMEGA * (cTot * 1e-6) * (v * v);
      const qRes = qMotor - qInj;
      const sRes = Math.sqrt(p * p + qRes * qRes);
      let fpEst = (sRes > 0.1) ? (p / sRes) : 1.0;
      if (fpEst > 1.0) fpEst = 1.0;

      const isActive = (i === activeStepDec);
      const rowBg = isActive ? 'bg-cyan-950/30 font-bold border-l-2 border-l-cyan-400' : 'hover:bg-scada-900/60';
      const fpColor = (qRes < -5.0) ? 'text-purple-400' : (fpEst >= 0.950 ? 'text-emerald-400 font-bold' : 'text-slate-400');

      html += `
        <tr class="${rowBg} transition">
          <td class="py-2 px-3 flex items-center gap-2">
            <span class="w-1.5 h-1.5 rounded-full ${isActive ? 'bg-cyan-400 animate-ping' : 'bg-slate-700'}"></span>
            <span>Paso ${i} [${b0}, ${b1}, ${b2}]</span>
          </td>
          <td class="py-2 px-3 text-red-400">${b0 ? '15 µF' : '—'}</td>
          <td class="py-2 px-3 text-blue-400">${b1 ? '30 µF' : '—'}</td>
          <td class="py-2 px-3 text-emerald-400">${b2 ? '60 µF' : '—'}</td>
          <td class="py-2 px-3 font-semibold text-white">${cTot.toFixed(0)} µF</td>
          <td class="py-2 px-3 text-cyan-400">${qInj.toFixed(1)} VAR</td>
          <td class="py-2 px-3 ${fpColor}">${fpEst.toFixed(3)} ${fpEst >= 0.95 ? '★' : ''}</td>
          <td class="py-2 px-3 text-center">
            ${isActive ? '<span class="px-2 py-0.5 rounded text-[10px] bg-emerald-500/20 text-emerald-400 border border-emerald-500/40">ACTIVA</span>' : '<span class="text-slate-600">—</span>'}
          </td>
          <td class="py-2 px-3 text-right">
            <button onclick="window.selectCombination(${i})" class="px-2 py-0.5 rounded bg-scada-850 hover:bg-scada-800 text-slate-300 border border-scada-700 text-[10px] transition disabled:opacity-30 disabled:cursor-not-allowed" ${state.isAutoControl ? 'disabled' : ''}>
              Activar
            </button>
          </td>
        </tr>
      `;
    }
    dom.truthTableBody.innerHTML = html;
  }

  window.selectCombination = function(code) {
    if (state.isAutoControl) {
      alert('Desactiva el Control Automático primero para seleccionar pasos manualmente.');
      return;
    }
    setStep(0, Boolean(code & 1));
    setStep(1, Boolean(code & 2));
    setStep(2, Boolean(code & 4));
  };

  // ==========================================================================
  // OSCILOSCOPIO DIGITAL DE 2 CANALES (CANVAS 60 FPS)
  // ==========================================================================
  function setupOscilloscope() {
    const canvas = dom.scopeCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);
    }
    window.addEventListener('resize', resize);
    resize();

    let lastTime = performance.now();

    function render(now) {
      requestAnimationFrame(render);
      const dt = (now - lastTime) / 1000.0;
      lastTime = now;

      if (!state.scope.paused) {
        state.scope.timeSec += dt;
      }

      const rect = canvas.getBoundingClientRect();
      const w = rect.width;
      const h = rect.height;

      ctx.fillStyle = '#05070a';
      ctx.fillRect(0, 0, w, h);

      // Cuadrícula Tektronix
      const divX = 10;
      const divY = 8;
      const stepX = w / divX;
      const stepY = h / divY;

      ctx.strokeStyle = '#121824';
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let x = 0; x <= w; x += stepX) { ctx.moveTo(x, 0); ctx.lineTo(x, h); }
      for (let y = 0; y <= h; y += stepY) { ctx.moveTo(0, y); ctx.lineTo(w, y); }
      ctx.stroke();

      const midX = w / 2;
      const midY = h / 2;
      ctx.strokeStyle = '#1f2b3f';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, midY); ctx.lineTo(w, midY);
      ctx.moveTo(midX, 0); ctx.lineTo(midX, h);
      ctx.stroke();

      const { vRms, iRms, phiDeg, deltaTMs, isMotorOff, isCapacitive } = state.telemetry;
      const vPeak = isMotorOff ? 0 : Math.SQRT2 * vRms;
      const iPeak = isMotorOff ? 0 : Math.SQRT2 * iRms;
      const phiRad = isMotorOff ? 0 : phiDeg * (Math.PI / 180.0) * (isCapacitive ? -1 : 1);

      // Escalas de visualización
      const vPixelsPerVolt = (h / 2) / 200.0;
      // Auto-escalamiento inteligente para que picos de inrush (10-15A RMS -> hasta 22A pico) nunca se corten
      const maxExpectedIPeak = Math.max(10.0, iPeak * 1.18);
      const iPixelsPerAmp = ((h / 2) * 0.82) / maxExpectedIPeak;

      const totalTimeSpanMs = divX * state.scope.timebaseMsDiv;
      const msPerPixel = totalTimeSpanMs / w;
      const timeOffset = state.scope.timeSec;

      // Canal 1: Tensión v(t) [Amarillo #FACC15]
      ctx.strokeStyle = '#facc15';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      for (let px = 0; px < w; px++) {
        const tSec = (px * msPerPixel / 1000.0) + (timeOffset * 0.05);
        const vInst = vPeak * Math.sin(OMEGA * tSec);
        const py = midY - (vInst * vPixelsPerVolt);
        if (px === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      // Canal 2: Corriente i(t) [Cian #22D3EE]
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      for (let px = 0; px < w; px++) {
        const tSec = (px * msPerPixel / 1000.0) + (timeOffset * 0.05);
        const iInst = iPeak * Math.sin(OMEGA * tSec - phiRad);
        const py = midY - (iInst * iPixelsPerAmp);
        if (px === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      // HUD Metrics
      dom.hudVpp.textContent = `${(vPeak * 2).toFixed(1)} Vp-p`;
      const isSurge = (iRms >= 9.5);
      dom.hudIpp.textContent = `${(iPeak * 2).toFixed(1)} Ap-p (${iRms.toFixed(2)} Arms) ${isSurge ? '⚡ [INRUSH]' : ''}`;
      dom.hudDt.textContent = `${deltaTMs.toFixed(2)} ms`;
      dom.hudPhi.textContent = `${phiDeg.toFixed(1)}° ${isCapacitive ? '(Adelanto)' : '(Atraso)'}`;
    }

    requestAnimationFrame(render);
  }

  // ==========================================================================
  // TRIÁNGULO DE POTENCIAS (PLANO COMPLEJO FASORIAL)
  // ==========================================================================
  function setupPowerTriangle() {
    const canvas = dom.triangleCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);
    }
    window.addEventListener('resize', resize);
    resize();

    function render() {
      requestAnimationFrame(render);

      const rect = canvas.getBoundingClientRect();
      const w = rect.width;
      const h = rect.height;

      ctx.fillStyle = '#05070a';
      ctx.fillRect(0, 0, w, h);

      const originX = w * 0.16;
      const originY = h * 0.65;

      const pVal = state.telemetry.pAct || 130.0;
      const qVal = state.telemetry.qRea || 30.0;
      const qMotor = (state.mode === 'sim') ? state.simMotor.q : (state.telemetry.qRea + state.telemetry.qInjected);
      const qInjected = state.telemetry.qInjected || 0.0;

      const maxVal = 1000.0;
      const scaleX = (w * 0.68) / maxVal;
      const scaleY = (h * 0.52) / maxVal;

      const pPx = pVal * scaleX;
      const qMotorPx = qMotor * scaleY;
      const qNetPx = qVal * scaleY;
      const qInjPx = qInjected * scaleY;

      // Ejes P y Q
      ctx.strokeStyle = '#1a2230';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(originX - 15, originY); ctx.lineTo(w - 20, originY);
      ctx.moveTo(originX, 15); ctx.lineTo(originX, h - 15);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.font = '9px JetBrains Mono';
      ctx.fillText('+P (W)', w - 45, originY - 6);
      ctx.fillText('+Q (Ind)', originX + 6, 25);
      ctx.fillText('-Q (Cap)', originX + 6, h - 18);

      if (state.telemetry.isMotorOff) {
        ctx.fillStyle = '#71717a';
        ctx.font = '11px JetBrains Mono';
        ctx.textAlign = 'center';
        ctx.fillText('MOTOR APAGADO — VECTORES EN REPOSO', w / 2, h / 2);
        return;
      }

      // 1. Vector Potencia Activa P (Verde esmeralda horizontal)
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + pPx, originY);
      ctx.stroke();

      ctx.fillStyle = '#4ade80';
      ctx.fillText(`P = ${pVal.toFixed(0)} W`, originX + (pPx / 2) - 20, originY + 14);

      // 2. Vector Q_motor original (Línea punteada naranja)
      ctx.strokeStyle = '#f59e0b';
      ctx.setLineDash([3, 3]);
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(originX + pPx, originY);
      ctx.lineTo(originX + pPx, originY - qMotorPx);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillStyle = '#fbbf24';
      ctx.fillText(`Q_mot = ${qMotor.toFixed(0)} VAR`, originX + pPx + 6, originY - (qMotorPx / 2));

      // 3. Vector Q_C inyectado por capacitores (Flecha azul hacia abajo)
      if (qInjPx > 4) {
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(originX + pPx + 10, originY - qMotorPx);
        ctx.lineTo(originX + pPx + 10, originY - qMotorPx + qInjPx);
        ctx.stroke();

        ctx.fillStyle = '#38bdf8';
        ctx.fillText(`-Q_C = ${qInjected.toFixed(0)} VAR`, originX + pPx + 15, originY - qMotorPx + (qInjPx / 2));
      }

      // 4. Vector Q_neta resultante
      ctx.strokeStyle = state.telemetry.isCapacitive ? '#c084fc' : '#eab308';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(originX + pPx, originY);
      ctx.lineTo(originX + pPx, originY - qNetPx);
      ctx.stroke();

      // 5. Fasor S_1 sin compensar (Gris tenue punteado)
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
      ctx.setLineDash([2, 2]);
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + pPx, originY - qMotorPx);
      ctx.stroke();
      ctx.setLineDash([]);

      // 6. Fasor S_2 compensado actual (Cian sólido brillante)
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + pPx, originY - qNetPx);
      ctx.stroke();

      // Arco del ángulo phi resultante
      const phiRad = state.telemetry.phiDeg * (Math.PI / 180.0);
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(originX, originY, 28, 0, -phiRad, true);
      ctx.stroke();

      ctx.fillStyle = '#22d3ee';
      ctx.fillText(`φ = ${state.telemetry.phiDeg.toFixed(1)}°`, originX + 34, originY - 6);

      // Textos inferiores
      const s1 = Math.sqrt(pVal * pVal + qMotor * qMotor);
      dom.triS1.textContent = `${s1.toFixed(0)} VA (FP ${(pVal / s1).toFixed(3)})`;
      dom.triS2.textContent = `${state.telemetry.sApp.toFixed(0)} VA (FP ${state.telemetry.fp.toFixed(3)})`;
    }

    requestAnimationFrame(render);
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
      source: (state.mode === 'serial') ? 'ARDUINO NANO' : 'SIMULADO',
      vRms: t.vRms,
      iRms: t.iRms,
      pAct: t.pAct,
      qRea: t.qRea,
      sApp: t.sApp,
      fp: t.fp,
      cTotalUF: t.cTotalUF,
      steps: [state.stepState[0] ? 1 : 0, state.stepState[1] ? 1 : 0, state.stepState[2] ? 1 : 0]
    };

    state.telemetryHistory.unshift(entry);
    if (state.telemetryHistory.length > state.maxHistorySize) state.telemetryHistory.pop();

    if (dom.recordCountBadge) dom.recordCountBadge.textContent = `${state.telemetryHistory.length} registros`;

    // Renderizar últimas 10 filas
    if (dom.telemetryTableBody) {
      let html = '';
      for (const s of state.telemetryHistory.slice(0, 10)) {
        html += `
          <tr class="hover:bg-scada-900/60">
            <td class="py-1 px-2 text-slate-400">${s.timestamp}</td>
            <td class="py-1 px-2 text-cyan-400 font-semibold">${s.source}</td>
            <td class="py-1 px-2 text-yellow-400">${s.vRms.toFixed(1)}</td>
            <td class="py-1 px-2 text-cyan-300">${s.iRms.toFixed(2)}</td>
            <td class="py-1 px-2 text-white">${s.pAct.toFixed(1)}</td>
            <td class="py-1 px-2 text-amber-400">${s.qRea.toFixed(1)}</td>
            <td class="py-1 px-2 ${s.fp >= 0.95 ? 'text-emerald-400 font-bold' : 'text-slate-300'}">${s.fp.toFixed(3)}</td>
            <td class="py-1 px-2 text-cyan-400">${s.cTotalUF.toFixed(0)}</td>
            <td class="py-1 px-2 text-slate-400 font-mono">[${s.steps.join(',')}]</td>
          </tr>
        `;
      }
      dom.telemetryTableBody.innerHTML = html;
    }
  }

  function exportCSV() {
    if (state.telemetryHistory.length === 0) {
      alert('No hay registros en el buffer para exportar.');
      return;
    }

    const headers = [
      'iso_timestamp',
      'origen_dato',
      'v_rms_v',
      'i_rms_a',
      'potencia_activa_p_w',
      'potencia_reactiva_q_var',
      'potencia_aparente_s_va',
      'factor_de_potencia_fp',
      'capacitancia_total_uf',
      'step1_d4_15uf',
      'step2_d5_30uf',
      'step3_d6_60uf'
    ];

    let csv = headers.join(',') + '\n';
    const reversed = [...state.telemetryHistory].reverse();
    for (const r of reversed) {
      csv += [
        r.isoDate,
        r.source,
        r.vRms.toFixed(2),
        r.iRms.toFixed(3),
        r.pAct.toFixed(2),
        r.qRea.toFixed(2),
        r.sApp.toFixed(2),
        r.fp.toFixed(4),
        r.cTotalUF.toFixed(1),
        r.steps[0],
        r.steps[1],
        r.steps[2]
      ].join(',') + '\n';
    }

    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    const timestampStr = new Date().toISOString().replace(/[:.]/g, '-');
    link.setAttribute('href', url);
    link.setAttribute('download', `POWERGuard5_SCADA_${timestampStr}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);

    logTerminal(`[CSV] Archivo descargado con ${reversed.length} lecturas.`);
    addAlarm('INFORMATIVA', `Archivo CSV exportado con ${reversed.length} muestras de telemetría.`, 'Guardado para informe técnico');
  }

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
    // 1. Navegación por pestañas
    function switchTab(targetId) {
      [dom.tabMonitor, dom.tabEngineering, dom.tabHistory].forEach(t => {
        if (t) t.classList.remove('active');
      });
      [dom.tabBtnMonitor, dom.tabBtnEngineering, dom.tabBtnHistory].forEach(b => {
        if (b) {
          b.classList.remove('bg-scada-800', 'border-scada-600', 'text-white', 'shadow-sm');
          b.classList.add('text-slate-400');
        }
      });

      const activeTab = document.getElementById(targetId);
      if (activeTab) activeTab.classList.add('active');

      if (targetId === 'tabMonitor') {
        dom.tabBtnMonitor.classList.add('bg-scada-800', 'border-scada-600', 'text-white', 'shadow-sm');
      } else if (targetId === 'tabEngineering') {
        dom.tabBtnEngineering.classList.add('bg-scada-800', 'border-scada-600', 'text-white', 'shadow-sm');
      } else if (targetId === 'tabHistory') {
        dom.tabBtnHistory.classList.add('bg-scada-800', 'border-scada-600', 'text-white', 'shadow-sm');
      }
    }

    if (dom.tabBtnMonitor) dom.tabBtnMonitor.addEventListener('click', () => switchTab('tabMonitor'));
    if (dom.tabBtnEngineering) dom.tabBtnEngineering.addEventListener('click', () => switchTab('tabEngineering'));
    if (dom.tabBtnHistory) dom.tabBtnHistory.addEventListener('click', () => switchTab('tabHistory'));

    // 2. Control Automático / Manual
    if (dom.toggleAutoBtn) {
      dom.toggleAutoBtn.addEventListener('click', () => {
        state.isAutoControl = !state.isAutoControl;
        if (state.isAutoControl) {
          dom.autoIndicatorDot.className = 'w-2 h-2 rounded-full bg-cyan-400 animate-pulse';
          dom.autoBtnText.textContent = 'AUTO';
          logTerminal('[CONTROL] Modo AUTOMÁTICO activo (Banda 0.950 – 0.990).');
          addAlarm('INFORMATIVA', 'Modo de control automático activado.', 'El lazo seleccionará los pasos automáticamente');
        } else {
          dom.autoIndicatorDot.className = 'w-2 h-2 rounded-full bg-amber-400';
          dom.autoBtnText.textContent = 'MANUAL';
          logTerminal('[CONTROL] Modo MANUAL habilitado (Pulsadores activos).');
          addAlarm('ADVERTENCIA', 'Control manual habilitado.', 'El usuario controla la conmutación de pasos');
        }
        updateUI();
      });
    }

    // 3. Conexión Web Serial
    if (dom.btnConnectSerial) dom.btnConnectSerial.addEventListener('click', connectSerial);

    // 4. Modo Presentación (Zen Mode)
    if (dom.btnTogglePresentation) {
      dom.btnTogglePresentation.addEventListener('click', () => {
        state.presentationMode = !state.presentationMode;
        if (state.presentationMode) {
          document.body.classList.add('presentation-mode');
          logTerminal('[HMI] Modo Presentación Activado.');
        } else {
          document.body.classList.remove('presentation-mode');
          logTerminal('[HMI] Modo Presentación Desactivado.');
        }
      });
    }

    // 5. Presets del Motor WEG
    function setMotorPreset(key) {
      const prevKey = state.motorPresetKey;
      state.motorPresetKey = key;
      state.simMotor = { ...MOTOR_PRESETS[key] };

      [dom.btnLoadEmpty, dom.btnLoadMid, dom.btnLoadFull, dom.btnLoadOff].forEach(b => {
        if (!b) return;
        b.classList.remove('bg-emerald-950/60', 'text-emerald-300', 'border-emerald-500/40', 'bg-scada-800', 'text-white', 'bg-rose-950/40', 'text-rose-300');
        b.classList.add('text-slate-400');
      });

      if (key === 'empty') {
        dom.btnLoadEmpty.classList.add('bg-emerald-950/60', 'text-emerald-300', 'border-emerald-500/40');
      } else if (key === 'mid') {
        dom.btnLoadMid.classList.add('bg-scada-800', 'text-white');
      } else if (key === 'full') {
        dom.btnLoadFull.classList.add('bg-scada-800', 'text-white');
      } else if (key === 'off') {
        dom.btnLoadOff.classList.add('bg-rose-950/40', 'text-rose-300');
      }

      logTerminal(`[MOTOR] Seleccionado: ${MOTOR_PRESETS[key].label}`);
      addAlarm('INFORMATIVA', `Estado de motor modificado: ${MOTOR_PRESETS[key].label}`, 'Supervisar reactiva');

      // Si se enciende desde OFF, disparar inrush
      if (prevKey === 'off' && key !== 'off') triggerInrushPeak();
    }

    if (dom.btnLoadEmpty) dom.btnLoadEmpty.addEventListener('click', () => setMotorPreset('empty'));
    if (dom.btnLoadMid) dom.btnLoadMid.addEventListener('click', () => setMotorPreset('mid'));
    if (dom.btnLoadFull) dom.btnLoadFull.addEventListener('click', () => setMotorPreset('full'));
    if (dom.btnLoadOff) dom.btnLoadOff.addEventListener('click', () => setMotorPreset('off'));

    // Botón Inrush
    if (dom.btnSimInrush) dom.btnSimInrush.addEventListener('click', triggerInrushPeak);

    // Reset Peak
    if (dom.btnResetPeak) dom.btnResetPeak.addEventListener('click', resetPeakCurrent);

    // Acciones rápidas
    if (dom.btnSendTest) dom.btnSendTest.addEventListener('click', () => {
      logTerminal('[COMANDO] Ejecutando test secuencial de LEDs...');
      sendSerialCommand('t');
      if (state.mode === 'sim') simulateLedTestSequence();
    });

    if (dom.btnSendCalib) dom.btnSendCalib.addEventListener('click', () => {
      logTerminal('[COMANDO] Calibrando referencias virtuales VB (2.5 V)...');
      sendSerialCommand('c');
      addAlarm('INFORMATIVA', 'Calibración de tierra virtual VB ejecutada.', 'Offsets en reposo');
    });

    if (dom.btnSendOff) dom.btnSendOff.addEventListener('click', () => turnOffAllSteps(true));

    // Conmutadores individuales del Banco 1:2:4
    if (dom.btnStepToggle1) dom.btnStepToggle1.addEventListener('click', () => setStep(0, !state.stepState[0]));
    if (dom.btnStepToggle2) dom.btnStepToggle2.addEventListener('click', () => setStep(1, !state.stepState[1]));
    if (dom.btnStepToggle3) dom.btnStepToggle3.addEventListener('click', () => setStep(2, !state.stepState[2]));

    // Pausa osciloscopio
    if (dom.btnPauseScope) {
      dom.btnPauseScope.addEventListener('click', () => {
        state.scope.paused = !state.scope.paused;
        dom.btnPauseScope.textContent = state.scope.paused ? 'Reanudar' : 'Pausa';
      });
    }

    // Exportación CSV
    if (dom.btnQuickExportCSV) dom.btnQuickExportCSV.addEventListener('click', exportCSV);
    if (dom.btnExportCSVTab) dom.btnExportCSVTab.addEventListener('click', exportCSV);

    if (dom.btnClearTelemetry) {
      dom.btnClearTelemetry.addEventListener('click', () => {
        state.telemetryHistory = [];
        recordTelemetryHistory();
      });
    }

    // Terminal Serie
    if (dom.btnClearTerminal) dom.btnClearTerminal.addEventListener('click', () => dom.terminalOutput.textContent = '');
    if (dom.termSendBtn) {
      dom.termSendBtn.addEventListener('click', () => {
        const cmd = dom.termInput.value.trim();
        if (cmd) { sendSerialCommand(cmd); dom.termInput.value = ''; }
      });
    }
    if (dom.termInput) {
      dom.termInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
          const cmd = dom.termInput.value.trim();
          if (cmd) { sendSerialCommand(cmd); dom.termInput.value = ''; }
        }
      });
    }

    // Panel Drawer de Alarmas
    function toggleAlarmDrawer(open) {
      if (open) {
        dom.alarmDrawer.classList.add('open');
        dom.alarmBackdrop.classList.add('open');
      } else {
        dom.alarmDrawer.classList.remove('open');
        dom.alarmBackdrop.classList.remove('open');
      }
    }
    if (dom.btnOpenAlarms) dom.btnOpenAlarms.addEventListener('click', () => toggleAlarmDrawer(true));
    if (dom.btnFooterOpenAlarms) dom.btnFooterOpenAlarms.addEventListener('click', () => toggleAlarmDrawer(true));
    if (dom.btnCloseAlarms) dom.btnCloseAlarms.addEventListener('click', () => toggleAlarmDrawer(false));
    if (dom.alarmBackdrop) dom.alarmBackdrop.addEventListener('click', () => toggleAlarmDrawer(false));

    if (dom.btnAckAllAlarms) {
      dom.btnAckAllAlarms.addEventListener('click', () => {
        state.alarms.forEach(a => a.acknowledged = true);
        updateAlarmFeed();
        logTerminal('[ALARMAS] Todas las alarmas han sido reconocidas.');
      });
    }

    // Modal Docs
    function toggleDocsModal(open) {
      if (open) {
        dom.modalDocs.classList.remove('hidden');
        dom.modalDocs.classList.add('flex');
      } else {
        dom.modalDocs.classList.remove('flex');
        dom.modalDocs.classList.add('hidden');
      }
    }
    if (dom.btnOpenDocs) dom.btnOpenDocs.addEventListener('click', () => toggleDocsModal(true));
    if (dom.btnCloseDocs) dom.btnCloseDocs.addEventListener('click', () => toggleDocsModal(false));
    if (dom.btnModalCloseAction) dom.btnModalCloseAction.addEventListener('click', () => toggleDocsModal(false));
    if (dom.modalDocs) dom.modalDocs.addEventListener('click', (e) => {
      if (e.target === dom.modalDocs) toggleDocsModal(false);
    });

    // Atajos de teclado para laboratorio
    window.addEventListener('keydown', (e) => {
      if (document.activeElement === dom.termInput) return;
      if (e.key === 't' || e.key === 'T') dom.btnSendTest.click();
      else if (e.key === '1') dom.btnStepToggle1.click();
      else if (e.key === '2') dom.btnStepToggle2.click();
      else if (e.key === '3') dom.btnStepToggle3.click();
      else if (e.key === '0') dom.btnSendOff.click();
      else if (e.key === 'c' || e.key === 'C') dom.btnSendCalib.click();
      else if (e.key === 'i' || e.key === 'I') triggerInrushPeak();
      else if (e.key === 'p' || e.key === 'P') dom.btnTogglePresentation.click();
    });
  }

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
  // ARRANQUE DEL SISTEMA
  // ==========================================================================
  function start() {
    initEventListeners();
    setupOscilloscope();
    setupPowerTriangle();

    logTerminal('==================================================================');
    logTerminal(' POWERGuard 5 — SCADA Industrial de Compensación de FP');
    logTerminal(' Instituto Kriete de Ingeniería y Ciencias · Circuitos Eléctricos');
    logTerminal(' • Condición Real: Motor WEG W01 0.5 HP en VACÍO CONTINUO');
    logTerminal(' • P = 130 W | Q_motor = 600 VAR | FP sin compensar = 0.212');
    logTerminal(' • Banco 105 µF (570 VAR): Q_neta = 30 VAR -> FP Compensado = 0.974');
    logTerminal(' • Picos de arranque: 10 A a 15 A medidos por sensor ACS712-30A');
    logTerminal('==================================================================');
    logTerminal('[SCADA] Sistema iniciado en modo simulación local.');

    addAlarm('INFORMATIVA', 'Sistema SCADA iniciado. Supervisión activa en 120 V / 60 Hz.', 'Banco de capacitores preparado');

    // Bucle periódico cada 500 ms (coincidente con ATmega328P)
    setInterval(() => {
      if (state.mode === 'sim') {
        runSimulationStep();
      } else {
        updateUI();
      }
    }, 500);

    // Ejecución inicial inmediata
    runSimulationStep();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', start);
  } else {
    start();
  }

})();
