# POWERGuard 5 — Sistema Automático de Compensación de Factor de Potencia

**Instituto Kriete de Ingeniería y Ciencias**  
*Materia:* Circuitos Eléctricos  
*Equipo:* **PowerGuard 5**  
- **Diana Carballo**  
- **Emma Palacios**  
- **Eduardo Figueroa**  
- **Juan Pablo Ramírez**  

---

## 1. Descripción del Proyecto

**POWERGuard 5** es una plataforma de instrumentación virtual y control automático de potencia reactiva diseñada para mitigar el bajo factor de potencia originado por cargas inductivas industriales. El sistema cuenta con adquisición de señales de tensión y corriente en tiempo real mediante un microcontrolador **Arduino Nano (ATmega328P)**, conmutación de un banco de capacitores con ponderación binaria **1:2:4 (105 µF máximo)** a través de optotriacs con cruce por cero y TRIACs de potencia, y un **Dashboard de Telemetría Web** conectado vía **Web Serial API**.

---

## 2. Especificaciones Técnicas del Hardware

### 2.1 Carga Principal (Motor de Inducción Monofásico)
- **Fabricante y Modelo:** WEG W01 Monofásico (Rotor Jaula de Ardilla).
- **Potencia Nominal:** 0.5 HP (0.37 kW) · 4 polos · 1735 RPM · 60 Hz.
- **Tensión de Operación:** 115/120 VAC.
- **Corriente Nominal a Plena Carga:** 8.24 A.
- **Condición Real de Banco de Pruebas (Laboratorio):**
  - **Operación Permanente en Vacío:** El motor se prueba sin carga mecánica en el eje.
  - **Factor de Potencia sin Compensar:** $\text{FP}_{\text{máx}} \approx 0.18 - 0.20$ inductivo puro ($P \approx 130\text{ W}$, $Q \approx 640\text{ VAR}$, $S \approx 653\text{ VA}$, $\phi \approx 78.5^\circ$).
  - **Pico de Corriente de Arranque (Inrush):** Entre **10 A y 15 A**, medidos con total fidelidad lineal gracias al sensor de efecto Hall ACS712-30A (rango $\pm 30\text{ A}$).
  - **Efecto de la Compensación:** Al conectar el banco binario de $105\,\mu\text{F}$ ($Q_C = 570\text{ VAR}$), la reactiva neta se reduce a $< 80\text{ VAR}$, catapultando el factor de potencia desde **0.20 hasta más de 0.95** y reduciendo drásticamente la corriente de línea de **5.44 A a solo ~1.2 A** (alivio térmico del 78% en conductores).

### 2.2 Etapa de Sensado y Acondicionamiento
- **Canal A0 (Sensor de Tensión):** Transformador reductor $T_1$ ($120/12\text{ VAC}$) + Divisor resistivo $100\text{ k}\Omega / 10\text{ k}\Omega$ centrado sobre referencia virtual de $V_B = 2.5\text{ V}$ mediante divisor simétrico de $5\text{ V}$ desacoplado con capacitor. Factor de escala de calibración: `CALIB_V = 110.0`.
- **Canal A1 (Sensor de Corriente):** Sensor de efecto Hall **ACS712-30A** (sensibilidad $66\text{ mV/A}$, aislamiento galvánico de $2.1\text{ kV}_{\text{rms}}$) con reposo en $2.5\text{ V}$. Factor de escala: `CALIB_I = 15.1515 A/V`.
- **Frecuencia de Muestreo ($f_s$):** 32 muestras/ciclo a $60\text{ Hz}$ ($f_s = 1920\text{ Hz}$, $T_s = 520.83\,\mu\text{s}$). Ventana de integración de 10 ciclos completos ($N = 320\text{ muestras}$) cada $500\text{ ms}$.

### 2.3 Banco de Capacitores y Etapa de Disparo (Ponderación 1:2:4)
| Paso | Pin Arduino | LED Físico | Rama Hardware | Capacitor ($370\text{ VAC}$) | $Q_C$ Inyectada ($120\text{ V}$) | $I_C$ Inyectada | Función de Ingeniería |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Paso 1** | **D4** | **ROJO** | MOC3063 (U3) + BTA16 (TR1) | $C_1 = 15\,\mu\text{F}$ | $81.4\text{ VAR}$ | $0.68\text{ A}$ | Peso Binario 1 · Ajuste Fino dentro de la banda de histéresis |
| **Paso 2** | **D5** | **AZUL** | MOC3063 (U4) + BTA16 (TR2) | $C_2 = 30\,\mu\text{F}$ | $162.9\text{ VAR}$ | $1.36\text{ A}$ | Peso Binario 2 · Escalón Intermedio |
| **Paso 3** | **D6** | **VERDE** | MOC3063 (U5) + BTA16 (TR3) | $C_3 = 60\,\mu\text{F}$ | $325.7\text{ VAR}$ | $2.71\text{ A}$ | Peso Binario 4 · Carga Base magnetizante del motor |

- **Optoacopladores de Cruce por Cero (Zero-Cross):** MOC3063 que conmutan los TRIACs únicamente cuando $v(t) \approx 0\text{ V}$, eliminando corrientes transitorias de inserción ($di/dt$).
- **Termistores de Protección:** NTC 5D-9 en serie con cada rama de capacitores para limitar picos de corriente residual.
- **Resistencias de Descarga Permanentes:** $100\text{ k}\Omega / 2\text{ W}$ en paralelo con cada capacitor.
  - Constantes de tiempo RC: $\tau_1 = 1.5\text{ s}$, $\tau_2 = 3.0\text{ s}$, $\tau_3 = 6.0\text{ s}$.
  - **Bloqueo de Seguridad por Software:** Se impone un retardo de $30\text{ s}$ ($5\tau_3$) antes de permitir la reconexión de cualquier paso desactivado, garantizando que el voltaje remanente decaiga por debajo del 1% del valor pico.

---

## 3. Memoria de Cálculo Implementada

### 3.1 Muestreo Discreto (Factor de Potencia Verdadero - True RMS)
$$\begin{aligned}
V_{\text{rms}} &= \sqrt{\frac{1}{N} \sum_{k=0}^{N-1} v_k^2} \\
I_{\text{rms}} &= \sqrt{\frac{1}{N} \sum_{k=0}^{N-1} i_k^2} \\
P &= \frac{1}{N} \sum_{k=0}^{N-1} (v_k \cdot i_k) \\
Q &= \frac{1}{N - N/4} \sum_{k=N/4}^{N-1} \left( v_{k - N/4} \cdot i_k \right) \quad (\text{donde } N/4 = 8 \text{ muestras representa un retardo de } 90^\circ) \\
S &= V_{\text{rms}} \cdot I_{\text{rms}} \\
\text{FP} &= \frac{|P|}{S} = \cos(\phi)
\end{aligned}$$

### 3.2 Dimensionamiento de la Compensación
Para elevar el factor de potencia desde $\text{FP}_1$ hasta la meta óptima $\text{FP}_2 = 0.97$ ($\tan\phi_2 = 0.2506$):
$$\begin{aligned}
\phi_1 &= \arccos(\text{FP}_1) \\
\Delta t &= \frac{\phi_1}{360^\circ \cdot 60\text{ Hz}} \times 1000\text{ ms} \\
Q_{C,\text{req}} &= P \cdot (\tan\phi_1 - \tan\phi_2) \\
C_{\text{req}} &= \frac{Q_{C,\text{req}}}{\omega \cdot V_{\text{rms}}^2} \times 10^6\,\mu\text{F} \quad (\text{con } \omega = 2\pi \cdot 60 = 376.99\text{ rad/s})
\end{aligned}$$

### 3.3 Validación de Error Experimental (Rúbrica)
$$\text{Error } (\%) = \left| \frac{\text{FP}_{\text{medido}} - \text{FP}_{\text{calculado}}}{\text{FP}_{\text{calculado}}} \right| \times 100\%$$

---

## 4. Instrucciones de Uso

### Opción A: Ejecución con Servidor Local (Recomendada para Web Serial API)
1. Conecta el **Arduino Nano** al puerto USB de la computadora.
2. Abre la carpeta del proyecto en una terminal o haz doble clic sobre `start_dashboard.bat`.
3. También puedes ejecutar:
   ```bash
   python start_server.py
   # o bien:
   npm start
   ```
4. El navegador se abrirá en `http://localhost:8080`.
5. Haz clic en **"Conectar Arduino Nano (USB)"**, selecciona el puerto COM correspondiente y confirma la conexión a 115200 baudios.

### Opción B: Modo Simulación (Sin Arduino)
1. Abre `index.html` en Google Chrome o Microsoft Edge.
2. El dashboard iniciará inmediatamente en **Modo Simulación**.
3. Selecciona cualquiera de los 4 presets de carga del motor WEG:
   - **Plena Carga:** 588 W / 744 VAR (FP 0.62)
   - **Carga Media:** 350 W / 650 VAR (FP 0.47)
   - **En Vacío:** 150 W / 600 VAR (FP 0.24)
   - **Apagar Motor:** 0 W / 0 VAR (activa desconexión automática de capacitores por seguridad)
4. Activa o desactiva el **Control Automático (0.95 - 0.99)** o conmuta manualmente los pasos 1, 2 y 3.

---

## 5. Comandos del Protocolo Serie (115200 Baudios)

| Comando | Acción en Firmware |
| :---: | :--- |
| `j` | Alterna modo de salida JSON (usado por el Dashboard). |
| `t` | Inicia rutina secuencial de prueba diagnóstica de los 3 LEDs y optotriacs (D4 -> D5 -> D6). |
| `1` | Conmuta el estado del Paso 1 (LED Rojo · 15 µF) si no está en temporizador de descarga. |
| `2` | Conmuta el estado del Paso 2 (LED Azul · 30 µF) si no está en temporizador de descarga. |
| `3` | Conmuta el estado del Paso 3 (LED Verde · 60 µF) si no está en temporizador de descarga. |
| `0` | Desconexión general de emergencia de los 3 capacitores. |
| `c` | Calibra y recalcula los offsets de tierra virtual $V_B$ en A0 y A1 (400 muestras). |

---

## 6. Estructura de Archivos

```
pwrguardia/
├── index.html                    # Dashboard interactivo principal (HTML5 + Tailwind CSS)
├── styles.css                    # Estilos Dark Industrial, diodos LED 3D y CRT
├── app.js                        # Motor matemático, Web Serial API, osciloscopio y lazo de control
├── start_server.py               # Servidor HTTP local con apertura automática de navegador
├── start_dashboard.bat           # Acceso directo para Windows
├── package.json                  # Scripts npm
├── README.md                     # Documentación técnica completa
└── arduino/
    └── POWERGuard_Firmware.ino   # Firmware para Arduino Nano (ATmega328P)
```

