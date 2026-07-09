#### Elaborado: 01/07/2026
#### Especificación del dataset experimental — Journal Track 3

Este documento define qué variables debe registrar la nueva campaña de adquisición para soportar, con un único dataset, los ejes (A) NLS, (B) calibración quasi-Lambertiana, (C) escala espacial completa y (D1) comparación cooperativo-vs-broadcast definidos en `overview.md`.

---

## 1. Niveles de la campaña

La campaña se divide en **tres sub-datasets** independientes:

1. **Calibración radiométrica** (eje B): caracterización de `R(φ)` del LED, independiente de la posición del receptor.
2. **Calibración de la constante `C`** (igual que en `GLOBECOM_2026`): medición a distancias conocidas, LED apuntando al nadir, PD apuntando al cenit.
3. **Campaña espacial principal** (~2000 puntos): dataset central para (A), (C), (D1) y opcionalmente (D2).

---

## 2. Sub-dataset 1 — Calibración radiométrica `R(φ)`

Objetivo: obtener el perfil real de irradiancia del LED en función del ángulo, independiente del `cos^m(φ)` ideal.

Variables a registrar por muestra de ángulo:
- `phi_cmd` — ángulo de irradiancia comandado al gimbal (rad o deg).
- `phi_meas` (si disponible) — ángulo efectivamente alcanzado (encoder del gimbal, resolución 0.1° con los Thorlabs PRM1Z8).
- `azimuth_cmd` — ángulo azimutal del gimbal, para verificar posible asimetría no solo en φ sino también en el plano azimutal (relevante si el patrón es asimétrico, no solo quasi-Lambertiano en magnitud).
- `d_fixed` — distancia fija LED–PD durante el barrido (constante, conocida con precisión).
- `V_raw[n]` — muestras crudas de voltaje (N muestras, ej. 1000 @ 1kHz) por ángulo.
- `V_dark[n]` — muestras con LED apagado (ruido de fondo / offset), en el mismo ángulo, para sustracción de línea base.
- `I_LED` — corriente de driving del LED monitoreada (para confirmar que el driver reparado entrega corriente estable; **crítico** dado el fallo previo).
- `T_ambient`, `T_LED` (si hay sensor) — temperatura ambiente / del LED, por posible deriva térmica del patrón.
- Metadata de sesión: fecha, hora, operador, LED_serial, PD_serial, ganancia del amplificador TIA+OPAM, estado de iluminación ambiente (luces del laboratorio encendidas/apagadas).

Rango de barrido sugerido: `φ ∈ [0°, 90°]` con paso fino (ej. 2°–5°) para permitir el ajuste de spline `R_spline(φ)` con buena resolución, cubriendo azimut en al menos 2–4 cortes (0°, 90°, 180°, 270°) para detectar asimetría.

---

## 3. Sub-dataset 2 — Calibración de la constante radiométrica `C`

Igual que el procedimiento ya usado en `GLOBECOM_2026`: PD directamente bajo el LED, LED apuntando al nadir, PD apuntando al cenit, a varias alturas conocidas.

Variables:
- `d_calib` — distancia conocida LED–PD (ej. 0.4, 0.6, 0.8, 1.0, 1.2 m).
- `V_raw[n]` — muestras de voltaje en cada distancia.
- `V_dark[n]` — línea base (LED apagado).
- `I_LED` — corriente de driving (verificación de estabilidad).
- Repetir con `R(φ)` calibrado (sub-dataset 1) para reportar `C` corregido vs. `C` bajo modelo Lambertiano ideal.

---

## 4. Sub-dataset 3 — Campaña espacial principal (~2000 puntos)

### 4.1 Variables de posicionamiento / ground truth (por punto)

- `point_id` — identificador único del punto.
- `x, y, z` — posición del receptor (ground truth), del sistema de coordenadas del robot/testbed.
- `pose_UR5` — pose completa del end-effector del UR5 (posición + orientación, cuaternión o matriz de rotación), **no solo la posición**. Esto es lo que habilita reprocesar el dataset en el pipeline broadcast sin medición `(K+1)` adicional.
- `n_r` — vector unitario de orientación del PD, derivado de `pose_UR5` (verificar signo/convención respecto al modelo teórico).
- `robot_repeatability` — especificación de repetibilidad del UR5 (referencia de datasheet, ej. ±0.03 mm), documentada como fuente de incertidumbre del ground truth.

### 4.2 Variables de direction-finding (`K` orientaciones, por punto)

Para cada orientación `i = 1,...,K` (K=9 según el codebook optimizado de `TCOM`):
- `n_t,i` — orientación comandada del LED (o los dos ángulos del gimbal transmisor).
- `V_raw_i[n]` — muestras crudas de voltaje (N muestras, ej. 1000 @ 1kHz, igual que `GLOBECOM_2026`).
- `V_dark_i[n]` — línea base con LED apagado en esa orientación (si el offset puede variar por orientación del gimbal, ej. luz ambiente direccional).
- `t_i` — timestamp de la medición (para análisis de jitter/sincronización, ver `TCOM` action_plan_reviewers.md ítem B2).

### 4.3 Variable de distance-recovery cooperativo — medición `(K+1)`

- `n_t,K+1` — orientación del LED alineada hacia el receptor (calculada a partir de la posición ground truth).
- `n_r,K+1` — orientación del PD reorientado hacia el LED (pose del UR5 en ese instante).
- `V_raw_{K+1}[n]`, `V_dark_{K+1}[n]` — mismas convenciones que arriba.
- `t_{K+1}` — timestamp.

### 4.4 Repeticiones para estadística de ruido

- `M_repeats` — número de repeticiones del escaneo completo `{K, K+1}` por punto (sugerido: 10–30, limitado por tiempo de campaña; ídem al rol de `M=1000` en Monte Carlo simulado, pero aquí acotado por restricciones físicas). Permite estimar varianza empírica por punto y compararla con `σ²` teórico usado en `TCOM`/`F_broadcast_Konly`.

### 4.5 Variables ambientales y de sistema (por sesión o periódicas)

- `I_LED` — corriente de driving monitoreada durante toda la sesión (detectar cualquier recurrencia del fallo del driver).
- `T_ambient` — temperatura ambiente.
- `ambient_light_state` — condición de luz ambiente (controlada/oscuridad, o nivel medido si hay fotómetro adicional).
- `session_date`, `operator`, `daq_sample_rate`, `amp_gain` — metadata de trazabilidad.

### 4.6 Barrido deliberado de tilt

Repetir el escaneo `{K}` (no necesariamente `{K+1}`) con el PD deliberadamente inclinado a ángulos conocidos `θ_tilt ∈ {0°, 5°, 10°, 20°}` y azimut de tilt variado. Es decir para una sola posición habrá uno que es vertical y otros tendran inclinación.

---

## 5. Formato de almacenamiento sugerido

- Un archivo maestro (`.mat` o `.csv`) con una fila por `(point_id, orientation_id, repeat_id)` conteniendo todas las variables de §4.1–4.2, y una tabla separada para las mediciones `(K+1)` (§4.3).
- Archivos separados para los sub-datasets de calibración (§2, §3), indexados por sesión/fecha.
- Mantener las muestras crudas `V_raw[n]` (no solo la media) para poder recomputar estadísticas de ruido y estimar `σ²` empírico, análogo a como se hace en simulación.
