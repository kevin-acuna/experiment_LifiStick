# Experimental Demonstration — PhD Thesis (Journal Track 3)

Proyecto reorganizado a partir de `Test1` para producir los tres sub-datasets
definidos en `../dataset_specification.md`. `Test1` queda intacto como respaldo.

El build (Visual Studio + NI-DAQmx + Thorlabs Kinesis) es el mismo que ya
funcionaba; solo se reorganizo `src/`.

---

## Estructura

```
src/
  experiment_config.h     Hiperparametros centralizados (motores, DAQ, codebook, tilt...)
  daq.h / daq.cpp         Adquisicion DAQ + resumen estadistico (media/mediana/std)
  datalog.h / datalog.cpp Timestamps, directorios de sesion, metadata, CSV, resume-state
  instrument.h/.cpp       Gimbal Thorlabs + LED (sin cambios)
  network_utils.h/.cpp    Cliente del servidor del robot (+ receivePose, receiverTilt)
  positions.h/.cpp        Carga de posiciones (sin cambios)
  sub1_radiometric.cpp    Sub-dataset 1: calibracion radiometrica R(phi)
  sub2_constant_c.cpp     Sub-dataset 2: calibracion de la constante C
  sub3_spatial.cpp        Sub-dataset 3: campana espacial ({K}, {K+1}, tilt)
  server/                 Servidor Python del robot (main.py devuelve la pose)
  positionsToSample/      Archivos de posiciones para sub3
output/                   Datasets generados (una carpeta por sesion)
```

---

## Build (Visual Studio, `Debug|x64`)

Solo **un** programa de experimento se compila a la vez. En `test.vcxproj`,
en el `ItemGroup` de `ClCompile`, pon `ExcludedFromBuild=false` en el programa
deseado y `true` en los otros dos:

- `sub1_radiometric.cpp`
- `sub2_constant_c.cpp`
- `sub3_spatial.cpp`  (activo por defecto)

Compila en la configuracion **Debug | x64** (la unica con las rutas de NI-DAQmx).

---

## Ejecucion

1. Arranca el servidor del robot (necesario para sub1 y sub3):
   ```
   python src/server/main.py
   ```
   (Para pruebas sin hardware puedes usar `src/emulator/emulator.py`.)
2. Ejecuta el binario compilado. Sigue las indicaciones en consola
   (metadata de sesion, medida de `V_dark`, etc.).

`sub2_constant_c.cpp` **no** usa el servidor (el PD se coloca manualmente).

---

## Convencion de orientaciones (marco global {G})

- **Inclinacion**: angulo desde `+Z` (vertical).
- **Azimut**: `atan2(n_y, n_x)`, desde `+X` hacia `+Y`.
- Misma convencion para el LED (`n_t`) y el PD (`n_r`).
- Al comandar el gimbal se aplica `fmod(az + 180, 360)` para alinear el cero
  mecanico con `+X` global; el dataset registra el azimut intencional.

---

## Formato de datos

Cada medida guarda el **resumen estadistico** (`v_mean`, `v_median`, `v_std`,
`n_samples`, `fs`) en vez de las muestras crudas. `v_std` conserva la varianza
empirica (sigma^2). `V_dark`, `I_LED`, temperaturas y demas trazabilidad van en
`metadata.txt` de cada sesion (metadata manual; `V_dark` se mide una sola vez).

### Sub-dataset 1 — `output/sub1_radiometric/<ts>/data.csv`
`sample_id, date, time, phi_cmd, azimuth_cmd, phi_meas, d_fixed, v_mean, v_median, v_std, n_samples, fs`

### Sub-dataset 2 — `output/sub2_constant_c/<ts>/data.csv`
`sample_id, date, time, d_calib, repeat_id, v_mean, v_median, v_std, n_samples, fs`

### Sub-dataset 3 — `output/sub3_spatial/<ts>/`
`master.csv` (una fila por `point_id, orientation_id, repeat_id`):
`point_id, x, y, z, scan_kind, tilt_cmd_deg, tilt_cmd_az, pose_px, pose_py, pose_pz, pose_qx, pose_qy, pose_qz, pose_qw, nr_incl, nr_az, orientation_id, nt_incl, nt_az, repeat_id, date, time, v_mean, v_median, v_std, n_samples, fs`

`kp1.csv` (medicion cooperativa K+1):
`point_id, repeat_id, nt_incl_kp1, nt_az_kp1, pose_px, pose_py, pose_pz, pose_qx, pose_qy, pose_qz, pose_qw, nr_incl, nr_az, date, time, v_mean, v_median, v_std, n_samples, fs`

- `pose_UR5`: posicion del end-effector + orientacion como cuaternion.
- `nr_incl, nr_az`: orientacion del PD proyectada, en inclinacion/azimut globales.
- `scan_kind`: `vertical` o `tilt`. El tilt es aleatorio **uniforme**
  (theta en `[0, TILT_MAX_DEG]`, azimut en `[0,360)`), generado por el
  orquestador C++ y ejecutado por el servidor, que reporta la pose alcanzada.

---

## Protocolo cliente C++ <-> servidor Python

Tras alcanzar una orientacion, el servidor responde:
```
reached px py pz qx qy qz qw nr_incl nr_az
```
Comandos: `vertical`, `pointed`, `tilt <theta> <az>`, `finished`.

---

## Parametros clave (`src/experiment_config.h`)

- `K_ORIENTATIONS = 9` (codebook TCOM), `M_REPEATS = 1`.
- `N_TILT_SCANS_PER_POINT = 1`, `TILT_MAX_DEG = 20`.
- DAQ: `Dev1/ai1`, 1000 muestras @ 1000 Hz.
- Distancias sub2: `{0.4, 0.6, 0.8, 1.0, 1.2}` m.
