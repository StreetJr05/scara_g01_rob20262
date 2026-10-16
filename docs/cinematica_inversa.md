# Cinemática inversa del SCARA RRP (implementada en el firmware)

Código: `firmware/main/kinematics.c` (C, ESP32), llamado desde `control.c` cada vez que llega una consigna por `/scara/cmd_pose`.

## 1. Planteamiento

Robot SCARA de configuración **RRP**: dos articulaciones de rotación con ejes verticales paralelos (q1 hombro, q2 codo) y una prismática vertical (P) accionada por un **solenoide binario** (sube con el solenoide energizado y baja al soltarlo).

Convenciones (vista superior, ángulos positivos antihorarios):

- q1: ángulo del brazo 1 respecto al eje +X de la base.
- q2: ángulo del brazo 2 respecto al brazo 1 (q2 = 0 → brazo extendido).
- L1 = 180 mm y L2 = 140 mm (valores provisionales en `scara_config.h` y en la xacro).

## 2. Cinemática directa

```
x = L1·cos(q1) + L2·cos(q1 + q2)
y = L1·sin(q1) + L2·sin(q1 + q2)
z = z_arriba   si P arriba (solenoide ON)
    z_abajo    si P abajo  (solenoide OFF)
```

## 3. Solución inversa (método geométrico, forma cerrada)

**Paso 1, ley de cosenos.** Con r² = x² + y²:

```
cos(q2) = c2 = (x² + y² − L1² − L2²) / (2·L1·L2)
```

**Paso 2, alcanzabilidad.** Debe cumplirse |c2| ≤ 1, es decir |L1 − L2| ≤ r ≤ L1 + L2 (un anillo entre 40 mm y 320 mm). Si no se cumple, la consigna se rechaza con `ERR_UNREACHABLE`.

**Paso 3, dos soluciones (codo + / codo −).**

```
s2 = ±sqrt(1 − c2²)
q2 = atan2(s2, c2)
```

**Paso 4, hombro.**

```
q1 = atan2(y, x) − atan2(L2·s2, L1 + L2·c2)
```

`atan2` resuelve el cuadrante sin ambigüedad, lo que evita usar `acos` para q1.

**Paso 5, eje Z (P binario).** Como P solo tiene dos estados, z se cuantiza:

```
P abajo  si  z < (z_arriba + z_abajo) / 2
P arriba si no
```

## 4. Soluciones múltiples: criterio de selección

Para cada consigna se calculan **las dos** soluciones. Cada una se lleva al rango articular (q + 2kπ) y se verifica contra los límites suaves. El criterio de selección es:

1. Si el parámetro `elbow_pref` fuerza un signo (+1 o −1) y esa solución es factible, se usa esa.
2. Si ambas son factibles, se elige la de **menor desplazamiento articular máximo**, `max(|Δq1|, |Δq2|)`. Como la trayectoria es sincronizada, el tiempo de movimiento depende de la articulación que más se desplaza, así que este criterio minimiza el tiempo y evita giros innecesarios del codo.
3. Si solo una es factible, se usa esa. Si ninguna lo es, la consigna se rechaza con `ERR_JOINT_LIMIT`.

El firmware publica en `/scara/event` qué solución tomó y si existían dos, por ejemplo: `GOTO #3 (200.0, 100.0, 25.0) mm -> IK q1=-12.16 q2=92.27 deg, elbow + (2 sols), P DOWN`.

## 5. Límites articulares

Los límites suaves (`J1_MIN_RAD … J2_MAX_RAD`) quedan unos grados **dentro** de los finales de carrera físicos. Se usan:

- en la IK, para descartar soluciones;
- en el jog, para recortar la referencia, una vez hecho Home.

Si durante un movimiento automático se activa un final de carrera, el robot entra en **FAULT** y apaga los motores.

## 6. Singularidades

Jacobiano del subsistema planar:

```
J = [ −L1·s1 − L2·s12   −L2·s12 ]
    [  L1·c1 + L2·c12    L2·c12 ]      det(J) = L1·L2·sin(q2)
```

| Singularidad | Condición | Significado | Tratamiento |
|---|---|---|---|
| Brazo extendido | q2 = 0 (r = L1 + L2) | borde exterior del espacio de trabajo; se pierde la movilidad radial | se rechaza si \|sin q2\| < 0.02 (`ERR_SINGULAR`) |
| Brazo plegado | q2 = ±π (r = \|L1 − L2\|) | borde interior; además queda fuera de los límites de q2 | se rechaza igual |
| Hombro | r = 0 (solo si L1 = L2) | q1 indeterminado | no ocurre con L1 ≠ L2 |

Cerca de det(J) = 0, un error cartesiano pequeño exige movimientos articulares muy grandes. Por eso se deja un margen alrededor de esas configuraciones.

## 7. Flujo completo de una consigna

```
Python (goto / pendant) --/scara/cmd_pose--> micro-ROS --> ik_solve() --> q_objetivo
  --> [sube P si estaba abajo] --> trayectoria quíntica sincronizada q_ref(t)
  --> PID discreto por articulación (encoders) --> PWM MC33926
  --> asentamiento |q_obj − q| < tol durante settle_ms
  --> [si z abajo: suelta solenoide (P baja) --> espera p_down_ms --> electroválvula ON]
  --> REACHED (/scara/event y bandera en /scara/state)
```

## 8. Validación

Una prueba en el PC (`gcc`, mismo `kinematics.c`) evaluó 200 000 configuraciones aleatorias dentro de los límites. Cada una se pasó por FK, luego por IK y de nuevo por FK. El error de ida y vuelta fue menor a 0.1 µm y 1 mrad en todos los casos. Las únicas configuraciones rechazadas fueron las singulares (|sin q2| < 0.02).
