# Caracterización: exactitud y precisión (borrador para el Notion)

> Borrador. El método de *ground truth* debe confirmarlo el ET (ver la sección 2).

## 1. Métricas (ISO 9283:1998, *Manipulating industrial robots — Performance criteria and related test methods*)

Para cada punto comandado P_j se registran n ≥ 10 posiciones alcanzadas p_i:

- Baricentro: p̄ = (1/n)·Σ p_i
- **Exactitud (AP)** = ‖p̄ − p_cmd‖, la distancia entre la posición media y la pedida. También se reporta el **error medio**, (1/n)·Σ‖p_i − p_cmd‖, que es el término que usa la guía.
- **Precisión / repetibilidad (RP)** = l̄ + 3·S_l, con l_i = ‖p_i − p̄‖ y S_l su desviación estándar muestral.

Protocolo: 5 puntos distribuidos en el espacio de trabajo y 10 ciclos P1→P2→…→P5, con un punto de aproximación común antes de cada uno para que todas las llegadas vengan de la misma dirección. Así lo hace el ciclo de prueba de ISO 9283. Lo ejecuta `ros2 run scara_control characterize` y lo analiza `ros2 run scara_control analyze`.

## 2. Ground truth propuesto: hoja de referencia impresa + marcador en el efector

1. Imprimir en láser, a escala 1:1, una retícula de 1 mm con los 5 puntos y el origen de la base marcados. Verificar la escala con calibrador sobre 200 mm (el error de escala de una impresora láser suele estar por debajo del 0.5 %).
2. Fijar la hoja a la mesa y registrarla con el marco de la base mediante dos referencias físicas (pines o esquinas de la base). Se mide la posición de esas referencias respecto al eje de q1.
3. Montar en el P un marcador de punta fina (o una punta con tinta). En cada repetición el robot baja el P (z abajo) y deja un punto.
4. Medir cada marca respecto a la retícula, con lupa o calibrador (resolución aproximada de 0.1–0.2 mm), o escaneando la hoja a 600 dpi (0.042 mm/píxel) y midiendo los centros de las marcas.

**Justificación.** Es una medición **externa e independiente** de los encoders: captura errores que los encoders no ven, como longitudes L1/L2 mal medidas, el offset del Home, holguras de la transmisión o la flexión de los brazos. Su resolución es al menos un orden de magnitud mejor que el error esperado del robot (milímetros), y es barata y reproducible.

Alternativas: cámara cenital con marcador ArUco (más automática, pero exige calibrar la cámara), o comparador/calibrador directo (mejor resolución, pero lento).

**Nota.** Los datos de encoder (`--source enc`) solo dan una repetibilidad *interna*. No son ground truth.

## 3. Resultados

*(pegar aquí la tabla `characterization_results/results_meas.md` y las gráficas `workspace_meas.png`, `per_point_meas.png` y `ap_rp_meas.png`)*
