# Punto 3 - Localización con EKF: Experimentación

## Objetivo

Queríamos responder dos preguntas:

1. **¿Cuánto mejora el EKF a la odometría?** Para eso comparamos la pose del EKF contra la odometría sola y contra la pose real (ground truth) del simulador.
2. **¿Qué covarianzas Q (predicción) y R (sensado) conviene usar?** La consigna pide que reflejen mayor incertidumbre al predecir, o sea, que el filtro confíe más en el láser que en la odometría.

## Metodología

Para que las corridas fueran comparables entre sí, automatizamos el protocolo. Los scripts están en `volume/experimentos/ekf/tools/`. Cada corrida sigue siempre los mismos pasos:

1. Stop de la simulación y se matan los nodos de la corrida anterior.
2. Se lanzan la odometría, el detector de postes (con `laser_frame=front_laser`) y el localizer.
3. Play de la simulación. El orden importa: Coppelia publica `/posts` **una sola vez** al dar Play, así que el localizer tiene que estar corriendo antes.
4. Se espera a que el EKF quede inicializado con la pose de arranque, (-2, 2, π/2) en `map`.
5. Se graba un bag (`/pose`, `/robot/odometry`, `/robot/ground_truth`, `/tf`, `/landmarks`) y un CSV a 10 Hz con las tres poses en el frame `map`, tomadas en el **mismo instante** de TF.
6. Se publica una velocidad constante en `/robot/cmd_vel` durante un tiempo fijo y después velocidad cero.

Al cambiar Q/R hay que recompilar, porque están en `localizer_ekf.cpp` y no son parámetros. `only_prediction` sí se pasa como parámetro del nodo.

### Mapa y sensor

- 16 postes en el borde de un cuadrado de 8×8 m, uno cada 2 m (x, y ∈ {-4, -2, 0, 2, 4}).
- Láser frontal de 180° y 4 m de alcance.
- El filtro solo corrige si ve **al menos 2 postes**, y asocia cada medición al poste del mapa más cercano dentro de 1 m.

### Movimientos

| Nombre | Velocidad constante | Duración | Qué pone a prueba |
|---|---|---|---|
| `rot` | ω = 0.4 rad/s | 30 s | Rotación lenta en el lugar, con tramos de ~4.5 s viendo un solo poste |
| `rotrapida` | ω = 1.5 rad/s | 20 s | Rotación rápida: las ruedas patinan y la odometría deriva en θ |
| `curva` | vx = 0.15 m/s, ω = 0.2 rad/s | 40 s | Círculo suave de radio 0.75 m |
| `orbita` | vy = -0.5 m/s, ω = -0.4 rad/s | 31 s | Movimiento lateral puro (círculo de 1.25 m de radio, ~10 m recorridos) |
| `circulo` | vx = -0.4, vy = 0.4 m/s, ω = 0.2 rad/s | 32 s | Una vuelta grande alrededor del centro (radio 2.8 m) con mucha componente lateral |
| `orbitarapida` | vy = -0.8 m/s, ω = -0.64 rad/s | 30 s | Como `orbita` pero más rápido, para forzar el patinaje |

### Configuraciones

Partimos de los valores del taller (`base`): Q = diag(0.001, 0.001, 0.02²) por paso de predicción y R = diag(0.001, (0.5°)²).

| Nombre | Q | R | Idea |
|---|---|---|---|
| `soloPred` | base | base | `only_prediction=true`: el EKF nunca corrige. Es la referencia sin láser |
| `base` | ×1 | ×1 | Valores originales |
| `q10`, `q100` | ×10, ×100 | ×1 | Confiar cada vez más en el láser |
| `qdiv10` | ÷10 | ×1 | Confiar más en la odometría |
| `r10`, `r100` | ×1 | ×10, ×100 | Confiar cada vez menos en el láser |
| `rdiv10` | ×1 | ÷10 | Confiar todavía más en el láser |

En total corrimos 6 movimientos × 8 configuraciones = 48 corridas. La tabla completa está en `volume/experimentos/ekf/resumen.md` y los gráficos en `volume/experimentos/ekf/plots/`.

## Resultados

### 1. EKF contra odometría

Errores medios contra el ground truth con la configuración `base`:

| Movimiento | Odometría (pos / θ) | EKF solo predicción (pos / θ) | EKF (pos / θ) |
|---|---|---|---|
| `circulo` | **174 cm / 40.2°** | 170 cm / 39.7° | **0.4 cm / 0.14°** |
| `orbita` | 29 cm / 9.0° | 29 cm / 9.6° | 0.9 cm / 0.22° |
| `rotrapida` | 0.2 cm / 15.0° | 0.2 cm / 12.6° | 0.3 cm / 0.28° |
| `curva` | 1.6 cm / 0.49° | 1.9 cm / 0.80° | 0.6 cm / 0.19° |
| `rot` | 0.2 cm / 1.6° | 0.2 cm / 0.9° | 0.5 cm / 0.38° |

Lo que vemos:

- **Cuando la odometría patina, el EKF la corrige casi por completo.** En el círculo grande el robot real hace una vuelta alrededor del centro, pero la odometría termina a más de 3 m del lugar real (1.74 m de error medio). El EKF se mantiene a 4 mm. En la órbita lateral pasa lo mismo a menor escala.
- **La rotación rápida muestra la corrección de la orientación.** La odometría deriva unos 15° de media y llega a 30°, porque al girar a 1.5 rad/s las ruedas patinan. El EKF queda en 0.28°.
- **Toda la mejora viene del láser.** `soloPred` da prácticamente lo mismo que la odometría, lo cual tiene sentido: sin correcciones, el filtro solo integra las mismas velocidades que la odometría.
- **En los movimientos suaves (`rot`, `curva`) la odometría ya es buena** y el EKF casi no tiene nada que corregir. En `rot` el EKF incluso tiene un poco más de error de posición que la odometría (0.5 cm contra 0.2 cm): el robot no se mueve del lugar, así que la odometría no acumula nada, y cada corrección le suma unos milímetros de ruido de la medición.

Figuras sugeridas: `plots/circulo_base_trayectoria.png` contra `plots/circulo_soloPred_trayectoria.png`, y `plots/rotrapida_base_error.png`.

### 2. Efecto de Q y R

Error medio de posición del EKF [cm] / σxy medio de la elipse [cm]:

| Config | `circulo` | `orbita` | `rotrapida` | `rot` |
|---|---|---|---|---|
| `qdiv10` | 0.5 / **4.6** | 1.0 / **6.4** | 0.6 / **11.5** | 0.5 / **8.7** |
| `base` | 0.4 / 12.4 | 0.9 / 17.6 | 0.3 / 18.2 | 0.5 / 25.1 |
| `q10` | 0.3 / 38.7 | 0.8 / 50.9 | 0.3 / 56.2 | 0.6 / 76.8 |
| `q100` | 0.3 / 119.3 | 0.8 / 166.3 | 0.5 / 169.4 | 1.1 / 242.0 |
| `rdiv10` | 0.3 / 11.5 | 0.8 / 15.6 | 0.3 / 16.7 | 0.6 / 26.4 |
| `r10` | 0.4 / 16.1 | 1.0 / 19.8 | 0.3 / 20.3 | 0.8 / 27.7 |
| `r100` | 1.2 / 21.8 | 1.6 / 28.1 | 0.5 / 27.7 | 0.6 / 41.6 |

Lo que vemos:

- **La precisión casi no depende de Q/R.** Mientras el robot ve postes (88–100 % del tiempo en estos recorridos), todas las configuraciones quedan entre 0.3 y 1.6 cm. Con 2 a 4 postes por escaneo la corrección es tan fuerte que domina. Solo `r100`, que confía mucho menos en el láser, empeora un poco.
- **Lo que sí cambia es el tamaño de la elipse.** σ crece casi proporcional a √Q. Con `base`, el filtro informa unos 12–25 cm de incertidumbre cuando el error real es de menos de 1 cm, o sea que es muy **pesimista**. Con `q100` llega a 1–2 m sin ninguna mejora en la precisión.
- **La razón es que Q se suma en cada predicción y el timer predice cada 10 ms.** Q = 0.001 por paso equivale a 0.1 m² por segundo. En `rot` con `soloPred`, σ llega a 2.5 m con el robot quieto en el lugar, porque Q no depende de la velocidad.

### 3. Caso de falla: salir de la zona de postes

En `orbitarapida` el patinaje es tan grande que el robot no hace el círculo planeado: sale en espiral y termina en (-4.3, -2.1), **afuera del cuadrado de postes**. Desde ahí el láser mira hacia afuera y ve uno o ningún poste, así que el filtro deja de corregir y solo predice con una odometría que ya tiene más de 4 m de error. La elipse crece hasta varios metros y, cuando vuelve a ver postes, la pose estimada ya está a más de 1 m de la real. La asociación por vecino más cercano no encuentra correspondencias y el filtro queda **perdido**.

En este movimiento todas las configuraciones fallan (errores máximos de 0.4 a 2 m). Como cada corrida patina distinto y el robot termina en lugares distintos, las diferencias entre configuraciones acá no nos parecieron confiables para elegir Q/R. Lo tomamos como un límite del método: el EKF necesita ver al menos 2 postes de forma continua, y si el error supera el radio de asociación no se recupera.

Figuras sugeridas: `plots/orbitarapida_base_trayectoria.png` y `plots/orbitarapida_q10_error.png`.

## Configuración final

Elegimos **Q ÷ 10** con R original:

```
Q = diag(0.0001, 0.0001, 0.00004)    // por paso de predicción (100 Hz)
R = diag(0.001, (0.5°)²)
```

Por qué:

1. **Misma precisión** que el resto de las configuraciones.
2. **Es la elipse más honesta.** σ de 5–11 cm, contra 12–25 cm de `base`, para errores reales de alrededor de 1 cm. Una elipse que representa bien la incertidumbre es la que conviene mostrar en RViz para el punto 3.5.
3. **Sigue cumpliendo la consigna.** Q = 0.0001 por paso son 0.01 m² por segundo de predicción, contra R = 0.001 m² por medición: en la etapa de corrección el láser sigue siendo la fuente más confiable.

## Limitaciones y posibles mejoras

- Q es fija por paso y no depende de Δt ni de la velocidad. Escribirla como `σ² · Δt`, o proporcional a la velocidad comandada, haría que la incertidumbre no dependa de la frecuencia del timer y que no crezca con el robot quieto.
- R es la misma para todos los postes. Los postes lejanos se detectan con menos puntos del láser y deberían pesar menos.
- Con solo 2 postes visibles o menos el filtro no corrige, y si el error supera 1 m no se recupera (no es un localizador global).

## Cómo reproducir

```bash
# dentro del contenedor, con Coppelia abierto con omni_ekf.ttt (sin Play)
bash /root/ros2_ws/src/robotica/experimentos/ekf/tools/run_all.sh
# o solo algunos movimientos
MOVS="circulo orbita" bash /root/ros2_ws/src/robotica/experimentos/ekf/tools/run_all.sh
# métricas y gráficos
python3 /root/ros2_ws/src/robotica/experimentos/ekf/tools/analyze.py /root/ros2_ws/src/robotica/experimentos/ekf
```

Las corridas de `rot` y `curva` se grabaron con la primera versión del muestreador, que tomaba cada TF en el último instante disponible y no en uno común. En movimientos lentos el desfase es despreciable. En `rotrapida` generaba picos falsos de hasta 4° en θ, por eso esa corrida se repitió con el muestreador corregido.
