# Punto 2 - Control a Lazo Cerrado: Experimentacion

## Controlador implementado

Se implemento un controlador proporcional (P) independiente para cada grado de libertad del robot omnidireccional. El error se computa en el marco del robot (body frame) y la accion de control es:

```
vx = K_X * dx
vy = K_Y * dy
w  = K_THETA * d_theta
```

Donde `dx`, `dy` son las componentes del error de posicion rotadas al frame del robot, y `d_theta` es el error angular normalizado.

La seleccion de objetivo se realiza mediante Pursuit-Based Goal Selection: se busca el waypoint mas cercano y luego se avanza por la trayectoria hasta encontrar uno a distancia `lookahead` del robot.

## Parametros

Se configuran en `launch/lazo_cerrado.launch.py`:

- `k_x`, `k_y`: ganancias proporcionales de posicion
- `k_theta`: ganancia proporcional de orientacion
- `lookahead`: distancia de seleccion de waypoint objetivo

## Experimentos realizados

Se realizaron multiples experimentos variando las ganancias y el lookahead para encontrar la combinacion que mejor siga la trayectoria cuadrada de 2m.

| # | K_X | K_Y | K_THETA | Lookahead | Observacion |
|---|-----|-----|---------|-----------|-------------|
| 1 | 0.5 | 0.5 | 0.5 | 0.6 | Trayectoria circular, esquinas completamente redondeadas |
| 2 | 2.0 | 2.0 | 1.5 | 0.3 | Cuadrado marcado pero odometria diverge del ground truth por patinaje |
| 3 | 0.6 | 0.6 | 0.8 | 0.25 | Buen balance, cuadrado definido con esquinas suaves |
| 4 | 0.6 | 0.6 | 1.0 | 0.2 | Sobre-correccion angular, trayectoria aparece rotada |
| 5 | 0.6 | 0.6 | 0.8 | 0.2 | Oscilaciones en esquinas |
| 6 | 0.8 | 0.8 | 0.8 | 0.25 | Patinaje en esquinas, peor que K=0.6 |
| 7 | 0.6 | 0.6 | 0.4 | 0.3 | Sin mejora, esquinas mas redondeadas por lookahead mayor |
| 8 | 0.6 | 0.6 | 0.8 | 0.1 | Mejor resultado. Esquinas marcadas, odometria pegada al ground truth |
| 9 | 0.6 | 0.6 | 0.8 | 0.15 | Similar al anterior, levemente menos definido |
| 10 | 0.6 | 0.6 | 0.8 | 0.05 | Demasiado lento, acumula drift por tiempo de recorrido |
| 11 | 0.8 | 0.8 | 0.8 | 0.1 | Patinaje en rotacion inicial, peor que K=0.6 |

## Analisis

### Lookahead

Es el parametro mas influyente en la forma de la trayectoria. Determina que tan "adelante" mira el robot para elegir su proximo objetivo:

- Valores altos (>0.5m): el goal salta al siguiente lado del cuadrado antes de completar la esquina actual, generando curvas
- Valores optimos (~0.1m): esquinas marcadas, el robot sigue pegado al path
- Valores muy bajos (<0.05m): la velocidad efectiva cae mucho (porque v = K * error, y el error siempre es chico), el robot tarda mas y acumula drift

### Ganancias K

- K altos (>1.0): generan velocidades grandes que causan patinaje en las ruedas, provocando que la odometria se desfase del ground truth
- K moderados (~0.6): velocidades controladas, sin patinaje, odometria confiable
- K_THETA ligeramente mayor que K_X/K_Y (0.8 vs 0.6) porque la orientacion "opuesta al centro" cambia continuamente durante el recorrido y requiere correccion angular activa

## Configuracion final

```
K_X = 0.6
K_Y = 0.6
K_THETA = 0.8
Lookahead = 0.1
```

## Comparacion con lazo abierto

Se corrio la misma trayectoria cuadrada con el controlador de lazo abierto (`ros2 launch lazo_abierto lazo_abierto.launch.py`). En condiciones ideales de simulacion, ambos controladores completan la trayectoria de forma similar. Pero se pudo notar una mejora de performance con el lazo cerrado.

La diferencia fundamental es que el lazo abierto ejecuta velocidades pre-calculadas sin feedback: si ocurre una perturbacion (patinaje, error de modelo, choque), el error se acumula sin correccion posible. El lazo cerrado usa la odometria como feedback y corrige desviaciones activamente, lo que lo hace robusto ante perturbaciones y necesario para un robot real.

## Nota sobre la rotacion inicial

La trayectoria incluye una fase de 1 segundo donde el robot rota en el lugar (~135 grados) para orientarse "opuesto al centro" del cuadrado. Esto se observa como un amontonamiento de flechas en el punto de inicio y es comportamiento programado en el generador de trayectoria, no un error del controlador.
