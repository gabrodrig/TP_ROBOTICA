# Figuras para el informe (punto 3)

Todas las figuras están en `plots/`. Los números de cada corrida están en `resumen.md`, y la explicación de los experimentos en `volume/robmovil_ekf/EXPERIMENTACION.md`.

Cómo se generaron: cada corrida publica una velocidad constante en `/robot/cmd_vel` durante un tiempo fijo y graba a 10 Hz las poses de odometría, EKF y ground truth en el frame `map`. Los scripts están en `tools/` (`run_all.sh`, `run_35.sh`, `analyze.py`, `plot_35.py`, `plot_qr.py`).

## Sección 4: Localización basada en EKF

### Movimientos usados (velocidad constante en el marco del robot)

| Movimiento | vx [m/s] | vy [m/s] | ω [rad/s] | Duración | Para qué |
|---|---|---|---|---|---|
| circulo | -0.4 | 0.4 | 0.2 | 32 s | Recorrido largo con mucha componente lateral: la odometría patina mucho |
| orbita | 0 | -0.5 | -0.4 | 31 s | Movimiento lateral puro; tramos con pocos postes a la vista |
| rotrapida | 0 | 0 | 1.5 | 20 s | Giro rápido en el lugar: la odometría deriva en θ |
| orbitarapida | 0 | -0.8 | -0.64 | 30 s | Caso de falla: el robot sale del área de postes |

Cada corrida tiene 3 gráficos: `_trayectoria` (plano x-y), `_componentes` (x, y, θ en el tiempo) y `_error` (error contra ground truth, σ de la elipse y postes vistos).

### EKF contra odometría (con y sin corrección)

| Figura | Qué muestra |
|---|---|
| `circulo_soloPred_trayectoria.png` | EKF sin corregir (solo predicción): queda pegado a la odometría, lejos del recorrido real |
| `circulo_base_trayectoria.png` | Mismo movimiento con corrección: el EKF queda sobre el ground truth. Error medio 174 cm → 0.4 cm. **Figura principal** |
| `circulo_base_componentes.png` | x, y, θ en el tiempo de las tres fuentes ("la pose a lo largo del tiempo contra la real", como pide la consigna) |
| `circulo_base_error.png`, `circulo_soloPred_error.png` | Error en el tiempo con y sin corrección |
| `orbita_base_trayectoria.png`, `orbita_soloPred_trayectoria.png` | Órbita lateral con y sin corrección. Error medio 29 cm → 0.9 cm |
| `orbita_base_error.png` | La σ crece cuando ve pocos postes y baja de golpe al volver a verlos |
| `orbita_base_componentes.png` | x, y, θ en el tiempo para la órbita |
| `rotrapida_base_error.png`, `rotrapida_soloPred_error.png` | Corrección de la orientación: odometría 15° de error medio, EKF 0.28° |
| `rotrapida_base_componentes.png` | θ en el tiempo durante la rotación rápida |

### Covarianzas Q y R (punto 3.3)

| Figura | Qué muestra |
|---|---|
| `qr_error_vs_sigma.png` | Error real del EKF contra la σ que informa, para cada configuración. El error casi no cambia; la σ sí. Se eligió Q ÷ 10 (sombreado) porque su σ es la más cercana al error real |

### Caso de falla

| Figura | Qué muestra |
|---|---|
| `orbitarapida_base_trayectoria.png` | El robot patina tanto que sale del área de postes; el EKF deja de corregir y se separa del ground truth |
| `orbitarapida_base_error.png`, `orbitarapida_q10_error.png` | La σ crece varios metros cuando ve menos de 2 postes |

## Sección 5: Seguimiento con EKF (punto 3.5)

Mismo cuadrado, mismas ganancias (Kx = Ky = 0.6, Kθ = 0.8, lookahead 0.1 m). Solo cambia la pose que usa el controlador como feedback.

| Figura | Qué muestra |
|---|---|
| `cuadrado_odom_trayectoria.png` | Feedback de odometría: la odometría (lo que cree el controlador) va sobre el cuadrado, el robot real no |
| `cuadrado_ekf_trayectoria.png` | Feedback de EKF: el robot real queda sobre el cuadrado |
| `cuadrado_trayectorias.png` | Las dos corridas lado a lado, con las cuatro curvas |
| `cuadrado_error_seguimiento.png` | Distancia del robot real al cuadrado en el tiempo. Media 4.3 cm → 0.4 cm, máximo 14.3 cm → 4.7 cm |
| `cuadrado_odom_componentes.png`, `cuadrado_ekf_componentes.png` | Setpoint, ground truth, odometría y EKF componente a componente |
| `cuadrado_odom_velocidades.png`, `cuadrado_ekf_velocidades.png` | Velocidades comandadas por el controlador |
| `rviz_cuadrado_ekf_t040s.png`, `_t080s.png`, `_t120s.png`, `_final.png` | Capturas de RViz con la elipse de covarianza en distintos tramos (pedido explícito de la consigna). La elipse está ampliada ×3 para que se vea |
| `rviz_cuadrado_odom_t080s.png`, `rviz_cuadrado_odom_final.png` | Mismas capturas con feedback de odometría, para comparar |

En las capturas: blanco = cuadrado pedido, celeste = recorrido real, verde = odometría, naranja = elipse del EKF (×3), azul = pose del EKF, magenta = goal actual. La vista de la corrida con EKF quedó girada unos 3° (se movió la cámara durante la grabación); no afecta los datos.
