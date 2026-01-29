# TP_ROBOTICA

Muchachos, lo que hice fue la parte de cinematica inversa y cinematica directa con la odometria
Llegue a testear solo la cinematica inversa, osea ver que cuando le mando comandos al robot, se mueve correctamente
No llegue a ver lo de rviz, que es donde se veria la odometria. Tampoco mire bien la consigna del 1 a ver si falta algo

Pero bueno cuestion, agarre el paquete modelo_diferencial, le cambie todos los nombres necesarios de diferencial/pioneer a omnidiferencial y trabaje sobre omni_odometry.cpp, que era pioneer_odometry.cpp. Ahi pueden comparar ambos codigos para ver bien los cambios. 

Use el paper que nos dieron con la consigna del tp, y la intro del tp. 
- Cambian las variables globales
- En vez de usar EncodersTicks, se usa MultiEncoderTicks, asi que intercambie todas las veces q aparecia la primera
- En vez de haber 2 publishers para 2 ruedas, hay 4 publishers para 4 ruedas
- En on_velocity_cmd, uso la ecuación 19 del paper (cinematica inversa) y publico los mensajes a las 4 ruedas
- En on_encoder_ticks es mas de lo mismo, mas o menos la misma estructura que antes pero ahora uso la ecuacion 21 del paper para cinematica directa. Las ecuaciones de delta_x, delta_y y delta_theta salen de la clase de cinematica (creo q era la 5 o 6), es igual q antes solo que en el diferencial, la parte de y se anulaba porque era 0
- Despues para poder testear el robot agarre el primer taller de ros2, q nos hacian implementar lo del keyboard para poder manejarlo y lo modifique para que sirva con el omnidireccional. Lo cambie para q en vez de las flechitas use awsd y ademas le agregue q y e. s y w para moverse en x, a y d para moverse en y, q y e para rotar. Si tocas una tecla del eje x y otra del eje y, te moves en diagonal

Salu2, me voy a la playa
(cualquier cosa preguntenme, sino se acuerdan como cargar los mensajes antes de iniciar los modulos y esas mamadas o lo q sea)
(en todo hay un .txt que se llama comandos coppelia, ahi esta la secuencia de comandos para agregar los mensajes tb)