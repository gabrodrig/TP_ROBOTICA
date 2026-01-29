#define SIMULACION 1
#define EJERCICIO 1

#if !SIMULACION
#include <NewPing.h>
#include <Encoder.h>
#include <Wire.h>
#include <PID_v1.h>
#include <SharpIR.h>
#include <TimerOne.h>
#endif

/****************************** algunas definiciones globales ************/

#define IR_FRONT 0
#define IR_LEFT  1
#define IR_RIGHT 2

#define MIN_DIST_SIDE 0.15
#define MIN_DIST_FRONT 0.10

#define SPEED_SLOW 0.85
#define SPEED_NORMAL 1.5
#define SPEED_FAST 2

/****************************** Setup *********************************/
void setup()
{
  
  /* inicializacion de sensores */
  motors_init();
  ir_init();
  bumper_init();

  
  #if SIMULACION 
    Serial.begin(9600);
  #else
	  pid_init();
	  Serial.begin(115200);
  #endif
  /* inicializacion del comportamiento */
  comportamiento_init();
}

/****************************** Loop principal ************************/
void loop()
{
  /* procesamiento de sensores */
  ir_loop();
  bumper_loop();
  #if !SIMULACION 
	sonar_loop();
	pid_loop();
	serial_loop();
  #endif
  
  #if EJERCICIO == 5
    sonar_loop();
  #endif
  /* procesamiento del comportamiento */
  comportamiento_loop();
}

/***************************** Comportamientos ************************/
/*
enum Estado { eAvanzar = 0, eRetroceder, eGirar };

Estado estado;

uint16_t g_timeStampMs;

void comportamiento_init()
{
  / /define estado inicial y ejecuta accion de la trancision inicial, segun ejercicio 
  estado = eAvanzar; 
  pid_set_speed(SPEED_NORMAL, SPEED_NORMAL);
  //... 
  
}

bool IsBumperLeftOn() { return bumper_left(); }
bool IsBumperRightOn() { return bumper_right(); }

void comportamiento_loop()
{
  // Proceso estados, chequeo condiciones y ejecuto transiciones
  switch (estado)
  {
      case eAvanzar: 
        {
          if (IsBumperLeftOn() || IsBumperRightOn())
          {
            pid_set_speed(-SPEED_NORMAL, -SPEED_NORMAL);
            g_timeStampMs = millis();
            estado = eRetroceder; 
          } 
        } break;
      case eRetroceder:
        {
      if ((millis() - g_timeStampMs) > 3000)
            {
                pid_set_speed(SPEED_NORMAL, -SPEED_NORMAL);
                g_timeStampMs = millis();
              estado = eGirar; 
            }              
        } break;
      case eGirar:
        {
      if ((millis() - g_timeStampMs) > 3000)
            {
                pid_set_speed(SPEED_NORMAL, SPEED_NORMAL);
              estado = eAvanzar; 
            }              
        } break;    
  }
  //...
}
*/

/***************************** FIN COMPLETAR ***************************/


/***************************** SOLO COMPLETAR AQUI ************************/


#if EJERCICIO == 1
enum Estado {AVANZAR};
#elif EJERCICIO == 2
#elif EJERCICIO == 3
#elif EJERCICIO == 4
#elif EJERCICIO == 5

#endif


Estado estado;

long t;

void comportamiento_init()
{
    
    estado = AVANZAR;
	  pid_set_speed(SPEED_NORMAL,SPEED_NORMAL);
  	t = millis();
  	#if EJERCICIO == 5
    #if SIMULACION
  	  sonar_set_up();
  	#endif
    #endif
    // define estado inicial y ejecuta accion de la trancision inicial, segun ejercicio 
    //estado = ESTADO_INICIAL; 
  
    //... 
  
}

void comportamiento_loop()
{
  // Proceso estados, chequeo condiciones y ejecuto transiciones
	
  	#if EJERCICIO == 1
  	
  	#elif EJERCICIO == 2
  	
  
  	#elif EJERCICIO == 3
  	
  
  	#elif EJERCICIO == 4
  	
  	#elif EJERCICIO == 5
  	
  	#endif
	
}

/***************************** FIN COMPLETAR ***************************/

/*************************** PID *******************************/

#if SIMULACION
	#define motLeftA  10
	#define motLeftB  11
	#define motRightA  5
	#define motRightB  6
#else
// Motores
	#define motLeftA  7
	#define motLeftB  6
	#define motRightA 5
	#define motRightB 4

	//Encoders
	#define encRightA 18
	#define encRightB 19
	#define encLeftA  20
	#define encLeftB  21 

	/* PID constantes */
	#define control_loop_time  10
	int last_control_loop_time;
	int pMotRight = 32;
	int iMotRight = 16;
	int dMotRight = 0;

	int pMotLeft  = 32;
	int iMotLeft  = 16;
	int dMotLeft  = 0;

  #define  pwm_min_right 75
  #define  pwm_min_left 75
	#define  pwm_max 255


/* status variables */
struct pid_status {
  long prevPos;
  long posDelta;  
  double input, output, setpoint;
  bool enable;
};

/* PID seting */
pid_status pid_mot_right_status, pid_mot_left_status; 
PID pid_mot_right(&pid_mot_right_status.input, &pid_mot_right_status.output, &pid_mot_right_status.setpoint, pMotRight, iMotRight, dMotRight, DIRECT);
PID pid_mot_left(&pid_mot_left_status.input, &pid_mot_left_status.output, &pid_mot_left_status.setpoint, pMotLeft, iMotLeft, dMotLeft, DIRECT);


/* Encoders seting */
Encoder enc_right(encRightA, encRightB);
Encoder enc_left(encLeftA, encLeftB);

void pid_init()
{
  pid_mot_left_status.setpoint = pid_mot_right_status.setpoint = 0;
  last_control_loop_time = millis();  
  
  pid_mot_right.SetMode(AUTOMATIC);
  pid_mot_right.SetSampleTime(control_loop_time);
  pid_mot_left.SetMode(AUTOMATIC);
  pid_mot_left.SetSampleTime(control_loop_time);
  
  pid_mot_left.SetOutputLimits(-(pwm_max - pwm_min_left), (pwm_max - pwm_min_left));
  pid_mot_right.SetOutputLimits(-(pwm_max - pwm_min_right), (pwm_max - pwm_min_right));
  
  pid_mot_right_status.enable = true;
  pid_mot_left_status.enable = true;
}

void pid_loop(){
  int time_now = millis();
  int delta_t = time_now  - last_control_loop_time;
  if (delta_t >= control_loop_time)
  {
    last_control_loop_time = time_now;

    // update pos y vel right 
    long posRight = -enc_right.read();
    pid_mot_right_status.posDelta = posRight - pid_mot_right_status.prevPos;
    pid_mot_right_status.prevPos = posRight;    
    float vel_right = (double)pid_mot_right_status.posDelta / (double)delta_t; // cuenta encoder por ms
    
    // update pos y vel Left 
    long posLeft = enc_left.read();
    pid_mot_left_status.posDelta = posLeft - pid_mot_left_status.prevPos;
    pid_mot_left_status.prevPos = posLeft; 
    float vel_left = (double)pid_mot_left_status.posDelta / (double)delta_t; // cuenta encoder por ms   

    pid_mot_right_status.input = vel_right; // cuenta encoder por ms
    pid_mot_right.Compute();
    pid_mot_left_status.input  = vel_left;  // cuenta encoder por ms
    pid_mot_left.Compute();
    
    int pwm_left = (pid_mot_left_status.output < 0 ? -1 : 1) * ((int)abs(pid_mot_left_status.output) + pwm_min_left);
    set_motor_left(pwm_left);

    int pwm_right = (pid_mot_right_status.output < 0 ? -1 : 1) * ((int)abs(pid_mot_right_status.output) + pwm_min_right);
    set_motor_right(pwm_right);
    Serial.print("L: vel: "); Serial.print(vel_left); Serial.print(" pwm: "); Serial.print(pwm_left);Serial.print("R: vel: "); Serial.print(vel_right); Serial.print(" pwm: "); Serial.println(pwm_right);
    /*Serial.print("L: vel: "); Serial.print(vel_left); Serial.print(" pwm: "); Serial.print(pwm_left); Serial.print(" t: "); Serial.print(time_now);Serial.print(" delta_t: "); Serial.println(delta_t);
    Serial.print("R: vel: "); Serial.print(vel_right); Serial.print(" pwm: "); Serial.print(pwm_right); Serial.print(" t: "); Serial.print(time_now);Serial.print(" delta_t: "); Serial.println(delta_t);*/
  }  
}
#endif
void pid_set_speed(float left, float right)
{
  pid_set_left(left);
  pid_set_right(right);

}

void pid_set_left(float v)
{
  #if SIMULACION
    v = mapFloat(v, 0, 2.0, 0, 128);
    set_motor_left(v);
  #else
    pid_mot_left.Reset();    /* reseteo para que el I se resetee ante cambios de consigna (sino quedaria el error viejo) */
    pid_mot_left_status.setpoint = v;
  #endif
}

void pid_set_right(float v)
{
  #if SIMULACION	
    v = mapFloat(v, 0, 2.0, 0, 128);
    set_motor_right(v);
  #else
    pid_mot_right.Reset();
    pid_mot_right_status.setpoint = v;
  #endif
}



void set_speed(float left, float right)
{
  set_motor_left(left);
  set_motor_right(right);
}

void set_motor_left(int speed)
{
  analogWrite(motLeftB, speed > 0 ? speed : 0);
  analogWrite(motLeftA, speed < 0 ? -speed : 0);
}


void set_motor_right(int speed)
{
  analogWrite(motRightB, speed > 0 ? speed : 0);
  analogWrite(motRightA, speed < 0 ? -speed : 0);
}

void motors_init(void)
{
  // inicializo los pines de los motores
  pinMode(motLeftA, OUTPUT);      // inicializo el pin A del motorIzq como una salida
  pinMode(motLeftB, OUTPUT);      // inicializo el pin B del motorIzq como una salida
  pinMode(motRightA, OUTPUT);      // inicializo el pin A del motorDer como una salida
  pinMode(motRightB, OUTPUT);      // inicializo el pin B del motorDer como una salida
  // seteo en 0 la velocidad de los motores
  analogWrite(motLeftA,0);        
  analogWrite(motLeftB,0);
  analogWrite(motRightA,0);        
  analogWrite(motRightB,0);
}

/******************** bumper ******************/

//Bumper
#if SIMULACION
  #define bumRight 2
  #define bumLeft 3
#else 
  #define bumRight 15
  #define bumLeft 14
#endif

bool bumper_state[2] = { false, false };

void bumper_init(void)
{
  pinMode(bumLeft, INPUT_PULLUP);           // inicializo el pin pulsador como una entrada
  pinMode(bumRight, INPUT_PULLUP);           // inicializo el pin pulsador como una entrada
}


void bumper_loop(void)
{
  bumper_state[0] = !digitalRead(bumLeft);
  bumper_state[1] = !digitalRead(bumRight);
  //Serial.print("BL: "); Serial.print(bumper_state[0]); Serial.print(" BR: "); Serial.println(bumper_state[1]);
}

bool bumper_left(void)
{
  return bumper_state[0];
}

bool bumper_right(void)
{
  return bumper_state[1];
}

/************** serial ******************/

void serial_loop(void)
{
  
}

/*************** sonar ********************/
float sonar_dist = -1;
long last_sonar_read = -1;

#if SIMULACION
  /* Completar la simulacion del sonar */
  # define sonarTrigg 7
  # define sonarEcho 4
  #define SONAR_READ_TIME 500
  #define SONAR_MIN_PARED 50
  #define SONAR_MAX_PARED 100
  
  void sonar_set_up(){
	pinMode(sonarTrigg, OUTPUT);
    pinMode(sonarEcho, INPUT);
    digitalWrite(sonarTrigg, LOW);
	last_sonar_read = 0;
  }

  void sonar_loop(){
    
    long t = millis();
    if (last_sonar_read == -1 || (t - last_sonar_read) > SONAR_READ_TIME){
      last_sonar_read = t;
      digitalWrite(sonarTrigg, HIGH);
      delayMicroseconds(10);
      digitalWrite(sonarTrigg, LOW);
      
      long t_sonar = pulseIn(sonarEcho, HIGH);
      sonar_dist = (0.0373 * t_sonar)/2;
      sonar_dist = (sonar_dist > SONAR_MAX_PARED*2) ? SONAR_MAX_PARED*2 : sonar_dist;
    }
    
  }

  /* Fin completar  */
#else
  #define sonarTrigg 3
  #define sonarEcho 2
  #define SONAR_READ_TIME 250
  #define SONAR_MIN_PARED 15
  #define SONAR_MAX_PARED 30

  NewPing sonar(sonarTrigg, sonarEcho, SONAR_MAX_PARED*2);
  void sonar_loop(void)
  {
    long t = millis();
    if (last_sonar_read == -1 || (t - last_sonar_read) > SONAR_READ_TIME)
    {
      last_sonar_read = t;
      sonar.ping_timer(on_sonar_echo);
    }
  }

  void on_sonar_echo(void)
  {
    if (sonar.check_timer())
      sonar_dist = sonar.convert_cm(sonar.ping_result);
    else
      sonar_dist = SONAR_MAX_PARED*2;
  }

#endif

float sonar_distance(void)
{
  return sonar_dist;
}

bool far_from_wall(){
  return sonar_dist != -1 && sonar_dist >= SONAR_MAX_PARED;
}

bool too_close_to_wall(){
  return sonar_dist != -1 && sonar_dist <= SONAR_MIN_PARED;
}

/************ infrarojos ********************/
#define IR_COUNT 3

#if SIMULACION
  #define IR_MAX_DIST 5000

  #define PIN_IR0 A1
  #define PIN_IR1 A0
  #define PIN_IR2 A2
#else<
  #define IR_NUM_READINGS 5
  #define IR_ERROR_PERCENT 93
  #define IR_MODEL 1 // GP2Y0A21Y

  #define IR_MAX_DIST 0.8

  #define PIN_IR0 A10
  #define PIN_IR1 A9
 #define PIN_IR2 A8


  SharpIR ir_0(IR_MODEL, PIN_IR0);
  SharpIR ir_1(IR_MODEL, PIN_IR1);
  SharpIR ir_2(IR_MODEL, PIN_IR2);

  SharpIR ir[IR_COUNT] =
  {
    ir_0,
    ir_1,
    ir_2
  }; 

  float readings_front [10] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
  float readings_left [10] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
  float readings_right [10] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
  int ir_reading_it = 0;
#endif

float ir_distances[IR_COUNT];

void ir_init(void)
{
  for (int i = 0; i < IR_COUNT; i++)
    ir_distances[i] = -1;
}

float average_ir_reading(float fs[]){
  float sum = 0.0;
  for(int i = 0; i<10; i++){
    sum += fs[i];
  }
  return sum / 10;
}

void ir_loop(void)
{
  #if SIMULACION
    ir_distances[0] = mapFloat(analogRead(PIN_IR0), 344.0, 1017.0, 0.06, 0.8); 
    ir_distances[1] = mapFloat(analogRead(PIN_IR1), 344.0, 1017.0, 0.06, 0.8); 
    ir_distances[2] = mapFloat(analogRead(PIN_IR2), 344.0, 1017.0, 0.06, 0.8); 

    //Serial.print("IR0: "); Serial.print(ir_distances[0]);
    //Serial.print(" IR1: "); Serial.print(ir_distances[1]);
    //Serial.print(" IR2: "); Serial.println(ir_distances[2]);
  #else
	for (int i = 0; i < IR_COUNT; i++)
	{
	ir_distances[i] = fabs(ir[i].getDistance(true) / 100.0f); 
	if (ir_distances[i] > IR_MAX_DIST) ir_distances[i] = IR_MAX_DIST; 
	}

  if (ir_reading_it >= 10){
    ir_reading_it = 0;
  }
  readings_front[ir_reading_it] = ir_distances[0];
  readings_left[ir_reading_it] = ir_distances[1];
  readings_right[ir_reading_it] = ir_distances[2];
  ir_reading_it++;

  ir_distances[0] = average_ir_reading(readings_front); 
  ir_distances[1] = average_ir_reading(readings_left); 
  ir_distances[2] = average_ir_reading(readings_right); 

  #endif
}

float ir_distance(unsigned int i)
{
  if (i >= IR_COUNT) return -1;
  else
  {
    return ir_distances[i];    
  }
}

#if SIMULACION
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
#endif
