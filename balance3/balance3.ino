/**********************************************************************
Run a balancing servo off a clock.
Get input from IMU:
 * Need to have "I2Cdev" folder in Arduino libraries
 * Need to have "MPU6050" folder in Arduino libraries
Estimate body angle
/**********************************************************************/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Pins.h"
#include "My_encoders.h"

/**********************************************************************/
 
#define MAX_ULONG 0xFFFFFFFF

// I had to increase this to 3ms to accomodate reading the IMU
#define SERVO_INTERVAL 3000 // microseconds

#define MAX_COMMAND 255

/**********************************************************************/

bool go = false;  /* motor enable */

// Encoder readings: these can be positive or negative.
long left_angle = 0;
long right_angle = 0;

MPU6050 accelgyro;

// Keep track of late control cycles. 
unsigned long servo_late = 0; // How many times am I late?
unsigned long max_servo_late = 0; // What was the worst one?

/**********************************************************************/
/**********************************************************************/
  double angle_m = 0;
  double gyro_m = 0;
  float Gyro_x, Gyro_y, Gyro_z;
  float accelz = 0;
  float angle;
  float angle6;
  float angle_err, q_bias;
  float Pdot[4] = { 0, 0, 0, 0};
  float P[2][2] = {{ 1, 0 }, { 0, 1 }};
  float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
  float angle_dot;
  float dt = 0.003, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
  double kp_balance = 55, kd_balance = 0.75;
  double kd_turn = 0.5;
  double angle_zero_actual = -2.5;         //x axle angle calibration
  double angle_tilt= 3;
  char sign = 1;
  int spin = 70;
  int count = 0;
  double angle_zero = angle_zero_actual;
  double angular_velocity_zero = 0; //x axle angular velocity calibration
  int balance_angle_min = -22;
  int balance_angle_max = 22;
  double bpm = 103; // Change this based on bpm of song, can also set directly using Serial read
  double msperbeat = 60000/bpm;

void motor_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void motor_stop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void motor_left_command( int speed )
{
    if ( speed >= 0 )
    {
      digitalWrite( AIN1, 1 );
      analogWrite( PWMA_LEFT, speed );
    }
  else
    {
      digitalWrite( AIN1, 0 );
      analogWrite( PWMA_LEFT, -speed );
    }
}

// reverses the sign of "speed"
void motor_right_command( int speed )
{
  if ( speed >= 0 )
    {
      digitalWrite( BIN1, 1 );
      analogWrite( PWMB_RIGHT, speed );
    }
  else
    {
      digitalWrite( BIN1, 0 );
      analogWrite( PWMB_RIGHT, -speed );
    }
}

/**********************************************************************/

void setup()
{

  Wire.begin();  // I2C (TWI) uses Wire library
 Serial.begin( 1000000 );  // run Serial fast to print out data
  Wire.setClock( 1000000UL ); // run I2C as fast as possible
  analogReference(INTERNAL1V1); // was INTERNAL on 328P, voltage

  motor_init(); // Initialize motors
  delay(1000);

  Encoder_init( &left_angle, &right_angle ); // Initialize (zero) encoders
  delay(1000); 

  accelgyro.initialize(); // Initialize IMU
  delay(1000); 
  
  // code starts disabled, so every time Arduino powers up nothing happens
  go = false; 
}

/******************************************************************/

/**
 * Elegoo inspired LQR + Kalman Filter 
 */
void Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0) {
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias;
}

// Take user input
void ProcessCommand()
{

  if ( Serial.available() <= 0 )
    return;

  int c = Serial.read();
  switch (c)
    {
      case 'S': case 's':
        Serial.println( "Stop!" );
        go = false;
        break;
      case 'G': case 'g':
      Serial.println( "Go!" );
        go = true;
        break;
      default:
        break;
    }
}

/**********************************************************************
Run a balancer with the given gains.
run time starts at the beginning and is a millisecond clock.
/**********************************************************************/

void run_balancer()
{
  // Clocks and time management
  // used to keep track of time intervals in servo
  unsigned long last_micros = micros();
  unsigned long last_millis = millis();
  int servo_time = 0; // Servo clock: needs to be signed, can be int
  // how long have we been running? Safety feature
  unsigned long run_time = 0; // milliseconds

  // keep track of voltage
  int voltage_raw = analogRead(VOL_MEASURE_PIN); //Read voltage value
  double voltage = (voltage_raw*1.1/1024)*((10+1.5)/1.5);
  int div = -1;
  // servo loop
  for( ; ; )
    {

/*****************************************
Timekeeping part */

      unsigned long current_micros = micros();
      if ( current_micros > last_micros )
        servo_time += current_micros - last_micros;
      else // rollover
        servo_time += (MAX_ULONG - last_micros) + current_micros;
      last_micros = current_micros;

      // It isn't time yet ...
      if ( servo_time < SERVO_INTERVAL )
        continue;

      // It is time! Reset the servo clock.
      servo_time -= SERVO_INTERVAL;

      // Keep track of maximum lateness
      if ( max_servo_late < servo_time )
        max_servo_late = servo_time;

      // Let's have some slop in counting late cycles.
      if ( servo_time > 50 )
        servo_late += 1;
    
      // handle the millisecond clocks
      unsigned long current_millis = millis();
      int millis_increment;
      if ( current_millis > last_millis )
        millis_increment = current_millis - last_millis;
      else // rollover
        millis_increment = (MAX_ULONG - last_millis) + current_millis;
      last_millis = current_millis;
  
      run_time += millis_increment;

/*****************************************
Administrative part */
      int body_angle;
        if ((angle < balance_angle_min || balance_angle_max < angle))
        {
         go = false;
         run_time = 0; 
         angle=0;
        }
        
        if(!go)
        {
          motor_stop();
          return;
        }
         if (((int) (run_time/msperbeat)) != div) {
          
           if(count%4 == 0){
            sign = 1;
           } else if(count%4==2){
            sign = -1;
           } else {
            sign = 0;
           }
           spin*=-1;
           count++;
           Serial.println((int)(run_time/msperbeat));
           div = (int)(run_time/msperbeat);
         }
        angle_zero = angle_zero_actual + sign * angle_tilt;


/*****************************************
State estimation part */


      // Read the sensors
      Read_encoders( &left_angle, &right_angle );
      // Current accelerometer and gyro zero values.
      int16_t ax, ay, az;
      int16_t gx, gy, gz;

      accelgyro.getAcceleration( &ax, &ay, &az );
      gx = accelgyro.getRotationX();
      gz = accelgyro.getRotationZ();

      // Elegoo Inspired Code
      float Angle = atan2(ay, az) * 57.2958; // angle in radians?
      Gyro_x = (gx - 128.1) / 131;
      Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);
      Gyro_z = -gz / 131;
      double balance_control_output = kp_balance * (angle - angle_zero) + kd_balance * (Gyro_x - angular_velocity_zero);
      double rotation_control_output = spin + kd_turn * Gyro_z;

/****************************************
*/  

      long command_long = 0;

      int commandLeft = (int) (balance_control_output - rotation_control_output);
      int commandRight = (int) (balance_control_output + rotation_control_output);

      if ( commandLeft > MAX_COMMAND )
        commandLeft = MAX_COMMAND;
      if ( commandLeft < -MAX_COMMAND )
        commandLeft = -MAX_COMMAND;
      if ( commandRight > MAX_COMMAND )
        commandRight = MAX_COMMAND;
      if ( commandRight < -MAX_COMMAND )
        commandRight = -MAX_COMMAND;

      motor_left_command( -commandLeft );
      motor_right_command( -commandRight );

    }
}

/**********************************************************************/

void loop()
{
  if ( !go )
    {
      motor_stop();
      delay(5000);
      go = true;
    }

  Encoder_init( &left_angle, &right_angle ); // set wheel encoders to zero
  run_balancer();
  go = false;
}
  
/**********************************************************************/
