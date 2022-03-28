#include <Wire.h>


#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW   0
#define PITCH 1
#define ROLL  2
#define THROTTLE 3

#define X     0     
#define Y     1     
#define Z     2    

#define MPU_ADDRESS 0x68  
#define FREQ        250   
#define SSF_GYRO    65.5  

#define STOPPED  0
#define STARTING 1
#define STARTED  2

#define Minmax_min 1000
#define Minmax_max 2000
//------------------------------------------------Recepteur variable-----------------------------------------------------------------

volatile unsigned long current_time;
volatile unsigned long timer[4];

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_duration[4] = {1500, 1500, 1000, 1500};

int mode_mapping[4];

//------------------------------------------------MPU variables----------------------------------------------------------------------

int gyro_raw[3] = {0, 0, 0};


long gyro_offset[3] = {0, 0, 0};


float gyro_angle[3]  = {0, 0, 0};


int acc_raw[3] = {0 , 0 , 0};

float acc_angle[3] = {0, 0, 0};

long acc_total_vector;

float angular_motions[3] = {0, 0, 0};

float measures[3] = {0, 0, 0};

int temperature;

boolean initialized;

//--------------------------------------------variable generation signal pwm------------------------------------
unsigned int  period;
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
        pulse_length_esc2 = 1000,
        pulse_length_esc3 = 1000,
        pulse_length_esc4 = 1000;
        
//------------------------------------------variable PID-------------------------------------------------------------
float pid_set_points[3] = {0, 0, 0};
float errors[3];
float error_sum[3]      = {0, 0, 0};
float delta_err[3]      = {0, 0, 0};
float previous_error[3] = {0, 0, 0};

float Kp[3] = {4.0, 1.3, 1.3};    
float Ki[3] = {0.02, 0.04, 0.04}; 
float Kd[3] = {0, 18, 18};


//----------------------------------------------------------------------------------------------------------------------

int status = STOPPED;

void setup() {

    Serial.begin(57600);
    Wire.begin();
    TWBR =12;
    
    DDRD |= B11110000;

    setupMpu6050Registers();

    configureChannelMapping();
  
    PCICR  |= (1 << PCIE0);  //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1); //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3); //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

    period = (1000000/FREQ) ;

    loop_timer = micros();
}

void loop() {

  //dumpChannels();

  // 1. First, read raw values from MPU-6050
    readSensor();

  // 2. Calculate angles from gyro & accelerometer's values
    calculateAngles();

  // 3. Calculate set points of PID controller
    calculateSetPoints();

  // 4. Calculate errors comparing angular motions to set points
    calculateErrors();

    if (isStarted()) {
  // 5. Calculate motors speed with PID controller
    pidController();    
    }
  // 6. Apply motors speed
    applyMotorSpeed();
}

/*void dumpChannels()
{
    for (int i = CHANNEL1; i <= CHANNEL4; i++) {
        Serial.print("Channel ");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(pulse_duration[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}*/

void configureChannelMapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

void setupMpu6050Registers() {
  // Configure power management
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
  Wire.write(0x00);                    // Apply the desired configuration to the register
  Wire.endTransmission();              // End the transmission
  
  // Configure the gyro's sensitivity
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1B);                    // Request the GYRO_CONFIG register
  Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
  Wire.endTransmission();              // End the transmission
  
  // Configure the acceleromter's sensitivity
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
  Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
  Wire.endTransmission();              // End the transmission
  
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1A);                    // Request the CONFIG register
  Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
  Wire.endTransmission();              // End the transmission
}


void calibrateMpu6050()
{
  int max_samples = 2000;

  for (int i = 0; i < max_samples; i++) {
    readSensor();

    gyro_offset[X] += gyro_raw[X];
    gyro_offset[Y] += gyro_raw[Y];
    gyro_offset[Z] += gyro_raw[Z];
    // Just wait a bit before next loop
    delay(3);
  }

  // Calculate average offsets
  gyro_offset[X] /= max_samples;
  gyro_offset[Y] /= max_samples;
  gyro_offset[Z] /= max_samples;
}

void readSensor() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);                    
  Wire.endTransmission();              
  Wire.requestFrom(MPU_ADDRESS, 14);   

  
  while (Wire.available() < 14);

  acc_raw[X]  = Wire.read() << 8 | Wire.read(); 
  acc_raw[Y]  = Wire.read() << 8 | Wire.read(); 
  acc_raw[Z]  = Wire.read() << 8 | Wire.read(); 
  temperature = Wire.read() << 8 | Wire.read(); 
  gyro_raw[X] = Wire.read() << 8 | Wire.read(); 
  gyro_raw[Y] = Wire.read() << 8 | Wire.read(); 
  gyro_raw[Z] = Wire.read() << 8 | Wire.read(); 
}

void calculateAngles()
{
  calculateGyroAngles();
  calculateAccelerometerAngles();

  if (initialized) {
    
    gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
    gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
  } else {
    
    resetGyroAngles();

    initialized = true;
  }

  
  measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
  measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
  measures[YAW]   = -gyro_raw[Z] / SSF_GYRO;


  angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
  angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
  angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
}

void calculateGyroAngles()
{
  // Subtract offsets
  gyro_raw[X] -= gyro_offset[X];
  gyro_raw[Y] -= gyro_offset[Y];
  gyro_raw[Z] -= gyro_offset[Z];


  gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
  gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO));

  // Transfer roll to pitch if IMU has yawed
  gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
  gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}


void calculateAccelerometerAngles()
{
 
  acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));


  if (abs(acc_raw[X]) < acc_total_vector) {
    acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); 
  }

  if (abs(acc_raw[Y]) < acc_total_vector) {
    acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
  }
}

void calculateSetPoints() {
    pid_set_points[YAW]   = calculateYawSetPoint(pulse_duration[mode_mapping[YAW]], pulse_duration[mode_mapping[THROTTLE]]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_duration[mode_mapping[PITCH]]);
    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], pulse_duration[mode_mapping[ROLL]]);
}

/**
 * Calculate the PID set point in °/s
 *
 * @param float angle         Measured angle (in °) on an axis
 * @param int   channel_pulse Pulse length of the corresponding receiver channel
 * @return float
 */
float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float set_point    = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of YAW axis in °/s
 *
 * @param int yaw_pulse      Receiver pulse length of yaw's channel
 * @param int throttle_pulse Receiver pulse length of throttle's channel
 * @return float
 */
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    float set_point = 0;

    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }

    return set_point;
}

void calculateErrors() {
    // Calculate current errors
    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];

    // Calculate sum of errors : Integral coefficients
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Keep values in acceptable range
    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

    // Calculate error delta : Derivative coefficients
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}

float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

void pidController() {
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = pulse_duration[mode_mapping[THROTTLE]];

    // Initialize motor commands with throttle
    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;

    // Do not calculate anything if throttle is 0
    if (throttle >= 1012) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
        yaw_pid   = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid  = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
        //afficahge des valeurs pour le debugage, mettre en commentaire si innutilisé
        Serial.print("TH :");
        Serial.print(throttle);
        Serial.print("\tRO :");
        Serial.print(roll_pid);
        Serial.print("\tPI :");
        Serial.print(pitch_pid);
        Serial.print("\tYA :");
        Serial.print(yaw_pid);
        Serial.print("\t == PULSE :");
        Serial.println(pulse_length_esc1);
    }

    // Prevent out-of-range-values
    pulse_length_esc1 = minMax(pulse_length_esc1, Minmax_min, Minmax_max);
    pulse_length_esc2 = minMax(pulse_length_esc2, Minmax_min, Minmax_max);
    pulse_length_esc3 = minMax(pulse_length_esc3, Minmax_min, Minmax_max);
    pulse_length_esc4 = minMax(pulse_length_esc4, Minmax_min, Minmax_max);
}

bool isStarted() {
    // When left stick is moved in the bottom left corner
    if (status == STOPPED && pulse_duration[mode_mapping[YAW]] <= 1012 && pulse_duration[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTING;
    }

    // When left stick is moved back in the center position
    if (status == STARTING && pulse_duration[mode_mapping[YAW]] == 1500 && pulse_duration[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTED;

        // Reset PID controller's variables to prevent bump start
        resetPidController();

        resetGyroAngles();
    }

    // When left stick is moved in the bottom right corner
    if (status == STARTED && pulse_duration[mode_mapping[YAW]] >= 1988 && pulse_duration[mode_mapping[THROTTLE]] <= 1012) {
        status = STOPPED;
        // Make sure to always stop motors when status is STOPPED
        stopAll();
    }

    return status == STARTED;
}

void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

void stopAll() {
    pulse_length_esc1 = 1000;
    pulse_length_esc2 = 1000;
    pulse_length_esc3 = 1000;
    pulse_length_esc4 = 1000;
}

void resetPidController() {
    errors[YAW]   = 0;
    errors[PITCH] = 0;
    errors[ROLL]  = 0;

    error_sum[YAW]   = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL]  = 0;

    previous_error[YAW]   = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL]  = 0;
}

void applyMotorSpeed() {
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    while ((now = micros()) - loop_timer < period);

    // Update loop timer
    loop_timer = now;

    // Set pins #4 #5 #6 #7 HIGH
    PORTD |= B11110000;

    // Wait until all pins #4 #5 #6 #7 are LOW
    while (PORTD >= 16) {
        now        = micros();
        difference = now - loop_timer;

        if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
        if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
        if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
        if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
    }
}

ISR(PCINT0_vect)
{
    current_time = micros();

    // Channel 1 -------------------------------------------------
    if (PINB & B00000001) {                                        // Is input 8 high ?
        if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;                       // Save current state
            timer[CHANNEL1]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL1] == HIGH) {                  // Input 8 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                            // Save current state
        pulse_duration[CHANNEL1] = current_time - timer[CHANNEL1]; // Stop timer & calculate pulse duration
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00000010) {                                        // Is input 9 high ?
        if (previous_state[CHANNEL2] == LOW) {                     // Input 9 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH;                       // Save current state
            timer[CHANNEL2]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL2] == HIGH) {                  // Input 9 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;                            // Save current state
        pulse_duration[CHANNEL2] = current_time - timer[CHANNEL2]; // Stop timer & calculate pulse duration
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100) {                                        // Is input 10 high ?
        if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;                       // Save current state
            timer[CHANNEL3]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL3] == HIGH) {                  // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                            // Save current state
        pulse_duration[CHANNEL3] = current_time - timer[CHANNEL3]; // Stop timer & calculate pulse duration
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B00001000) {                                        // Is input 11 high ?
        if (previous_state[CHANNEL4] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH;                       // Save current state
            timer[CHANNEL4]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL4] == HIGH) {                  // Input 11 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;                            // Save current state
        pulse_duration[CHANNEL4] = current_time - timer[CHANNEL4]; // Stop timer & calculate pulse duration
    }
}
