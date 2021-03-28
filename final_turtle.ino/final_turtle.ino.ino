/********************************* TASK 1 *********************************/
// On your breadboard, connect the pins listed below to the DRV8833 input pins
#define M1IN1       12
#define M1IN2       11
#define M2IN1       10
#define M2IN2       2

#define ULT_SEN     9
#define VELOCITY    ( 330 / 10000.0 )   // The ultrasonic velocity (cm/us) in air

float tmpPos = 0.0;
float finPos = 0.0;
float valSum = 0.0;
float tmpVal = 0;
int pinArr[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
float avArr[8];

byte noLine = 0;
float threshold_left = -100.0;
float threshold_right = 200.0;

/********************************* TASK 2 *********************************/
// Refer to page 9 of the DRV8833 datasheet, complete this function where
// power        is an integer ranging from -255 to 255, with the sign 
//              of the value indicates the motor spin direction, and the
//              magnitude of the value indicating the motor power
// in1 and in2  is the arduino pin number of the pins connected to the 
//              DRV8833 IN1 and IN2 input pins
// You may use the analogWrite() function
void motorDrive(int power, int in1, int in2) {
  if (power >= 0) {
    analogWrite(in2, 0);
    analogWrite(in1, power);
    } else {
    analogWrite(in1, 0);
    analogWrite(in2, abs(power));
    }
}

void turn_right() {
  motorDrive(75, M1IN1, M1IN2);
  motorDrive(50, M2IN1, M2IN2);
}

void turn_left() {
  motorDrive(50, M1IN1, M1IN2);
  motorDrive(75, M2IN1, M2IN2);
}

void setup() {
    /********************************* TASK 3 *********************************/
    // Using the motorDrive function, set the 2 motors to spin full power in one
    // direction for 10 seconds, then reverse direction at half power for 10 seconds

    // You may notice the motors does not spin up. Troubleshoot the issue by reading 
    // the DRV8833 datasheet, you might need to make additional connections on your 
    // breadboard.

    Serial.begin(9600);
    
//    motorDrive(255, M1IN1, M1IN2);
//    motorDrive(255, M2IN1, M2IN2);
//    delay(10000);
//    motorDrive(-127, M1IN1, M1IN2);
//     motorDrive(-127, M2IN1, M2IN2);
//    delay(10000);
}

void loop() {
    /********************************* TASK 4 *********************************/
    // Program the arduino to control power to the motors according to finger position 
    // relative to the line sensor, set the power to -255 when the finger covers
    // the left most reflectance sensor and set the power to 255 when the finger covers
    // the right most reflectance sensor.    
  
   Serial.print(ultRead(ULT_SEN));
   Serial.print("\t");
  
  noLine = 1;
  tmpPos = 0.0;
  finPos = 0.0;
  valSum = 0.0;
  for (int i=0; i<8; i++) {
    tmpVal = 1024 - analogRead(pinArr[i]);
    tmpPos += tmpVal * i;
    if (tmpVal > 20) {
      noLine = 0;
      }
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" data: ");
    Serial.print("\t");
    Serial.print(tmpVal);
    Serial.print("\t");
    valSum += tmpVal;
    }

    tmpPos = tmpPos/valSum;
    tmpPos = tmpPos/8.0*512 - 210;

    for (int j=0; j<7; j++) {
      avArr[j] = avArr[j+1];
      }
    avArr[7] = tmpPos;
    
    for (int k=0; k<8; k++) {
      finPos += avArr[k];
      } 

    finPos = finPos/8.0;

    if (noLine) {
      Serial.println("No line detected");
      motorDrive(0, M1IN1, M1IN2);
      motorDrive(0, M2IN1, M2IN2);
      } else {
      Serial.print("Line Position: ");
      Serial.print(finPos);
      Serial.print("\t");
    
      motorDrive(75, M1IN1, M1IN2);
      motorDrive(75, M2IN1, M2IN2);

      if (finPos < threshold_left) {
        Serial.println("Turn left");
        turn_left();
      } else if (finPos > threshold_right) {
        Serial.println("Turn right");
        turn_right();
      } else {
        Serial.println();
      }  
      }
    
        
    
}

volatile byte trigState = 0;
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
float ultRead(int pin) {
    // Set the sensor pin high for 10us and then low
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);

    // Set the sensor pin as an input
    pinMode(pin, INPUT);

    // Setup an interrupt to trigger on a changing edge
    attachInterrupt(pin, ultRiseRoutine, CHANGE);

    // Wait for both the rising and falling edge to occur
    while(trigState != 2) {
        delayMicroseconds(1);
//    Serial.println(trigState);
    }

//    Serial.println("acq");

    // Detach the interrupt pin
    trigState = 0;
    detachInterrupt(pin);
    pinMode(pin, OUTPUT);
     
    // Return the measured distance in cm using the VELOCITY constant and time difference
    // Make sure you handle the possibility of the micros() counter overflow between the 
    // rising and falling edge
    if (endTime < startTime) {
       return VELOCITY * ((pow(2,32) - 1 - startTime) + endTime);
      } else {
       return VELOCITY * (endTime - startTime);
      }

}

/********************************* TASK 3.5 *******************************/
// Record the time when the rising and falling edge occurs
void ultRiseRoutine() {
    if (trigState == 0) {  // rising edge
        startTime = micros();
        trigState = 1;
    } else {
        endTime = micros();
        trigState = 2;
    }
}
