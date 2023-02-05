
/*---------------------------------------------------------- KNIZNICE -------------------------------------------------------*/

#include <Servo.h>          // Include Servo Library
#include <NewPing.h>        // Include Newping Library
#include <TimerFreeTone.h>  // Include TONE library


/*---------------------------------------------------------- BUZZER -----------------------------------------------------------*/

#define buzzer 13 //buzzer to arduino pin 13

/*---------------------------------------------------------- RELAYS -----------------------------------------------------------*/
//RELAYS

#define relay_left_up A2
#define relay_right_up A1
#define relay_left_down 12
#define relay_right_down A0

/*----------------------------------------------------- DISTANCE SENSORS ----------------------------------------------------*/

//DISTANCE SENSOR

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
int distance;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(A5, 2, MAX_DISTANCE), // Dolný senzor 2
  NewPing(A4, A3, MAX_DISTANCE) // Zadný senzor 3
};

/*---------------------------------------------------------- PWM ------------------------------------------------------------*/
//PWM SPEED CONTROL

int motorPin0 = 5; // pin to connect to motor module
int motorPin1 = 6; // pin to connect to motor module
int motorPin2 = 3; // pin to connect to motor module
int mSpeed = 0;// variable to hold speed value
int priemer;

/*--------------------------------------------------------- SERVO -----------------------------------------------------------*/
//SERVO

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

/*------------------------------------------------------- STORE TIME -----------------------------------------------------------*/

const unsigned long period = 1000;
unsigned long startMillis;
unsigned long currentMillis;



/*--------------------------------------------------------- SETUP -----------------------------------------------------------*/

void setup()
{
  pinMode(buzzer, OUTPUT); // set Buzzer as output

  myservo1.attach(0);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(1);  // attaches the servo on pin 9 to the servo object

  pinMode(11, INPUT); //INFRARED SENSOR

  pinMode(motorPin0, OUTPUT); // set mtorPin as output
  pinMode(motorPin1, OUTPUT); // set mtorPin as output
  pinMode(motorPin2, OUTPUT); // set mtorPin as output

  pinMode(A2, OUTPUT); //Relay 1 - u
  pinMode(A1, OUTPUT); //Relay 2 - u
  pinMode(A0, OUTPUT); //Relay 3 - d
  pinMode(12, OUTPUT); //Relay 4 - d

  myservo1.write(100);   // Set at 100 degrees.
  delay(2000);              // Wait for 2s.
  int distance = readPing1();    // Get Ping Distance.
  delay(100);               // Wait for 100ms.

  analogWrite(motorPin0, 0); // send mSpeed value to motor
  analogWrite(motorPin1, 0); // send mSpeed value to motor
  analogWrite(motorPin2, 0); // send mSpeed value to motor
}

/*---------------------------------------------------------- LOOP -------------------------------------------------------*/
void loop()
{
  int distanceRight = 0;  //Initialize right side distance
  int distanceLeft = 0;   //Initialize left side distance
  int distanceBack = 0;   //Initialize left side distance
  currentMillis = millis();
  if (currentMillis -  startMillis >= period)
  {
    myservo1.write(100);    //make servo position 90 degree
    makeboth_move();
  }
  analogWrite(motorPin0, 200); // send mSpeed value to motor

 if (distance <= 20 || digitalRead(11) == LOW)
  {
    moveBackward();
    TimerFreeTone(buzzer, 5000, 200);
    Stop();   //call stop function to stop the robot
    TimerFreeTone(buzzer, 5000, 100);
    TimerFreeTone(buzzer, 8000, 200);
    TimerFreeTone(buzzer, 1500, 100);
    distanceRight = lookRight();    //call lookRight function to save distance in distanceRight variable
    delay(500);   //wait for 500ms
    distanceLeft = lookLeft();    //call lookLeft function to save distance in distanceLeft variable
    delay(500);   //wait for 500ms
    if (distanceRight <= 30 && distanceLeft <= 30)
    {
      TimerFreeTone(buzzer, 8000, 100);
      TimerFreeTone(buzzer, 900, 200);
      TimerFreeTone(buzzer, 8000, 100);
      TimerFreeTone(buzzer, 900, 200);
      do
      {
         analogWrite(motorPin0, 0); // send mSpeed value to motor
         distanceBack = lookBackward();
         moveBackward(); 
         if(distanceBack < 32)
         {
          moveForward(); 
         }
      }
      while(distanceBack > 30);
      
    }

    if (distanceRight > distanceLeft)
    {
      turnRight();    //call function  to turn right
      delay(300);   //wait for 300ms
      Stop();   //call stop function to stop robot
      startMillis = millis();
    }
    else
    {
      turnLeft();   //call function to turn left
      delay(300);   //wait for 300ms
      Stop();   //call stop function to stop robot
      startMillis = millis();
    }
    }

    else
    {
      moveForward();    //call moveForward function to move robot in forward direction
    }

    distance = readPing1();    //call readPing function to calculate Distance

  }


  /*---------------------------------------------------------- LOOK RIGHT -------------------------------------------------------*/

  int lookRight()     // lookRight Function for Servo Motor
  {
    myservo1.write(160);   //make servo position at 0 degree
    delay(700);   //wait for 500ms
    int distance = readPing1();    //read distance
    delay(200);   //wait for 100ms
    return distance;    //return distance whenever lookRight function is called
  }

  /*---------------------------------------------------------- LOOK LEFT -------------------------------------------------------*/

  int lookLeft()      // lookLeft Function for Servo Motor
  {
    myservo1.write(40);   //make servo position at 0 degree
    delay(700);   //wait for 500ms
    int distance = readPing1();    //read distance
    delay(200);   //wait for 100ms
    return distance;    //return distance whenever lookLeft function is called
  }
  /*---------------------------------------------------------- LOOK BACKWARD -----------------------------------------------------*/

  int lookBackward()      // lookLeft Function for Servo Motor
  {
    int distance = readPing2();    //read distance
    delay(200);   //wait for 100ms
    return distance;    //return distance whenever lookLeft function is called
  }


  /*---------------------------------------------------------- PING DISTANCE  -----------------------------------------------------*/

  int readPing1()      // readPing Function for Ultrasonic Sensor SPODNY.
  {
    delay(100);                 // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    int cm = sonar[0].ping_cm();   //Send ping, get ping distance in centimeters (cm).
    if (cm == 0)
    {
      cm = 250;
    }
    return cm;    //return distance whenever readPing function is called
  }


  int readPing2()      // readPing Function for Ultrasonic Sensor HORNY.
  {
    delay(100);                 // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    int cm = sonar[1].ping_cm();   //Send ping, get ping distance in centimeters (cm).
    if (cm == 0)
    {
      cm = 250;
    }
    return cm;    //return distance whenever readPing function is called
  }


  /*---------------------------------------------------------- STOP -----------------------------------------------------*/
  void Stop()       // Stop Function for Motor Driver.
  {
    analogWrite(motorPin0, 0); // send mSpeed value to motor
    digitalWrite(relay_left_down, HIGH);
    digitalWrite(relay_right_down, HIGH);
    digitalWrite(relay_left_up, LOW);
    digitalWrite(relay_right_up, LOW);
  }

  /*--------------------------------------------------------- FORWARD -----------------------------------------------------*/

  void moveForward()    // Move Forward Function for Motor Driver.
  {
    digitalWrite(relay_left_down, HIGH);
    digitalWrite(relay_right_down, HIGH);
    digitalWrite(relay_left_up, HIGH);
    digitalWrite(relay_right_up, HIGH);
  }

  /*--------------------------------------------------------- BACKWARD ----------------------------------------------------*/

  void moveBackward()   // Move Backward Function for Motor Driver.
  {
    digitalWrite(relay_left_down, LOW);
    digitalWrite(relay_right_down, LOW);
    digitalWrite(relay_left_up, LOW);
    digitalWrite(relay_right_up, LOW);
  }

  /*--------------------------------------------------------- RIGHT ----------------------------------------------------*/

  void turnRight()      // Turn Right Function for Motor Driver.
  {
    myservo1.write(160);
    makeleft_move();
  }

  /*---------------------------------------------------------- LEFT ----------------------------------------------------*/
  void turnLeft()       // Turn Left Function for Motor Driver.
  {
    myservo1.write(40);
    makeright_move();
  }


  void makeright_move()
  {
    analogWrite(motorPin0, 200); // send mSpeed value to motor
    analogWrite(motorPin2, 180); // send mSpeed value to motor
    analogWrite(motorPin1, 0); // send mSpeed value to motor
  }


  void makeleft_move()
  {
    analogWrite(motorPin0, 200); // send mSpeed value to motor
    analogWrite(motorPin1, 180); // send mSpeed value to motor
    analogWrite(motorPin2, 0); // send mSpeed value to motor
  }


  void makeboth_move()
  {
    analogWrite(motorPin0, 200); // send mSpeed value to motor
    analogWrite(motorPin1, 180); // send mSpeed value to motor
    analogWrite(motorPin2, 180); // send mSpeed value to motor
  }
