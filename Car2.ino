
/*---------------------------------------------------------- KNIZNICE -------------------------------------------------------*/

#include <Servo.h>          // SERVO KNIZNICA
#include <NewPing.h>        // NEWPING KNIZNICA
#include <TimerFreeTone.h>  // TONE KNIZNICA


class MACROS
{
  public:
#define BZUCIAK 5 //buzzer to arduino pin 13
#define relay_left 4
#define relay_right 2
#define SONAR_NUM 1      // Pocet ultrasonickych senzorov
#define MAX_DISTANCE 200 // Maximalna vziadalenost pre pingovanie

};

class VYPIS {
    public:
    virtual void f() { Serial.println("OBRATENE VOZIDLO"); }
};

class VYPIS2 : public VYPIS {
    public:
    virtual void f() { Serial.println("NEOBRATENE VOZIDLO"); }
};

    int z;
    const int zpin = A2;
    int distance;
    int motorPin0 = 3;
    const unsigned long period = 1000;
    unsigned long startMillis;




NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(A0, A1, MAX_DISTANCE), // Dolný senzor 2
};
Servo myservo1;  // create servo object to control a servo





void setup()
{
  Serial.begin(9600);
  pinMode(BZUCIAK, OUTPUT); // set Buzzer as output

  myservo1.attach(6);  // attaches the servo on pin 6 to the servo object
  

  pinMode(A5, INPUT); //INFRARED SENSOR

  pinMode(motorPin0, OUTPUT); // set mtorPin as output

  pinMode(4, OUTPUT); //Relay 1 - u
  pinMode(2, OUTPUT); //Relay 2 - u


  myservo1.write(100);   // Set at 100 degrees.
  delay(2000);              // Wait for 2s.
  int distance = readPing1();    // Get Ping Distance.
  delay(100);               // Wait for 100ms.

  analogWrite(motorPin0, 120); // send mSpeed value to motor
}

/*---------------------------------------------------------- LOOP -------------------------------------------------------*/
void loop()
{
  Stop();
  VYPIS* obj;
  obj = new VYPIS();
  obj->f(); 
  obj = new VYPIS2();
  obj->f();
  int x = analogRead(zpin);
  if (x > 300)
  {
    funkcia_main();
  }
}


void funkcia_main()
{
  int distanceRight = 0;  //Initialize right side distance
  int distanceLeft = 0;   //Initialize left side distance
  int distanceBack = 0;   //Initialize left side distance


  if (distance <= 30)
  {
    moveBackward();
    TimerFreeTone(BZUCIAK, 5000, 200);
    Stop();   //call stop function to stop the robot
    TimerFreeTone(BZUCIAK, 5000, 100);
    TimerFreeTone(BZUCIAK, 8000, 200);
    TimerFreeTone(BZUCIAK, 1500, 100);
    distanceRight = lookRight();    //call lookRight function to save distance in distanceRight variable
    delay(500);   //wait for 500ms
    distanceLeft = lookLeft();    //call lookLeft function to save distance in distanceLeft variable
    delay(500);   //wait for 500ms
    if (distanceRight <= 30 && distanceLeft <= 30)
    {
      TimerFreeTone(BZUCIAK, 8000, 100);
      TimerFreeTone(BZUCIAK, 900, 200);
      TimerFreeTone(BZUCIAK, 8000, 100);
      TimerFreeTone(BZUCIAK, 900, 200);
      do
      {
        moveBackward();

      }
      while (digitalRead(A5) == HIGH);;

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
  myservo1.write(0);   //make servo position at 0 degree
  delay(700);   //wait for 500ms
  int distance = readPing1();    //read distance
  delay(200);   //wait for 100ms
  return distance;    //return distance whenever lookRight function is called
}

/*---------------------------------------------------------- LOOK LEFT -------------------------------------------------------*/

int lookLeft()      // lookLeft Function for Servo Motor
{
  myservo1.write(80);   //make servo position at 0 degree
  delay(700);   //wait for 500ms
  int distance = readPing1();    //read distance
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

/*---------------------------------------------------------- STOP -----------------------------------------------------*/
void Stop()       // Stop Function for Motor Driver.
{
  analogWrite(motorPin0, 0); // send mSpeed value to motor
  digitalWrite(relay_left, LOW);
  digitalWrite(relay_right, LOW);
}

/*--------------------------------------------------------- FORWARD -----------------------------------------------------*/

void moveForward()    // Move Forward Function for Motor Driver.
{
  digitalWrite(relay_left, LOW);
  digitalWrite(relay_right, HIGH);
  makeboth_move();
}

/*--------------------------------------------------------- BACKWARD ----------------------------------------------------*/

void moveBackward()   // Move Backward Function for Motor Driver.
{
  digitalWrite(relay_left, HIGH);
  digitalWrite(relay_right, LOW);
  makeboth_move();
}

/*--------------------------------------------------------- RIGHT ----------------------------------------------------*/

void turnRight()      // Turn Right Function for Motor Driver.
{
  myservo1.write(0);
  makeboth_move();
}

/*---------------------------------------------------------- LEFT ----------------------------------------------------*/
void turnLeft()       // Turn Left Function for Motor Driver.
{
  myservo1.write(80);
  makeboth_move();
}


void makeboth_move()
{
  analogWrite(motorPin0, 120); // send mSpeed value to motor
}
