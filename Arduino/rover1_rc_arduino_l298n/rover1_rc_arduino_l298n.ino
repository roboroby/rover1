/*
 * Author: Roby
 * deepsouthrobotics.com
 * April 7, 2017
 * 
 * Code running Rover 1 with Radio/Arduino/L298N
 * 
 */

const int arduinoPinCorrespondingToRcChannel1 = 6; //Steering
const int arduinoPinCorrespondingToRcChannel3 = 7; //Speed

const long pulseInDelayMicros = 250000; //Number of microseconds to wait for pulse reading to be completed
const float steerSensitivity = 0.75; //Higher number makes more aggressive turning
const int deadZone = 15; //Between 0 and 255 -- any steering/throttle below this is ignored

/*
 * Adjust channel1 and channel3 min/mid/max
 * per your radio
 */
const int channel1Min = 1040;
const int channel1Mid = 1434;
const int channel1Max = 1840;

const int channel3Min = 1040;
const int channel3Mid = 1445;
const int channel3Max = 1850;

// L298N to Arduino
// right motor
const int enA = 11;
const int in1 = 12;
const int in2 = 13;

// L298N to Arduino
// left motor 
const int enB = 10;
const int in3 = 9;
const int in4 = 8;

int channel1 = 0; // Steering channel
int channel3 = 0; // Speed channel

int leftMotorSpeed = 0; //pwm speed calculated for left motor
int rightMotorSpeed = 0; //pwm speed calculated for right motor

void setup() 
{
  //Debugging:
  //Serial.begin(9600);
  
  pinMode(arduinoPinCorrespondingToRcChannel1, INPUT); // Set our input pins as such
  pinMode(arduinoPinCorrespondingToRcChannel3, INPUT);

  //Set L298N control pins as OUTPUT
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

/*
 * This is where we do the mixing -- converting steer and speed input
 * into left and right motor speed
 */
void calculateLeftAndRightMotorSpeed(int speedChannelScaled, int steerChannelScaled) 
{
  leftMotorSpeed = speedChannelScaled + (steerSensitivity * steerChannelScaled);
  rightMotorSpeed = speedChannelScaled - (steerSensitivity * steerChannelScaled);

  // neither motor can go faster than maximum speed
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  //Debugging:
  /*
  Serial.print("Left motor speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("  Right motor speed: ");
  Serial.print(rightMotorSpeed);
  Serial.println();
  */
}

void setLeftMotorForward() 
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}

void setLeftMotorBackward() 
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); 
}

void setRightMotorForward() 
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
}

void setRightMotorBackward() 
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);  
}

void stop() 
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}

void loop() 
{
  // Read the pulse width of each channel
  channel1 = pulseIn(arduinoPinCorrespondingToRcChannel1, HIGH, pulseInDelayMicros); 
  channel3 = pulseIn(arduinoPinCorrespondingToRcChannel3, HIGH, pulseInDelayMicros);

  //Debugging
  /*
  Serial.print("Channel1: ");
  Serial.println(channel1);
  Serial.print("Channel3: ");
  Serial.println(channel3);
  */
  
  //Handle case where radio is turned off (or out of range) -- channel1 and channel3 are zero
  if(channel1 == 0 && channel3 == 0) 
  {
    //stop motors
    Serial.println("No signal from radio channel 1 or radio channel 3");
    stop();
  }
  else
  {
    //Now we've got channel1 and channel3 radio pwm, convert to -255 to 255
    if(channel1 < channel1Mid) 
    {
      channel1 = map(channel1, channel1Mid, channel1Min, 0, -255);
    }
    else 
    {
      channel1 = map(channel1, channel1Mid, channel1Max, 0, 255);
    }
  
    if(channel3 < channel3Mid) 
    {
      channel3 = map(channel3, channel3Mid, channel3Min, 0, -255);
    }
    else 
    {
      channel3 = map(channel3, channel3Mid, channel3Max, 0, 255);
    }

    //Now calculate left and right motor speed from channel1 
    //and channel3 scaled values
    calculateLeftAndRightMotorSpeed(channel3, channel1);

    //If the calculated speed for both motors is in the dead zone
    //then stop rover
    if((abs(leftMotorSpeed) <= deadZone) && (abs(rightMotorSpeed) <= deadZone))
    {
      stop();
      Serial.println("In dead zone");
    }
    else //Run logic to set left and right motor speed via the L298N controller
    {
      if(leftMotorSpeed >= 0) 
      {
        setLeftMotorForward();
      }
      else 
      {
        setLeftMotorBackward();
      }
      
      analogWrite(enB, abs(leftMotorSpeed));
      
      if(rightMotorSpeed >= 0) 
      {
        setRightMotorForward();
      }
      else 
      {
        setRightMotorBackward();
      }
      analogWrite(enA, abs(rightMotorSpeed));
    }

  }

}

