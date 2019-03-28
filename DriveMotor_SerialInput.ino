// connect motor controller pins to Arduino digital pins
// motor one
int enA = 5;
int in1 = 6;
int in2 = 7;
boolean packetEnded = false;
boolean backward = false;
int steps;


void setup()
{
  Serial.begin(9600);
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 40);  // set speed to 200 out of possible range 0~25
}

void loop()
{

  while (Serial.available() > 0) {
    //Read each character, check if negative, end of command or
    //digit, and adjust step value accordingly
    char aChar = Serial.read();
    if (aChar == '-') {
      backward = true;
    }
    else if (aChar == ';') {
      packetEnded = true;
    }
    else if (aChar >= '0' && aChar <= '9') {
      steps *= 10;
      steps += aChar - '0';
    }

  }

  if (packetEnded) {
    //Perform steps
    analogWrite(enA, steps);
    Serial.print(steps);
    Serial.println(" Speed Pulse");
    //delay(200);
    packetEnded = false;
    steps = 0;
  }
}



void demoOne()
{
  // this function will run the motors in both directions at a fixed speed- can change the in1 and in2 High/Low to chage direction
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 150);  // set speed to 200 out of possible range 0~25

  delay(2000);

  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);
}
