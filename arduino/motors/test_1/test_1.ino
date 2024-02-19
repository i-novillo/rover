// Motor A
int EN = 10;
int IN1 = 9;
int IN2 = 8;


void setup ()
{
 pinMode (EN, OUTPUT);
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
}
void loop() {
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (EN, 50);
}
