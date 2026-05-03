// L298N H-Bridge Connection PINs
#define L298N_enA 4 // PWM
#define L298N_in1 53  // Dir Motor A
#define L298N_in2 50  // Dir Motor A

double cmd = 0.0;

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
  Serial.setTimeout(10);
}

void loop() {
  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n').toDouble();
  }

  int speed = constrain(cmd * 100, 0, 255);
  analogWrite(L298N_enA, speed);
}
