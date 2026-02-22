#define X1_PIN A0
#define Y1_PIN A1
#define X2_PIN A2
#define Y2_PIN A3
#define SWITCH1_PIN D4
#define SWITCH2_PIN D2

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(10);
  Serial.begin(9600);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
}

int x1Pos = 0;
int y1Pos = 0;
int x2Pos = 0;
int y2Pos = 0;

int ULIM = 255;
int LLIM = -255;

void set_pos(uint16_t axis_value, int* pos) {
  if (*pos <= ULIM) {
    if (axis_value > 900) {
      *pos += 4;
    } else if (axis_value > 800) {
      *pos += 3;
    } else if (axis_value > 700) {
      *pos += 2;
    } else if (axis_value > 600) {
      *pos++;
    }
    if (*pos > ULIM) *pos = ULIM;
  }

  if (*pos >= LLIM) {
    if (axis_value < 100) {
      *pos -= 4;
    } else if (axis_value < 200) {
      *pos -= 3;
    } else if (axis_value < 300) {
      *pos -= 2;
    } else if (axis_value < 400) {
      *pos--;
    }
    if (*pos < LLIM) *pos = LLIM;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t x1 = analogRead(X1_PIN);
  uint16_t y1 = analogRead(Y1_PIN);
  uint16_t x2 = analogRead(X2_PIN);
  uint16_t y2 = analogRead(Y2_PIN);
  uint8_t sw1 = digitalRead(SWITCH1_PIN);
  uint8_t sw2 = digitalRead(SWITCH2_PIN);
  if (!sw1) {
    Serial.println("Sw1 pressed");
    delay(100);
  }
  if (!sw2) {
    Serial.println("Sw2 pressed");
    delay(100);
  }

  set_pos(x1, &x1Pos);
  set_pos(y1, &y1Pos);
  set_pos(x2, &x2Pos);
  set_pos(y2, &y2Pos);

  Serial.print("x1: ");
  Serial.print(x1);
  Serial.print(" y1: ");
  Serial.print(y1);
  Serial.print(" x2: ");
  Serial.print(x2);
  Serial.print(" y2: ");
  Serial.print(y2);
  Serial.print(" x1Pos: ");
  Serial.print(x1Pos);
  Serial.print(" y1Pos: ");
  Serial.print(y1Pos);
  Serial.print(" x2Pos: ");
  Serial.print(x2Pos);
  Serial.print(" y2Pos: ");
  Serial.print(y2Pos);

  Serial.println();
}
