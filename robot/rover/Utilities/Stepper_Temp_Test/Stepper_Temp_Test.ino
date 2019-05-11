#define M3_ENABLE_PIN      17
#define M3_STEP_PIN        21
#define M3_DIR_PIN         20
// 19&18 are on port B with bits 2&3 respectively
#define M3_ENCODER_PORT    GPIOB_PDIR
#define M3_ENCODER_SHIFT   CORE_PIN19_BIT
#define M3_ENCODER_A       19
#define M3_ENCODER_B       18
#define M3_LIMIT_SW_FLEX   22
#define M3_LIMIT_SW_EXTEND 23
#define M3_ENCODER_RESOLUTION 2000
#define M3_STEP_RESOLUTION 1.8 // I think it's the same for all our steppers
#define M3_GEAR_RATIO      36.0 // belt reduction chained to worm gear drive
#define M3_MINIMUM_ANGLE   -115.0
#define M3_MAXIMUM_ANGLE   35.0

#define M4_ENABLE_PIN      16
#define M4_STEP_PIN        14
#define M4_DIR_PIN         15
// 11&12 are on port C with bits 6&7 respectively
#define M4_ENCODER_PORT    GPIOC_PDIR
#define M4_ENCODER_SHIFT   CORE_PIN11_BIT
#define M4_ENCODER_A       11
#define M4_ENCODER_B       12
#define M4_LIMIT_SW_FLEX   25
#define M4_LIMIT_SW_EXTEND 24
#define M4_ENCODER_RESOLUTION 2000
#define M4_STEP_RESOLUTION 1.8 // I think it's the same for all our steppers
#define M4_GEAR_RATIO      35.55555555 // belt reduction chained to worm gear drive
#define M4_MINIMUM_ANGLE   -55.0
#define M4_MAXIMUM_ANGLE   46. //40.0

void setup() {
  // put your setup code here, to run once:
pinMode(M3_ENABLE_PIN, OUTPUT);
  digitalWrite(M3_ENABLE_PIN, HIGH); // cut power
  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M3_STEP_PIN, OUTPUT);

  pinMode(M4_ENABLE_PIN, OUTPUT);
  digitalWrite(M4_ENABLE_PIN, HIGH); // cut power
  pinMode(M4_DIR_PIN, OUTPUT);
  pinMode(M4_STEP_PIN, OUTPUT);

  digitalWrite(M3_ENABLE_PIN, LOW);
  digitalWrite(M4_ENABLE_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(M3_STEP_PIN, HIGH);
  digitalWrite(M3_STEP_PIN, LOW);
  digitalWrite(M4_STEP_PIN, HIGH);
  digitalWrite(M4_STEP_PIN, LOW);
  delay(30);
}
