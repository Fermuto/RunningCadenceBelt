void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_NUM_4, OUTPUT);
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(GPIO_NUM_4, HIGH); 
  delay(100);
  Serial.print("Bruh");
}
