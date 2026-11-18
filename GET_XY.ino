void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   Serial.print(" A10 ");
   Serial.print(analogRead(A10));
   Serial.print(" || A0 ");
   Serial.println(analogRead(A0));

}
