
int32_t MicrosMerker = 0;

float A = 245234656.3463257245724;
float B = 547457547.4376257244574;
float C = 0;

long D = 10474836;
long E = 213;
long F = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
MicrosMerker = micros();
F = D*E;
MicrosMerker = micros() - MicrosMerker;
Serial.print("long Multiplikation dauert us: ");
Serial.println(MicrosMerker);

MicrosMerker = micros();
C = A*B;
MicrosMerker = micros() - MicrosMerker;
Serial.print("float Multiplikation dauert us: ");
Serial.println(MicrosMerker);

MicrosMerker = micros();
F = D*E;
MicrosMerker = micros() - MicrosMerker;
Serial.print("long Multiplikation2 dauert us: ");
Serial.println(MicrosMerker);

MicrosMerker = micros();
C = A*B;
MicrosMerker = micros() - MicrosMerker;
Serial.print("float Multiplikation2 dauert us: ");
Serial.println(MicrosMerker);

MicrosMerker = micros();
F = D*E;
MicrosMerker = micros() - MicrosMerker;
Serial.print("long Multiplikation3 dauert us: ");
Serial.println(MicrosMerker);

MicrosMerker = micros();
C = A*B;
MicrosMerker = micros() - MicrosMerker;
Serial.print("float Multiplikation3 dauert us: ");
Serial.println(MicrosMerker);


delay(5000);
}
