void setup() {
Serial.begin(115200);

Serial.println();
Serial.println("Laenge der Datentypen beim ESP8266");
Serial.println();
Serial.print("sizeof void :        "); Serial.print(sizeof(void));         Serial.println(" byte");
Serial.print("sizeof bool :        "); Serial.print(sizeof(bool));         Serial.println(" byte");
Serial.print("sizeof char :        "); Serial.print(sizeof(char));         Serial.println(" byte");
Serial.print("sizeof byte :        "); Serial.print(sizeof(byte));         Serial.println(" byte");
Serial.print("sizeof uint8_t :     "); Serial.print(sizeof(uint8_t));      Serial.println(" byte");
Serial.println();
Serial.print("sizeof short :       "); Serial.print(sizeof(short));        Serial.println(" byte");
Serial.print("sizeof uint16_t :    "); Serial.print(sizeof(uint16_t));     Serial.println(" byte");
Serial.print("sizeof int16_t :     "); Serial.print(sizeof(int16_t));      Serial.println(" byte");
Serial.println();
Serial.print("sizeof word :        "); Serial.print(sizeof(word));         Serial.println(" byte");
Serial.print("sizeof int :         "); Serial.print(sizeof(int));          Serial.println(" byte");
Serial.print("sizeof long :        "); Serial.print(sizeof(long));         Serial.println(" byte");
Serial.print("sizeof float :       "); Serial.print(sizeof(float));        Serial.println(" byte");
Serial.print("sizeof uint32_t :    "); Serial.print(sizeof(uint32_t));     Serial.println(" byte");
Serial.print("sizeof int32_t :     "); Serial.print(sizeof(int32_t));      Serial.println(" byte");
Serial.println();
Serial.print("sizeof double :      "); Serial.print(sizeof(double));       Serial.println(" byte");
Serial.print("sizeof long long :   "); Serial.print(sizeof(long long));    Serial.println(" byte");
Serial.print("sizeof uint64_t :    "); Serial.print(sizeof(uint64_t));     Serial.println(" byte");
}

void loop() {
}
