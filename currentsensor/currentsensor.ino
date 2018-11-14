

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  Serial.print(currentSensor(analogRead(A0)));
  Serial.print("\n");
}

long readInternalVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                                                    // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                                         // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;                                  // Back-calculate AVcc in mV
  return result;
}

double currentSensor(int RawADC) {

  int    Sensitivity    = 66; // mV/A
  long   InternalVcc    = readInternalVcc();
  double ZeroCurrentVcc = InternalVcc / 2;
  double SensedVoltage  = (RawADC * InternalVcc) / 1024;
  double Difference     = SensedVoltage - ZeroCurrentVcc;
  double SensedCurrent  = Difference / Sensitivity;
  return SensedCurrent;                                        // Return the Current
}
