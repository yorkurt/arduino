# Modified Arduino library for the qik2s12v10 for use with AltSoftSerial

### Library Reference

#### PololuQik

- `PololuQik(unsigned char receivePin, unsigned char transmitPin,
  unsigned char resetPin)` <br> Constructor; sets pins as specified.
  The receive and transmit pins are specified from the perspective of
  the **Arduino** (so the receive pin should be connected to the TX
  pin on the qik and the transmit pin should be connected to the RX
  pin on the qik).
- `void init(long speed = 9600)` <br> Resets the qik and initializes
   serial communication with it at the specified baud rate (defaults
   to 9600 bps if unspecified).
- `char getFirmwareVersion()` <br> Returns an ASCII byte that
  represents the version of the firmware running on the qik.
- `byte getErrors()` <br> Returns a byte that indicates any errors
  that have been detected since the errors were last read.  See the
  user's guides for the meaning of each bit.
- `byte getConfigurationParameter(byte parameter)` <br> Returns the
  current value of the specified configuration parameter.
- `byte setConfigurationParameter(byte parameter, byte value)` <br> Sets
  the specified configuration parameter to the specified value.

- `void setM0Speed(int speed)` <br> Sets speed and direction for motor
  M0. `speed` should be between -127 and 127 in 7-bit mode or -255 and
  255 in 8-bit mode.  Positive values correspond to motor current
  flowing from the + pin to the - pin. Negative values correspond to
  motor current flowing from the - pin to the + pin.
- `void setM1Speed(int speed)` <br> Sets speed and direction for motor
  M1.
- `void setSpeeds(int m0Speed, int m1Speed)` <br> Sets speed and
  direction for both motors.

#### PololuQik2s12v10

- `void setM0Brake(unsigned char brake)` <br> Sets brake for motor
  M0. `brake` should be between 0 and 127. 0 corresponds to full
  coast, and 127 corresponds to full brake.
- `void setM1Brake(unsigned char brake)` <br> Sets brake for motor
  M1.
- `void setBrakes(unsigned char m0Brake, unsigned char m1Brake)` <br>
  Sets brake for both motors.
- `unsigned char getM0Current()` <br> Returns raw current reading from
  motor M0.
- `unsigned char getM1Current()` <br> Returns raw current reading from
  motor M1.
- `unsigned int getM0CurrentMilliamps()` <br> Returns current reading
  from motor M0 in milliamps.
- `unsigned int getM1CurrentMilliamps()` <br> Returns current reading
  from motor M1 in milliamps.
- `unsigned char getM0Speed()` <br> Returns the present target motor
  speed for motor M0.
- `unsigned char getM1Speed()` <br> Returns the present target motor
  speed for motor M1.

## Version history
* 2.0.0 (2016-08-16): Updated library to work with the Arduino Library Manager.
* 1.1.1 (2016-06-08): Made SoftwareSerial include to be a system header include (thanks Dippyskoodlez).
* 1.1.0 (2013-05-22): Added calls to `SoftwareSerial::listen()` for better behavior when using multiple `PololuQik` instances.
* 1.0.0 (2012-05-07): Original release.
