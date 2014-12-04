#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


const float vPow = 5.0;
const float r1 = 3300;
const float r2 = 1000;
const int batteryPin = 0;
const int fateOutTime = 3000;

const int statusPin = 2;
int sleepStatus = LOW;
const int bluetoothPin = 4;
uint8_t brightnessState = 50;


#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

String inputCommand = "";
char command = 'f';
String param = "";
boolean fadeIn = true;

void setup() {
  pinMode(statusPin, INPUT);
  pinMode(bluetoothPin, OUTPUT);

  digitalWrite(bluetoothPin,HIGH);

  Serial.begin(9600);
  strip.begin();
  strip.setBrightness(brightnessState);
  strip.show(); // Initialize all pixels to 'off'
  inputCommand.reserve(12);
  Serial.println('.');
}

void loop() {

  sleepStatus = digitalRead(statusPin);   // read sleep pin here. only active
  if (sleepStatus == LOW) {            // start to put the device in sleep
    sleepNow();                      // sleep function called here
  }

  switch (command) {
  case  'r':
    rainbow(20);
    break;
  case  'c' :
    colorWipe(param.toInt(), 1000);
    break;
  case 'a' :
    fade(param.toInt());
    break;
  default: 
    fire(random(1000,10000));
  }

}



void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputCommand += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      char newCommand = inputCommand.charAt(0);
      String newParam = inputCommand.substring(1);
      if(newCommand == 'p'){
        preferenceEvent(newParam);
      }
      else{
        command = newCommand;
        param = newParam;
      }
      Serial.println(inputCommand + ":0:ok");
      inputCommand = "";
    } 
  }
}

void preferenceEvent(String preferenceCommand){
  char preference = preferenceCommand.charAt(0);
  String paramPreference =  preferenceCommand.substring(1);
  switch (preference) {

  case  'b' :
    brightnessState = paramPreference.toInt();
    strip.setBrightness(paramPreference.toInt());
    break;
  case 'v':
    //Serial.println('p' + paramPreference + ":0:" + readBattery());
    Serial.println(readBattery());
    break;
  case 's':
    sleepNow();
    break;
  default: 
    Serial.println('p' + paramPreference + ":1:invalid");
  }

}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();

  }
  delay(wait);
}

void fire(uint8_t wait) {
  strip.setPixelColor(random(0,strip.numPixels()), strip.Color(10,0,0));
  strip.setPixelColor(random(0,strip.numPixels()), strip.Color(random(200,255),153,0));
  strip.show();
  delay(wait);




}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

void fade( uint8_t wait){
  uint8_t brightness = strip.getBrightness();
  Serial.println(brightness,DEC);
  if(fadeIn){
    if(brightness<100){
      strip.setBrightness(brightness+1);
    }
    else{
      fadeIn = false;
    }
  }
  else{
    if(brightness>10){
      strip.setBrightness(brightness-1);
    }
    else{
      fadeIn = true;
    }
  }
  strip.show();
  delay(wait);
}

void fadeOut(){
  uint8_t brightness = strip.getBrightness();
 
  int wait = fateOutTime / brightness;

  Serial.println("fadeOut");
  for (; brightness>10; brightness--) {
    Serial.println(brightness, DEC);
    Serial.println(wait, DEC);
    strip.setBrightness(brightness);
    strip.show();
    delay(wait);
  }




}


float readBattery(){
  float v = (analogRead(0) * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  return v2;
}



long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


void wakeUpNow(){
  
  digitalWrite(bluetoothPin,HIGH);
 
  strip.setBrightness(brightnessState);
 // strip.show(); 

}


void sleepNow()         // here we put the arduino to sleep
{
  fadeOut();

  digitalWrite(bluetoothPin,LOW);
  strip.clear();
  strip.show();
  delay(200);
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep modus.
   *
   * In the avr/sleep.h file, the call names of these sleep modus are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible,
   * so we choose the according sleep modus: SLEEP_MODE_PWR_DOWN
   *
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();              // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now is time to enable a interrupt. we do it here so an
   * accidentally pushed interrupt button doesn't interrupt
   * our running program. if you want to be able to run
   * interrupt code besides the sleep function, place it in
   * setup() for example.
   *
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
   *
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */

  attachInterrupt(0,wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW

    sleep_mode();                // here the device is actually put to sleep!!
  //

  sleep_disable();             // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);          // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.
  delay(1000);                 // wat 2 sec. so humans can notice the
  // interrupt.
  // LED to show the interrupt is handled

}


