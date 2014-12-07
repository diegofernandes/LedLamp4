#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


const float vPow = 5.0;
const float r1 = 3300;
const float r2 = 1000;

int sleepStatus = LOW;
uint8_t brightnessState = 50;


#define PIN 6
#define STATUS_PIN 2
#define BLUETOOTH_PIN 4
#define BATTERRY_PIN A0
#define FADE_OUT_TIME 3000



Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

String inputCommand = "";
char command = 'f';
String param = "";
boolean fadeIn = true;

void setup() {
  pinMode(STATUS_PIN, INPUT);
  pinMode(BLUETOOTH_PIN, OUTPUT);

  digitalWrite(BLUETOOTH_PIN,HIGH);

  Serial.begin(9600);
  strip.begin();
  strip.setBrightness(brightnessState);
  strip.show(); // Initialize all pixels to 'off'
  inputCommand.reserve(12);
  Serial.println('.');
}

void loop() {

  sleepStatus = digitalRead(STATUS_PIN);   // read sleep pin here. only active
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
  int wait = FADE_OUT_TIME / brightness;
  for (; brightness>10; brightness--) {
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

  digitalWrite(BLUETOOTH_PIN,HIGH);

  strip.setBrightness(brightnessState);


}


void sleepNow(){
  fadeOut();

  digitalWrite(BLUETOOTH_PIN,LOW);
  strip.clear();
  strip.show();
  delay(200);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 

  sleep_enable();
  attachInterrupt(0,wakeUpNow, LOW);
  sleep_mode();
  sleep_disable(); 
  detachInterrupt(0); 
  delay(1000);

}




