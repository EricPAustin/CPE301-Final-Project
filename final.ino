/****************************************
 * Name: Eric Austin and Omar Vega
 * Assignment: Final Project
 * Date: 5/11/2024
 ****************************************/

#include <dht_nonblocking.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;

// Assigning pins to specific LED colors
#define BLUE 53 // Activated during motor operation.
#define GREEN 51 // Lights up in idle state, before fan activation.
#define RED 47 // Indicates low water level; other LEDs off until resolved.
#define YELLOW 49 // On during system's disabled state.

#define DHT_SENSOR_TYPE DHT_TYPE_11
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

const int A = 0b00000010;

// LED intensity configuration
int redValue = 255;
int greenValue = 255;
int blueValue = 255;
int yellowValue = 255;

// Register pointers for ADC configuration
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// Water sensor and servo motor variable initialization
int adc_id = 0;
int HistoryValue = 0;
char printBuffer[128];

// LiquidCrystal display initialization with pin numbers
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

static const int DHT_SENSOR_PIN = 2; // Temperature sensor pin
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

// Port configuration for various functions
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;

int inPin = 52;         // Input pin number
int outPin = 50;        // Output pin number

int state = LOW;        // Current state of the output pin
int reading;            // Current reading from the input pin
int previous = HIGH;    // Previous reading from the input pin

long time = 0;          // Last toggle time of the output pin
long debounce = 200;    // Debounce time; adjust if output flickers

void setup( ) {
  Serial.begin( 9600);
  adc_init(); // Initialize water and servo controls
  myservo.attach(13);
  myservo.write(90); // Center servo position at 90 degrees
  lcd.begin(16, 2); // Set LCD columns and rows

  *ddr_l |= B00000100; // Configure pins for LEDs
  *ddr_b |= B00000100;
  *ddr_b |= B00000001;
  *ddr_l |= B00000001;
  *port_l |= B00000001; // Set initial LED states

  *ddr_e |= 0x20; // DC motor configuration
  *ddr_e |= 0x08;
  *ddr_g |= 0x20;

  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);

  // RTC error handling
 if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
   while (1);
 }
 if (! rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
 }
}

void loop( ) {
  float temperature;
  float humidity;
  reading = digitalRead(inPin);

  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    state = !state;
    Serial.println(state == HIGH ? "ON" : "OFF");
    time = millis();    
  }

  digitalWrite(outPin, state);
  previous = reading;

  if(state == HIGH){
    if( measure_environment( &temperature, &humidity ) == true ) {
      timeStamp();
      Serial.printf("Temperature: %.1f deg. F, Humidity = %.1f%%\n", temperature, humidity);
    }

    int value = adc_read(adc_id); // Read ADC value for water sensor

    if(value < 50){ errorLED(value); }
    
    if(abs(HistoryValue - value) > 10){
      sprintf(printBuffer,"Water level is %d\n", value);
      Serial.print(printBuffer);
      HistoryValue = value;
    }

    servoLoop();
  
    if(temperature > 0){
      lcdScreen(temperature, humidity);
    }

    motorToggle(temperature, value);
  }
  else if(state == LOW){
    lcd.setCursor(0, 0);
    lcd.print("STATUS:           ");
    lcd.setCursor(0, 1);
    lcd.print("IDLE...          ");

    *port_l |= B00000001;
    *port_b &= B11111011;
  }
}

void errorLED(int waterLevel) {
  // Activate RED LED and display error message on low water level
  *port_b &= B11111010; // Turn off BLUE LED
  *port_l |= B00000100; // Activate RED LED
  lcd.setCursor(0, 0);
  lcd.print("Error!         ");
  lcd.setCursor(0, 1);
  lcd.print("WATER TOO LOW!      ");
  waterLevel = adc_read(adc_id);
  delay(4000);
  if (waterLevel < 50) {
    errorLED(waterLevel); // Recursive call if water level remains low
  }
}

void servoLoop() {
  // Read voltage and adjust servo angle accordingly
  int voltage = adc_read(A); // Read voltage from potentiometer
  int angle = voltage / 5.7; // Convert voltage to servo angle
  myservo.write(angle); // Update servo position
}

void lcdScreen(float temperature, float humidity) {
  // Update LCD with current humidity and temperature
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print(humidity, 1);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  lcd.print("Fahrenheit: ");
  lcd.print(temperature, 1);
}

void timeStamp() {
  // Log time for temperature readings
  DateTime now = rtc.now();
  Serial.print("\nTime: ");
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  int hour = now.hour();
  hour -= 4; // Time zone adjustment if needed
  Serial.print(hour, DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  delay(3000);
}

static bool measure_environment(float *temperature, float *humidity) {
  // Measure temperature and humidity at defined intervals
  static unsigned long measurement_timestamp = millis();
  if (millis() - measurement_timestamp > 3000ul) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurement_timestamp = millis();
      return true;
    }
  }
  return false;
}

unsigned int adc_read(unsigned int adc_channel_num) {
  // Read ADC value from specified channel
  int channel_selector;
  *my_ADMUX &= B11100000; // Clear the channel selection bits in ADMUX (MUX 4:0)
  *my_ADCSRB &= B11110111; // Clear the MUX5 bit in ADCSRB

  // Assign the correct channel using MUX 5:0
  if (adc_channel_num < 8) {
    *my_ADMUX |= adc_channel_num;
  } else if ((adc_channel_num > 7) && (adc_channel_num < 16)) {
    channel_selector = (adc_channel_num - 8);
    *my_ADCSRB |= B00001000;
    *my_ADMUX |= channel_selector;
  }

  // Start the conversion and wait for it to finish
  *my_ADCSRA |= B01000000; // Start the conversion
  while ((*my_ADCSRA & 0x40) != 0); // Wait for conversion to complete
  
  return (*my_ADC_DATA & 0x03FF); // Return the conversion result
}

void motorToggle(float temperature, float value) {
  if (temperature > 76 && value > 50) {
    // Engage motor control if temperature and water level are adequate
    *port_e |= 0x08; // Enable bit on PE3 to turn on motor
    *port_b |= B00000001; // Activate BLUE LED
    *port_l &= 11111010; // Turn off RED and YELLOW LEDs
    *port_b &= B11111011; // Turn off GREEN LED
  }
  
  if (temperature < 76) {
    // Disengage motor control if temperature is too low
    *port_e &= 0x00; // Disable motor
    *port_b &= B11111110; // Turn off BLUE LED
    *port_l &= B11111010; // Turn off RED and YELLOW LEDs
    *port_b |= B00000100; // Activate GREEN LED
  }
  *port_e |= 0x20; // Ensure motor controller PE5 is set to output
  *port_g &= 0x20; // Ensure PG5 output is reset
}


void adc_init() {
  // Initialize ADC for reading water sensor and controlling servo
  *my_ADCSRA |= B10000000; // Set the enable bit in ADCSRA
  *my_ADCSRA &= B11110111; // Adjust as necessary for setup
  *my_ADCSRA &= B11011111; // Clear the ADIF bit in ADCSRA by writing a 1 to it
  
  *my_ADCSRB &= B11111000; // Ensure MUX5 is clear
  
  *my_ADMUX |= (1 << REFS0); // Set reference voltage selection
}