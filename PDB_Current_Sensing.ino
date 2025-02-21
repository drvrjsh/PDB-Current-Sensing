#include <LiquidCrystal.h>
#include <Wire.h>
#include <movingAvg.h>

#define TEMP_UPDATE_INTERVAL 100
#define LCD_UPDATE_INTERVAL 1000
#define MOVING_AVG_SAMPLES 1000

unsigned long temp_previous_millis = 0;
unsigned long lcd_previous_millis = 0;

const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int motor_count = 0;
int moving_avg_sample_count = 0;

void setup() {
  Wire.begin();

  lcd.begin(16, 2);

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  //Begin I2C transmission
  Wire.beginTransmission(54);

  //Temperature sensor
  //Mux to switch position three
  digitalWrite(11, HIGH);   
  digitalWrite(12, LOW);

  int tempinput = analogRead(A7); 
  tempinput = map(tempinput, 0, 1023, 0, 5); //Sensors datasheet maps 5v range directly to 1023
  tempinput = (tempinput/1000) * 0.0195;
  Wire.write(tempinput); //I2C write temperature sensor data

  //Asynchronously output temperature sensor data to LCD  
  if (millis() >= TEMP_UPDATE_INTERVAL + temp_previous_millis) { 
    temp_previous_millis += TEMP_UPDATE_INTERVAL;

    String tempfinal = String(tempinput); 

    lcd.setCursor(0, 1);
    lcd.print("                "); //Clear temperature sensor data line

    lcd.setCursor(0, 1);
    lcd.print("Temp: " + tempfinal + (char)223 + "C"); //Output temperature sensor data to LCD
  }

  //Motors 1-5
  if (motor_count <= 4) {
    movingAvg currentSensor(MOVING_AVG_SAMPLES);

    currentSensor.begin();
    int analogarray[] = {analogRead(A6), analogRead(A1), analogRead(A2), analogRead(A3), analogRead(A0)}; //Store motor voltage input
    int reading = currentSensor.reading(analogarray[2]);

    moving_avg_sample_count += 1;

    if (moving_avg_sample_count == (MOVING_AVG_SAMPLES - 1)) {
      analogarray[2] = currentSensor.getAvg();
      Serial.println(analogarray[2]);
      currentSensor.reset();
      moving_avg_sample_count = 0;
    }
    
    String display;
    float analogoutput;
    
    analogoutput = (4.695/40.96) * analogarray[motor_count]; //Converts voltage input from current sensor to a current in amps
    int analogoutputconverted = analogoutput * 100; //Multiplies 2 decimal voltage input by factor for I2C output
    Wire.write(analogoutputconverted); //I2C write motor current data

    //Asynchronously output motor current to LCD  
    if (millis() >= LCD_UPDATE_INTERVAL + lcd_previous_millis) {
      lcd_previous_millis += LCD_UPDATE_INTERVAL; 

      String analogmath = String(analogoutput);
      display = "Motor " + String(motor_count+1) + ": " + analogmath + "A"; 

      lcd.setCursor(0, 0);
      lcd.print("                "); //Clear motor current data line

      lcd.setCursor(0, 0);
      lcd.print(display); //Output motor current data to LCD

      motor_count += 1; //Increment motor count 
    }
  }

  //Motor 6
  if (motor_count == 5) {
    //Mux to switch position two
    digitalWrite(11, LOW);    
    digitalWrite(12, HIGH);

    int currentsense = analogRead(A7); 
    Wire.write(currentsense); //I2C write motor current data
    
    //Asynchronously output motor current to LCD
    if (millis() >= LCD_UPDATE_INTERVAL + lcd_previous_millis) {
      lcd_previous_millis += LCD_UPDATE_INTERVAL;

      String currentstring = String(currentsense);
      String display2 = "Motor 6: " + currentstring + "A";

      lcd.setCursor(0, 0);
      lcd.print("                "); //Clear motor current data line

      lcd.setCursor(0, 0);
      lcd.print(display2); //Output motor current data to LCD

      motor_count = 0; //Reset motor count
    }
  }

  Wire.endTransmission();
}

