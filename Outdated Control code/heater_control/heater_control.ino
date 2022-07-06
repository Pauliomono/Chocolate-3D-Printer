// thermistor pins
#define THERMISTORPIN 13
#define THERMISTORPIN_VAT 15        
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 4700  
//heater pins  
#define HEATER_PIN 8
#define VAT_PIN 9

//super important temp stuff
#define TARGET_TEMP 45
#define TARGET_TEMP_VAT 45

//Variables
int samples[NUMSAMPLES];
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;

//PID constants
#define kp 30
#define ki 1.8
#define kd 18
int PID_p = 0;
int PID_i = 0; 
int PID_d = 0;

void setup(void) {
    pinMode(HEATER_PIN , OUTPUT);
    pinMode(VAT_PIN , OUTPUT);
    pinMode(THERMISTORPIN, INPUT);
    pinMode(THERMISTORPIN_VAT, INPUT);
    
  cli();//stop interrupts
  //set timer1 interrupt at FREQUENCY (Hz)
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;//(16*10^6) / (FREQUENCY*1024) - 1; //(must be <65536) 15624;
  TCCR1B |= (1 << WGM12);// turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);// Set CS10 and CS12 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt

  
  Serial.begin(9600);
  analogReference(DEFAULT);
  sei();//allow interrupts
  Time = millis(); 
}

double temp(int pin) {
  uint8_t i;
  float T;

  T = analogRead(pin);
  
  // convert the value to resistance
  T = 1023/T - 1;
  T = SERIESRESISTOR / T;
  
  float steinhart;
  steinhart = T / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  return steinhart;
}

//Interupt Routine for temp control
ISR(TIMER1_COMPA_vect){
  float extruder_temp;
  float vat_temp;
  extruder_temp = temp(THERMISTORPIN);
  vat_temp = temp(THERMISTORPIN_VAT);
  Serial.print(extruder_temp);
  Serial.print("   ");
  Serial.print(vat_temp);

  //error between setpoint and actual
  PID_error = TARGET_TEMP - extruder_temp;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if((PID_error >= -4)&&(PID_error <= 4))
  {
    PID_i = PID_i + (ki * PID_error);
  }
  else {
    PID_i = 0;
  }

  timePrev = Time;
  Time = millis();
    
  elapsedTime = (Time - timePrev) / 1000; 
  //calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value
  PID_value = 1*(PID_p + PID_i + PID_d);

  if(PID_value < 0){
    PID_value = 0; 
    }
  if(PID_value > 255){
    PID_value = 255;
    }
    
  Serial.print("   ");
  Serial.println(PID_value);
    
  //Write to heater
  analogWrite(HEATER_PIN,PID_value);
  previous_error = PID_error; 

  //simple bang-bang vat control
  if (vat_temp <= TARGET_TEMP_VAT){
    digitalWrite(VAT_PIN, HIGH);
  }
  else{
    digitalWrite(VAT_PIN, LOW);
  }
  }

  
void loop(void) {

  delay(200);//this part doesnt matter
}
