
/*************************************************************
  Download latest ERa library here:
    https://github.com/eoh-jsc/era-lib/releases/latest
    https://www.arduino.cc/reference/en/libraries/era
    https://registry.platformio.org/libraries/eoh-ltd/ERa/installation

    ERa website:                https://e-ra.io
    ERa blog:                   https://iotasia.org
    ERa forum:                  https://forum.eoh.io
    Follow us:                  https://www.fb.com/EoHPlatform
 *************************************************************/

// Enable debug console
// Set CORE_DEBUG_LEVEL = 3 first
// #define ERA_DEBUG

#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "631f087c-e170-4762-9113-cd6a417b5410"

#include <Arduino.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>

// wifi information
const char ssid[] = "iPhone 5s";
const char pass[] = "123456789";


#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       34 // LM35 pin
#define buzzer 27         // buzzer pin
#define led 25            // led pin

ERaTimer timer;

float tempC;
int value;
long long ms=0;

// get value from V1 virtual pin to turn on/off the led
ERA_WRITE(V1)
{
  //get integer value
  value=param.getInt();
}

/* This function print uptime every second */
void timerEvent() 
{
  // send the temp value to V0
  ERa.virtualWrite(V0,tempC);
  ERA_LOG("Timer", "Uptime: %d", ERaMillis() / 1000L);
}

void setup() 
{
    /* Setup debug console */
    Serial.begin(115200);

    // Connect wifi
    ERa.begin(ssid, pass);

    /* Setup timer called function every second */
    timer.setInterval(1000L, timerEvent);

    // configuration user 
    pinMode(PIN_LM35, INPUT);
    pinMode(buzzer,OUTPUT);
    pinMode(led,OUTPUT);
}

void loop() {
    ERa.run();
    timer.run();

  // read anlog signal
  int adcVal = analogRead(PIN_LM35);

  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  tempC = roundf(milliVolt *100 / 10)/100-26; //Round to 2 decimal places

  // fire alarm
  if(tempC>=70)
    {
      //turn on fire alarm system
      digitalWrite(buzzer,HIGH);
      //blink led interval 500ms
      if(millis()-ms>=500)
      {
      //toggle led pin
      digitalWrite(led,!digitalRead(led));
      //update value for ms
      ms=millis();
      }
    }
  else
  {
    // turn off the fire alarm system
    digitalWrite(buzzer,LOW); 
    // button on
    if(value==1)
    {
      digitalWrite(led,HIGH);
    }
    //button off
    else 
    {
      digitalWrite(led,LOW);
    }
  }
}

