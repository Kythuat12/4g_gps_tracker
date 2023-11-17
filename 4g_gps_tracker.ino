#include <ArduinoLowPower.h>

#include <wdt_samd21.h>
#include <stdio.h>
#include <string.h>


#define DEBUG true
#define MODE_1A

#define DTR_PIN 9
#define RI_PIN 8

#define LTE_PWRKEY_PIN 5
#define LTE_RESET_PIN 6
#define LTE_FLIGHT_PIN 7
#define BATTERY_PIN A1

int adc_val = 0;
float current_voltage ;
float current_percent ;
const float zero_percent_voltage = 3.3;
const float max_voltage = 4.2;
const int number_of_levels = 1023;

String url = "https://webhook.site/29bc0fdc-ebb8-42f4-9efa-59eeb0697bd6";
String loc = "";
String longitude = "";
String lattitude = "";
String content_type = "data";

//FUNCTION TO PASS AT COMMAND

String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  Serial1.println(command);
  
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      response += c;
    }
  }
  if (debug)
  {
    SerialUSB.print(response);
  }
  return response;
}

//FUNCTION TO CHECK SIGNAL STRENGTH

int check_signal(void)
{
  while(1)
  {
    String sig = sendData("AT+CSQ",3000,DEBUG);
    int i=0;
    String strength;
    while(sig[i]!=':')i++;

    String loc_2 = sig.substring(i+2);

    i=0;
    while(loc_2[i]!=',')i++;

    strength = loc_2.substring(0,i);

    int strength_1 = strength.toInt();
    return strength_1;
  }
}

//FUNCTION TO GET LATITUDE AND LONGITUDE STRING
void gpsLocation(String local)
{
//  char* loc_2 = local;
//  int p=0;
  
  while(1)
  {
    int i=0;
    while(local[i]!=':')i++;

    String loc_2 = local.substring(i+2);

    i=0;
    while(loc_2[i]!=',')i++;

    lattitude = loc_2.substring(0,i);
    SerialUSB.println(lattitude);

    int j = i+3;
    int k = j;

    while(loc_2[k]!=',')k++;

    longitude = loc_2.substring(j,k);
    SerialUSB.println(longitude); 

    return;
    
  }
}

// Data 

void data_map(String local, const int percent_pin)
{
  int leng1 = local.length();
  local.remove(leng1-4,4);
  sendData("AT+HTTPINIT\r\n", 3000, DEBUG);
  int leng2 = local.length();

  String http_str = "AT+HTTPPARA=\"URL\",\"" + url + "\"\r\n";
  String content = "AT+HTTPPARA=\"CONTENT\",\"" + content_type + "\"\r\n";
  sendData(http_str, 3000, DEBUG);
  sendData(content, 3000, DEBUG);
  local.trim();
  
  
  String DataLeng = "AT+HTTPDATA=";
  String Sdata= DataLeng + leng2 + ",1000\r\n";
  
  sendData(Sdata,3000,DEBUG);
  String data = local + "," + percent_pin;
  sendData(data,1000,DEBUG);

  String action = "AT+HTTPACTION=1\", \"HTTP_PEER_CLOSED\" \r\n";
  sendData("AT+HTTPACTION=1\r\n", 3000, DEBUG);
  sendData("AT+HTTPTERM\r\n", 3000, DEBUG);
}



void setup(){
  SerialUSB.begin(115200);
  //while (!SerialUSB)
//  {
    ; // wait for Arduino serial Monitor port to connect
//  }

  delay(100);

  Serial1.begin(115200);

  // wdt_init ( WDT_CONFIG_PER_16K );
  //INITIALIZING GSM MODULE

  pinMode(LTE_RESET_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);

  pinMode(LTE_PWRKEY_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);
  delay(100);
  digitalWrite(LTE_PWRKEY_PIN, HIGH);
  delay(2000);
  digitalWrite(LTE_PWRKEY_PIN, LOW);

  pinMode(LTE_FLIGHT_PIN, OUTPUT);
  digitalWrite(LTE_FLIGHT_PIN, LOW); //Normal Mode
  // digitalWrite(LTE_FLIGHT_PIN, HIGH);//Flight Mode

  SerialUSB.println("Maduino Zero 4G Test Start!");

  

  SerialUSB.println(sendData("AT+CGMM\r\n", 3000, DEBUG));
  sendData("AT+CPIN?\r\n",3000,DEBUG);
  
  sendData("AT+COPS?\r\n",3000,DEBUG);

  sendData("AT+CNUM\r\n",3000,DEBUG);

  //INITIALIZING GPS MODULE
  sendData("AT+CGPS=0",3000,DEBUG);
  sendData("AT+CGPS=1",3000,DEBUG);
  delay(60000);
  wdt_init ( WDT_CONFIG_PER_8K );
  
}

void loop(){

  // Percent Pin
    adc_val = analogRead(BATTERY_PIN);
    SerialUSB.println(adc_val);
    current_voltage = 2 * adc_val * zero_percent_voltage / number_of_levels;
    current_percent = 100 * (current_voltage - zero_percent_voltage) / (max_voltage - zero_percent_voltage);
    if (current_percent < 0)
    {
      current_percent = 0;
    }
    if (current_percent > 100)
    {
      current_percent = 100;
    }

//CHECKING SIGNAL STRENGTH
    if(check_signal()>=10)
    {
      loc = sendData("AT+CGPSINFO\r\n",3000,DEBUG);
      gpsLocation(loc);
      if(lattitude!=""&& longitude!="")
      {
        wdt_disable ( );
        data_map(loc,current_percent);
      }
      else
      {
        // wdt_init ( WDT_CONFIG_PER_2K );
      }
      // delay(5000);  
    }    
}
