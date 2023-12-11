#include <SD.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <Arduino.h>
#include <wdt_samd21.h>
#include <PubSubClient.h>
#include "ArduinoJson.h"

#define DEBUG true
#define MODE_1A

#define DTR_PIN 9
#define RI_PIN 8

#define LTE_PWRKEY_PIN 5
#define LTE_RESET_PIN 6
#define LTE_FLIGHT_PIN 7

#define SD_CS_PIN 4

#define BATTERY_PIN A1

float adc_reference_voltage =3.3;
int adc_val = 0;
float current_percent ;
float current_voltage ;
const float zero_percent_voltage = 3.3;
const float max_voltage = 4.2;
const int number_of_levels = 4095;
float factor = 2;
// const int margin = 10;


String url = "https://webhook.site/e8038649-f9db-4638-9ab6-5430340dafb2";
String LBs_Server = "lbs-simcom.com:3002";
String broker = "tcp://test.mosquitto.org";
String loc = "";
String locGNSS = "";
String loc_LBs = "";
String longitude = "";
String lattitude = "";
String day = "";
String time = "";
String height = "";
String speed = "";
String imei = "";
String content_type = "data";
String topic = "datagps";

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
    String sig = sendData("AT+CSQ",2000,DEBUG);
    //rssi : cuong do tin hieu nhan duoc,ber: ty le loi bit kenh
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

void get_imei(String at_imei)
{
  at_imei.trim();
  int length_imei =  at_imei.length();
  int i=0;
  while(at_imei[i]!=':') i++;
  String imei = at_imei.substring(i+2,length_imei-3);
  imei.trim();

  SerialUSB.println(imei);
  return;
}

//FUNCTION TO GET LATITUDE AND LONGITUDE STRING
void gpsLocation(String local)
{
//  char* loc_2 = local;
//  int p=0;
  bool temp = true;
  while(temp)
  {
    int i=0;
    while(local[i]!=':')i++;

    String loc_2 = local.substring(i+2);

    i=0;
    while(loc_2[i]!=',')i++;
    lattitude = loc_2.substring(0,i);
    // SerialUSB.println(lattitude);

    int j = i+3;
    int k = j;
    while(loc_2[k]!=',')k++;
    longitude = loc_2.substring(j,k);
    // SerialUSB.println(longitude); 

    int n = k+3;
    int m = n;
    while(loc_2[m]!=',')m++;
    day = loc_2.substring(n,m);
    // SerialUSB.println(day); 

    int p = m+1;
    int q = p;
    while(loc_2[q]!=',')q++;
    time = loc_2.substring(p,q);
    // SerialUSB.println(time);

    int a = q+1;
    int b = a;
    while(loc_2[b]!=',')b++;
    height = loc_2.substring(a,b);
    // SerialUSB.println(height);

    int c = b+1;
    int d = c;
    while(loc_2[d]!=',')d++;
    speed = loc_2.substring(c,d);
    // SerialUSB.println(speed);

    return;
  }
}

// LBs (GSM) su dung khi khong co song ve tinh GPS
void LBs(String local)
{
  while(1)
  {
    local.trim();
    int h = local.length();
    int i=0;
    while(local[i]!=',')i++;

    String loc_2 = local.substring(i+1);

    i=0;
    while(loc_2[i]!=',')i++;
    lattitude = loc_2.substring(0,i);
    // SerialUSB.println(lattitude);

    int j = i+1;
    int k = j;
    while(loc_2[k]!=',')k++;
    longitude = loc_2.substring(j,k);
    // SerialUSB.println(longitude); 

    int n = k+1;
    int m = n;
    while(loc_2[m]!=',')m++;
    loc_2.substring(n,m);


    int p = m+1;
    int q = p;
    while(loc_2[q]!=',')q++;
    day = loc_2.substring(p,q);
    // SerialUSB.println(day);

    time = loc_2.substring(h-q,h);
    // SerialUSB.println(time);

    return;
    
  }
}
// LBs (GSM) su dung khi khong co song ve tinh GPS


//HTTP Post Data String

void send_data_http(String local)
{
  int lengthloc = local.length();
  sendData("AT+HTTPINIT\r\n", 500, DEBUG);
  String http_str = "AT+HTTPPARA=\"URL\",\"" + url + "\"\r\n";
  sendData(http_str, 500, DEBUG);
  
  String http_data = "AT+HTTPDATA=";
  String Sdata= http_data + lengthloc + ",500\r\n";
  
  sendData(Sdata,500,DEBUG);
  sendData(local,500,DEBUG);

  String action = "AT+HTTPACTION=1\", \"HTTP_PEER_CLOSED\" \r\n";
  sendData("AT+HTTPACTION=1\r\n", 500, DEBUG);
  sendData("AT+HTTPTERM\r\n", 500, DEBUG);
}

//HTTP Post Data String

// MQTT Callback
void mqttCallback(char *topic, byte *payload, unsigned int length)
{

  SerialUSB.print("Co tin nhan moi tu topic: ");
  SerialUSB.println(topic);
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);

  if (String(topic) == "mode")
  {
    if (message == "Sleep")
    {
      SerialUSB.print("Che do Sleep");
    }
    if (message == "DeepSleep")
    {
      SerialUSB.print("Che do DeepSleep");
    }
  }
}
// MQTT Callback


//MQTT AT SIM

void send_data_mqtt(String local)
{
  
  int lengthloc = local.length();
  // sendData("AT+CGDCONT=1,\"IP\"",3000,DEBUG);
  sendData("AT+CMQTTSTART",500,DEBUG);
  sendData("AT+CMQTTACCQ=0,\"client1\"",500,DEBUG);// client 1
  String mqttconnect = "AT+CMQTTCONNECT=0,\""+ broker + "\",60,1";
  sendData(mqttconnect,500,DEBUG);
  sendData("AT+CMQTTTOPIC=0,7",500,DEBUG);
  sendData(topic,500,DEBUG);
  String payload = "AT+CMQTTPAYLOAD=0,";
  String mqttpayload = payload + lengthloc;
  sendData(mqttpayload,500,DEBUG);
  sendData(local,500,DEBUG);
  sendData("AT+CMQTTPUB=0,1,60",500,DEBUG);//Gui tin len may chu
  sendData("AT+CMQTTSUBTOPIC=0,4,1",500,DEBUG);
  sendData("mode",500,DEBUG);
  sendData("AT+CMQTTSUB=0",500,DEBUG);
  // sendData("AT+CMQTTSUB=0,\""+ mode + "\",2,1",3000,DEBUG);
  // sendData("AT+CMQTTDISC=0,60",3000,DEBUG);// Ngat ket noi khoi may chu 
  // sendData("AT+CMQTTREL=0",3000,DEBUG);// Giai phong client
  // sendData("AT+CMQTTSTOP",3000,DEBUG);
}
//MQTT AT SIM

// Write SD
void write_SD(String data_write)
{
  File dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println(data_write);
      SerialUSB.println("data.csv write");
      dataFile.close();
      }
    else {
      SerialUSB.println("error opening data.csv write");
    }
}
// Write SD

//Read SD
void read_SD()
{
  File dataFile = SD.open("data.csv");
  if (dataFile && dataFile.size() > 2) {

  // SerialUSB.println("Data size:");
  // SerialUSB.println(dataFile.size());

  // read from the file until there's nothing else in it:
  while (dataFile.available()) {
  String write_SD = dataFile.readString();
  // SerialUSB.println(data_SD);
  send_data_http(write_SD);  
  }
  // close the file:
  dataFile.close();
  SD.remove("data.csv");
  } else {
  // if the file didn't open, print an error:
  SerialUSB.println("error opening data.csv read");
  }
}
//Read SD

// Sleep
void Sleep_MCU_Sim(int time_Sleep)
{
  sendData("AT+CFUN=0",500,DEBUG);
  sendData("АT+CSCLK=1",500,DEBUG);
  digitalWrite(DTR_PIN, HIGH);// UART
  LowPower.sleep(time_Sleep);
  sendData("AT+CFUN=1",500,DEBUG);
  sendData("АT+CSCLK=0",500,DEBUG);
  digitalWrite(DTR_PIN, LOW);
  delay(1000);
}

//Deep Sleep
void DeepSleep_MCU_Sim(int time_DeepSleep)
{
  sendData("AT+CFUN=4",500,DEBUG);
  sendData("АT+CSCLK=1",500,DEBUG);
  digitalWrite(DTR_PIN, HIGH);// UART
  LowPower.deepSleep(time_DeepSleep);
  sendData("AT+CFUN=1",500,DEBUG);
  sendData("АT+CSCLK=0",500,DEBUG);
  digitalWrite(DTR_PIN, LOW);
  delay(1000);
}


void setup(){
  SerialUSB.begin(115200);
  //while (!SerialUSB)
//  {
    ; // wait for Arduino serial Monitor port to connect
//  }

  delay(100);

  Serial1.begin(115200);
  analogReadResolution(12); 
  //INITIALIZING GSM MODULE

  pinMode(LTE_RESET_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);

  pinMode(LTE_PWRKEY_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(LTE_PWRKEY_PIN, HIGH);
  delay(2000);
  digitalWrite(LTE_PWRKEY_PIN, LOW);

  pinMode(LTE_FLIGHT_PIN, OUTPUT); //FLight mode control
  digitalWrite(LTE_FLIGHT_PIN, LOW); //Normal Mode
  // digitalWrite(LTE_FLIGHT_PIN, HIGH);//Flight Mode

  //SD
  SerialUSB.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS_PIN)) {
    SerialUSB.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  SerialUSB.println("card initialized.");
  //SD

  SerialUSB.println("Maduino Zero 4G Test Start!");


  SerialUSB.println(sendData("AT+CGMM\r\n", 3000, DEBUG));
  sendData("AT+CPIN?\r\n",3000,DEBUG);
  
  sendData("AT+COPS?\r\n",3000,DEBUG);

  sendData("AT+CNUM\r\n",3000,DEBUG);

  imei = sendData("AT+SIMEI?",1000,DEBUG);
  // SerialUSB.println(imei);
  // get_imei(imei);
  // SerialUSB.println(imei);


  sendData("AT+CSCLK=0",3000,DEBUG);
  // sendData("AT+CHSICSLEEP=0",3000,DEBUG);
  // sendData("AT+CFUN=7",3000,DEBUG);
  // sendData("AT+CFUN=6",3000,DEBUG);
  sendData("AT+CFUN=1",3000,DEBUG);

  //mqtt
  // init_mqtt();
  //mqtt

  //LBs
  // sendData("AT+CNETSTOP",3000,DEBUG);
  sendData("AT+CNETSTART",3000,DEBUG);
  String lbs_server = "AT+CLBSCFG=1,3,\"" + LBs_Server + "\"\r\n";

  sendData(lbs_server,3000,DEBUG);
  //LBs

  sendData("AT+CGPSPMD=65407",3000,DEBUG);
  //INITIALIZING GPS MODULE
  sendData("AT+CGPS=0",3000,DEBUG);
  sendData("AT+CGPS=1",3000,DEBUG);
  delay(60000);
  wdt_init ( WDT_CONFIG_PER_8K );
  
}

void loop(){

  // get_imei(imei);

  // Percent Pin
    adc_val = analogRead(BATTERY_PIN);
    // SerialUSB.println(adc_val);

    current_voltage = factor * adc_val * (float)adc_reference_voltage / (float)number_of_levels;
    // SerialUSB.println(current_voltage);

    current_percent = 100 * (current_voltage - zero_percent_voltage) / (max_voltage - zero_percent_voltage);
    // SerialUSB.println(current_percent);
    
    if (current_percent < 0)
    {
      current_percent = 0;
    }
    if (current_percent > 100)
    {
      current_percent = 100;
    }


//CHECKING SIGNAL STRENGTH, SET MODE
    if(check_signal()>=10)
    {
      loc = sendData("AT+CGPSINFO\r\n",500,DEBUG);
      // locGNSS = sendData("AT+CGNSSINFO\r\n",500,DEBUG);

      // SerialUSB.print(locGNSS);
      int check_gps = loc.length();
      // SerialUSB.print(check_gps);
      
      
      if(check_gps > 41)
      {
        gpsLocation(loc);
        wdt_disable ( );
        String dataGPS = day + "," + time + "," + lattitude + "," + longitude + "," + height + "," + speed;

        String sendingStr = "";
        StaticJsonDocument<200> doc;
        doc["day"] = day;
        doc["time"] = time;
        doc["lat"] = lattitude;
        doc["lon"] = longitude;
        doc["height"] = height;
        doc["speed"] = speed;
        serializeJson(doc, sendingStr);
        // SerialUSB.print(sendingStr);

        // send_data_http(dataGPS);

        //Mode Speed
        // if(speed!="0.0"){
        //   write_SD(dataGPS);
        // }
        // if(speed=="0.0"){
        //   read_SD();
        // }
        //Mode Speed

        //Set mode theo muc pin
        if(current_percent < 50)
        {
          send_data_http(dataGPS);
          Sleep_MCU_Sim(300000);
        }
        if(current_percent < 10)
        {
          send_data_http(dataGPS);
          DeepSleep_MCU_Sim(300000);
        }
        else
        {
          // SerialUSB.print(imei);
          // send_data_http(dataGPS);  
          send_data_mqtt(sendingStr);

          delay(10000);
        }
        //Set mode theo muc pin
      }
      else
      {
        wdt_disable ( );
        wdt_init ( WDT_CONFIG_PER_16K );
        // SerialUSB.print(imei);
        loc_LBs = sendData("AT+CLBS=4",500,DEBUG); //Location Based Service
        // SerialUSB.println(loc_LBs);
        LBs(loc_LBs);
        wdt_disable ( );
        String data_LBs = day + "," + time + "," + lattitude + "," + longitude;
        //
        String sendingStr = "";
        StaticJsonDocument<128> doc;
        doc["name"] = "Device1";
        doc["iconColor"] = "red";
        doc["icon"] = "bicycle";
        doc["day"] = day;
        doc["time"] = time;
        doc["lat"] = lattitude;
        doc["lon"] = longitude;
        
        serializeJson(doc, sendingStr);
        // SerialUSB.print(sendingStr);
        // send_data_http(data_LBs);
        send_data_mqtt(sendingStr);
        
        // write_SD(data_LBs);
        delay(10000);
      }
    }    
}