#include <SD.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <Arduino.h>
#include <wdt_samd21.h>



#define DEBUG true
#define MODE_1A

#define DTR_PIN 9
#define RI_PIN 8

#define LTE_PWRKEY_PIN 5
#define LTE_RESET_PIN 6
#define LTE_FLIGHT_PIN 7

#define SD_CS_PIN 4

#define BATTERY_PIN A1


int adc_val = 0;
float current_percent ;
float current_voltage ;
const float zero_percent_voltage = 3.3;
const float max_voltage = 4.2;
const int number_of_levels = 1023;
// const int margin = 10;


String url = "https://webhook.site/61a33674-7404-4a9a-9082-84215a659903";
String loc = "";
String longitude = "";
String lattitude = "";
String day = "";
String time = "";
String height = "";
String speed = "";
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



//Post Data String

void data_map(String local,float percent_pin)
{
  int lengthloc = local.length();
  sendData("AT+HTTPINIT\r\n", 500, DEBUG);

  String http_str = "AT+HTTPPARA=\"URL\",\"" + url + "\"\r\n";
  // String content = "AT+HTTPPARA=\"CONTENT\",\"" + content_type + "\"\r\n";
  sendData(http_str, 500, DEBUG);
  // sendData(content, 500, DEBUG);
  
  
  String DataLeng = "AT+HTTPDATA=";
  String Sdata= DataLeng + lengthloc + ",500\r\n";
  
  sendData(Sdata,500,DEBUG);
  String data = local + "," + percent_pin;
  sendData(data,500,DEBUG);

  String action = "AT+HTTPACTION=1\", \"HTTP_PEER_CLOSED\" \r\n";
  sendData("AT+HTTPACTION=1\r\n", 500, DEBUG);
  sendData("AT+HTTPTERM\r\n", 3000, DEBUG);
}

//Post Data String




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
  delay(1000);
  digitalWrite(LTE_PWRKEY_PIN, HIGH);
  delay(2000);
  digitalWrite(LTE_PWRKEY_PIN, LOW);

  pinMode(LTE_FLIGHT_PIN, OUTPUT);
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


  // sendData("AT+CSCLK=0",3000,DEBUG);
  // sendData("AT+CHSICSLEEP=0",3000,DEBUG);
  // sendData("AT+CFUN=7",3000,DEBUG);
  // sendData("AT+CFUN=6",3000,DEBUG);
  // sendData("AT+CFUN=1",3000,DEBUG);



  //INITIALIZING GPS MODULE
  sendData("AT+CGPS=0",3000,DEBUG);
  sendData("AT+CGPS=1",3000,DEBUG);
  delay(60000);
  wdt_init ( WDT_CONFIG_PER_8K );
  
}

void loop(){

  // Percent Pin
    adc_val = analogRead(BATTERY_PIN);
    // SerialUSB.println(adc_val);

    // uint16_t lo = max(adc_val - margin, 0);
    // uint16_t hi = min(adc_val + margin, UINT16_MAX); 

    // LowPower.attachAdcInterrupt(BATTERY_PIN, repetitionsIncrease, ADC_INT_OUTSIDE, lo, hi);

    current_voltage = 2 * adc_val * zero_percent_voltage / number_of_levels;
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


//CHECKING SIGNAL STRENGTH, CHECK CURRENT PERCENT PIN SET MODE
    if(check_signal()>=10)
    {
      loc = sendData("AT+CGPSINFO\r\n",1000,DEBUG);
      gpsLocation(loc);
      
      if(lattitude!=""&& longitude!="")
      {
        wdt_disable ( );
        String dataGPS = lattitude + "," + longitude + "," + day + "," + time + "," + height + "," + speed + "," + current_percent;


        //Set mode khi co su thay doi ve van toc
        //SD
        if(speed!="0.0") {
          File dataFile = SD.open("datalog2.csv", FILE_WRITE);
          // if the file is available, write to it:
          if (dataFile) {
            dataFile.println(dataGPS);
            dataFile.close();
          }
          // if the file isn't open, pop up an error:
          else {
            SerialUSB.println("error opening datalog2.csv write");
          }
        }


  //AT send file
        // sendData("AT+HTTPINIT",3000,DEBUG);
        // String http_str = "AT+HTTPPARA=\"URL\",\"" + url + "\"\r\n";
        // sendData(http_str, 3000, DEBUG);
        // String http_post_file = "AT+HTTPPOSTFILE=\"datalog2.csv\",0,0\r\n";
        // sendData(http_post_file,3000,DEBUG);
        // sendData("AT+HTTPHEAD",3000,DEBUG);

        // String http_read_file = "AT+HTTPREADFILE=\"datalog2.dat\"\r\n";
        // sendData(http_read_file,3000,DEBUG);

        // sendData("AT+HTTPHEAD",3000,DEBUG);
        // sendData("AT+HTTPTERM ",3000,DEBUG);
   //AT send file


        // re-open the file for reading:
        if(speed=="0.0"){
          File dataFile = SD.open("datalog2.csv");
          if (dataFile && dataFile.size() > 2) {

            // SerialUSB.println("Data size:");
            // SerialUSB.println(dataFile.size());

            // read from the file until there's nothing else in it:
            while (dataFile.available()) {
              String data_SD = dataFile.readString();
              // SerialUSB.println(data_SD);
              data_map(data_SD,current_percent);
              
            }
            // close the file:
            dataFile.close();
            SD.remove("datalog2.csv");
          } else {
            // if the file didn't open, print an error:
            SerialUSB.println("error opening datalog2.csv read");
          }
        }



        

        //Remove dataSD
        
        //SD
        //Set mode khi co su thay doi ve van toc

        //Set mode theo muc pin
        if(current_percent < 50)
        {
          
          data_map(dataGPS,current_percent);
          //Sleep mode CPU sleep
          LowPower.sleep(10000);
          delay(5000);
          //Sleep mode CPU sleep
          // LowPower.sleep(10000);

        //Sleep mode CPU Sleep, Sim Sleep USB (FIX)
          // sendData("AT+CHSICSLEEP=1",3000,DEBUG);
          // sendData("AT+CFUN=7",30000,DEBUG);
          
          //TEST SLEEP SIM MINIMUM FUNC + UART 
          // sendData("AT+CFUN=0",3000,DEBUG);
          // sendData("AT+CSCLK=1",3000,DEBUG);
          //TEST SLEEP SIM MINIMUM FUNC + UART 

          // SerialUSB.println("zzZ");
          // LowPower.sleep(60000);  //3600000
          // sendData("AT&D0",3000,DEBUG);
          // sendData("AT+CFUN=1",3000,DEBUG);
          // sendData("AT+CHSICSLEEP=0",3000,DEBUG);

          //TEST SLEEP SIM MINIMUM FUNC + UART 
          // sendData("AT+CFUN=1",3000,DEBUG);
          // sendData("AT+CSCLK=0",3000,DEBUG);
          //TEST SLEEP SIM MINIMUM FUNC + UART 

          // SerialUSB.println("Wakeup");
          // LowPower.deepSleep(5000);
          // LowPower.detachAdcInterrupt();
        //Sleep mode CPU Sleep, Sim Sleep USB

        }
        if(current_percent < 10)
        {
          // Deep sleep mode
          data_map(dataGPS,current_percent);
          LowPower.deepSleep(300000);
          //Config
          //Config
          delay(5000);
        }
        else
        {
          // data_map(dataGPS,current_percent);
          // test_pin1(current_percent);
          // LowPower.sleep(1200000);
          delay(1000);
        }
        //Set mode theo muc pin
      }
      else
      {

      }
    }    
}