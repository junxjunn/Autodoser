#define tdsPin A0
#define phPin A1
#define tdsArrayLength 40
#define phArrayLength 40
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6oSVyXwQV"
#define BLYNK_TEMPLATE_NAME "Auto Doser Prototype Monitor"
#define BLYNK_DEVICE_NAME "Auto Doser Prototype"
#define BLYNK_AUTH_TOKEN "jTfY1mY9R06S4OxfCjOGXmiCPr0cPZwJ"

#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <SimpleDHT.h>
#include <OneWire.h>

int tdsArray[tdsArrayLength];
int tdsArrayIndex = 0;
int phArray[phArrayLength];
int phArrayIndex = 0;

SimpleDHT22 dht22(D5);
int DS18S20_Pin = D3;
int M1 = D7;
int M2 = D9;

unsigned long postStart = 700000;
unsigned long dhtStart = 0;
unsigned long readStart = 0;
unsigned long printStart = 0;
unsigned long ds18b20Start = 0;
unsigned long tdsStart = 0;
unsigned long phStart = 0;
unsigned long doseStart = 0;

static float phValue = 0;
static float phVoltage = 0;
static float tdsValue = 0;
static float tdsVoltage = 0;
static float phThreshold = 7.0;
static float tdsThreshold = 1.30;

float water_temp = 0;
float air_temp = 0;
float air_humidity = 0;

static unsigned long postDelay = 720000;
static unsigned long dhtDelay = 1000;
static unsigned long ds18b20Delay = 1000;
static unsigned long readDelay = 2000;
static unsigned long printDelay = 800;
static unsigned long tdsDelay = 20;
static unsigned long phDelay = 20;
static unsigned long doseDelay = 360000;

///////////////////////////////////////
//    WiFi settings and Host name    //
//                                   //
  const char* ssid = "CityFarm_maxis";
  const char* password = "iamacityfarmer";
  const char* host = "app.cityfarm.my";
  const int httpPort = 80;
//                                   //
///////////////////////////////////////

/////////////////////////
//  PH Function   //
double phAverageArray(int* phArr, int phNum){
  int ph, phMax, phMin;
  double phAvg;
  long phAmount = 0;
  if (phNum <=0){
    return 0;
  }
  if (phNum < 5){
    for (ph=0; ph<phNum; ph++){
      phAmount += phArr[ph];
     }
     phAvg = phAmount/phNum;
    return phAvg;
    }
  else{
    if (phArr[0] < phArr[1]){
      phMin = phArr[0];
      phMax = phArr[1];
    }
    else{
      phMin = phArr[1];
      phMax = phArr[0];
    }
     for (ph=2; ph<phNum; ph++){
      if(phArr[ph]<phMin){
        phAmount += phMin; 
        phMin = phArr[ph];
      }
      else{
        if(phArr[ph]>phMax){
          phAmount += phMax;
          phMax=phArr[ph];
        }
        else{
        phAmount += phArr[ph];
        }
      }
     }
     phAvg = (double)phAmount/(phNum-2);
  }
  return phAvg;
}
/////

//TDS Funnction//
double tdsAverageArray(int* tdsArr, int tdsNum){
  int tds, tdsMax, tdsMin;
  double tdsAvg;
  long tdsAmount = 0;
  if (tdsNum <=0){
    return 0;
  }
  if (tdsNum < 5){
    for (tds=0; tds<tdsNum; tds++){
      tdsAmount += tdsArr[tds];
     }
     tdsAvg = tdsAmount/tdsNum;
    return tdsAvg;
    }
  else{
    if (tdsArr[0] < tdsArr[1]){
      tdsMin = tdsArr[0];
      tdsMax = tdsArr[1];
    }
    else{
      tdsMin = tdsArr[1];
      tdsMax = tdsArr[0];
    }
     for (tds=2; tds<tdsNum; tds++){
      if(tdsArr[tds]<tdsMin){
        tdsAmount += tdsMin; 
        tdsMin = tdsArr[tds];
      }
      else{
        if(tdsArr[tds]>tdsMax){
          tdsAmount += tdsMax;
          tdsMax=tdsArr[tds];
        }
        else{
        tdsAmount += tdsArr[tds];
        }
      }
     }
     tdsAvg = (double)tdsAmount/(tdsNum-2);
  }
  return tdsAvg;
}

//////////////////////////////////////////////////
//                  DS18B20                     //
OneWire ds(DS18S20_Pin);
float getTemp(){

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      return -1;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      return -1;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      return -1;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);


  for (int i = 0; i < 9; i++) { 
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); 
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
//                                              //
//////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);
  pinMode(D7, OUTPUT);
  pinMode(D9, OUTPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  ////////////////////////////////////////////
  //           Begin WiFi connection        //
  //                                        //
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
  //                                        //
  ////////////////////////////////////////////
}

void loop() {
  unsigned long postCurrent = millis();
  unsigned long dhtCurrent = millis();
  unsigned long ds18b20Current = millis();
  unsigned long readCurrent = millis();
  unsigned long printCurrent = millis();
  unsigned long tdsCurrent = millis();
  unsigned long phCurrent = millis();
  unsigned long doseCurrent = millis();

//TDS
if (tdsCurrent - tdsStart >= tdsDelay){
    tdsArray[tdsArrayIndex++] = analogRead(tdsPin);
    if (tdsArrayIndex == tdsArrayLength){
      tdsArrayIndex = 0;
      tdsValue = tdsAverageArray(tdsArray, tdsArrayLength)*3.3/4096;
      tdsValue = 0.33461*pow(tdsValue,5)-1.73810*pow(tdsValue,4)+3.29958*pow(tdsValue,3)-2.72133*pow(tdsValue,2)+1.68313*tdsValue + 0.03301;
      //readStart = readCurrent;
    }
    tdsStart = tdsCurrent;
}
//PH
if (phCurrent - phStart >= phDelay){
    phArray[phArrayIndex++] = analogRead(phPin);
    if (phArrayIndex == phArrayLength){
      phArrayIndex = 0;
      phValue = phAverageArray(phArray, phArrayLength)*3.3/4096;
      phValue = phValue*(-6.2464) + 15.734;
    }
    phStart = phCurrent;
  }

if (readCurrent - readStart >= readDelay){
  //////////////////////////////////////////////
  //  DHT22 for air temperature and humidity  //
  //float air_temp = 0; 
  //float air_humidity = 0;
  
  //if (dhtCurrent - dhtStart >= dhtDelay){
  dht22.read2(&air_temp, &air_humidity, NULL);

  Serial.print("Air Temperature (C): ");
  Serial.println(air_temp, 1);
  Blynk.virtualWrite(V3, air_temp);

  Serial.print("Air Humidity (%): ");
  Serial.println(air_humidity, 1);
  Blynk.virtualWrite(V4, air_humidity);
  //dhtStart = dhtCurrent;
  //}
  //                                         //
  /////////////////////////////////////////////

  ///////////////////////////////
  //          DS18B20          //  
  //if(ds18b20Current - ds18b20Start >= ds18b20Delay){
  //float water_temp = 0;
  float temperature = getTemp();
  Serial.print("Water temperature (C): ");
  Serial.println(temperature);

  if (temperature >= 0) {
    water_temp = temperature;
  }
  Blynk.virtualWrite(V2, water_temp);
  //ds18b20Start = ds18b20Current;
  //}
  //                            //
  ////////////////////////////////
  
  //PH
  Serial.print("pH Value: ");
  Serial.println(phValue, 2);
  Blynk.virtualWrite(V0, phValue);
  
  //TDS
  Serial.print("TDS Value: ");
  Serial.print(tdsValue, 2);
  Serial.println("mS/cm");
  Blynk.virtualWrite(V1, tdsValue);

  Blynk.run();
  readStart = readCurrent;
}
if (doseCurrent - doseStart >= doseDelay){
  if (phValue >= phThreshold){
        digitalWrite(M2, HIGH);
        delay(1000);
        digitalWrite(M2, LOW);
      Serial.println("pH down solution is dosed.");
    }

    if (tdsValue <= tdsThreshold){
        digitalWrite(M1, HIGH);
        delay(2500);
        digitalWrite(M1, LOW);
        Serial.println("AB nutrient solution is dosed.");
      }
      doseStart = doseCurrent;
}

if (postCurrent - postStart >= postDelay){
  /////////////////////////////////////////////////////////////////
  //               Uploading data to server                      //
  if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;

        Serial.println("Done");
        Serial.print("Connecting to ");
        Serial.println(host);

        if (!client.connect(host, httpPort)) {
            Serial.println("Connection failed");
            return;
        }

        //String ec = "0.0";
        
         String data = "{ ";
          data += "\"ph\": " + String(phValue) + ", ";
          data += "\"ec\": " + String(tdsValue) + ", ";
          data += "\"water_temp\": " + String(water_temp) + ", ";
          data += "\"air_temp\": " + String(air_temp) + ", ";
          data += "\"humidity\": " + String(air_humidity) + ", ";
          data += "\"device_id\": \"123abc\"}";

        // Make an HTTPS POST request
        client.print("POST /device/data HTTP/1.1\r\n");
        client.print("Host: ");
        client.print(host);
        client.print("\r\n");
        client.print("Content-Type: application/json\r\n");
        client.print("Content-Length: ");
        client.print(data.length());
        client.print("\r\n\r\n");
        client.print(data);

        while (client.available()) {
            String line = client.readStringUntil('\r');
            Serial.print(line);
        }

        client.stop();
        Serial.println("POST Success!");
    } else {
        Serial.println("WiFi not connected");
    }
    postStart = postCurrent;
    }
}
