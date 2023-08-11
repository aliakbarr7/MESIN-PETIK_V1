//Nama Program    : MESIN PETIK TEH V1.0
//Nama Pembuat    : Ali Akbar
//Tanggal Di Buat : 23/07/2023


#include <Arduino.h>

void sendMessage(String outgoing, byte Node_MesinPetik, byte otherNode);
void onReceive(int packetSize);
void LEDCOLOR(String color);

#include "sub_module/MPU9250_RPY.h"
#include "sub_module/MPU9250.h"
extern MPU9250 IMU;
extern int roll;
extern int pitch;
extern int yaw;
int roll_calibrate = 0;
int pitch_calibrate = 0;
int yaw_calibrate = 0;
int roll_c = 0;
int pitch_c = 0;
int yaw_c = 0;


#define LEDGPS 4
#define LEDKIRI 15
#define LEDKANAN 13
#define LEDRED 25
#define LEDGREEN 33
#define LEDBLUE 32




//Inisialisasi library axp20x (Power Management)
#include <axp20x.h>
AXP20X_Class axp;
//Inisialisasi variabel untuk menyimpan addres axp192
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;
String VBAT;

//Inisialisasi library GPS Neo 6
#include <TinyGPS++.h>
TinyGPSPlus gps;
//Menggunakan pin serial 1 pada esp32
HardwareSerial GPS(1);
//Inisialisasi variabel untuk menampung nilai koordinat dari GPS
String Latitude, Longitude;


//DHT22
#include "DHT.h" 
#define DHTPIN 14
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
//Inisialisasi Variabel Sensor
float suhu;
int kelembaban;



//Inisialisasi library SPI
#include <SPI.h>
//Inisialisasi library LoRa
#include <LoRa.h>
//Inisialisasi pin yang digunakan LoRa => ESP32
#define SS   18     
#define RST  14   
#define DIO0 26   
#define SCK  5
#define MISO 19
#define MOSI 27
//Insialisasi variabel untuk kirim dan terima pesan pada komunikasi LoRa
String RSI;         //Variabel untuk menampung nilai RSI
String outgoing;            //Pesan keluar
byte msgCount = 0;          //Menghitung jumlah pesan keluar
byte Node_Repeater1 = 0xFA;     //Addres Node Gateway
byte Node_MesinPetik = 0xAA;    //Addres Node Telemetri GPS
String pesan;               //Variabel untuk menampung pesan yang akan dikirim ke Gateway


//Inisialisasi variabel untuk menampung nilai millis
unsigned long lastSendTime = 0;
unsigned long timesensor = 0;




void setup() {
  //Memulai komunikasi serial
  Serial.begin(9600);

  //Mulai menjalankan komunikasi I2C
  Wire.begin(21, 22);

  //Mulai menjalankan library DHT22
  dht.begin();

  //Memulai komunikasi SPI
  SPI.begin(SCK, MISO, MOSI, SS);
  //Setting pin LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6)) {            
    Serial.println("Gagal menjalankan LoRa. Periksan wiring rangkaian.");
  }

  //Mulai menjalankan GPS
  GPS.begin(9600, SERIAL_8N1, 34, 12);

  //Mulai menjalankan MPU9250
  MPU9250Setup();


  //Mulai menjalankan library AXP (Power Management)
  int ret = axp.begin(Wire, slave_address);
  if (ret) {
    Serial.println("Ooops, AXP202/AXP192 power chip detected ... Check your wiring!");
  }
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                 AXP202_VBUS_CUR_ADC1 |
                 AXP202_BATT_CUR_ADC1 |
                 AXP202_BATT_VOL_ADC1,
                 true);

  pinMode(LEDGPS, OUTPUT);
  pinMode(LEDKIRI, OUTPUT);
  pinMode(LEDKANAN, OUTPUT);
  pinMode(LEDRED, OUTPUT);
  pinMode(LEDGREEN, OUTPUT);
  pinMode(LEDBLUE, OUTPUT);

  Serial.println("Kalibrasi Sensor 5 Detik...");


  while(millis() < 5000){

     MPU9250Loop();
    
    roll_calibrate = roll;
    pitch_calibrate = pitch;
    yaw_calibrate = yaw;


  }

  Serial.println(roll_calibrate);
  Serial.println(pitch_calibrate);
  Serial.println(yaw_calibrate);


}


void loop() {

  
  

  //Mulai menjalankan MPU9250
  MPU9250Loop();

  roll = roll - roll_calibrate;
  pitch = pitch - pitch_calibrate;
  yaw =  yaw - yaw_calibrate;

  //Membaca Nilai Suhu dan Kelembaban
  suhu = dht.readTemperature();
  kelembaban = dht.readHumidity();


  
  
  if (millis() - lastSendTime > 1000) {
    lastSendTime = millis();  

    LEDCOLOR("RED");

    if (axp.isBatteryConnect()) {
      VBAT = axp.getBattVoltage();
    } else {
      VBAT = "NAN";
    }

    
    if (gps.location.isValid()) {
      Latitude = String(gps.location.lat(), 9);
      Longitude = String(gps.location.lng(), 9);
    } else {
      Latitude = "NAN";
      Longitude = "NAN";
    }
    
    
    pesan = String() + "MKRPPTK0823" + "," + Latitude + "," + Longitude + "," + suhu + "," + kelembaban + "," + roll + "," + pitch + "," + yaw + "," + VBAT + ",*";
    

  
  }


  if (millis() - timesensor > 1000) {
    timesensor = millis(); 

    if (roll >= -5 && roll <= 5){
      digitalWrite(LEDKIRI, HIGH);
      digitalWrite(LEDKANAN, HIGH);
    }

    else if (roll < -5){
      digitalWrite(LEDKIRI, LOW);
      digitalWrite(LEDKANAN, HIGH);
    }

    else if (roll > 5){
      digitalWrite(LEDKIRI, HIGH);
      digitalWrite(LEDKANAN, LOW);
    }
  
  
  }


  else {
    while (GPS.available()) {
      gps.encode(GPS.read());
    }
  }

  onReceive(LoRa.parsePacket());
  
}


void sendMessage(String outgoing, byte Node_MesinPetik, byte otherNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(otherNode);              // add destination address
  LoRa.write(Node_MesinPetik);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
 
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
 
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
 
  String incoming = "";
 
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
 
  if (incomingLength != incoming.length()) {   // check length for error
   // Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }
 
  // if the recipient isn't this device or broadcast,
  if (recipient != Node_MesinPetik && recipient != Node_Repeater1) {
    //Serial.println("This message is not for me.");
    return;                             // skip rest of function
  } 

  if (incoming == "MESINPETIK1"){ 
    sendMessage(pesan,Node_MesinPetik,Node_Repeater1);
    Serial.println(pesan);
    RSI = String(LoRa.packetRssi());
    delay(100);
  }

}


void LEDCOLOR(String color) {
  if (color == "RED") {
    digitalWrite(LEDRED, HIGH);
    digitalWrite(LEDGREEN, LOW);
    digitalWrite(LEDBLUE, LOW);
  } else if (color == "GREEN") {
    digitalWrite(LEDRED, LOW);
    digitalWrite(LEDGREEN, HIGH);
    digitalWrite(LEDBLUE, LOW);
  } else if (color == "BLUE") {
    digitalWrite(LEDRED, LOW);
    digitalWrite(LEDGREEN, LOW);
    digitalWrite(LEDBLUE, HIGH);
  } else if (color == "OFF") {
    digitalWrite(LEDRED, LOW);
    digitalWrite(LEDGREEN, LOW);
    digitalWrite(LEDBLUE, LOW);
  }
}