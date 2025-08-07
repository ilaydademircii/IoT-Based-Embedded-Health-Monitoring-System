#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "MAX30105.h"
#include <Firebase_ESP_Client.h>
#include <Adafruit_MLX90614.h>

#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// I2C Pin Definitions for ESP8266
#define SDA_PIN D2 // GPIO4
#define SCL_PIN D1 // GPIO5

// GPS Pin Definitions
#define GPS_RX D7        // GPIO13
#define GPS_TX D8        // GPIO15
#define GPS_RESET_PIN D5 // GPIO14

// I2C Addresses ekle
#define MLX90614_ADDRESS 0x5A // MLX90614 I2C address
#define MAX30102_ADDRESS 0x57 // MAX30102 I2C address
#define MPU6050_ADDRESS 0x68  // MPU6050 I2C address

/* * Pin Bağlantıları
 * ESP8266 NodeMCU | MAX30102 | MPU6050 | MLX90614  | GYNEOMV2
 * D2 (GPIO4)      | SDA      | SDA     | SDA       |
 * D1 (GPIO5)      | SCL      | SCL     | SCL       |
 * 3.3V            | VCC      | VCC     | VCC       |
 * GND             | GND      | GND     | GND       | GND
 * 5V              |          |         |           | VCC
 * D7 (GPIO13)     |          |         |           | TX
 * D8 (GPIO15)     |          |         |           | RX
 */

#define FIFO_DATA 0x07
#define MODE_CONFIG 0x09

// Buffer tanımlamaları
#define BUFFER_LENGTH 100 // Geçici tampon bellek boyutu
uint32_t redBuffer[BUFFER_LENGTH];
uint32_t irBuffer[BUFFER_LENGTH];
uint8_t bufferIndex = 0;
unsigned long lastBeat = 0;
float beatsPerMinute = 0;
int beatCount = 0;

// Sensör register tanımlamaları
#define LED_PULSE_AMP 0x0C    // LED pulse amplitude register
#define SPO2_CONFIG 0x0A      // SpO2 configuration register
#define LED_MODE_CONTROL 0x09 // LED mode control register

// MPU6050 register tanımlamaları
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

int16_t AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ;

// MPU6050 Dönüşüm faktörleri
const float ACC_SCALE = 16384.0; // ±2g için
const float GYRO_SCALE = 131.0;  // ±250 °/s için

// Sıcaklık zamanlama değişkenleri
unsigned long lastTempReport = 0;
const unsigned long TEMP_INTERVAL = 5000; // GY-906 için 5 saniye

// Zamanlama değişkenleri
unsigned long lastMAX30102Print = 0;
unsigned long lastMPU6050Print = 0;

// Zamanlama aralıkları
const unsigned long MAX30102_PRINT_INTERVAL = 1000; // MAX30102 için 1 saniye
const unsigned long MPU6050_PRINT_INTERVAL = 3000;  // MPU6050 için 3 saniye

// GPS nesneleri ve değişkenleri
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
unsigned long lastGpsDisplay = 0;
bool gpsConnectionLost = false;
const unsigned long GPS_UPDATE_INTERVAL = 10000;

MAX30105 particleSensor;

//  BPM ve SpO2 değişkenleri
const int STABILITY_THRESHOLD = 10000; // Stabil kabul edilecek maksimum değişim
unsigned long startTime;
long lastIrValue = 0;
const int MIN_BPM = 40;
const int MAX_BPM = 180;
const int MAX_BPM_CHANGE = 20;
const int BPM_WINDOW_SIZE = 10;
float bpmWindow[BPM_WINDOW_SIZE] = {0}; // yumuşatma(smoothing) ve gürültü veya dalgalanma azaltma
int bpmWindowIndex = 0;                 // dairesel buffer
unsigned long lastPeakTime = 0;
float currentBPM = 0;
int validBPMCount = 0;
const int MIN_SIGNAL_QUALITY = 5000;
long irValue = 0;
long redValue = 0;
bool isStable = false;
unsigned long lastStableTime = 0;
const unsigned long STABILITY_TIMEOUT = 3000;
const int SPO2_BUFFER_SIZE = 100;
float lastValidSpO2 = 0;
bool hasValidSpO2 = false;
long peakValue = 0;
long troughValue = 0;

// Add SpO2 smoothing değişkenleri
const int SPO2_WINDOW_SIZE = 5;
float spo2Window[SPO2_WINDOW_SIZE] = {0};
int spo2WindowIndex = 0;
boolean isPeak(long value)
{
  if (bufferIndex < 4)
    return false;

  long avg = 0;
  for (int i = 0; i < 4; i++)
  {
    avg += irBuffer[(bufferIndex - i + BUFFER_LENGTH) % BUFFER_LENGTH];
  }
  avg /= 4;

  if (value > peakValue)
    peakValue = value;
  if (value < troughValue || troughValue == 0)
    troughValue = value;

  long diff = peakValue - troughValue;
  if (diff < 500)
    return false;

  long dynamicThreshold = troughValue + (diff * 75 / 100); // Dinamik eşik, sinyalin dip değerine, genliğin %75’i eklenerek hesaplanır

  return (value > dynamicThreshold);
}

float calculateSpO2(long irACValue, long irDCValue, long redACValue, long redDCValue)
{
  float R = (((float)redACValue / redDCValue) / ((float)irACValue / irDCValue));
  float SpO2 = 110.0 - 25.0 * R;

  if (SpO2 > 100)
    SpO2 = 100;
  if (SpO2 < 0)
    SpO2 = 0;

  if (SpO2 < 80 || SpO2 > 100)
  {
    return -1;
  }

  return SpO2;
}

long calculateAC(long values[], int size)
{
  long max = values[0];
  long min = values[0];
  for (int i = 1; i < size; i++)
  {
    if (values[i] > max)
      max = values[i];
    if (values[i] < min)
      min = values[i];
  }
  return max - min;
}

long calculateDC(long values[], int size)
{
  long sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += values[i];
  }
  return sum / size;
}

void updateAndPrintSpO2(float newSpO2)
{
  if (newSpO2 >= 90 && newSpO2 <= 100)
  {
    spo2Window[spo2WindowIndex] = newSpO2;
    spo2WindowIndex = (spo2WindowIndex + 1) % SPO2_WINDOW_SIZE;

    float sum = 0;
    int count = 0;
    for (int i = 0; i < SPO2_WINDOW_SIZE; i++)
    {
      if (spo2Window[i] > 0)
      {
        sum += spo2Window[i];
        count++;
      }
    }

    if (count > 0)
    {
      float smoothedSpO2 = sum / count;
      lastValidSpO2 = smoothedSpO2;
      hasValidSpO2 = true;

      Serial.print("SpO2: ");
      Serial.print(smoothedSpO2, 1);
      Serial.println("%");
    }
  }
  else if (hasValidSpO2)
  {
    Serial.print("Son geçerli SpO2: ");
    Serial.print(lastValidSpO2, 1);
    Serial.println("%");
  }
}
void scanI2C()
{
  Serial.println("I2C cihazlari taraniyor...");
  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("I2C cihaz bulundu: 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("I2C tarama tamamlandi.");
}

void resetGPS()
{
  if (GPS_RESET_PIN >= 0)
  {
    pinMode(GPS_RESET_PIN, OUTPUT);
    digitalWrite(GPS_RESET_PIN, LOW);
    delay(100);
    digitalWrite(GPS_RESET_PIN, HIGH);
  }
  delay(3000);
}

void configureGPS()
{
  const char *commands[] = {
      "$PMTK251,9600*17",                                  // Baud hızını 9600’e ayarla
      "$PMTK220,1000*1F",                                  // Güncelleme hızını 1Hz’e ayarla
      "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28", // NMEA çıktısını yapılandır
      "$PMTK101*32"};                                      // Sıcak başlatma
  for (const char *cmd : commands)
  {
    gpsSerial.println(cmd);
    delay(250);
  }
}

void setupMLX()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // I2C hızını düşür (100kHz)
  delay(100);

  for (int i = 0; i < 5; i++)
  { // 5 kez yeniden deneme
    if (mlx.begin())
    {
      Serial.println("MLX90614 sensor hazir");
      return;
    }
    Serial.println("MLX90614 baslatilamadi, yeniden deneme...");
    delay(1000);
  }

  Serial.println("MLX90614 baslatilamadi! I2C adresini kontrol edin.");
  scanI2C(); // I2C tarayıcıyı çalıştır
  while (1)
  {
    delay(1000);
  }
}

// WiFi bağlantı bilgileri
const char *ssid = "emir";
const char *password = "12345678a";

// Firebase bağlantı bilgileri
#define API_KEY "AIzaSyDcNHjkdZf1o3pMVkk3kxdIE9aIGxkRFYw"

#define DATABASE_URL "proje1-f9077-default-rtdb.europe-west1.firebasedatabase.app"

// Firebase değişkenleri
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Firebase güncelleme aralığı
unsigned long lastFirebaseUpdate = 0;
const unsigned long FIREBASE_UPDATE_INTERVAL = 5000; // Her 5 saniyede bir güncelle

// Kullanıcı kimlik bilgileri
// #define USER_EMAIL "emir@hotmail.com" // Kullanıcı e-posta adresi
#define USER_EMAIL "ilaydademirci571@gmail.com" // Kullanıcı e-posta adresi
// #define USER_PASSWORD "123456" // Kullanıcı şifresi
#define USER_PASSWORD "Zehra123" // Kullanıcı şifresi
String USER_EMAIL_STR(USER_EMAIL);

// Firebase veri yolu oluşturucu (userEmail'i dönüştürerek kullan)
String getFirebasePath(const char *sensorType)
{
  String userEmailPath(USER_EMAIL_STR);
  userEmailPath.replace(".", "_"); // Noktaları alt çizgi ile değiştir
  return String("/users/") + userEmailPath + "/sensor_data/" + sensorType;
}

void initFirebase()
{
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  config.token_status_callback = TokenStatusCallback();
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("Firebase initialized");
}

// Wifi bağlantısı
void connectToWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Wifi bağlantı kontrolü
void checkWiFiConnection()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectToWiFi();
  }
}
// Firebase bağlantı durum kontrolü için  fonksiyon
bool checkFirebaseConnection()
{
  bool isConnected = false;
  FirebaseJson testJson;
  testJson.set("test", "connection");

  Serial.println("\n----- Firebase Bağlantı Kontrolü -----");

  if (!WiFi.isConnected())
  {
    Serial.println("✗ WiFi bağlantısı yok!");
    return false;
  }
  Serial.println("✓ WiFi bağlı");

  if (!Firebase.ready())
  {
    Serial.println("✗ Firebase hazır değil!");
    return false;
  }
  Serial.println("✓ Firebase servisi hazır");

  // Test verisi gönderme
  if (Firebase.RTDB.setJSON(&fbdo, "/test_connection", &testJson))
  {
    Serial.println("✓ Firebase veritabanına erişilebiliyor");
    isConnected = true;
  }
  else
  {
    Serial.println("✗ Firebase veritabanına erişilemiyor!");
    Serial.print("Hata: ");
    Serial.println(fbdo.errorReason());
  }

  Serial.println("----- Bağlantı Kontrolü Tamamlandı -----\n");
  return isConnected;
}
void setup()
{
  Serial.begin(9600);
  delay(1000); // Sensörün stabil olması için bekleme

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Initialize GPS
  gpsSerial.begin(9600);
  resetGPS();
  configureGPS();

  // MAX30102 yapılandırması
  Wire.beginTransmission(MAX30102_ADDRESS);
  Wire.write(LED_MODE_CONTROL);
  Wire.write(0x40); // Sıfırlama
  Wire.endTransmission();
  delay(100);

  // SpO2 konfigürasyonu
  Wire.beginTransmission(MAX30102_ADDRESS);
  Wire.write(SPO2_CONFIG);
  Wire.write(0x27); // Örnekleme hızı 100Hz
  Wire.endTransmission();

  // LED akımlarını artır
  Wire.beginTransmission(MAX30102_ADDRESS);
  Wire.write(LED_PULSE_AMP);
  Wire.write(0x3F); // IR LED akımı maksimum
  Wire.write(0x3F); // Red LED akımı maksimum
  Wire.endTransmission();

  // Multi-LED modunu ayarla
  Wire.beginTransmission(MAX30102_ADDRESS);
  Wire.write(MODE_CONFIG);
  Wire.write(0x03); // Multi-LED mode
  Wire.endTransmission();

  // MPU6050 başlatma
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0); // Wake up MPU6050
  Wire.endTransmission(true);

  Serial.println("MPU6050 Ready!");

  // MLX90614 için I2C haberleşmeyi başlat
  Wire.beginTransmission(MLX90614_ADDRESS);
  Wire.write(0x06); // MLX90614 EEPROM configuration register
  Wire.write(0x00); // Default configuration
  Wire.endTransmission(true);
  delay(100);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1)
      ;
  }

  setupMLX();

  particleSensor.setup();
  Serial.println("MPU6050 VE Mlx90614 Hazır");

  // Wifi bağlantısını başlat
  WiFi.mode(WIFI_STA);
  connectToWiFi();

  // Firebase başlatma
  initFirebase();
}

void loop()
{
  yield();
  // Wifi bağlantısı ve Firebase iletişiminin kesintiye uğramasını önlemek için

  // Her 30 saniyede bir bağlantı kontrolü yap
  static unsigned long lastConnectionCheck = 0;
  if (millis() - lastConnectionCheck > 30000)
  {
    if (!checkFirebaseConnection())
    {
      Serial.println("Bağlantı sorunu tespit edildi, sistem yeniden başlatılıyor...");
      delay(1000);
      ESP.restart();
    }
    lastConnectionCheck = millis();
  }

  // Firebase bağlantı kontrolü
  if (Firebase.ready() && WiFi.status() == WL_CONNECTED)
  {

    if (millis() - lastTempReport >= TEMP_INTERVAL)
    {
      float objectTemp = mlx.readObjectTempC();
      if (!isnan(objectTemp) && objectTemp <= 45)
      {
        FirebaseJson tempJson;
        tempJson.set("value", objectTemp);
        Firebase.RTDB.setJSON(&fbdo, getFirebasePath("temperature").c_str(), &tempJson);
      }

      lastTempReport = millis();
    }

    // MPU6050 verilerini yazdırma
    if (millis() - lastMPU6050Print >= MPU6050_PRINT_INTERVAL)
    {
      Wire.beginTransmission(MPU6050_ADDRESS);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDRESS, 14, true);

      // Verileri okuma
      AccX = Wire.read() << 8 | Wire.read();
      AccY = Wire.read() << 8 | Wire.read();
      AccZ = Wire.read() << 8 | Wire.read();
      Temp = Wire.read() << 8 | Wire.read();
      GyroX = Wire.read() << 8 | Wire.read();
      GyroY = Wire.read() << 8 | Wire.read();
      GyroZ = Wire.read() << 8 | Wire.read();

      delay(500);

      // MPU6050 verilerini JSON nesnesi olarak ekle veya güncelle
      FirebaseJson accelJson;
      accelJson.set("x", AccX / ACC_SCALE);
      accelJson.set("y", AccY / ACC_SCALE);
      accelJson.set("z", AccZ / ACC_SCALE);
      Firebase.RTDB.setJSON(&fbdo, getFirebasePath("acceleration").c_str(), &accelJson);

      FirebaseJson gyroJson;
      gyroJson.set("x", GyroX / GYRO_SCALE);
      gyroJson.set("y", GyroY / GYRO_SCALE);
      gyroJson.set("z", GyroZ / GYRO_SCALE);
      Firebase.RTDB.setJSON(&fbdo, getFirebasePath("gyroscope").c_str(), &gyroJson);

      lastMPU6050Print = millis();
    }

    // GPS verilerini Firebase yazdırma
    while (gpsSerial.available())
    {
      if (gps.encode(gpsSerial.read()))
      {
        if (millis() - lastGpsDisplay > GPS_UPDATE_INTERVAL)
        {
          if (gps.location.isValid())
          {

            // GPS verilerini JSON nesnesi olarak ekle veya güncelle
            FirebaseJson gpsJson;
            gpsJson.set("latitude", gps.location.lat());
            gpsJson.set("longitude", gps.location.lng());
            gpsJson.set("altitude", gps.altitude.meters());
            Firebase.RTDB.setJSON(&fbdo, getFirebasePath("gps").c_str(), &gpsJson);
          }
          lastGpsDisplay = millis();
        }
      }
    }

    if (millis() - lastMAX30102Print >= MAX30102_PRINT_INTERVAL)
    {

      // Read MAX30102 data
      irValue = particleSensor.getIR();
      redValue = particleSensor.getRed();

      if (irValue < MIN_SIGNAL_QUALITY)
      {
        currentBPM = 0;
        validBPMCount = 0;
        peakValue = 0;
        troughValue = 0;
        lastValidSpO2 = 0;

        delay(20);
      }

      isStable = abs(irValue - lastIrValue) < STABILITY_THRESHOLD;
      lastIrValue = irValue;

      irBuffer[bufferIndex] = irValue;
      bufferIndex = (bufferIndex + 1) % BUFFER_LENGTH;

      if (isStable && isPeak(irValue) && (millis() - lastPeakTime > 600))
      {
        unsigned long peakDelta = millis() - lastPeakTime;
        lastPeakTime = millis();
        float bpm = 60000.0 / peakDelta;

        if (bpm >= MIN_BPM && bpm <= MAX_BPM && (validBPMCount == 0 || abs(bpm - currentBPM) <= MAX_BPM_CHANGE))
        {
          bpmWindow[bpmWindowIndex] = bpm;
          bpmWindowIndex = (bpmWindowIndex + 1) % BPM_WINDOW_SIZE;

          float sum = 0;
          int count = 0;
          for (int i = 0; i < BPM_WINDOW_SIZE; i++)
          {
            if (bpmWindow[i] > 0)
            {
              sum += bpmWindow[i];
              count++;
            }
          }
          if (count > 0)
          {
            currentBPM = sum / count;
            validBPMCount++;
          }
        }

        peakValue = 0;
        troughValue = 0;
      }

      if (!isStable && millis() - lastStableTime > STABILITY_TIMEOUT)
      {
        validBPMCount = 0;
        currentBPM = 0;
      }
      else
      {
        lastStableTime = millis();
      }

      static long irBufferSpO2[SPO2_BUFFER_SIZE];
      static long redBufferSpO2[SPO2_BUFFER_SIZE];
      static int spo2Index = 0;

      irBufferSpO2[spo2Index] = irValue;
      redBufferSpO2[spo2Index] = redValue;
      spo2Index++;

      if (spo2Index >= SPO2_BUFFER_SIZE)
      {
        spo2Index = 0;

        long irAC = calculateAC(irBufferSpO2, SPO2_BUFFER_SIZE);
        long irDC = calculateDC(irBufferSpO2, SPO2_BUFFER_SIZE);
        long redAC = calculateAC(redBufferSpO2, SPO2_BUFFER_SIZE);
        long redDC = calculateDC(redBufferSpO2, SPO2_BUFFER_SIZE);

        if (irAC > 1000 && redAC > 1000)
        {
          float newSpO2 = calculateSpO2(irAC, irDC, redAC, redDC);
          if (newSpO2 > 0)
          {
            updateAndPrintSpO2(newSpO2);
          }
        }
        else
        {
          Serial.println("Sinyal kalitesi düşük, SpO2 hesaplanamadı.");
        }

        // Kalp atış hızı ve SpO2 verilerini JSON nesnesi olarak ekle veya güncelle
        FirebaseJson bpmJson;
        bpmJson.set("value", currentBPM);
        Firebase.RTDB.setJSON(&fbdo, getFirebasePath("heart_rate").c_str(), &bpmJson);

        FirebaseJson spo2Json;
        spo2Json.set("value", lastValidSpO2);
        Firebase.RTDB.setJSON(&fbdo, getFirebasePath("spo2").c_str(), &spo2Json);
      }
    }
  }

  // Check WiFi connection periodically
  checkWiFiConnection();

  delay(10);
  yield(); // İşletim sisteminin arka plan görevlerinin gerçekleştirmesine izin ver
  // böylece sistem kilitlenmesini önleriz
}