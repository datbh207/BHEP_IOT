#include <Wire.h>
#include <Adafruit_GFX.h>    //OLED libraries
#include <Adafruit_SSD1306.h> //OLED libraries
#include <Adafruit_MLX90614.h> //Cảm biến nhiệt độ
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include "ESP32Servo.h"

MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0; //Thời điểm nhịp cuối cùng xảy ra
float beatsPerMinute;
int beatAvg;

double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;

double SpO2 = 0;
double ESpO2 = 90.0;//giá trị ban đầu
double FSpO2 = 0.7; //hệ số lọc cho SpO2 ước tính
double frate = 0.95; //bộ lọc thông thấp cho giá trị IR/LED đỏ để loại bỏ thành phần AC
int i = 0;
int Num = 30;
#define FINGER_ON 7000    //Lượng tia hồng ngoại tối thiểu (để xác định ngón tay có bật hay không)
#define MINIMUM_SPO2 90.0 //Lượng oxy trong máu tối thiểu

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

// Biểu đồ nhịp tim
static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,        
  0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
  0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
  0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,
};
// Hình ảnh nhịp tim
static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
  0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
  0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
  0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
  0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
  0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
  0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00
};
// Sơ đồ Oxy
static const unsigned char PROGMEM O2_bmp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x3f, 0xc3, 0xf8, 0x00, 0xff, 0xf3, 0xfc,
  0x03, 0xff, 0xff, 0xfe, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0x7e,
  0x1f, 0x80, 0xff, 0xfc, 0x1f, 0x00, 0x7f, 0xb8, 0x3e, 0x3e, 0x3f, 0xb0, 0x3e, 0x3f, 0x3f, 0xc0,
  0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3e, 0x2f, 0xc0,
  0x3e, 0x3f, 0x0f, 0x80, 0x1f, 0x1c, 0x2f, 0x80, 0x1f, 0x80, 0xcf, 0x80, 0x1f, 0xe3, 0x9f, 0x00,
  0x0f, 0xff, 0x3f, 0x00, 0x07, 0xfe, 0xfe, 0x00, 0x0b, 0xfe, 0x0c, 0x00, 0x1d, 0xff, 0xf8, 0x00,
  0x1e, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00,
  0x0f, 0xe0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int Tonepin = 4; //Chân còi

void setup() {
  Serial.begin(9600);
  Serial.println("System Start");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);
  //檢查
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("Not Found MAX30102");
    while (1);  
  }
  //Tùy Chọn
  byte ledBrightness = 0x7F; //亮度建議=127, Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only(心跳), 2 = Red + IR(血氧)
  int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();

  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  mlx.begin(); // Bật cảm biến nhiệt độ
}

void loop() {
  long irValue = particleSensor.getIR();   // Kiểm tra xem có ngón tay trên cảm biến hay ko
  // Kiểm tra xem ngón tay có trên cảm biến hay ko ( irValue > 7000)
  Serial.print(irValue);
  if (irValue > FINGER_ON ) {
    // Đo nhiệt độ
    float nhietdo = mlx.readObjectTempC(); 
    nhietdo += 1;
    //Đo nhịp tim trung bình
    if (checkForBeat(irValue) == true) {
      //Calculate beatsPerMinute
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {  //Check if the BPM value is within a valid range
        rates[rateSpot++] = (byte)beatsPerMinute;         //Store this reading in the array
        rateSpot %= RATE_SIZE;                            //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    //Đo lượng SpO2
    uint32_t ir, red ;
    double fred, fir;
    if (particleSensor.available()) {
      i++;
      ir = particleSensor.getFIFOIR(); //Đọc Hồng ngoại
      red = particleSensor.getFIFORed(); //Đọc Đèn đỏ
      //Serial.println("red=" + String(red) + ",IR=" + String(ir) + ",i=" + String(i));
      fir = (double)ir;
      fred = (double)red;
      aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
      avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
      sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
      sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level

      if ((i % Num) == 0) {
        double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
        SpO2 = -23.3 * (R - 0.4) + 100;
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
        if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        if (ESpO2 > 100) ESpO2 = 99.9;
        //Serial.print(",SPO2="); Serial.println(ESpO2);
        sumredrms = 0.0; sumirrms = 0.0; SpO2 = 0;
        i = 0;
      }
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

    // In ra Serial
    Serial.print("Nhiet Do: " + String(nhietdo));
    Serial.print(", Bpm:" + String(beatAvg));
    if (beatAvg > 0)  Serial.print(", SPO2:" + String(ESpO2));
    else Serial.print(", SPO2:" + String(ESpO2));
    Serial.println();

    // Hiển thị lên OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    // display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);
    // display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE);

    display.setCursor(0, 10);
    display.print("Nhiet Do: "); display.print(nhietdo);
    display.setCursor(0, 30);
    display.print("BPM :"); display.print(beatAvg); 
    display.setCursor(0, 50);
    display.print("ESpO2: "); display.print(String(ESpO2) + "%");
    display.display();
  }
  else {
    for (byte rx = 0 ; rx < RATE_SIZE ; rx++) rates[rx] = 0;
    beatAvg = 0; // beatsPerMinute = 0; lastBeat = 0;
    avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0; rateSpot = 0;
    SpO2 = 0; ESpO2 = 90.0; 
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(30, 5);
    display.println("Finger");
    display.setCursor(30, 35);
    display.println("Please");
    display.display();
    }
  
}


