#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

 
bool initBluetooth_()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
 
}
 
String getDeviceAddress_() {
    const uint8_t* point = esp_bt_dev_get_address();
    String addressString = "";

    for (int i = 0; i < 6; i++) {
        char str[3];
        sprintf(str, "%02X", (int)point[i]);
        addressString += str;

        if (i < 5) {
            addressString += ":";
        }
    }
    return addressString;
}

void setup_10() {
  Serial.begin(9600);
  
  // Start I2C Communication SDA = 5 and SCL = 4 on Wemos Lolin32 ESP32 with built-in SSD1306 OLED
  Wire.begin(5, 4);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000); // Pause for 2 seconds
  Serial.println(F("SSD1306 allocation succeeded"));

  initBluetooth_();
   String mac_adr = getDeviceAddress_();
   Serial.println(mac_adr);


  // Clear the buffer.
display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("BLE MAC Address:");
  display.setCursor(0, 20);
  display.println(mac_adr);
  display.display(); 
}
 
void loop_10() {
  
}