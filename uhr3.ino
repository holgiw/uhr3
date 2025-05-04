
// Prozessor
//#define ESP32_D1 // -- funktioniert nicht !!!! SPI & LittleFs ???
#define ESP32_S2

// TFT
//#define GC9A01
#define GC9D01
//#define ILI9341 


//#define DEBUG

#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <time.h>
#include <set>
#include <base64.h>

#include "build_defs.h"

TFT_eSPI tft = TFT_eSPI();
WebServer server(80);
Preferences preferences;
bool showSecondHand = true;


#ifdef ESP32_D1  // Mini D1 ESP32 oder LOLIN32 (der mit dem Akku Anschluß)
// ##############################################################################
// wires
//               ESP32 PIN    TFT
//               3.3V         vcc     3v3             red
//               GND          gnd     ground          black
//see C : \Users\hwage\Documents\Arduino\libraries\TFT_eSPI\user_setups\Setup304__ESP32S3_GC9D01.h


#define LED_BOARD_2 2    // pin2  LED ESP32 D1 Mini
#define LED_BOARD_22 22  // pin22 LED LOLIN32

#define ADC_3V 33
#define ADC_PIN 34
#define ADC_GND 14

#define BUTTON1 16

#ifdef GC9D01
#define TFT_Backlight 21  // Backlight
#endif
#endif


#ifdef ESP32_S2  // Lolin S2 Pico
// ##############################################################################
// wires
//               ESP32 PIN    TFT
//               3.3V         vcc     3v3             red
//               GND          gnd     ground          blue
//  see C:\Users\hwage\Documents\Arduino\libraries\TFT_eSPI\user_setups\Setup304__ESP32S3_GC9D01.h




#define LED_BOARD_2 15 // BUILTIN LED

#define ADC_3V 1
#define ADC_PIN 2
#define ADC_GND 4

#define BUTTON1 16

#ifdef GC9D01
#define TFT_Backlight 3  // Backlight
#define BACKLIGHT_CHANNEL 0  // PWM channel
#define BACKLIGHT_FREQ 5000
#define BACKLIGHT_RESOLUTION 8
#endif

#endif

#ifdef GC9A01
#include "clock_default_240.h"

#define TFT_WIDTH 240
#define TFT_HEIGHT 240

#define CLOCK_WIDTH 240   
#define CLOCK_HEIGHT 240

#define HAND_WIDTH 21
#define HAND_HEIGHT 131

#define TFT_TEXT_SIZE 2
#endif

#ifdef GC9D01
#include "clock_default_160.h"

#define TFT_WIDTH 160
#define TFT_HEIGHT 160

#define CLOCK_WIDTH 160
#define CLOCK_HEIGHT 160

#define HAND_WIDTH 13
#define HAND_HEIGHT 86

#define TFT_TEXT_SIZE 1
#endif

#ifdef ILI9341
#include "clock_default_240.h"

#define TFT_WIDTH 160
#define TFT_HEIGHT 160

#define CLOCK_WIDTH 240
#define CLOCK_HEIGHT 240

#define HAND_WIDTH 21
#define HAND_HEIGHT 131

#define TFT_TEXT_SIZE 2
#endif

#define TRANSPARENT_COLOR 0x0120

TFT_eSprite backgroundSprite = TFT_eSprite(&tft);
TFT_eSprite hourHandSprite = TFT_eSprite(&tft);
TFT_eSprite minuteHandSprite = TFT_eSprite(&tft);
TFT_eSprite secondHandSprite = TFT_eSprite(&tft);

String wifi_ssid;
String wifi_pass;
String wifi_ssid2;
String wifi_pass2;
String timezone = "CET-1CEST,M3.5.0,M10.5.0/3";
String selectedBackground = "/face_default.bmp";

unsigned long secStartMicros = 0;
int lastSec = -1;
uint8_t bahnhofTick = 0;
uint32_t bahnhofLastMillis = 0;
bool bahnhofWaiting = false;

uint16_t rowBuffer[CLOCK_WIDTH];

uint16_t adc_min = 0;
uint16_t adc_max = 0;   
bool use_adc = false; 
bool photoresistorFound = false;


uint8_t currentBrightness = 255;
uint8_t lastAppliedBrightness = 255;
uint8_t targetBrightness = 255;
int lowThreshold = 40;
int highThreshold = 60;
uint8_t minBrightness = 100;  // 
uint8_t maxBrightness = 255;  // Obergrenze 

#define ADC_SMOOTHING 10
int adcHistory[ADC_SMOOTHING];
int adcIndex = 0;


File uploadFile;
String uploadFilePath = "";
bool uploadSuccess = false;

uint16_t setPixelBrightness(uint16_t pixel) {

#ifdef GC9D01
    return pixel;
#endif

    if (currentBrightness == 255 || pixel == TRANSPARENT_COLOR || pixel == 0x0000) {
        return pixel;
    }

    // Farbe anpassen
    uint8_t r = (pixel & 0xF800) >> 11;
    uint8_t g = (pixel & 0x07E0) >> 5;
    uint8_t b = (pixel & 0x001F);

    r = r * currentBrightness / 255;
    g = g * currentBrightness / 255;
    b = b * currentBrightness / 255;

    return (r << 11) | (g << 5) | b;

}

void loadClockFace() {
    if (LittleFS.exists(selectedBackground)) {
        File bmp = LittleFS.open(selectedBackground, "r");
        if (bmp) {
            loadBmpToSprite(&backgroundSprite, selectedBackground.c_str());
            bmp.close();
          //  Serial.println("[BG] Custom background loaded: " + selectedBackground);
        }
        else {
          //  Serial.println("[BG] Failed to open custom background.");
            for (int y = 0; y < CLOCK_WIDTH; y++) {

                for (int x = 0; x < CLOCK_WIDTH; x++) {
                    uint16_t px = clockFace[y * CLOCK_WIDTH + x];

                    rowBuffer[x] = setPixelBrightness(px);

                }
                backgroundSprite.pushImage(0, y, CLOCK_WIDTH, 1, rowBuffer);
            }
        }
    }
    else {
      //  Serial.println("[BG] Background not found, using default.");
        for (int y = 0; y < CLOCK_WIDTH; y++) {

            for (int x = 0; x < CLOCK_WIDTH; x++) {
                uint16_t px = clockFace[y * CLOCK_WIDTH + x];

                rowBuffer[x] = setPixelBrightness(px);

            }
            backgroundSprite.pushImage(0, y, CLOCK_WIDTH, 1, rowBuffer);
        }
     //   Serial.println("[BG] Default background loaded.");
    }

}

void loadHandSprites() {
    String set = preferences.getString("handset", "");
#ifdef DEBUG
    Serial.println("[HANDS] Active hand set: " + set);
#endif
    bool usedDefault = false;
    if (set != "") {
        struct HandConfig {
            String label;
            TFT_eSprite* sprite;
            const uint16_t* fallback;
        } hands[3] = {
            {"hour", &hourHandSprite, handHour},
            {"minute", &minuteHandSprite, handMinute},
            {"second", &secondHandSprite, handSecond}
        };

        for (auto& h : hands) {
            String path = "/hand_" + set + "_" + h.label + ".bmp";
#ifdef DEBUG
            Serial.println("[HANDS] Looking for: " + path);
#endif

            if (LittleFS.exists(path)) {
                if (!loadHandBmp(h.sprite, path.c_str(), HAND_WIDTH, HAND_HEIGHT)) {
                    for (int y = 0; y < HAND_HEIGHT; y++) {

                        for (int x = 0; x < HAND_WIDTH; x++) {
                            uint16_t px = h.fallback[y * HAND_WIDTH + x];

                            rowBuffer[x] = setPixelBrightness(px);

                        }
                        h.sprite->pushImage(0, y, HAND_WIDTH, 1, rowBuffer);
                    }
                    usedDefault = true;
                    Serial.println("[HANDS] Failed to load " + h.label + ", fallback used.");
                }
                else {
#ifdef DEBUG
                    Serial.println("[HANDS] Loaded " + h.label);
#endif
                }
                // Serial.println("found");
            }
            else {
                h.sprite->pushImage(0, 0, HAND_WIDTH, HAND_HEIGHT, h.fallback);
                usedDefault = true;
                Serial.println("[HANDS] Missing " + h.label + ", using default.");
            }
        }

#ifdef DEBUG
        if (!usedDefault) {
            Serial.println("[HANDS] Loaded handset: " + set);
        }
        else {
            Serial.println("[HANDS] Incomplete set, used default for missing hands.");
        }
#endif
    }
    else {
        for (int y = 0; y < HAND_HEIGHT; y++) {

            for (int x = 0; x < HAND_WIDTH; x++) {
                uint16_t px = handHour[y * HAND_WIDTH + x];
                rowBuffer[x] = setPixelBrightness(px);
            }
            hourHandSprite.pushImage(0, y, HAND_WIDTH, 1, rowBuffer);
        }
        for (int y = 0; y < HAND_HEIGHT; y++) {

            for (int x = 0; x < HAND_WIDTH; x++) {
                uint16_t px = handMinute[y * HAND_WIDTH + x];
                rowBuffer[x] = setPixelBrightness(px);
            }
            minuteHandSprite.pushImage(0, y, HAND_WIDTH, 1, rowBuffer);
        }
        for (int y = 0; y < HAND_HEIGHT; y++) {

            for (int x = 0; x < HAND_WIDTH; x++) {
                uint16_t px = handSecond[y * HAND_WIDTH + x];
                rowBuffer[x] = setPixelBrightness(px);
            }
            secondHandSprite.pushImage(0, y, HAND_WIDTH, 1, rowBuffer);
        }
#ifdef DEBUG
        Serial.println("[HANDS] No set selected, using defaults.");
#endif
    }
}

// Hilfsfunktion zum Laden von Zeiger-BMPs 
bool loadHandBmp(TFT_eSprite* sprite, const char* filename, int width, int height) {
    File bmp = LittleFS.open(filename, "r");
    if (!bmp) return false;

    uint8_t header[54];
    if (bmp.read(header, 54) != 54 || header[0] != 'B' || header[1] != 'M') {
        bmp.close();
        return false;
    }

    int32_t bmpWidth = *(int32_t*)&header[18];
    int32_t bmpHeight = *(int32_t*)&header[22];
    uint16_t bpp = *(uint16_t*)&header[28];
    uint32_t offset = *(uint32_t*)&header[10];

    if (bmpWidth != width || abs(bmpHeight) != height || bpp != 16) {
        bmp.close();
        return false;
    }

    bool flip = bmpHeight > 0;
    bmpHeight = abs(bmpHeight);

    bmp.seek(offset);
    int rowSize = ((width * 2 + 3) / 4) * 4;
    for (int y = 0; y < bmpHeight; y++) {
        int row = flip ? bmpHeight - 1 - y : y;
        uint8_t rowBuffer[rowSize];
        if (bmp.read(rowBuffer, rowSize) != rowSize) break;

        uint16_t* pixelData = (uint16_t*)rowBuffer;
        for (int x = 0; x < width; x++) {

            if (pixelData[x] == 0xFFFF) {
                pixelData[x] = TRANSPARENT_COLOR;
            }

            pixelData[x] = setPixelBrightness(pixelData[x]);

        }
        sprite->pushImage(0, row, width, 1, (uint16_t*)rowBuffer, TRANSPARENT_COLOR);
    }

    bmp.close();
    return true;
}

void loop() {
    server.handleClient();
    checkWiFiReconnect();
    updateClock();
    updateBrightness();
    checkNightlyTimeSync();
    delay(20);
    yield();
}

void updateClock() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    bool bahnhofMode = preferences.getBool("bahnhof", true);
    int actualSec = timeinfo.tm_sec;
    static int lastSec = -1;

    float secAngle;
    if (bahnhofMode) {
        static uint8_t bahnhofTick = 0;
        static uint32_t bahnhofLastMillis = 0;
        static bool bahnhofWaiting = false;

        if (bahnhofLastMillis == 0) {
            bahnhofTick = actualSec;
            bahnhofLastMillis = millis();
            bahnhofWaiting = false;
        }

        if (!bahnhofWaiting) {
            if (millis() - bahnhofLastMillis >= 966) {
                bahnhofTick++;
                bahnhofLastMillis += 966;
                if (bahnhofTick >= 60) {
                    bahnhofTick = 60;
                    bahnhofWaiting = true;
                }
            }
        }
        else {
            if (actualSec == 0) {
                bahnhofTick = 0;
                bahnhofWaiting = false;
                bahnhofLastMillis = millis();
            }
        }

        float subTick = (millis() - bahnhofLastMillis) / 966.0f;
        if (subTick > 1.0f) subTick = 1.0f;
        float smoothSec = (bahnhofTick >= 60) ? 60.0f : bahnhofTick + easeInOutSine(subTick);
        secAngle = smoothSec * 6.0f;
    }
    else {
        if (actualSec != lastSec) {
            lastSec = actualSec;
        }
        secAngle = actualSec * 6.0f;
    }

    float smoothMin = bahnhofMode ? timeinfo.tm_min : (timeinfo.tm_min + timeinfo.tm_sec / 60.0f);
    float minAngle = smoothMin * 6.0f;
    float hourAngle = (timeinfo.tm_hour % 12 + smoothMin / 60.0f) * 30.0f;

   // backgroundSprite.fillSprite(TFT_BLACK);
    loadClockFace();

    hourHandSprite.pushRotated(&backgroundSprite, hourAngle, TRANSPARENT_COLOR);
    minuteHandSprite.pushRotated(&backgroundSprite, minAngle, TRANSPARENT_COLOR);
    if (showSecondHand) {
        secondHandSprite.pushRotated(&backgroundSprite, secAngle, TRANSPARENT_COLOR);
    }

    uint16_t color = preferences.getUInt("centerColor", TFT_RED);
    uint8_t size = preferences.getUInt("centerSize", 6);

    if (size > 0) {
        color = setPixelBrightness(color);
        backgroundSprite.fillCircle(CLOCK_WIDTH / 2, CLOCK_HEIGHT / 2, size, color);
    }
    
    backgroundSprite.pushSprite(0, 0);
}
void updateBrightness() {

    if (use_adc) {

        if (currentBrightness != lastAppliedBrightness) {
            loadClockFace();
            loadHandSprites();
            lastAppliedBrightness = currentBrightness;
        }

        int adcRaw = analogRead(ADC_PIN);
        adcHistory[adcIndex] = adcRaw;
        adcIndex = (adcIndex + 1) % ADC_SMOOTHING;

        int avg = 0;
        for (int i = 0; i < ADC_SMOOTHING; i++) avg += adcHistory[i];
        avg /= ADC_SMOOTHING;

        int lightPercent = map(avg, 0, 4095, 5, 100);
       
        if (lightPercent < lowThreshold && targetBrightness != minBrightness) targetBrightness = minBrightness;

        else if (lightPercent > highThreshold && targetBrightness != maxBrightness) targetBrightness = maxBrightness;


        if (currentBrightness != targetBrightness) {
            if (currentBrightness < targetBrightness) currentBrightness++;
            else currentBrightness--;
        }
    }
    else {
        currentBrightness = 255;    
        targetBrightness = 255;
    }

#ifdef TFT_Backlight
    ledcWrite(TFT_Backlight, currentBrightness);  // 0–255

    // invertiert
   // ledcWrite(TFT_Backlight, 255 - currentBrightness);

   // Serial.println(255 - currentBrightness);
#endif

}

float easeInOutSine(float t) {
    return -(cos(PI * t) - 1.0f) / 2.0f;
}

String encodeBmpToBase64(const uint16_t* data, int width, int height) {
    const int headerSize = 54;
    const int rowSize = ((width * 2 + 3) / 4) * 4;
    const int dataSize = rowSize * height;
    const int fileSize = headerSize + dataSize;

    uint8_t* bmpData = new uint8_t[fileSize];
    memset(bmpData, 0, fileSize);

    bmpData[0] = 'B'; bmpData[1] = 'M';
    *(uint32_t*)&bmpData[2] = fileSize;
    *(uint32_t*)&bmpData[10] = headerSize;
    *(uint32_t*)&bmpData[14] = 40;
    *(int32_t*)&bmpData[18] = width;
    *(int32_t*)&bmpData[22] = -height;
    *(uint16_t*)&bmpData[26] = 1;
    *(uint16_t*)&bmpData[28] = 16;
    *(uint32_t*)&bmpData[34] = dataSize;

    for (int y = 0; y < height; y++) {
        uint8_t* rowPtr = bmpData + headerSize + y * rowSize;
        for (int x = 0; x < width; x++) {
            uint16_t px = data[y * width + x];
            if (px == TRANSPARENT_COLOR) px = 0xFFFF; // transparent → weiß für Vorschau
            ((uint16_t*)rowPtr)[x] = px;
        }
    }

    String result = base64::encode(bmpData, fileSize);
    delete[] bmpData;

    return result;
}

//
// NTP-Zeitsynchronisation um 02:00:05 und 03:00:05
//
void checkNightlyTimeSync() {
    static bool triggered2 = false;
    static bool triggered3 = false;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    if (timeinfo.tm_hour == 2 && timeinfo.tm_min == 0 && timeinfo.tm_sec == 5 && !triggered2) {
        Serial.println("[TIME SYNC] Triggered at 02:00:05");
        setupNTP();
        triggered2 = true;
    }

    if (timeinfo.tm_hour == 3 && timeinfo.tm_min == 0 && timeinfo.tm_sec == 5 && !triggered3) {
        Serial.println("[TIME SYNC] Triggered at 03:00:05");
        setupNTP();
        triggered3 = true;
    }

    if (timeinfo.tm_hour == 4 && triggered2 && triggered3) {
        triggered2 = false;
        triggered3 = false;
    }
}

void checkWiFiReconnect() {
    static unsigned long lastAttempt = 0;
    if (WiFi.status() == WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - lastAttempt < 30000) return;
    lastAttempt = now;

    Serial.println("[WiFi] Disconnected. Attempting reconnect...");
    WiFi.disconnect();
    connectWiFi(true);
}

void setup() {

    Serial.begin(115200);
    delay(500);

    preferences.begin("clock", false);

    pinMode(BUTTON1, INPUT_PULLDOWN);

    // auf Fotowiderstand prüfen
#ifdef ADC_3V
    analogReadResolution(12);

    // ADC +3,3 / GND über GPIO
    pinMode(ADC_GND, OUTPUT);
    pinMode(ADC_3V, OUTPUT);

    digitalWrite(ADC_GND, false);
    digitalWrite(ADC_3V, false);
    delay(10);
    adc_min = analogRead(ADC_PIN);

    digitalWrite(ADC_GND, true);
    digitalWrite(ADC_3V, true);
    delay(10);
    adc_max = analogRead(ADC_PIN);

    Serial.printf("ADC min: %d max: %d\n", adc_min, adc_max);

    if (adc_min < 2000 && adc_max > 2000) {
        Serial.println("found photoresistor");
        digitalWrite(ADC_GND, 0);
        digitalWrite(ADC_3V, 1);
        use_adc = true;
        photoresistorFound = true;
        // evtl überschreiben
        use_adc = preferences.getBool("use_adc", true);
    }
    else {
        pinMode(ADC_GND, INPUT);
        pinMode(ADC_3V, INPUT);
        use_adc = false;
    }
    
#else
    use_adc = false;
#endif

    minBrightness = preferences.getUChar("minBrightness", 100);
    maxBrightness = preferences.getUChar("maxBrightness", 255);

    pinMode(LED_BOARD_2, OUTPUT); digitalWrite(LED_BOARD_2, HIGH);

    String hostname = "Clock-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    WiFi.setHostname(hostname.c_str());
    Serial.println("[WiFi] Hostname set to: " + hostname);
    

    if (!LittleFS.begin(true)) {
        Serial.println("[LittleFS] Mount Failed");
    }
    else {
        Serial.println("[LittleFS] Listing all files in root:");
        File root = LittleFS.open("/");
        File entry = root.openNextFile();
        while (entry) {
            Serial.printf(" - %s (%d bytes)  ", entry.name(), entry.size());
            entry = root.openNextFile();
        }
    }

    tft.init();
    delay(50);
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);

#ifdef GC9A01
    uint8_t rotation = preferences.getUChar("storientation", 0);
    tft.setRotation(rotation);
    Serial.printf("[TFT] Using stored rotation: %d\n", rotation);
#endif

#ifdef TFT_Backlight
    pinMode(TFT_Backlight, OUTPUT);
    ledcAttach(TFT_Backlight, BACKLIGHT_FREQ, BACKLIGHT_RESOLUTION);
    ledcWrite(TFT_Backlight, 255);
#endif


    backgroundSprite.createSprite(CLOCK_WIDTH, CLOCK_HEIGHT);
    backgroundSprite.setSwapBytes(true);
    backgroundSprite.setColorDepth(16);

    hourHandSprite.createSprite(HAND_WIDTH, HAND_HEIGHT);
    hourHandSprite.setSwapBytes(true);
    hourHandSprite.setColorDepth(16);
    hourHandSprite.setPivot(HAND_WIDTH / 2, HAND_HEIGHT * 0.77);

    minuteHandSprite.createSprite(HAND_WIDTH, HAND_HEIGHT);
    minuteHandSprite.setSwapBytes(true);
    minuteHandSprite.setColorDepth(16);
    minuteHandSprite.setPivot(HAND_WIDTH / 2, HAND_HEIGHT * 0.77);

    secondHandSprite.createSprite(HAND_WIDTH, HAND_HEIGHT);
    secondHandSprite.setSwapBytes(true);
    secondHandSprite.setColorDepth(16);
    secondHandSprite.setPivot(HAND_WIDTH / 2, HAND_HEIGHT * 0.77);

    loadClockFace();
    loadHandSprites();

    showSecondHand = preferences.getBool("secondhand", true);
      

    wifi_ssid = preferences.getString("ssid", "");
    wifi_pass = preferences.getString("pass", "");
    wifi_ssid2 = preferences.getString("ssid2", "");
    wifi_pass2 = preferences.getString("pass2", "");
     
    selectedBackground = preferences.getString("background", "/faces/default");

    bool apMode = digitalRead(BUTTON1) == HIGH;
    if (!apMode) apMode = !connectWiFi(true);
    if (apMode) startAP();

    if (!apMode) setupNTP();
    setupWebServer();
    server.begin();

    digitalWrite(LED_BOARD_2, LOW);
}

void startAP() {
#ifdef TFT_BACKLIGHT
    ledcWrite(TFT_Backlight, 255);
#endif
    WiFi.softAP("clock123", "clock123");
    Serial.println("[WiFi] Started Access Point: clock123");

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextSize(TFT_TEXT_SIZE);
    tft.setCursor(20, (CLOCK_HEIGHT / 2) - (CLOCK_HEIGHT / 8)) ;
    tft.println("AP active");
    tft.setCursor(20, (CLOCK_HEIGHT / 2));
    tft.println("clock123 clock123");
    tft.setCursor(20, (CLOCK_HEIGHT / 2 ) + (CLOCK_HEIGHT / 8));
    tft.println(WiFi.softAPIP());
}

bool connectWiFi(bool verbose_mode) {
#ifdef TFT_BACKLIGHT
    ledcWrite(TFT_Backlight, 255);
#endif
    if (wifi_ssid == "" && wifi_ssid2 == "") return false;

    Serial.println(wifi_ssid);
    // Serial.println(wifi_pass);
    Serial.println(wifi_ssid2);
    // Serial.println(wifi_pass2);
           
    Serial.println("[WiFi] Trying primary SSID...");

    if (verbose_mode) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(TFT_TEXT_SIZE);
        tft.setCursor(20, (CLOCK_HEIGHT / 2) - (CLOCK_HEIGHT / 8));
        tft.println("Connect to:");
        tft.setCursor(20, (CLOCK_HEIGHT / 2));
        tft.println(wifi_ssid);
    }
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(200);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WiFi] Connected to primary: " + wifi_ssid);
        if (verbose_mode) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextSize(TFT_TEXT_SIZE);
            tft.setCursor(20, (CLOCK_HEIGHT / 2) - (CLOCK_HEIGHT / 8));
            tft.println("Connected to:");
            tft.setCursor(20, (CLOCK_HEIGHT / 2) );
            tft.println(wifi_ssid);
            tft.setCursor(20, (CLOCK_HEIGHT / 2) + (CLOCK_HEIGHT / 8));
            tft.println(WiFi.localIP());
        }
        delay(3000);
        if (!WiFi.softAPgetStationNum()) updateClock();
        return true;
    }

    Serial.println("[WiFi] Primary failed, trying secondary SSID...");

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(TFT_TEXT_SIZE);
    tft.setCursor(20, (CLOCK_HEIGHT / 2) - (CLOCK_HEIGHT / 8));
    tft.println("Connect to:.");
    tft.setCursor(20, (CLOCK_HEIGHT / 2));
    tft.println(wifi_ssid2);

    WiFi.begin(wifi_ssid2.c_str(), wifi_pass2.c_str());
    start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
        delay(200);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WiFi] Connected to : " + wifi_ssid2);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(TFT_TEXT_SIZE);
        tft.setCursor(20, (CLOCK_HEIGHT / 2) - (CLOCK_HEIGHT / 8));
        tft.println("Connected to");
        tft.setCursor(20, (CLOCK_HEIGHT / 2) );
        tft.println(wifi_ssid2);
        tft.setCursor(20, ((CLOCK_HEIGHT / 2) +  (CLOCK_HEIGHT / 8)));
        tft.println(WiFi.localIP());
        delay(5000);
        if (!WiFi.softAPgetStationNum()) updateClock();
        return true;
    }

    Serial.println("[WiFi] Both connections failed.");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(TFT_TEXT_SIZE);
    tft.setCursor(20, (CLOCK_HEIGHT / 2) - (CLOCK_HEIGHT / 8));
    tft.println("WiFi failed");
    tft.setCursor(20, CLOCK_HEIGHT / 2);
    tft.println("AP Mode active");
    return false;    
}


void setupNTP() {
     
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextSize(TFT_TEXT_SIZE);
    tft.setCursor(10, (CLOCK_HEIGHT / 2));

    String timezone = preferences.getString("timezone", "DE");

    if (timezone == "DE") timezone = "CET-1CEST,M3.5.0,M10.5.0/3";         // auto
    else if (timezone == "DE_S") timezone = "CEST-2";                      // permanent summer
    else if (timezone == "DE_W") timezone = "CET-1";                       // permanent winter
    else if (timezone == "UK") timezone = "GMT0BST,M3.5.0/1,M10.5.0";      // auto
    else if (timezone == "UK_S") timezone = "BST-1";                       // permanent summer
    else if (timezone == "UK_W") timezone = "GMT0";                        // permanent winter

    //Serial.printf("[NTP] Timezone: %s\n", timezone.c_str());


    //Serial.println("[NTP] " + timezone);
    configTzTime(timezone.c_str(), "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    int attempts = 0;
    while (!getLocalTime(&timeinfo, 5000) && attempts < 10) {
        attempts++;
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setCursor(10, (CLOCK_HEIGHT / 2));
        tft.printf("NTP failed (%d/10)...", attempts);
        delay(2000);
    }
    if (attempts >= 10) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setCursor(10, (CLOCK_HEIGHT / 2));
        tft.println("NTP timeout! Using last known time.");
        delay(3000);
    }
}

void setupWebServer() {
   
    server.on("/applydisplaysettings", HTTP_POST, []() {
        // Save to Preferences
        bool bahnhof = server.hasArg("enableBahnhof");
        bool second = server.hasArg("showSecond");
        preferences.putBool("bahnhof", bahnhof);
        preferences.putBool("secondhand", second);

        if (server.hasArg("rotation")) {
            uint8_t rot = server.arg("rotation").toInt();
            if (rot <= 3) {
                preferences.putUChar("storientation", rot);
                tft.setRotation(rot); // sofort anwenden
                loadClockFace();      // neu zeichnen mit neuer Ausrichtung
                loadHandSprites();
            }
        }

        timezone = server.arg("timezone");
        preferences.putString("timezone", timezone);
        
        setupNTP();


        // Apply runtime variables
        showSecondHand = second;

        server.send(200, "text/html",
            "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='3; url=/'><title>Gespeichert</title></head>"
            "<body style='font-family:Arial;text-align:center;'><h2>Saved</h2><p>Back...</p></body></html>");
        });

   



    server.on("/brightness", HTTP_POST, []() {
        String html = "<!DOCTYPE html><html><head><title>Brightness Settings</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>body{font-family:Arial;text-align:center;}input{margin:8px;padding:8px;width:80%;}</style></head><body>";
        html += "<h2>Brightness Settings</h2><form method='POST' action='/save_brightness'>";

        if (photoresistorFound) {
            html += "<label><input type='checkbox' name='use_adc' value='1' " + String(use_adc ? "checked" : "") + "> Enable Auto Brightness</label><br>";

            html += "<label>Low Threshold (0 - 100):</label><br><input name='lowThreshold' type='number' min='0' max='100' value='" + String(lowThreshold) + "'><br>";
            html += "<label>High Threshold (0 - 100):</label><br><input name='highThreshold' type='number' min='0' max='100' value='" + String(highThreshold) + "'><br>";
        }

        html += "<label>Min Brightness (0 - 255):</label><br><input name='minBrightness' type='number' min='0' max='255' value='" + String(minBrightness) + "'><br>";

        html += "<label>Max Brightness (0 - 255):</label><br><input name='maxBrightness' type='number' min='0' max='255' value='" + String(maxBrightness) + "'><br>";

        html += "<button type='submit'>Save</button></form>";
        html += "<br><a href='/'>Back</a></body></html>";
        server.send(200, "text/html", html);
        });

    server.on("/brightness", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head><title>Brightness Settings</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>body{font-family:Arial;text-align:center;}input{margin:8px;padding:8px;width:80%;}</style></head><body>";
        html += "<h2>Brightness Settings</h2><form method='POST' action='/save_brightness'>";

        if (photoresistorFound) {
            html += "<label><input type='checkbox' name='use_adc' value='1' " + String(use_adc ? "checked" : "") + "> Enable Auto Brightness</label><br>";

            html += "<label>Low Threshold (0 - 100):</label><br><input name='lowThreshold' type='number' min='0' max='100' value='" + String(lowThreshold) + "'><br>";
            html += "<label>High Threshold (0 - 100):</label><br><input name='highThreshold' type='number' min='0' max='100' value='" + String(highThreshold) + "'><br>";
        }

        html += "<label>Min Brightness (0 - 255):</label><br><input name='minBrightness' type='number' min='0' max='255' value='" + String(minBrightness) + "'><br>";

        html += "<label>Max Brightness (0 - 255):</label><br><input name='maxBrightness' type='number' min='0' max='255' value='" + String(maxBrightness) + "'><br>";

        html += "<button type='submit'>Save</button></form>";
        html += "<br><a href='/'>Back</a></body></html>";
        server.send(200, "text/html", html);
        });


    server.on("/save_brightness", HTTP_POST, []() {
        use_adc = server.hasArg("use_adc");
        lowThreshold = server.arg("lowThreshold").toInt();
        highThreshold = server.arg("highThreshold").toInt();
        
        maxBrightness = (uint8_t)server.arg("maxBrightness").toInt();
        minBrightness = (uint8_t)server.arg("minBrightness").toInt();

        preferences.putBool("use_adc", use_adc);
        preferences.putInt("lowThreshold", lowThreshold);
        preferences.putInt("highThreshold", highThreshold);
        preferences.putUChar("currentBrightness", currentBrightness);
        preferences.putUChar("maxBrightness", maxBrightness);        
        preferences.putUChar("minBrightness", minBrightness);


        server.send(200, "text/html",
            "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2; url=/brightness'><title>Saved</title></head>"
            "<body style='font-family:Arial;text-align:center;'><h2>Settings saved</h2><p>Returning...</p></body></html>");
        });



    server.on("/files", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head><title>All Files</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:Arial;text-align:center;}table{margin:auto;}th,td{padding:8px;}</style></head><body>";
        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();
        html += "Connected to: <strong>" + WiFi.SSID() + "</strong> (" + WiFi.localIP().toString() + ")" + "&nbsp;&nbsp;";
        html += "Storage: " + String(used / 1024) + " KB / " + String(total / 1024) + " KB <hr>";

        html += "<h2>All Files on LittleFS</h2><table border='1'><tr><th align=left>Filename</th><th>Size (bytes)</th><th>Info</th><th>Action</th></tr>";

        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file) {
            String info = getBmpInfo(file.name());
            String name = file.name();
            html += "<tr><td align=left>" + name + "</td><td align=right>" + String(file.size()) + "</td>";
            html += "<td align=right>" + String(info) + "</td>";
            html += " <td><a href = '/delete?file=" + name + "' onclick = 'return confirm(\"Delete " + name + "?\")'>Delete</a></td></tr>";
            file = root.openNextFile();
        }
        html += "</table><br><a href='/'>Back</a></body></html>";
        server.send(200, "text/html", html);
        });


    server.on("/status", HTTP_GET, []() {

        char version[32];
        sprintf(version, "%d-%02d-%02d %02d:%02d%:%02d", BUILD_YEAR, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN, BUILD_SEC);

        String html = "<!DOCTYPE html><html><head><title>All Files</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:Arial;}table{margin:auto;}th,td{padding:8px;}</style></head><body>";

        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();
        html += "Connected to: <strong>" + WiFi.SSID() + "</strong> (" + WiFi.localIP().toString() + ")" + "&nbsp;&nbsp;Storage: " + String(used / 1024) + " KB / " + String(total / 1024) + " KB<hr>";

        html += "<h2>ESP System Status</h2><ul>";

        String tzLabel = preferences.getString("timezone", "DE");
        String tzDesc;

        if (tzLabel == "DE") tzDesc = "CET-1CEST (Auto DST)";
        else if (tzLabel == "DE_S") tzDesc = "CEST-2 (Permanent Summer Time)";
        else if (tzLabel == "DE_W") tzDesc = "CET-1 (Permanent Winter Time)";
        else if (tzLabel == "UK") tzDesc = "GMT0BST (Auto DST)";
        else if (tzLabel == "UK_S") tzDesc = "BST-1 (Permanent Summer Time)";
        else if (tzLabel == "UK_W") tzDesc = "GMT0 (Permanent Winter Time)";
        else tzDesc = "(Unknown)";
         
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            char nowStr[32];
            strftime(nowStr, sizeof(nowStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
            html += "<li>Current Time: " + String(nowStr) + "</li>";
            html += "<li>Timezone: " + tzDesc + "</li><br>";
        }
        
        html += "<li>Compiled on: <strong>" + (String)version + "</strong></li><br>";

#ifdef GC9A01
        html += "<li>TFT driver: GC9A01</li>";
#elif defined(GC9D01)
        html += "<li>TFT driver: GC9D01</li>";
#elif defined(ILI9341)
        html += "<li>TFT driver: ILI9341</li>";
#else 
        html += "<li>TFT driver: unknown</li>";
#endif


        html += "<br>";
        html += "<li>ChipModel: " + String(ESP.getChipModel()) + "</li>";
        html += "<li>ChipRevision: " + String(ESP.getChipRevision()) + "</li>";
        html += "<li>ChipCores: " + String(ESP.getChipCores()) + "</li>";
        html += "<li>Chip ID: " + String((uint32_t)ESP.getEfuseMac(), HEX) + "</li>";
        html += "<li>CPU Frequency: " + String(getCpuFrequencyMhz()) + " MHz</li><br>";

        html += "<li>Hostname: " + String(WiFi.getHostname()) + "</li>";
        html += "<li>IP Address: " + WiFi.localIP().toString() + "</li>";
        html += "<li>MAC Address: " + WiFi.macAddress() + "</li>";
        html += "<li>WiFi SSID: " + String(WiFi.SSID()) + "</li>";
        html += "<li>WiFi Mode: " + String(WiFi.getMode() == WIFI_AP ? "AP" : (WiFi.getMode() == WIFI_STA ? "STA" : "AP_STA")) + "</li>";  
        html += "<li>WiFi Channel: " + String(WiFi.channel()) + "</li>";
        html += "<li>Signal Strength (RSSI): " + String(WiFi.RSSI()) + " dBm</li><br>";   

        html += "<li>SDK Version: " + String(ESP.getSdkVersion()) + "</li><br>";
        
        html += "<li>Flash Size: " + String(ESP.getFlashChipSize() / 1024) + " KB</li>";
        html += "<li>Free Heap: " + String(ESP.getFreeHeap() / 1024) + " KB</li>";
        html += "<li>Sketch Size: " + String(ESP.getSketchSize() / 1024) + " KB</li><br>";
       
        html += "<li>LittleFS Size: " + String(LittleFS.totalBytes() / 1024) + " KB</li>";
        html += "<li>LittleFS Used: " + String(LittleFS.usedBytes() / 1024) + " KB</li>";   
        html += "<li>LittleFS Free: " + String((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024) + " KB</li><br>";   
              

        unsigned long seconds = millis() / 1000;
        unsigned long days = seconds / 86400;
        unsigned long hours = (seconds % 86400) / 3600;
        unsigned long minutes = (seconds % 3600) / 60;
        unsigned long secs = seconds % 60;
        html += "<li>Uptime: " + String(days) + "d " + String(hours) + "h " + String(minutes) + "m " + String(secs) + "s</li>";
        
        if (photoresistorFound) {
            html += "<li>Photoresistor found on GPIO " + String(ADC_PIN) + "</li>";
        } else { 
            html += "<li>Photoresistor not found on GPIO " + String(ADC_PIN) + "</li><br>";
        }
        html += "<li>Actual brightness (0-255): " + String(currentBrightness) + "</li><br>"; 


        html += "<li>TFT_SCLK: " + String(TFT_SCLK) + "</li>";
        //html += "<li>TFT_MISO: " + String(TFT_MISO) + "</li>";  
        html += "<li>TFT_MOSI: " + String(TFT_MOSI) + "</li>";  
        html += "<li>TFT_CS: " + String(TFT_CS) + "</li>";  
        html += "<li>TFT_DC: " + String(TFT_DC) + "</li>";  
        html += "<li>TFT_RST: " + String(TFT_RST) + "</li>";

#ifndef TFT_Backlight 
        html += "<li>TFT_Backlight: none</li>";
#else
        html += "<li>TFT_Backlight: " + String(TFT_Backlight) + "</li>";  
#endif
        html += "<br>";
        html += "<li>Contact: holger.wagenlehner@gmx.de</li>";



        html += "</ul><br><a href='/'>Back</a></body></html>";
        server.send(200, "text/html", html);
        });

    server.on("/preview_defaultface", HTTP_GET, []() {
        const int headerSize = 54;
        const int rowSize = ((CLOCK_WIDTH * 2 + 3) / 4) * 4;
        const int dataSize = rowSize * CLOCK_HEIGHT;
        const int fileSize = headerSize + dataSize;

        uint8_t* bmpData = new uint8_t[fileSize];
        memset(bmpData, 0, fileSize);

        bmpData[0] = 'B'; bmpData[1] = 'M';
        *(uint32_t*)&bmpData[2] = fileSize;
        *(uint32_t*)&bmpData[10] = headerSize;
        *(uint32_t*)&bmpData[14] = 40;
        *(int32_t*)&bmpData[18] = CLOCK_WIDTH;
        *(int32_t*)&bmpData[22] = -CLOCK_HEIGHT;
        *(uint16_t*)&bmpData[26] = 1;
        *(uint16_t*)&bmpData[28] = 16;
        *(uint32_t*)&bmpData[34] = dataSize;

        for (int y = 0; y < CLOCK_WIDTH; y++) {
            uint8_t* rowPtr = bmpData + headerSize + y * rowSize;
            for (int x = 0; x < CLOCK_HEIGHT; x++) {
                uint16_t px = clockFace[y * CLOCK_WIDTH + x];
                if (px == TRANSPARENT_COLOR) px = 0xFFFF;
                ((uint16_t*)rowPtr)[x] = px;
            }
        }

        server.send_P(200, "image/bmp", (const char*)bmpData, fileSize);
        delete[] bmpData;
        });

    server.on("/listfilesFaces", HTTP_GET, []() {

        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();


        String html = "<!DOCTYPE html><html><head><title>Clock Face Files</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:Arial;text-align:center;}table{margin:auto;}th,td{padding:8px;}</style></head>";


        html += "Connected to : <strong>" + WiFi.SSID() + " </strong > (" + WiFi.localIP().toString() + ")" + (WiFi.SSID() == wifi_ssid2 ? " (secondary)" : "") + " &nbsp;&nbsp; ";
        html += "Storage used : " + String(used / 1024) + " KB / " + String(total / 1024) + " KB<hr>";

        html += "<h2>Manage Clock Face Files " + String(CLOCK_WIDTH) + " x " + String(CLOCK_HEIGHT) + "</h2><table border='1'><tr><th>Preview</th><th>Action</th></tr>";

        // Add built-in default face
        html += "<tr><td>default (built-in)<br>";
        html += "<img src='/preview_defaultface' style='width:80px;height:80px;border:1px solid #ccc'></td>";
        html += "<td><a href='/setbackground?file=face_default.bmp'>Set</a></td></tr>";

        File root = LittleFS.open("/");
        File file = root.openNextFile();


        bool anyFile = false;
        while (file) {
            String name = file.name();
            Serial.println(name);
            if (!file.isDirectory() && name.startsWith("face_") && name.endsWith(".bmp")) {
                anyFile = true;
                String shortName = name;
                String info = getBmpInfo(name);
                html += "<tr><td>" + shortName + "<br><img src='/file?name=" + name + "' style='width:80px;height:80px;border:1px solid #ccc'><br><small>" + String(info) + "</small></td>";
                html += "<td><a href='/setbackground?file=" + shortName + "'>Set</a> | ";
                html += "<a href='/delete?file=" + shortName + "'>Delete</a></td></tr>";
            }
            file = root.openNextFile();
        }

        if (!anyFile) html += "<tr><td colspan='3'>No BMP files found in /faces.</td></tr>";
        html += "</table><hr>";

        html += "<h3>Upload New Clock Face</h3>";
        html += "<small>Requirements: " + String(CLOCK_WIDTH) + " x " + String(CLOCK_HEIGHT) + " pixels, 16-bit BMP (RGB565), name must start with <code>face_</code></small><br><br>";
        html += "<form method = 'POST' action = '/upload' enctype = 'multipart/form-data' onsubmit = 'showProgress()'>";
        html += "<input type='file' name='upload' accept='.bmp' required><br>";
        
        html += "<button type='submit'>Upload BMP</button>";
        html += "<div id='progress' style='display:none;'>Uploading... please wait</div>";
        html += "<script>function showProgress(){document.getElementById('progress').style.display='block';}</script></form>";
        html += "<br><a href='/'>Back</a></body> </html>";
        server.send(200, "text/html", html);
        });

    server.on("/", HTTP_GET, []() {
       

        String html = "<!DOCTYPE html><html><head><title>Clock Setup</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>body{font-family:Arial;text-align:center;}input,select,button{margin:10px;padding:10px;width:80%;}</style></head><body>";
        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();
        html += "Connected to: <strong>" + WiFi.SSID() + "</strong> (" + WiFi.localIP().toString() + ")" + "&nbsp;&nbsp;";
        html += "Storage: " + String(used / 1024) + " KB / " + String(total / 1024) + " KB <hr>";
        html += "<h2>Clock Setup</h2>";

        html += "<form action = '/save' method = 'POST'>";

        html += "<h3>Primary WiFi</h3>";
        html += "<input name='ssid' placeholder='SSID' value='" + wifi_ssid + "'><br>";
        

        html += "<input name='pass' id='pass' placeholder='Password' type='password' value=''><br>";
        html += "<small>Password is hidden. Leave empty to keep current.</small><br><br>";


        html += "<h3>Alternative WiFi</h3>";
        html += "<input name='ssid2' placeholder='SSID 2' value='" + wifi_ssid2 + "'><br>";
        html += "<input name='pass2' placeholder='Password 2' type='password' value=''><br>";
        html += "<small>Password is hidden. Leave empty to keep current.</small><br>";

        html += "<br>";
        

        html += "<button type='submit'>Save</button></form><hr>";



        html += "<form action='/applydisplaysettings' method='POST'>";

        String timezone = preferences.getString("timezone", "DE");

        html += "<br>";
        html += "<strong>Timezone</strong><br>";
        html += "<select name='timezone'>";
        html += String("<option value='DE'") + (timezone == "DE" ? " selected" : "") + ">Germany (auto DST)</option>";
        html += String("<option value='DE_S'") + (timezone == "DE_S" ? " selected" : "") + ">Germany (always summer)</option>";
        html += String("<option value='DE_W'") + (timezone == "DE_W" ? " selected" : "") + ">Germany (always winter)</option>";
        html += String("<option value='UK'") + (timezone == "UK" ? " selected" : "") + ">UK (auto DST)</option>";
        html += String("<option value='UK_S'") + (timezone == "UK_S" ? " selected" : "") + ">UK (always summer)</option>";
        html += String("<option value='UK_W'") + (timezone == "UK_W" ? " selected" : "") + ">UK (always winter)</option>";
        html += "</select><br>";
                     

        
        html += "<table style='margin:auto;text-align:left;'><tr>";

        html += "<td><input type='checkbox' name='enableBahnhof' value='1' ";
        html += preferences.getBool("bahnhof", true) ? "checked" : "";
        html += "> Train Station Mode</td>";

        html += "<td><input type='checkbox' name='showSecond' value='1' ";
        html += preferences.getBool("secondhand", true) ? "checked" : "";
        html += "> Show Seconds</td>";

#ifndef GC9D01
        uint8_t rot = preferences.getUChar("storientation", 0);
        html += "<td>Rotation: <select name='rotation'>";
        for (int i = 0; i <= 3; i++) {
            html += "<option value='" + String(i) + "'";
            if (i == rot) html += " selected";
            html += ">" + String(i) + "</option>";
        }
        html += "</select></td>";
#endif

        html += "<td valign=bottom><button type='submit'>Apply</button></td>";
        html += "</tr></table></form>";


        html += "<hr>";


        html += "<a href='/listfilesFaces'><button>Manage Clock Face Files</button></a><br><br>";
        html += "<a href='/handsets'><button>Manage Hand Sets</button></a><br><br>";

        html += "<form action='/syncnow' method='POST'><button type='submit'>Sync Time Now</button></form><br>";
        html += "<form action='/brightness' method='POST'><button type='submit'>Brightness Settings</button></form><br>";
        html += "<a href='/status'>Systemstatus</a><br>";
        html += "<a href='/files'>File Manager</a><br>";

        html += "<br><a href='/reboot'>Reboot</a><br>";
        
        html += "</body></html>";
        server.send(200, "text/html", html);
        });

    server.on("/save", HTTP_POST, []() {
        if (server.hasArg("ssid")) {
            if (server.arg("ssid") != "") preferences.putString("ssid", server.arg("ssid"));       
            if (server.arg("ssid2") != "") preferences.putString("ssid2", server.arg("ssid2"));       

            if (server.arg("pass") != "") preferences.putString("pass", server.arg("pass"));
            if (server.arg("pass2") != "") preferences.putString("pass2", server.arg("pass2"));

            preferences.putBool("secondhand", server.hasArg("show"));
            preferences.putBool("bahnhof", server.hasArg("enableBahnhof"));    

            server.send(200, "text/html", "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='10; url=/'><title>Rebooting</title></head><body style='font-family:Arial;text-align:center;'><h2>Rebooting...</h2><p>Returning to main page in 10 seconds.</p></body></html>");
            delay(1000);
            ESP.restart();
        }
        });

    server.on("/upload", HTTP_GET, []() {
        server.send(200, "text/html", "<form method='POST' action='/upload' enctype='multipart/form-data' onsubmit='showProgress()'><input type='file' name='upload' accept='.bmp'><br><br><button type='submit'>Upload BMP</button><div id='progress' style='display:none;'>Uploading... please wait ⏳</div><script>function showProgress(){document.getElementById('progress').style.display='block';}</script></form><br><a href='/listfilesFaces'>Back to file list</a>");
        });

    server.on("/upload", HTTP_POST, []() {
        if (uploadSuccess) {
            server.sendHeader("Location", "/listfilesFaces", true);
            server.send(302, "text/plain", "");
        }
        else {
            String errorHtml = "<!DOCTYPE html><html><head><title>Upload Failed</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body style='font-family:Arial;text-align:center;'>";
            errorHtml += "<h2>Upload failed</h2>";
            errorHtml += "<p>Only .bmp files starting with <code>face_</code> or <code>hand_</code> are accepted.</p>";
            errorHtml += "<a href='/upload'>Try again</a></body></html>";
            server.send(400, "text/html", errorHtml);
        }
        }, handleFileUpload);

    server.on("/setbackground", HTTP_GET, []() {
        if (server.hasArg("file")) {
            String file = server.arg("file");
            file.replace("..", "");
            if (!file.startsWith("/")) file = "/" + file;

            if (file == "/face_default.bmp") {
                selectedBackground = file;
                preferences.putString("background", file);
                loadClockFace();
                loadHandSprites();
                server.sendHeader("Location", "/listfilesFaces", true);
                server.send(302, "text/plain", "");
                return;
            }

            if (LittleFS.exists(file)) {
                selectedBackground = file;
                preferences.putString("background", file);
                loadClockFace();
                loadHandSprites();
                server.sendHeader("Location", "/listfilesFaces", true);
                server.send(302, "text/plain", "");
                return;
            }
        }
        server.send(404, "text/plain", "File not found");
        });


    server.on("/delete", HTTP_GET, []() {
        if (server.hasArg("file")) {
            String path = server.arg("file");
            path.replace("..", "");
            if (!path.startsWith("/")) path = "/" + path;
            if (LittleFS.exists(path)) {
                LittleFS.remove(path);
                server.sendHeader("Location", "/listfilesFaces", true);
                server.send(302, "text/plain", "");
            }
            else {
                server.send(404, "text/plain", "File not found");
            }
        }
        });

    server.on("/file", HTTP_GET, []() {
        if (server.hasArg("name")) {
            String path = server.arg("name");
            if (!path.startsWith("/")) path = "/" + path;
            if (LittleFS.exists(path)) {
                File f = LittleFS.open(path, "r");
                server.streamFile(f, "image/bmp");
                f.close();
                return;
            }
        }
        server.send(404, "text/plain", "File not found");
        });

    server.on("/handsets", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head><title>Clock Hand Set Files</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:Arial;text-align:center;}table{margin:auto;}th,td{padding:10px;}img{height:50px;}</style></head><body>";
        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();
        html += "Connected to: <strong>" + WiFi.SSID() + "</strong> (" + WiFi.localIP().toString() + ")" + "&nbsp;&nbsp;Storage: " + String(used / 1024) + " KB / " + String(total / 1024) + " KB<hr>";
        html += "<h2>Manage Clock Hand Sets " + String(HAND_WIDTH) + " x " + String(HAND_HEIGHT) + "</h2><table border = '1'><tr><th>Set</th><th>Preview</th><th>Action</th></tr>";

        String activeSet = preferences.getString("handset", "");
        std::set<String> foundSets;

        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file) {
            String name = file.name();
            Serial.println(name);
            if (!file.isDirectory() && name.startsWith("hand_set") && name.endsWith(".bmp")) {
                int start = 5;
                int end = name.indexOf('_', start);
                if (end > start) {
                    String set = name.substring(start, end);
                    Serial.println("[HANDS] Found set: " + set);
                    foundSets.insert(set);
                }
            }
            file = root.openNextFile();
        }

        if (foundSets.empty()) {
            html += "<tr><td colspan='3'>No hand sets found.</td></tr>";
        }

        // Always show default as built-in
        html += "<tr><td>default (built-in)</td><td>";
        html += "<img src='data:image/bmp;base64," + encodeBmpToBase64(handHour, HAND_WIDTH, HAND_HEIGHT) + "'> ";
        html += "<img src='data:image/bmp;base64," + encodeBmpToBase64(handMinute, HAND_WIDTH, HAND_HEIGHT) + "'> ";
        html += "<img src='data:image/bmp;base64," + encodeBmpToBase64(handSecond, HAND_WIDTH, HAND_HEIGHT) + "'>";
        html += "</td><td><a href='/sethandset?set='>Set</a></td></tr>";

        for (const String& set : foundSets) {
            html += "<tr><td>" + set + (set == activeSet ? " (active)" : "") + "</td><td>";
            String hourPath = "/hand_" + set + "_hour.bmp";
            String minutePath = "/hand_" + set + "_minute.bmp";
            String secondPath = "/hand_" + set + "_second.bmp";
            html += LittleFS.exists(hourPath) ? "<img src='/file?name=" + hourPath + "'> " : "<img src='data:image/bmp;base64," + encodeBmpToBase64(handHour, HAND_WIDTH, HAND_HEIGHT) + "'> ";
            html += LittleFS.exists(minutePath) ? "<img src='/file?name=" + minutePath + "'> " : "<img src='data:image/bmp;base64," + encodeBmpToBase64(handMinute, HAND_WIDTH, HAND_HEIGHT) + "'> ";
            html += LittleFS.exists(secondPath) ? "<img src='/file?name=" + secondPath + "'>" : "<img src='data:image/bmp;base64," + encodeBmpToBase64(handSecond, HAND_WIDTH, HAND_HEIGHT) + "'>";

            html += "</td><td><a href='/sethandset?set=" + set + "'>Set</a> | <a href='/deletehandset?set=" + set + "' onclick=\"return confirm('Delete this hand set?')\">Delete</a></td></tr>";
        }

        html += "</table><hr>";
               
        uint8_t centerSize = preferences.getUInt("centerSize", 6);
        uint32_t centerColor = preferences.getUInt("centerColor", 0x000000);

        html += "<h2>Centre point</h2><form action='/setcenter' method='POST'>";
        html += "<label>Size (Pixel):</label><br><input name='size' type='number' min='1' max='50' value='" + String(centerSize) + "'><br>";
        html += "<label>Color (RGB hex, e.g. FF0000 = Red, 000000 = Black):</label><br><input name='color' value='" + String((centerColor >> 11 & 0x1F) * 255 / 31 << 16 | (centerColor >> 5 & 0x3F) * 255 / 63 << 8 | (centerColor & 0x1F) * 255 / 31, HEX) + "'><br>";
        html += "<button type='submit'>Apply</button></form><hr>";

        html += "<h3>Upload New Hand Set</h3>";
        html += "<small>Requirements: " + String(HAND_WIDTH) + " x " + String(HAND_HEIGHT) + " pixels, 16-bit BMP (RGB565), name must start with <code>hand_</code></small><br><br>";
        html += "<form method='POST' action='/uploadhandset' enctype='multipart/form-data'>";
        html += "Set name: <input name='set' required><br>";
       
        html += "Type: <select name='target'><option value='hour'>Hour</option><option value='minute'>Minute</option><option value='second'>Second</option></select><br>";
        html += "File: <input type='file' name='upload' accept='.bmp' required><br><br>";
        html += "<button type='submit'>Upload to Set</button></form><br>";
        html += "<a href='/'>Back</a></body></html>";
        server.send(200, "text/html", html);
        });

    server.on("/setcenter", HTTP_POST, []() {
        if (server.hasArg("size") && server.hasArg("color")) {
            uint8_t size = server.arg("size").toInt();
            uint32_t rgb = (uint32_t)strtoul(server.arg("color").c_str(), nullptr, 16);

            // Convert 24-bit RGB888 to RGB565
            uint8_t r = (rgb >> 16) & 0xFF;
            uint8_t g = (rgb >> 8) & 0xFF;
            uint8_t b = rgb & 0xFF;
            uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);

            preferences.putUInt("centerSize", size);
            preferences.putUInt("centerColor", rgb565);
        }
        server.send(200, "text/html",
            "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='3; url=/handsets'>"
            "<title>Updated</title></head><body style='font-family:Arial;text-align:center;'>"
            "<h2>Centre point updated</h2><p>back to handsets...</p></body></html>");
        });


    server.on("/uploadhandset", HTTP_POST, []() {
        if (uploadSuccess && server.hasArg("set") && server.hasArg("target")) {
            // Sicherheitsprüfung auf Dateinamenmuster
            if (!uploadFilePath.endsWith(".bmp") || !uploadFilePath.startsWith("/hand_")) {
                String errorHtml = "<!DOCTYPE html><html><head><title>Upload Failed</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body style='font-family:Arial;text-align:center;'>";
                errorHtml += "<h2>Upload failed</h2>";
                errorHtml += "<p>Only .bmp files starting with <code>hand_</code> are accepted for handset upload.</p>";
                errorHtml += "<a href='/handsets'>Try again</a></body></html>";
                server.send(400, "text/html", errorHtml);
                return;
            }
            String set = server.arg("set");
            String target = server.arg("target");
            String dir = "/";
            if (!LittleFS.exists(dir)) LittleFS.mkdir(dir);
            String finalPath = "/hand_set" + set + "_" + target + ".bmp";
            LittleFS.rename(uploadFilePath, finalPath);
            Serial.println("[UPLOAD] Hand uploaded to: " + finalPath);
            server.sendHeader("Location", "/handsets", true);
            server.send(302, "text/plain", "");
        }
        else {
            server.send(500, "text/html", " Upload failed!<br><a href='/handsets'>Try again</a>");
        }
        }, handleFileUpload);

    server.on("/sethandset", HTTP_GET, []() {
        if (server.hasArg("set")) {
            String chosen = server.arg("set");
            preferences.putString("handset", chosen);
            loadClockFace();
            loadHandSprites();
            updateClock();
            server.sendHeader("Location", "/handsets", true);
            server.send(302, "text/plain", "");
        }
        else {
            server.send(400, "text/plain", "Missing set name");
        }
        });

    server.on("/deletehandset", HTTP_GET, []() {
        if (server.hasArg("set")) {
            String set = server.arg("set");
            String targets[] = { "hour", "minute", "second" };
            for (const String& target : targets) {
                String path = "/hand_" + set + "_" + target + ".bmp";
                Serial.println("[DELETE] Looking for: " + path);
                if (LittleFS.exists(path)) {
                    LittleFS.remove(path);
                    Serial.println("[DELETE] Removed: " + path);
                }
            }
            server.sendHeader("Location", "/handsets", true);
            server.send(302, "text/plain", "");
        }
        else {
            server.send(400, "text/plain", "Missing set name");
        }
        });

    server.on("/reboot", HTTP_GET, []() {
        server.send(200, "text/html", "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='10; url=/'><title>Rebooting</title></head><body style='font-family:Arial;text-align:center;'><h2>Rebooting...</h2><p>Returning to main page in 10 seconds.</p></body></html>");
        delay(1000);
        ESP.restart();
        });    

    server.on("/syncnow", HTTP_POST, []() {
        setupNTP();
        struct tm timeinfo;
        getLocalTime(&timeinfo);

        char timeStr[32];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

        String html = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='5; url=/'>"
            "<title>Time Synced</title></head><body style='font-family:Arial;text-align:center;'>"
            "<h2>Time synced</h2><p>" + String(timeStr) + "</p><p>Returning to main page in 5 seconds.</p></body></html>";

        server.send(200, "text/html", html);
        });

}

void handleFileUpload() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        uploadFilePath = upload.filename;
        uploadFilePath.replace("..", "");
        if (!uploadFilePath.startsWith("/")) uploadFilePath = "/" + uploadFilePath;

        // Nur bestimmte Dateinamenmuster zulassen
        if (!uploadFilePath.endsWith(".bmp") ||
            !(uploadFilePath.startsWith("/face_") || uploadFilePath.startsWith("/hand_"))) {
            Serial.println("[UPLOAD] Invalid filename: must start with 'face_' or 'hand_' and end with '.bmp'");
            uploadSuccess = false;
            return;
        }

        uploadFilePath.replace("..", "");
        if (!uploadFilePath.startsWith("/")) uploadFilePath = "/" + uploadFilePath;

        Serial.println("[UPLOAD] Start: " + uploadFilePath);
        uploadFile = LittleFS.open(uploadFilePath, FILE_WRITE);
        uploadSuccess = uploadFile ? true : false;
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadSuccess && uploadFile) {
            uploadFile.write(upload.buf, upload.currentSize);
        }
    }
    else if (upload.status == UPLOAD_FILE_END) {
        if (uploadSuccess && uploadFile) {
            uploadFile.close();
            if (LittleFS.exists(uploadFilePath)) {
                Serial.println("[UPLOAD] Finished OK: " + uploadFilePath);
                String ext = uploadFilePath;
                ext.toLowerCase();
                if (ext.endsWith(".bmp")) {
                    bool isHand = uploadFilePath.indexOf("hour") > 0 || uploadFilePath.indexOf("minute") > 0 || uploadFilePath.indexOf("second") > 0;
                    if (uploadFilePath.startsWith("/face_")) {
                        Serial.println("[UPLOAD] Detected Clock Face upload");
                    }
                    else if (uploadFilePath.startsWith("/hand_")) {
                        Serial.println("[UPLOAD] Detected Clock Hand upload");
                    }                    
                }
            }
            else {
                Serial.println("[UPLOAD] Finished but file missing!");
                uploadSuccess = false;
            }
        }
        else {
            Serial.println("[UPLOAD] Failed during writing");
        }
    }
}

bool loadBmpToSprite(TFT_eSprite* sprite, const char* filename) {
    File bmp = LittleFS.open(filename, "r");
    if (!bmp) return false;

    uint8_t header[54];
    if (bmp.read(header, 54) != 54 || header[0] != 'B' || header[1] != 'M') {
        bmp.close();
        return false;
    }

    int32_t width = *(int32_t*)&header[18];
    int32_t height = *(int32_t*)&header[22];
    uint16_t bpp = *(uint16_t*)&header[28];
    uint32_t offset = *(uint32_t*)&header[10];

    if (width != CLOCK_WIDTH || abs(height) != CLOCK_HEIGHT || bpp != 16) {
        bmp.close();
        return false;
    }

    bool flip = height > 0;
    height = abs(height);

    bmp.seek(offset);
    for (int y = 0; y < height; y++) {
        int row = flip ? height - 1 - y : y;
        bmp.read((uint8_t*)rowBuffer, CLOCK_WIDTH * 2);

        for (int x = 0; x < CLOCK_HEIGHT; x++) {
            rowBuffer[x] = setPixelBrightness(rowBuffer[x]);
        }

        sprite->pushImage(0, row, CLOCK_WIDTH, 1, rowBuffer);
    }

    bmp.close();
    return true;
}

bool checkBmpFormat(const String& filename, int expectedWidth = CLOCK_WIDTH, int expectedHeight = CLOCK_HEIGHT) {
    File bmpFile = LittleFS.open(filename, "r");
    if (!bmpFile) {
        Serial.println("[BMP Check] Failed to open file");
        return false;
    }

    uint8_t header[54];
    if (bmpFile.read(header, 54) != 54) {
        Serial.println("[BMP Check] Failed to read header");
        bmpFile.close();
        return false;
    }

    if (header[0] != 'B' || header[1] != 'M') {
        Serial.println("[BMP Check] Not a BMP file");
        bmpFile.close();
        return false;
    }

    int32_t width = *(int32_t*)&header[18];
    int32_t height = *(int32_t*)&header[22];
    uint16_t bpp = *(uint16_t*)&header[28];

    bmpFile.close();

    if (width != expectedWidth || abs(height) != expectedHeight || bpp != 16) {
        Serial.printf("[BMP Check] Invalid BMP dimensions or format: %d x %d, %d bpp", width, height, bpp);
        return false;
    }

    Serial.println("[BMP Check] BMP format valid");
    return true;
}

String getBmpInfo(const String& filename) {

    String file;
    if (filename.startsWith("/")) file = filename;
    else file = "/" + filename; if (!filename.startsWith("/")) file = "/" + filename;

    //Serial.println("[BMP Info] Checking file: " + file);
    File bmp = LittleFS.open(file, "r");
    if (!bmp) return "n/a";

    uint8_t header[54];
    if (bmp.read(header, 54) != 54 || header[0] != 'B' || header[1] != 'M') {
        bmp.close();
        return "n/a";
    }

    int32_t width = *(int32_t*)&header[18];
    int32_t height = *(int32_t*)&header[22];
    uint16_t bpp = *(uint16_t*)&header[28];
    bmp.close();

    return String(abs(width)) + " x " + String(abs(height)) + " / " + String(bpp) + " bpp";
}

