#include <Arduino.h>
#include <USBHID.h>
#include <map>
#include "USB.h"
#include "config.h"
#include "keyboardMultiple.h"
#include <SimpleKalmanFilter.h>

static const uint8_t TT1 = 38;
static const uint8_t TT2 = 37;
static const uint8_t TT3 = 36;
static const uint8_t TT4 = 35;
static const uint8_t TT5 = 34;
static const uint8_t TT6 = 33;
static const uint8_t R1 = 15;
static const uint8_t R2 = 16;
static const uint8_t R3 = 13;
static const uint8_t R4 = 14;
static const uint8_t R5 = 11;
static const uint8_t R6 = 12;

USBHIDKeyboard Keyboard;
Adafruit_MPR121 capA = Adafruit_MPR121();
Adafruit_MPR121 capB = Adafruit_MPR121();
Adafruit_MPR121 capC = Adafruit_MPR121();
Adafruit_MPR121 capD = Adafruit_MPR121();

uint32_t last_status;
std::map<uint8_t, int> PKEYS;
int FKEYS[32];

static int8_t IR_TX_PIN[6] = {TT1, TT2, TT3, TT4, TT5, TT6};
static int8_t IR_RX_PIN[6] = {R1, R2, R3, R4, R5, R6};
uint8_t KEYS[42] = {'6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
                    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
                    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
                    ',', '.', '0', '1', '2', '3', '4', '5',
                    'y', 'y', SLIDER_THRESHOLDS, IR_ACTIVATION};

void KeySetup() {  // 键盘与AIR初始化
    for (int i = 0; i < 6; i++) {
        pinMode(IR_TX_PIN[i], OUTPUT);
    }
    for (int i = 0; i < 6; i++) {
        pinMode(IR_RX_PIN[i], INPUT);
    }
    Keyboard.begin();
    DebugSerialDevice.println("[INFO] Key Setup OK");
}

void IRSetup() {
    for (int i = 0; i < 6; i++) {
        pinMode(IR_RX_PIN[i], INPUT);
    }
}

bool isIROpen() {
    if (KEYS[38] == 'y') {
        return 1;
    }
    return 0;
}

bool isSliderOpen() {
    if (KEYS[39] == 'y') {
        return 1;
    }
    return 0;
}

int calTouch(int bl, int fd) {  // 触摸数值计算
  int cal = bl - fd;
  // return cal > 4 ? 8 * cal : (cal > 0 ? cal : 0) ;
  return cal;
}


#define EstimateError 5
#define MeasurementError 5
#define ProcessNoise 0.01
SimpleKalmanFilter IRKalman[6] = {
    {MeasurementError, EstimateError, ProcessNoise},
    {MeasurementError, EstimateError, ProcessNoise},
    {MeasurementError, EstimateError, ProcessNoise},
    {MeasurementError, EstimateError, ProcessNoise},
    {MeasurementError, EstimateError, ProcessNoise},
    {MeasurementError, EstimateError, ProcessNoise},
};
#define INITIRDATA 8192
int IR_data[6];
int IR_raw_data[6];
int IR_bl[6];
int IR_bl_loop[6];
int IR_last_data[6] = {
    INITIRDATA, INITIRDATA, INITIRDATA, INITIRDATA, INITIRDATA, INITIRDATA
};
// bool FIR[6] = {1,1,1,1,1,1};
void IRImproveCheck() {
    for (int i=0; i<6; i++) {
        digitalWrite(IR_TX_PIN[i], HIGH);
        delay(2);
        IR_data[i] = (int)IRKalman[i].updateEstimate(analogRead(IR_RX_PIN[i]));
        digitalWrite(IR_TX_PIN[i], LOW);
        #if IRDEBUG
        DebugSerialDevice.print(IR_data[i]);
        DebugSerialDevice.print("-");
        #endif
    }
    #if IRDEBUG
    DebugSerialDevice.println();
    #endif
    for (int i=0; i<6; i++) {
        if (IR_data[i] - IR_last_data[i] > KEYS[41]*256) {
            Keyboard.addKey(KEYS[i+32]);
            Keyboard.sendKey();
            IR_last_data[i] = IR_data[i];
        }
        if (IR_data[i] - IR_last_data[i] < -KEYS[41]*256) {
            Keyboard.delKey(KEYS[i+32]);
            Keyboard.sendKey();
            IR_last_data[i] = IR_data[i];
        }
    }
}

bool setKeys(uint8_t keys[]) {
    memcpy(KEYS, keys, 42);
    return 1;
}

uint8_t* getKeys() { return KEYS; }

void sliderSetup() {  // 触摸初始化
    DebugSerialDevice.println("[INFO] Address 0x5A Slider Setup");
    while (!(capA.testBegin(0x5A))) {
        delay(300);
        DebugSerialDevice.println("[ERROR] Address 0x5A Slider Setup Failed, Retrying");
    }
    DebugSerialDevice.println("[INFO] Address 0x5B Slider Setup");
    while (!(capB.testBegin(0x5B))) {
        delay(300);
        DebugSerialDevice.println("[ERROR] Address 0x5B Slider Setup Failed, Retrying");
    }
    DebugSerialDevice.println("[INFO] Address 0x5C Slider Setup");
    while (!(capC.testBegin(0x5C))) {
        delay(300);
        DebugSerialDevice.println("[ERROR] Address 0x5C Slider Setup Failed, Retrying");
    }
    DebugSerialDevice.println("[INFO] Address 0x5D Slider Setup");
    while (!(capD.testBegin(0x5D))) {
        delay(300);
        DebugSerialDevice.println("[ERROR] Address 0x5D Slider Setup Failed, Retrying");
    }

    // 切换模式
    if ((uint8_t)capA.touched() != 0 || (uint8_t)capB.touched() != 0 ||
        (uint8_t)capC.touched() != 0 || (uint8_t)capD.touched() != 0 ||
        (uint8_t)capA.touched() != 0 || (uint8_t)capB.touched() != 0 ||
        (uint8_t)capC.touched() != 0 || (uint8_t)capD.touched() != 0) {
        while (!(capA.begin(0x5D, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD) & 
                 capB.begin(0x5C, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD) & 
                 capC.begin(0x5B, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD) &
                 capD.begin(0x5A, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD))) {
            delay(300);
        }
        DebugSerialDevice.println("[INFO] Slider Run In Gloves Mode");
    } else {
        while (!(capA.begin(0x5D, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD) & 
                 capB.begin(0x5C, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD) & 
                 capC.begin(0x5B, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD) &
                 capD.begin(0x5A, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD))) {
            delay(300);
        }
        DebugSerialDevice.println("[INFO] Slider Run In Hands Mode");
    }
    DebugSerialDevice.println("[INFO] Slider Setup OK");

    for (int i=0; i<255; i++) {
        PKEYS[i] = 0;
    }
}

void sliderScan() {  // 触摸扫描
    uint32_t sensors;
    uint8_t capa_data, capb_data, capc_data, capd_data;
    capa_data = capA.touched();
    capb_data = capB.touched();
    capc_data = capC.touched();
    capd_data = capD.touched();

    sensors |= (u_int32_t)((capa_data & 0b11000000) >> 6) << 0;
    sensors |= (u_int32_t)((capa_data & 0b110000) >> 4) << 2;
    sensors |= (u_int32_t)((capa_data & 0b1100) >> 2) << 4;
    sensors |= (u_int32_t)((capa_data & 0b11) >> 0) << 6;

    sensors |= (u_int32_t)((capb_data & 0b11000000) >> 6) << 8;
    sensors |= (u_int32_t)((capb_data & 0b110000) >> 4) << 10;
    sensors |= (u_int32_t)((capb_data & 0b1100) >> 2) << 12;
    sensors |= (u_int32_t)((capb_data & 0b11) >> 0) << 14;

    sensors |= (u_int32_t)((capc_data & 0b11000000) >> 6) << 16;
    sensors |= (u_int32_t)((capc_data & 0b110000) >> 4) << 18;
    sensors |= (u_int32_t)((capc_data & 0b1100) >> 2) << 20;
    sensors |= (u_int32_t)((capc_data & 0b11) >> 0) << 22;

    sensors |= (u_int32_t)((capd_data & 0b11000000) >> 6) << 24;
    sensors |= (u_int32_t)((capd_data & 0b110000) >> 4) << 26;
    sensors |= (u_int32_t)((capd_data & 0b1100) >> 2) << 28;
    sensors |= (u_int32_t)((capd_data & 0b11) >> 0) << 30;

    if (last_status != sensors) {
        for (int i = 0; i < 32; i++) {
            if (sensors & (1 << i)) {
                if (!(last_status & (1 << i))) {
                    Keyboard.addKey(KEYS[i]);
                    Keyboard.sendKey();
                    PKEYS[KEYS[i]]++;
                    // DebugSerialDevice.print("PressKEY: ");
                    // DebugSerialDevice.println(i + 1);
                }
            } else {
                // DebugSerialDevice.println(last_status, BIN);
                if (last_status & (1 << i)) {
                    PKEYS[KEYS[i]]--;
                    // DebugSerialDevice.print("ReleaseKEY: ");
                    // DebugSerialDevice.println(i + 1);
                    if (!(PKEYS[KEYS[i]])) {
                        Keyboard.delKey(KEYS[i]);
                        Keyboard.sendKey();
                    }
                }
            }
        }
        last_status = sensors;
    }
    // Keyboard.sendKey();
}

#define PSTHRESHOLD 3
#define RSTHRESHOLD 160
#define PSLOWTHRESHOLD 50
#define INITIAL 0
int lastKeyRaw[32] = {
        INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL,
        INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL,
        INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL,
        INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL, INITIAL,};
int16_t bl_data[32];
int16_t bl_improve[32];
int32_t bl_loop[32] = {8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192,
                       8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192,
                       8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192,
                       8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192};
                       
int16_t fl_data[32];
bool small_down_scan[32];
SimpleKalmanFilter KalmanFilter[32];

int mapRealKeys(int i) {
    if (i == 0) return 11;
    if (i == 1) return 10;
    if (i == 2) return 7;
    if (i == 3) return 6;
    if (i == 4) return 4;
    if (i == 5) return 5;
    if (i == 6) return 0;
    if (i == 7) return 1;
    return 0;
}

void sliderImproveKalmanScan() {
    // 扫描触摸
    for (int i=0; i<8; i++) {
        if (calTouch(bl_improve[i], fl_data[i]) - lastKeyRaw[i] < -RSTHRESHOLD) bl_improve[i] = fl_data[i];
        fl_data[i] = (int)KalmanFilter[i].updateEstimate(capA.filteredData(mapRealKeys(i)));
        bl_loop[i]++;
        if (bl_loop[i] > 3 && !FKEYS[i]) {
            bl_improve[i] = fl_data[i];
            bl_loop[i] = 0;
        }
        if (calTouch(bl_improve[i], fl_data[i]) < -5) bl_improve[i] = fl_data[i];
    }
    for (int i=8; i<16; i++) {
        if (calTouch(bl_improve[i], fl_data[i]) - lastKeyRaw[i] < -RSTHRESHOLD) bl_improve[i] = fl_data[i];
        fl_data[i] = (int)KalmanFilter[i].updateEstimate(capB.filteredData(mapRealKeys(i-8)));
        bl_loop[i]++;
        if (bl_loop[i] > 3 && !FKEYS[i]) {
            bl_improve[i] = fl_data[i];
            bl_loop[i] = 0;
        }
        if (calTouch(bl_improve[i], fl_data[i]) < -5) bl_improve[i] = fl_data[i];
    }
    for (int i=16; i<24; i++) {
        if (calTouch(bl_improve[i], fl_data[i]) - lastKeyRaw[i] < -RSTHRESHOLD) bl_improve[i] = fl_data[i];
        fl_data[i] = (int)KalmanFilter[i].updateEstimate(capC.filteredData(mapRealKeys(i-16)));
        bl_loop[i]++;
        if (bl_loop[i] > 3 && !FKEYS[i]) {
            bl_improve[i] = fl_data[i];
            bl_loop[i] = 0;
        }
        if (calTouch(bl_improve[i], fl_data[i]) < -5) bl_improve[i] = fl_data[i];
    }
    for (int i=24; i<32; i++) {
        if (calTouch(bl_improve[i], fl_data[i]) - lastKeyRaw[i] < -RSTHRESHOLD) bl_improve[i] = fl_data[i];
        fl_data[i] = (int)KalmanFilter[i].updateEstimate(capD.filteredData(mapRealKeys(i-24)));
        bl_loop[i]++;
        if (bl_loop[i] > 3 && !FKEYS[i]) {
            bl_improve[i] = fl_data[i];
            bl_loop[i] = 0;
        }
        if (calTouch(bl_improve[i], fl_data[i]) < -5) bl_improve[i] = fl_data[i];
    }
    // 扫描结束
    
    // 判断值
    for (int i=0; i<32; i++) {
        int cal = calTouch(bl_improve[i], fl_data[i]);
        #if SLIDERDEBUG
        DebugSerialDevice.print(cal);
        DebugSerialDevice.print("-");
        #endif
        if (cal - lastKeyRaw[i] > PSTHRESHOLD) lastKeyRaw[i] = cal;
        if (cal > (KEYS[40]+2) && !FKEYS[i]) {
            Keyboard.addKey(KEYS[i]);
            Keyboard.sendKey();
            FKEYS[i] = true;
            PKEYS[KEYS[i]]++;
            bl_loop[i] = 0;
        }
        if ((cal < (KEYS[40]+2) && PKEYS[KEYS[i]] && FKEYS[i]) || (cal - lastKeyRaw[i] < -RSTHRESHOLD && PKEYS[KEYS[i]])) {
            Keyboard.delKey(KEYS[i]);
            Keyboard.sendKey();
            FKEYS[i] = false;
            PKEYS[KEYS[i]]--;
            lastKeyRaw[i] = cal;
        }
        if (cal - lastKeyRaw[i] < -RSTHRESHOLD) lastKeyRaw[i] = cal;

    }
    #if SLIDERDEBUG
    DebugSerialDevice.println();
    #endif
}


void ChangeMode(int i) {    // 1手套 2空手
    if (i == 1) {
        while (!(capA.begin(0x5D, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD) & 
                 capB.begin(0x5C, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD) & 
                 capC.begin(0x5B, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD) &
                 capD.begin(0x5A, &Wire, GlovesPressThresholds, GlovesReleaseThresholds, GlovesMPR121_CHARGE_CURRENT, GlovesMPR121_ENCODING_PERIOD))) {
            delay(300);
        }
    }
    if (i == 2) {
        while (!(capA.begin(0x5D, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD) & 
                 capB.begin(0x5C, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD) & 
                 capC.begin(0x5B, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD) &
                 capD.begin(0x5A, &Wire, PressThresholds, ReleaseThresholds, MPR121_CHARGE_CURRENT, MPR121_ENCODING_PERIOD))) {
            delay(300);
        }
    }
}

void KeyTest() {  // 用以测试键盘按下
    delay(1000);
    DebugSerialDevice.println("StartPressKeys");
    Keyboard.press('1');
    Keyboard.press('2');
    Keyboard.press('3');
    Keyboard.press('4');
    Keyboard.press('5');
    Keyboard.press('6');
    Keyboard.press('7');
    Keyboard.press('8');
    delay(1000);
    DebugSerialDevice.println("StopPressKeys");
    Keyboard.releaseAll();
}

void KeyTest2() {  // 用以测试原始键盘报文
    delay(1000);
    DebugSerialDevice.println("StartPressKeys");
    hid_keyboard_report60_t report;
    report.modifier = 0x00;
    report.reserved = 0x00;
    report.keycode[0] = 0x04;
    report.keycode[1] = 0x05;
    report.keycode[2] = 0x06;
    report.keycode[3] = 0x07;
    report.keycode[4] = 0x08;
    report.keycode[5] = 0x09;
    report.keycode[6] = 0x0a;
    report.keycode[7] = 0x0b;
    report.keycode[8] = 0x0c;
    report.keycode[9] = 0x0d;
    report.keycode[10] = 0x0e;
    report.keycode[11] = 0x0f;
    report.keycode[12] = 0x10;
    report.keycode[13] = 0x11;
    report.keycode[14] = 0x12;
    report.keycode[15] = 0x13;
    report.keycode[16] = 0x14;
    report.keycode[17] = 0x15;
    for (int i = 17; i < 60; i++) {
        report.keycode[i] = 0x00;
    }
    Keyboard.hidRaw(report);
    delay(1000);
    DebugSerialDevice.println("StopPressKeys");
    report.modifier = 0x00;
    report.reserved = 0x00;
    for (int i = 0; i < 60; i++) {
        report.keycode[i] = 0x00;
    }
    Keyboard.hidRaw(report);
}

void IRTest() {
    static int loop = 0;
    loop++;
    if (loop > 400) {
        
        for (int i = 0; i < 6; i++) {
            digitalWrite(IR_TX_PIN[i], LOW);
            int pinval = analogRead(IR_RX_PIN[i]);
            DebugSerialDevice.print(pinval);
            DebugSerialDevice.print("-");
            digitalWrite(IR_TX_PIN[i], HIGH);
        }
        DebugSerialDevice.println();
    }
}