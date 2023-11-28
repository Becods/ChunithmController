#include <Arduino.h>
#include "config.h"
#include "key.h"
#include "vendor.h"

void setup() {
    DebugSerialDevice.begin(115200);
    DebugSerialDevice.setTimeout(0);
    /*
    boolean waitForInput = true;
    while (waitForInput) {
        if (DebugSerialDevice.available()) {
            char input = DebugSerialDevice.read();
            if (input == 's') {
                waitForInput = false;
            }
        }
    }
    eraseEEPROM();
    */
    EEPROMSetup();
    VendorSetup();
    KeySetup();
    sliderSetup();
    IRSetup();
    readEEPROM();
    Wire.setClock(3400000);
    USB.PID(0x8222);
    USB.productName("MChunithm");
    USB.manufacturerName("Becod");
    USB.serialNumber("2333");
    USB.begin();
}

void loop() {
    if (isIROpen()) {
        IRImproveCheck();
    }
    if (isSliderOpen()) {
        sliderImproveKalmanScan();
    }
    setKeysMap();
}
