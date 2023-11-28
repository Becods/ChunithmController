#include "config.h"
#include "key.h"
#include <Arduino.h>
#include "EEPROM.h"
#include "USB.h"
#include "USBHIDVendor.h"

USBHIDVendor Vendor;

uint8_t defaultKEYS[42] = {'6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', 'g',
                           'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r',
                           's', 't', 'u', 'v', 'w', 'x', 'y', 'z', ',', '.', '0',
                           '1', '2', '3', '4', '5',
                           'y', 'y', SLIDER_THRESHOLDS, IR_ACTIVATION};


void EEPROMSetup() {
    EEPROM.begin(64);
    if (EEPROM.read(63)) {
        for (int i=0; i<64; i++) {
            EEPROM.write(i, 0);
        }
        EEPROM.commit();
    }
    if (EEPROM.read(1) == 0x00 || EEPROM.read(41) == 0x00 ) {
        for (int i=0; i<42; i++) {
            EEPROM.write(i, defaultKEYS[i]);
        }
        EEPROM.commit();
    }
    
}

void VendorSetup() {
    Vendor.begin();
}

bool writeEEPROM() {
    uint8_t* writeKEYS = getKeys();
    DebugSerialDevice.print("[INFO] Write EEPROM: ");
    for (int i=0; i<42; i++) {
        EEPROM.write(i, writeKEYS[i]);
        DebugSerialDevice.print(writeKEYS[i]);
        DebugSerialDevice.print(" ");
    }
    EEPROM.commit();
    DebugSerialDevice.println();
    DebugSerialDevice.println("[INFO] Write EEPROM OK");
    return 1;
}

bool eraseEEPROM() {
    EEPROM.begin(64);
    for (int i=0; i<64; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
    return 1;
}

bool readEEPROM() {
    uint8_t readKEYS[42];
    for (int i=0; i<42; i++) {
        readKEYS[i] = EEPROM.read(i);
        // DebugSerialDevice.println(EEPROM.read(i));
    }
    setKeys(readKEYS);
    #if USBDEBUG
        DebugSerialDevice.print("[INFO] Read KEYS: ");
        for (int i=0; i<42; i++) {
            DebugSerialDevice.print(readKEYS[i]);
            DebugSerialDevice.print(" ");
        }
        DebugSerialDevice.println();
        DebugSerialDevice.println("[INFO] Read EEPROM OK");
    #endif
    return 1;
}

String cmd;
bool setKeysMap() {
    if (Vendor.available()) {
        cmd = Vendor.readStringUntil('%');
        if (cmd == "SetKeys") {
            cmd = "";
            uint8_t _setKeys[42];
            #if USBDEBUG
                DebugSerialDevice.print("[INFO] Set KEYS: ");
            #endif
            // Vendor.printf("KeySetReady");
            while (true) {
                if (Vendor.available()) {
                    for (int i=0; i<42; i++) {
                        _setKeys[i] = Vendor.read();
                        #if USBDEBUG
                            DebugSerialDevice.print(_setKeys[i]);
                            DebugSerialDevice.print(" ");
                        #endif
                    }
                    setKeys(_setKeys);
                    writeEEPROM();
                    #if USBDEBUG
                        DebugSerialDevice.println();
                        DebugSerialDevice.println("[INFO] Set KEYS OK");
                    #endif
                    break;
                }
            }
        }

        if (cmd == "GetKeys") {
            uint8_t* getKEYS = getKeys();
            String str;
            for (int i=0; i<40; i++) {
                str += String((char)getKEYS[i]);
            }
            str += (String('/') + String(getKEYS[40]));
            str += (String('/') + String(getKEYS[41]));
            Vendor.print(str);
            cmd = "";
        }

        if (cmd == "GetIr") {
            String str;
            for (int e=0; e<5; e++) {
                str += (String(IR_data[e] + String('/')));
            }
            str += String(IR_data[5]);
            Vendor.print(str);
            cmd = "";
        }

        if (cmd == "Gloves") {
            ChangeMode(1);
            cmd = "";
        }

        if (cmd == "Hands") {
            ChangeMode(2);
        }

        return 1;
    }
    return 0;
}