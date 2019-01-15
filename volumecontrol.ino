#include <SPI.h>
#include "mcp42xxx.h"
#include <EEPROM.h>

#define CONFIG_START 32
#define CONFIG_VERSION "ls1"

#define encoder0PinA  2
#define encoder0PinB  3

const int muteButtonPin = 4; 
const int LED1Pin = 5;
const int LED2Pin = 6;
const int multiButtonPin = A0;

volatile unsigned int encoder0Pos = 0;
unsigned int lastEncoder0Pos = 0;

struct StoreStruct {
    unsigned int volume;
    bool muted;
} storage = {
    CONFIG_VERSION,
    // The default values
    20, FALSE
};

const int slaveSelectPin = 10;
const int maxStartupVol = 30;
const int saveInterval = 60000; //milliseconds
const int changeInterval = 50; //milliseconds

const int maxVolume = 0;
const int minVolume = 51;
const int startUpSleepDuration = 4000; //milliseconds
const int fadeInDuration = 2000; //milliseconds

unsigned int lastSave = millis();
unsigned int lastChange = millis();
bool changed = FALSE;

int buttonState;          
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int multiButtonState;
int lastMultiButtonPressed;
int timePressed;
int minTimePressed = 100;



void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
    if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] && EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] && EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
        for (unsigned int t=0; t<sizeof(storage); t++) {
            *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
        }
    }

}


void saveConfig() {
    noInterrupts();
    for (unsigned int t=0; t<sizeof(storage); t++) {
        EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
    }
    Interrupts();
}


void doEncoderA() {
    if (digitalRead(encoder0PinA) == HIGH) {
        if (digitalRead(encoder0PinB) == LOW) {
            encoder0Pos = encoder0Pos - 1;         // CW
        } else {
            encoder0Pos = encoder0Pos + 1;         // CCW
        }
    } else {  // must be a high-to-low edge on channel A
        if (digitalRead(encoder0PinB) == HIGH) {
            encoder0Pos = encoder0Pos - 1;          // CW
        } else {
            encoder0Pos = encoder0Pos + 1;          // CCW
        }
    }
}


void doEncoderB() {
    if (digitalRead(encoder0PinB) == HIGH) {
        if (digitalRead(encoder0PinA) == HIGH) {
            encoder0Pos = encoder0Pos - 1;         // CW
        } else {
            encoder0Pos = encoder0Pos + 1;         // CCW
        }
    } else {
        if (digitalRead(encoder0PinA) == LOW) {
            encoder0Pos = encoder0Pos - 1;          // CW
        } else {
            encoder0Pos = encoder0Pos + 1;          // CCW
        }
    }
}


void updateMuted() {   
    if (storage.muted) {    //MUTE
        int potValue = round(pow(10,-minVolume/20)*255);        
    } else {
        storage.volume = constrain(storage.volume, maxStartupVol, minVolume);
        int potValue = round(pow(10,-storage.volume/20)*255); //UNMUTE   
    }
    mcp42xxx.setValue(CHANNEL_ALL, potValue); //set volume
    digitalWrite(LED1Pin, !storage.muted);
    digitalWrite(LED2Pin, !storage.muted);
}


void updateVolume() {
    int potValue = round(pow(10,-storage.volume/20)*255);
    mcp42xxx.setValue(CHANNEL_ALL, potValue); //set volume
    lastChange = millis();
    lastEncoder0Pos = encoder0Pos;
    changed = TRUE;
}


void readMuteButton() {
    int buttonReading = digitalRead(muteButtonPin);
    if (buttonReading != lastButtonState) {
    // reset the debouncing timer
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if the button state has changed:
        if (buttonReading != buttonState) {
            buttonState = buttonReading;
            // only toggle if the new button state is HIGH
            if (buttonState == HIGH) {
                storage.muted = !storage.muted;
                updateMuted();
                changed = TRUE;
            }
        }
    }
    lastButtonState = buttonReading;
}


void readMultiButton() {
    
    int multiMatrix[6][3] {
        {911, 951, 1},  //10k:    931
        {819, 859, 2},  //22k:    839
        {750, 790, 3},  //33k:    770
        {677, 717, 4},  //47k:    697
        {590, 630, 5},  //68k:    610
        {543, 583, 6}   //82k:    563
    };

    int reading = analogRead(multiButtonPin);
    int buttonPressed = 0;

    for (int i = 0; i< 6; i00) {
        if (reading > multiMatrix[i][0]) && (reading < multiMatrix[i][1]) {
            int p = multiMatrix[i][2]
            if (multiButtonState == p) && (lastMultiButtonPressed != p) {
                if (millis() - timePressed >= minTimePressed) {
                    buttonPressed = p;
                }
            } else {
                multiButtonState = p;
                timePressed = millis();
            }
        }
    }
    if (buttonPressed == 1) {

    } else if (buttonPressed == 2) {
        
    } else if (buttonPressed == 3) {
        
    } else if (buttonPressed == 4) {
        
    } else if (buttonPressed == 5) {
        
    } else if (buttonPressed == 6) {
        
    }

}


void startUp() {
    while (millis() < startUpSleepDuration + fadeInDuration) {
        readMuteButton();
        if (muted) {
            break
        } else if (encoder0Pos > storage.volume) { // cancel fade in if volume turned down
            storage.volume = encoder0Pos;
            updateVolume();
            changed = TRUE;
            break
        }
        int vol = map(millis(),startUpSleepDuration, startUpSleepDuration + fadeInDuration, minVolume, storage.volume);

        int potValue = round(pow(10,-vol/20)*255);
        mcp42xxx.setValue(CHANNEL_ALL, potValue); //set volume
    }
}


void setup() {
    // initialize SPI
    SPI.begin();
    MCP42xxx mcp42xxx(10);   //slavePin 10

    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);
    pinMode(muteButtonPin, INPUT);
    pinMode(LED1Pin, OUTPUT);
    pinMode(LED2Pin, OUTPUT);

    loadConfig();
    storage.volume = constrain(storage.volume, maxStartupVol, minVolume);
    encoder0Pos = storage.volume;

    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoderA, CHANGE);
    // encoder pin on interrupt 1 (pin 3)
    attachInterrupt(1, doEncoderB, CHANGE); 

    updateMuted();
    startUp();
}


void loop() {

    int reading = digitalRead(muteButtonPin);

    if (changed) {
        if (millis() - lastChange > saveInterval && millis() - lastSave > saveInterval) {
            saveConfig();
            lastSave = millis();
            changed = FALSE;
        }
    }
    encoder0Pos = constrain(encoder0Pos, maxVolume, minVolume); //ensure encoderpos is within limits

    if (lastEncoder0Pos != encoder0Pos) { //encoder has turned
        if (!storage.muted && millis() - lastChange > changeInterval) { //not muted and change is not too soon since last
            storage.volume = encoder0Pos;
        } else { // if muted or change too often
            encoder0Pos = storage.volume;
            updateVolume();
        }
    }

    readMuteButton();
    readMultiButton();
}

