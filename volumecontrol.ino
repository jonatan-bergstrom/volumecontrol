#include <SPI.h>
#include "mcp42xxx.h"
#include <EEPROM.h>
#include <AnalogButtons.h> //https://github.com/rlogiacco/AnalogButtons/
#include <Arduino.h>
#include <TM1637Display.h> //https://github.com/avishorp/TM1637
#include "OVC3860.h" https://github.com/tomaskovacik/OVC3860

#define resetBTPin 9
OVC3860 BT(&Serial1, resetBTPin);
uint16_t lastBTState;

#define ANALOG_PIN A1

#define CLK 2
#define DIO 3
TM1637Display display(CLK, DIO);
display.setBrightness(0x0f);

const uint8_t SEG_ON[] = {
    ,
    ,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
    SEG_C | SEG_E | SEG_G                           // n
    };

const uint8_t SEG_OFF[] = {
    ,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
    SEG_A | SEG_E | SEG_F | SEG_G,                   // F
    SEG_A | SEG_E | SEG_F | SEG_G,                   // F
    };

const uint8_t SEG_PAIR[] = {
    ,
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,          // P
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_G , // a    
    SEG_C ,                                         // i
    SEG_E | SEG_G,                                  // r
    };

unsigned int lastDisplayUpdate;
int displayOnTime = 2000;

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

void updateDisplay() {
    if (millis() - lastDisplayUpdate > displayOnTime) {
        display.clear();
    }
}

void muteButtonClick() {
    storage.muted = !storage.muted;
    updateMuted();
    changed = TRUE;
}
void playButtonClick() {
    BT.musicTogglePlayPause();
}
void nextButtonClick() {
    BT.musicNextTrack();
}
void prevButtonClick() {
    BT.musicPreviousTrack();
}
void bndButtonClick() {
    if (BT.BTState == Discoverable) {
        BT.pairingExit();
    }
}
void bndButtonHold() {
    if (BT.BTState == Discoverable) {
        BT.pairingExit();
    } else {
        BT.pairingInit();
    }

}
void oneButtonClick() {}
void twoButtonClick() {}
void threeButtonClick() {}
void fourButtonClick() {}
void fiveButtonClick() {}
void sixButtonClick() {}
void sixButtonClick() {}

AnalogButtons analogButtons(ANALOG_PIN, 30);

Button oneButton   = Button(930,&oneButtonClick);//10k  930
Button twoButton   = Button(838,&twoButtonClick);//22k  838
Button threeButton = Button(769,&threeButtonClick);//33k  769
Button fourButton  = Button(696,&fourButtonClick);//47k  696
Button fiveButton  = Button(609,&fiveButtonClick);//68k  609
Button sixButton   = Button(511,&sixButtonClick);//100k 511
Button muteButton  = Button(409,&muteButtonClick);//150k 409
Button playButton  = Button(319,&playButtonClick);//220k 319
Button nextButton  = Button(237,&nextButtonClick);//330k 237
Button prevButton  = Button(179,&prevButtonClick);//470k 179
Button bndButton   = Button(130,&bndButtonClick, &bndButtonHold, 2000, 10000);//680k 130


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
        display.setSegments(SEG_OFF);     
    } else {
        display.setSegments(SEG_ON);  
        storage.volume = constrain(storage.volume, maxStartupVol, minVolume);
        int potValue = round(pow(10,-storage.volume/20)*255); //UNMUTE   
    }
    lastDisplayUpdate = Millis();
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


void startUp() {
    display.setSegments(SEG_ON);

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

    BT.begin();

    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);

    pinMode(LED1Pin, OUTPUT);
    pinMode(LED2Pin, OUTPUT);

    analogButtons.add(oneButton);
    analogButtons.add(twoButton);
    analogButtons.add(threeButton);
    analogButtons.add(fourButton);
    analogButtons.add(fiveButton);
    analogButtons.add(sixButton);
    analogButtons.add(muteButton);
    analogButtons.add(playButton);
    analogButtons.add(nextButton);
    analogButtons.add(prevButton);
    analogButtons.add(bndButton);

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
            display.showNumberDec(storage.volume);
            lastDisplayUpdate = millis();
            updateVolume();
        }
    }

    analogButtons.check();

    updateDisplay();

    BT.getNextEventFromBT();

    If (BT.BTState == Discoverable) {
        display.setSegments(SEG_PAIR); 
        lastDisplayUpdate = millis();
        lastBTState = Discoverable;
    }

    if (lastBTState == Discoverable) && (BT.BTState != Discoverable) {
        display.clear();
    }
}
