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

#define analogButtonsPin A1

#define CLK 2
#define DIO 3
TM1637Display display(CLK, DIO);
display.setBrightness(0x0f);

unsigned int lastDisplayUpdate;
int displayOnTime = 2000;
int displayScrollInterval = 200; //move one character every n milliseconds
int displayScrollStartDelay = 400; //wait n milliseconds before starting to scroll

#define CONFIG_START 32
#define CONFIG_VERSION "ls1"

#define encoder0PinA 2
#define encoder0PinB 3

#define muteButtonPin 4
#define LED1Pin 5
#define LED2Pin 6
#define multiButtonPin A0

#define tempSensorPin A1
#define fanPin A2

#define slaveSelectPin = 10;

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

int numTempReadings = 10;
float sumTempReadings = 15 * numTempReadings;
unsigned long lastTempReading = 0;
int tempReadingInterval = 200; //how often to read temp in milliseconds
float fanStartThreshold = 40; //degrees celsius
float fanFullThreshold = 70; //degrees celsius
float fanMinDuty = 0.4; // minimum % PWM

struct Characters {
    char character;
    uint8_t hexCode;
};

Characters charArray[53] = {
  {"0", 0x7E}, {"1", 0x30}, {"2", 0x6D}, {"3", 0x79}, {"4", 0x33}, {"5", 0x5B}, {"6", 0x5F}, 
  {"7", 0x70}, {"8", 0x7F}, {"9", 0x7B}, {" ", 0x00}, {"A", 0x77}, {"a", 0x7D}, {"B", 0x7F}, 
  {"b", 0x1F}, {"C", 0x4E}, {"c", 0x0D}, {"D", 0x7E}, {"d", 0x3D}, {"E", 0x4F}, {"e", 0x6f}, 
  {"F", 0x47}, {"f", 0x47}, {"G", 0x5E}, {"g", 0x7B}, {"H", 0x37}, {"h", 0x17}, {"I", 0x30}, 
  {"i", 0x10}, {"J", 0x3C}, {"j", 0x38}, {"K", 0x37}, {"k", 0x17}, {"L", 0x0E}, {"l", 0x06}, 
  {"M", 0x55}, {"m", 0x55}, {"N", 0x15}, {"n", 0x15}, {"O", 0x7E}, {"o", 0x1D}, {"P", 0x67}, 
  {"p", 0x67}, {"Q", 0x73}, {"q", 0x73}, {"R", 0x77}, {"r", 0x05}, {"S", 0x5B}, {"s", 0x5B}, 
  {"T", 0x46}, {"t", 0x0F}, {"U", 0x3E}, {"u", 0x1C}, {"V", 0x27}, {"v", 0x23}, {"W", 0x3F}, 
  {"w", 0x2B}, {"X", 0x25}, {"x", 0x25}, {"Y", 0x3B}, {"y", 0x33}, {"Z", 0x6D}, {"z", 0x6D}
};

void setDisplay(char message) {
    while (message.length() < 4) {
        message = " " + message;
    }

    uint8_t hexMessage[message.length()];

    for (int c=0; c<message.length(): c++){
        hexMessage[c] = 0x00;
        for (i=0; i<sizeof(charArray); i++) {
            if (message.charAt(c) == charArray[i].character) {
                hexMessage[c] = charArray[i].hexCode;
                break;
            }
        }
    }
    display.setSegments(hexMessage);
    lastDisplayUpdate = Millis();
}

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
    if (BT.BTState == IncomingCall) {
        BT.callAnswer();
    } else if (BT.BTState == OngoingCall || BT.BTState == OutgoingCall) {
        BT.callHangUp();
    } else {
        BT.musicTogglePlayPause();
    }
    
}
void nextButtonClick() {
    if (BT.BTState == IncomingCall) {
        BT.callReject();
    } else {
        BT.musicNextTrack();
    }
}
void prevButtonClick() {
    if (BT.BTState == IncomingCall) {
        BT.callReject();
    } else {
        BT.musicPreviousTrack();
    }
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

AnalogButtons analogButtons(analogButtonsPin, 30);

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
        setDisplay("OFF");    
    } else {
        setDisplay("ON");
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


void startUp() {
    setDisplay("ON");

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

void updateFan() {
    if (millis() - lastTempReading = tempReadingInterval) {
        lastTempReading = millis();
        float temp = ((float)analogRead(A0) * 5.0 / 1024.0) - 0.5;
        temp = temp / 0.01;
        meanTemp = (sumTempReadings * (numTempReadings - 1) + temp) / numTempReadings;

        if (meanTemp >= fanStartThreshold) {
            duty = map(meanTemp, fanStartThreshold, fanFullThreshold , fanMinDuty * 255, 255);
            duty = constrain(duty, fanMinDuty * 255, 255)
            analogWrite(fanPin, duty);
        } else {
            analogWrite(fanPin, 0);
        }
    }
}


void setup() {
    // initialize SPI
    SPI.begin();
    MCP42xxx mcp42xxx(slaveSelectPin);   //slavePin 10

    BT.begin();

    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);
    pinMode(LED1Pin, OUTPUT);
    pinMode(LED2Pin, OUTPUT);
    pinMode(tempSensorPin, INPUT);
    pinMode(fanPin, OUTPUT);

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
        if (!storage.muted && millis() - lastChange >= changeInterval) { //not muted and change is not too soon since last
            storage.volume = encoder0Pos;
            setDisplay(String(storage.volume));
            updateVolume();
        } else { // if muted or change too often
            encoder0Pos = storage.volume;
            if (storage.muted) {
                setDisplay("OFF");
            }
        }
    }

    analogButtons.check();

    updateFan();

    updateDisplay();

    BT.getNextEventFromBT();

    If (BT.BTState == Discoverable) {
        setDisplay("Pair");

        lastBTState = Discoverable;
    }

    if (lastBTState == Discoverable) && (BT.BTState != Discoverable) {
        display.clear();
    }
}
