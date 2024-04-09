/*
Last Updated by: Mitchell
Last Updated on: 10/12/2020
Contains all function executions for use in the main.cpp file for SeniorDesignMbed
TODO Create Array for Sonar values to test if box is empty
TODO Impliment sonar code
*/

#ifndef STUFF
#define STUFF
#include "mbed.h"
#include "functions.h"
#include "rtos.h"
#endif
#include <mpr121.h>
#include <stdio.h>
#include "TextLCD.h"
Serial pc(USBTX, USBRX);

Mutex validMutex;
Mutex mutexLCD;
Mutex checkMutex;
Mutex filledMutex;
/*GENERAL DEBUGGING LEDs*/
DigitalOut led1(LED1);//Monitor the Solenoid status
DigitalOut led2(LED2);//Monitor the Keypad Interrupt
DigitalOut led4(LED4);//Monitor Sonar sensor


/*KEYPAD SETUP*/
I2C i2c(p9,p10); // Setup the i2c bus on pins 9 and 10
// Setup the Mpr121:
// constructor(DevI2C object, i2c address of the mpr121)
Mpr121 mpr121(&i2c, Mpr121::ADD_VSS);
/*END KEYPAD SETUP*/

/*Sonar SETUP*/
#define WALL_VALUE 210

/*Solenoid SETUP*/
DigitalOut control(p8);

/*LCD SETUP*/
TextLCD lcd(p15, p16, p17, p18, p19, p20); // rs, e, d4-d7

// Other Variables
volatile int check = 0;                     //Current keypad input value
volatile int code = 4873;                   //Pass code value
volatile bool identification = false;       //bool if keypad input was correct
volatile int index = 0;                     //Location in values array
volatile bool filled = false;               //Bool if box is filled
volatile int values[5] = {0,0,0,0,0};       //Array to hold sonar values

void keypadPressed()
{
    led2 = 1;
    //BEGIN BUTTON INTERPRETATION
    int keyCode = 13;
    int i=0;
    int value=mpr121.read(0x00);
    value +=mpr121.read(0x01)<<8;
    // LED demo mod
    i=0;
    // puts key number out to LEDs for demo
    for (i=0; i<12; i++) {
        if (((value>>i)&0x01)==1) keyCode=i;
    }
    //END BUTTON INTERPRETATION
    //BEGIN KEY ACTIONS
    if(keyCode == 13) {
    } else if(keyCode!= 10 && keyCode != 11) { // keys 0-9 represent the # --call updateCheck(keyCode); call writeLCDLine2(keyCode)
        updateCheck(keyCode);
    } else if (keyCode == 10) {         // Key 10 is backspace --updateCheck(keyCode);
        updateCheck(keyCode);
    } else if (keyCode == 11) {         // Key 11 is enter    -- call function to compare check and code and set column = 0;
        validMutex.lock();
        identification = enter();
        validMutex.unlock();
    } else {}
    led2 = 0;
}

void writeIPAddress(int ip0, int ip1, int ip2, int ip3)//Display IP address from 4 input variables
{
    mutexLCD.lock();
    lcd.locate(0,0);
    lcd.printf("%d.%d.%d.%d",ip0,ip1,ip2,ip3);
    mutexLCD.unlock();
}

void clearLCDLine1()    //Clear line 2
{
    mutexLCD.lock();
    lcd.locate(0,0);
    lcd.printf("                ");
    mutexLCD.unlock();
}

void clearLCDLine2()    //Clear line 2
{
    mutexLCD.lock();
    lcd.locate(0,1);
    lcd.printf("                ");
    mutexLCD.unlock();

}

void displayMessage(const char* message)    //Displays a message on LCD line 1
{
    mutexLCD.lock();
    lcd.locate(0,0);
    lcd.printf(message);
    mutexLCD.unlock();
}

void displayMessageL2(const char* message)  //Displays a message on LCD line 2
{
    mutexLCD.lock();
    lcd.locate(0,1);
    lcd.printf(message);
    mutexLCD.unlock();
}

void displayCode()  //Display the input code to the LCD
{    
    clearLCDLine2();
    mutexLCD.lock();
    lcd.locate(0,1);
    checkMutex.lock();
    lcd.printf("%d", check);
    checkMutex.unlock();
    mutexLCD.unlock();
}

void openSolenoid()         //Control the status of the solenoid
{
    control = 1;
    led1 = 1;
}

void closeSolenoid()        //Control the status of the solenoid
{
    control = 0;
    led1 = 0;
}

void updateCheck(int keyCode)   //Update "check" variable to the new value based on the input from the keypad
{
    if(keyCode != 10) {
        checkMutex.lock();
        check *= 10;
        check += keyCode;
        if(check > 999999)  //Reset value if above 6 digits long
        {
            check = 0;
        }
        checkMutex.unlock();
    } else {
        checkMutex.lock();
        check /= 10;
        checkMutex.unlock();
    }
}

bool enter()                //Compare the "check" value with the "code" value and clear the value of check
{
    //clearLCDLine2();
    checkMutex.lock();
    if(check == code) {

        check = 0;
        checkMutex.unlock();
        return 1;
    } else {
        check = 0;
        checkMutex.unlock();
        return 0;
    }
}

bool validation()          // Return if the "check" value matches the "code" (issue sending identification into main)
{
    validMutex.lock();
    bool correct = identification;
    identification = false;
    validMutex.unlock();
    return correct;
}

void dist(int distance) //Interrut function to update values received from the sonar sensor and determines if box is filled
{
    int boxFill = 0;
    //put code here to execute when the distance has changed
    values[index] = distance;

    //fill array with 5 values
    index++;
    if (index > 4) {
        index = 0;
    }
    //walk the list
    //check if each value is above or below threshold values
    //if all - 1 are outside threshold range, box is filled
    //else box is empty
    for(int i = 0; i<5; ++i) {
        if(values[i] < (WALL_VALUE - 20) || values[i] > (WALL_VALUE+20)) {
            ++boxFill;
        }

    }
    if(boxFill > 3) {
        filledMutex.lock();
        filled = true;
        filledMutex.unlock();
        led4 = 1;
    } else {
        filledMutex.lock();
        filled = false;
        filledMutex.unlock();
        led4 = 0;
    }
}

bool filledStatus()     //Returns the current filled status of the box
{
    bool status;
    filledMutex.lock();
    status = filled;
    filledMutex.unlock();
    return status;
}

void setCode(float newCode) //Sets the unlock code to the value of newCode
{
    code = newCode;
}

void setIdentification()    //Sets identification to true
{
    validMutex.lock();
    identification = 1;
    validMutex.unlock();
}