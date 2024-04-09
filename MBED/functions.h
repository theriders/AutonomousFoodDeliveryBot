/*
Last Updated by: Mitchell
Last Updated on: 11/9/2020
Contains all function declarations to be used in the main.cpp file for SeniorDesignMbed
*/
#ifndef FUNCTIONS
#define FUNCTIONS
/*
Interupt function to be called when a button is pressed
*/
void keypadPressed(); 

/*
Function to IP address to LCD line 1

@param IP string of IP address to be displayed
*/
void writeIPAddress(int ip0, int ip1, int ip2, int ip3);

/*
Function to clear LCD line 1
*/
void clearLCDLine1();

/*
Function to clear LCD line 2
*/
void clearLCDLine2();

/*
Function to write messages to line 1 of LCD

@param const char* message - message to be displayed on line 1 of the LCD
*/
void displayMessage(const char* message);

/*
Function to write messages to line 2 of LCD

@param const char* message - message to be displayed on line 2 of the LCD
*/
void displayMessageL2(const char* message);

/*
Function to display input code
*/
void displayCode();

/*
Function to open Solenoid
*/
void openSolenoid();

/*
Function to close Solenoid
*/
void closeSolenoid();

/*
Function to update test keypad input value
@param int keyCode - int specifying which number key was pressed
*/

void updateCheck(int keyCode);

/*
Function to run when key 11 is pressed
*/
bool enter();

/*
Function to send verification of key punch to main
*/
bool validation();

/*
Function to print distance
*/
void dist(int distance);

/*
Function to return filled state of box
*/
bool filledStatus();

/*
Function to set check value
*/
void setCode(float newCode);
/*
Function to set identification to 1

*/
void setIdentification();
#endif