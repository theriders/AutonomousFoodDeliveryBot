/*
Last Updated by: Mitchell
Last Updated on: 11/9/2020
main execuable for The AutoVendors Senior Design Fall 2020
*/

#ifndef STUFF
#define STUFF
#include "mbed.h"
#include "functions.h"
#include "rtos.h"
#endif
#include <ros.h>
#include "ultrasonic.h"
#include <std_msgs/Float32.h>//Message type to be sent by the publisher
Mutex stateMutex;           //Mutex for the box State
Mutex navMutex;             //Mutex for navigation
Mutex unlockCodeMutex;      //Mutex for unlockCode
InterruptIn interrupt(p25); //Interrupt pin for keypad changed from pin26
//Limit Swithc Setup
DigitalIn ls(p11);
DigitalOut led3(LED3);//Monitor the limitswitch status

int unlockCode = 4873; //Varible to hold the unlock code CHANGED from float to int

ros::NodeHandle nh;         //ROS node Handler

//PUBLISHER SETUP
std_msgs::Float32 box_msg;  //Create message to be sent by ublisher

ros::Publisher boxState("boxState", &box_msg);  //create the publisher function_name(publisher_name,message)

//SUBSCRIBER SETUP
void messageCb(const std_msgs::Float32& toggle_msg);
void webCom(const std_msgs::Float32& toggle_msg);
ros::Subscriber<std_msgs::Float32> navMessage("navMsg", &messageCb);
ros::Subscriber<std_msgs::Float32> webCommands("webCommands", &webCom);

ultrasonic mu(p6, p5, .1, 1, &dist);//Trigger p6 Echo p5

volatile int stateOfBox = 0;    //Current state of the box: 0 = open, 1 = closed(filled), 2 = closed(empty)
volatile float navMess = 5;     //Current state of navMessage: 1 = home, 2 = delivery arrived, 3 = in transit,4 = delivery Requested, 0 = emergency open 5 = nothing recieved yet
//Start of function declarations
void deliveryArrived();

void deliveryRequested();

void pubBoxState(void const *args);

void sonar(void const *args);

bool limitSwitchStatus();
//End function declarations

int main()
{
    ls.mode(PullUp);        //Initialize pu;;up resistor for limit switch
    nh.initNode();          //Initialize Node handler
    nh.advertise(boxState); //advertise boxState
    int ip0, ip1, ip2, ip3; //Initialize ip address variables
    Thread t1(pubBoxState, NULL, osPriorityNormal, DEFAULT_STACK_SIZE);//Create thread to periodically run nh.spinOnce();
    nh.subscribe(navMessage);//Subscribe to navMessage
    nh.subscribe(webCommands);//Subscribe to webCommands
    interrupt.fall(&keypadPressed);     //When keypad is pressed, 
    interrupt.mode(PullUp);             //set interrupt to pullup mode
    Thread t4(sonar);       //Thread to check sonar data
    int navCopy;            //variable to hold a copy of navMsg
    int preventLoop = 0;    //Variable to prevent delivery loops in case messages arrive late
    while(1) {
        // START OF MAIN State Machine //
        navMutex.lock();
        navCopy = navMess;  //copy the global navMessage into a local variable
        navMutex.unlock();
        if(navCopy == 0) {
            //Emergency OPEN and reset
            clearLCDLine1();
            displayMessage("Resetting");
            openSolenoid();
            Thread::wait(1000);
            closeSolenoid();
            navMutex.lock();
            navMess = 5;
            navMutex.unlock();
            preventLoop = 0;
        } else if(navCopy == 1) {
            //waiting for delivery - waiting at home prints IP address to LCD line 1 
            writeIPAddress(ip0,ip1,ip2,ip3);
            displayMessageL2("Wait for order");
            Thread::wait(1000);
        } else if (navCopy == 2) {
            //delivery arrived,get new unlock code, run arrival function
            if(preventLoop == 0){
            nh.getParam("/unlock_code", &unlockCode,1);  //Get parameter set by ROS for the verification code
            setCode(unlockCode);
            deliveryArrived();
            }
            Thread::wait(2000);
            preventLoop = 1;        //value set to 1 to prevent reentering the delivery loopp in case no new navMessage is received
        } else if (navCopy == 3) {
            //bot is traveling wait and rint IP address to LCD line 1
            preventLoop = 0;
            writeIPAddress(ip0,ip1,ip2,ip3);
            clearLCDLine2();
            displayMessageL2("Traveling");
            Thread::wait(2000);
        } else if (navCopy == 4) {
            //delivery requested, run delivery requested function
            if(preventLoop == 0){
            deliveryRequested();
            }
            Thread::wait(2000);
            preventLoop = 1;
        } else if (navCopy == 5) {
            //Waiting for server to connect
            displayMessage("Waiting on");
            displayMessageL2("Server");
            //Grab the IP address paprameters//
            while(!nh.getParam("/ip_three", &ip3)); 
            while(!nh.getParam("/ip_two", &ip2));
            while(!nh.getParam("/ip_one", &ip1));
            while(!nh.getParam("/ip_zero", &ip0));
            }
        //END MAIN LOOP
    }
}

void deliveryArrived()//Function to run when the robot arrives at the destination
{
    clearLCDLine1();
    displayMessage("Enter Code");
    while(validation() == 0) { //Waiting for the key code to be entered    
        displayCode();          //Display current input value for the code
        Thread::wait(500);
    }
    clearLCDLine1();
    clearLCDLine2();
    //Open box protocol
    displayMessage("Opening Box");
    openSolenoid();
    Thread::wait(2000);
    closeSolenoid();
    //Change to new box state : Open
    stateMutex.lock();
    stateOfBox = 0;
    box_msg.data = stateOfBox;
    stateMutex.unlock();
    //Send box state to core node
    boxState.publish( &box_msg);
    int count = 0;
    clearLCDLine1();
    displayMessage("Remove Item");
    while(filledStatus() == 1) { //Waiting for item to be removed from the box
        ++count;                    //Increment timing counter        
        if(count == 60) {           //Timeout code if 30 seconds pass box might still be locked open box again
            count = 0;
            openSolenoid();
            Thread::wait(2000);
            closeSolenoid();
        }
        Thread::wait(500);
    }
    clearLCDLine1();
    displayMessage("Please Close Box");
    while(limitSwitchStatus() == true) { //Waiting for box top to be closed    
        Thread::wait(500);
    }
    clearLCDLine1();
    displayMessage("Thank You");
    //Change to new box state : Closed and Empty
    stateMutex.lock();
    stateOfBox = 2;
    box_msg.data = stateOfBox;
    stateMutex.unlock();
    //Send box state to core node
    boxState.publish( &box_msg);
}

void deliveryRequested()//Function to be executed when robot receives an order
{
    clearLCDLine1();
    clearLCDLine2();
    //Open box protocol
    displayMessage("Opening Box");
    openSolenoid();
    Thread::wait(2000);
    closeSolenoid();
    //Change to new box state : Open
    stateMutex.lock();
    stateOfBox = 0;
    box_msg.data = stateOfBox;
    stateMutex.unlock();
    //Send box state to core node
    boxState.publish( &box_msg);
    int count = 0;
    clearLCDLine1();
    displayMessage("Insert Item");
    while(filledStatus() == false) { //Waiting for item to be placed into the box
        ++count;                    //Increment timing counter
        if(count == 60) {           //Timeout code if 30 seconds pass box might still be locked open box again
            count = 0;
            openSolenoid();
            Thread::wait(2000);
            closeSolenoid();
        }
        Thread::wait(500);
    }
    clearLCDLine1();
    displayMessage("Please Close Box");
    while(limitSwitchStatus() == true) { //Waiting for box to be closed    
        Thread::wait(500);
    }
    clearLCDLine1();
    displayMessage("Thank You");
    //Send new box state : closed and filled
    stateMutex.lock();
    stateOfBox = 1;
    box_msg.data = stateOfBox;
    stateMutex.unlock();
    boxState.publish( &box_msg);
    //Thread::wait(5000);
}

void pubBoxState(void const* args)  //Thread to control the node handler
{
    while (true) {
        Thread::wait(2000);         //Wait 2 seconds
        nh.spinOnce();              //Uppdate all messages
    }
}

void messageCb(const std_msgs::Float32& toggle_msg)//receives nav message and assigns to nav message variable
{
    navMutex.lock();
    navMess = toggle_msg.data;      //Assignes new nav message to global variable
    navMutex.unlock();
}

void webCom(const std_msgs::Float32& toggle_msg)//receives web command and sets identification
{
    float webMsg = toggle_msg.data;
    if(webMsg == 0){            //Set identification == true
    setIdentification();
    }
}

void sonar(void const *args)
{
    mu.startUpdates();//start measuring the distance
    while(1) {
        mu.checkDistance();     //call checkDistance() as much as possible, as this is where
        //the class checks if dist needs to be called.
        Thread::wait(10);
    }
}

bool limitSwitchStatus()    //Read the status of the limit switch
{
    bool status = ls;
    led3 = ls;
    return status;
}