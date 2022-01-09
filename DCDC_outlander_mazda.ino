/*
  This program provides a skeleton framework for writing CAN programs for the EVTVDue.  It takes advantage of the EEPROM chip and CAN transceiver chips provided on
  the EVTVDue CAN microcontroller.  ANd it shows how to both read and write CAN frames using advanced interrupt techniques that only burden the microcontroller when an
  actual CAN frame is received or where this is a need to actually send a CAN frame.

  It also illustrates how to load up a CAN frame with data prior to sending, and how to retrieve it from received CAN frames.

  Finally, it provides routines to print data to an ASCII terminal screen connected to the NATIVE USB PORT on the microcontroller, and allows keyboard entry and 
  processing from the native USB port.

  Note that EVTVDue CAN microcontroller ONLY provides CAN0 and does not provide a CAN1 output.   It also ONLY supports the NATIVE USB port and so this code will not 
  work on the programming port as written.
  
  copyright 2015 Jack Rickard, Collin Kidder
    
   07/24/2015
   For further info, contact jack@evtv.me 
   
  This sketch uses the EVTVDue Microcontroller and is compatible with Arduino Due  Use BOARD select Arduino Due (Native USB port).

Mitsubishi Outlander Charger CAN
0x377h 8bytes DC-DC converter status
  B0+B1 = 12V Battery voltage (h04DC=12,45V -> 0,01V/bit)
  B2+B3 = 12V Supply current  (H53=8,3A -> 0,1A/bit)
  B4 =  Temperature 1   (starts at -40degC, +1degC/bit)
  B5 =  Temperature 2   (starts at -40degC, +1degC/bit)
  B6 =  Temperature 3   (starts at -40degC, +1degC/bit)
  B7 =  Statusbyte    (h20=standby, h21=error, h22=in operation)
    bit0(LSB) = Error 
`   bit1    = In Operation
    bit3      =
    bit4      =
    bit5      = Ready
    bit6    = 
    bit7(MSB) = 
  
  
0x389h 8bytes Charger status
  B0 = HV-Batteryvoltage    (hAB=342V -> 2V/bit)
  B1 = AC-Mains voltage     (hE2=226V -> 1V/bit)
  B2 = DC-charge current (1?) (h5B=9,1A -> 0,1A/bit
  B3 = Temperature 1    (starts at -40degC, +1degC/bit)
  B4 = Temperature 2    (starts at -40degC, +1degC/bit)
  B5 = Statusbyte     (CA while charging)
    bit0(LSB) = 
`   bit1    = Mains voltage present
    bit2      = 
    bit3      = Charging
    bit4      = Error (no CAN messages received)
    bit5      =
    bit6    = DC-DC converter request active
    bit7(MSB) = 1KHz pilot present    
  B6 = AC-Mains current     (h91=14,5A -> 0,1A/bit)
  B7 = DC-charge current (2?) (same as B3, sometimes differs by 0,1A)


0x38ah 8bytes Charger status
  B0 = Temperature    Raises together with other temperatures, but with +5deg offset
  B1 = Temperature    (starts at -40degC, +1degC/bit)
  B2 = DC-bus voltage     (9B=310V -> 2V/bit)
  B3 = PWM-signal from EVSE   (h19=25% -> 1%/bit)
  B4 = Status
    bit0(LSB) = 
`   bit1    =
    bit2    = Wait for mains voltage 
    bit3      = Ready for charging
    bit4      =
    bit5      =
    bit6    = 
    bit7(MSB) = 
  B5 = 00
  B6 = 00
  B7 = 00


SEND
0x285 B3  = hB6 enables charging, other bytes can stay 00
0x286 B1+B2   = voltage setpoint    (0E74=370.0V, 0,1V/bit) -> How to enable CV mode ??
      B3  = current setpoint DC-side  (78=12A -> 0,1A/bit)

OPERATORS
and             &&
and_eq          &=
bitand          &
bitor           |
not             !
not_eq          !=
or              ||
or_eq           |=
xor             ^
xor_eq          ^=

   
*/

#include <due_can.h>  //https://github.com/collin80/due_can
#include <due_wire.h> //https://github.com/collin80/due_wire
#include <DueTimer.h>  //https://github.com/collin80/DueTimer
#include <Wire_EEPROM.h> //https://github.com/collin80/Wire_EEPROM


//These two lines redefine any serial calls to the native Serial port on the CANdue or EVTVDue and enables syntactic sugar to allow streaming to serial port Serial<<"Text";
#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;

 // Useful macros for setting and resetting bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// PP pin from EVSE
int PP_pin = 36; // input connected to pin 36
int CP_pin = 44; // CP signal
int PP_relay = 30; // relay output connected to pin 30

// DCDC pin Enable
int Enable_pin = 38; // Enable input connected to pin 38
int DCDC_active = 46; // relay output connected to pin 46
bool auxState = false; //create + name flag

// BMS pin input
int BMS_pin = 42; // BMS input connected to pin 42

//PWM output pins
int PWM1 = 2;
int PWM2 = 3;


//*********GENERAL VARIABLE   DATA ******************

CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME gearFrame; //We want a separate one for msg 228 as we use a timer interrupt to send this one every 10 ms


//Other ordinary variables
float Version=1.30;
uint16_t page=300;    //EEPROM page to hold variable data.  We save time by setting up a structure and saving a large block
int i;
unsigned long elapsedtime,time228,timestamp,startime,lastime;  //Variables to compare millis for timers
const unsigned int ontime = 120000; //3min for on time
boolean debug=false;
uint8_t logcycle=0;
uint8_t voltage = 0;
uint8_t Ctemp = 0;
uint8_t aux1 = 0;
uint8_t aux2 = 0;
uint8_t auxc1 = 0;
uint8_t auxc2 = 0;
uint8_t DCDCstatus = 0;
float auxvoltage = 0; 
float auxcurrent = 0;
//******* END OF GENERAL VARIABLE DATA***********






//*********EEPROM  DATA ******************
/*
 * This section provides a data struction and function for reading and writing EEPROM data containing persistent configuration variable we want to retain from 
 * one session to the next.  ANy kind of legal variable can be added to the structure and periodically during main loop execution we will write all variable data as a block\
 * to the EEPROM.  This minimizes the time delay hit of EEPROM writes.  You can update these variables as often as you like in other sections of the program and 
 * it will be actually saved to EEPROM during the next block write.
 * 
 * The initialize function is normally executed on new hardware where no previously written EEPROM data is found.  You can set default values here and on the first program 
 * execution those values will be loaded and used by the program.  Subsequent changes will be written to EEPROM and during the next session, the updated variables will be 
 * retrieved from EEPROM.
 */

//Store any configuration variables or other data you want to be persistent between power cycles.  Available in myVars.variable

class EEPROMvariables {
  public:
    uint8_t CANport;
    uint32_t datarate;
    uint16_t transmitime;
    uint8_t varsGood;
};
EEPROMvariables myVars;

void initializeEEPROMPage()
{
  //If no EEPROM data can be found at the EEPROM page, initialize the variables held in myVars here.  It will then be written to the designated EEPROM page
  
  myVars.CANport=0;
  myVars.datarate=500000;
  myVars.transmitime=800;
  myVars.varsGood=20;
  EEPROM.write(page,myVars);

}

//*********EEPROM  DATA ******************

\





//********************SETUP FUNCTION*******I*********
/*
 * The SETUP function in the Arduino IDE simply lists items that must be performed prior to entering the main program loop.  In this case we initialize Serial 
 * communications via the USB port, set an interrupt timer for urgent outbound frames, zero our other timers, and load our EEPROM configuration data saved during the 
 * previous session.  If no EEPROM data is found, we initialize EEPROM data to default values
 * 
 */
void setup() 
  {
   
    Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.
    Timer4.attachInterrupt(sendCANframeURGENT).start(100000); // This sets an interrupt time to send an URGENT CAN frame every 100ms.  The function can be changed
                                                             // and the time can be changed.
    
    lastime=startime=time228=timestamp=millis();  //Zero our other timers
    elapsedtime=millis();

 //Load/validate EEPROM variables
    Wire.begin();
    EEPROM.setWPPin(19);     
    EEPROM.read(page, myVars);
    if(myVars.varsGood!=20)initializeEEPROMPage(); //We check to see if we have a saved page from earlier session. If not, initialize EEPROM variables and save.
 
//  Initialize CAN ports 
    if(myVars.CANport==0) initializeCAN(0);     
       else initializeCAN(1); 

  Can0.begin(CAN_BPS_500K); //directly declare CAN0 at 500K

 //Print welcome screen and menu  
    Serial<<"\n\n Startup successful. EVTV Motor Werks -  Program Name Version "<<Version<<"\n\n";
    printMenu();    

pinMode(PP_pin,INPUT_PULLUP); // set PP pin to input with using built in pull up resistor
pinMode(PP_relay,OUTPUT); // 
pinMode(Enable_pin,INPUT); // BMS LOW PIN
pinMode(DCDC_active,OUTPUT); 
pinMode(BMS_pin,INPUT_PULLUP); //BMS HIGH PIN
pinMode(PWM1,OUTPUT);
pinMode(PWM2,OUTPUT); 

Can0.watchFor();  

}
   
//********************END SETUP FUNCTION*******I*********





//********************MAIN PROGRAM LOOP*******I*********
/*
 * This is the main program loop. Lacking interrupts, this loop is simply repeated endlessly.  Show is an example of code that only executes after a certain
 * amount of time has passed. myVars.transmitime.  It also uses a counter to perform a very low priority function every so many loops.
 * Any other entries will be performed with each loop and on Arduino Due, this can be very fast.
 */



void loop()
{ 
  CAN_FRAME incoming;

  if (Can0.available() > 0) {
    Can0.read(incoming); 
    if(incoming.id==0x389){
      voltage=incoming.data.bytes[0] ;
      Ctemp=incoming.data.bytes[4] ;      
    } 
    if(incoming.id==0x377){
      aux1=incoming.data.bytes[0] ;
      aux2=incoming.data.bytes[1] ;   
      auxc1=incoming.data.bytes[2] ;
      auxc2=incoming.data.bytes[3] ;
      DCDCstatus=incoming.data.bytes[7] ;
      auxvoltage=((aux1 * 256) + aux2); //recalculate two bit voltage value
      auxcurrent=((auxc1 * 256) + auxc2); //recalculate two bit current value      
    }  
    }
    
if(digitalRead(PP_pin) == LOW) { // if PP_pin senses EVSE
digitalWrite(PP_relay,HIGH); // turn on Cp_relay  
//chargertempctrl();

  if(millis()-lastime > myVars.transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
    {   
     lastime=millis();        //Zero our timer
       sendCANframeA();
       printstatus(); 
       sendCANframeB();
       printstatus(); 
       } 
       
//analogWrite(PWM1, 180); // Write 70% PWM inverted duty for both Outlander chargers   
           
}
else {
digitalWrite(PP_relay,LOW); // turn off PP_relay
//analogWrite(PWM1, 0); // Write 0% PWM duty for both Outlander chargers  
 } 

if (digitalRead(Enable_pin) == HIGH) { // if Enable_pin senses ignition 
digitalWrite(DCDC_active,HIGH); // turn on DCDC_active relay 
} 
else {
digitalWrite(DCDC_active,LOW); // turn off DCDC_active relay
}
    
if (auxvoltage < 1200) { // if aux voltage is low and DCDC is off
auxState = true; // set the flag to true   
elapsedtime=millis();  
}

DCDCauxcharge();


     if(logcycle++ > 200) //Every 30 seconds, save configuration and data to EEPROM
       {  
        logcycle=0;
        Timer4.stop();    //Turn off the 10ms interrupt for 228 to avoid conflicts
        EEPROM.write(page, myVars);
        Timer4.start();
        }         
}
//********************END MAIN PROGRAM LOOP*******I*********






  
 //********************USB SERIAL INPUT FROM KEYBOARD *******I*********
 /* These routines use an automatic interrupt generated by SERIALEVENT to process keyboard input any time a string terminated with carriage return is received 
  *  from an ASCII terminal program via the USB port.  The normal program execution is interrupted and the checkforinput routine is used to examine the incoming
  *  string and act on any recognized text.
  * 
  */
   
void serialEventRun(void) 
{
   if (Serial.available())checkforinput(); //If serial event interrupt is on USB port, go check for keyboard input           
}

void DCDCauxcharge()
{ 
if ((auxState == true) && (digitalRead(Enable_pin) == LOW)) { // auxvoltage went below 12.2V
digitalWrite(DCDC_active,HIGH);   
if (millis() - elapsedtime >= ontime) { // if aux voltage is low and for 5min 
digitalWrite(DCDC_active,LOW); // turn off DCDC_active relay 
elapsedtime=millis();
auxState = false;
}}
else { // if auxvoltage is OK 
auxState = false; // turn off DCDC_active relay
}
}

void chargertempctrl()
{
if (Ctemp >= 95) { // if Ctemp is high
digitalWrite(DCDC_active,HIGH); // turn off DCDC_active relay 
} 
else { // if Ctemp is lower 
digitalWrite(DCDC_active,LOW); // turn off DCDC_active relay
}}

void checkforinput()
{
  //Checks for input from Serial Port 1 indicating configuration items most likely.  Handle each case.
  
   if (Serial.available()) 
   {
    int inByte = Serial.read();
    switch (inByte)
     {
    	 case '?':
      		printMenu();
      		break; 	
       case 'h':
      		printMenu();
      		break;
     	 case 'D':
      		debug=(!debug);
      		break;
	     case 'd':
      		debug=(!debug);
      		break;
     
       case 'i':
      		getInterval();
      		break;
       case 'I':
      		getInterval();
      		break;
	     case 'c':
      		getPort();
      		break;
       case 'C':
      		getPort();
      		break;
       case 'k':
      		getRate();
      		break;
       case 'K':
      		getRate();
      		break;
      
   	  }    
    }
}

void getInterval()
{
	Serial<<"\n Enter the interval in ms between each CAN frame transmission : ";
		while(Serial.available() == 0){}               
		float V = Serial.parseFloat();	
		if(V>0)
		  {
        Serial<<V<<"CAN frame interval\n\n";
        myVars.transmitime=abs(V);
      }
}
       

void getRate()
{
	Serial<<"\n Enter the Data Rate in Kbps you want for CAN : ";
		while(Serial.available() == 0){}               
		float V = Serial.parseFloat();	
		if(V>0)
		  {
       Serial<<V<<"\n\n";
       myVars.datarate=V*1000;
       initializeCAN(myVars.CANport);
		  }
}


void getPort()
{
	Serial<<"\n Enter port selection:  c0=CAN0 c1=CAN1 ";
	while(Serial.available() == 0){}               
	int P = Serial.parseInt();	
	myVars.CANport=P;
	if(myVars.CANport>1) Serial<<"Entry out of range, enter 0 or 1 \n";
  else 
     {
     if(myVars.CANport==0) initializeCAN(0); //If CANdo is 0, initialize CAN0 
     if(myVars.CANport==1) initializeCAN(1);    //If CANdo is 1, initialize CAN1
   	 }           
}

//********************END USB SERIAL INPUT FROM KEYBOARD ****************









//******************** USB SERIAL OUTPUT TO SCREEN ****************
/*  These functions are used to send data out the USB port for display on an ASCII terminal screen.  Menus, received frames, or variable status for example
 *   
 */

void printMenu()
{
  Serial<<"\f\n=========== Tesla Model S CAN Controller  Version "<<Version<<" ==============\n************ List of Available Commands ************\n\n";
  Serial<<"  ? or h  - Print this menu\n";
  Serial<<"  c - sets CAN port ie c0 or c1\n";
  Serial<<"  d - toggles debug DISPLAY FRAMES to print CAN data traffic\n";
  Serial<<"  i - set interval in ms between CAN frames i550 \n";
  Serial<<"  k - set data rate in kbps ie k500 \n";
 
 Serial<<"**************************************************************\n==============================================================\n\n"; 
}

void printFrame(CAN_FRAME *frame,int sent)
{ 
  char buffer[300];
  sprintf(buffer,"msgID 0x%03X; %02X; %02X; %02X; %02X; %02X; %02X; %02X; %02X  %02d:%02d:%02d.%04d\n", frame->id, frame->data.bytes[0], 
  frame->data.bytes[1],frame->data.bytes[2], frame->data.bytes[3], frame->data.bytes[4], frame->data.bytes[5], frame->data.bytes[6],
  frame->data.bytes[7], hours(), minutes(), seconds(), milliseconds());
  
   if(sent)Serial<<"Sent ";
    else Serial<<"Received ";       
   Serial<<buffer<<"\n";
}

void printstatus()
{
  /* This function prints an ASCII statement out the SerialUSB port summarizing various data from program variables.  Typically, these variables are updated
   *  by received CAN messages that operate from interrupt routines. This routine also time stamps the moment at which it prints out.
   */
  
    char buffer[300]; 
  //  sprintf(buffer,"CAN%i Int:%ims Regen:%i%% Throt:%i%% RPM:%i MPH:%i Torq %iNm %3.1fvdc %iAmps %3.1fkW MotT1:%iC InvT5:%iC  %02d:%02d:%02d.%04d\n\n",myVars.CANport,
//    myVars.transmitime,regenpercent,throttle,rpm,mph, torq1,voltage,current,power,temperature1,temperature5,hours(),minutes(),seconds(),milliseconds());
 
      sprintf(buffer,"CAN%i Rate:%i Int:%ims  %02d:%02d:%02d.%04d\n\n",myVars.CANport,myVars.datarate, myVars.transmitime,hours(),minutes(),seconds(),milliseconds());
 
    Serial<<buffer;
}


int milliseconds(void)
{
  int milliseconds = (int) (micros()/100) %10000 ;
  return milliseconds;
}


 int seconds(void)
{
    int seconds = (int) (micros() / 1000000) % 60 ;
    return seconds;
}


int minutes(void)
{
    int minutes = (int) ((micros() / (1000000*60)) % 60);
    return minutes;
}

    
int hours(void)
{    
    int hours   = (int) ((micros() / (1000000*60*60)) % 24);
    return hours;
}  



//******************** END USB SERIAL OUTPUT TO SCREEN ****************






//******************** CAN ROUTINES ****************************************
/* This section contains CAN routines to send and receive messages over the CAN bus
 *  INITIALIZATION routines set up CAN and are called from program SETUP to establish CAN communications.
 *  These initialization routines allow you to set filters and interrupts.  On RECEIPT of a CAN frame, an interrupt stops execution of the main program and 
 *  sends the frame to the specific routine used to process that frame by Message ID. Once processed, the main program is resumed.
 *  
 *  Frames can also be sent, either from the main control loop, or by a Timer interrupt allowing specific frames to be sent on a regular interrupt interval.
 *  
 *  For example a frame that MUST go out every 10 ms would use the Timer Interrupt in SETUP to cause a CAN function here to send a frame every 10 ms.  Other frames 
 *  could be sent normally from the main program loop or conditionally based on some received or calculated data.
 *  
 */

void initializeCAN(int which)
{
  //Initialize CAN bus 0 or 1 and set filters to capture incoming CAN frames and route to interrupt service routines in our program.
  
  if(which)  //If 1, initialize Can1.  If 0, initialize Can0.
    {
      pinMode(48,OUTPUT);
      if (Can1.begin(myVars.datarate,48)) 
        {
          Serial.println("Using CAN1 - initialization completed.\n");
          Can1.setNumTXBoxes(4);
          Can1.setRXFilter(0, 0x106, 0x7FF, false);
          Can1.setCallback(0, handleCANframe);
          Can1.setRXFilter(1, 0x106, 0x7FF, false);
          Can1.setCallback(1, handleCANframe);
          Can1.setRXFilter(2, 0x274, 0x7FF, false);
          Can1.setCallback(2, handleCANframe);
          Can1.setRXFilter(3, 0x1D6, 0x7FF, false);
          Can1.setCallback(3, handleCANframe);
          Can1.setRXFilter(4, 0x30A, 0x7FF, false);
          Can1.setCallback(4, handleCANframe);
          Can1.setRXFilter(5, 0x308, 0x7ff, false);
          Can1.setCallback(5, handleCANframe);
          Can1.setRXFilter(6, 0x268, 0x700, false);
          Can1.setCallback(6, handleCANframe);           
        }
           else Serial.println("CAN1 initialization (sync) ERROR\n");
    } 
  else
    {//else Initialize CAN0
     pinMode(50,OUTPUT);
     if (Can0.begin(myVars.datarate,50)) 
        {
          Serial.println("Using CAN0 - initialization completed.\n");
          Can0.setNumTXBoxes(4);
          Can0.setRXFilter(0, 0x389, 0x390, false);
          Can0.setCallback(0, handleCANframe);
          Can0.setRXFilter(1, 0x389, 0x390, false);
          Can0.setCallback(1, handleCANframe); 
          Can0.setRXFilter(2, 0x377, 0x378, false);
          Can0.setCallback(2, handleCANframe);
          Can0.setRXFilter(3, 0x377, 0x378 , false);
          Can0.setCallback(3, handleCANframe);                       
          Can0.setRXFilter(4, 0x100, 0x700, false);
          Can0.setCallback(4, handleCANframe);
          }
        else Serial.println("CAN0 initialization (sync) ERROR\n");
    }
}   

void handleCANframe(CAN_FRAME *frame)
//This routine handles CAN interrupts from a capture of any other CAN frames from the inverter not specifically targeted.  
//If you add other specific frames, do a setRXFilter and CallBack mailbox for each and do a method to catch those interrupts after the fashion
//of this one.
{  
    //This routine basically just prints the general frame received IF debug is set.  Beyond that it does nothing.
    
    if(debug) printFrame(frame,0); //If DEBUG variable is 1, print the actual message frame with a time stamp showing the time received.      
}

void handle389frame(CAN_FRAME *frame)
//This routine handles CAN interrupts from  Address 0x106 CAN frame   
{   
int voltage;  
int Ctemp;
//  float throttle;
//  int rpm;
//  int wheelrpm;
//  int torq2;
//  int torq1;
voltage=word(frame->data.bytes[0]*2); 
Ctemp=word(frame->data.bytes[4]);  
//    throttle=round(frame->data.bytes[6]/2.56);
//    rpm=word(frame->data.bytes[5],frame->data.bytes[4]);
//    wheelrpm=rpm/9.73;
//    torq2=(word(frame->data.bytes[3],frame->data.bytes[2]))&8191;
//      if(torq2>4095)torq2=(8192-torq2)*-1;
//      torq2/=10;
//    torq1=(word(frame->data.bytes[1],frame->data.bytes[0]))&8191;
//      if(torq1>4095)torq1=(8192-torq1)*-1;
//     torq1/=10;
      
    if(debug)printFrame(frame,0); //If DEBUG variable is 1, print the actual message frame with a time stamp showing the time received.    
}

// 0x377h 8bytes DC-DC converter status
//  B0+B1 = 12V Battery voltage (h04DC=12,45V -> 0,01V/bit)
//  B2+B3 = 12V Supply current  (H53=8,3A -> 0,1A/bit)
//void handle377frame(CAN_FRAME *frame)
////This routine handles CAN interrupts from  Address 0x377 CAN frame   
//{    
//float auxvoltage; 
//float auxcurrent;
//auxvoltage=word((frame->data.bytes[0], frame->data.bytes[1])); // word(Highbyte, Lowbyte)
//auxcurrent=word((frame->data.bytes[2], frame->data.bytes[3])); // word(Highbyte, Lowbyte)
//int combined = ((int) highByte << 8) | lowByte;

//}

void sendCANframeB()
{
        outframe.id = 0x02FF;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x01;
        outframe.data.bytes[1]=0xE8;  
        outframe.data.bytes[2]=0x03; 
        outframe.data.bytes[3]=0x28; //
        outframe.data.bytes[4]=0x0F;
        if(digitalRead(BMS_pin) == HIGH) { // if BMS_pin is high
        outframe.data.bytes[5]=0x78; // 78=12A -> 0,1A/bit
        } 
        else {
        outframe.data.bytes[5]=0x00;
        } 
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
       
        if(debug) {printFrame(&outframe,1);} //If the debug variable is set, show our transmitted frame
                      
        if(myVars.CANport==0) Can0.sendFrame(outframe);    //Mail it
         else Can1.sendFrame(outframe); 
}

void sendCANframeA() {
outframe.id = 0x286; // Set our transmission address ID
outframe.length = 8; // Data payload 8 bytes
outframe.extended = 0; // Extended addresses - 0=11-bit 1=29bit
outframe.rtr=1; //No request
outframe.data.bytes[0]=0x28;
outframe.data.bytes[1]=0x0F; // 0F3C=3900, 0DDE=3550, 0,1V/bit
if(voltage < 193) { // if Charger senses less than 386V
outframe.data.bytes[2]=0x78; // 78=120 12A, 50=80 8A, 32=50 5A, 1E=30, 3A 14=20 2A at 0,1A/bit
}
else if(voltage <= 194) { // if Charger senses less than or equal 388V
outframe.data.bytes[2]=0x1E;
}
else {  //any other case
outframe.data.bytes[2]=0x00;
}
outframe.data.bytes[3]=0x37; // why 37?
outframe.data.bytes[4]=0x00;
outframe.data.bytes[5]=0x00;
outframe.data.bytes[6]=0x0A;
outframe.data.bytes[7]=0x00;
if(debug) {printFrame(&outframe,1); } //If the debug variable is set, show our transmitted frame
if(myVars.CANport==0) Can0.sendFrame(outframe); //Mail it
else Can1.sendFrame(outframe);
}


void sendCANframeURGENT() {
        outframe.id = 0x285;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x00; 
        outframe.data.bytes[1]=0x00;
        if(digitalRead(BMS_pin) == HIGH) { // All is well with BMS signal
        outframe.data.bytes[2]=0xB6; // PP EVSE connected signal
        }   
        else {
        outframe.data.bytes[2]=0x00; //if BMS senses pack voltage reached full
        }      
        outframe.data.bytes[3]=0x00;
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=0x00;
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
        if(debug) {printFrame(&outframe,1);} //If the debug variable is set, show our transmitted frame
        if(myVars.CANport==0) Can0.sendFrame(outframe);    //Mail it
         else Can1.sendFrame(outframe); 
}

//******************** END CAN ROUTINES ****************
