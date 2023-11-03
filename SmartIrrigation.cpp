/* Use a photoresitor and soil sensor to drive a pump and water plants when soil 
is dry. Using libraries from arduino and sample examples plus custom code*/
   
#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif   

//========================================================================================
// CONSTANTS THAT USER NEEDs TO SET             
//========================================================================================
const long threshold=75;           // % moisuture content as threshold to begin watering
const long criticalThreshold=50;   // point where moisutre content is critical is 50%
const long waterTillThreshold=90;  // % moisuture content as threshold when we stop watering
const boolean debug = true;        // Debug state or not
const double sleepTime = 5;        // minutes
const double dayRollover=24;       // hours
const long waterTime =5;           // Water Time = waterTime in seconds
const long delayTime = 60000;      // Pause in the loop  (millisecs)
const long sunnyThreshold = 900;   // water if dry only when sun is down
const long absorbTime = 5000;      // time to allow water to be absorbed (millisecs)
//=========================================================================================

//Constants
const int pResistor = A0;       // Photoresistor at Arduino analog pin A0
const int soilSensor= A1;       // soil moisture sensor Arduino analog pin A1
const int outPin=9;             // Led & Pump pin at Arduino digital pin 9
const int npnPin=10;            // Digital Pin 10 to powersoil sensor to prevent corrosion
const long sensorDelay=20;      // time after power up for soil sensor to come online
const long minMoisture=200;     // Full saturation, sensor reading is at the lowest reading is 200
const long maxMoisture=1023;    // sensor reading at which soils is considered fully dry - 1023
U8X8_SSD1306_128X64_NONAME_HW_I2C display(/* reset=*/ U8X8_PIN_NONE);  // initialize OLED display 

//Variables
long light;				              // Store light from photoresistor (0-1023)
long moisture=0;                // Store light from SoilSensor (250-1023)
double startTime = 0;           // Start time for the device
long timesWatered = 0;          // Number of time the water pump is activated during day
long durationWatered = 0;       // Total Amount of time water pump is run during day

// Variables for data analysis (capture data  since baord restart or day rollover)
double dayStartTime=0;          // start time for current day
long totalDurationWatered=0,totalTimesWatered=0,loopCount=0,waterLoopCount=0;
long moistureMax=0,moistureAdj=0,maxSunny=0;
long minSunny=9999,moistureMin=9999; // intial min values - sensor max values are 1023
double avgMoisture=0,adjAvgMoisture=0,avgLight=0; //Avg Daily value
String days[3][10];             // Array to store 10 datapoints(columns) for 3 days(rows)
int dayCount=0;                 // Number of days board has been on


void setup(){
  Serial.begin(9600);           // Initializing board  
  display.begin();              // initialize with the I2C addr 0x3D (for the 128x64)
  display.setPowerSave(0);      // wake up display
  display.setFont(u8x8_font_amstrad_cpc_extended_r);
  startTime=dayStartTime=millis();  // Start the time - global and first day
  pinMode(outPin, OUTPUT);      // Set outPin - 9 pin for LED Motor (digital)
  pinMode(npnPin, OUTPUT);      // Set npnPin - 10 pin for soil sensor (digital)
  pinMode(soilSensor, INPUT);   // Set soilSensor - A1 pin as an analog input
  pinMode(pResistor, INPUT);    // Set pResistor - A0 pin as an analog input   
  digitalWrite(outPin, LOW);    //  Outpupt signal to LED, motor off
}

void loop(){  
  display.setPowerSave(0);      // wake up display
  display.clearDisplay();       // Clear Display  
  digitalWrite(npnPin, HIGH);   // Switch power on to record analog moiture reading 
  delay(sensorDelay);           // time after power up for soil sensor to come online
  light = analogRead(pResistor);   // Read photoresistor
  moisture = map(constrain(analogRead(soilSensor),minMoisture,maxMoisture),maxMoisture,minMoisture,0,100);
  digitalWrite(npnPin, LOW);    // Switch power off to prevent soil sensor from corroding  
  calculateAverages();          // For data analysis run calcualtions and store data in array
  if (moisture > threshold){    // If moisture is ok write status and repeat loop
    writeState(false);
  } else if ((light < sunnyThreshold )||(moisture < criticalThreshold)) {  // else water
    water();                          //  Let Water run
  }
  delay(delayTime);                   //Small delay to allow time to read OLED display output
  sleep(sleepTime*60000-delayTime);   // Put display to sleep (total loop time ~ 5 mins)
  
}

void water(){  
  timesWatered++;
  long startMoisture=moisture;   // Moisture befor watering begins to adjust subsequent readings
  while (((moisture <= threshold) && (light >= sunnyThreshold))||
        ((moisture <= waterTillThreshold) && (light < sunnyThreshold))) {    
    writeState(true);            // write staus update to OLED display
    digitalWrite(outPin, HIGH);  // Output signal to LED, pump on
    delay(waterTime*1000);       // Let the pump run for this time
    digitalWrite(outPin, LOW);   // Stop pump to allow water to absorb
    delay(absorbTime);           // Let water be absorbed for this time
    digitalWrite(npnPin, HIGH);  // Switch power on to record analog moiture reading 
    delay(sensorDelay);          // delay to allow analog soil sensor to come online
    moisture = map(constrain(analogRead(soilSensor),minMoisture,maxMoisture),maxMoisture,minMoisture,0,100);
    digitalWrite(npnPin, LOW);   // Switch power off to prevent soil sensor from corroding   
    delay(sensorDelay);          // delay to allow analog soil sensor to go offline       
    light = analogRead(pResistor); // take light reading
    durationWatered = durationWatered + waterTime;
    storeArray();                // For data analysis run calculations and store readings in array
  } 
  // calculate Adjustment to subsequent moisture reading when water is completed in this sequence
  if (moisture > startMoisture) moistureAdj = moistureAdj+ (moisture - startMoisture); 
  waterLoopCount=loopCount;     // Store last time watered information
}

void calculateAverages(){
  if ((millis()-dayStartTime)>= dayRollover*3600000){   // Check day rollover, reset/initialize variables
    dayStartTime=dayStartTime+dayRollover*3600000; 
    storeArray();
    days[dayCount%3][8]=String(waterLoopCount);
    totalDurationWatered=totalDurationWatered+durationWatered;
    totalTimesWatered=totalTimesWatered+timesWatered;
    avgMoisture=adjAvgMoisture=avgLight=0;
    moistureMax=maxSunny=durationWatered=timesWatered=moistureAdj=waterLoopCount=loopCount=0;
    minSunny=moistureMin=9999;
    dayCount++;    
  }
  if (loopCount==0)days[dayCount%3][9]=pad(moisture,0,2)+"%";
  loopCount++;
  avgMoisture = (avgMoisture*(loopCount-1)+moisture)/loopCount; // calculate appropriate averages
  adjAvgMoisture = (adjAvgMoisture*(loopCount-1)+moisture-moistureAdj)/loopCount;
  avgLight = (avgLight*(loopCount-1)+light)/loopCount; 
  storeArray();                                                 // store data in array
}

void storeArray(){     
  int index=dayCount%3;    
  maxSunny=max(light,maxSunny);
  moistureMax=max(moisture,moistureMax);
  minSunny=min(light,minSunny);
  moistureMin=min(moisture,moistureMin);
  days[index][0]=pad(avgMoisture,0,2)+"%";
  days[index][1]=pad(avgLight,0,4);
  days[index][2]=pad(durationWatered,0,3);
  days[index][3]=pad(timesWatered,0,2);
  days[index][4]=pad(adjAvgMoisture,0,2)+"%";
  days[index][5]=pad(moistureMin,0,2)+"%";
  days[index][6]=pad(moistureMax,0,2)+"%";
  days[index][7]=pad(maxSunny,0,4);
}


void writeState(boolean isDry){                   // Write output to OLED
  String state="";
  int index = (dayCount%3);
  if (isDry)state = "D-"; else state="W-";  
  writeMsg(1,state+String(moisture)+"%-"+String(light)+"-"+String(dayCount)+"-"+String(loopCount));
  writeMsg(2,String(waterLoopCount)+"-"+days[max(dayCount-1,0)%3][8]+"-"
  +pad(totalDurationWatered,0,4)+"-"+pad(totalTimesWatered,0,2));  
  writeMsg(3,days[0][0]+days[0][1]+"-"+days[0][2]+"-"+days[0][3]); 
  writeMsg(4,days[0][4]+days[0][5]+days[0][6]+days[0][9]+days[0][7]); 
  writeMsg(5,days[1][0]+days[1][1]+"-"+days[1][2]+"-"+days[1][3]); 
  writeMsg(6,days[1][4]+days[1][5]+days[1][6]+days[1][9]+days[1][7]); 
  writeMsg(7,days[2][0]+days[2][1]+"-"+days[2][2]+"-"+days[2][3]); 
  writeMsg(8,days[2][4]+days[2][5]+days[2][6]+days[2][9]+days[2][7]); 
  writeMsg(index*2+3,days[index][0]+days[index][1]+"*"+days[index][2]+"*"+days[index][3]);
  writeMsg(index*2+4,days[index][4]+days[index][5]+days[index][6]+days[index][9]+days[index][7]);
}

// OLED and other utility functions
void sleep(long sleep){
  if (sleep<0) sleep = 10;
  display.setPowerSave(1);
  writeMsg(1,"sleeping...");      // putting to sleep
  delay(sleep);
}

void writeMsg (int row, String str){
  if (debug){
    Serial.println(str);
  }  
  msg(0,row-1,str);
}

void msg (int c, int r, String txt)
{
  int len = txt.length();
  for (int i=0; i< len; i++)
  {
    display.setCursor(c++,r);
    display.write(txt[i]);
  }
}

String pad(double num, int digits, int pad){ // formatting text output
    String fmtOut = String(num,digits);
    fmtOut.trim();
    if (num > pow(10,pad)) return fmtOut;
    if (pad>0) {
      for (int i=1; i<=pad ; i++){
        if (num < pow(10,i)){
          pad = pad-i;
          break;
        }
      }
    }
    if (pad>0){      
      for (int i=1; i<=pad ; i++){
        fmtOut=" "+fmtOut;
      }
    }
    return fmtOut;
}