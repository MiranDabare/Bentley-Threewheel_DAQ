///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// THREE WHEELER DATA ACQUISITION CODE ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


///// RANDOM ////////
unsigned long time_1;
float gear;

///////// ENCODER FILES ////////////
float value=0;
float pulse=0;
float rpm;
int oldtime=0;
int time;
float Velocity1=0;
float Velocity2=0;
float Avg_Vel=0;
float wheel_radius=0.15;
int slots = 60;
///////////////////////////
float valueR=0;
float pulseR=0;
float rpmR;
int oldtimeR=0;
float Velocity1R=0;
float Velocity2R=0;
float Avg_VelR=0;

///////// ENCODER INTERUPT //////////////
const byte interruptPin =2;
const byte interruptPinR =3;



float time_new=0;
float time_prev=0;

void isr() //interrupt service routine
{
pulse++;
}

void isrR() //interrupt service routine
{
pulseR++;
}

//////////////////////////////// SETUP ////////////////////////////////////////////////

void setup() {






//////////ENCODER INTERUPT//////////////////

pinMode(interruptPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(2),isr,RISING);  //attaching the interrupt
pinMode(interruptPinR, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(3),isrR,RISING);  //attaching the interrupt
Serial.begin(9600);
}


//////////////////////////////// LOOP ////////////////////////////////////////////////


void loop() {

 


////////////////////////////// DATA ACQUISITION /////////////////////////////

time_new = millis()- time_prev;


///////////////////// LEFT WHEEL SPEED CALCULATION ///////////////////

time=millis()-oldtime;        //finds the time 
rpm=((pulse/time)*60000)/slots;         //calculates rpm
Velocity1= (rpm*(2*3.14/60))*wheel_radius*3.6;
Avg_Vel = (Velocity1+Velocity2)/2;
oldtime=millis();
Velocity2=Velocity1;
pulse=0;
//attachInterrupt(0,isr,RISING);





//////////////// RIGHT WHEEL SPEED CALCULATION //////////////////////////

time=millis()-oldtimeR;        //finds the time 
rpmR=((pulseR/time)*60000)/slots;         //calculates rpm
Velocity1R= (rpmR*(2*3.14/60))*wheel_radius*3.6;
Avg_VelR = (Velocity1R+Velocity2R)/2;
oldtimeR=millis();
Velocity2R=Velocity1R;
pulseR=0;
//attachInterrupt(0,isr,RISING);



//ENCODER PRINT

Serial.print("Right RPM: ");
Serial.print(rpm);
Serial.print("Right Velocity : ");
Serial.print(Avg_Vel);

Serial.print("Left RPM: ");
Serial.print(rpmR);
Serial.print("Left Velocity : ");
Serial.println(Avg_VelR);
delay(50);
}
