/*team 3xM designed the line following code using PD. the car is able to drive whether on black or white line
 * by Mahra Alneyadi
 */
float Kp=93.5, Ki=0, Kd=121.5; //these values may not work well for your robot so keep tunning 
//kp=93.5 kd=121.5 >>best value so far with the speed of 75
//if the robot is not getting stable keep tunning it.. 
//for the sharp edges turn play with the Ki starting from 0.1 or 0.001

int P=0, I=0, D=0, error, prevEr=0, prevI=0, PIDvalue=0;
int sensor[]={0,0,0,0,0};
int inspd=75, leftspeed, rightspeed;
int flag;
int line=0; //stop line


void setup()
{
/*--------------determing each I/O pin and its type---------------*/
 pinMode(5,OUTPUT); //PWM Pin 1
 pinMode(6,OUTPUT); //PWM Pin 2
 pinMode(7,OUTPUT); //Left Motor Pin 1
 pinMode(8,OUTPUT); //Left Motor Pin 2
 pinMode(9,OUTPUT); //Right Motor Pin 1
 pinMode(10,OUTPUT);  //Right Motor Pin 2

 pinMode(A0,INPUT); //left most IR
 pinMode(A1,INPUT); //left middle IR
 pinMode(A2,INPUT); //middle IR 
 pinMode(A3,INPUT); //right middle IR
 pinMode(A4,INPUT); //right most IR

}

/*-------loop start--------*/

void loop(){
  
 readIR(); // read the values of IR sensors and its errors
 
 /* because we are having two finish lines so we want the car to stop at the last line */
if(error==5 && line==2){ //if we got the error =5 and the lin incremented to reach2 the car stops
  Stop();
  line=0; //set the line to 0 again
}

else{
  
 PID(); // calculating the PID 
 
 /**----------------- we will control the speed of motors by the PID value in range of 0-100-----------
 because some of the PID values could exceeds the limit of the motor speed so we add contrain----------**/
 
 leftspeed=constrain(inspd-PIDvalue,0,85); //left speed = initial speed - PID 
 rightspeed=constrain(inspd+PIDvalue,0,75); //right speed = initial speed + PID 
 
 analogWrite(5,leftspeed);//assign the left speed value to ENA
 analogWrite(6,rightspeed); //assign the right speed value to ENB
 
 /* -----we call the forward function because the motor will always drive forward 
   and it changes its direction depending on the PID value/speed of each motor-----*/
 forward(); 
}
}

void readIR(){
  /*IR line tracking sensor when detects black returns value of 0*/
  sensor[0]=digitalRead(A0);//left
  sensor[1]=digitalRead(A1);
  sensor[2]=digitalRead(A2);//middle
  sensor[3]=digitalRead(A3);
  sensor[4]=digitalRead(A4);//right
  
/*-----------this condition will decide which line the car is following-----------*/
      if (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) 
      flag=0;//middle detects the black line
      
 else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) 
      flag=1;//middle detects the white line

 switch(flag){ 

  case 0: {
 /*------------the car detecting black line then it will perform the conditions below----------*/

     if (sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) error=4;//right most IR 
else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) error=3;//right middle and righ most IR detects detects the line
else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==1) error=2; //right middle detects the line
else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) error=1; //middle and right middle IR detects the line
else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) error=0;//middle detects the lin return 0 error
else if (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) error=0;//middle, middle left and middle right detects the lin return 0 error
else if (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) error=-1;//middle and left middle IR detects the line
else if (sensor[0]==1 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) error=-2;//left middle detects the line
else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) error=-3;//left middle and left most IR detects detects the line
else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) error=-4;//left most IR
else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) {error= 5; line++;}
   }
   
/*-----------------------------------------------------------------------------------*/
   
   case 1:{
      /*-----------the car detecting white line then it will perform the conditions below---------------*/

     if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) error=4;//right most IR 
else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) error=3;//right middle and righ most IR detects detects the line
else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0) error=2; //right middle detects the line
else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) error=1; //middle and right middle IR detects the line
else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) error=0;//middle detects the lin return 0 error
else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) error=0;//middle, middle left and middle right detects the lin return 0 error
else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) error=-1;//middle and left middle IR detects the line
else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) error=-2;//left middle detects the line
else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) error=-3;//left middle and left most IR detects detects the line
else if (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) error=-4;//left most IR

   }
}
}


/*-----------PID controller functions-------------*/
void PID(){ 
   P = error; //proportional 
    I = I + error; //integral
    D = error - prevEr; //derivative
    PIDvalue = (Kp*P) + (Ki*I)+ (Kd*D); //function
    prevI=I;
    prevEr=error;

}

/*------------direction functions----------------*/
void forward()
{
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
 
}

void Stop()
{
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
}
