#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>

const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;
float roll, pitch, yaw;
int c=0;
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
//int echo=A4;
//int trig=A5;
long duration, cm;
int buzzer=2;
// Definitions Arduino pins connected to input H Bridge
int IN1 = 2;
int IN2 = 12;
int IN3 = A3;
int IN4 = 13;

int sr=A2;   //sensor right
int sl=A1;   //sensor left
int svr=0;
int svl=0;

int en=11;
int enl=11;
int enr=3;
float lspeed=128;
float rspeed=128;
int vspeed=100; 
int tspeed=255; 
 
int A=A2;
int B=A1;
int counterA = 0;
int currentStateA;
int lastStateA;
int counterB=0;
int lastStateB;
int currentStateB;
int error=0;
int preverror=0;
//int average;
//int afterramp=0;

//char state;
SoftwareSerial Bluetooth(A5, A4); // RX, TX  
int Data; // the data received

void setup(){
  /*digitalWrite(IN1,HIGH); //move forward
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,255);
  delay(100);*/
 
  //Bluetooth.begin(9600);

  //Serial.begin(9600);
  /*Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission*/
  lcd.begin(16, 2);              // start the library*/

//pinMode(trig,OUTPUT);
//pinMode(echo,INPUT);
// Set the output pins
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);

pinMode(A,INPUT);
pinMode(B,INPUT);

//pinMode(sr,INPUT);
//pinMode(sl,INPUT);
//pinMode(enr, OUTPUT);
pinMode(enl, OUTPUT);
pinMode(enr,OUTPUT);



/*while(millis()<1500){
digitalWrite(IN1,HIGH); //move forward
      digitalWrite(IN2,LOW);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      analogWrite (en,128);
}
analogWrite (en,0);
delay(1000);
calculate_angle();
lcd.setCursor(0,1);
  lcd.print("pitch");
  lcd.setCursor(8,1);
  lcd.print(pitch);
  delay(1000);
  digitalWrite(IN1,HIGH); //move forward
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,255);
  delay(600);
  analogWrite (en,0);
  delay(4000);
  digitalWrite(IN1,LOW); //turn right
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,tspeed);
  delay(1200);
  digitalWrite(IN1,HIGH); //move forward
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,128);
  delay(500);
  lastStateA = digitalRead(A);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Distance");
  while(counterA<230){
    calculate_distance();
    lcd.setCursor(0,1);
    lcd.print(counterA);
    lcd.print("cm");
    follow_line(64);
  }
  analogWrite (en,0);
  delay(2000);
  digitalWrite(IN1,HIGH); //move forward
      digitalWrite(IN2,LOW);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      analogWrite (en,128);
      delay(250);
  while(1){
    calculate_distance();
    lcd.setCursor(0,1);
    lcd.print(counterA);
    lcd.print("cm");
    follow_line(64);
  }*/






//calculate_IMU_error();
//while(1){};
lastStateA=digitalRead(A);
lastStateB = digitalRead(B);
}
//while(millis()<10001){//while under 10seconds, loop
//  lcd.print(millis()/1000);
//}

//digitalWrite(IN1, LOW);
//digitalWrite(IN2, LOW);
//digitalWrite(IN3, LOW);
//digitalWrite(IN4, LOW);//stop moving

void loop(){
  //follow_line(64);
  /*if (Bluetooth.available()){ //wait for data received
    Data=Bluetooth.read();
    if(Data=='1'){  
      digitalWrite(IN1,HIGH); //move forward
      digitalWrite(IN2,LOW);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      analogWrite (en,128);
    }
    else if(Data=='2'){
      digitalWrite(IN1,LOW); //move backward
      digitalWrite(IN2,HIGH);
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
      analogWrite (en,128);
    }
    else if(Data=='3'){
      digitalWrite(IN1,HIGH); //turn left
      digitalWrite(IN2,LOW);
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
      analogWrite (en,128);
    }
    else if(Data=='4'){
      digitalWrite(IN1,LOW); //turn right
      digitalWrite(IN2,HIGH);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      analogWrite (en,128);
    }
    else if(Data=='5'){
      analogWrite (en,0);//stop
    }
    else if(Data=='6'){
      tone(buzzer,900);
      delay(500);
      noTone(buzzer);
    }
    }
    else{;}
    delay(100);
  }*/
  digitalWrite(IN1,HIGH); //move forward
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (enl,lspeed);
  analogWrite (enr,rspeed);
  calculate_distance();
  error=counterB-counterA;
  lspeed=(500-(5*error)-(0.2*(error-preverror)))/1000*255;
  rspeed=(500+(5*error)+(0.2*(error-preverror)))/1000*255;
  preverror=error;
  lcd.setCursor(0,0);
  lcd.print(error);
  lcd.setCursor(0,1);
  lcd.print(counterB);
  lcd.setCursor(4,1);
  lcd.print(counterA);
  lcd.clear();

}


  /*if(Serial.available() > 0){ // Checks whether data is comming from the serial port
  state = Serial.read();
  } // Reads the data from the serial port
  if (state == 'A') {
  digitalWrite(IN1,HIGH); //move forward
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,128);
  }else if (state == 'B') {
  digitalWrite(IN1,LOW); //move backward
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite (en,128);
  }else if(state == 'C') { 
  digitalWrite(IN1,HIGH); //turn left
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite (en,128);
  }else if(state == 'D') {
  digitalWrite(IN1,LOW); //turn right
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,128);
  }else if(state == 'E') { 
    analogWrite (en,0);//stop
  }*/

  //calculate_IMU_error();
  /*calculate_angle();
  lcd.setCursor(0,0);
  lcd.print("pitch");
  lcd.setCursor(0,1);
  lcd.print(pitch);*/
  //follow_line(128);
  //delay(100);
//}
  /*digitalWrite(trig, LOW);
   delayMicroseconds(2);
   digitalWrite(trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(trig, LOW);
   duration = pulseIn(echo, HIGH);
   cm = duration*0.0343/2;
   
   if(cm<10){
    digitalWrite(IN1,LOW); //turn right
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite (en,128);
   }
  else {follow_line(128);}
  delay(100);
*/
  /*calculate_angle; 
  if(pitch>20){
    follow_line(tspeed);
    afterramp=1;
  /*}else if(afterramp==1&&pitch<10){
    delay(4000);
    while(yaw<360){
      digitalWrite(IN1,LOW); //turn right
      digitalWrite(IN2,HIGH);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      analogWrite (en,64);
      calculate_angle();
    }
    afterramp=0;
  }else{*/
    //follow_line(128);
  //}
  /*
  calculate_angle();
  lcd.setCursor(0,0);
  lcd.print("roll");
  lcd.setCursor(8,0);
  lcd.print(roll);
  lcd.setCursor(0,1);
  lcd.print("pitch");
  lcd.setCursor(8,1);
  lcd.print(pitch);
  //delay(300);*/

  
  //lcd.setCursor(0,1);
  //lcd.print(counterA);
  //lcd.print("cm");
  
  //Serial.println(yaw);
  //lcd.setCursor(0,1);
  //lcd.print(gyroAngleX); 

void calculate_distance(){
   // Read the current state of CLK
	currentStateA = digitalRead(A);
  currentStateB = digitalRead(B);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateA != lastStateA  && currentStateA == 1){

		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		//if (digitalRead(B) != currentStateA) {
		//	counter --;
		//} else {
			// Encoder is rotating CW so increment
			counterA ++;
    //}
  }
  if (currentStateB != lastStateB  && currentStateB == 1){

		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		//if (digitalRead(B) != currentStateA) {
		//	counter --;
		//} else {
			// Encoder is rotating CW so increment
			counterB ++;
    }
  
  lastStateA = currentStateA;
  lastStateB = currentStateB;
  //average= (counterA + counterB)/2;
}
void follow_line(int speed){
  svr=digitalRead(sr);
  svl=digitalRead(sl);
  if(svl==HIGH && svr==LOW)
  {
  //analogWrite (enr,0); //stop moving
  analogWrite (en,0);
  }

  if(svl==LOW  && svr==LOW)
  {
  digitalWrite(IN1,LOW); //turn right
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,tspeed);
  }
 
  if(svl==HIGH && svr==HIGH)
  { 
  digitalWrite(IN1,HIGH); //turn left
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite (en,tspeed);
  }
  
  if(svl==LOW && svr==HIGH)
  {
  digitalWrite(IN1,HIGH); //move forward
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (en,speed);
  }
}
void calculate_angle(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) -0.83; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) -3.73; // AccErrorY ~(-1.58)
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroX = GyroX - 3.12; // GyroErrorX ~(3.12)
  GyroY = GyroY - 5.96; // GyroErrorY ~(5.96)
  GyroZ = GyroZ - 0.73; // GyroErrorZ ~ (0.73)
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  //roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  //pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  roll=atan2(AccY,AccZ)*180.0/PI;
  pitch=atan2(-AccX,sqrt(AccY*AccY+AccZ*AccZ))*180.0/PI;
}
void calculate_IMU_error(){
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
