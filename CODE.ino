/*************************************************************************/
/**Kinematics Analysis and Modeling of 6 Degree of Freedom Robotic Arm  **/
/*************************************************************************/
#include <math.h>
#include<Servo.h>

float px,py,pz;
float ox,oy,oz;
float nx,ny,nz;
float ax,ay,az;
float d1,d2,d3,d4,d5,d6;
float a4 = 90;
float a3 = 90;
float D5 = 30;
float D1 = 45;

int x,y;
float d;

int L1=7;
int L2=5;
char inSerial[15];
int a;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
void setup() {
  myservo1.attach(4);
  myservo2.attach(5);
  myservo3.attach(6);
  myservo4.attach(7);
  myservo5.attach(8);
  myservo6.attach(9);
  Serial.begin(9600);
  Serial.println("Enter 1 for FORWARD KINEMATIC OR 2 INVERSE KINEMATC : ");
}


void loop() {
  if (Serial.available() > 0) {    
    a=Serial.parseInt();      
    if (a == 1) {
      Serial.println("--------FORWARD KINEMATIC---------");
      Serial.print("Enter the first angle : ");
      while (1){
          d1 = Serial.parseInt(); 
          if (d1>-1){
              Serial.println(d1);
              break;}
              }
      Serial.print("Enter the second angle : ");
      while (1){
          d2 = Serial.parseInt();
          if (d2>-1){
              Serial.println(d2);
              break;}
              }
      Serial.print("Enter the 3 angle : ");
      while (1){
          d3 = Serial.parseInt();
          if (d3>-1){
              Serial.println(d3);
              break;}
              }
      Serial.print("Enter the 4 angle : ");
      while (1){
          d4 = Serial.parseInt();
          if (d4>-1){
              Serial.println(d4);
              break;}
              }
      Serial.print("Enter the 5 angle : ");
      while (1){
          d5 = Serial.parseInt();
          if (d5>-1){
              Serial.println(d5);
              break;}
              }
       Serial.print("Enter the 6 angle : ");
      while (1){
          d6 = Serial.parseInt();
          if (d6>-1){
              Serial.println(d6);
              break;}
              }
      float C1 = cos(d1);
      float C2 = cos(d2);
      float C3 = cos(d3);
      float C4 = cos(d4);
      float C5 = cos(d5);
      float C6 = cos(d6);
      float S1 = sin(d1);
      float S2 = sin(d2);
      float S3 = sin(d3);
      float S4 = sin(d4);
      float S5 = sin(d5);
      float S6 = sin(d6);
      float C23 = cos(d2+d3);
      float S23 = sin(d2+d3);
      float C345 = sin(d5+d3+d4);
      float S345 = sin(d5+d3+d4);
      float C12 = C1*C2 - S1*S2;
      float S12 = C1*S2 - S1*C2;
      //float C234 = C2*(C3*C4-S3*S4)-S2*(C4*S3+C3*S4);
      //float S234 = S2*(C3*C4 - S3*S4)+C2*(S3*C4+C3*S4);
      nx = C6 * C12 * C345 - S6 * S12 ;
      ny = C6 * C12 * C345 + C12 * S6 ;
      nz = C6 * S345 ;
      ox = -C12*S6*C345 - S12*C6;
      oy = -S6*S12*C345 + C12*C6;
      oz = -S6 * S345;
      ax = -C12 * S345;
      ay = -S12 * S345;
      az = C345;
      px = a4 * C12 * C3 * C4 - a4 * C12 * S3 * S4 + S12 * D5 + a3 * C12 * C3 ;
      py = a4 * S12 * C3 * C4 - a4 * S12 * S3 * S4 - C12 * D5 + a3 * S12 * C3 ;
      pz = a4 * S3 * C4 + a4 * C3 * S4 + a3 * S3 + D1 ;
      //x = L1 * cos(d1) + L2 * cos(d1+d2);
      //y = L1 * sin(d1) + L2 * sin(d1+d1);
      Serial.println("The points :");
      Serial.print("nx = ");
      Serial.print(nx);
      Serial.print(" ox = ");
      Serial.print(ox);
      Serial.print(" ax = ");
      Serial.print(ax);
      Serial.print(" px = ");
      Serial.println(px);

      Serial.print("ny = ");
      Serial.print(ny);
      Serial.print(" oy = ");
      Serial.print(oy);
      Serial.print(" ay = ");
      Serial.print(ay);
      Serial.print(" py = ");
      Serial.println(py);

      Serial.print("nz = ");
      Serial.print(nz);
      Serial.print(" oz = ");
      Serial.print(oz);
      Serial.print(" az = ");
      Serial.print(az);
      Serial.print(" pz = ");
      Serial.println(pz);

      myservo1.write(d1);
      myservo2.write(d2);
      myservo3.write(d3);
      myservo4.write(d4);
      myservo5.write(d5);
      myservo6.write(d6);
      Serial.println("Enter 1 for FORWARD KINEMATIC OR 2 INVERSE KINEMATC : ");
    }
    else if (a == 2){
      Serial.println("--------INVERSE KINEMATIC---------");
      Serial.print("Enter nx : ");
      while (1){
          nx = Serial.parseInt();
          if (nx>-1){
              Serial.println(nx);
              break;}}
      Serial.print("Enter ny : ");
      while (1){
          ny = Serial.parseInt();
          if (ny>-1){
              Serial.println(ny);
              break;}}
      Serial.print("Enter nz : ");
      while (1){
          nz = Serial.parseInt();
          if (nz>1){
              Serial.println(nz);
              break;}}
      Serial.print("Enter ox : ");
      while (1){
          ox = Serial.parseInt();
          if (ox>-1){
              Serial.println(ox);
              break;}}
      Serial.print("Enter oy : ");
      while (1){
          oy = Serial.parseInt();
          if (oy>-1){
              Serial.println(oy);
              break;}}
      Serial.print("Enter oz : ");
      while (1){
          oz = Serial.parseInt();
          if (oz>1){
              Serial.println(oz);
              break;}}
      Serial.print("Enter ax : ");
      while (1){
          ax = Serial.parseInt();
          if (ax>-1){
              Serial.println(ax);
              break;}}
      Serial.print("Enter ay : ");
      while (1){
          ay = Serial.parseInt();
          if (ay>-1){
              Serial.println(ay);
              break;}}
      Serial.print("Enter az : ");
      while (1){
          az = Serial.parseInt();
          if (az>1){
              Serial.println(az);
              break;}}
      Serial.print("Enter px : ");
      while (1){
          px = Serial.parseInt();
          if (px>-1){
              Serial.println(px);
              break;}}
      Serial.print("Enter py : ");
      while (1){
          py = Serial.parseInt();
          if (py>-1){
              Serial.println(py);
              break;}}
      Serial.print("Enter pz : ");
      while (1){
          pz = Serial.parseInt();
          if (pz>1){
              Serial.println(pz);
              break;}}
      float S3 = sin(d3);
      float S34 = (a3*S3-pz+D1)/a4;
      float d34 = atan2(S34,sqrt(1-S34*S34));
      d4 = d34 -d3;
      d2 = atan2(px,-py)+atan2(sqrt(px*px+py*py-(D1+D5)*(D1+D5)),D1 + D5);
      float d12 = (ax/ay)*atan2(ay,ax);
      d1 = d12-d2;
      float C12 = cos(d12);
      float S12 = sin(d12);
      d6 = atan2(sqrt(1-(C12*oy - S12*ox)*(C12*oy - S12*ox)),(C12 * oy - S12 * ox)); 
      float S2 = sin(d2);
      float d23 = atan2(px,-py)+atan2(sqrt(px*px+py*py-(a3*S2 + D1+D5)*(a3*S2 + D1+D5)),(a3*S2 + D1+D5));
      d3 = d23 - d2;
      float d345 = (1/az)*atan2(sqrt(1-az*az),az);
      d5 = d345 - d3 - d4;
      
      Serial.print("D1 = ");
      Serial.println(d1);
      Serial.print("D2 = ");
      Serial.println(d2);
      Serial.print("D3 = ");
      Serial.println(d3);
      Serial.print("D4 = ");
      Serial.println(d4);
      
      Serial.print("D5 = ");
      Serial.println(d5);
      Serial.print("D6 = ");
      Serial.println(d6);
      myservo1.write(d1);
      myservo2.write(d2);
      myservo3.write(d3);
      myservo4.write(d4);
      myservo5.write(d5);
      myservo6.write(d6);
      Serial.println("Enter 1 for FORWARD KINEMATIC OR 2 INVERSE KINEMATC : ");
      }
    else   {
      Serial.println("Not a number.");
    }
  } // end: if (Serial.available() > 0)

}
