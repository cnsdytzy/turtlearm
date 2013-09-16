

#include <Servo.h>
#include <math.h> 
#include <ros.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

geometry_msgs::Point Angulos;
std_msgs::Bool libre;
ros::Publisher pub_Angulo("Angulos", &Angulos);
ros::NodeHandle  na;

//Parametros propios del robot
const float L0=70; // longeur entre le sol et l'axe du servo bras
const float L1=110; // longeur entre l'axe du servo bras et du servo coude
const float L2=110; // longeur entre du servo coude et l'axe du servo outil
const float L3=105; // longeur entre l'axe du servo outil et la pointe de l'outil

int a0;
int a1=10;
int a2=125;
int a3=133;

boolean Ouvert;

Servo servo0;  //Hitec HS 311
Servo servo1;  //Hitec HS 311
Servo servo2;  //Hitec HS 311
Servo servo3;  //MiniServo
Servo servo4;

float angleservo0;
float angleservo1; 
float angleservo2;
float angleservo3;
int anglePince=-90;
int Speed=50;                   
int x, y, z;


void Moves_cb( const geometry_msgs::Point& cmd_msg){   
    Moves(cmd_msg.x,cmd_msg.y,cmd_msg.z);   
}

void Open_cb( const std_msgs::Bool& Open_msg){
   if(Open_msg.data==true){
    Open();
   }
}

void Close_cb( const std_msgs::Bool& Close_msg){
   if(Close_msg.data==true){
    Close();
   }
}

void Speed_cb( const std_msgs::Int16& cmd_msg){
  Speed=cmd_msg.data;
}


ros::Subscriber<geometry_msgs::Point> sub2("Moves", Moves_cb);
ros::Subscriber<std_msgs::Int16> sub("Speed", Speed_cb);
ros::Subscriber<std_msgs::Bool> sub3("Open", Open_cb);
ros::Subscriber<std_msgs::Bool> sub4("Close", Close_cb);



void setup(){
  //Serial.begin(115200);
  libre.data=false;
  na.initNode();
  na.advertise(pub_Angulo);
  na.subscribe(sub);
  na.subscribe(sub2);
  na.subscribe(sub3);
  na.subscribe(sub4);
  
  servo0.attach(8); 
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  servo4.attach(12);
 
 Init();
 delay(1000);


}


void loop(){ //bucle infinita
pub_Angulo.publish(&Angulos);
na.spinOnce();
delay(1);
 
}

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Funcion de paso angulo a impulsion/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
int AngToPulsHS311(float valeur_Ang) {
        float impuls=0;
        impuls=9.397*valeur_Ang+554.4;
        return impuls;  
}

int AngToPulsHS645(float valeur_Ang) {
        float impuls=0;
        impuls=10.32*valeur_Ang+551.5;
        return impuls;  
}

int AngToPulsHbKing(float valeur_Ang) {
        float impuls=0;
        impuls=10.44*valeur_Ang+455.9;
        return impuls;  
}


///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Funcion de paso impulsion a angulo/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
float PulsToAngHS311(float valeur_Puls) {
        float Angle=0;
        Angle=0.106*valeur_Puls-58.94;
        return Angle;  
}


//////////ouverture fermerure pince/////

boolean Open(){
  servo4.writeMicroseconds(AngToPulsHS311(150));
  //Ouvert=true;
}


boolean Close(){
  servo4.writeMicroseconds(AngToPulsHS311(32));
  //Ouvert=false;
}


////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////   Programme calcule des angles //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
float Calculate(float Px, float Py, float Pz){ 
  
//boolean first=true;
//anglePince=-60;
int last_anglePince=-120;
float ra;
ra=sqrt(sq(Px)+sq(Py));

float angle0=-180,angle1=-180,angle2=-180,angle3=-180;

 angle0=180/PI*atan2(Px,Py);
 angle1=(180/PI*atan2(Pz-L3*sin(PI/180*anglePince),ra-L3*cos(PI/180*anglePince))+180/PI*acos((pow(ra-L3*cos(PI/180*anglePince),2)+pow(Pz-L3*sin(PI/180*anglePince),2)+pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(ra-L3*cos(PI/180*anglePince),2)+pow(Pz-L3*sin(PI/180*anglePince),2))))); 
 angle2=(180/PI*atan2(Pz-L3*sin(PI/180*anglePince)-L1*sin(PI/180*angle1),ra-L3*cos(PI/180*anglePince)-L1*cos(PI/180*angle1))-angle1);
 angle3=(anglePince-angle2-angle1);
 
//angle1<-a1||angle1>180-a1||
//last_anglePince<90&&
while (isnan(angle2)||angle1<-a1||angle1>190-a1){
 last_anglePince++; 
 angle1=(180/PI*atan2(Pz-L3*sin(PI/180*last_anglePince),ra-L3*cos(PI/180*last_anglePince))+180/PI*acos((pow(ra-L3*cos(PI/180*last_anglePince),2)+pow(Pz-L3*sin(PI/180*last_anglePince),2)+pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(ra-L3*cos(PI/180*last_anglePince),2)+pow(Pz-L3*sin(PI/180*last_anglePince),2))))); 
 angle2=(180/PI*atan2(Pz-L3*sin(PI/180*last_anglePince)-L1*sin(PI/180*angle1),ra-L3*cos(PI/180*last_anglePince)-L1*cos(PI/180*angle1))-angle1);
 angle3=(last_anglePince-angle2-angle1); 
 //anglePince=last_anglePince;
 }

x=Px;
y=Py;
z=Pz;


angleservo0=angle0;
angleservo1=angle1;
angleservo2=angle2;
angleservo3=angle3;

Angulos.x=angleservo1;
Angulos.y=angleservo2;
Angulos.z=angleservo3;

return angleservo0,angleservo1,angleservo2,angleservo3;

delay(50);
} 




////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////Programme position initiale nono//////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

int Pose(float Px, float Py, float Pz){ 
Calculate(Px,Py,Pz);

Angulaire(angleservo0,angleservo1,angleservo2,angleservo3);

delay(50);
} 


///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Funcion de initializacion /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
int Init (){
Pose(1,0,10);
Open();
delay(100);
}

/////////////////////////////fin du programme d'initialization///////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Programme Move déplacement angulaire des servo////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

 
int Move(float Px, float Py, float Pz){// valeur de speed entre 10 et 100
int b0,b1,b2,b3;
b0=angleservo0;
b1=angleservo1;
b2=angleservo2;
b3=angleservo3;

Calculate(Px,Py,Pz);

int n=(110-Speed)/5;

 for (int i=1;i<=n;i++){
    Angulaire(((angleservo0-b0)*i+b0*n)/n,((angleservo1-b1)*i+b1*n)/n,((angleservo2-b2)*i+b2*n)/n,((angleservo3-b3)*i+b3*n)/n);
  
  delay(50);
  }

}
///////////////////////////Fin de la fonction Move//////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////Fonction deplacement rectiligne MOVES////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
float Moves (float Px, float Py, float Pz){
  
int xa,ya,za;
xa=x;
ya=y;
za=z;

int n=(110-Speed)/5;

  for (int i=1;i<=n;i++){
    Pose(((Px-xa)*i+n*xa)/n,((Py-ya)*i+n*ya)/n,((Pz-za)*i+n*za)/n);
    
  
  delay(50);
  }
   
}


//////////////////////////////fin de la fonction Moves///////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Programme de mise en position angulaire des servo/////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
int Angulaire(int t0,int t1, int t2,int t3){

  
  servo1.writeMicroseconds(AngToPulsHbKing(180-(t1+a1)));
  servo2.writeMicroseconds(AngToPulsHS645(t2+a2));
  servo3.writeMicroseconds(AngToPulsHS311(t3+a3)); 
  servo0.writeMicroseconds(AngToPulsHS311(t0));
  delay(10);
}
//////////////////////////////fin de la fonction Angulaire///////////////////////////////////




