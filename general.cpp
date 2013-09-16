#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "SensorPacket.h"

ros::Publisher Moves;
ros::Publisher Close;
ros::Publisher Open;
ros::Publisher cmd_vel;
geometry_msgs::Point Objecto_a_recuperar;
ros::Time init_time, current_time, stop_time, reprise_time;
ros::Duration Duration_left;

std_msgs::Bool Detected;
std_msgs::Bool OK;
bool Obj_detected,Running,Bumper_izquierda,Bumper_derecha;


void Avanzar(float velocidad_x, float velocidad_y){
	Running=true;
	geometry_msgs::Twist msg; 
	msg.linear.x = velocidad_x; // move forward velocidad_x m/s
	msg.linear.y = velocidad_y; // move forward velocidad_y m/s  
	cmd_vel.publish(msg);
	std::cerr << "Avanzando "<< std::endl;
	//return Running;
}

void Stop(void){
	Running=false;
	geometry_msgs::Twist msg;   
	cmd_vel.publish(msg);
	std::cerr << "Para "<< std::endl;
//return Running;
}

void Girar(float velocidad_ang_z, float angulo){
	geometry_msgs::Twist msg; 
	msg.angular.z = velocidad_ang_z;   
	cmd_vel.publish(msg);
	ros::spinOnce();
	std::cerr << "Girando "<< std::endl;
	ros::Duration(2).sleep();
	
}
	
void Recuperar(const geometry_msgs::Point& Centroide){
geometry_msgs::Point Init;
geometry_msgs::Point Largage;
geometry_msgs::Point Point1;
OK.data=true;
	Init.x=1;	
	Init.y=0;
	Init.z=10;

	Largage.x=40;	
	Largage.y=250;
	Largage.z=-30;

	Point1.x=Centroide.x;	
	Point1.y=Centroide.y;
	Point1.z=Centroide.z+50;

	Moves.publish(Point1);
	ros::Duration(3).sleep();
	Moves.publish(Centroide);
	ros::Duration(3).sleep();
	Close.publish(OK);
	ros::Duration(1).sleep();
	Moves.publish(Point1);
  	ros::Duration(3).sleep();
	Moves.publish(Largage);
	ros::Duration(3).sleep();
	Open.publish(OK);
	ros::Duration(1).sleep();
	Moves.publish(Init);	
	ros::Duration(4).sleep();

}


void ControlBrazo(const geometry_msgs::Point& Centroide){ 
 
	if(Centroide.x>-10&&Centroide.x<220&&Centroide.y<170&&Centroide.y>-170&&Centroide.z<0&&Centroide.z>-180){
	Obj_detected=true;	
	Objecto_a_recuperar.x=Centroide.x;
	Objecto_a_recuperar.y=Centroide.y;
	Objecto_a_recuperar.z=Centroide.z;
	std::cerr << "Send object detected "<< ros::Time::now()<<std::endl;
  	}
	else{
	Obj_detected=false;
	}

}

void sensor(const irobot_create_2_1::SensorPacket& msg){ 
  Bumper_izquierda=msg.bumpLeft;
  Bumper_derecha=msg.bumpRight;
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "general");
  ros::NodeHandle nn;


  ros::Subscriber sub = nn.subscribe("Objet", 1, ControlBrazo); 
  ros::Subscriber sub2 = nn.subscribe("sensorPacket", 1, sensor);  
  Moves=nn.advertise<geometry_msgs::Point > ("Moves", 1);
  Close=nn.advertise<std_msgs::Bool > ("Close", 1);
  Open=nn.advertise<std_msgs::Bool > ("Open", 1);
  cmd_vel=nn.advertise<geometry_msgs::Twist > ("cmd_vel", 1);

  Running=false;
  Obj_detected=false;
  Bumper_izquierda=false;
  Bumper_derecha=false;

  std::cerr << "Beginin "<< std::endl;
  std::cerr << "Ezperando que se apreta un bumper para arrancar!! "<< std::endl;
  ros::spinOnce();
  ros::Duration(2).sleep();
  while (nn.ok()) {
  ros::spinOnce();
    if(Bumper_izquierda||Bumper_derecha){
      break;
    }
  ros::Duration(0.1).sleep();
  }
 ros::Duration(1).sleep();
 ros::spinOnce();
  while (nn.ok()) {
	Avanzar(0.03, 0);	
	Duration_left=ros::Duration(8.0);
	init_time = ros::Time::now();
	while(current_time-init_time <Duration_left&&nn.ok()){
	ros::spinOnce();
	  if(Bumper_izquierda||Bumper_derecha){
      		break;
    	  }
  	  else if(Obj_detected){
	    if(Running){	      
	      Stop();
	      stop_time = ros::Time::now();
	      Duration_left=Duration_left-(stop_time-init_time);
 	      std::cerr << "Object_detected, time left "<<Duration_left<< std::endl;
	      std::cerr << "Posicion a del objeto"	<<Objecto_a_recuperar.x<<' '
						        <<Objecto_a_recuperar.y<<' '
						        <<Objecto_a_recuperar.z<< std::endl;	
	    }
	    else{  
	    ros::Duration(2).sleep();
	    ros::spinOnce();
	    std::cerr << "Posicion del objeto"	<<Objecto_a_recuperar.x<<' '
						<<Objecto_a_recuperar.y<<' '
						<<Objecto_a_recuperar.z<< std::endl;
            Recuperar(Objecto_a_recuperar);  
	    }
	   }
	   else{
	     if(Running==false){
		reprise_time= ros::Time::now();
	        init_time=reprise_time;
	        Avanzar(0.03, 0);	      
	     }
	   current_time=ros::Time::now();
 	   }
	 }
	Stop(); 
	ros::Duration(1).sleep();
	Girar(-0.3, 3);
	Stop(); 
	ros::Duration(1).sleep();
	
  }

  return 0;
}

