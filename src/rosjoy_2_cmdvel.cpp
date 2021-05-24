#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"

#define gear_ratio 26
#define wheel_radius 0.08
#define distance 0.4
#define rpm_limit 2500
#define countNum 10

#define RPM_MODE 1
#define CMD_VEL_MODE 2



static double fb_vel=0.0;
static double rl_vel=0.0;

static int r_rpm = 0;
static int l_rpm = 0;

static const int step=4; //stop상황에서의 count step

static int stop_count=0;
static bool stop_or_not = false;
static uint8_t CAN_mode_num = 0;   //모드변경. 각각의 CAN 배열에 해당하는 번호를 의미
static int butt_count[10]={0,0,0,0,0,0,0,0,0,0};

bool rpm_limit_check(double linear_vel,double angular_vel);
void butt_count_reset(void);
//void print_status(int a);
static char PWR_ON[]="PWR_ON";
static char PWR_OFF[]="PWR_OFF";
static char SM0505[]="SM0505";
static char MD_gogo[]="MD_gogo";
static char TQ_OFF[]="TQ_OFF";

static char*mode=NULL;

static int operating_mode = 2;    //cmd_vel 모드로 시작 

/*========================================================================================================*/
/*=========================================== JOY Callback함수 ============================================*/
void JOYCallback(const sensor_msgs::Joy::ConstPtr& joymsg)
{
  /*double fb=0.0,rl=0.0;

  if(joymsg->axes[1]>0){fb= 0.001;} //전진 :+ 0.0005
  if(joymsg->axes[1]<0){fb=-0.001;} //후진 :-
  if(joymsg->axes[3]>0){rl= 0.002;} //좌회전:+ 0.001
  if(joymsg->axes[3]<0){rl=-0.002;} //우회전:-*/

  if((joymsg->buttons[4]!=0) && (joymsg->buttons[5]!=0)){    //오른쪽 + 왼쪽 검지버튼 = 정지
    butt_count[5]++;  if(butt_count[5]>countNum){stop_or_not=true;  butt_count_reset();}
  }

  if(joymsg->buttons[7]!=0){                                 //왼쪽 트리거 = angular.z를 0으로 
    butt_count[6]++; if(butt_count[6]>5){rl_vel=0;  butt_count_reset();}
  }

  if((joymsg->buttons[6]!=0) && (fb_vel<0.3&&fb_vel>-0.3)){  //오른쪽 트리거 = linear.x를 0으로 
    butt_count[8]++; 
    if(butt_count[8]>15){
      fb_vel=0.5*fb_vel;
      if(butt_count[8]>30){
        fb_vel=0;butt_count_reset();
      }
    }
  }


  if(joymsg->buttons[3]!=0){                         //(네모)operating mode = [rpm_mode]
    butt_count[3]++;
    if(butt_count[3]>countNum){
      CAN_mode_num = 4;
      operating_mode = RPM_MODE;
      butt_count_reset();
    }
  }

  if(joymsg->buttons[0]!=0&&joymsg->buttons[1]==0){  //(엑스)nothing, CAN통신 안보냄
    butt_count[0]++;
    if(butt_count[0]>countNum){
      CAN_mode_num=0; 
      mode=PWR_OFF;
      butt_count_reset();
    }
  }

  if(joymsg->buttons[2]!=0){                         //(세모)SM0505[velocity control mode on]
    butt_count[2]++;
    if(butt_count[2]>countNum){
      CAN_mode_num=3; 
      mode=SM0505;
      butt_count_reset();
    }
  }

  if(joymsg->buttons[1]!=0&&joymsg->buttons[0]==0){  //(동그라미)operating mode = [cmd_vel_mode]
    butt_count[1]++;
    if(butt_count[1]>countNum){
      CAN_mode_num=1; 
      operating_mode=CMD_VEL_MODE;
      butt_count_reset();
    }
  }

  if(joymsg->buttons[0]!=0&&joymsg->buttons[1]!=0){  //(동그라미+엑스) TQ_OFF, 자연정지
    butt_count[7]++;
    if(butt_count[7]>countNum){
      CAN_mode_num=7; 
      mode=TQ_OFF;
      butt_count_reset();
    }
  }


  /****오른쪽 버튼 [3]:네모, [2]:세모 ,[1]:O, [0]:X ********/
  if(operating_mode == CMD_VEL_MODE){
    
    double fb=0.0,rl=0.0;

    if(joymsg->axes[1]>0){fb= 0.001;} //전진 :+ 0.0005
    if(joymsg->axes[1]<0){fb=-0.001;} //후진 :-
    if(joymsg->axes[3]>0){rl= 0.002;} //좌회전:+ 0.001
    if(joymsg->axes[3]<0){rl=-0.002;} //우회전:-

  if(rpm_limit_check(fb_vel+fb,rl_vel+rl)&&!stop_or_not){fb_vel+=fb; rl_vel+=rl;}


  if(!stop_or_not){ROS_INFO("\nMODE : cmd_vel_mode \n<cmd_vel>\n[linearX : %lf]\n[angularZ : %lf] ",fb_vel,rl_vel);}


  if(stop_or_not){stop_count++;}
  if(stop_count== step){fb_vel=fb_vel*0.9; rl_vel = rl_vel*0.8;ROS_INFO("STOPING..");}    //int step을 줄이면 더 빨리 멈춤
  if(stop_count==step*2){fb_vel=fb_vel*0.9; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count==step*3){fb_vel=fb_vel*0.9; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count==step*4){fb_vel=fb_vel*0.9; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count==step*5){fb_vel=fb_vel*0.9; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count==step*6){fb_vel=fb_vel*0.8; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count==step*7){fb_vel=fb_vel*0.8; rl_vel = rl_vel*0.5;ROS_INFO("STOPING..");}
  if(stop_count==step*8){fb_vel=fb_vel*0.7; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count==step*9){fb_vel=fb_vel*0.7; rl_vel = rl_vel*0.7;ROS_INFO("STOPING..");}
  if(stop_count >step*10){fb_vel=0.0; rl_vel=0.0; stop_or_not=false;stop_count=0;butt_count_reset();ROS_INFO("STOP_END");}

  }
  else if(operating_mode == RPM_MODE){

    int r_step=0, l_step=0;

    if(joymsg->axes[1]>0){l_step= 1;} //왼쪽 전진 :+ 0.0005
    if(joymsg->axes[1]<0){l_step=-1;} //후진 :-
    if(joymsg->axes[2]>0){r_step= 1;} //오른쪽 전진 :+ 0.001
    if(joymsg->axes[2]<0){r_step=-1;} //후진 :-
    if(!((r_rpm+r_step > rpm_limit) && (r_rpm+r_step < -1*rpm_limit) && (l_rpm+l_step > rpm_limit) && (l_rpm+l_step < -1*rpm_limit))){
      r_rpm+=r_step;
      l_rpm+=l_step;
    }
    if(!stop_or_not){ROS_INFO("\nMODE : rpm_mode \n<RPM>\n[LEFT : %d]\n[RIGHT : %d] ",l_rpm,r_rpm);}

    if(stop_or_not){stop_count++;}
    if(stop_count== step){r_rpm=r_rpm*0.9; l_rpm = l_rpm*0.8;ROS_INFO("STOPING..");}    //int step을 줄이면 더 빨리 멈춤
    if(stop_count==step*2){r_rpm=r_rpm*0.9; l_rpm = l_rpm*0.9;ROS_INFO("STOPING..");}
    if(stop_count==step*3){r_rpm=r_rpm*0.9; l_rpm = l_rpm*0.9;ROS_INFO("STOPING..");}
    if(stop_count==step*4){r_rpm=r_rpm*0.9; l_rpm = l_rpm*0.9;ROS_INFO("STOPING..");}
    if(stop_count==step*5){r_rpm=r_rpm*0.9; l_rpm = l_rpm*0.9;ROS_INFO("STOPING..");}
    if(stop_count==step*6){r_rpm=r_rpm*0.8; l_rpm = l_rpm*0.8;ROS_INFO("STOPING..");}
    if(stop_count==step*7){r_rpm=r_rpm*0.8; l_rpm = l_rpm*0.8;ROS_INFO("STOPING..");}
    if(stop_count==step*8){r_rpm=r_rpm*0.7; l_rpm = l_rpm*0.7;ROS_INFO("STOPING..");}
    if(stop_count==step*9){fb_vel=r_rpm*0.7; l_rpm = l_rpm*0.7;ROS_INFO("STOPING..");}
    if(stop_count >step*10){r_rpm=0.0; l_rpm=0.0; stop_or_not=false;stop_count=0;butt_count_reset();ROS_INFO("STOP_END");}

  }
}
/*=========================================== JOY Callback함수 ============================================*/
/*========================================================================================================*/




/*=============================================================================================================*/
/*========================================== RPM_Limit_check 함수 ==============================================*/
bool rpm_limit_check(double linear_vel,double angular_vel)
{
  int R_RPM = (gear_ratio*30.0*((2*linear_vel) + (distance*angular_vel))/(2*wheel_radius*3.141593));
  int L_RPM = (gear_ratio*30.0*((2*linear_vel) - (distance*angular_vel))/(2*wheel_radius*3.141593));
  if(R_RPM>rpm_limit||L_RPM>rpm_limit||R_RPM<-1*rpm_limit||L_RPM<-1*rpm_limit){return false;}  //rpm_linit=2500
  else {return true;}
}
/*========================================== RPM_Limit_check 함수 ==============================================*/
/*=============================================================================================================*/



/*==========================================*/
/*=========== Butt_count_reset =============*/
void butt_count_reset(void)
{
  int i;
  for(i=0;i<10;i++){butt_count[i]=0;}
}
/*=========== Butt_count_reset =============*/
/********************************************/





/*===========================================================================*/
/*============================= main 함수 ====================================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosjoy_2_cmdvel");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher mode_pub = nh.advertise<std_msgs::UInt8>("/mode", 1000);
  ros::Subscriber joy_sub = nh.subscribe("joy", 100, JOYCallback);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel_msg;
    std_msgs::UInt8 mode_msg;

    cmd_vel_msg.linear.x = fb_vel;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = rl_vel;

    mode_msg.data = operating_mode;



    cmd_vel_pub.publish(cmd_vel_msg);
    mode_pub.publish(mode_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
/*===========================================================================*/
/*===========================================================================*/
