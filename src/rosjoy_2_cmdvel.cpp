#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include <stdio.h>

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif


#define gear_ratio 30
#define wheel_radius 0.08 //m
#define distance 0.4      //m
#define rpm_limit 2000
#define countNum 10


//modes
#define RPM_MODE 1
#define CMD_VEL_MODE 2
#define TORQUE_OFF 0
#define DYNAMIC_CMD_VEL_MODE 3
#define ONE_HAND 4
#define JoyNotUse 5
#define Back_Hand_Control_ 6

/******* 조이스틱 속도 스케일링**********/
#define MAX_LINEAR_VEL 0.75//1.25   //dymanic cmd_vel 모드에서 linear x 속도의 P gain에 해당
#define MAX_ANGULAR_VEL 1.5 //2  //dynamic cmd_vel 모드에서 angular z 속도의 P gain에 해당


/********조이스틱 Axes & Butten*******/
/*#define*/ int AXES_LEFT_UPDOWN = 1;
//#define AXES_LEFT_RL -1

//#define AXES_RIGHT_UPDOWN -1
/*#define*/ int AXES_RIGHT_RL = 3;

/*#define*/ int BTTN_TRIANGLE = 2;
/*#define*/ int BTTN_SQUARE = 3;
/*#define*/ int BTTN_X = 0;
/*#define*/ int BTTN_O = 1;

/*#define*/ int BTTN_AXES_UPDOWN = 7;
/*#define*/ int BTTN_AXES_LEFTRIGHT = 6;









static double fb_vel=0.0;
static double rl_vel=0.0;

static int r_rpm = 0;
static int l_rpm = 0;

static const int step=4; //stop상황에서의 count step

static int stop_count=0;
static bool stop_or_not = false;
static uint8_t CAN_mode_num = 0;   //모드변경. 각각의 CAN 배열에 해당하는 번호를 의미
static int butt_count[11]={0,0,0,0,0,0,0,0,0,0,0};

bool rpm_limit_check(double linear_vel,double angular_vel);
void butt_count_reset(void);
void set_vels_zero(void);

//void print_status(int a);
static char PWR_ON[]="PWR_ON";
static char PWR_OFF[]="PWR_OFF";
static char SM0505[]="SM0505";
static char MD_gogo[]="MD_gogo";
static char TQ_OFF[]="TQ_OFF";

static char*mode=NULL;

static int operating_mode = JoyNotUse;    //JoyNotUse(5) 모드로 시작

int bttn_set_num=-1;
int axes_set_num=-1;

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}



int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


void JOY_setting_Callback(const sensor_msgs::Joy::ConstPtr& joymsg){

 for(int i = 0;i<8;i++){
   if(joymsg->buttons[i]!=0){
     bttn_set_num = i;
   }
 }
 for(int j=0;j<7;j++){
   if(joymsg->axes[j]>0.5 || joymsg->axes[j]<-0.5){
    axes_set_num = j;
   }
 }
}


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


  if(joymsg->buttons[BTTN_SQUARE]!=0 && joymsg->buttons[2]==0){                         //(네모)operating mode = [rpm_mode]
    butt_count[0]++;
    if(butt_count[0]>countNum){
      CAN_mode_num = 4;
      operating_mode = RPM_MODE;
      butt_count_reset();
      set_vels_zero();
    }
  }

  if(joymsg->buttons[BTTN_X]!=0&&joymsg->buttons[BTTN_O]==0){  //(엑스)TQ_OFF
    butt_count[1]++;
    if(butt_count[1]>countNum){
      CAN_mode_num=0;
      operating_mode = TORQUE_OFF;
      butt_count_reset();
      set_vels_zero();
    }
  }

  if(joymsg->buttons[BTTN_TRIANGLE]!=0){                         //(세모) operating mode = [dynamic control mode(game mode)]
    butt_count[3]++;
    if(butt_count[3]>countNum){
      CAN_mode_num=3;
      operating_mode = DYNAMIC_CMD_VEL_MODE;

      butt_count_reset();
      set_vels_zero();
    }
  }

  if(joymsg->buttons[BTTN_O]!=0&&joymsg->buttons[BTTN_SQUARE]==0){  //(동그라미)operating mode = [cmd_vel_mode]
    butt_count[2]++;
    if(butt_count[2]>countNum){
      CAN_mode_num=1;
      operating_mode=CMD_VEL_MODE;
      butt_count_reset();
      set_vels_zero();
    }
  }

  if(joymsg->buttons[BTTN_X]!=0&&joymsg->buttons[BTTN_O]!=0){  //(동그라미+엑스) TQ_OFF, 자연정지
    butt_count[7]++;
    if(butt_count[7]>countNum){
      CAN_mode_num=7;
      mode=TQ_OFF;
      butt_count_reset();
      set_vels_zero();

    }
  }
  if(joymsg->buttons[BTTN_O]!=0&&joymsg->buttons[BTTN_SQUARE]!=0){  //(동그라미+네모)operating mode = [cmd_vel_mode]
    butt_count[8]++;
    if(butt_count[8]>countNum){
      CAN_mode_num=8;
      operating_mode=ONE_HAND;
      butt_count_reset();
      set_vels_zero();

    }
  }

  if(joymsg->axes[BTTN_AXES_LEFTRIGHT]==1){  //(왼쪽 화살표 버튼 [<<])operating mode = [joy_not_use]
    butt_count[9]++;
    if(butt_count[9]>countNum){
      CAN_mode_num=9;
      operating_mode=JoyNotUse;  //use joystick to control Drok_arm
      butt_count_reset();
      set_vels_zero();
    }
  }

  if(joymsg->axes[BTTN_AXES_LEFTRIGHT]== -1){  //(dhfms쪽 화살표 버튼 [>>])operating mode = [back_hand_control]
    butt_count[10]++;
    if(butt_count[10]>countNum){
      CAN_mode_num=10;
      operating_mode=Back_Hand_Control_;  //use joystick to control Back_hand
      butt_count_reset();
      set_vels_zero();
    }
  }


  /****오른쪽 버튼 [0]:네모, [3]:세모 ,[2]:O, [1]:X ********/
  if(operating_mode == CMD_VEL_MODE){

    double fb=0.0,rl=0.0;

    if(joymsg->axes[AXES_LEFT_UPDOWN]>0){fb= 0.001;} //전진 :+ 0.0005
    if(joymsg->axes[AXES_LEFT_UPDOWN]<0){fb=-0.001;} //후진 :-
    if(joymsg->axes[AXES_RIGHT_RL]>0){rl= 0.002;} //좌회전:+ 0.001
    if(joymsg->axes[AXES_RIGHT_RL]<0){rl=-0.002;} //우회전:-

  if(rpm_limit_check(fb_vel+fb,rl_vel+rl)&&!stop_or_not){fb_vel+=fb; rl_vel+=rl;}


  if(!stop_or_not){ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : cmd_vel_mode \n\n<cmd_vel>\n[linearX : %lf]\n[angularZ : %lf] ",fb_vel,rl_vel);}


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
    if(joymsg->axes[5]>0){r_step= 1;} //오른쪽 전진 :+ 0.001
    if(joymsg->axes[5]<0){r_step=-1;} //후진 :-
    if(!((r_rpm+r_step > rpm_limit) && (r_rpm+r_step < -1*rpm_limit) && (l_rpm+l_step > rpm_limit) && (l_rpm+l_step < -1*rpm_limit))){
      r_rpm+=r_step;
      l_rpm+=l_step;
    }
    if(!stop_or_not){ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : rpm_mode \n<RPM>\n[LEFT : %d]\n[RIGHT : %d] ",l_rpm,r_rpm);}

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
  else if(operating_mode == DYNAMIC_CMD_VEL_MODE){

    fb_vel = MAX_LINEAR_VEL*(joymsg->axes[AXES_LEFT_UPDOWN]);
    rl_vel = MAX_ANGULAR_VEL*(joymsg->axes[AXES_RIGHT_RL]);
    ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : DYNAMIC_CMD_VEL_MODE \n\n<dynamic_cmd_vel>\n[linearX : %lf]\n[angularZ : %lf]",fb_vel,rl_vel);

  }
  else if(operating_mode == ONE_HAND){

    fb_vel = MAX_LINEAR_VEL*(joymsg->axes[1]);
    rl_vel = MAX_ANGULAR_VEL*(joymsg->axes[0]);
    ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : ONE_HAND_MODE \n\n<One_Hand_cmd_vel>\n[linearX : %lf]\n[angularZ : %lf]",fb_vel,rl_vel);

  }
  else if(operating_mode == TORQUE_OFF){

    ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : TORQUE_OFF \n");
  }
  else if(operating_mode == JoyNotUse){

    ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : JoyNotUse \n");
  }

  else if(operating_mode == Back_Hand_Control_){

    ROS_INFO("\n======================\n   (DYNAMIC_CMD_VEL)\n(RPM mode)(CMD_VEL mode)\n      (TQ_OFF)\n======================\nMODE : Back_Hand_Control_ \n");
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
  if(R_RPM>rpm_limit||L_RPM>rpm_limit||R_RPM<-1*rpm_limit||L_RPM<-1*rpm_limit){return false; ROS_WARN("rpm_limit!!");}  //rpm_linit=2500
  else {return true;}
}
/*========================================== RPM_Limit_check 함수 ==============================================*/
/*=============================================================================================================*/



/*==========================================*/
/*=========== Butt_count_reset =============*/
void butt_count_reset(void)
{
  int i;
  for(i=0;i<(sizeof(butt_count)/sizeof(int));i++){butt_count[i]=0;}
}
/*=========== Butt_count_reset =============*/
/********************************************/


/*===========================================*/
/*============ SET VELOCITY ZERO ============*/
void set_vels_zero(void)
{
  fb_vel=0.0;
  rl_vel=0.0;
  r_rpm=0;
  l_rpm=0;
}

/*===========================================================================*/
/*============================= main 함수 ====================================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosjoy_2_cmdvel");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher mode_pub = nh.advertise<std_msgs::Int8>("/mode", 10);
  ros::Publisher teleop_onoff_pub = nh.advertise<std_msgs::Int8>("/teleop_onoff", 10);

  //ros::Subscriber joy_sub = nh.subscribe("joy", 100, JOYCallback);

  ros::Rate loop_rate(50);


  char ch='n';
  printf("조이스틱 버튼을 재설정 하시겠습니까? [yN] : ");
  for(int k = 0;k<200;k++){  //4초 대기
    if(kbhit()){
      ch = getch();
      break;
    }
    loop_rate.sleep();
  }
  if(ch == 'y' || ch=='Y'){
    ros::Subscriber joy_setting_sub = nh.subscribe("joy", 100, JOY_setting_Callback);

    printf("\n삼각형 버튼을 누르세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(bttn_set_num != -1){
      BTTN_TRIANGLE = bttn_set_num;
      bttn_set_num = -1;
    }
    printf("\n사각형 버튼을 누르세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(bttn_set_num != -1){
      BTTN_SQUARE = bttn_set_num;
      bttn_set_num = -1;
    }
    printf("\n동그라미 버튼을 누르세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(bttn_set_num != -1){
      BTTN_O = bttn_set_num;
      bttn_set_num = -1;
    }
    printf("\n엑스 버튼을 누르세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(bttn_set_num != -1){
      BTTN_X = bttn_set_num;
      bttn_set_num = -1;
    }
    printf("\n왼쪽화살표 버튼을 누르세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(bttn_set_num != -1){
      BTTN_AXES_LEFTRIGHT = axes_set_num;
      bttn_set_num = -1;
    }

    printf("\n위쪽화살표 버튼을 누르세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(bttn_set_num != -1){
      BTTN_AXES_UPDOWN = axes_set_num;
      bttn_set_num = -1;
    }
    printf("\n왼쪽 조이스틱을 위로 미세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(axes_set_num != -1){
      AXES_LEFT_UPDOWN = axes_set_num;
      bttn_set_num = -1;
    }
    printf("\n오른쪽 조이스틱을 왼쪽으로 미세요\n");
    for(int k = 0;k<200;k++){ //4초 대기
      ros::spinOnce();
      loop_rate.sleep();

    }
    if(axes_set_num != -1){
      AXES_RIGHT_RL = axes_set_num;
      bttn_set_num = -1;
    }

  joy_setting_sub.shutdown();
  }
  else{
    printf("\n기존 설정된 조이스팅 세팅값을 사용합니다.\n");
  }

  ros::Subscriber joy_sub = nh.subscribe("joy", 10, JOYCallback);

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel_msg;
    std_msgs::Int8 mode_msg;
    std_msgs::Int8 teleop_onoff_msg;

    cmd_vel_msg.linear.x = fb_vel;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = rl_vel;

    mode_msg.data = operating_mode;



    if(operating_mode == Back_Hand_Control_){
      teleop_onoff_msg.data = 3;
    }
    else if(operating_mode == TORQUE_OFF){
      teleop_onoff_msg.data = 4;
    }
    else if(!(operating_mode == JoyNotUse)){
      cmd_vel_pub.publish(cmd_vel_msg);
      teleop_onoff_msg.data = 1;
    }
    else{ //operating_mode == Joy_not_use    -> use joystick to control Drok_arm
      teleop_onoff_msg.data = 2;
    }
    mode_pub.publish(mode_msg);
    teleop_onoff_pub.publish(teleop_onoff_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
/*===========================================================================*/
/*===========================================================================*/
