#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "kar_api.h"

KARSlave::KARSlave() : Slave(2000,"192.168.4.200",7){
  pthread_mutex_init(&moving_mutex,NULL);
  pthread_mutex_init(&err_s_mutex,NULL);
  limitstate = new int[8];
  calib_matrix = MatrixXd(6,6);
  calib_matrix << 0.113968674, 0.029522539,-0.646862966,-13.01761675, 0.370726892, 13.81545976,
                  0.477337883,  45.5007938,-0.320252662,-7.453172237,-0.227743897,-7.941477789,
                  -19.04522409, 4.344326468, -20.1082049,  0.43143785,-20.98900545,   1.3964193,
                  0.00268359, 0.251867143,  -0.2816617,-0.031614355, 0.287879378,-0.066184843,
                  0.321786038,-0.082730577, -0.15694134, 0.076631463,-0.177481228,-0.062598202,
                  0.006532045, 0.544146593, 0.006643124, 0.178591605, 0.005531052, 0.183404847;
  forcemomentoffset = VectorXd(6);
  forcemomentoffset <<  6.08,281.377,-505.970,1.53175623628147,-0.764013044544867,8.65344578652897;
  coef_force = 16.4032;
  coef_moment = 23.236932046192;
  forcerotate = MatrixXd(2,2);
  forcerotate << sin(-2.588488),cos(-2.588488),cos(-2.588488),-sin(-2.588488);//力センサの回転
  char *inifilename ="armdata.ini"; 
  int l_err_s = 0;
  l_err_s =  kar_init(inifilename);
  if(l_err_s != KAR_FUNC_OK){
    std::cout << "init error :" << l_err_s << std::endl;
    kar_end();
    exit(1);
  }
  std::cout << "KAR init finished " << std::endl;
  kar_set_mode(KAR_MODE_NORMAL);
#if defined(KARSLPARTENT)
  init_server();//initのタイミングに要注意
#endif

}

KARSlave::~KARSlave(){
  kar_end();
	sleep(1);
  delete[] limitstate;
	std::cout << "End prog" << std::endl;
}

void KARSlave::showlimitstate(){
  kar_get_limit(limitstate);
  std::cout << "limit state :" << limitstate[7] << " "<< std::flush;
  for(int ii=0;ii<6;ii++){
    std::cout << limitstate[ii] << " " << std::flush;
  }
  std::cout << std::endl;
}

int KARSlave::calib(void){
	int mode,axisn,err_signal;
	char str[256];                                                                                                                                                                                
	pthread_mutex_lock(&moving_mutex);
  err_signal = kar_get_mode(&mode);
	if(err_signal == KAR_FUNC_OK){
    std::cout << "mode :"<< mode << std::endl;
  }

  kar_clear_limit();//clear limit switch }
  int kardirect[8]    = {KAR_MODE_REVERSE,KAR_MODE_REVERSE,KAR_MODE_REVERSE,KAR_MODE_FORWORD,KAR_MODE_REVERSE,KAR_MODE_REVERSE,KAR_MODE_REVERSE,KAR_MODE_REVERSE};
  int kardirectrev[8] = {KAR_MODE_FORWORD,KAR_MODE_FORWORD,KAR_MODE_FORWORD,KAR_MODE_REVERSE,KAR_MODE_FORWORD,KAR_MODE_FORWORD,KAR_MODE_FORWORD,KAR_MODE_FORWORD};

  showlimitstate();
  axisn = 8;
  mode = kardirect[0] + axisn*10;
  kar_set_mode(mode);
  kar_get_mode(&mode);
  std::cout << "mode : " << mode << std::endl;//mode=182
  std::cout <<"aaa"<<std::endl;
  while(mode){err_signal = kar_get_mode(&mode); }
  mode = kardirectrev[0] + axisn*10;
  kar_set_mode(mode);
  kar_get_mode(&mode);
  std::cout << "mode : " << mode << std::endl;//mode=181
  std::cout<<"bbbb"<<std::endl;
  sleep(1);
  std::cout<<"ccccc"<<std::endl;
  kar_clear_limit();
  mode = KAR_MODE_ORG + axisn*10;
  kar_set_mode(mode);
  kar_get_mode(&mode);
  std::cout << "mode : " << mode << std::endl;//mode=180
  std::cout <<"ddddddd"<<std::endl;
  while(mode){err_signal = kar_get_mode(&mode);}
  mode = kardirectrev[0] + axisn*10;
  kar_set_mode(mode);
  kar_get_mode(&mode);
  std::cout << "mode : " << mode << std::endl;//mode=181
  std::cout <<"eeeee"<<std::endl;
  sleep(4);
  std::cout <<"fff"<<std::endl;
  showlimitstate();
  axisn = 5;//干渉よけsasaa12121
  mode = kardirect[0] + axisn*10;
  kar_set_mode(mode);
  kar_get_mode(&mode);
  while(mode){err_signal = kar_get_mode(&mode);}
  std::cout << "mode : " << mode << std::endl;//mode=0
  std::cout <<"gggg"<<std::endl;

  for(axisn=1;axisn<7;axisn++){
    if(axisn==4){
    std::cout<<"clear limit start"<<std::endl;
    kar_clear_limit();
    showlimitstate();
    std::cout<<"clear limit over"<<std::endl;
    std::cout<<"clear limit sleep over"<<std::endl;
    mode = kardirect[axisn] + axisn*10;
    std::cout <<"mode is"<<mode<<std::endl;
    std::cout<<"set start"<<std::endl;
    kar_set_mode(mode);
//    std::cout <<"set over"<<std::endl;
    sleep(3);
    std::cout <<"set sleep over"<<std::endl;
    std::cout <<"get start"<<std::endl;
    kar_get_mode(&mode);
    std::cout << kar_get_mode(&mode) << std::endl;
    sleep(3);
    std::cout <<"get sleep over"<<std::endl;
//    sleep(3);
//    std::cout <<"get sleep over"<<std::endl;
    std::cout << axisn<<"aclear"<<std::endl;
    std::cout <<"aclear"<<std::endl;
    std::cout << "mode : " << mode << std::endl;
    while(mode){err_signal = kar_get_mode(&mode);}
    std::cout <<"success"<<std::endl;
    sleep(3);
    std::cout <<"clear limit 2 start"<<std::endl;
    kar_clear_limit();
    std::cout <<"clear limit 2 over"<<std::endl;
//    sleep(3);
    std::cout <<"clear limit 2 sleep over"<<std::endl;
    mode = kardirectrev[axisn] + axisn*10;
    std::cout <<"set 2 start"<<std::endl;
    kar_set_mode(mode);
    std::cout <<"set 2 over"<<std::endl;
//    sleep(1);
    std::cout <<"set sleep 2 over"<<std::endl;
    std::cout <<"get 2 start"<<std::endl;
    kar_get_mode(&mode);
    std::cout <<"get 2 over"<<std::endl;
//    sleep(3);
    std::cout <<"get sleep 2 over"<<std::endl;
    std::cout<<axisn<<"bclear"<<std::endl;
    std::cout<<"bclear"<<std::endl;
    std::cout << "mode : " << mode << std::endl;
    sleep(3);
    std::cout <<"bbbbbbclear"<<std::endl;
    mode = KAR_MODE_ORG + axisn*10;
    std::cout <<"set 3 start"<<std::endl;
    kar_set_mode(mode);
    std::cout <<"set 3 over"<<std::endl;
    kar_get_mode(&mode);
    std::cout << "mode : " << mode << std::endl;
    std::cout<<"hhh"<<std::endl;
    while(mode){err_signal = kar_get_mode(&mode);}
    mode = kardirectrev[axisn] + axisn*10;
    kar_clear_limit();
    kar_set_mode(mode);
    kar_get_mode(&mode);
    std::cout << "mode : " << mode;
    std::cout <<"iiiii"<<std::endl;
    sleep(2);
    std::cout <<"jjjjjj"<<std::endl;


    }else{
    kar_clear_limit();
    showlimitstate();
    mode = kardirect[axisn] + axisn*10;
    kar_set_mode(mode);
    kar_get_mode(&mode);
    std::cout << kar_get_mode(&mode) <<std::endl;//this will be 0
    std::cout << axisn<<"aclear"<<std::endl;
    std::cout <<"aclear"<<std::endl;
    std::cout << "mode : " << mode << std::endl;
    while(mode){err_signal = kar_get_mode(&mode);}
    kar_clear_limit();
    mode = kardirectrev[axisn] + axisn*10;
    kar_set_mode(mode);
    kar_get_mode(&mode);
    std::cout<<axisn<<"bclear"<<std::endl;
    std::cout<<"bclear"<<std::endl;
    std::cout << "mode : " << mode << std::endl;
    sleep(1);
    std::cout <<"bbbbbbclear"<<std::endl;
    mode = KAR_MODE_ORG + axisn*10;
    kar_set_mode(mode);
    kar_get_mode(&mode);
    std::cout << "mode : " << mode << std::endl;
    std::cout<<"hhhhh"<<std::endl;
    while(mode){err_signal = kar_get_mode(&mode);}
    mode = kardirectrev[axisn] + axisn*10;
    kar_clear_limit();
    kar_set_mode(mode);
    kar_get_mode(&mode);
    std::cout << "mode : " << mode;
    std::cout <<"iiiii"<<std::endl;
    sleep(2);
    std::cout <<"jjjjjj"<<std::endl;}
    if(axisn==2){
      std::cout <<"kkkk"<<std::endl;
      sleep(3);
      std::cout <<"llll"<<std::endl;
    }
    if(axisn==3){
      std::cout<<"mmmm"<<std::endl;
      sleep(3);
      std::cout<<"nnnnnn"<<std::endl;
    }
    if(axisn==4){
      std::cout <<"oooooo"<<std::endl;
      sleep(12);
      std::cout<<"pppppp"<<std::endl;
    }
    if(axisn==5){
      std::cout<<"qqqqqq"<<std::endl;
      sleep(3);
      std::cout<<"rrrrrr"<<std::endl;
    }
    if(axisn==6){
      std::cout<<"sssssss"<<std::endl;
      sleep(3);
      std::cout<<"uuuuuu"<<std::endl;
    }
    showlimitstate();
  }
  kar_set_mode(KAR_MODE_STOP);
  std::cout<<"vvvv"<<std::endl;
  sleep(3);
  std::cout<<"wwwwww"<<std::endl;
  kar_set_mode(KAR_MODE_NORMAL);
  pthread_mutex_unlock(&moving_mutex);
  pthread_mutex_lock(&err_s_mutex);
  err_s = err_signal;
  pthread_mutex_unlock(&err_s_mutex);
  std::cout <<"end"<<std::endl;
  return 0;
}

int KARSlave::set_defoko_angle(){
  double targ[8] = { -18.6979 , 5.02352 , 98.8579 , -0.251937 , 75.2884 , 20.6719,KAR_KEEP_ANGLE,0.3};
  kar_clear_limit();
  pthread_mutex_lock(&moving_mutex);
  kar_set_mode(KAR_MODE_NORMAL);
  int ret = kar_set_target(KAR_PUSH_ANGLE,targ,10000);
  int waitt = 10000;
  kar_wait_interpolation(KAR_WAIT_ALL,waitt);
  pthread_mutex_unlock(&moving_mutex);
  return ret;
}

void KARSlave::stretch(){
  VectorXd ang_org = VectorXd::Zero(7);
  VectorXd ang_x1 = VectorXd::Zero(7);
  VectorXd ang_x2 = VectorXd::Zero(7);
  VectorXd ang_y1 = VectorXd::Zero(7);
  VectorXd ang_y2 = VectorXd::Zero(7);
  VectorXd ang_z1 = VectorXd::Zero(7);
  VectorXd ang_z2 = VectorXd::Zero(7);
  VectorXd ang_pitch = VectorXd::Zero(7);
  VectorXd ang_roll1 = VectorXd::Zero(7);
  VectorXd ang_roll2 = VectorXd::Zero(7);
  VectorXd ang_yaw1 = VectorXd::Zero(7);
  VectorXd ang_yaw2 = VectorXd::Zero(7);
  ang_org   << 0.3,-18.6979 , 5.02352 , 98.8579 , -0.251937 , 75.2884 , 20.6719;
  ang_roll1 << 0.3,-8.96105 , 3.48896 , 106.254 , -43.3522 , 68.6812 , 45.9612;
  ang_roll2 << 0.3,-5.44756 , 5.03074 , 110.51 , -65.0025 , 72.6649 , 59.731;
  ang_pitch << 0.3,-24.076 , -6.3382 , 124.408 , 80.958 , 20.6235 , -53.3371;
  ang_x1 << 0.3,-22.725 , -7.16469 , 113.376 , -0.19327 , 72.943 , 16.6379;
  ang_x2 << 0.3, -20.5241 , -1.09037 , 106.5 , -0.225984 , 73.7529 , 18.845;
  ang_y1 << 0.3,-28.2182 , 9.92327 , 93.5407 , -0.480918 , 75.302 , 11.0103;
  ang_y2 << 0.3, -8.33545 , 1.97482 , 102.757 , -0.403459 , 74.4956 , 31.0769;
  ang_z1 << 0.3, -18.6979 , 9.80742 , 108.08 , -0.277855 , 61.2821 , 20.7414;
  ang_z2 << 0.3, -18.6979 , 17.5648 , 113.943 , -0.329662 , 47.6628 , 20.83;
  double time = 3.0;
  pthread_mutex_lock(&moving_mutex);
  kar_set_mode(KAR_MODE_NORMAL);
  for(int ii=0;ii<10;ii++){
    time -= 0.15*(double)ii;
    set_angle(ang_org,2.0);
    set_angle(ang_pitch,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_roll1,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_roll2,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_x1,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_x2,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_y1,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_y2,2.0);
    set_angle(ang_org,2.0);
    set_angle(ang_z1,2.0);
  }
  set_angle(ang_org,3.0);
  pthread_mutex_unlock(&moving_mutex);
}

int KARSlave::set_angle(VectorXd ang,double t){
  double *targ = new double[8];
  for(int ii=0;ii<6;ii++){
    targ[ii] = ang(ii+1);
  }
  std::cout << "rail: " << ang[0] << std::endl; 
  kar_clear_limit();
  pthread_mutex_lock(&moving_mutex);
  kar_set_mode(KAR_MODE_NORMAL);
  targ[6] = KAR_KEEP_ANGLE;
  targ[7] = ang(0);
  int ret =  kar_set_target(KAR_PUSH_ANGLE,targ,(int)(t*1000));
  int waitt = 10000;
  kar_wait_interpolation(KAR_WAIT_ALL,waitt);
  pthread_mutex_unlock(&moving_mutex);
  delete[] targ;
  if(ret==-7){showlimitstate();}
  return ret;
}

int KARSlave::change_angle(VectorXd ang,double t){
  double *targ = new double[8];
  for(int ii=0;ii<6;ii++){
    targ[ii] = ang(ii+1);
  }
  std::cout << "rail: " << ang[0] << std::endl; 
  kar_clear_limit();
  pthread_mutex_lock(&moving_mutex);
  kar_set_mode(KAR_MODE_NORMAL);
  targ[6] = KAR_KEEP_ANGLE;
  targ[7] = ang(0);
  int ret =  kar_set_target(KAR_CHANGE_ANGLE,targ,(int)(t*1000));
  int waitt = 10000;
  pthread_mutex_unlock(&moving_mutex);
  delete[] targ;
  if(ret==-7){showlimitstate();}
  return ret;
}


int KARSlave::set_sequence(Sequence<double,VectorXd> &seq){
  int motion_num = seq.size();
  double **seqqd;
  seqqd = new double*[motion_num];
  for(int ii=0;ii<motion_num;ii++){
    seqqd[ii] = new double[8];
  }
  int *time = new int[motion_num];
  double l_t=0.0,l_old_t=0.0;
  VectorXd l_jointangle;
  for(int ii=0;ii<motion_num;ii++){
    seq.get(ii,l_t,l_jointangle);
    if(l_t<0.001){
      time[ii] = (int)(5000);//最初は安全目の値に
    }else{
      time[ii] = (int)(1000.0*(l_t - l_old_t));
    }
    seqqd[ii][7] = l_jointangle(0);//rail
    seqqd[ii][6] = KAR_KEEP_ANGLE;//存在しないない軸
    for(int jj=0;jj<6;jj++){
      seqqd[ii][jj] = l_jointangle(jj+1);
    }
    l_old_t = l_t;
  }
  std::cout << std::endl;
  
  std::cout << "　CAUTION*CAUTION*CAUTION     start moving      CAUTION*CAUTION*CAUTION" << std::endl;
  pthread_mutex_lock(&moving_mutex);
  int ret = kar_set_target_sequence(KAR_PUSH_ANGLE, seqqd, time,motion_num);
  pthread_mutex_unlock(&moving_mutex);
  std::cout << " moving to " << seqqd[motion_num-1][7] << " err: "<< ret << std::endl;
  //std::cout << "<======================seq num is" << motion_num << std::endl;
  delete[] time;
  for(int ii=0;ii<motion_num;ii++){
    delete[] seqqd[ii];
  }
  delete[] seqqd;
  return ret;
}

void KARSlave::get_angle(){
  double *buff = new double [8];
  kar_get_angle(buff);
  current_angle(0) =  buff[7];
  for(int ii=1;ii<6;ii++){
    current_angle(ii+1) = buff[ii];
  }
  //showvec(current_angle);
   delete buff;
}

void KARSlave::get_force(){
  int *buff = new int [6];
  kar_get_force(buff);
  VectorXd sensorval(6);
  VectorXd l_current_force(6);
  for(int ii=0;ii<6;ii++){
    sensorval(ii) = ((((double)buff[ii]/65535)*20)-10);
  }
  l_current_force = calib_matrix*sensorval + forcemomentoffset;
  for(int ii=0;ii<3;ii++){
    l_current_force(ii) = coef_force*l_current_force(ii);//単位変換 [N]へ
  }
  for(int ii=3;ii<6;ii++){
    l_current_force(ii) = coef_moment*l_current_force(ii);//単位変換 [Nm]へ
  }
  l_current_force.block(0,0,2,1) = forcerotate*l_current_force.block(0,0,2,1);
  l_current_force.block(3,0,2,1) = forcerotate*l_current_force.block(3,0,2,1);

  current_force = l_current_force;
  delete buff;
}


void KARSlave::show_angle(){
  std::cout << "angles are " << std::endl;
  for(int ii=0;ii<7;ii++){
    std::cout << ii << " : "<< current_angle(ii) << "\t";
  }
  std::cout << std::endl;
}
void KARSlave::geterr_do(){
  int l_err_s;
  pthread_mutex_lock(&err_s_mutex);
  l_err_s = err_s;
  pthread_mutex_unlock(&err_s_mutex);
  kar_get_error(&err_num);
  error = err_num*10.0+l_err_s;
  //if(error!=0){std::cout << error << std::endl;}
}

void KARSlave::setq_do(){
  int l_err_s = change_angle(target_angle,set_time);
  showvec(target_angle);
  pthread_mutex_lock(&err_s_mutex);
  err_s = l_err_s;
  pthread_mutex_unlock(&err_s_mutex);
}

void KARSlave::getq_do(){
  get_angle();
}

void KARSlave::setsq_do(){
  int l_err_s = set_sequence(target_seq);
  pthread_mutex_lock(&err_s_mutex);
  err_s = l_err_s;
  pthread_mutex_unlock(&err_s_mutex);
  sleep(3);
  target_seq.clear();
}

void KARSlave::getf_do(){
  get_force();
}
void KARSlave::calib_do(){
  int l_err_s =  calib(); 
  pthread_mutex_lock(&err_s_mutex);
  err_s = l_err_s;
  pthread_mutex_unlock(&err_s_mutex);
}
void KARSlave::defoko_do(){
  int l_err_s = set_defoko_angle();
  pthread_mutex_lock(&err_s_mutex);
  err_s = l_err_s;
  pthread_mutex_unlock(&err_s_mutex);
}

#if defined(KARSL_IS_MAIN)
int main(){
  KARSlave *ks = new KARSlave;
  
  /*test
  ks->set_defoko_angle();
  VectorXd ang = VectorXd::Zero(7);
  Sequence<double,VectorXd> seq;
  ang << 0.3,-18.6979 , 5.02352 , 98.8579 , -0.251937 , 75.2884 , 20.6719;
  double time = 0;
  for(int ii=0;ii<8;ii++){
    time += 1.0;
    ang(0) += 0.1;
    seq.push_back(time,ang);
  }
  ks->set_sequence(seq);
  */

  ks->waittoallend();
  delete ks;
}
#endif
