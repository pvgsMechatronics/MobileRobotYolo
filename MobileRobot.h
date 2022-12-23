#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <List.hpp>

#define PI 3.1415926535897932384626433832795

class MobileRobot
{
  private:
    //states of the robot
    float x = 0; //position X of the robot
    float y = 0; //position Y of the robot
    float vx = 0; //linear speed on X axis 
    float vy = 0; //linear speed on Y axis
    float v = 0; //escalar linear speed of the robot
    float w = 0; //angular speed of the robot on Z axis
    float theta = 0; //theta angle of the robot

    //control parameters 
    float K[4] = {0,0,0,0}; //gain vector of dynamic control
    float k[2] = {0,0}; //gain vector of path tracking control
    float u[2] = {0,0}; //control signal
    float sat = 0; //saturation of the motors
    float T = 0; //sample time
    float sd = 0; //distance from the centroid of the robor to his sensor

    //odometry and log
    String log = String(""); //save the states of the system [x, y, theta, v, w, u[0], u[1]]
    Adafruit_MPU6050 sensor; //class to read the MPU6050 sensor
    float offset_a = 0; //to calibrate aceleration x
    float offset_w = 0; //to calibrate angular speed (gyroZ)

    //trajectory tracking
    List<float> path_x;
    List<float> path_y;
    float x_ref = 0;
    float y_ref = 0;
    bool on_path = false;

  public:
    MobileRobot(float _K[4], float _k[2], float _sat, float _T, float _sd){
      sd = _sd;
      T = _T;
      sat = _sat;
      for (int i=0; i<4; i++){
        K[i] = _K[i];
        if (i<2){
          k[i] = _k[i];
        } 
      }
    }


    void calibrate(int N, int RM1, int RM2, int LM1, int LM2){
      pinMode(LM1, OUTPUT); 
      pinMode(LM2, OUTPUT);
      pinMode(RM1, OUTPUT);
      pinMode(RM2, OUTPUT);
      ledcAttachPin(RM1, 0);
      ledcAttachPin(RM2, 1);
      ledcAttachPin(LM1, 2);
      ledcAttachPin(LM2, 3);
      ledcSetup(0, 1000, 16); 
      ledcSetup(1, 1000, 16);
      ledcSetup(2, 1000, 16);
      ledcSetup(3, 1000, 16);
      while (!sensor.begin()){
        Serial.println("Failed to find MPU6050 chip"); 
        delay(T*1000);  
      }
      Serial.println("MPU6050 Found!");
      sensor.setAccelerometerRange(MPU6050_RANGE_16_G);
      sensor.setGyroRange(MPU6050_RANGE_2000_DEG);
      sensor.setFilterBandwidth(MPU6050_BAND_184_HZ);
      robot_stop();
      offset_a = 0;
      Serial.println("MPU6050 Calibration...");
      for (int i=0; i<N; i++){
        sensors_event_t a, g, temp;
        sensor.getEvent(&a, &g, &temp);
        offset_a = offset_a + a.acceleration.x;
        offset_w = offset_w + a.gyro.z;
        delay(int(T*1000));
        Serial.print(".");
      }
      robot_off();
      offset_a = (-1)*(offset_a)/N;
      offset_w = (-1)*(offset_w)/N;
      Serial.println("");
      Serial.println("MPU6050 Calibrated!");
    }

    void update_pos(float _x, float _y){
      x = _x;
      y = _y;
    }

    void actuators(float u[2]){
      int signal0 = int((65536/sat)*u[0]);
      int signal1 = int((65536/sat)*u[1]);
      //right motor 
      if (u[0] >= 0){
        ledcWrite(0, 0);
        ledcWrite(1, signal0);
      }
      else{
        ledcWrite(1, 0);
        ledcWrite(0, -signal0);
      }
      //left motor 
      if (u[1] >= 0){
        ledcWrite(2, 0); 
        ledcWrite(3, signal1);
      }
      else{
        ledcWrite(3, 0); 
        ledcWrite(2, -signal1);
      }
      delay(int(T*1000));
    }


    void robot_stop(){
      ledcWrite(3, 65536);
      ledcWrite(2, 65536);
      ledcWrite(1, 65536);
      ledcWrite(0, 65536);
      delay(1000);
    }


    void robot_off(){
      ledcWrite(3, 0);
      ledcWrite(2, 0);
      ledcWrite(1, 0);
      ledcWrite(0, 0);
    }  


    void update_log(){
      log = String(log + "[");
      log = String(log + String(x, 5) + ",");
      log = String(log + String(y, 5) + ",");
      log = String(log + String(theta, 5) + ",");
      log = String(log + String(v, 5) + ",");
      log = String(log + String(w, 5) + ",");
      log = String(log + String(u[0], 5) + ",");
      log = String(log + String(u[1], 5) + "]");
    }
    String get_log(){
      return log;
    }


    void update_odometry(){
      sensors_event_t _a, _g, _temp;
      sensor.getEvent(&_a, &_g, &_temp);
      float a = _a.acceleration.x + offset_a;
      float new_w = _g.gyro.z + offset_w;
      theta = theta + w*T + ((new_w-w)/T)*(pow(T,2)/2);
      w = new_w;  
      x = x + vx*T + a*cos(theta)*(pow(T,2)/2);
      y = y + vy*T + a*sin(theta)*(pow(T,2)/2);
      v = v + a*T; 
      vx = v*cos(theta);
      vy = v*cos(theta);
      update_log();   
    }

    bool get_on_path(){
      return on_path;
    }

    void receive_path(String path){
      float p;
      while (path.indexOf("[") != (-1))
      {
        p = path.substring(path.indexOf("[") + 1, path.indexOf(",")).toFloat();
        path_x.addLast(p);
        p = String(path.substring(path.indexOf(",") + 1, path.indexOf("]"))).toFloat();
        path_y.addLast(p);
        path = path.substring(path.indexOf("]") + 1);
      } 
    }

    void controler(float vr, float wr){

      float T[4] = {30.3030, 1.1818, 30.3030, -1.1818};
      float wd = T[0]*v + T[1]*w;
      float we = T[2]*v + T[3]*w;
      float wdr = T[0]*vr + T[1]*wr;
      float wer = T[2]*wr + T[3]*wr;
      float u[2];

      u[0] = wdr -wd*K[0] - wdr*K[1]; //right motor control signal
      u[1] = wer -wd*K[2] - wer*K[3]; //left motor control signal

      //saturation
      if (u[0] > sat){
        u[0] = sat;
      }
      if (u[1] > sat){
        u[1] = sat;
      } 

      actuators(u);
      update_odometry();
    }

    void follow_path(float v_ref){
      if (path_x.getSize() == 0){
        return;        
      }
      if (path_x.getSize() == 1){
        path_x.removeFirst();
        path_y.removeFirst();
        robot_stop();
        robot_off();
        update_odometry();
        on_path = false;
        return;
      }
      if (!on_path){
        on_path = true;
        x_ref = path_x[0];
        y_ref = path_y[0];
        return;
      }   
      if (sqrt(pow(path_x[1]-x,2)+pow(path_y[1]-y,2)) <= 0.1){
        path_x.removeFirst();
        path_y.removeFirst();
        return;
      }
      
      //if the distance between the target and the robot is less than 10cm, the robot jumps to the next target
      float w_ref, vx_ref, vy_ref;
      float cost = (path_x[1]-path_x[0])/sqrt(pow(path_x[1]-path_x[0],2)+pow(path_y[1]-path_y[0],2)); // cos = adj/hip
      float sint = (path_y[1]-path_y[0])/sqrt(pow(path_x[1]-path_x[0],2)+pow(path_y[1]-path_y[0],2)); // sin = opo/hip

      x_ref = x_ref + v_ref*cost*T;
      y_ref = y_ref + v_ref*sint*T;

      vx_ref = k[0]*(x_ref - x);  
      vy_ref = k[1]*(y_ref - y);
      v_ref = vx_ref*cos(theta) + vy_ref*sin(theta);
      w_ref = (-1)*vx_ref*sin(theta)/sd + vy_ref*cos(theta)/sd;  
      controler(v_ref, w_ref);    

      return;
    }
};
