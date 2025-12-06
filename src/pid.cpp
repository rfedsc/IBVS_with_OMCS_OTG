#include <iostream>

class PositionPID{
public:
    //构造函数初始化PID参数
    PositionPID(double Kp,double Ki,double Kd)
        :Kp(Kp),Ki(Ki),Kd(Kd),prev_error(0),intergral(0),output(0){}
    
    //计算PID控制量
    double calculate(double setpoint,double current_value,double dt){
        //计算误差
        double error = setpoint - current_value;
        
        //积分(防止积分值过大,可以加上积分上限)
        integral += error*dt;
        
        //微分(避免大误差时微分过大,可以加上微分上限)
        double derivative = (error-prev_error)/dt;
        
        //计算PID控制输出
        output = Kp*error + Ki*integral + Kd*derivative;
        
        //更新历史误差
        prev_error = error;
    
        return output;
    }
    void reset(){
        prev_error = 0;
        integral = 0;
        output = 0;
    }
private:
    double Kp,Ki,Kd;
    double prev_error;
    double integral;
    double output;
}



