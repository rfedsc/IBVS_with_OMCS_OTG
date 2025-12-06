import time
class PIDController:
    def __init__(self,Kp,Ki,Kd,setpoint,output_limits=(None,None)):
        #PID控制器的增益
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
                
        #目标位置
        self.setpoint = setpoint
        #当前误差
        self.prev_error = 0.0
        self.error = 0
        self.integral = 0.0
        #输出限值
        self.output_limits = output_limits
        
    def compute(self,current_position):
        self.error = self.setpoint - current_position
        #计算积分项(避免积分过饱和)
        self.integral += self.error
        #计算微分项(避免大变化的误差)
        self.derivative = self.error - self.prev_error
        
        #PID控制公式
        self.output = self.Kp*self.error + self.Ki*self.integral + self.Kd*self.derivative
        #更新前一个误差
        self.prev_error = self.error
        print(f"当前误差为:\r\n {self.error}")
        print(f"前一个误差为:\r\n{self.prev_error}")
        
        #进行输出限幅
        if self.output_limits[0] is not None:
            self.output = max(self.output,self.output_limits[0])
        if self.output_limits[1] is not None:
            self.output = min(self.output,self.output_limits[1])

        return self.output



