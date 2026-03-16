import numpy as np
import time
import serial

class Zdt(object):
    def __init__(self):
        self.interpolate = 256
        self.pul_per_rev = self.interpolate * 200
        self.vel = 1000  # RPM
        self.acc = 253     # 加速度
        self.delay = 0.005
        self.uart = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust port and baudrate as needed
        
    def Emm_V5_Pos_Control(self,addr, dir, vel, acc, clk, raF, snF): # 地址电机，设置方向为CW，速度为1000RPM，加速度为50，脉冲数为2000，相对运动，无多机同步
        cmd = bytearray(16)
        cmd[0] = addr                      # 地址
        cmd[1] = 0xFD                      # 功能码
        cmd[2] = dir                       # 方向
        cmd[3] = (vel >> 8) & 0xFF         # 速度(RPM)高8位字节
        cmd[4] = vel & 0xFF                # 速度(RPM)低8位字节 
        cmd[5] = acc                       # 加速度，注意：0是直接启动
        cmd[6] = (clk >> 24) & 0xFF        # 脉冲数高8位字节(bit24 - bit31)
        cmd[7] = (clk >> 16) & 0xFF        # 脉冲数(bit16 - bit23)
        cmd[8] = (clk >> 8) & 0xFF         # 脉冲数(bit8  - bit15)
        cmd[9] = clk & 0xFF                # 脉冲数低8位字节(bit0  - bit7)
        cmd[10] = 0x01 if raF else 0x00    # 相位/绝对标志，true为0x01，false为0x00
        cmd[11] = 0x01 if snF else 0x00    # 多机同步运动标志，true为0x01，false为0x00
        cmd[12] = 0x6B                     # 校验字节
        self.uart.write(cmd[:13])

    def deg2pulse(self,deg:float, pul_per_rev:int) -> tuple[int,int]:
        """
        Convert degrees to pulses.
        
        Parameters:
        deg (float): Angle in degrees.
        pul_per_rev (int): Pulses per revolution, default is 2000.
        
        Returns:
        int: Direction.
        int: Equivalent pulses for the given angle.
        """
        pulse = int(np.round((deg / 360.0) * pul_per_rev,0)) 
        return int(pulse > 0) , abs(pulse)

    def zdt_yp(self,yaw:float,pitch:float):
        Emm_V5_Pos_Control = self.Emm_V5_Pos_Control
        deg2pulse = self.deg2pulse
        yaw_dir , yaw_pulse = deg2pulse(yaw,self.pul_per_rev)
        pitch_dir , pitch_pulse = deg2pulse(pitch,self.pul_per_rev)
        Emm_V5_Pos_Control(addr=1,dir=yaw_dir,vel=self.vel,acc=self.acc,clk=yaw_pulse,raF=False,snF=False)  # 相对运动
        time.sleep(self.delay)  # 等待一段时间以确保命令执行
        Emm_V5_Pos_Control(addr=2,dir=pitch_dir,vel=self.vel,acc=self.acc,clk=pitch_pulse,raF=False,snF=False)  # 相对运动
        time.sleep(self.delay)  # 等待一段时间以确保命令执行
        
    def zdt_abs_yaw(self,yaw:float,vel_=300,acc_=200):
        Emm_V5_Pos_Control = self.Emm_V5_Pos_Control
        deg2pulse = self.deg2pulse
        yaw_dir , yaw_pulse = deg2pulse(yaw,self.pul_per_rev)
        Emm_V5_Pos_Control(addr=1,dir=yaw_dir,vel=vel_,acc=acc_,clk=yaw_pulse*4,raF=True,snF=False)  # 相对运动
        time.sleep(self.delay)  # 等待一段时间以确保命令执行
        
    def zdt_homing(self,addr):
        cmd = bytearray(16)
        cmd[0] = addr
        cmd[1] = 0x9A
        cmd[2] = 0x00
        cmd[3] = 0x00
        cmd[4] = 0x6B
        self.uart.write(cmd[:5])
        self.uart.flush()
        return self.uart.read(5) == bytearray([addr,0x9A,0x02,0x6B])

    def zdt_check_homing(self,addr):
        cmd = bytearray(16)
        cmd[0] = addr
        cmd[1] = 0x3B
        cmd[2] = 0x6B
        self.uart.write(cmd[:3])
        self.uart.flush()
        resp = self.uart.read(4)
        if len(resp) == 4:
            return (resp[0] == addr and resp[1] == 0x3B and (resp[2] & 0x0C) == 0x00)
        else:
            return False
        

    
    def __del__(self):
        if self.uart.is_open:
            self.uart.close()
    

if __name__ == "__main__":
    # Example usage
    zdt = Zdt()
    print("控制张大头云台运动，输入两个角度，小数")
    y,p =  map(float, input("yaw pitch: ").split())
    zdt.zdt_yp(yaw=y,pitch=p)
    del zdt  # 清理资源