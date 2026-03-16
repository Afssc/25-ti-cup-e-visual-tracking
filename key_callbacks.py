import zdt
import lazer
import cv
import time

def test2(zdt:zdt.Zdt, lazer:lazer.Lazer):
    """
    赛题2!! 
    """
    if (zdt.zdt_homing(addr=1)):
        print("1号云台回零完成")
        if(zdt.zdt_homing(addr=2)):
            print("2号云台回零完成") 
            if (zdt.zdt_check_homing(addr=1)):
                print("1号云台回零检查通过")
                if(zdt.zdt_check_homing(addr=2)):
                    print("2号云台回零检查通过")
                    print("开启激光！！！")
                    time.sleep(0.05)
                    lazer.turn_on()
                    
def test2_close(cv_config:cv.GimbalConfig,cap):
    clear_camera_buffer(cap, 10)
    cv_config.keep_aim = False
    cv.cv_loop(cap,cv_config)
    clear_camera_buffer(cap, 10)

                    
def turnoff_lazer(lazer:lazer.Lazer):
    """
    赛题2!! 
    """
    print("关闭激光！！！")
    time.sleep(0.05)
    lazer.turn_off()

def create_yaw_cv_function(angle: float,zdt_instance: zdt.Zdt,cv_cap, cv_config):
    """
    函数工厂：传入角度，返回一个函数对象
    该函数会让张大头云台移动到指定yaw角度，然后开始cv_loop
    
    Args:
        angle (float): 目标yaw角度
        
    Returns:
        function: 可执行的函数对象
    """
    def yaw_and_cv():
        """
        执行云台yaw轴移动到指定角度并开始CV循环
        """
        # 开始前先清空一次缓冲（可选）
        clear_camera_buffer(cv_cap, 10)
        
        print(f"张大头云台移动到yaw角度: {angle}度")
        zdt_instance.zdt_abs_yaw(angle)
        time.sleep(0.7)
        print(f"云台已移动到yaw={angle}度，开始CV循环...")
        cv.cv_loop(cv_cap, cv_config)
        
        # CV循环结束后清空帧缓冲，避免下次调用时的帧滞留
        print("CV循环结束，清空帧缓冲...")
        clear_camera_buffer(cv_cap, 10)
    
    return yaw_and_cv 

def clear_camera_buffer(cap, num_frames=5):
    """
    清空摄像头帧缓冲
    
    Args:
        cap: cv2.VideoCapture对象
        num_frames: 要丢弃的帧数
    """
    print(f"清空摄像头缓冲，丢弃{num_frames}帧...")
    for i in range(num_frames):
        ret, _ = cap.read()
        if not ret:
            print(f"缓冲清理完成，实际丢弃{i}帧")
            break
    else:
        print(f"缓冲清理完成，丢弃{num_frames}帧") 

    
def debug_on(cap,gimbalcfg:cv.GimbalConfig):
    gimbalcfg.debug = True
    cv.cv_loop(cap,gimbalcfg)
    gimbalcfg.debug = False
    

