#!/usr/bin/env python3
# key_array.py - 树莓派矩阵按键检测程序
import signal
import sys
import time
import cv2
import cv
from key_callbacks import *
import RPi.GPIO as GPIO

# GPIO配置
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 引脚定义
ALL_PINS = [18, 23, 24, 25, 12, 16, 20, 21]
ROW_PINS = ALL_PINS[:4]  # 行引脚: [18, 23, 24, 25]
COL_PINS = ALL_PINS[4:]  # 列引脚: [12, 16, 20, 21]


# 按键状态记录
last_key_state = [[False for _ in range(4)] for _ in range(4)]
key_press_time = [[0 for _ in range(4)] for _ in range(4)]

KEY_MAP = []

# 防抖延时(毫秒)
DEBOUNCE_DELAY = 50

def setup_gpio():
    """初始化GPIO引脚"""
    # 设置行引脚为输出，初始为高电平
    for pin in ROW_PINS:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
    
    # 设置列引脚为输入，启用上拉电阻
    for pin in COL_PINS:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
def key_map_init(zdt_instance,lazer_instance, cv_cap, cv_config:cv.GimbalConfig):
    """初始化按键映射"""
    # 按键映射，4x4矩阵
    global KEY_MAP
    yt = lambda angle: create_yaw_cv_function(angle,zdt_instance,cv_cap,cv_config) #太长了，写短
    lz_off = lazer_instance.turn_off  # 激光关闭函数引用
    # t2 = lambda:test2(zdt_instance,lazer_instance)  
    t2 = lambda:test2_close(cv_config=cv_config,cap=cv_cap)  
    cv_debug_on = lambda :debug_on(cv_cap,cv_config)

    KEY_MAP = [
        [yt(-45+90)  , yt(-18+90)  , yt(18+90)   , yt(45+90) ],
        [yt(-72+90)  , t2          , lz_off      , yt(72+90) ],
        [yt(-108+90) , lambda:None , cv_debug_on , yt(108+90)],
        [yt(-135+90) , yt(-162+90) , yt(162+90)  , yt(135+90)]
    ]


def scan_keys():
    """扫描矩阵按键"""
    pressed_keys = []
    
    for row in range(4):
        # 设置当前行为低电平
        GPIO.output(ROW_PINS[row], GPIO.LOW)
        time.sleep(0.001)  # 短暂延时确保电平稳定
        
        # 检查每一列
        for col in range(4):
            if GPIO.input(COL_PINS[col]) == GPIO.LOW:
                # 按键被按下
                current_time = int(time.time() * 1000)  # 转换为毫秒整数
                
                # 防抖处理
                if not last_key_state[row][col]:
                    # 新按下的按键
                    if current_time - key_press_time[row][col] > DEBOUNCE_DELAY:
                        last_key_state[row][col] = True
                        key_press_time[row][col] = current_time
                        pressed_keys.append((row, col, KEY_MAP[row][col]))
                        
            else:
                # 按键被释放
                if last_key_state[row][col]:
                    current_time = int(time.time() * 1000)  # 转换为毫秒整数
                    if current_time - key_press_time[row][col] > DEBOUNCE_DELAY:
                        last_key_state[row][col] = False
                        key_press_time[row][col] = current_time
        
        # 恢复当前行为高电平
        GPIO.output(ROW_PINS[row], GPIO.HIGH)
        
    return pressed_keys

def on_key_press(row, col, key_callback):
    """按键按下事件处理函数"""
    print(f"按键按下:  (位置: 行{row+1}, 列{col+1})")
    key_callback()  # 调用按键对应的函数
    
    # 在这里添加你的按键处理逻辑
    # if key_value.isdigit():
    #     print(f"数字键: {key_value}")
    # elif key_value in ['A', 'B', 'C', 'D']:
    #     print(f"功能键: {key_value}")
    # elif key_value == '*':
    #     print("星号键被按下")
    # elif key_value == '#':
    #     print("井号键被按下")

def cleanup_gpio():
    """清理GPIO资源"""
    print("清理GPIO资源...")
    GPIO.cleanup()

def signal_handler(sig, frame):
    """信号处理函数，用于优雅退出"""
    print("\n接收到退出信号，正在清理资源...")
    cleanup_gpio()
    sys.exit(0)

def main():
    """主函数"""
    print("矩阵按键检测程序启动...")
    print("按键映射:")
    for i, row in enumerate(KEY_MAP):
        print(f"第{i+1}行: {' '.join(row)}")
    print("按 Ctrl+C 退出程序\n")
    
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 初始化GPIO
    setup_gpio()
    zdt_instance = zdt.Zdt()
    lazer_instance = lazer.Lazer()
    gimbalcfg = cv.GimbalConfig(yp_move_callback=zdt_instance.zdt_yp,lazer_instance=lazer_instance)

    cap = cv2.VideoCapture(0)
    assert cap.isOpened()
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Set auto exposure to manual mode
    cap.set(cv2.CAP_PROP_EXPOSURE, -8)  # Set exposure value (adjust as needed)
     
    key_map_init(zdt_instance, lazer_instance, cap, gimbalcfg)
    
    try:
        while True:
            # 扫描按键
            pressed_keys = scan_keys()
            
            # 处理按下的按键
            for row, col, key_callback in pressed_keys:
                on_key_press(row, col, key_callback)
            
            # 短暂延时，避免过度占用CPU
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()