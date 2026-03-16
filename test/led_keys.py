#!/usr/bin/env python3
# led_keys.py
import signal
import sys
import time
import RPi.GPIO as GPIO

LED_PIN  = 17
KEY1_PIN = 5
KEY2_PIN = 6

# 使用 BCM 编号
GPIO.setmode(GPIO.BCM)

# LED 输出，默认低电平（熄灭）
GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)

# 按键输入，内部上拉
GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 事件回调 -----------------------------------------------------------
def key1_pressed(channel):
    print("KEY1 pressed → LED ON")
    GPIO.output(LED_PIN, GPIO.HIGH)

def key2_pressed(channel):
    print("KEY2 pressed → LED OFF")
    GPIO.output(LED_PIN, GPIO.LOW)

# 注册中断（下降沿触发，按键按下）
GPIO.add_event_detect(
    KEY1_PIN, GPIO.FALLING, callback=key1_pressed, bouncetime=200
)
GPIO.add_event_detect(
    KEY2_PIN, GPIO.FALLING, callback=key2_pressed, bouncetime=200
)

# 优雅退出 -----------------------------------------------------------
def cleanup(signum, frame):
    print("\nCleaning up GPIO...")
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

print("等待按键事件（Ctrl+C 退出）")
# 主循环只做睡眠，事件由中断处理
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    cleanup(None, None)
