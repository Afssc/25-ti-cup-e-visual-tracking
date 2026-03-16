#!/usr/bin/env python
import time
import signal
import sys
import serial
import RPi.GPIO as GPIO

LED_PIN   = 17
KEY1_PIN  = 5
KEY2_PIN  = 6
UART_PORT = '/dev/ttyUSB0'
UART_BAUD = 115200
TIMEOUT   = 0.2

# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 串口
try:
    ser = serial.Serial(UART_PORT, UART_BAUD, timeout=TIMEOUT)
except serial.SerialException as e:
    print("无法打开串口:", e)
    sys.exit(1)

# ------------------ 工具函数 ------------------
def send_cmd(cmd_bytes):
    ser.write(cmd_bytes)
    ser.flush()
    return ser.read(5)

def homing_ok(flag_byte: int) -> bool:
    """
    判断回零是否完成（无正在回零、无回零失败）
    flag_byte: 从 0x3B 命令返回的第 3 字节
    返回 True 表示已完成回零
    """
    return (flag_byte & 0x0C) == 0x00   # 0x04|0x08 均为 0

# ------------------ KEY1 流程 ------------------
def key1_pressed(channel):
    t0 = time.perf_counter()
    print("KEY1 → 发送 01 9A 00 00 6B")
    resp1 = send_cmd(b'\x01\x9A\x00\x00\x6B')
    if resp1 == b'\x01\x9A\x02\x6B':
        print("收到 01 9A 02 6B → 发送 02 9A 00 00 6B")
        resp2 = send_cmd(b'\x02\x9A\x00\x00\x6B')
        if resp2 == b'\x02\x9A\x02\x6B':
            print("收到 02 9A 02 6B → 点亮 LED")
            resp3 = send_cmd(b'\x01\x3B\x6B')
            if resp3 and len(resp3) == 4:
                if homing_ok(resp3[2]):
                    print("1号到位")
                    resp4 = send_cmd(b'\x01\x3B\x6B')
                    if resp4 and len(resp4) == 4:
                        if homing_ok(resp4[2]):
                            print("2号到位")
                            print("开启激光")
                            GPIO.output(LED_PIN, GPIO.HIGH)
                            elapsed = time.perf_counter() - t0
                            print(f"KEY1→LED 耗时: {elapsed*1000:.1f} ms")

# ------------------ KEY2 关灯 ------------------
def key2_pressed(channel):
    print("KEY2 → 熄灭 LED")
    GPIO.output(LED_PIN, GPIO.LOW)

# ------------------ 事件注册 ------------------
GPIO.add_event_detect(KEY1_PIN, GPIO.FALLING, callback=key1_pressed, bouncetime=300)
GPIO.add_event_detect(KEY2_PIN, GPIO.FALLING, callback=key2_pressed, bouncetime=300)

# ------------------ 退出清理 ------------------
def cleanup(signum, frame):
    print("\n清理 GPIO 与串口...")
    GPIO.cleanup()
    ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

print("等待按键：KEY1 开灯 / KEY2 关灯（Ctrl+C 退出）")
try:
    signal.pause()
except KeyboardInterrupt:
    cleanup(None, None)
