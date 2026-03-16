import RPi.GPIO as GPIO

class Lazer:
    """激光控制类"""    
    def __init__(self, led_pin=17):
        self.led_pin = led_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT, initial=GPIO.LOW)
        self.state = 0

    def turn_on(self):
        """打开激光"""
        GPIO.output(self.led_pin, GPIO.HIGH)
        self.state = 1
        print("激光已打开")

    def turn_off(self):
        """关闭激光"""
        GPIO.output(self.led_pin, GPIO.LOW)
        self.state = 0
        print("激光已关闭")

    def cleanup(self):
        """清理GPIO资源"""
        # self.turn_off()
        # self.state = 0
        GPIO.cleanup()

    def __del__(self):
        """析构函数，确保清理GPIO资源"""
        self.cleanup()
        print("Lazer对象已销毁，GPIO资源已清理")
