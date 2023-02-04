from mpu6050 import mpu6050
import time


def imu_data(mpu):
    
    print("Temp : "+str(mpu.get_temp()))
    print()

    gyro_data = mpu.get_gyro_data()

    return gyro_data

while True:
    mpu = mpu6050(0x68)
    test = imu_data(mpu)
    print("Gyro X : "+str(test['x']))
    print("Gyro Y : "+str(test['y']))
    print("Gyro Z : "+str(test['z']))
    print()
    print("-------------------------------")
    time.sleep(5)




    def forward(self, distance, pwm):
        # Motor # 1
        # Creating a PWM object
        #myPWM = GPIO.PWM(ENA_PIN, 100)
        self.myPWM.start(pwm)

        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

        # Motor # 2
        # myPWM2 = GPIO.PWM(ENB_PIN, 100)
        self.myPWM2.start(pwm)

        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        velocity = 3.1555e-3*(pwm) + 0.0267

        # Time
        t = distance/velocity
        # Time measurement is in seconds
        time.sleep(t)