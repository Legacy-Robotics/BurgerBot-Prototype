
from time import sleep
from gpiozero import AngularServo

#Indexer
indexServo = AngularServo(24, min_angle=0, max_angle=180)

#Motors
leftMotor = AngularServo(12, min_angle= 0, max_angle= 100)
rightMotor = AngularServo(13, min_angle= 0, max_angle = 100)

#Turret control
pitchServo = AngularServo(25 , min_angle=-90, max_angle=90)
yawServo = AngularServo(23, min_angle=-90, max_angle=90)

def setMotorSpeed(speed):
    leftMotor.angle = speed
    rightMotor.angle = speed

def reset():
    yawServo.angle = 0
    pitchServo.angle = 0
    indexServo.angle = 0
    setMotorSpeed(0)

if __name__ == '__main__':
    # rospy.init_node('listener', anonymous=True)
    # listen()
    # rospy.spin()

    reset()

    sleep(5)

    # move yaw
    yawServo.angle = -90
    sleep(5)
    yawServo.angle = 90
    sleep(5)

    # move pitch
    pitchServo.angle = 50
    sleep(5)
    pitchServo.angle = 0
    sleep(5)

    # Shoot and index    
    setMotorSpeed(50)

    sleep(5)

    indexServo.angle = 22

    sleep(5)

    indexServo.angle = 44

    sleep(5)

    indexServo.angle = 66

    sleep(5)

    # end program
    reset()