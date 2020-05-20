import RPi.GPIO as GPIO
import threading
from time import sleep
import cv2

motor_left_pin = 16
motor_right_pin = 22
motor_enable_pin = 18
calibration_button = 8
barrier_left_button = 10
barrier_right_button = 12

pwm = 0

Enc_A = 23
Enc_B = 24

Rotary_counter = 0
Current_A = 1
Current_B = 1

LockRotary = threading.Lock()
 

def gpio_init():
    GPIO.cleanup()
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)


def steering_module_init():
    global motor_left_pin, motor_right_pin, motor_enable_pin
    global calibration_button, barrier_left_button, barrier_right_button

    GPIO.setup(calibration_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(barrier_left_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(barrier_right_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(motor_left_pin, GPIO.OUT)
    GPIO.setup(motor_right_pin, GPIO.OUT)
    GPIO.setup(motor_enable_pin, GPIO.OUT)
    pwm = GPIO.PWM(motor_enable_pin, 100)
    
    
def encoder_init():
    GPIO.setup(Enc_A, GPIO.IN)
    GPIO.setup(Enc_B, GPIO.IN)
    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotary_interrupt)
    GPIO.add_event_detect(Enc_B, GPIO.RISING, callback=rotary_interrupt)


def rotary_interrupt(A_or_B):
    global Rotary_counter, Current_A, Current_B, LockRotary

    Switch_A = GPIO.input(Enc_A)
    Switch_B = GPIO.input(Enc_B)

    if Current_A == Switch_A and Current_B == Switch_B:
        return

    Current_A = Switch_A
    Current_B = Switch_B

    if (Switch_A and Switch_B):
        LockRotary.acquire()
        if A_or_B == Enc_B:
            Rotary_counter += 1
        else:
            Rotary_counter -= 1
        LockRotary.release()


def read_encoder():
    global Rotary_counter, LockRotary
    LockRotary.acquire()
    encoder_value = Rotary_counter
    LockRotary.release()
    return encoder_value


def turn_right(pwm_value):
    global motor_left_pin, motor_right_pin, pwm
    GPIO.output(motor_left_pin, GPIO.LOW)
    GPIO.output(motor_right_pin, GPIO.HIGH)
    pwm.start(pwm_value)


def turn_left(pwm_value):
    global motor_left_pin, motor_right_pin, pwm
    GPIO.output(motor_left_pin, GPIO.HIGH)
    GPIO.output(motor_right_pin, GPIO.LOW)
    pwm.start(pwm_value)


def stop():
    global motor_left_pin, motor_right_pin, pwm, motor_enable_pin
    GPIO.output(motor_left_pin, GPIO.LOW)
    GPIO.output(motor_right_pin, GPIO.LOW)
    pwm.stop()
    GPIO.output(motor_enable_pin, GPIO.LOW)
    GPIO.cleanup()

def  calibration ():
    global calibration_button, barrier_left_button, barrier_right_button
    current_enc_value = read_encoder()
    ret0 = 0 
    if ret0 == 0:
       ret0 = right_calibration()
       turn_right(50)
       if GPIO.input(barrier_right_button) == GPIO.HIGH:
          stop()
          right_calibration()
    if ret0 == 1:
       turn_left(50)
       if GPIO.input(barrier_left_button) == GPIO.HIGH:
          stop()
          left_calibration(speed)

def right_calibration():
    current_enc_value = read_encoder()
    right_max = open('right_max.txt', 'w', buffering=1)
    right_max.write("%d\r\n" % (current_enc_value))
    return 1 

def left_calibration(speed):
    current_enc_value = read_encoder()
    left_max = open('left_max.txt', 'w', buffering=1)
    left_max.write("%d\r\n" % (current_enc_value))
    sleep(5)

         



def steering_main(data, speed):
    current_enc_value = read_encoder()
    # print("current_enc_value: ", current_enc_value)
    # print("received_turn_value: ", data)
    # sleep(0.1)

    if data > 0:
        if data > current_enc_value:
            turn_right(speed)
            return 0, current_enc_value
        elif data == current_enc_value:
            stop()
            return 1, current_enc_value
        elif data < current_enc_value:
            turn_left(speed)
            return 0, current_enc_value
    elif data < 0:
        if data < current_enc_value:
            turn_left(speed)
            return 0, current_enc_value
        elif data == current_enc_value:
            stop()
            return 1, current_enc_value
        elif data > current_enc_value:
            turn_right(speed)
            return 0, current_enc_value
    else:
        return 0, current_enc_value



if __name__ == '__main__':

    img = cv2.imread("data/Moveonlogoblack800x800.jpg")
    img = cv2.resize(img, (200, 200))
    
    gpio_init()
    steering_module_init()
    encoder_init()

    ret = 0

    desired_turn_value = 398
    speed_ = 80

    while 1:
        if GPIO.input(calibration_button) == GPIO.HIGH:
           calibration ()

        cv2.imshow("img", img)
        k = cv2.waitKey(5) & 0xFF

        if k == 27:
          stop()
          cv2.destroyAllWindows()
          break

        ret, enc_value = steering_main(desired_turn_value, speed_)

        if ret == 1:
           print("{} 'e gitti".format(enc_value))
           sleep(5)

           desired_turn_value = -100
           speed_ = 20

        else:
            ret, enc_value = steering_main(desired_turn_value, speed_)
            print("{} 'e gidiyor... su anki deger: {}".format(desired_turn_value, enc_value))


