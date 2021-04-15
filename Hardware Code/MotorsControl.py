from StepperControl import Stepper_28BYJ_48
from DCMotorControl import FIT_Motor
from time import sleep


def main():
    dc_motor = FIT_Motor(16,12,10)
    stepper = Stepper_28BYJ_48(4,17,27,22) # change pins accordingly, use GPIO #
    stepper._update()

    raw_input("Press any key to begin")

    dc_motor.setSpeed(10.4)

    print("Turning to the right")
    for i in range(0, 500):
        stepper.step()
        sleep(0.001)

    dc_motor.setSpeed(5.2)
    sleep(1)

    print("Turning back to main position")
    stepper.changeDirection()
    for i in range(0, 500):
        stepper.step()
        sleep(0.001)
    
    sleep(1)
    dc_motor.setStop()

    raw_input("Press any key to end")

if __name__ == "__main__":
    main()