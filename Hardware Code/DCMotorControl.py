from gpiozero import OutputDevice
from gpiozero import PWMOutputDevice
from gpiozero import DigitalInputDevice
from gpiozero import SmoothedInputDevice
from time import sleep
# import matplotlib.pyplot as plt

class FIT_Motor:
    def __init__(self, in1, in2, in3):
        self.direction = OutputDevice(in1, initial_value=True)  # direction Pin
        self.speed = PWMOutputDevice(in2, initial_value=1, frequency=980)      # speed Pin
        self.encoder = DigitalInputDevice(in3)
        #self.encoder = SmoothedInputDevice(in3, threshold=0.01, partial=True)
        self.total_time = 1.0/(self.speed.frequency/2.0)

    def changeDirection(self):
        self.direction.toggle()

    def setStop(self):
        self.speed.on()

    def setSpeed(self, speed):
        speed_val = abs(speed) / 20.8
        if speed_val < 0.01:
            self.setStop()
        else:
            self.speed.blink(on_time=self.total_time-speed_val*self.total_time, off_time=self.total_time*speed_val)

    def readEncoder(self, val):
        # print("Nothing")
        reading = self.encoder.value
        return reading
        # tot_time = 0
        # for i in range(8):
        #     if val == 1:
        #         meas_time = self.encoder.active_time
        #     if val == 0:
        #         meas_time = self.encoder.inactive_time
        #     if meas_time == None:
        #         meas_time = 0
        #     tot_time += meas_time
        # if tot_time == 0:
        #     return 0
        # else:
        #     return (60.0 / (45.0*6.0*2.0*tot_time)) / 8.0

def main():
    DC_Motor = FIT_Motor(16,12,10)     # change pins accordingly, use GPIO #

    #data_log = open("encoder_data.txt", "w")

    # plot_x = []
    # plot_y = []

    raw_input("Press any key to begin")
    DC_Motor.setSpeed(20.8)
    #DC_Motor.update()
    print("half speed")
    for i in range(2000):
        sleep(0.001)
        #plot_x.append(i)
        #plot_y.append(DC_Motor.readEncoder(1))
        #print(DC_Motor.readEncoder(1) * 7.85 / 60.0)
        reading = DC_Motor.readEncoder(1)
        print(reading)
        #data_log.write(str(reading))
        #data_log.write("\n")

    # plt.figure(1)
    # plt.clf()    
    # plt.plot(plot_x, plot_y, 'r-')
    # plt.xlim((0,200))

    DC_Motor.setStop()
    #DC_Motor.update()
    print("stahp")
    sleep(2)

    #data_log.write("\n")

    # plot_x = []
    # plot_y = []

    DC_Motor.changeDirection()
    DC_Motor.setSpeed(2.9)
    #DC_Motor.update()
    print("change dir")
    for i in range(2000):
        sleep(0.001)
        #plot_x.append(i)
        #plot_y.append(DC_Motor.readEncoder(1))
        #print(DC_Motor.readEncoder(1) * 7.85 / 60.0)
        reading = DC_Motor.readEncoder(1)
        print(reading)
        #data_log.write(str(reading))
        #data_log.write("\n")

    DC_Motor.setStop()
    print("stop")

    # plt.figure(2)
    # plt.clf()
    # plt.plot(plot_x, plot_y, 'r-')
    # plt.xlim((0,200))

    #data_log.close()

    raw_input("Press any key to exit")

    # plt.show()

if __name__ == "__main__":
    main()