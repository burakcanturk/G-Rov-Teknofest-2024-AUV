from serial import Serial
from time import sleep
import os
from swd import Swd
import struct

ESC_MIN = 1000
ESC_STOP = 1500
ESC_MAX = 2000

SEND_ADDRESS_ROW = 0
DEBUG_ON_ROW = 1
URF_VAL_ROW = 2
ULF_VAL_ROW = 3
URB_VAL_ROW = 4
ULB_VAL_ROW = 5
LRF_VAL_ROW = 6
LLF_VAL_ROW = 7
LRB_VAL_ROW = 8
LLB_VAL_ROW = 9
RELAY0_VAL_ROW = 10
RELAY1_VAL_ROW = 11
RELAY2_VAL_ROW = 12
RELAY3_VAL_ROW = 13
RELAY4_VAL_ROW = 14
ROLICAM_ANGLE_ROW = 15
ROLICAM_SPEED_ROW = 16
ROLICAM_DIM_ROW = 17
ROLICAM_RESET_ROW = 18
FILTER_RESET_ROW = 19

DIS_FRONT_VAL_ROW = 128
DIS_BACK_VAL_ROW = 129
DIS_RIGHT_VAL_ROW = 130
DIS_LEFT_VAL_ROW = 131
DIS_LOWER_VAL_ROW = 132
GYRO_ROLL_VAL_ROW = 133
GYRO_PITCH_VAL_ROW = 134
GYRO_YAW_VAL_ROW = 135
VOLTAGE_VAL_ROW = 136
CURRENT_VAL_ROW = 137
POWER_VAL_ROW = 138

rov_vals_incoming = {
    "upper_right_front_val": ESC_STOP,
    "upper_left_front_val": ESC_STOP,
    "upper_right_back_val": ESC_STOP,
    "upper_left_back_val": ESC_STOP,
    "lower_right_front_val": ESC_STOP,
    "lower_left_front_val": ESC_STOP,
    "lower_right_back_val": ESC_STOP,
    "lower_left_back_val": ESC_STOP,
    "relay0_val": 0,
    "relay1_val": 0,
    "relay2_val": 0,
    "relay3_val": 0,
    "relay4_val": 0,
    "rolicam_angle": 0,
    "rolicam_speed": 0,
    "rolicam_dim": 0,
    "rolicam_reset": 0,
    "filter_reset": 0
}

rov_vals_outgoing = {
  "dis_front": 0,
  "dis_back": 0,
  "dis_right": 0,
  "dis_left": 0,
  "dis_lower": 0,
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "voltage": 0.0,
  "current": 0.0,
  "power": 0.0
}

rovAddresses = {
    "debug_on": 0x00000000,
    "upper_right_front_val": 0x00000000,
    "upper_left_front_val": 0x00000000,
    "upper_right_back_val": 0x00000000,
    "upper_left_back_val": 0x00000000,
    "lower_right_front_val": 0x00000000,
    "lower_left_front_val": 0x00000000,
    "lower_right_back_val": 0x00000000,
    "lower_left_back_val": 0x00000000,
    "relay0_val": 0x00000000,
    "relay1_val": 0x00000000,
    "relay2_val": 0x00000000,
    "relay3_val": 0x00000000,
    "relay4_val": 0x00000000,
    "rolicam_angle": 0x00000000,
    "rolicam_speed": 0x00000000,
    "rolicam_dim": 0x00000000,
    "rolicam_reset": 0x00000000,
    "filter_reset": 0x00000000,
    "dis_front": 0x00000000,
    "dis_back": 0x00000000,
    "dis_right": 0x00000000,
    "dis_left": 0x00000000,
    "dis_lower": 0x00000000,
    "roll": 0x00000000,
    "pitch": 0x00000000,
    "yaw": 0x00000000,
    "voltage": 0x00000000,
    "current": 0x00000000,
    "power": 0x00000000
}

def constrain(val, min_, max_):
    if val < min_:
        return min_
    elif val > max_:
        return max_
    else:
        return val

class rovVehicleUart(Serial):

    def __init__(self, port, baud):
        self.__init__(port, baud)

    def getAddress(self, show = False):
        self.write(SEND_ADDRESS_ROW.to_bytes(1, "little"))
        sleep(0.1)
        for (k, v) in rovAddresses.items():
            deger = self.readline().decode().strip()
            rovAddresses[deger.split()[0]] = int(deger.split()[1])
        if show:
            for (k, v) in rovAddresses.items():
                print(f"{k}:\t{hex(v)}")

    def setDebugMode(self, val):
        rov_vals_incoming["debug_on"] = constrain(val, 0, 1)
        self.write(DEBUG_ON_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["debug_on"].to_bytes(1, "little"))
        self.read(1)

    def setUpperRightFrontMotor(self, val):
        rov_vals_incoming["upper_right_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(URF_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["upper_right_front_val"].to_bytes(2, "little"))
        self.read(1)

    def setUpperLeftFrontMotor(self, val):
        rov_vals_incoming["upper_left_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(ULF_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["upper_left_front_val"].to_bytes(2, "little"))
        self.read(1)

    def setUpperRightBackMotor(self, val):
        rov_vals_incoming["upper_right_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(URB_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["upper_right_back_val"].to_bytes(2, "little"))
        self.read(1)

    def setUpperLeftBackMotor(self, val):
        rov_vals_incoming["upper_left_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(ULB_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["upper_left_back_val"].to_bytes(2, "little"))
        self.read(1)

    def setLowerRightFrontMotor(self, val):
        rov_vals_incoming["lower_right_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(LRF_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["lower_right_front_val"].to_bytes(2, "little"))
        self.read(1)

    def setLowerLeftFrontMotor(self, val):
        rov_vals_incoming["lower_left_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(LLF_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["lower_left_front_val"].to_bytes(2, "little"))
        self.read(1)

    def setLowerRightBackMotor(self, val):
        rov_vals_incoming["lower_right_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(LRB_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["lower_right_back_val"].to_bytes(2, "little"))
        self.read(1)

    def setLowerLeftBackMotor(self, val):
        rov_vals_incoming["lower_left_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write(LLB_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["lower_left_back_val"].to_bytes(2, "little"))
        self.read(1)

    def setRelay0(self, val):
        rov_vals_incoming["relay0_val"] = constrain(val, 0, 1)
        self.write(RELAY0_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["relay0_val"].to_bytes(1, "little"))
        self.read(1)

    def setRelay1(self, val):
        rov_vals_incoming["relay1_val"] = constrain(val, 0, 1)
        self.write(RELAY1_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["relay1_val"].to_bytes(1, "little"))
        self.read(1)

    def setRelay2(self, val):
        rov_vals_incoming["relay2_val"] = constrain(val, 0, 1)
        self.write(RELAY2_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["relay2_val"].to_bytes(1, "little"))
        self.read(1)

    def setRelay3(self, val):
        rov_vals_incoming["relay3_val"] = constrain(val, 0, 1)
        self.write(RELAY3_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["relay3_val"].to_bytes(1, "little"))
        self.read(1)

    def setRelay4(self, val):
        rov_vals_incoming["relay4_val"] = constrain(val, 0, 1)
        self.write(RELAY4_VAL_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["relay4_val"].to_bytes(1, "little"))
        self.read(1)

    def setRolicamAngle(self, val):
        rov_vals_incoming["rolicam_angle"] = constrain(val, 0, 180)
        self.write(ROLICAM_ANGLE_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["rolicam_angle"].to_bytes(1, "little"))
        self.read(1)

    def setRolicamSpeed(self, val):
        rov_vals_incoming["rolicam_speed"] = constrain(val, 0, 100)
        self.write(ROLICAM_SPEED_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["rolicam_speed"].to_bytes(1, "little"))
        self.read(1)

    def setRolicamDim(self, val):
        rov_vals_incoming["rolicam_dim"] = constrain(val, 0, 100)
        self.write(ROLICAM_DIM_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["rolicam_dim"].to_bytes(1, "little"))
        self.read(1)

    def setRolicamReset(self, val):
        rov_vals_incoming["rolicam_reset"] = constrain(val, 0, 1)
        self.write(ROLICAM_RESET_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["rolicam_reset"].to_bytes(1, "little"))
        self.read(1)

    def setFilterReset(self, val):
        rov_vals_incoming["filter_reset"] = constrain(val, 0, 1)
        self.write(FILTER_RESET_ROW.to_bytes(1, "little"))
        self.write(rov_vals_incoming["filter_reset"].to_bytes(1, "little"))
        self.read(1)

    def stopLowerMotors(self):
        self.setLowerRightFrontMotor(ESC_STOP)
        self.setLowerLeftFrontMotor(ESC_STOP)
        self.setLowerRightBackMotor(ESC_STOP)
        self.setLowerLeftBackMotor(ESC_STOP)

    def stopUpperMotors(self):
        self.setUpperRightFrontMotor(ESC_STOP)
        self.setUpperLeftFrontMotor(ESC_STOP)
        self.setUpperRightBackMotor(ESC_STOP)
        self.setUpperLeftBackMotor(ESC_STOP)

    def stopAllMotors(self):
        self.stopLowerMotors()
        self.stopUpperMotors()

    def goForward(self, left_speed, right_speed):
        self.setLowerRightFrontMotor(ESC_STOP + constrain(right_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP + constrain(left_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP + constrain(right_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP + constrain(left_speed, -500, 500))

    def goBackward(self, left_speed, right_speed):
        self.setLowerRightFrontMotor(ESC_STOP - constrain(right_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP - constrain(left_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP - constrain(right_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP - constrain(left_speed, -500, 500))

    def turnRight(self, speed):
        self.setLowerRightFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP + constrain(speed, -500, 500))

    def turnLeft(self, speed):
        self.setLowerRightFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP - constrain(speed, -500, 500))

    def goRight(self, front_speed, back_speed):
        self.setLowerRightFrontMotor(ESC_STOP - constrain(front_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP + constrain(front_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP + constrain(back_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP - constrain(back_speed, -500, 500))

    def goLeft(self, front_speed, back_speed):
        self.setLowerRightFrontMotor(ESC_STOP + constrain(front_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP - constrain(front_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP - constrain(back_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP + constrain(back_speed, -500, 500))

    def goUp(self, speed):
        self.setUpperRightFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setUpperLeftFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setUpperRightBackMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setUpperLeftBackMotor(ESC_STOP + constrain(speed, -500, 500))

    def goDown(self, speed):
        self.setUpperRightFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setUpperLeftFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setUpperRightBackMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setUpperLeftBackMotor(ESC_STOP - constrain(speed, -500, 500))

    def getFrontDis(self):
        self.write(DIS_FRONT_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["dis_front"] = int.from_bytes(self.read(2), "little", signed = True)
        return rov_vals_outgoing["dis_front"]

    def getBackDis(self):
        self.write(DIS_BACK_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["dis_back"] = int.from_bytes(self.read(2), "little", signed = True)
        return rov_vals_outgoing["dis_back"]

    def getRightDis(self):
        self.write(DIS_RIGHT_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["dis_right"] = int.from_bytes(self.read(2), "little", signed = True)
        return rov_vals_outgoing["dis_right"]

    def getLeftDis(self):
        self.write(DIS_LEFT_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["dis_left"] = int.from_bytes(self.read(2), "little", signed = True)
        return rov_vals_outgoing["dis_left"]

    def getLowerDis(self):
        self.write(DIS_LOWER_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["dis_lower"] = int.from_bytes(self.read(2), "little", signed = True)
        return rov_vals_outgoing["dis_lower"]

    def getRoll(self):
        self.write(GYRO_ROLL_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["roll"] = round(struct.unpack("f", self.read(4))[0], 2)
        return rov_vals_outgoing["roll"]

    def getPitch(self):
        self.write(GYRO_PITCH_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["pitch"] = round(struct.unpack("f", self.read(4))[0], 2)
        return rov_vals_outgoing["pitch"]

    def getYaw(self):
        self.write(GYRO_YAW_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["yaw"] = round(struct.unpack("f", self.read(4))[0], 2)
        return rov_vals_outgoing["yaw"]

    def getVoltage(self):
        self.write(VOLTAGE_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["voltage"] = round(struct.unpack("f", self.read(4))[0], 2)
        return rov_vals_outgoing["voltage"]

    def getCurrent(self):
        self.write(CURRENT_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["current"] = round(struct.unpack("f", self.read(4))[0], 2)
        return rov_vals_outgoing["current"]

    def getPower(self):
        self.write(POWER_VAL_ROW.to_bytes(1, "little"))
        rov_vals_outgoing["power"] = round(struct.unpack("f", self.read(4))[0], 2)
        return rov_vals_outgoing["power"]

class rovVehicleSwd(Swd):

    def __init__(self):
        #self = Swd()
        super().__init__()
        os.system("st-flash reset")
        #print(dir(self))

    def debugOn(self):
        self.write_mem(rovAddresses["debug_on"], b"\x01")

    def debugOff(self):
        self.write_mem(rovAddresses["debug_on"], b"\x00")

    def setUpperRightFrontMotor(self, val):
        rov_vals_incoming["upper_right_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["upper_right_front_val"], rov_vals_incoming["upper_right_front_val"].to_bytes(2, "little"))

    def setUpperLeftFrontMotor(self, val):
        rov_vals_incoming["upper_left_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["upper_left_front_val"], rov_vals_incoming["upper_left_front_val"].to_bytes(2, "little"))

    def setUpperRightBackMotor(self, val):
        rov_vals_incoming["upper_right_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["upper_right_back_val"], rov_vals_incoming["upper_right_back_val"].to_bytes(2, "little"))

    def setUpperLeftBackMotor(self, val):
        rov_vals_incoming["upper_left_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["upper_left_back_val"], rov_vals_incoming["upper_left_back_val"].to_bytes(2, "little"))

    def setLowerRightFrontMotor(self, val):
        rov_vals_incoming["lower_right_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["lower_right_front_val"], rov_vals_incoming["lower_right_front_val"].to_bytes(2, "little"))

    def setLowerLeftFrontMotor(self, val):
        rov_vals_incoming["lower_left_front_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["lower_left_front_val"], rov_vals_incoming["lower_left_front_val"].to_bytes(2, "little"))

    def setLowerRightBackMotor(self, val):
        rov_vals_incoming["lower_right_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["lower_right_back_val"], rov_vals_incoming["lower_right_back_val"].to_bytes(2, "little"))

    def setLowerLeftBackMotor(self, val):
        rov_vals_incoming["lower_left_back_val"] = constrain(val, ESC_MIN, ESC_MAX)
        self.write_mem(rovAddresses["lower_left_back_val"], rov_vals_incoming["lower_left_back_val"].to_bytes(2, "little"))

    def setRelay0(self, val):
        rov_vals_incoming["relay0_val"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["relay0_val"], rov_vals_incoming["relay0_val"].to_bytes(1, "little"))

    def setRelay1(self, val):
        rov_vals_incoming["relay1_val"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["relay1_val"], rov_vals_incoming["relay1_val"].to_bytes(1, "little"))

    def setRelay2(self, val):
        rov_vals_incoming["relay2_val"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["relay2_val"], rov_vals_incoming["relay2_val"].to_bytes(1, "little"))

    def setRelay3(self, val):
        rov_vals_incoming["relay3_val"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["relay3_val"], rov_vals_incoming["relay3_val"].to_bytes(1, "little"))

    def setRelay4(self, val):
        rov_vals_incoming["relay4_val"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["relay4_val"], rov_vals_incoming["relay4_val"].to_bytes(1, "little"))

    def setRolicamAngle(self, val):
        rov_vals_incoming["rolicam_angle"] = constrain(val, 0, 180)
        self.write_mem(rovAddresses["rolicam_angle"], rov_vals_incoming["rolicam_angle"].to_bytes(1, "little"))

    def setRolicamSpeed(self, val):
        rov_vals_incoming["rolicam_speed"] = constrain(val, 0, 180)
        self.write_mem(rovAddresses["rolicam_speed"], rov_vals_incoming["rolicam_speed"].to_bytes(1, "little"))

    def setRolicamDim(self, val):
        rov_vals_incoming["rolicam_dim"] = constrain(val, 0, 100)
        self.write_mem(rovAddresses["rolicam_dim"], rov_vals_incoming["rolicam_dim"].to_bytes(1, "little"))

    def setRolicamReset(self, val):
        rov_vals_incoming["rolicam_reset"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["rolicam_reset"], rov_vals_incoming["rolicam_reset"].to_bytes(1, "little"))

    def setFilterReset(self, val):
        rov_vals_incoming["filter_reset"] = constrain(val, 0, 1)
        self.write_mem(rovAddresses["filter_reset"], rov_vals_incoming["filter_reset"].to_bytes(1, "little"))

    def stopLowerMotors(self):
        self.setLowerRightFrontMotor(ESC_STOP)
        self.setLowerLeftFrontMotor(ESC_STOP)
        self.setLowerRightBackMotor(ESC_STOP)
        self.setLowerLeftBackMotor(ESC_STOP)

    def stopUpperMotors(self):
        self.setUpperRightFrontMotor(ESC_STOP)
        self.setUpperLeftFrontMotor(ESC_STOP)
        self.setUpperRightBackMotor(ESC_STOP)
        self.setUpperLeftBackMotor(ESC_STOP)

    def stopAllMotors(self):
        self.stopLowerMotors()
        self.stopUpperMotors()

    def goForward(self, left_speed, right_speed):
        self.setLowerRightFrontMotor(ESC_STOP + constrain(right_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP + constrain(left_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP + constrain(right_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP + constrain(left_speed, -500, 500))

    def goBackward(self, left_speed, right_speed):
        self.setLowerRightFrontMotor(ESC_STOP - constrain(right_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP - constrain(left_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP - constrain(right_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP - constrain(left_speed, -500, 500))

    def turnRight(self, speed):
        self.setLowerRightFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP + constrain(speed, -500, 500))

    def turnLeft(self, speed):
        self.setLowerRightFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP - constrain(speed, -500, 500))

    def goRight(self, front_speed, back_speed):
        self.setLowerRightFrontMotor(ESC_STOP - constrain(front_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP + constrain(front_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP + constrain(back_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP - constrain(back_speed, -500, 500))

    def goLeft(self, front_speed, back_speed):
        self.setLowerRightFrontMotor(ESC_STOP + constrain(front_speed, -500, 500))
        self.setLowerLeftFrontMotor(ESC_STOP - constrain(front_speed, -500, 500))
        self.setLowerRightBackMotor(ESC_STOP - constrain(back_speed, -500, 500))
        self.setLowerLeftBackMotor(ESC_STOP + constrain(back_speed, -500, 500))

    def goUp(self, speed):
        self.setUpperRightFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setUpperLeftFrontMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setUpperRightBackMotor(ESC_STOP + constrain(speed, -500, 500))
        self.setUpperLeftBackMotor(ESC_STOP + constrain(speed, -500, 500))

    def goDown(self, speed):
        self.setUpperRightFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setUpperLeftFrontMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setUpperRightBackMotor(ESC_STOP - constrain(speed, -500, 500))
        self.setUpperLeftBackMotor(ESC_STOP - constrain(speed, -500, 500))

    def getFrontDis(self):
        rov_vals_outgoing["dis_front"] = int.from_bytes(bytes(self.read_mem(rovAddresses["dis_front"], 2)), "little", signed = True)
        return rov_vals_outgoing["dis_front"]

    def getBackDis(self):
        rov_vals_outgoing["dis_back"] = int.from_bytes(bytes(self.read_mem(rovAddresses["dis_back"], 2)), "little", signed = True)
        return rov_vals_outgoing["dis_back"]

    def getRightDis(self):
        rov_vals_outgoing["dis_right"] = int.from_bytes(bytes(self.read_mem(rovAddresses["dis_right"], 2)), "little", signed = True)
        return rov_vals_outgoing["dis_right"]

    def getLeftDis(self):
        rov_vals_outgoing["dis_left"] = int.from_bytes(bytes(self.read_mem(rovAddresses["dis_left"], 2)), "little", signed = True)
        return rov_vals_outgoing["dis_left"]

    def getLowerDis(self):
        rov_vals_outgoing["dis_lower"] = int.from_bytes(bytes(self.read_mem(rovAddresses["dis_lower"], 2)), "little", signed = True)
        return rov_vals_outgoing["dis_lower"]

    def getRoll(self):
        rov_vals_outgoing["roll"] = round(struct.unpack("f", bytes(self.read_mem(rovAddresses["roll"], 4)))[0], 2)
        return rov_vals_outgoing["roll"]

    def getPitch(self):
        rov_vals_outgoing["pitch"] = round(struct.unpack("f", bytes(self.read_mem(rovAddresses["pitch"], 4)))[0], 2)
        return rov_vals_outgoing["pitch"]

    def getYaw(self):
        rov_vals_outgoing["yaw"] = round(struct.unpack("f", bytes(self.read_mem(rovAddresses["yaw"], 4)))[0], 2)
        return rov_vals_outgoing["yaw"]

    def getVoltage(self):
        rov_vals_outgoing["voltage"] = round(struct.unpack("f", bytes(self.read_mem(rovAddresses["voltage"], 4)))[0], 2)
        return rov_vals_outgoing["voltage"]

    def getCurrent(self):
        rov_vals_outgoing["current"] = round(struct.unpack("f", bytes(self.read_mem(rovAddresses["current"], 4)))[0], 2)
        return rov_vals_outgoing["current"]

    def getPower(self):
        rov_vals_outgoing["power"] = round(struct.unpack("f", bytes(self.read_mem(rovAddresses["power"], 4)))[0], 2)
        return rov_vals_outgoing["power"]

if __name__ == "__main__":

    port = "/dev/ttyUSB0"
    baud = 115200

    grovUart = rovVehicleUart(port, baud)
    #sleep(3)
    grovUart.adresAl()

    grovSwd = rovVehicleSwd()
    grovSwd.debugOn()

    grovUart.stopAllMotors()

    print("Sistem baslatildi.")
