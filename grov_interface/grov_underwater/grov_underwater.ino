#include <Servo.h>
#include <SerialTransfer.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
#include <Base.h>

#define upper_right_front_esc_pin PB12
#define upper_left_front_esc_pin PB13
#define upper_right_back_esc_pin PB14
#define upper_left_back_esc_pin PB15
#define lower_right_front_esc_pin PA8
#define lower_left_front_esc_pin PA11
#define lower_right_back_esc_pin PA12
#define lower_left_back_esc_pin PA15

#define relay0_pin PB3
#define relay1_pin PB4
#define relay2_pin PB5
#define relay3_pin PB8
#define relay4_pin PB9

#define dis_sw0_pin PA6
#define dis_sw1_pin PA5
#define dis_sw2_pin PA4

#define ESC_MIN 1000
#define ESC_STOP 1500
#define ESC_MAX 2000

#define SEND_ADDRESS_ROW 0
#define DEBUG_ON_ROW 1
#define URF_VAL_ROW 2
#define ULF_VAL_ROW 3
#define URB_VAL_ROW 4
#define ULB_VAL_ROW 5
#define LRF_VAL_ROW 6
#define LLF_VAL_ROW 7
#define LRB_VAL_ROW 8
#define LLB_VAL_ROW 9
#define RELAY0_VAL_ROW 10
#define RELAY1_VAL_ROW 11
#define RELAY2_VAL_ROW 12
#define RELAY3_VAL_ROW 13
#define RELAY4_VAL_ROW 14
#define ROLICAM_ANGLE_ROW 15
#define ROLICAM_SPEED_ROW 16
#define ROLICAM_DIM_ROW 17
#define ROLICAM_RESET_ROW 18
#define FILTER_RESET_ROW 19

#define DIS_FRONT_VAL_ROW 128
#define DIS_BACK_VAL_ROW 129
#define DIS_RIGHT_MUX_VAL_ROW 130
#define DIS_LEFT_MUX_VAL_ROW 131
#define DIS_LOWER_VAL_ROW 132
#define GYRO_ROLL_VAL_ROW 133
#define GYRO_PITCH_VAL_ROW 134
#define GYRO_YAW_VAL_ROW 135
#define VOLTAGE_VAL_ROW 136
#define CURRENT_VAL_ROW 137
#define POWER_VAL_ROW 138

#define dis_front_mux 0
#define dis_back_mux 1
#define dis_right_mux 2
#define dis_left_mux 3
#define dis_lower_mux 4

HardwareSerial ser(USART1);
HardwareSerial dis(USART2);
HardwareSerial rolicam(USART3);

Servo lower_right_front_esc;
Servo lower_left_front_esc;
Servo lower_right_back_esc;
Servo lower_left_back_esc;
Servo upper_right_front_esc;
Servo upper_left_front_esc;
Servo upper_right_back_esc;
Servo upper_left_back_esc;

MPU6050 mpu6050(Wire);
Madgwick filter;

Base base;

SerialTransfer serialTransfer;

struct __attribute__((packed)) RoliCamPacket {
  int angle;
  int speed;
  int reset;
  int dim;
} roliCamVals;

byte val_row;

struct rovValsIncoming {
  bool debug_on;
  uint16_t upper_right_front_val;
  uint16_t upper_left_front_val;
  uint16_t upper_right_back_val;
  uint16_t upper_left_back_val;
  uint16_t lower_right_front_val;
  uint16_t lower_left_front_val;
  uint16_t lower_right_back_val;
  uint16_t lower_left_back_val;
  bool relay0_val;
  bool relay1_val;
  bool relay2_val;
  bool relay3_val;
  bool relay4_val;
  uint8_t rolicam_angle = 0;
  uint8_t rolicam_speed = 100;
  uint8_t rolicam_dim = 0;
  bool rolicam_reset = 0;
  bool filter_reset = 0;
} rov_vals_incoming;

struct rovValsOutgoing {
  int16_t dis_front;
  int16_t dis_back;
  int16_t dis_right;
  int16_t dis_left;
  int16_t dis_lower;
  float roll;
  float pitch;
  float yaw;
  float voltage;
  float current;
  float power;
} rov_vals_outgoing;

bool debug_on = false;

struct gyroVals {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} gyro_vals;

void SetRoliCam(int targetAngle, int targetDim, int targetSpeed, int targetReset);
void disOpen(int num);
int16_t getDis(int num);
void readGyro();
float readRoll();
float readPitch();
float readYaw();

void setup() {

  ser.begin(115200);

  dis.begin(115200);

  rolicam.begin(9600);
  serialTransfer.begin(rolicam);

  pinMode(relay0_pin, OUTPUT);
  pinMode(relay1_pin, OUTPUT);
  pinMode(relay2_pin, OUTPUT);
  pinMode(relay3_pin, OUTPUT);
  pinMode(relay4_pin, OUTPUT);

  digitalWrite(relay0_pin, HIGH);
  digitalWrite(relay1_pin, HIGH);
  digitalWrite(relay2_pin, HIGH);
  digitalWrite(relay3_pin, HIGH);
  digitalWrite(relay4_pin, HIGH);

  pinMode(dis_sw0_pin, OUTPUT);
  pinMode(dis_sw1_pin, OUTPUT);
  pinMode(dis_sw2_pin, OUTPUT);

  upper_right_front_esc.attach(upper_right_front_esc_pin, ESC_MIN, ESC_MAX);
  upper_left_front_esc.attach(upper_left_front_esc_pin, ESC_MIN, ESC_MAX);
  upper_right_back_esc.attach(upper_right_back_esc_pin, ESC_MIN, ESC_MAX);
  upper_left_back_esc.attach(upper_left_back_esc_pin, ESC_MIN, ESC_MAX);
  lower_right_front_esc.attach(lower_right_front_esc_pin, ESC_MIN, ESC_MAX);
  lower_left_front_esc.attach(lower_left_front_esc_pin, ESC_MIN, ESC_MAX);
  lower_right_back_esc.attach(lower_right_back_esc_pin, ESC_MIN, ESC_MAX);
  lower_left_back_esc.attach(lower_left_back_esc_pin, ESC_MIN, ESC_MAX);

  upper_right_front_esc.writeMicroseconds(ESC_STOP);
  upper_left_front_esc.writeMicroseconds(ESC_STOP);
  upper_right_back_esc.writeMicroseconds(ESC_STOP);
  upper_left_back_esc.writeMicroseconds(ESC_STOP);
  lower_right_front_esc.writeMicroseconds(ESC_STOP);
  lower_left_front_esc.writeMicroseconds(ESC_STOP);
  lower_right_back_esc.writeMicroseconds(ESC_STOP);
  lower_left_back_esc.writeMicroseconds(ESC_STOP);

  base.begin();
  base.currentOffset(-1.48);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
  filter.begin(25);

  //delay(5000);

  //ser.println("Sistem baslatildi.");
}

void loop() {

  /*int16_t dis_on = getDis(0, mesafe_on_tx_pin);
    int16_t dis_arka = getDis(1, mesafe_arka_tx_pin);
    int16_t dis_sag = getDis(2, mesafe_sag_tx_pin);
    int16_t dis_sol = getDis(3, mesafe_sol_tx_pin);
    int16_t dis_asagi = getDis(4, mesafe_asagi_tx_pin);

    ser.print("On: ");
    ser.println(dis_on);
    ser.print("Arka: ");
    ser.println(dis_arka);
    ser.print("Sag: ");
    ser.println(dis_sag);
    ser.print("Sol: ");
    ser.println(dis_sol);
    ser.print("Asagi: ");
    ser.println(dis_asagi);

    ser.println("---------------------------------------------------------");*/

  if (ser.available() > 0) {

    val_row = ser.read();

    switch (val_row) {

      case SEND_ADDRESS_ROW:
        ser.print("debug_on ");
        ser.print((uint32_t)(&debug_on));
        ser.print("\n");
        ser.print("upper_right_front_val ");
        ser.print((uint32_t)(&rov_vals_incoming.upper_right_front_val));
        ser.print("\n");
        ser.print("upper_left_front_val ");
        ser.print((uint32_t)(&rov_vals_incoming.upper_left_front_val));
        ser.print("\n");
        ser.print("upper_right_back_val ");
        ser.print((uint32_t)(&rov_vals_incoming.upper_right_back_val));
        ser.print("\n");
        ser.print("upper_left_back_val ");
        ser.print((uint32_t)(&rov_vals_incoming.upper_left_back_val));
        ser.print("\n");
        ser.print("lower_right_front_val ");
        ser.print((uint32_t)(&rov_vals_incoming.lower_right_front_val));
        ser.print("\n");
        ser.print("lower_left_front_val ");
        ser.print((uint32_t)(&rov_vals_incoming.lower_left_front_val));
        ser.print("\n");
        ser.print("lower_right_back_val ");
        ser.print((uint32_t)(&rov_vals_incoming.lower_right_back_val));
        ser.print("\n");
        ser.print("lower_left_back_val ");
        ser.print((uint32_t)(&rov_vals_incoming.lower_left_back_val));
        ser.print("\n");
        ser.print("relay0_val ");
        ser.print((uint32_t)(&rov_vals_incoming.relay0_val));
        ser.print("\n");
        ser.print("relay1_val ");
        ser.print((uint32_t)(&rov_vals_incoming.relay1_val));
        ser.print("\n");
        ser.print("relay2_val ");
        ser.print((uint32_t)(&rov_vals_incoming.relay2_val));
        ser.print("\n");
        ser.print("relay3_val ");
        ser.print((uint32_t)(&rov_vals_incoming.relay3_val));
        ser.print("\n");
        ser.print("relay4_val ");
        ser.print((uint32_t)(&rov_vals_incoming.relay4_val));
        ser.print("\n");
        ser.print("rolicam_angle ");
        ser.print((uint32_t)(&rov_vals_incoming.rolicam_angle));
        ser.print("\n");
        ser.print("rolicam_speed ");
        ser.print((uint32_t)(&rov_vals_incoming.rolicam_speed));
        ser.print("\n");
        ser.print("rolicam_dim ");
        ser.print((uint32_t)(&rov_vals_incoming.rolicam_dim));
        ser.print("\n");
        ser.print("rolicam_reset ");
        ser.print((uint32_t)(&rov_vals_incoming.rolicam_reset));
        ser.print("\n");
        ser.print("filter_reset ");
        ser.print((uint32_t)(&rov_vals_incoming.filter_reset));
        ser.print("\n");
        ser.print("dis_front ");
        ser.print((uint32_t)(&rov_vals_outgoing.dis_front));
        ser.print("\n");
        ser.print("dis_back ");
        ser.print((uint32_t)(&rov_vals_outgoing.dis_back));
        ser.print("\n");
        ser.print("dis_right ");
        ser.print((uint32_t)(&rov_vals_outgoing.dis_right));
        ser.print("\n");
        ser.print("dis_left ");
        ser.print((uint32_t)(&rov_vals_outgoing.dis_left));
        ser.print("\n");
        ser.print("dis_lower ");
        ser.print((uint32_t)(&rov_vals_outgoing.dis_lower));
        ser.print("\n");
        ser.print("roll ");
        ser.print((uint32_t)(&rov_vals_outgoing.roll));
        ser.print("\n");
        ser.print("pitch ");
        ser.print((uint32_t)(&rov_vals_outgoing.pitch));
        ser.print("\n");
        ser.print("yaw ");
        ser.print((uint32_t)(&rov_vals_outgoing.yaw));
        ser.print("\n");
        ser.print("voltage ");
        ser.print((uint32_t)(&rov_vals_outgoing.voltage));
        ser.print("\n");
        ser.print("current ");
        ser.print((uint32_t)(&rov_vals_outgoing.current));
        ser.print("\n");
        ser.print("power ");
        ser.print((uint32_t)(&rov_vals_outgoing.power));
        ser.print("\n");
        break;

      case DEBUG_ON_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.debug_on, sizeof(rov_vals_incoming.debug_on));
        debug_on = rov_vals_incoming.debug_on;
        ser.write('+');
        break;

      case URF_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.upper_right_front_val, sizeof(rov_vals_incoming.upper_right_front_val));
        upper_right_front_esc.writeMicroseconds(rov_vals_incoming.upper_right_front_val);
        ser.write('+');
        break;

      case ULF_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.upper_left_front_val, sizeof(rov_vals_incoming.upper_left_front_val));
        upper_left_front_esc.writeMicroseconds(rov_vals_incoming.upper_left_front_val);
        ser.write('+');
        break;

      case URB_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.upper_right_back_val, sizeof(rov_vals_incoming.upper_right_back_val));
        upper_right_back_esc.writeMicroseconds(rov_vals_incoming.upper_right_back_val);
        ser.write('+');
        break;

      case ULB_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.upper_left_back_val, sizeof(rov_vals_incoming.upper_left_back_val));
        upper_left_back_esc.writeMicroseconds(rov_vals_incoming.upper_left_back_val);
        ser.write('+');
        break;

      case LRF_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.lower_right_front_val, sizeof(rov_vals_incoming.lower_right_front_val));
        lower_right_front_esc.writeMicroseconds(rov_vals_incoming.lower_right_front_val);
        ser.write('+');
        break;

      case LLF_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.lower_left_front_val, sizeof(rov_vals_incoming.lower_left_front_val));
        lower_left_front_esc.writeMicroseconds(rov_vals_incoming.lower_left_front_val);
        ser.write('+');
        break;

      case LRB_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.lower_right_back_val, sizeof(rov_vals_incoming.lower_right_back_val));
        lower_right_back_esc.writeMicroseconds(rov_vals_incoming.lower_right_back_val);
        ser.write('+');
        break;

      case LLB_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.lower_left_back_val, sizeof(rov_vals_incoming.lower_left_back_val));
        lower_left_back_esc.writeMicroseconds(rov_vals_incoming.lower_left_back_val);
        ser.write('+');
        break;

      case RELAY0_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.relay0_val, sizeof(rov_vals_incoming.relay0_val));
        digitalWrite(relay0_pin, not rov_vals_incoming.relay0_val);
        ser.write('+');
        break;

      case RELAY1_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.relay1_val, sizeof(rov_vals_incoming.relay1_val));
        digitalWrite(relay1_pin, not rov_vals_incoming.relay1_val);
        ser.write('+');
        break;

      case RELAY2_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.relay2_val, sizeof(rov_vals_incoming.relay2_val));
        digitalWrite(relay2_pin, not rov_vals_incoming.relay2_val);
        ser.write('+');
        break;

      case RELAY3_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.relay3_val, sizeof(rov_vals_incoming.relay3_val));
        digitalWrite(relay3_pin, not rov_vals_incoming.relay3_val);
        ser.write('+');
        break;

      case RELAY4_VAL_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.relay4_val, sizeof(rov_vals_incoming.relay4_val));
        digitalWrite(relay4_pin, not rov_vals_incoming.relay4_val);
        ser.write('+');
        break;

      case ROLICAM_ANGLE_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.rolicam_angle, sizeof(rov_vals_incoming.rolicam_angle));
        SetRoliCam(rov_vals_incoming.rolicam_angle,
                   rov_vals_incoming.rolicam_dim,
                   rov_vals_incoming.rolicam_speed,
                   rov_vals_incoming.rolicam_reset);
        ser.write('+');
        break;

      case ROLICAM_SPEED_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.rolicam_speed, sizeof(rov_vals_incoming.rolicam_speed));
        SetRoliCam(rov_vals_incoming.rolicam_angle,
                   rov_vals_incoming.rolicam_dim,
                   rov_vals_incoming.rolicam_speed,
                   rov_vals_incoming.rolicam_reset);
        ser.write('+');
        break;

      case ROLICAM_DIM_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.rolicam_dim, sizeof(rov_vals_incoming.rolicam_dim));
        SetRoliCam(rov_vals_incoming.rolicam_angle,
                   rov_vals_incoming.rolicam_dim,
                   rov_vals_incoming.rolicam_speed,
                   rov_vals_incoming.rolicam_reset);
        ser.write('+');
        break;

      case ROLICAM_RESET_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.rolicam_reset, sizeof(rov_vals_incoming.rolicam_reset));
        SetRoliCam(rov_vals_incoming.rolicam_angle,
                   rov_vals_incoming.rolicam_dim,
                   rov_vals_incoming.rolicam_speed,
                   rov_vals_incoming.rolicam_reset);
        ser.write('+');
        break;

      case FILTER_RESET_ROW:
        ser.readBytes((uint8_t*)&rov_vals_incoming.filter_reset, sizeof(rov_vals_incoming.filter_reset));
        if (rov_vals_incoming.filter_reset) filter = Madgwick();
        ser.write('+');
        break;

      //---------------------------------------------------------------

      case DIS_FRONT_VAL_ROW:
        rov_vals_outgoing.dis_front = getDis(dis_front_mux);
        ser.write((uint8_t*)&rov_vals_outgoing.dis_front, sizeof(rov_vals_outgoing.dis_front));
        break;

      case DIS_BACK_VAL_ROW:
        rov_vals_outgoing.dis_back = getDis(dis_back_mux);
        ser.write((uint8_t*)&rov_vals_outgoing.dis_back, sizeof(rov_vals_outgoing.dis_back));
        break;

      case DIS_RIGHT_MUX_VAL_ROW:
        rov_vals_outgoing.dis_right = getDis(dis_right_mux);
        ser.write((uint8_t*)&rov_vals_outgoing.dis_right, sizeof(rov_vals_outgoing.dis_right));
        break;

      case DIS_LEFT_MUX_VAL_ROW:
        rov_vals_outgoing.dis_left = getDis(dis_left_mux);
        ser.write((uint8_t*)&rov_vals_outgoing.dis_left, sizeof(rov_vals_outgoing.dis_left));
        break;

      case DIS_LOWER_VAL_ROW:
        rov_vals_outgoing.dis_lower = getDis(dis_lower_mux);
        ser.write((uint8_t*)&rov_vals_outgoing.dis_lower, sizeof(rov_vals_outgoing.dis_lower));
        break;

      case GYRO_ROLL_VAL_ROW:
        rov_vals_outgoing.roll = readRoll();
        ser.write((uint8_t*)&rov_vals_outgoing.roll, sizeof(rov_vals_outgoing.roll));
        break;

      case GYRO_PITCH_VAL_ROW:
        rov_vals_outgoing.pitch = readPitch();
        ser.write((uint8_t*)&rov_vals_outgoing.pitch, sizeof(rov_vals_outgoing.pitch));
        break;

      case GYRO_YAW_VAL_ROW:
        rov_vals_outgoing.yaw = readYaw();
        ser.write((uint8_t*)&rov_vals_outgoing.yaw, sizeof(rov_vals_outgoing.yaw));
        break;

      case VOLTAGE_VAL_ROW:
        rov_vals_outgoing.voltage = base.busVoltage();
        ser.write((uint8_t*)&rov_vals_outgoing.voltage, sizeof(rov_vals_outgoing.voltage));
        break;

      case CURRENT_VAL_ROW:
        rov_vals_outgoing.current = base.shuntCurrent();
        ser.write((uint8_t*)&rov_vals_outgoing.current, sizeof(rov_vals_outgoing.current));
        break;

      case POWER_VAL_ROW:
        rov_vals_outgoing.power = base.busPower();
        ser.write((uint8_t*)&rov_vals_outgoing.power, sizeof(rov_vals_outgoing.power));
        break;
    }
  }

  if (debug_on) {
    upper_right_front_esc.writeMicroseconds(rov_vals_incoming.upper_right_front_val);
    upper_left_front_esc.writeMicroseconds(rov_vals_incoming.upper_left_front_val);
    upper_right_back_esc.writeMicroseconds(rov_vals_incoming.upper_right_back_val);
    upper_left_back_esc.writeMicroseconds(rov_vals_incoming.upper_left_back_val);
    lower_right_front_esc.writeMicroseconds(rov_vals_incoming.lower_right_front_val);
    lower_left_front_esc.writeMicroseconds(rov_vals_incoming.lower_left_front_val);
    lower_right_back_esc.writeMicroseconds(rov_vals_incoming.lower_right_back_val);
    lower_left_back_esc.writeMicroseconds(rov_vals_incoming.lower_left_back_val);
    digitalWrite(relay0_pin, not rov_vals_incoming.relay0_val);
    digitalWrite(relay1_pin, not rov_vals_incoming.relay1_val);
    digitalWrite(relay2_pin, not rov_vals_incoming.relay2_val);
    digitalWrite(relay3_pin, not rov_vals_incoming.relay3_val);
    digitalWrite(relay4_pin, not rov_vals_incoming.relay4_val);
    SetRoliCam(rov_vals_incoming.rolicam_angle,
               rov_vals_incoming.rolicam_dim,
               rov_vals_incoming.rolicam_speed,
               rov_vals_incoming.rolicam_reset);
    if (rov_vals_incoming.filter_reset) filter = Madgwick();
    rov_vals_outgoing.dis_front = getDis(dis_front_mux);
    rov_vals_outgoing.dis_back = getDis(dis_back_mux);
    rov_vals_outgoing.dis_right = getDis(dis_right_mux);
    rov_vals_outgoing.dis_left = getDis(dis_left_mux);
    rov_vals_outgoing.dis_lower = getDis(dis_lower_mux);
    rov_vals_outgoing.roll = readRoll();
    rov_vals_outgoing.pitch = readPitch();
    rov_vals_outgoing.yaw = readYaw();
    rov_vals_outgoing.voltage = base.busVoltage();
    rov_vals_outgoing.current = base.shuntCurrent();
    rov_vals_outgoing.power = base.busPower();
  }
}

void SetRoliCam(int targetAngle, int targetDim, int targetSpeed, int targetReset) {

  roliCamVals.angle = constrain(targetAngle, 0, 180);
  roliCamVals.dim = constrain(targetDim, 0, 100);
  roliCamVals.speed = constrain(targetSpeed, 0, 100);
  roliCamVals.reset = constrain(targetReset, 0, 1);

  uint16_t sendSize = 0;
  sendSize = serialTransfer.txObj(roliCamVals, sendSize);
  serialTransfer.sendData(sendSize);
}

void disOpen(int num) {
  num = constrain(num, 0, 7);
  digitalWrite(dis_sw0_pin, (num >> 0) & 1);
  digitalWrite(dis_sw1_pin, (num >> 1) & 1);
  digitalWrite(dis_sw2_pin, (num >> 2) & 1);
}
int16_t getDis(int num) {

  int16_t distance = -3;

  disOpen(num);

  unsigned char buffer_RTT[4] = {0};
  uint8_t CS;

  while (dis.available() > 0) dis.read();

  dis.write('+');
  delay(20);

  if (dis.available() > 0) {
    //delay(4);
    if (dis.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = dis.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        //return distance;
      }
      else distance = -1;
      //else return -1;
    }
    else distance = -2;
    //else return -2;
  }
  else distance = -3;
  //else return -3;

  return distance;
}

void readGyro() {
  mpu6050.update();
  gyro_vals.ax = mpu6050.getAccX();
  gyro_vals.ay = mpu6050.getAccY();
  gyro_vals.az = mpu6050.getAccZ();
  gyro_vals.gx = mpu6050.getGyroX();
  gyro_vals.gy = mpu6050.getGyroY();
  gyro_vals.gz = mpu6050.getGyroZ();
}

float readRoll() {
  readGyro();
  filter.updateIMU(gyro_vals.gx,
                   gyro_vals.gy,
                   gyro_vals.gz,
                   gyro_vals.ax,
                   gyro_vals.ay,
                   gyro_vals.az);
  return filter.getRoll();
}

float readPitch() {
  readGyro();
  filter.updateIMU(gyro_vals.gx,
                   gyro_vals.gy,
                   gyro_vals.gz,
                   gyro_vals.ax,
                   gyro_vals.ay,
                   gyro_vals.az);
  return filter.getPitch();
}

float readYaw() {
  readGyro();
  filter.updateIMU(gyro_vals.gx,
                   gyro_vals.gy,
                   gyro_vals.gz,
                   gyro_vals.ax,
                   gyro_vals.ay,
                   gyro_vals.az);
  return filter.getYaw();
}
