// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef _FOC_PROTOCOL_H
#define _FOC_PROTOCOL_H

#define START_FRAME 0xABCD
#define START_FRAME_IMU 0xACDC

typedef struct {
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

typedef struct {
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  wheelR_cnt;
   int16_t  wheelL_cnt; 
   int16_t  left_dc_curr;
   int16_t  right_dc_curr;   
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;

typedef struct {
   uint16_t start;  // START_FRAME_IMU=0xACDC
   uint16_t imuId;  // 0=imu0(Master board), 1=imu1(Slave board)
   int16_t  accelX;
   int16_t  accelY;
   int16_t  accelZ;
   int16_t  gyroX;
   int16_t  gyroY;
   int16_t  gyroZ;
   uint16_t checksum;
} SerialImu;

#endif
