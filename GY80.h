
#ifndef GY80_h
#define GY80_h
#include <Arduino.h>
#include <Wire.h>
///////////Gyro registers//////////
#define GYR_ADDRESS (0xD2 >> 1)
#define GYRO_Samples 5
#define L3G4200D_WHO_AM_I      0x0F
#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27
#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D
#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F
#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_THS_XH   0x32
#define L3G4200D_INT1_THS_XL   0x33
#define L3G4200D_INT1_THS_YH   0x34
#define L3G4200D_INT1_THS_YL   0x35
#define L3G4200D_INT1_THS_ZH   0x36
#define L3G4200D_INT1_THS_ZL   0x37
#define L3G4200D_INT1_DURATION 0x38
//////////////////////////////////////

/////////Accelero registers/////////
#define ADXAddress 0xA7>>1
#define Register_ID 0
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37
/////////////////////////////////////

////////Magneto registers/////////
#define HMC5883L_Address 0x1E
#define HMC5883L_CFGR_A 0x00 
#define HMC5883L_CFGR_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_OUT_X_H 0x03
//////////////////////////////////

class GY80
{
	public:
		GY80(void);
		float xAccel, yAccel, zAccel;
		typedef struct vector
		{
			float x, y, z;
		} vector;
		
		typedef struct dof
		{
			float yaw, pitch, roll;
		} dof;
		
		dof rotations;
		
		vector gyro, acc, mag;
		
		void init_acc(void);
		void init_gyro(void);
		void init_mag(void);
		
		void read_acc(void);
		void read_gyro(void);
		void read_mag(void);
		
		void getAccoffset(void);
		void calcRotations(void);
		
		bool getRotation(float gravity[], float geomagnetic[]);
	private:
		float _gx, _gy, _gz, _gyro_sumX, _gyro_sumY, _gyro_sumZ;
		bool useCalibrate;
		float dpsPerDigit;
		int X0,X1,Y0,Y1,Z1,Z0;
		uint8_t xla,xha,yla,yha,zla,zha;
		int magY, magX, magZ;

		void writeReg(int address, uint8_t reg, uint8_t value);
		uint8_t readReg(int address, uint8_t reg);
		
		
};

#endif