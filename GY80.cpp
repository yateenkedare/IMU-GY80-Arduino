#include <GY80.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

GY80::GY80(void)	{
}

void GY80::writeReg(int address, uint8_t reg, uint8_t value)	{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

uint8_t GY80::readReg(int address, uint8_t reg)	{
	uint8_t value;
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(GYR_ADDRESS, 1);
	value = Wire.read();
	Wire.endTransmission();			
	return value;
}

void GY80::init_acc(void)	{
	writeReg(ADXAddress, Register_2D, 8);
}

void GY80::init_gyro(void)	{
	writeReg(GYR_ADDRESS,L3G4200D_CTRL_REG1, 0x0F);
}

void GY80::init_mag(void)	{
	writeReg(HMC5883L_Address, HMC5883L_MODE, HMC5883L_CFGR_A);
}

void GY80::read_acc(void)	{
	
	writeReg(ADXAddress,Register_X0,Register_X1);
	Wire.requestFrom(ADXAddress,2); 
	if(Wire.available()<=2)   
	{
		X0 = Wire.read();
		X1 = Wire.read(); 
		X1=X1<<8;
		acc.x=X0+X1;   
	}

	//------------------Y
	writeReg(ADXAddress,Register_Y0,Register_Y1);
	Wire.requestFrom(ADXAddress,2); 
	if(Wire.available()<=2)   
	{
		Y0 = Wire.read();
		Y1 = Wire.read(); 
		Y1=Y1<<8;
		acc.y=Y0+Y1;
	}
	//------------------Z
	writeReg(ADXAddress,Register_Z0,Register_Z1);
	Wire.requestFrom(ADXAddress,2); 
	if(Wire.available()<=2)   
	{
		Z0 = Wire.read();
		Z1 = Wire.read(); 
		Z1=Z1<<8;
		acc.z=Z0+Z1;
	}
}

void GY80::read_gyro(void)	{
	Wire.beginTransmission(GYR_ADDRESS);
	Wire.write(L3G4200D_OUT_X_L | (1 << 7)); 
	Wire.endTransmission();
	Wire.requestFrom(GYR_ADDRESS, 6);

	while (Wire.available() < 6);
	
	xla = Wire.read();
	xha = Wire.read();
	yla = Wire.read();
	yha = Wire.read();
	zla = Wire.read();
	zha = Wire.read();

	gyro.x = xha << 8 | xla;
	gyro.y = yha << 8 | yla;
	gyro.z = zha << 8 | zla;
}

void GY80::read_mag(void)	{
	Wire.beginTransmission(HMC5883L_Address);
	Wire.write(HMC5883L_OUT_X_H); //select register 3, X MSB register
	Wire.endTransmission();


	//Read data from each axis, 2 registers per axis
	Wire.requestFrom(HMC5883L_Address, 6);
	if(6<=Wire.available())
	{
		magX = Wire.read()<<8; //X msb
		magX |= Wire.read(); //X lsb
		magZ = Wire.read()<<8; //Z msb
		magZ |= Wire.read(); //Z lsb
		magY = Wire.read()<<8; //Y msb
		magY |= Wire.read(); //Y lsb
	}
	mag.x = magX;
	mag.y = magY;
	mag.z = magZ;
}

bool GY80::getRotation(float gravity[], float geomagnetic[])	{
	float _Sy=0, _Sp=0, _Sr =0;
	float Ax = gravity[0];
	float Ay = gravity[1];
	float Az = gravity[2];
	float Ex = geomagnetic[0];
	float Ey = geomagnetic[1];
	float Ez = geomagnetic[2];
	float Hx = Ey*Az - Ez*Ay;
	float Hy = Ez*Ax - Ex*Az;
	float Hz = Ex*Ay - Ey*Ax;
	float normH = (float)sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
	if (normH < 0.1f) {
		return false;
	}
	float invH = 1.0f / normH;
	Hx *= invH;
	Hy *= invH;
	Hz *= invH;
	float invA = 1.0f / (float)sqrt(Ax*Ax + Ay*Ay + Az*Az);
	Ax *= invA;
	Ay *= invA;
	Az *= invA;
	float Mx = Ay*Hz - Az*Hy;
	float My = Az*Hx - Ax*Hz;
	float Mz = Ax*Hy - Ay*Hx;
	float _y, _p, _r;
	_y = (float)atan2(Hy, My);
	_p = (float)asin(-Ay);
	_r = (float)atan2(-Ax, Az);
	
	_y = (float)(_y *180/3.14);
	_p = (float)(_p*180/3.14);
	_r = (float)(_r*180/3.14);
	
	if(_y <	0)	_Sy += _y+360;
	else	_Sy += _y;
	if(_p<0)	_Sp += _p+360;
	else	_Sp += _p;
	if(_r<0)	_Sr += _r+360;
	else	_Sr += _r;
	rotations.yaw = _Sy;
	rotations.pitch = _Sp;
	rotations.roll = _Sr;
	return true;
}