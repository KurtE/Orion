#include "RoboClawStream.h"
#include <Arduino.h>

#define USE_STREAM_TIMEDREAD

//
// Constructor
//
RoboClawStream::RoboClawStream(Stream *pstream, uint32_t tout,bool doack)
{
	ack=doack;
    _timeout = tout;
    _pstream = pstream;
    _pstream->setTimeout(tout);
}

//
// Destructor
//
RoboClawStream::~RoboClawStream()
{
}


//
// Simple forwarders to real stream functions.
int RoboClawStream::available(void)
{
    return _pstream->available();
}
    
int RoboClawStream::peek(void)
{
    return _pstream->peek();
}

int RoboClawStream::read(void)
{
    return _pstream->read();
}

void RoboClawStream::flush(void)
{
    _pstream->flush();
}

size_t RoboClawStream::write(uint8_t b)
{
    return _pstream->write(b);
}



bool RoboClawStream::write_n(uint8_t cnt, ... )
{
	uint8_t crc=0;
	
	//send data with crc
	va_list marker;
	va_start( marker, cnt );     /* Initialize variable arguments. */
	for(uint8_t index=0;index<cnt;index++){
		uint8_t data = va_arg(marker, int);
		crc+=data;
		_pstream->write(data);
	}
	va_end( marker );              /* Reset variable arguments.      */
	if(ack)
		_pstream->write(crc&0x7F | 0x80);
	else
		_pstream->write(crc&0x7F);
	if(ack)
		if(timedRead()==0xFF)
			return true;
	return false;
}

bool RoboClawStream::ForwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1FORWARD,speed);
}

bool RoboClawStream::BackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1BACKWARD,speed);
}

bool RoboClawStream::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINMB,voltage);
}

bool RoboClawStream::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXMB,voltage);
}

bool RoboClawStream::ForwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2FORWARD,speed);
}

bool RoboClawStream::BackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2BACKWARD,speed);
}

bool RoboClawStream::ForwardBackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M17BIT,speed);
}

bool RoboClawStream::ForwardBackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M27BIT,speed);
}

bool RoboClawStream::ForwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFORWARD,speed);
}

bool RoboClawStream::BackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDBACKWARD,speed);
}

bool RoboClawStream::TurnRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDRIGHT,speed);
}

bool RoboClawStream::TurnLeftMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLEFT,speed);
}

bool RoboClawStream::ForwardBackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFB,speed);
}

bool RoboClawStream::LeftRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLR,speed);
}

int RoboClawStream::timedRead()
{
    uint8_t b;
    if (_pstream->readBytes((char*)&b, (size_t)1))
        return b;
    else
        return -1;
}


bool RoboClawStream::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
    uint8_t data[6];
	uint8_t crc;
    uint8_t cbRead;

	_pstream->write(address);
	crc=address;
	_pstream->write(cmd);
	crc+=cmd;

	//send data with crc
	va_list marker;
	va_start( marker, cmd );     /* Initialize variable arguments. */
	for(uint8_t index=0;index<cnt;index++){
		uint32_t *ptr = (uint32_t *)va_arg(marker, void *);
        cbRead = _pstream->readBytes((char*)data, ((size_t)(index==(cnt-1))? 5 : 4));    // hack read checksum on last read
        *ptr = ((uint32_t)data[0]<<24) | ((uint32_t)data[1]<<16) | ((uint32_t)data[2]<<8) | ((uint32_t)data[3]);
        crc += data[0] + data[1] + data[2] + data[3];
	}
	va_end( marker );              /* Reset variable arguments.      */
    // Assuming cnt is not zero last iteration should have read in the crc value

    // BugBug Could actually look to see if we read in data...
	return ((crc&0x7F)==data[4]);
}

uint32_t RoboClawStream::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status,bool *valid){
    // use stream functions
    uint8_t data[6];
	uint8_t crc;
	uint32_t value;

	_pstream->write(address);
	crc=address;
	_pstream->write(cmd);
	crc+=cmd;

    uint8_t cbRead = _pstream->readBytes((char*)data, 6);
    value = ((uint32_t)data[0]<<24) | ((uint32_t)data[1]<<16) | ((uint32_t)data[2]<<8) | ((uint32_t)data[3]);
    crc += data[0] + data[1] + data[2] + data[3] + data[4];

	if(status)
		*status = data[4];

	if(valid)
		*valid = (cbRead == 6) && ((crc&0x7F)==data[5]);
		
	return value;
}

uint32_t RoboClawStream::ReadEncM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ENC,status,valid);
}

uint32_t RoboClawStream::ReadEncM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ENC,status,valid);
}

uint32_t RoboClawStream::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1SPEED,status,valid);
}

uint32_t RoboClawStream::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2SPEED,status,valid);
}

bool RoboClawStream::ResetEncoders(uint8_t address){
	return write_n(2,address,RESETENC);
}

bool RoboClawStream::ReadVersion(uint8_t address,char *version){
	uint8_t crc;
	_pstream->write(address);
	crc=address;
	_pstream->write(GETVERSION);
	crc+=GETVERSION;
	
    uint8_t cb = _pstream->readBytesUntil(0, version, 32);
    version[cb] = '\0'; // Make sure it is null terminated
	for(uint8_t i=0;i<32;i++){
		crc+=version[i];
		if(version[i]==0){
			if((crc&0x7F)==timedRead()) // calls our one byte helper function.
				return true;
			else
				return false;
		}
	}
	return false;
}

uint16_t RoboClawStream::Read2(uint8_t address,uint8_t cmd,bool *valid){
	uint8_t crc;
	uint16_t value;	
    uint8_t data[3];

	_pstream->write(address);
	crc=address;
	_pstream->write(cmd);
	crc+=cmd;

    uint8_t cbRead = _pstream->readBytes((char*)data, 3);
    value = ((uint16_t)data[0]<<8) | ((uint16_t)data[1]);
    crc += data[0] + data[1];

	if(valid)
		*valid = (cbRead == 3) && ((crc&0x7F)==data[2]);
		
	return value;
}

uint16_t RoboClawStream::ReadMainBatteryVoltage(uint8_t address,bool *valid){
	return Read2(address,GETMBATT,valid);
}

uint16_t RoboClawStream::ReadLogicBattVoltage(uint8_t address,bool *valid){
	return Read2(address,GETLBATT,valid);
}

bool RoboClawStream::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINLB,voltage);
}

bool RoboClawStream::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXLB,voltage);
}

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

bool RoboClawStream::SetM1VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){
	uint32_t kd = kd_fp*65536;
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool RoboClawStream::SetM2VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){
	uint32_t kd = kd_fp*65536;
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t RoboClawStream::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t RoboClawStream::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ISPEED,status,valid);
}

bool RoboClawStream::DutyM1(uint8_t address, uint16_t duty){
	return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool RoboClawStream::DutyM2(uint8_t address, uint16_t duty){
	return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool RoboClawStream::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2){
	return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

bool RoboClawStream::SpeedM1(uint8_t address, uint32_t speed){
	return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

bool RoboClawStream::SpeedM2(uint8_t address, uint32_t speed){
	return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

bool RoboClawStream::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
	return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClawStream::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

bool RoboClawStream::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
bool RoboClawStream::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2){
	return write_n(10,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClawStream::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawStream::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawStream::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed2),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClawStream::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawStream::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawStream::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClawStream::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2){
	bool valid;
	uint16_t value = Read2(address,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}

bool RoboClawStream::ReadCurrents(uint8_t address, uint8_t &current1, uint8_t &current2){
	bool valid;
	uint16_t value = Read2(address,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>8;
		current2 = value;
	}
	return valid;
}

bool RoboClawStream::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

bool RoboClawStream::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClawStream::DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel){
	return write_n(6,address,M1DUTY,SetWORDval(duty),SetWORDval(accel));
}

bool RoboClawStream::DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel){
	return write_n(6,address,M2DUTY,SetWORDval(duty),SetWORDval(accel));
}

bool RoboClawStream::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2){
	return write_n(10,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(accel1),SetWORDval(duty2),SetWORDval(accel2));
}

bool RoboClawStream::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClawStream::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClawStream::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClawStream::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClawStream::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	uint32_t value;
	bool valid = read_n(1,address,GETMINMAXMAINVOLTAGES,&value);
	min=value>>16;
	max = value&0xFFFF;
	return valid;
}
			
bool RoboClawStream::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	uint32_t value;
	bool valid = read_n(1,address,GETMINMAXLOGICVOLTAGES,&value);
	min=value>>16;
	max = value&0xFFFF;
	return valid;
}

bool RoboClawStream::SetM1PositionPID(uint8_t address,float kd_fp,float kp_fp,float ki_fp,float kiMax_fp,uint32_t deadzone,uint32_t min,uint32_t max){			
	uint32_t kd=kd_fp*1024;
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kiMax=kiMax_fp*1024;
	return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClawStream::SetM2PositionPID(uint8_t address,float kd_fp,float kp_fp,float ki_fp,float kiMax_fp,uint32_t deadzone,uint32_t min,uint32_t max){			
	uint32_t kd=kd_fp*1024;
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kiMax=kiMax_fp*1024;
	return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClawStream::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd,KiMax;
	bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	KiMax = ((float)KiMax_fp)/1024;
	return valid;
}

bool RoboClawStream::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd,KiMax;
	bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	KiMax = ((float)KiMax_fp)/1024;
	return valid;
}

bool RoboClawStream::SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClawStream::SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClawStream::SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool RoboClawStream::ReadTemp(uint8_t address, uint16_t &temp){
	bool valid;
	temp = Read2(address,GETTEMP,&valid);
	return valid;
}

uint8_t RoboClawStream::ReadError(uint8_t address,bool *valid){
	uint8_t crc;
	_pstream->write(address);
	crc=address;
	_pstream->write(GETERROR);
	crc+=GETERROR;
	
	uint8_t value = timedRead();
	crc+=value;

	if(valid)
		*valid = ((crc&0x7F)==timedRead());
	else
		timedRead();
		
	return value;
}

bool RoboClawStream::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode){
	bool valid;
	uint16_t value = Read2(address,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

bool RoboClawStream::SetM1EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool RoboClawStream::SetM2EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM2ENCODERMODE,mode);
}

bool RoboClawStream::WriteNVM(uint8_t address){
	return write_n(2,address,WRITENVM);
}
