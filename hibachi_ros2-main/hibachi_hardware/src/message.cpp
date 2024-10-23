
#include "hibachi_hardware/message.h"
#include <memory>
#include <cstring>
#include <cstdlib>

hibachi_base::Message* hibachi_base::Message::parse(const uint8_t* data, size_t length)
{
	uint16_t type = ((((uint16_t)data[1]) << 8) | (((uint16_t)data[0])));
	switch (type)
	{
	case EchoMessage::MESSAGE_TYPE:
		return new EchoMessage(data, length);
	case WheelEncoderData::MESSAGE_TYPE:
		return new WheelEncoderData(data, length);
	// case FourWheelEncoderData::MESSAGE_TYPE:
    //     return new FourWheelEncoderData(data, length);
    case FourWheelEncoderPosition::MESSAGE_TYPE:
        return new FourWheelEncoderPosition(data, length);
	case FourWheelEncoderSpeed::MESSAGE_TYPE:
        return new FourWheelEncoderSpeed(data, length);
	case MPUData::MESSAGE_TYPE:
		return new MPUData(data, length);
	default:
		return NULL;
	}
}

hibachi_base::Message::Message(const uint8_t* data, size_t length)
	: _data(NULL)
	, _size(length)
{
	_data = (uint8_t*)malloc(_size);
	memcpy(_data, data, _size);
}

hibachi_base::Message::Message(uint16_t messageType, size_t payloadLength)
	: _data(NULL)
	, _size(payloadLength + 4)
{
	_data = (uint8_t*)malloc(_size);
	_data[0] = messageType & 0xFF;
	_data[1] = (messageType >> 8) & 0xFF;
	_data[2] = payloadLength & 0xFF;
	_data[3] = (payloadLength >> 8) & 0xFF;
}

hibachi_base::Message::~Message()
{
	if (_data != NULL)
	{
		free(_data);
	}
}

uint16_t hibachi_base::Message::getType() const
{
	return (
	(((uint16_t)_data[1]) << 8) | 
	(((uint16_t)_data[0]))
	);
}

uint8_t* hibachi_base::Message::getData() const
{
	return _data;
}

size_t hibachi_base::Message::getSize() const
{
	return _size;
}

int8_t hibachi_base::Message::getI8(size_t offset) const
{
	return (int8_t)getU8(offset);
}

int16_t hibachi_base::Message::getI16(size_t offset) const
{
	return (int16_t)getU16(offset);
}

int32_t hibachi_base::Message::getI32(size_t offset) const
{
	return (int32_t)getU32(offset);
}

uint8_t hibachi_base::Message::getU8(size_t offset) const
{
	return _data[offset + 4];
}

uint16_t hibachi_base::Message::getU16(size_t offset) const
{
	return (
	(((uint16_t)_data[offset + 5]) << 8) | 
	(((uint16_t)_data[offset + 4]))
	);
}

uint32_t hibachi_base::Message::getU32(size_t offset) const
{
	return (
	(((uint32_t)_data[offset + 7]) << 24) | 
	(((uint32_t)_data[offset + 6]) << 16) | 
	(((uint32_t)_data[offset + 5]) << 8)  | 
	(((uint32_t)_data[offset + 4]))
	);
}

void hibachi_base::Message::setI8(size_t offset, int8_t value)
{
	setU8(offset, (uint8_t)value);
}

void hibachi_base::Message::setI16(size_t offset, int16_t value)
{
	setU16(offset, (uint16_t)value);
}

void hibachi_base::Message::setI32(size_t offset, int32_t value)
{
	setU32(offset, (uint32_t)value);
}

void hibachi_base::Message::setU8(size_t offset, uint8_t value)
{
	_data[offset + 4] = value;
}

void hibachi_base::Message::setU16(size_t offset, uint16_t value)
{
	_data[offset + 4] = (uint8_t)((value) & 0xFF);
	_data[offset + 5] = (uint8_t)((value >> 8) & 0xFF);
}

void hibachi_base::Message::setU32(size_t offset, uint32_t value)
{
	_data[offset + 4] = (uint8_t)((value) & 0xFF);
	_data[offset + 5] = (uint8_t)((value >> 8) & 0xFF);
	_data[offset + 6] = (uint8_t)((value >> 16) & 0xFF);
	_data[offset + 7] = (uint8_t)((value >> 24) & 0xFF);
}

hibachi_base::SetSkidSteerMotorSpeed::SetSkidSteerMotorSpeed(double frontLeftWheel,
                                                    double frontRightWheel,
                                                    double rearLeftWheel,
                                                    double rearRightWheel) 
    : Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
        setU16(FRONT_LEFT_SPEED, (int16_t) (frontLeftWheel * 1000));
        setU16(FRONT_RIGHT_SPEED, (int16_t) (frontRightWheel * 1000));
        setU16(REAR_LEFT_SPEED, (int16_t) (rearLeftWheel * 1000));
        setU16(REAR_RIGHT_SPEED, (int16_t) (rearRightWheel * 1000));
}

hibachi_base::SetMotorSpeed::SetMotorSpeed(double leftWheel, double rightWheel)
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
	if (leftWheel > 0.0)
	{
		setU8(L_DIR, 1);
		setU8(L_PWR, (uint8_t)(leftWheel * 255));
	}
	else if (leftWheel < 0.0)
	{
		setU8(L_DIR, 2);
		setU8(L_PWR, (uint8_t)(-leftWheel * 255));
	}
	else
	{
		setU8(L_DIR, 0);
		setU8(L_PWR, 0);
	}
	if (rightWheel > 0.0)
	{
		setU8(R_DIR, 1);
		setU8(R_PWR, (uint8_t)(rightWheel * 255));
	}
	else if (rightWheel < 0.0)
	{
		setU8(R_DIR, 2);
		setU8(R_PWR, (uint8_t)(-rightWheel * 255));
	}
	else
	{
		setU8(R_DIR, 0);
		setU8(R_PWR, 0);
	}
	
	setU8(L_DIR, 5);
    setU8(L_PWR, 205);
}

hibachi_base::GetFourWheelEncoder::GetFourWheelEncoder()
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
}

hibachi_base::GetWheelEncoder::GetWheelEncoder()
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
}

hibachi_base::GetMPUData::GetMPUData()
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
}

hibachi_base::ResetOdometry::ResetOdometry()
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
}

hibachi_base::SetPIDGains::SetPIDGains(uint8_t wheel, double kP, double kI, double kD)
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
        setU8(WHEEL_SELECTOR, wheel);
        setU16(PID_KP_GAIN, (int16_t) (kP * 1000));
        setU16(PID_KI_GAIN, (int16_t) (kI * 1000));
        setU16(PID_KD_GAIN, (int16_t) (kD * 1000));
}

hibachi_base::EchoMessage::EchoMessage(uint32_t value)
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
	setU16(VALUE, value);
}

hibachi_base::EchoMessage::EchoMessage(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

uint32_t hibachi_base::EchoMessage::getValue() const
{
	return getU16(VALUE);
}

hibachi_base::WheelEncoderData::WheelEncoderData(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

int32_t hibachi_base::WheelEncoderData::getLeftPulses() const
{
	return (int32_t)getI16(L_PULSES);
}

int32_t hibachi_base::WheelEncoderData::getRightPulses() const
{
	return (int32_t)getI16(R_PULSES);
}

hibachi_base::FourWheelEncoderData::FourWheelEncoderData(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

int32_t hibachi_base::FourWheelEncoderData::getFrontLeftPulses() const
{
	return (int32_t)getI16(FRONT_LEFT_PULSES);
}

int32_t hibachi_base::FourWheelEncoderData::getFrontRightPulses() const
{
	return (int32_t)getI16(FRONT_RIGHT_PULSES);
}

int32_t hibachi_base::FourWheelEncoderData::getRearLeftPulses() const
{
	return (int32_t)getI16(REAR_LEFT_PULSES);
}

int32_t hibachi_base::FourWheelEncoderData::getRearRightPulses() const
{
	return (int32_t)getI16(REAR_RIGHT_PULSES);
}

hibachi_base::FourWheelEncoderPosition::FourWheelEncoderPosition(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

bool hibachi_base::FourWheelEncoderPosition::getWheelsAngPosition(double &front_left, double &front_right, double &rear_left, double &rear_right) {
	front_left = getI16(FRONT_LEFT_ANGPOS) / 1000.0;
	front_right = getI16(FRONT_RIGHT_ANGPOS) / 1000.0;
	rear_left = getI16(REAR_LEFT_ANGPOS) / 1000.0;
	rear_right = getI16(REAR_RIGHT_ANGPOS) / 1000.0;

	return true;
}

hibachi_base::FourWheelEncoderSpeed::FourWheelEncoderSpeed(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

bool hibachi_base::FourWheelEncoderSpeed::getWheelsAngSpeed(double &front_left, double &front_right, double &rear_left, double &rear_right) {
	front_left = getI16(FRONT_LEFT_ANGSPEED) / 1000.0;
	front_right = getI16(FRONT_RIGHT_ANGSPEED) / 1000.0;
	rear_left = getI16(REAR_LEFT_ANGSPEED) / 1000.0;
	rear_right = getI16(REAR_RIGHT_ANGSPEED) / 1000.0;

	return true;
}


hibachi_base::MPUData::MPUData(const uint8_t* data, size_t length)
	: Message(data, length)
{

}

int32_t hibachi_base::MPUData::getAccelerationX()
{
	return (int32_t)getI16(ACC_X);
}

int32_t hibachi_base::MPUData::getAccelerationY()
{
	return (int32_t)getI16(ACC_Y);
}

int32_t hibachi_base::MPUData::getAccelerationZ()
{
	return (int32_t)getI16(ACC_Z);
}

int32_t hibachi_base::MPUData::getGyroX()
{
	return (int32_t)getI16(GYR_X);
}

int32_t hibachi_base::MPUData::getGyroY()
{
	return (int32_t)getI16(GYR_Y);
}

int32_t hibachi_base::MPUData::getGyroZ()
{
	return (int32_t)getI16(GYR_Z);
}
