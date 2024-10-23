
#ifndef ___HIBACHI_BASE_MESSAGE_H___
#define ___HIBACHI_BASE_MESSAGE_H___

#include <stddef.h>
#include <stdint.h>

namespace hibachi_base
{
	class Message
	{
	public:
		static Message* parse(const uint8_t* data, size_t length);

	protected:
		Message(const uint8_t* data, size_t length);
		Message(uint16_t messageType, size_t payloadLength);
		virtual ~Message();

	public:
		uint16_t getType() const;
		uint8_t* getData() const;
		size_t getSize() const;

	public:
		int8_t getI8(size_t offset) const;
		int16_t getI16(size_t offset) const;
		int32_t getI32(size_t offset) const;
		uint8_t getU8(size_t offset) const;
		uint16_t getU16(size_t offset) const;
		uint32_t getU32(size_t offset) const;
		void setI8(size_t offset, int8_t value);
		void setI16(size_t offset, int16_t value);
		void setI32(size_t offset, int32_t value);
		void setU8(size_t offset, uint8_t value);
		void setU16(size_t offset, uint16_t value);
		void setU32(size_t offset, uint32_t value);

	private:
		uint8_t* _data;
		size_t _size;

	};
    
    class SetSkidSteerMotorSpeed
        : public Message
    {
    public:
        enum { MESSAGE_TYPE = 0x0005 };
        enum
        {
            FRONT_LEFT_SPEED = 0,
            FRONT_RIGHT_SPEED = 2,
            REAR_LEFT_SPEED = 4,
            REAR_RIGHT_SPEED = 6,
            PAYLOAD_LEN = 8,
        };
    public:
        SetSkidSteerMotorSpeed(double frontLeftWheel, double frontRightWheel,
                      double rearLeftWheel, double rearRightWheel);
    };
            

	class SetMotorSpeed 
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0001 };
		enum
		{
			L_DIR = 0,
			L_PWR = 1,
			R_DIR = 2,
			R_PWR = 3,
			PAYLOAD_LEN = 4,
		};

	public:
		SetMotorSpeed(double leftWheel, double rightWheel);
        
	};
    
    class GetFourWheelEncoder
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0004 };
		enum
		{
			PAYLOAD_LEN = 0,
		};

	public:
		GetFourWheelEncoder();

	};

	class GetWheelEncoder
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0002 };
		enum
		{
			PAYLOAD_LEN = 0,
		};

	public:
		GetWheelEncoder();

	};

	class GetMPUData
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0003 };
		enum
		{
			PAYLOAD_LEN = 0,
		};

	public:
		GetMPUData();

	};

	class ResetOdometry
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0006 };
		enum
		{
			PAYLOAD_LEN = 0,
		};

	public:
		ResetOdometry();

	};

	class SetPIDGains
        : public Message
    {
    public:
        enum { MESSAGE_TYPE = 0x0010 };
        enum
        {
            WHEEL_SELECTOR = 0,
			PID_KP_GAIN = 1,
			PID_KI_GAIN = 3,
			PID_KD_GAIN = 5,
            PAYLOAD_LEN = 7,
        };
    public:
        SetPIDGains(uint8_t wheel, double kP, double kI, double kD);
    };

	class EchoMessage
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x00AA };
		enum
		{
			VALUE = 0,
			PAYLOAD_LEN = 2,
		};

	public:
		EchoMessage(uint32_t value);
		EchoMessage(const uint8_t* data, size_t length);
		uint32_t getValue() const;

	};

	class WheelEncoderData
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x7002 };
		enum
		{
			L_PULSES = 0,
			R_PULSES = 2,
			PAYLOAD_LEN = 4,
		};

	public:
		WheelEncoderData(const uint8_t* data, size_t length);

	public:
		int32_t getLeftPulses() const;
		int32_t getRightPulses() const;

	};

	class FourWheelEncoderData
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x7004 };
		enum
		{
			FRONT_LEFT_PULSES = 0,
			FRONT_RIGHT_PULSES = 2,
            REAR_LEFT_PULSES = 4,
            REAR_RIGHT_PULSES = 6,
            PAYLOAD_LEN = 8,
		};

	public:
		FourWheelEncoderData(const uint8_t* data, size_t length);

	public:
		int32_t getFrontLeftPulses() const;
		int32_t getFrontRightPulses() const;
        int32_t getRearLeftPulses() const;
		int32_t getRearRightPulses() const;

	};
    
    class FourWheelEncoderPosition
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x7004 };
		enum
		{
			FRONT_LEFT_ANGPOS = 0,
			FRONT_RIGHT_ANGPOS = 2,
            REAR_LEFT_ANGPOS = 4,
            REAR_RIGHT_ANGPOS = 6,
            PAYLOAD_LEN = 8,
		};

	public:
		FourWheelEncoderPosition(const uint8_t* data, size_t length);

	public:
		bool getWheelsAngPosition(double &front_left, double &front_right, double &rear_left, double &rear_right);
	};

	class FourWheelEncoderSpeed
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x7006 };
		enum
		{
			FRONT_LEFT_ANGSPEED = 0,
			FRONT_RIGHT_ANGSPEED = 2,
            REAR_LEFT_ANGSPEED = 4,
            REAR_RIGHT_ANGSPEED = 6,
            PAYLOAD_LEN = 8,
		};

	public:
		FourWheelEncoderSpeed(const uint8_t* data, size_t length);

	public:
		bool getWheelsAngSpeed(double &front_left, double &front_right, double &rear_left, double &rear_right);
	};

	class MPUData
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x7003 };
		enum
		{
			ACC_X = 0,
			ACC_Y = 2,
			ACC_Z = 4,
			GYR_X = 6,
			GYR_Y = 8,
			GYR_Z = 10,
			PAYLOAD_LEN = 12,
		};

	public:
		MPUData(const uint8_t* data, size_t length);

	public:
		int32_t getAccelerationX();
		int32_t getAccelerationY();
		int32_t getAccelerationZ();
		int32_t getGyroX();
		int32_t getGyroY();
		int32_t getGyroZ();

	};

}

#endif // ___HIBACHI_BASE_MESSAGE_H___

