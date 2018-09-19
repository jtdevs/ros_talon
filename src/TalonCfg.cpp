#include <talon_bridge/talon.h>

namespace talon
{
	bool TalonSRX::setPID(talonsrx::SetPID::Request  &req, talonsrx::SetPID::Response &res)
	{
		TalonSRX::setKP(req.kp);
		TalonSRX::setKI(req.ki);
		TalonSRX::setKD(req.kd);
		TalonSRX::setKF(0.0);
		return true;
	}

	void TalonSRX::setKP(float value){
		int32_t rawbits = 0;
		uint32_t urawbits;
        if (value > 1023)
            value = 1023;
        else if (value < 0)
            value = 0;
        urawbits = (uint32_t)(value * FLOAT_TO_FXP_10_22);
        rawbits = (int32_t)urawbits;

        can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((int32_t)KP >> 4);
		f.data[1] = (unsigned char)((unsigned char)((int32_t)KP & 0xF) << 4);
		f.data[2] = (unsigned char)(rawbits >> 0x18);
		f.data[3] = (unsigned char)(rawbits >> 0x10);
		f.data[4] = (unsigned char)(rawbits >> 0x08);
		f.data[5] = (unsigned char)(rawbits >> 0x00);
		f.data[6] = 0x00;
		f.data[7] = 0x00;
		_CANSender.publish(f);
    }

	void TalonSRX::setKI(float value){
		int32_t rawbits = 0;
		uint32_t urawbits;
        if (value > 1023)
            value = 1023;
        else if (value < 0)
            value = 0;
        urawbits = (uint32_t)(value * FLOAT_TO_FXP_10_22);
        rawbits = (int32_t)urawbits;

        can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((int32_t)KI >> 4);
		f.data[1] = (unsigned char)((unsigned char)((int32_t)KI & 0xF) << 4);
		f.data[2] = (unsigned char)(rawbits >> 0x18);
		f.data[3] = (unsigned char)(rawbits >> 0x10);
		f.data[4] = (unsigned char)(rawbits >> 0x08);
		f.data[5] = (unsigned char)(rawbits >> 0x00);
		f.data[6] = 0x00;
		f.data[7] = 0x00;
		_CANSender.publish(f);
	}

	void TalonSRX::setKD(float value){
		int32_t rawbits = 0;
		uint32_t urawbits;
        if (value > 1023)
            value = 1023;
        else if (value < 0)
            value = 0;
        urawbits = (uint32_t)(value * FLOAT_TO_FXP_10_22);
        rawbits = (int32_t)urawbits;

        can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((int32_t)KD >> 4);
		f.data[1] = (unsigned char)((unsigned char)((int32_t)KD & 0xF) << 4);
		f.data[2] = (unsigned char)(rawbits >> 0x18);
		f.data[3] = (unsigned char)(rawbits >> 0x10);
		f.data[4] = (unsigned char)(rawbits >> 0x08);
		f.data[5] = (unsigned char)(rawbits >> 0x00);
		f.data[6] = 0x00;
		f.data[7] = 0x00;
		_CANSender.publish(f);
	}

	void TalonSRX::setKF(float value){
		int32_t rawbits = 0;
		if (value > 512) /* bounds check doubles that are outside s10.22 */
            value = 512;
        else if (value < -512)
            value = -512;
        rawbits = (int32_t)(value * FLOAT_TO_FXP_10_22);

        can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((int32_t)KF >> 4);
		f.data[1] = (unsigned char)((unsigned char)((int32_t)KF & 0xF)<<4);
		f.data[2] = (unsigned char)(rawbits >> 0x18);
		f.data[3] = (unsigned char)(rawbits >> 0x10);
		f.data[4] = (unsigned char)(rawbits >> 0x08);
		f.data[5] = (unsigned char)(rawbits >> 0x00);
		f.data[6] = 0;
		f.data[7] = 0;
		_CANSender.publish(f);
	}

	void TalonSRX::setFeedback2QuadEncoder()
	{
		can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((int)330 >> 4);
		f.data[1] = (unsigned char)((unsigned char)((int)330 & 0xF)<<4);
		for (int i = 2; i < 8; i++)
			f.data[i] = 0;
		_CANSender.publish(f);
	}

	void TalonSRX::setZero()
	{
		can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((int)401 >> 4);
		f.data[1] = (unsigned char)((unsigned char)((int)401 & 0xF)<<4);
		for (int i = 2; i < 8; i++)
			f.data[i] = 0;
		_CANSender.publish(f);
	}

	void TalonSRX::ClearStickyFaults()
	{
		can_msgs::Frame f;
		f.id = PARAM_SET | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = (unsigned char)((uint32_t)StickyFaults >> 4);
		f.data[1] = (unsigned char)(((unsigned char)((uint32_t)StickyFaults & 0xF) << 4));
		for (int i = 2; i < 8; i++)
			f.data[i] = 0;
		_CANSender.publish(f);
	}

}; // namespace talon_interface