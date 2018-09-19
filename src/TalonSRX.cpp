#include <talon_bridge/talon.h>

namespace talon
{


	TalonSRX::TalonSRX(ros::NodeHandle* nh)
    {
    	_CANSender = nh->advertise<can_msgs::Frame>("sent_messages", 10);
    	_posPub = nh->advertise<std_msgs::Float32>("talonsrx/current_position", 10);
    	_statusPub = nh->advertise<talonsrx::Status>("talonsrx/status",10);
    	_CANReceiver = nh->subscribe("received_messages", 10, &TalonSRX::processCanFrame, this);
    	_spid = nh->advertiseService("talonsrx/SetPID", &TalonSRX::setPID, this);
    	_fcenter = nh->advertiseService("talonsrx/FindCenter", &TalonSRX::FindCenter, this);
    	//prevent the talon from going idle
		_talon_timer = nh->createTimer(ros::Duration(0.05), &TalonSRX::enableFrame, this);
      	_nh = nh;
	}


	void TalonSRX::setup(unsigned char ID, unsigned char mode)
	{
		_ccwLimit = 0;
		_cwLimit = 0;
		_center = 2;
		_data32 = 0x00;
		_baseArbID = 0x00;
		_ignoreTopics = 0;
		_pauseFunction = 0;
		_statusTemp = 0.0;
		_statusOutputCurrent = 0.0;
		_statusBusVoltage = 0.0;
		_statusClosedLoopError = 0.0;
		//device ID 0x3F is reserved for general addressing on enable frames.
		if(ID == 0x3F)
			ROS_ERROR("Using reserved ID.");

		//Build the base ArbID used in every frame.
		_baseArbID = (uint)(TALON | ID);
		TalonSRX::ClearStickyFaults();
		switch(mode){
			case modePercentOutput:
				_percent = 0;
				_modeFunc = &TalonSRX::percentOutput;
				_TalonInput = _nh->subscribe("talonsrx/motor_percent", 10, &TalonSRX::setPercentVal, this);
				break;

			case modeServoPosition:
				_pos = 0.0;
				_modeFunc = &TalonSRX::ServoPos;
				_TalonInput = _nh->subscribe("talonsrx/steering_angle", 10, &TalonSRX::setPos, this);
				TalonSRX::findCenter();
				break;

			case modeMotionProfile:
				_modeFunc = NULL;
				break;

			default:
				_modeFunc = NULL;
				break;
		}
	}

	//This frame needs to be sent at least at 10Hz. I'm doing it at 20Hz.
	void TalonSRX::enableFrame(const ros::TimerEvent& event)
	{
		can_msgs::Frame f;
		f.id = 0x000401BF;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		f.data[0] = 1;
		for (int i = 1; i < 8; i++)
			f.data[i] = 0;
		_CANSender.publish(f);
		if(_pauseFunction == 0){
			if(_modeFunc != NULL){
				(*this.*_modeFunc)();
			}
		}
		else{
			TalonSRX::percentOutput();
		}
		if(_recoverFunc != NULL){
			(*this.*_recoverFunc)();
		}
		TalonSRX::publishStatus();
	}

	void TalonSRX::unpackStatus1(const can_msgs::Frame &f)
	{
		if((f.data[0] & 1) == 1)
			ROS_ERROR("ERROR: TALON REPORTS HARDWARE FAILURE.");
	    _cwLimit = (unsigned char)((f.data[3] >> 6) & 1) == 0;
		_ccwLimit = (unsigned char)((f.data[3] >> 7) & 1) == 0;
		if((_ccwLimit || _cwLimit) && _recoverFunc == NULL)
			TalonSRX::recoverLims();
	}

	void TalonSRX::unpackStatus2(const can_msgs::Frame &f)
	{
		unsigned char H = (unsigned char)(f.data[5]);
		unsigned char L = (unsigned char)(f.data[6]);
		H &= 0xFF;
		L &= 0xC0;
		int32_t raw = 0;
		raw |= H;
        raw <<= 8;
        raw |= L;
        raw >>= 6;
        _statusOutputCurrent = 0.125 * raw + 0.0;
	}

	void TalonSRX::unpackStatus3(const can_msgs::Frame &f)
	{
	    int32_t _data = 0;
	    for (int i = 0; i <3; i++)
	    {
	    	_data |= f.data[i];
	    	_data <<= 8;
	    }
	    _data >>= 8;
	    
	    if((int)((f.data[7] >> 4) & 1) == 1)
	    	_data *= 8;

	    std_msgs::Float32 msg;
	    msg.data = (float)_data*-360.0/6045.0;
	    _currentPos = _data;
	    _posPub.publish(msg);
	    _center = (unsigned char)((f.data[7] >> 5) & 1);
	}

	void TalonSRX::unpackStatus4(const can_msgs::Frame &f)
	{
		unsigned char L = (unsigned char)(f.data[0]);
		int32_t raw = 0;
		raw |= L;
		_statusBusVoltage = 0.05 * raw + 4.0;
		L = (unsigned char)(f.data[7]);
		L &= 0x3F;
		raw = 0;
		raw |= L;
		_statusTemp = (float)raw;
	}

	void TalonSRX::unpackStatus13(const can_msgs::Frame &f)
	{
		unsigned char H = (unsigned char)(f.data[0]);
        unsigned char M = (unsigned char)(f.data[1]);
        unsigned char L = (unsigned char)(f.data[2]);
        int32_t error = 0;
        error |= H;
        error <<= 8;
        error |= M;
        error <<= 8;
        error |= L;
        error <<= 8;
        error >>= 8;
        _statusClosedLoopError = (float)error*360.0/6045.0;

	}

	void TalonSRX::processCanFrame(const can_msgs::Frame &f){
		_data32 = f.id;

		unsigned char device_number = (unsigned char)(_data32>>0);
		device_number = device_number<<2;
		device_number = device_number>>2;

		unsigned char device_id = (unsigned char)(_data32>>24);
		device_id = device_id&0x0F;

		uint32_t command_id = (_data32&0x0007FFC0);

		if((int) device_id == 2){
			switch(command_id){
				case STATUS_01:
					unpackStatus1(f);
					break;
				case STATUS_02:
					unpackStatus2(f);
					break;
				case STATUS_03:
					unpackStatus3(f);
					break;
				case STATUS_04:
					unpackStatus4(f);
					break;
				case STATUS_13:
					unpackStatus13(f);
					break;
				case PARAM_SET:
                    ROS_INFO("PARAM_SET received.\n \tdata[0] = %#02x.\n \tdata[1] = %#02x.\n \tdata[2] = %#02x.\n \tdata[3] = %#02x.\n\tdata[4] = %#02x.\n \tdata[5] = %#02x.\n \tdata[6] = %#02x.\n \tdata[7] = %#02x.\n",f.data[0],f.data[1],f.data[2],f.data[3],f.data[4],f.data[5],f.data[6],f.data[7]);
                    break;

				default:
					break;
			}
		}
		else if(f.is_error){
				ROS_ERROR("Frame received on can0 from unknown device is error. ArbID: %#010X.\n", f.id);
				for(int i = 0; i < 8; i++)
					ROS_ERROR("\tdata[%d] = %#02x.\n",i,f.data[i]);
		}
	}

	void TalonSRX::publishStatus(){
		talonsrx::Status s;
		s.header.stamp = ros::Time::now();
		s.header.frame_id = "0";
		s.Temperature = _statusTemp;
		s.OutputCurrent = _statusOutputCurrent;
		s.BusVoltage = _statusBusVoltage;
		s.ClosedLoopError = _statusClosedLoopError;
		_statusPub.publish(s);
	}

}; // namespace talon_interface
