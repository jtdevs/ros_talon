#include <talon_bridge/talon.h>

namespace talon
{
	bool TalonSRX::FindCenter(talonsrx::SetPID::Request  &req, talonsrx::SetPID::Response &res){
		TalonSRX::findCenter();
		return true;
	}
	void TalonSRX::findCenter(){
		_ignoreTopics = 1;
		_percent = 0;
		_pauseFunction = 1;
		if(_center == 1){
			_recoverFunc = &TalonSRX::findCenterR;
		}
		else if(_center == 2)
			_recoverFunc = &TalonSRX::findCenter;
		else{
			_recoverFunc = &TalonSRX::findCenterL;
		}
	}

	void TalonSRX::findCenterR()
	{
		if(_center){
			_percent = -20;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
			setZero();
		}
	}

	void TalonSRX::findCenterL()
	{
		if(!_center){
			_percent = 20;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
			setZero();
		}
	}

	void TalonSRX::recoverLims(){
		_ignoreTopics = 1;
		_percent = 0;
		_pauseFunction = 1;
		if(_cwLimit == 1 && _ccwLimit == 0){
			_recoverFunc = &TalonSRX::recoverCW;
		}
		else if(_ccwLimit == 1 && _cwLimit == 0){
			_recoverFunc = &TalonSRX::recoverCCW;
		}
		else{
			ROS_ERROR("TALON_ERROR: BOTH LIMITS TRIGGERED. CHECK HARDWARE.");
		}
	}

	void TalonSRX::recoverCW()
	{
		if(_cwLimit){
			_percent = -20;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
		}
	}

	void TalonSRX::recoverCCW()
	{
		if(_ccwLimit){
			_percent = 20;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
		}
	}

}; // namespace talon_interface