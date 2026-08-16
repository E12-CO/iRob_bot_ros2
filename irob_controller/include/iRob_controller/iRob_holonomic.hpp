#ifndef IROB_KINEMATICS_HOLONOMIC_HPP
#define IROB_KINEMATICS_HOLONOMIC_HPP

#include <stdint.h>
#include <string>

typedef struct{
	std::string irob_robot_type;
	float robot_length;
	float robot_width;
	float robot_wheel_radius;
	float robot_gear_ratio;
}irob_robot_param_t;

typedef struct{
	float vel_x;
	float vel_y;
	float vel_az;
}cmd_vel_t;

typedef struct{
	float v1;
	float v2;
	float v3;
	float v4;			
}wheel_vel_t;

namespace irob_holonomic_base{
	
class irob_holonomic{
	public:
		virtual int init_iRob(irob_robot_param_t *irob_param_ptr_t) = 0;
		virtual void forward_kinematic(
			cmd_vel_t *cmd_ptr_t,		// Output XY,aZ 
			wheel_vel_t *wheel_ptr_t	// Input Wheel RPM measurement
			) = 0;
		virtual void inverse_kinematic(
			cmd_vel_t *cmd_ptr_t,		// Input XY,aZ command
			wheel_vel_t *wheel_ptr_t	// Output wheel velocity
			) = 0;
		virtual ~irob_holonomic(){}
	
	protected:
		irob_holonomic(){}
};

}

#endif