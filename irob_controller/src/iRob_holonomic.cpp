// iRob kinematic plug-in for 3 and 4 wheels holonomic drivetrain system.
// By TinLethax at Robot Club KMITL (RB26)

#include "iRob_controller/iRob_holonomic.hpp"
#include <cmath>

#include <stdint.h>

// Math constants
#define RPM_TO_RAD_S  	0.1047f 	// 1 rpm == 0.1047 rad/s 
#define RAD_S_TO_RPM	9.549297f 	// 1 rad/s ~= 0.54 RPM
#define SINE_120 		0.866025f 	// Sine(120 degree) in rad unit
#define F2_SQRT3      	1.1547f 	// 2/sqrt(3)

namespace irob_holonomic_plugins{
	
	// Omni 3 wheels plug-in
	class Omni3 : public irob_holonomic_base::irob_holonomic{
		public:
			int init_iRob(
				irob_robot_param_t *irob_param_ptr_t) override{
					
				if(!irob_param_ptr_t){
					// Return error on null pointer
					return -1;
				}

				if(irob_param_ptr_t->irob_robot_type != 
					IROB_HOLO_OMNI3){
					// Return error on wrong holonomic type
					return -1;
				}
				irob_param_t.irob_robot_type = 
					irob_param_ptr_t->irob_robot_type;
					
					
				if(irob_param_ptr_t->robot_length < 0.001){
					// Return error on small robot length
					return -1;
				}
				irob_param_t.robot_length = 
					irob_param_ptr_t->robot_length;
				
				
				if(irob_param_ptr_t->robot_wheel_radius < 0.001){
					// Return error on small wheel radius
					return -1;
				}
				irob_param_t.robot_wheel_radius = 
					irob_param_ptr_t->robot_wheel_radius;

				
				irob_param_t.robot_gear_ratio =
					irob_param_ptr_t->robot_gear_ratio;
				
				return 0;
			}
			
			void forward_kinematic(
				cmd_vel_t *cmd_ptr_t,		// Output XY,aZ 
				wheel_vel_t *wheel_ptr_t	// Input Wheel RPM measurement
				) override{
				
				wheel_ptr_t->v1 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// LF
				wheel_ptr_t->v2 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// LB
				wheel_ptr_t->v3 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// RB
				
				cmd_ptr_t->vel_x =
					(wheel_ptr_t->v2 - wheel_ptr_t->v3) *
					SINE_120 *
					irob_param_t.robot_wheel_radius
				;
				
				cmd_ptr_t->vel_y = 
					(	wheel_ptr_t->v1 - 
						((wheel_ptr_t->v2 - wheel_ptr_t->v3) * 0.5)
					) *
					irob_param_t.robot_wheel_radius
				;
				
				cmd_ptr_t->vel_az = 
					((wheel_ptr_t->v1 	+
					wheel_ptr_t->v2 	+
					wheel_ptr_t->v3) / 3) 	* 
					(
						irob_param_t.robot_wheel_radius	/
						irob_param_t.robot_length						
					)
				;	
			}
			
			void inverse_kinematic(
				cmd_vel_t *cmd_ptr_t,		// Input XY,aZ command
				wheel_vel_t *wheel_ptr_t	// Output wheel velocity
				) override{
				
				wheel_ptr_t->v1 = 
					(
					cmd_ptr_t->vel_y	+
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v2 = 
					(
					(cmd_ptr_t->vel_x	* SINE_120)	+
					(cmd_ptr_t->vel_y	* -0.5)		+
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v3 = 
					(
					(cmd_ptr_t->vel_x	* -SINE_120)+
					(cmd_ptr_t->vel_y	* -0.5)		+
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
			
				wheel_ptr_t->v1 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v2 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v3 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
			}
			
		protected:
			irob_robot_param_t irob_param_t;
			
	};
	
	// Omni 4 wheels plug-in
	class Omni4 : public irob_holonomic_base::irob_holonomic{
		public:
			int init_iRob(
				irob_robot_param_t *irob_param_ptr_t) override{
					
				if(!irob_param_ptr_t){
					// Return error on null pointer
					return -1;
				}

				if(irob_param_ptr_t->irob_robot_type != 
					IROB_HOLO_OMNI4){
					// Return error on wrong holonomic type
					return -1;
				}
				irob_param_t.irob_robot_type = 
					irob_param_ptr_t->irob_robot_type;
					
					
				if(irob_param_ptr_t->robot_length < 0.001){
					// Return error on small robot length
					return -1;
				}
				irob_param_t.robot_length = 
					irob_param_ptr_t->robot_length;
				
				
				if(irob_param_ptr_t->robot_wheel_radius < 0.001){
					// Return error on small wheel radius
					return -1;
				}
				irob_param_t.robot_wheel_radius = 
					irob_param_ptr_t->robot_wheel_radius;

				irob_param_t.robot_gear_ratio =
					irob_param_ptr_t->robot_gear_ratio;
				
				return 0;
			}
			
			void forward_kinematic(
				cmd_vel_t *cmd_ptr_t,		// Output XY,aZ 
				wheel_vel_t *wheel_ptr_t	// Input Wheel RPM measurement
				) override{
				
				wheel_ptr_t->v1 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// LF
				wheel_ptr_t->v2 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// LB
				wheel_ptr_t->v3 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// RB
				wheel_ptr_t->v4 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// RF
				
				cmd_ptr_t->vel_x =
					(	wheel_ptr_t->v1 +	
						wheel_ptr_t->v2 +
						wheel_ptr_t->v3 +
						wheel_ptr_t->v4
					) * 
					(irob_param_t.robot_wheel_radius / 4)
				;
				
				cmd_ptr_t->vel_y = 
					((	-wheel_ptr_t->v1 +	
						wheel_ptr_t->v2 -
						wheel_ptr_t->v3 +
						wheel_ptr_t->v4
					) * 
					(irob_param_t.robot_wheel_radius / 4)
				);
				
				cmd_ptr_t->vel_az = 
					(	-wheel_ptr_t->v1 -	
						wheel_ptr_t->v2 +
						wheel_ptr_t->v3 +
						wheel_ptr_t->v4
					) * 
					(irob_param_t.robot_wheel_radius / 
					(4 * irob_param_t.robot_length)
					)
				;	
			}
			
			void inverse_kinematic(
				cmd_vel_t *cmd_ptr_t,		// Input XY,aZ command
				wheel_vel_t *wheel_ptr_t	// Output wheel velocity
				) override{
				
				wheel_ptr_t->v1 = // LF
					(
					cmd_ptr_t->vel_x	-
					cmd_ptr_t->vel_y	-
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v2 = // LB
					(
					cmd_ptr_t->vel_x	+
					cmd_ptr_t->vel_y	-
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v3 = // RB
					(
					cmd_ptr_t->vel_x	-
					cmd_ptr_t->vel_y	+
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
			
				wheel_ptr_t->v4 = // RF
					(
					cmd_ptr_t->vel_x	+
					cmd_ptr_t->vel_y	+
					(cmd_ptr_t->vel_az	* irob_param_t.robot_length)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v1 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v2 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v3 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v4 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
			}
		
		protected:
			irob_robot_param_t irob_param_t;
	};
	
	// Mecanum 4 wheels plug-in
	class Meca4 : public irob_holonomic_base::irob_holonomic{
		public:
			int init_iRob(
				irob_robot_param_t *irob_param_ptr_t) override{
					
				if(!irob_param_ptr_t){
					// Return error on null pointer
					return -1;
				}

				if(irob_param_ptr_t->irob_robot_type != 
					IROB_HOLO_MECA4){
					// Return error on wrong holonomic type
					return -1;
				}
				irob_param_t.irob_robot_type = 
					irob_param_ptr_t->irob_robot_type;
					
					
				if(irob_param_ptr_t->robot_length < 0.001){
					// Return error on small robot length
					return -1;
				}
				irob_param_t.robot_length = 
					irob_param_ptr_t->robot_length;
					
				if(irob_param_ptr_t->robot_width < 0.001){
					// Return error on small robot length
					return -1;
				}
				irob_param_t.robot_width = 
					irob_param_ptr_t->robot_width;	
				
				
				if(irob_param_ptr_t->robot_wheel_radius < 0.001){
					// Return error on small wheel radius
					return -1;
				}
				irob_param_t.robot_wheel_radius = 
					irob_param_ptr_t->robot_wheel_radius;

				robot_total_len = 
					irob_param_t.robot_length +
					irob_param_t.robot_width;
				
				irob_param_t.robot_gear_ratio =
					irob_param_ptr_t->robot_gear_ratio;
				
				return 0;
			}
			
			void forward_kinematic(
				cmd_vel_t *cmd_ptr_t,		// Output XY,aZ 
				wheel_vel_t *wheel_ptr_t	// Input Wheel RPM measurement
				) override{
				
				wheel_ptr_t->v1 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// LF
				wheel_ptr_t->v2 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// LB
				wheel_ptr_t->v3 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// RB
				wheel_ptr_t->v4 *= RPM_TO_RAD_S / irob_param_t.robot_gear_ratio;// RF
				
				cmd_ptr_t->vel_x =
					(	wheel_ptr_t->v1 +	
						wheel_ptr_t->v2 +
						wheel_ptr_t->v3 +
						wheel_ptr_t->v4
					) * 
					(irob_param_t.robot_wheel_radius / 4)
				;
				
				cmd_ptr_t->vel_y = 
					((	-wheel_ptr_t->v1 +	
						wheel_ptr_t->v2 -
						wheel_ptr_t->v3 +
						wheel_ptr_t->v4
					) * 
					(irob_param_t.robot_wheel_radius / 4)
				);
				
				cmd_ptr_t->vel_az = 
					(	-wheel_ptr_t->v1 -	
						wheel_ptr_t->v2 +
						wheel_ptr_t->v3 +
						wheel_ptr_t->v4
					) * 
					(irob_param_t.robot_wheel_radius / 
					(4 * robot_total_len)
					)
				;	
			}
			
			void inverse_kinematic(
				cmd_vel_t *cmd_ptr_t,		// Input XY,aZ command
				wheel_vel_t *wheel_ptr_t	// Output wheel velocity
				) override{
				
				wheel_ptr_t->v1 = // LF
					(
					cmd_ptr_t->vel_x	-
					cmd_ptr_t->vel_y	-
					(cmd_ptr_t->vel_az	* robot_total_len)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v2 = // LB
					(
					cmd_ptr_t->vel_x	+
					cmd_ptr_t->vel_y	-
					(cmd_ptr_t->vel_az	* robot_total_len)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v3 = // RB
					-(
					cmd_ptr_t->vel_x	-
					cmd_ptr_t->vel_y	+
					(cmd_ptr_t->vel_az	* robot_total_len)
					) / irob_param_t.robot_wheel_radius
				;
			
				wheel_ptr_t->v4 = // RF
					-(
					cmd_ptr_t->vel_x	+
					cmd_ptr_t->vel_y	+
					(cmd_ptr_t->vel_az	* robot_total_len)
					) / irob_param_t.robot_wheel_radius
				;
				
				wheel_ptr_t->v1 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v2 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v3 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
				wheel_ptr_t->v4 *= RAD_S_TO_RPM * irob_param_t.robot_gear_ratio;
			}
		
		
		protected:
			irob_robot_param_t irob_param_t;
			float robot_total_len;
		
	};

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(irob_holonomic_plugins::Omni3, irob_holonomic_base::irob_holonomic)
PLUGINLIB_EXPORT_CLASS(irob_holonomic_plugins::Omni4, irob_holonomic_base::irob_holonomic)
PLUGINLIB_EXPORT_CLASS(irob_holonomic_plugins::Meca4, irob_holonomic_base::irob_holonomic)