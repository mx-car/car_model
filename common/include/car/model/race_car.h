/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MODEL_RACE_CAR_H
#define CAR_MODEL_RACE_CAR_H

#include <cstdint>
#include <array>
#include <car/math/value.h>
#include <car/math/motion.h>
#include <car/com/objects/object.h>
#include <Servo.h>

namespace car
{
    namespace encoder
    {
        class Encoder;
    };
    namespace bldc
    {
        class Driver;
        class Motor;
    }
    namespace math
    {
        class PID;
    }
    namespace time
    {
        class CycleRate;
    }
    
    struct VehileParameters{
        car::com::objects::ControlParameter control;  
        bool put_into_eeprom();
        bool get_from_eeprom();
    };

    namespace model
    {
        typedef car::math::Value<car::math::AckermannState> AckermannStateStamped;
        /**
        * Class to read angle encoder values
        */
        class RaceCar
        {
        public:
            static const int LEFT = 0;
            static const int RIGHT = 1;
            static const int FORWARD = 0;
            static const int BACKWARD = 1;
        /**
        * Constructor
        */
            RaceCar();
            void init();

            void update();
            void apply_parameter();
            AckermannStateStamped &get_cmd_raw();
            AckermannStateStamped &get_state_raw();

            Servo                 *steering_servo_; 
            car::bldc::Motor      *motor_[2];
            car::math::PID        *pid_rps_[2];
            car::encoder::Encoder *encoder_;
            car::bldc::Driver     *board_;
            car::time::CycleRate  *cycle_pwm_control_;
            AckermannStateStamped cmd_raw_;
            AckermannStateStamped state_raw_;
            car::com::objects::ControlParameter control_parameter_;
            car::com::objects::AckermannConfig *ackermann_config_;
            car::com::objects::PoseStamped pose_stamped_;
            car::VehileParameters vehile_parameters_;   /// stored in eeeprom


        
        };
    } // namespace car
} // namespace car
#endif // CAR_MODEL_RACE_CAR_H
