/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_MODEL_RACE_CAR_H
#define CAR_MODEL_RACE_CAR_H

#include <cstdint>
#include <array>
#include <car/math/value.h>
#include <car/math/motion.h>
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
        /**
        * Constructor
        */
            RaceCar();

            void update();
            AckermannStateStamped &get_cmd_raw();
            AckermannStateStamped &get_state_raw();

        private:
            Servo                 *steering_servo_; 
            car::bldc::Motor      *motor_[2];
            car::math::PID        *pid_rps_[2];
            car::encoder::Encoder *encoder_;
            car::bldc::Driver     *board_;
            car::time::CycleRate  *cycle_pwm_control_;
            AckermannStateStamped cmd_raw_;
            AckermannStateStamped state_raw_;
        };
    } // namespace car
} // namespace car
#endif // CAR_MODEL_RACE_CAR_H
