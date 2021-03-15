/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */
#include <cstdlib>
#include <cstring>
#include <car/encoder/encoder_as5048a.h>
#include <car/bldc/driver.h>
#include <car/math/angle.h>
#include <car/math/pid.h>
#include <car/time/cycle_rate.h>
#include <car/model/race_car.h>
#include <Servo.h>
#include <EEPROM.h>


using namespace car::model;

RaceCar::RaceCar()
{
    vehile_parameters_.get_from_eeprom();

    ackermann_config_ = NULL;
    board_ = new car::bldc::Driver;
    motor_[LEFT] = new car::bldc::Motor(std::array<uint8_t, 3>({33, 26, 31}),
                                        std::array<uint8_t, 3>({10, 22, 23}),
                                        std::array<uint8_t, 3>({A15, A16, A17}), 2, car::math::Direction::COUNTERCLOCKWISE);
    motor_[RIGHT] = new car::bldc::Motor(std::array<uint8_t, 3>({28, 8, 25}),
                                         std::array<uint8_t, 3>({5, 6, 9}),
                                         std::array<uint8_t, 3>({A15, A16, A17}), 14, car::math::Direction::COUNTERCLOCKWISE);
    encoder_ = new car::encoder::AS5048A(std::array<uint8_t, 2>({motor_[LEFT]->pin_encoder_cs(), motor_[RIGHT]->pin_encoder_cs()}), 13);

    board_->init(motor_[LEFT], motor_[RIGHT]);

    motor_[LEFT]->init(11, std::array<car::math::AngleDeg, 2>({vehile_parameters_.angle_offest_motor0[0], vehile_parameters_.angle_offest_motor0[1]}),
    // motor_[LEFT]->init(11, std::array<car::math::AngleDeg, 2>({-65, -65 - 90}),
                       std::bind(&car::encoder::Encoder::read, encoder_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                       std::bind(&car::bldc::Driver::update_pwm, board_, std::placeholders::_1, std::placeholders::_2),
                       std::bind(&car::bldc::Driver::couple_pwm, board_, std::placeholders::_1, std::placeholders::_2));

    motor_[RIGHT]->init(11, std::array<car::math::AngleDeg, 2>({vehile_parameters_.angle_offest_motor1[0], vehile_parameters_.angle_offest_motor1[1]}),
    //motor_[RIGHT]->init(11, std::array<car::math::AngleDeg, 2>({-80 + 90, -80}),
                        std::bind(&car::encoder::Encoder::read, encoder_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                        std::bind(&car::bldc::Driver::update_pwm, board_, std::placeholders::_1, std::placeholders::_2),
                        std::bind(&car::bldc::Driver::couple_pwm, board_, std::placeholders::_1, std::placeholders::_2));


    control_config_.stamp = car::com::objects::Time::now();
    control_config_.pid[LEFT] = car::com::objects::PIDConfig(0.1, -1, 1, 0.2, 0.05, 0.01);
    control_config_.pid[RIGHT] = car::com::objects::PIDConfig(0.1, -1, 1, 0.2, 0.05, 0.01);

    const auto &pid_l = control_config_.pid[LEFT];
    const auto &pid_r = control_config_.pid[RIGHT];
    pid_rps_[ LEFT] = new car::math::PID(pid_l.dt, pid_l.min, pid_l.max, pid_l.Kp, pid_l.Ki, pid_l.Kd);
    pid_rps_[RIGHT] = new car::math::PID(pid_r.dt, pid_r.min, pid_r.max, pid_r.Kp, pid_r.Ki, pid_r.Kd);

    cycle_pwm_control_ = new car::time::CycleRate(10);
    cmd_raw_().couble(false);
    steering_servo_ = new Servo;
    steering_servo_->attach(4);     

}

void RaceCar::update()
{
    motor_[LEFT ]->update_pwm();
    motor_[RIGHT]->update_pwm();

    if (cycle_pwm_control_->passed())
    {
        motor_[LEFT ]->couple(cmd_raw_().coubled[LEFT ]);
        motor_[RIGHT]->couple(cmd_raw_().coubled[RIGHT]);

        motor_[LEFT ]->update_control();
        motor_[RIGHT]->update_control();

        state_raw_.stamp = motor_[LEFT]->position().stamp;
        state_raw_.value.v[LEFT ] = motor_[LEFT ]->rps();
        state_raw_.value.v[RIGHT] = motor_[RIGHT]->rps();

        if (cmd_raw_().coubled[LEFT]){
            motor_[LEFT ]->set_power(pid_rps_[LEFT ]->update(cmd_raw_().v[LEFT ], motor_[LEFT ]->rps()));
        }
        if (cmd_raw_().coubled[RIGHT]){
            motor_[RIGHT]->set_power(pid_rps_[RIGHT]->update(cmd_raw_().v[RIGHT], motor_[RIGHT]->rps()));
        }
        steering_servo_->write(cmd_raw_().steering*40+90);
    }
}
car::math::AckermannStateStamped &RaceCar::get_cmd_raw()
{
    return cmd_raw_;
}

car::model::AckermannStateStamped &RaceCar::get_state_raw()
{
    return state_raw_;
}

bool car::VehileParameters::put_into_eeprom(){
    EEPROM.put(0,*this);
    return true;
}

bool car::VehileParameters::get_from_eeprom(){
    EEPROM.get(0,*this);
    return true;
}