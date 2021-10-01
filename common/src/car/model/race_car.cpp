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
}

void RaceCar::init()
{
    ackermann_config_ = NULL;
    board_ = new car::bldc::Driver;

    apply_parameter();

    motor_[LEFT] = new car::bldc::Motor(std::array<uint8_t, 3>({33, 26, 31}),
                                        std::array<uint8_t, 3>({10, 22, 23}),
                                        std::array<uint8_t, 3>({A15, A16, A17}), 2, car::math::Direction::COUNTERCLOCKWISE);
    motor_[RIGHT] = new car::bldc::Motor(std::array<uint8_t, 3>({28, 8, 25}),
                                         std::array<uint8_t, 3>({5, 6, 9}),
                                         std::array<uint8_t, 3>({A15, A16, A17}), 14, car::math::Direction::COUNTERCLOCKWISE);
    encoder_ = new car::encoder::AS5048A(std::array<uint8_t, 2>({motor_[LEFT]->pin_encoder_cs(), motor_[RIGHT]->pin_encoder_cs()}), 13);

    board_->init(motor_[LEFT], motor_[RIGHT]);

    motor_[LEFT]->init(vehile_parameters_.control.bldc[LEFT].nr_of_coils, std::array<car::math::AngleDeg, 2>({
                                        vehile_parameters_.control.bldc[LEFT].angle_offset[FORWARD], 
                                        vehile_parameters_.control.bldc[LEFT].angle_offset[BACKWARD]}),
                       std::bind(&car::encoder::Encoder::read, encoder_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                       std::bind(&car::bldc::Driver::update_pwm, board_, std::placeholders::_1, std::placeholders::_2),
                       std::bind(&car::bldc::Driver::couple_pwm, board_, std::placeholders::_1, std::placeholders::_2));

    motor_[RIGHT]->init(vehile_parameters_.control.bldc[RIGHT].nr_of_coils, std::array<car::math::AngleDeg, 2>({
                                        vehile_parameters_.control.bldc[RIGHT].angle_offset[FORWARD], 
                                        vehile_parameters_.control.bldc[RIGHT].angle_offset[BACKWARD]}),
                        std::bind(&car::encoder::Encoder::read, encoder_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                        std::bind(&car::bldc::Driver::update_pwm, board_, std::placeholders::_1, std::placeholders::_2),
                        std::bind(&car::bldc::Driver::couple_pwm, board_, std::placeholders::_1, std::placeholders::_2));

    cycle_pwm_control_ = new car::time::CycleRate(10);
    cmd_raw_().couble(false);
    steering_servo_ = new Servo;
    steering_servo_->attach(4);     
}

void RaceCar::apply_parameter()
{
    using namespace car::com::objects;
    bool parameters_applied = false;

    if(vehile_parameters_.control.pid[LEFT] != control_parameter_target_.pid[LEFT]) {
        parameters_applied = true;
        const auto &pid = vehile_parameters_.control.pid[LEFT] = control_parameter_target_.pid[LEFT];   
        pid_rps_[ LEFT] = new car::math::PID(pid.dt, pid.min, pid.max, pid.Kp, pid.Ki, pid.Kd);
    }
    if(vehile_parameters_.control.pid[RIGHT] != control_parameter_target_.pid[RIGHT]) {
        parameters_applied = true;
        const auto &pid = vehile_parameters_.control.pid[RIGHT] = control_parameter_target_.pid[RIGHT];    
        pid_rps_[ RIGHT] = new car::math::PID(pid.dt, pid.min, pid.max, pid.Kp, pid.Ki, pid.Kd);
    }

    if(vehile_parameters_.control.bldc[LEFT] != control_parameter_target_.bldc[LEFT]) {
        parameters_applied = true;
        const auto &bldc = vehile_parameters_.control.bldc[LEFT] = control_parameter_target_.bldc[LEFT];
        motor_[LEFT]->set_phase_offsets(std::array<car::math::AngleDeg, 2>({bldc.angle_offset[FORWARD], bldc.angle_offset[BACKWARD]}));
    }
    if(vehile_parameters_.control.bldc[RIGHT] != control_parameter_target_.bldc[RIGHT]) {
        parameters_applied = true;
        const auto &bldc = vehile_parameters_.control.bldc[RIGHT] = control_parameter_target_.bldc[RIGHT];
        motor_[RIGHT]->set_phase_offsets(std::array<car::math::AngleDeg, 2>({bldc.angle_offset[FORWARD], bldc.angle_offset[BACKWARD]}));
    }

    if(parameters_applied) {
        control_parameter_current_ = vehile_parameters_.control;
        control_parameter_current_.stamp = car::com::objects::Time::now();
    }
}
void RaceCar::update()
{
    motor_[LEFT ]->update_pwm();
    motor_[RIGHT]->update_pwm();

    if (cycle_pwm_control_->passed())
    {

        apply_parameter();
        
        motor_[LEFT ]->couple(cmd_raw_().coubled[LEFT ]);
        motor_[RIGHT]->couple(cmd_raw_().coubled[RIGHT]);

        motor_[LEFT ]->update_control();
        motor_[RIGHT]->update_control();

        state_raw_.stamp = motor_[LEFT]->position().stamp;
        state_raw_.value.v[LEFT ] = motor_[LEFT ]->rps();
        state_raw_.value.v[RIGHT] = motor_[RIGHT]->rps();
        state_raw_.value.mode = cmd_raw_().mode;

        if(cmd_raw_().mode == cmd_raw_().MODE_PWM){
            if (cmd_raw_().coubled[LEFT]){
                motor_[LEFT ]->set_power(cmd_raw_().v[LEFT ]);
            }
            if (cmd_raw_().coubled[RIGHT]){
                motor_[RIGHT]->set_power(cmd_raw_().v[RIGHT]);
            }
        }
        if(cmd_raw_().mode == cmd_raw_().MODE_VELOCITY ){
            if (cmd_raw_().coubled[LEFT]){
                motor_[LEFT ]->set_power(pid_rps_[LEFT ]->update(cmd_raw_().v[LEFT ], motor_[LEFT ]->rps()));
            }
            if (cmd_raw_().coubled[RIGHT]){
                motor_[RIGHT]->set_power(pid_rps_[RIGHT]->update(cmd_raw_().v[RIGHT], motor_[RIGHT]->rps()));
            }
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