/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#include <Arduino.h>
#include <vector>
#include <array>
#include <vector>
#include <numeric>
#include <limits> // std::numeric_limits
#include <algorithm>

#include <car/model/race_car.h>
#include <car/bldc/driver.h>
#include <car/math/pid.h>
#include <car/encoder/encoder_as5048a.h>
#include <car/math/angle.h>
#include <car/math/motion.h>
#include <car/time/cycle_rate.h>
#include <car/com/objects/text.h>
#include <car/com/mc/interface.h>
#include <car/com/objects/error.h>

car::model::RaceCar *rcar;
car::time::CycleRate cycle_pwm(2); /// object for a constant cycle control

int loop_count = 0;
size_t delay_count = 0;
car::com::objects::Array16SC8 delays;

uint32_t timer_count;

car::com::mc::Interface msg_tx;      /// object to hande the serial communication
car::com::mc::Interface msg_rx;      /// object to hande the serial communication
car::time::CycleRate cycle_com(100); /// object for a constant cycle control
car::com::objects::Text text;        /// object to send
car::com::objects::Error *error = NULL;

// the setup routine runs once when you press reset:
void setup()
{
  rcar = new car::model::RaceCar();
  car::math::AngleDeg::init();

  Serial.begin(115200); /// init serial
  while (!Serial)
    ;
  Serial.println("hello");

  delay(1000);
  msg_rx.try_sync(); /// blocks until a sync message arrives

  delays.clear();
}
// the loop routine runs over and over again forever:
void loop()
{
  if (delay_count >= delays.size())
  {
    delay_count = 0;
    delays.clear();
  }
  int32_t cycle_delay = cycle_pwm.wait();
  if (cycle_delay < 0)
  {
    error = new car::com::objects::Error;
    sprintf(error->info, "%6ld micros", cycle_delay);
  }
  rcar->update();

  if (cycle_com.passed() > 0)
  {
    using namespace car::com::objects;
    msg_tx.reset(); /// removes all objects in message
    if (!text.empty())
    {
      msg_tx.push_object(Object(text, TYPE_TEXT));
      text.clear();
    }
    {
      /// send current state
      AckermannState ackermann_state;
      ackermann_state.copy_from(rcar->get_state_raw().value);
      ackermann_state.stamp = car::com::objects::Time::fromMicros(rcar->get_state_raw().stamp);
      msg_tx.push_object(Object(ackermann_state, TYPE_ACKERMANN_STATE));
    }
    if (rcar->ackermann_config_ != NULL){
      /// send used ackermann_config
      // msg_tx.push_object(Object(*rcar->ackermann_config_, TYPE_ACKERMANN_CONFIG));
    }
    {
      /// send used ackermann_config
      msg_tx.push_object(Object(rcar->pose_stamped_, TYPE_POSE_STAMPED));
    }
    {
      /// send target state
      AckermannState ackermann_state;
      ackermann_state.copy_from(rcar->get_cmd_raw().value);
      ackermann_state.stamp = car::com::objects::Time::fromMicros(rcar->get_cmd_raw().stamp);
      msg_tx.push_object(Object(ackermann_state, TYPE_ACKERMANN_CMD));
    }
    {
      /// config control
      msg_tx.push_object(Object(rcar->control_config_, TYPE_CONTROL_CONFIG));
    }

    if (error != NULL)
    {
      msg_tx.push_object(Object(error, TYPE_ERROR));
      delete error;
      error = NULL;
    }
    msg_tx.send(); /// sends the message
  }
  if (msg_rx.receive())
  { /// check for messages
    using namespace car::com::objects;
    static Object object;
    while (msg_rx.pop_object(object).isValid())
    {
      switch (object.type)
      {
      case TYPE_SYNC:                       /// case sync object
        Time::compute_offset(msg_rx.stamp); /// set clock
        break;
      case TYPE_ACKERMANN_CONFIG: /// case sync object
        if (rcar->ackermann_config_ == NULL){
          rcar->ackermann_config_ = new car::com::objects::AckermannConfig;
        }
        object.get(*rcar->ackermann_config_);
        break;
      case TYPE_ACKERMANN_CMD:
      { /// case sync object
        AckermannState cmd;
        object.get(cmd);
        cmd.copy_to(rcar->get_cmd_raw().value);
        rcar->get_cmd_raw().stamp = car::com::objects::Time::toMicros(cmd.stamp);
      }
      break;
      default: /// case unkown type
        text.print("%zu Unknown type in %zu", object.type, msg_rx.seq);
        continue;
      }
    }
  }
  loop_count++;
}