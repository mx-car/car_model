/* EEPROM Read
 *
 * Reads the value of each byte of the EEPROM and prints it 
 * to the computer.
 */

#include <Arduino.h>
#include <car/model/race_car.h>
#include <car/encoder/encoder_as5048a.h>
#include <car/bldc/driver.h>
#include <car/time/cycle_rate.h>

bool put_into_eeprom = true;
bool get_from_eeprom = true;
void setup()
{
  Serial.begin(115200);
  while (!Serial);
}

void loop()
{
  if(put_into_eeprom){
    car::VehileParameters params;
    //**  mx01 **/
    params.angle_offest_motor0[0] = -50;
    params.angle_offest_motor0[1] = -50 - 90;
    params.angle_offest_motor1[0] =  130 + 90;
    params.angle_offest_motor1[1] =  130;


    //**  mx02 **/
    /*
    params.angle_offest_motor0[0] = -65;
    params.angle_offest_motor0[1] = -65 - 90;
    params.angle_offest_motor1[0] = -80 + 90;
    params.angle_offest_motor1[1] = -80;
    */

    params.put_into_eeprom();
  }

  if(get_from_eeprom){
    car::VehileParameters params;
    params.get_from_eeprom();

    Serial.print("motor0:");
    Serial.print(params.angle_offest_motor0[0]);
    Serial.print(", ");
    Serial.println(params.angle_offest_motor0[1]);
    Serial.print("motor1:");
    Serial.print(params.angle_offest_motor1[0]);
    Serial.print(", ");
    Serial.println(params.angle_offest_motor1[1]);
    Serial.println("---------------------");
    put_into_eeprom = false;
  }
  delay(500);
}
