#ifndef ACTUATORS_H_
#define ACTUATORS_H_

void servo_init(unsigned int f_pwm);
void servo_set_kisobran_position(int8_t angle);
void servo_set_vrata_position(int8_t angle);

#endif
