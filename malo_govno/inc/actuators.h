#ifndef ACTUATORS_H_
#define ACTUATORS_H_

void servo_init(unsigned int f_pwm);
void servo_set_umbrella_position(int8_t angle);
void servo_set_door_position(int8_t angle);
void actuators_umbrella(void);
void actuators_setup(void);
void servo_set_left_grabber_position(int8_t angle);
void servo_set_right_grabber_position(int8_t angle);

void servo_set_grabbers(int8_t angle);

#endif
