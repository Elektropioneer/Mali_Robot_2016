#ifndef ACTUATORS_H_
#define ACTUATORS_H_

void servo_init(unsigned int f_pwm);
<<<<<<< HEAD
<<<<<<< HEAD
void servo_set_kisobran_position(int8_t angle);
void servo_set_vrata_position(int8_t angle);
void actuators_kisobran(void);
void actuators_setup_kisobran(void);

=======
void servo_set_position(int8_t angle);
>>>>>>> e0b240b1c14c4966c5f4aab6a35058ead1d19ffe
=======
void servo_set_position(int8_t angle);
>>>>>>> 07363ff7400dbe0af1895fa548283af7a30f1eb3
#endif
