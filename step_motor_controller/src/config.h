#ifndef CONFIG_H
#define CONFIG_H

#define PUL_PIN 5
#define DIR_PIN 6
#define ENA_PIN 7
#define RELAY_PIN 8  // Pin for power cycle relay control
const int16_t STEPS_PER_REVOLUTION = 25600.0; // 1/256 microstepping with 200 steps/rev motor
const float CALIBRATED_STEPS_PER_DEGREE = STEPS_PER_REVOLUTION / 360.0;  // SW5-SW8 1 1 1 0 (1/256 microstep, 25600 pules/rev)


#endif