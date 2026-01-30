
#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "config.h" // pins
#include "stepper.h"

uint16_t currentSteps = 0;  // Always positive, wraps at STEPS_PER_REVOLUTION


typedef enum motorStatus_t
{
    READY,
    RUNNING,
} motorStatus_t;

typedef enum motorDirection_t
{
    CW,
    CCW,
} motorDirection_t;

motorStatus_t motorStatus = READY;

Stepper stepper(PUL_PIN, DIR_PIN, ENA_PIN);

uint8_t new_motor_status = 0;
uint8_t old_motor_status = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    // Use EEPROM.get() to read multi-byte values (uint16_t = 2 bytes)
    EEPROM.get(0, currentSteps);
    stepper.init();
    stepper.enable();

    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);  // Relay control for power cycle

    digitalWrite(DIR_PIN, 1);
    digitalWrite(ENA_PIN, 0);
    digitalWrite(RELAY_PIN, LOW);  // Ensure relay is off initially
    
    // stepper.move(100, 1);
    // stepper.run_until_done();
    // stepper.move(100, 0);
    // stepper.run_until_done();

}

void loop()
{
    // digitalWrite(PUL_PIN, 1);
    // delayMicroseconds(1000);
    // digitalWrite(PUL_PIN, 0);
    // delayMicroseconds(1000);
    if (Serial.available())
    {

        // StaticJsonDocument<256> doc;
        JsonDocument doc;
        String input = Serial.readStringUntil('\n');

        DeserializationError error = deserializeJson(doc, input);

        if (error)
        {
            Serial.println("{\"status\":\"ERROR\",\"msg\":\"Bad JSON\"}");
            // return;
        }

        const char *command = doc["command"];

         
        
        if (strcmp(command, "power cycle") == 0)
        {
            // Trigger relay to power cycle the DDC264EVM board
            Serial.println("{\"status\":\"OK\",\"msg\":\"Triggering power cycle relay\"}");
            digitalWrite(RELAY_PIN, HIGH);
            delay(2000);  // Hold relay on for 2000ms
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("{\"status\":\"OK\",\"msg\":\"Power cycle relay triggered\"}");
        }
        
        if (strcmp(command, "position") == 0)
        {
            uint16_t memSteps;
            EEPROM.get(0, memSteps);
            Serial.println("{\"status\":\"OK\",\"msg\":\"Position: " + String(round(currentSteps/CALIBRATED_STEPS_PER_DEGREE)) + " degrees / "+ String(currentSteps)+" steps"+ "\"}");
            Serial.println("{\"status\":\"OK\",\"msg\":\"Position (memory): " + String(round(memSteps/CALIBRATED_STEPS_PER_DEGREE)) + " degrees / "+ String(memSteps)+" steps"+ "\"}");
            
        }
        if (strcmp(command, "home") == 0)
        {
            Serial.println("{\"status\":\"OK\",\"msg\":\"current offset: "+String(round(currentSteps/CALIBRATED_STEPS_PER_DEGREE)) + " degrees / "+ String(currentSteps)+" steps"+ "\"}");
            
            // Calculate shortest path to home (0)
            uint16_t stepsToHome;
            uint8_t direction;
            
            if (currentSteps <= STEPS_PER_REVOLUTION / 2) {
                // Closer to go CCW (backwards)
                stepsToHome = currentSteps;
                direction = 1;  // CCW
            } else {
                // Closer to go CW (forward to wrap around)
                stepsToHome = STEPS_PER_REVOLUTION - currentSteps;
                direction = 0;  // CW
            }
            
            if (stepsToHome > 0) {
                stepper.move(stepsToHome, direction);
            }
            
            currentSteps = 0;
            EEPROM.put(0, currentSteps);
           
        }
        if (strcmp(command, "set home") == 0)
        {
            currentSteps = 0;
            EEPROM.put(0, currentSteps);
            Serial.println("{\"status\":\"OK\",\"msg\":\"Current position set to 0\"}");
        }
        if (strcmp(command, "go") == 0)
        {
            const char *direction = doc["direction"];
            uint16_t steps = doc["steps"];
            uint16_t degrees = doc["degrees"];

            if (degrees)
            {
                steps = round(degrees * CALIBRATED_STEPS_PER_DEGREE);
                
            }
            if(stepper.is_running()) {
                Serial.println("{\"status\":\"ERROR\",\"msg\":\"Motor is busy!. To use command wait until end or stop with \"stop\" command!");
            }

            if (strcmp(direction, "CCW") == 0)
            {
                stepper.move(steps, 1);
                // Modular arithmetic: wrap around at STEPS_PER_REVOLUTION
                if (steps > currentSteps) {
                    // Wrapping backwards past 0
                    currentSteps = STEPS_PER_REVOLUTION - (steps - currentSteps);
                } else {
                    currentSteps -= steps;
                }
                // Normalize to 0 if we hit exactly 0 or full revolution
                if (currentSteps == STEPS_PER_REVOLUTION) {
                    currentSteps = 0;
                }
               // Serial.println("{\"status\":\"OK\",\"msg\":\"New position: " + String(round(currentSteps/CALIBRATED_STEPS_PER_DEGREE)) + " degrees / "+ String(currentSteps)+" steps"+ "\"}");
                EEPROM.put(0, currentSteps);
            }
            else if (strcmp(direction, "CW") == 0)
            {
                stepper.move(steps, 0);
                // Modular arithmetic: wrap around at STEPS_PER_REVOLUTION
                currentSteps = (currentSteps + steps) % STEPS_PER_REVOLUTION;
               // Serial.println("{\"status\":\"OK\",\"msg\":\"New position: " + String(round(currentSteps/CALIBRATED_STEPS_PER_DEGREE)) + " degrees / "+ String(currentSteps)+" steps"+ "\"}");
                EEPROM.put(0, currentSteps);

            }
            else
            {
                Serial.println("{\"status\":\"ERROR\",\"msg\":\"Invalid command!");
                return;
            }

        }

        if (strcmp(command, "absolute position") == 0)
        {
            float degrees = doc["degrees"];
            
            if(stepper.is_running()) {
                Serial.println("{\"status\":\"ERROR\",\"msg\":\"Motor is busy! Wait until end or stop with stop command\"}");
                return;
            }
            
            // Convert degrees to steps
            uint16_t targetSteps = (uint16_t)(degrees * CALIBRATED_STEPS_PER_DEGREE) % STEPS_PER_REVOLUTION;
            
            // Calculate shortest path from currentSteps to targetSteps
            int16_t delta = targetSteps - currentSteps;
            uint16_t stepsToMove;
            uint8_t direction;
            
            // Normalize delta to range [-STEPS_PER_REVOLUTION/2, STEPS_PER_REVOLUTION/2]
            if (delta > STEPS_PER_REVOLUTION / 2) {
                // Going CW would wrap around; go CCW instead
                stepsToMove = STEPS_PER_REVOLUTION - delta;
                direction = 1;  // CCW
            } else if (delta < -(STEPS_PER_REVOLUTION / 2)) {
                // Going CCW would wrap around; go CW instead
                stepsToMove = STEPS_PER_REVOLUTION + delta;
                direction = 0;  // CW
            } else if (delta >= 0) {
                // Normal CW movement
                stepsToMove = delta;
                direction = 0;  // CW
            } else {
                // Normal CCW movement
                stepsToMove = -delta;
                direction = 1;  // CCW
            }
            
            if (stepsToMove > 0) {
                stepper.move(stepsToMove, direction);
                currentSteps = targetSteps;
                EEPROM.put(0, currentSteps);
                Serial.println("{\"status\":\"OK\",\"msg\":\"Moving to " + String(degrees) + " degrees (" + String(targetSteps) + " steps)\"}");
            } else {
                Serial.println("{\"status\":\"OK\",\"msg\":\"Already at target position: " + String(degrees) + " degrees\"}");
            }
        }

        if (strcmp(command, "status") == 0)
        {
            if (!stepper.is_running())
            {
                Serial.println("{\"status\":\"OK\",\"msg\":\"Ready\"}");
            }
            else 
            {
                Serial.println("{\"status\":\"BUSY\",\"msg\":\"Moving\"}");
            }
        }

        if (strcmp(command, "stop") == 0)
        {
            stepper.stop();
        }

        if (strcmp(command, "disable_motor") == 0)
        {
            stepper.disable_motor();
        }

        if (strcmp(command, "enable_motor") == 0)
        {
            stepper.enable_motor();
        }
        if (strcmp(command, "run continuous") == 0) {
            uint16_t delay_us = doc["delay"] | DELAY_MICROS;
            uint8_t dir = doc["dir"] | 1;
            stepper.start_continuous(dir, delay_us);
            Serial.println("{\"status\":\"OK\",\"msg\":\"Continuous started\"}");
        } else if (strcmp(command, "stop") == 0) {
            stepper.stop();
            Serial.println("{\"status\":\"OK\",\"msg\":\"Stopped\"}");
        } else if (strcmp(command, "set speed") == 0) {
            uint16_t delay_us = doc["delay"] | DELAY_MICROS;
            stepper.set_speed(delay_us);    
            Serial.println("{\"status\":\"OK\",\"msg\":\"Speed updated\"}");
        }

        

        if (Serial.read() == 'h')
        {
            Serial.println("help command received");
        }
    }

    new_motor_status = stepper.is_running();
    uint8_t motor_started = new_motor_status && !old_motor_status;
    uint8_t motor_stopped = !new_motor_status && old_motor_status;
    old_motor_status = new_motor_status;

    if (motor_started)
    {
        Serial.println("{\"status\":\"BUSY\",\"msg\":\"Move started\"}");
    }

    if (motor_stopped)
    {
        Serial.println("{\"status\":\"OK\",\"msg\":\"Move complete\"}");
    }
    stepper.update();
}
