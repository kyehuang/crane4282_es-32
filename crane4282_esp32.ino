#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "armDriver.h"

# define BaudRate 115200 
#define UPDATE_ARM_DELAY 1.0
#define SERIAL_READ_DELAY 10
#define SERIAL_WRITE_DELAY 250
#define UPDATE_STEPPER_DELAY 50.0

// Enum to define the arm actions
enum class ArmAction {
    NO_ACTION,
    TURN_BASE_LEFT,
    TURN_BASE_RIGHT,
    MOVE_ARM_UP,
    MOVE_ARM_DOWN,
    SHIFT_ARM_RIGHT,
    SHIFT_ARM_LEFT,
    ROTATE_FINGER_RIGHT,
    ROTATE_FINGER_LEFT
};
ArmAction armAction = ArmAction::NO_ACTION;

// arm control variables
# define NUM_OF_SERVOS 7
const uint8_t servoMinAngles[] = {0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 180, 160, 180, 180, 180, 180};
float currentAngles[NUM_OF_SERVOS];
uint8_t targetAngles[NUM_OF_SERVOS];

// crane control variables
# define NUM_OF_CRANE_MOTOR 2
float CraneState[NUM_OF_CRANE_MOTOR];
# define PIN_MOTOR1 32  // dc motor 1
# define PIN_MOTOR2 33  // dc motor 2

# define PIN_PULSE 27   // stepper motor pulse
# define PIN_DIR 26     // stepper motor direction
# define PIN_ENA 25     // stepper motor enable

// Define task handles
TaskHandle_t armControlTask;
TaskHandle_t serialCommunicationTask;
TaskHandle_t serialWriterTask;
TaskHandle_t craneControlTask;

// Function declarations
void armControlTaskFunction(void *parameter);
void serialCommunicationTaskFunction(void *parameter);
void serialWriterTaskFunction(void *parameter);
void craneControlTaskFunction(void *parameter);
void motor_init();
void DC_motor_execute(int state);
void Stepper_motor_execute(int state);

void setup() {
    // Start serial communication
    Serial.begin(BaudRate);

    // Initialize the servo driver
    for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
        currentAngles[i] = (float) servoMinAngles[i];
        switch (i) {
            case 0:
                targetAngles[0] = 90;
                break;
            case 1:
                targetAngles[1] = 90;
                break;
            case 2:
                targetAngles[2] = 90;
                break;
            case 3:
                targetAngles[3] = 90;
                break;
            case 4:
                targetAngles[4] = 90;
                break;
            case 5:
                targetAngles[5] = 90;
                break;
            case 6:
                targetAngles[6] = 90;
                break;          
            default:
                break;
        }
    }
    
    // Initialize the motor pins
    motor_init();
    
    // Create the arm control task
    Serial.println("xTaskCreatePinnedToCore armControlTaskFunction ");
    delay(100);
        xTaskCreatePinnedToCore(
            armControlTaskFunction, // Task function
            "Arm Control Task",     // Task name
            2048,                   // Stack size (in bytes)
            NULL,                   // Task parameters
            1,                      // Task priority
            &armControlTask,        // Task handle
            0                       // Core ID (0 or 1)
    );

    // Create the serial communication task
    Serial.println("xTaskCreatePinnedToCore serialCommunicationTaskFunction ");
    delay(100);
    xTaskCreatePinnedToCore(
            serialCommunicationTaskFunction, // Task function
            "Serial Communication Task",     // Task name
            2048,                            // Stack size (in bytes)
            NULL,                            // Task parameters
            3,                               // Task priority
            &serialCommunicationTask,        // Task handle
            1                                // Core ID (0 or 1)
    );

    // Create the serial communication task
    Serial.println("xTaskCreatePinnedToCore serialWriterTaskFunction ");
    delay(100);
    xTaskCreatePinnedToCore(
            serialWriterTaskFunction, // Task function
            "Serial Writer Task",     // Task name
            2048,                     // Stack size (in bytes)
            NULL,                     // Task parameters
            2,                        // Task priority
            &serialWriterTask,        // Task handle
            0                         // Core ID (0 or 1)
    );
    delay(100);

    // Create the crane control task
    Serial.println("xTaskCreatePinnedToCore craneControlTaskFunction ");
    delay(100);
    xTaskCreatePinnedToCore(
            craneControlTaskFunction, // Task function
            "Crane Control Task",     // Task name
            2048,                     // Stack size (in bytes)
            NULL,                     // Task parameters
            1,                        // Task priority
            &craneControlTask,        // Task handle
            0                         // Core ID (0 or 1)
    );
    delay(100);

}

void loop() {
    // put your main code here, to run repeatedly:
}

// Task function for arm control
void armControlTaskFunction(void *parameter) {
    // Create an instance of ArmManager

    // uint8_t currentAngles[ArmManager::NUM_SERVOS];
    ArmManager armManager(NUM_OF_SERVOS, servoMinAngles, servoMaxAngles);

    Serial.println("armControlTaskFunction start");

    for (;;) {
        // Control the robot arm using ArmManager
        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
            armManager.setServoTargetAngle(i, targetAngles[i]);
        }

        armManager.moveArm();
        // armManager.printStatus();
        armManager.getCurrentAngles(currentAngles);
        armAction = ArmAction::NO_ACTION;

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
    }
}

// Task function for crane control
void craneControlTaskFunction(void *parameter) {
    // Create an instance of CraneManager
    // CraneManager craneManager(NUM_OF_CRANE_MOTOR);

    Serial.println("craneControlTaskFunction start");

    for (;;) {
        // Control the crane using CraneManager
        for (uint8_t i = 0; i < NUM_OF_CRANE_MOTOR; i++) {
            // craneManager.setCraneState(i, CraneState[i]);
            if (i == 0) {
                DC_motor_execute(CraneState[i]);
            } else if (i == 1) {
                Stepper_motor_execute(CraneState[i]);
            }
        }

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
    }
}

// Task function for serial reader
void serialCommunicationTaskFunction(void *parameter) {
    Serial.println("serialCommunicationTaskFunction start ");
    String jsonString = "";
    for (;;) {
        if (Serial.available()) {
            // Read the incoming serial data
            jsonString = Serial.readStringUntil('\n');
            // String jsonString = Serial.readString();
            Serial.println("receive:");
            Serial.println(jsonString);
            // Parse the JSON string
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, jsonString);

            // Check if the JSON parsing was successful
            if (error) {
                Serial.print("JSON parsing error: ");
                Serial.println(error.c_str());
            } else {
                // Check if the parsed JSON document contains the "servo_target_angles" array
                if (doc.containsKey("servo_target_angles")) {
                    // Get the "servo_target_angles" array from the JSON document
                    JsonArray servoAngles = doc["servo_target_angles"].as<JsonArray>();

                    // Check if the array size matches the number of servos
                    if (servoAngles.size() == NUM_OF_SERVOS) {
                        // Update the target angles in the ArmManager
                        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
                            targetAngles[i] = servoAngles[i];
                        }
                    } else {
                        Serial.println("Invalid number of servo target angles.");
                    }
                } else if (doc.containsKey("crane_state")) {
                    // Get the crane state from the JSON document
                    JsonArray target_craneState = doc["crane_state"].as<JsonArray>();
                    
                    uint8_t i = 0;
                    for (const auto &value: target_craneState) {
                        float state = value.as<float>();
                        CraneState[i] = state;
                        i++;
                    }
                
                } else {
                    Serial.println("Missing servo_target_angles in JSON");
                }
            }
        }

        // Wait for some time before the next iteration
        vTaskDelay(SERIAL_READ_DELAY / portTICK_PERIOD_MS);
    }
}

// Task function for serial writer
void serialWriterTaskFunction(void *parameter) {
    Serial.println("serialWriterTaskFunction start");

    for (;;) {
        StaticJsonDocument<512> doc;
        // Populate the JSON document with the current angles
        JsonArray servoAngles = doc.createNestedArray("servo_current_angles");
        JsonArray craneState = doc.createNestedArray("crane_current_state");
        // armManager.getCurrentAngles(currentAngles);

        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
            servoAngles.add(currentAngles[i]);
        }

        for (uint8_t i = 0; i < NUM_OF_CRANE_MOTOR; i++) {
            craneState.add(CraneState[i]);
        }
        // Serialize the JSON document to a string
        String jsonString;
        serializeJson(doc, jsonString);

        // Print the JSON string
        Serial.println(jsonString);



        // Read incoming serial data

        // Wait for some time befCraneStateore the next iteration
        vTaskDelay(SERIAL_WRITE_DELAY / portTICK_PERIOD_MS);
    }
}

void motor_init() {
    // Set the motor pins as output
    pinMode(PIN_MOTOR1, OUTPUT);
    pinMode(PIN_MOTOR2, OUTPUT);

    pinMode(PIN_PULSE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);

    for (uint8_t i = 0; i < NUM_OF_CRANE_MOTOR; i++) {
        CraneState[i] = 0;
    }
}

void DC_motor_execute(int state) {
    // Execute the motor state
    switch (state) {
        case 0:
            // Stop the motor
            digitalWrite(PIN_MOTOR1, HIGH);
            digitalWrite(PIN_MOTOR2, LOW);
            break;
        case 1:
            // Move the motor up
            digitalWrite(PIN_MOTOR1, LOW);
            digitalWrite(PIN_MOTOR2, LOW);
            break;
        case -1:
            // Move the motor down
            digitalWrite(PIN_MOTOR1, HIGH);
            digitalWrite(PIN_MOTOR2, HIGH);
            break;
        default:
            break;
    }
}

void Stepper_motor_execute(int state) {
    // Execute the motor state
    switch (state) {
        case 0:
            // Stop the motor
            digitalWrite(PIN_ENA, HIGH);
            break;
        case 1:
            // Move the motor forward            
            digitalWrite(PIN_ENA, LOW);
            digitalWrite(PIN_DIR, HIGH);
            digitalWrite(PIN_PULSE, HIGH);
            delayMicroseconds(UPDATE_STEPPER_DELAY);
            digitalWrite(PIN_PULSE, LOW);
            delayMicroseconds(UPDATE_STEPPER_DELAY);         
            break;
        case -1:
            // Move the motor backward
            digitalWrite(PIN_ENA, LOW);
            digitalWrite(PIN_DIR, LOW);
            digitalWrite(PIN_PULSE, HIGH);
            delayMicroseconds(UPDATE_STEPPER_DELAY);            
            digitalWrite(PIN_PULSE, LOW);
            delayMicroseconds(UPDATE_STEPPER_DELAY);            
            break;
        default:
            break;
    }
}