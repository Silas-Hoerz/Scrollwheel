/**
 * Scrollwheel Basic Raw Data Example
 * * This example demonstrates how to initialize the Scrollwheel PCB,
 * read the continuous 360-degree angle from the radial slider,
 * and detect presses on the central touch button.
 * * Data is output via Serial formatted as: "Angle;ButtonState"
 */
#include <Arduino.h>
#include "Scrollwheel.h"

// Define the interrupt pin connected to INT on the JST SH connector
// Adjust this to match your microcontroller setup (e.g., 2 for Arduino Uno)
#define INTERRUPT_PIN 2

Scrollwheel wheel;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("Initializing Scrollwheel...");

    // begin() checks if the internal configuration of the chip matches the library.
    // If it returns false, it means the config was just updated and written to flash.
    if (!wheel.begin(INTERRUPT_PIN))
    {
        Serial.println("Configuration updated. Allowing sensor to reset...");
        delay(500);
    }

    // Set reading mode to Hybrid: Reacts instantly to hardware interrupts,
    // but also polls every 20ms as a fallback to prevent I2C lockups.
    wheel.setUpdateMode(UpdateMode::kHybrid, 10);

    // Optional: Print hardware diagnostics
    wheel.printDiagnostics();
    Serial.println("Ready. Format: [Angle];[Button 0/1]");
}

void loop()
{
    // update() manages the polling and interrupt logic internally
    if (wheel.update())
    {

        int angle = wheel.getSliderAngle(); // Returns 0-359 or -1 if not touched
        bool button_pressed = wheel.isButtonPressed();

        // Print angle (or '#' if not touched)
        if (angle >= 0)
        {
            Serial.print(angle);
        }
        else
        {
            Serial.print("#");
        }

        Serial.print(";");

        // Print button state
        Serial.println(button_pressed ? "1" : "0");
    }
}