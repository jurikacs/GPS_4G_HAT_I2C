#include <iostream>

#include "../Drivers/Qwiic_Button.h"

QwiicButton button;
uint8_t brightness = 100;   //The brightness to set the LED to when the button is pushed
                            //Can be any value between 0 (off) and 255 (max)

static void setup() {
  printf("Qwiic button example Light When Pressed\n");

  //check if button will acknowledge over I2C
  if (button.begin() == false) {
    printf("Device did not acknowledge! Freezing.\n");
    return;
  }
  printf("Button acknowledged.");

  //Start with the LED off
  button.LEDoff();
}

static void loop() {
  //check if button is pressed, and tell us if it is!
  if (button.isPressed() == true) {
    printf("The button is pressed!\n");
    button.LEDon(brightness);
    while(button.isPressed() == true) {
      sleep_ms(500);  //wait for user to stop pressing
    }
    printf("The button is not pressed.\n");
    button.LEDoff();
  }
  sleep_ms(20); //Don't hammer too hard on the I2C bus
}

void (*setup_LightWhenPressed)(void) = setup;
void (*loop_LightWhenPressed)(void) = loop;
