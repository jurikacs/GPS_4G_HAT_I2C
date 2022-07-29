// GPS_4G_HAT_I2C.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//

#include <iostream>

#include "Drivers/i2c_RPi.h"

typedef void (*fptr)(void);
#ifdef WIN
    #include <conio.h>
    #define kbhit _kbhit
#else
    extern bool kbhit(void);
#endif

extern fptr setup_BasicReadings;
extern fptr loop_BasicReadings;

extern fptr setup_LightWhenPressed;
extern fptr loop_LightWhenPressed;

extern fptr setup_MC3479;
extern fptr loop_MC3479;


fptr setup;
fptr loop;

const char* menu[] = {
    "A - Acceleration sensor sketch",
    "B - Button sketch",
    "P - Pressure sensor sketch",
    "Q - Quit",
};

void show_menu(const char** menu, int length) {

    printf("\r\nType sketch ID:\r\n\r\n");
    for (int i = 0; i < length; i++) {
        printf("%s\r\n", menu[i]);
    }
    printf("\r\n");
}

int main()
{
    std::cout << "GPS-4G-HAT Paspberry CLI console\n";
    show_menu(menu, sizeof(menu) / sizeof(char*));
    std::cout << "press return to end sketch\n";

    int sketch_no = -1;
    while (true) {
        
        int ch = EOF;
        while (ch == EOF) {
            ch = getchar();
        }
        
        switch (ch & 0xDF) {
        case 'A':
            sketch_no = 0;
            setup = setup_MC3479;
            loop = loop_MC3479;
            break;
        case 'B':
            sketch_no = 1;
            setup = setup_LightWhenPressed;
            loop = loop_LightWhenPressed;
            break;
        case 'P':
            sketch_no = 2;
            setup = setup_BasicReadings;
            loop = loop_BasicReadings;
            break;
        case 'Q':
            exit(0);
        case '\n':
        case '\r':
           continue;
        default:
            printf("pressed %c\r\n", ch);
            continue;
        }
        printf("sketch %s selected\r\n", menu[sketch_no]);
        setup();
        while (!kbhit()){
           loop();
        } 
        printf("end of sketch %s\r\n", menu[sketch_no]);
        show_menu(menu, sizeof(menu) / sizeof(char*));

        sleep_ms(1000);
        ch = getchar();
    }
}
