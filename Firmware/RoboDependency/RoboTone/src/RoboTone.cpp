#include <Arduino.h>
#include "RoboTone.h"

// Input sequence for the first 20 notes of Chopin's "Funeral March" with rounded frequencies
Buzz melody_funeral[20] = {
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {98, 500, 0, 0},  // G2 (98.00 Hz rounded to 98)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {147, 500, 0, 0}, // D3 (146.83 Hz rounded to 147)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {175, 500, 0, 0}, // F3 (174.61 Hz rounded to 175)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {98, 500, 0, 0},  // G2 (98.00 Hz rounded to 98)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {147, 500, 0, 0}, // D3 (146.83 Hz rounded to 147)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {175, 500, 0, 0}, // F3 (174.61 Hz rounded to 175)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {185, 500, 0, 0}, // F♯3 (185.00 Hz rounded to 185)
    {117, 500, 0, 0}, // B♭2 (116.54 Hz rounded to 117)
    {175, 500, 0, 0}  // F3 (174.61 Hz rounded to 175)
};

// PI2NOS timeout melody (commonly reproduced approximation)
// No pause between notes
Buzz melody_pi2nos[13] = {
    {392, 200, 0, 0}, // G4 (392.00 Hz)
    {440, 200, 0, 0}, // A4 (440.00 Hz)
    {523, 200, 0, 0}, // C5 (523.25 Hz -> 523)
    {440, 200, 0, 0}, // A4 (440.00 Hz)
    {392, 200, 0, 0}, // G4 (392.00 Hz)
    {330, 200, 0, 0}, // E4 (329.63 Hz -> 330)
    {294, 200, 0, 0}, // D4 (293.66 Hz -> 294)
    {330, 200, 0, 0}, // E4
    {392, 200, 0, 0}, // G4
    {440, 200, 0, 0}, // A4
    {392, 200, 0, 0}, // G4
    {330, 220, 0, 0}, // E4
    {262, 320, 0, 0}  // C4 (261.63 Hz -> 262) final tone
};

// Hz, duration, repeat ,pauze
Buzz tones[20] = {
    {50, 100, 0, 0},   // 0
    {100, 100, 0, 0},  // 1
    {500, 100, 0, 0},  // 2
    {1000, 100, 0, 0}, // 3
    {1500, 100, 0, 0}, // 4
    {2000, 100, 0, 0}, // 5
    {2500, 100, 0, 0}, // 6
    {3000, 100, 0, 0}, // 7
    {150, 500, 0, 50}, // 8
    {100, 500, 0, 50}, // 9
    {2500, 25, 0, 0},  // 10
    {98, 100, 0, 0},   // 11
    {117, 100, 0, 0},  // 12
    {147, 100, 0, 0},  // 13
    {117, 100, 0, 0},  // 14
    {175, 100, 0, 0},  // 15
    {117, 100, 0, 0},  // 16
    {185, 100, 0, 0},  // 17
    {117, 100, 0, 0},  // 18
    {175, 100, 0, 0}   // 19
};

void beep(int sound, QueueHandle_t buzzer)
{
    Buzz Data;
    switch (sound)
    {
    case -1:
        xQueueSend(buzzer, (void *)&tones[8], 10);
        xQueueSend(buzzer, (void *)&tones[9], 10);
        xQueueSend(buzzer, (void *)&tones[10], 10);
        // for (int i = 0; i < 20; i++)
        // {
        //     xQueueSend(buzzer, (void *)&melody[i], 10);
        // }
        break;
    case 1:
        for (int i = 3; i < 6; i++)
        {
            xQueueSend(buzzer, (void *)&tones[i], 10);
        }
        break;
    case 2:
        for (int i = 6; i > 3; i--)
        {
            xQueueSend(buzzer, (void *)&tones[i], 10);
        }
        break;
    case 3:
        for (int i = 3; i > 0; i--)
        {
            xQueueSend(buzzer, (void *)&tones[i], 10);
        }
        break;
    case 4:
        for (int i = 0; i < 13; i++)
        {
            xQueueSend(buzzer, (void *)&melody_pi2nos[i], 10);
        }
        break;
    case 10:
        xQueueSend(buzzer, (void *)&tones[10], 5);
        break;

    case 500:
        Data.hz = 500;
        Data.repeat = 0;
        Data.pause = 0;
        Data.duration = 100;
        xQueueSend(buzzer, (void *)&Data, 10); // update buzzer
        break;
    case 1000:
        Data.hz = 1000;
        Data.repeat = 0;
        Data.pause = 0;
        Data.duration = 100;
        xQueueSend(buzzer, (void *)&Data, 10); // update buzzer
        break;
    case 1500:
        Data.hz = 1500;
        Data.repeat = 0;
        Data.pause = 0;
        Data.duration = 100;
        xQueueSend(buzzer, (void *)&Data, 10); // update buzzer
        break;
    case 2000:
        Data.hz = 2000;
        Data.repeat = 0;
        Data.pause = 0;
        Data.duration = 100;
        xQueueSend(buzzer, (void *)&Data, 10); // update buzzer
        break;

    default:
        Data.hz = 500;
        Data.repeat = 0;
        Data.pause = 100;
        Data.duration = 1000;
        xQueueSend(buzzer, (void *)&Data, 10); // update buzzer
        break;
    }
}
