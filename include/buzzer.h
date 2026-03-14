#pragma once
#include <Arduino.h>

#define BUZZER_PIN 30
struct buzzer
{
    const int *melody;
    const int *duration;
    int noteCount;
    int currentNote;
    unsigned long noteStartTime;
    bool isPlaying;
    bool inGap;
};

const int MELODY_BOOT[] = {392, 440, 523};
const int DURATION_BOOT[] = {100, 100, 300};

const int MELODY_ARMED[] = {349, 415};
const int MELODY_DISARMED[] = {415, 349};
const int DURATION_ARMED_DISARMED[] = {100, 200};

const int MELODY_LOWBAT[] = {261, 261, 261};
const int DURATION_LOWBAT[] = {100, 100, 100};

buzzer Buzzer = {nullptr, nullptr, 0, 0, 0, false, false};

void playMelody(const int *melody, const int *duration, int count)
{
    noTone(BUZZER_PIN);
    Buzzer.melody = melody;
    Buzzer.duration = duration;
    Buzzer.noteCount = count;
    Buzzer.currentNote = 0;
    Buzzer.isPlaying = true;
    Buzzer.inGap = false;
    Buzzer.noteStartTime = millis();

    if (melody[0] > 0)
    {
        tone(BUZZER_PIN, melody[0]);
    }
}

void updateBuzzer()
{
    if (!Buzzer.isPlaying)
        return;

    unsigned long now = millis();
    int noteDuration = Buzzer.duration[Buzzer.currentNote];
    int toneDuration = noteDuration * 0.9;
    int gapDuration = noteDuration - toneDuration;

    if (!Buzzer.inGap)
    {
        // Currently in tone phase
        if (now - Buzzer.noteStartTime >= (unsigned long)toneDuration)
        {
            noTone(BUZZER_PIN);
            Buzzer.inGap = true;
            Buzzer.noteStartTime = now;
        }
    }
    else
    {
        // Currently in gap phase
        if (now - Buzzer.noteStartTime >= (unsigned long)gapDuration)
        {
            Buzzer.currentNote++;
            if (Buzzer.currentNote >= Buzzer.noteCount)
            {
                // Melody finished
                Buzzer.isPlaying = false;
                return;
            }
            // Start next note
            Buzzer.inGap = false;
            Buzzer.noteStartTime = now;
            if (Buzzer.melody[Buzzer.currentNote] > 0)
            {
                tone(BUZZER_PIN, Buzzer.melody[Buzzer.currentNote]);
            }
            // If note is 0 (rest), just leave noTone active
        }
    }
}
