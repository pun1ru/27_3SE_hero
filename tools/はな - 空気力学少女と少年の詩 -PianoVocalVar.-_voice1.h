#include <stdint.h>

typedef struct
{
    uint16_t freq;
    uint32_t duration;
    uint32_t gap;
} Note;

static const Note melody_voice_1[] =
{
    {1480, 93, 6216},
    {1245, 128, 20143},
    {1109, 185, 201519},
    {1976, 116, 0},
};

static const uint32_t melody_voice_1_count =
    sizeof(melody_voice_1) / sizeof(melody_voice_1[0]);
