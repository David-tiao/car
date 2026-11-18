#include "race_vision_protocol.h"
#include <string.h>

uint8_t RaceVision_ParseSeq(const char *line, char code[RACE_TASK_CODE_SIZE])
{
    char candidate[RACE_TASK_CODE_SIZE];
    uint8_t colors[2][3], positions[2][3], index;
    const char *cursor;
    if (strncmp(line, "#SEQ,", 5U) != 0) return 0U;
    cursor = line + 5;
    for (index = 0U; index < 12U; index++) {
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (*cursor < '1' || *cursor > '6') return 0U;
        candidate[index + index / 3U] = *cursor++;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (index != 11U) {
            if (*cursor++ != ',') return 0U;
        } else if (*cursor != '\0') {
            return 0U;
        }
    }
    candidate[3] = '+';
    candidate[7] = '+';
    candidate[11] = '+';
    candidate[15] = '\0';
    if (!Race_ParseTask(candidate, colors, positions)) return 0U;
    memcpy(code, candidate, sizeof(candidate));
    return 1U;
}

void RaceVisionDecoder_Reset(RaceVisionDecoder *decoder)
{
    memset(decoder, 0, sizeof(*decoder));
}

/* One frame collector shared by QR compatibility and all visual packets. */
static uint8_t decoder_line(RaceVisionDecoder *decoder, uint8_t byte,
                            char line[RACE_VISION_LINE_MAX])
{
    if (byte == '\r' || byte == '\n') {
        uint8_t complete = decoder->collecting && !decoder->dropping;
        decoder->line[decoder->length] = '\0';
        if (complete) memcpy(line, decoder->line, decoder->length + 1U);
        RaceVisionDecoder_Reset(decoder);
        return complete;
    }
    if (decoder->dropping) return 0U;
    if (!decoder->collecting) {
        if (byte != '#') return 0U;
        decoder->collecting = 1U;
    }
    if ((byte < 32U && byte != '\t') || byte > 126U ||
        decoder->length >= RACE_VISION_LINE_MAX - 1U) {
        decoder->dropping = 1U;
        return 0U;
    }
    decoder->line[decoder->length++] = (char)byte;
    return 0U;
}

uint8_t RaceVisionDecoder_Push(RaceVisionDecoder *decoder, uint8_t byte,
                              char code[RACE_TASK_CODE_SIZE])
{
    char line[RACE_VISION_LINE_MAX];
    return decoder_line(decoder, byte, line) && RaceVision_ParseSeq(line, code);
}

/* Strict parser. Never trust positions or color without checking packet form. */
#include <stdlib.h>
#include <limits.h>

static uint8_t number_field(const char **cursor, long lo, long hi,
                            char separator, int16_t *value)
{
    char *end;
    long n;
    if (**cursor < '0' || **cursor > '9') return 0U;
    n = strtol(*cursor, &end, 10);
    if (end == *cursor || n < lo || n > hi || *end != separator) return 0U;
    *value = (int16_t)n;
    *cursor = end + (separator != '\0');
    return 1U;
}

uint8_t RaceVision_ParseEvent(const char *line, RaceVisionEvent *event)
{
    const char *p;
    int16_t a, b, c, d, e, f;
    if (!line || !event) return 0U;
    memset(event, 0, sizeof(*event));
    if (RaceVision_ParseSeq(line, event->task_code)) {
        event->kind = VISION_SEQ;
        return 1U;
    }
    if (strcmp(line, "#DISK,1") == 0 || strcmp(line, "#DISK,0") == 0) {
        event->kind = VISION_DISK;
        event->disk_locked = (uint8_t)(line[6] == '1');
        return 1U;
    }
    if (strcmp(line, "#TAKE_OK") == 0) {
        event->kind = VISION_TAKE_ACK;
        return 1U;
    }
    if (strcmp(line, "#ARM_OK,1") == 0 || strcmp(line, "#ARM_OK,0") == 0) {
        event->kind = VISION_ARM_ACK;
        event->arm_enabled = (uint8_t)(line[8] == '1');
        return 1U;
    }
    if (strncmp(line, "#SLOT,", 6U) == 0) {
        p = line + 6;
        if (!number_field(&p, 0, 319, ',', &a) ||
            !number_field(&p, 0, 239, ',', &b) ||
            !number_field(&p, 1, 165, '\0', &c)) return 0U;
        event->kind = VISION_SLOT;
        event->x = a; event->y = b; event->radius = c;
        return 1U;
    }
    if (strncmp(line, "#OBJ,", 5U) == 0) {
        p = line + 5;
        if (!number_field(&p, 1, 6, ',', &a) ||
            !number_field(&p, 0, 319, ',', &b) ||
            !number_field(&p, 0, 239, ',', &c) ||
            !number_field(&p, 1, 200, ',', &d) ||
            !number_field(&p, 1, 200, ',', &e) ||
            !number_field(&p, 1, 32000, ',', &f)) return 0U;
        /* Last field is distance (0 until calibrated). */
        if (!number_field(&p, 0, 32000, '\0', &f)) return 0U;
        event->kind = VISION_OBJ;
        event->color = (uint8_t)a;
        event->x = b; event->y = c;
        return 1U;
    }
    return 0U;
}

uint8_t RaceVisionDecoder_PushEvent(RaceVisionDecoder *decoder, uint8_t byte,
                                    RaceVisionEvent *event)
{
    char line[RACE_VISION_LINE_MAX];
    return decoder_line(decoder, byte, line) && RaceVision_ParseEvent(line, event);
}
