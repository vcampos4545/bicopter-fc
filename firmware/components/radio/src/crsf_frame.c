#include "crsf_frame.h"

#include <string.h>

// --- CRC8/DVB-S2 --------------------------------------------------------------------------------

uint8_t crsf_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// --- frame sync ----------------------------------------------------------------------------------

void crsf_frame_sync_reset(crsf_frame_sync_t *sync)
{
    sync->len = 0;
}

bool crsf_frame_sync_push_byte(crsf_frame_sync_t *sync, uint8_t byte, size_t *out_frame_len)
{
    if (sync->len == 0) {
        if (byte != CRSF_SYNC_BYTE) {
            return false; // garbage before sync; discard
        }
        sync->buf[sync->len++] = byte;
        return false;
    }

    if (sync->len == 1) {
        // Length byte: covers type(1) + payload + crc(1), so >= 2. Reject an implausible declared
        // length (would overflow our fixed scratch buffer, or claims a payload smaller than any
        // real CRSF frame type carries) and resync -- treating `byte` itself as a possible new
        // sync byte, in case the previous byte was noise that happened to match CRSF_SYNC_BYTE.
        if (byte < 2 || (size_t)(byte + 2) > CRSF_MAX_FRAME_SIZE) {
            sync->len = 0;
            if (byte == CRSF_SYNC_BYTE) {
                sync->buf[sync->len++] = byte;
            }
            return false;
        }
        sync->buf[sync->len++] = byte;
        return false;
    }

    sync->buf[sync->len++] = byte;
    size_t declared_total_len = 2 + sync->buf[1];
    if (sync->len >= declared_total_len) {
        *out_frame_len = sync->len;
        sync->len = 0; // ready for the next frame regardless of how the caller validates this one
        return true;
    }
    return false;
}

// --- RC_CHANNELS_PACKED ---------------------------------------------------------------------------

bool crsf_parse_rc_channels_frame(const uint8_t *frame, size_t frame_len,
                                   crsf_rc_channels_t *out_channels)
{
    if (frame == NULL || out_channels == NULL || frame_len != CRSF_RC_CHANNELS_FRAME_SIZE) {
        return false;
    }
    if (frame[0] != CRSF_SYNC_BYTE) {
        return false;
    }
    if (frame[1] != frame_len - 2) {
        return false;
    }
    if (frame[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        return false;
    }

    // CRC8 covers type + payload: everything after the length byte, excluding the CRC byte itself.
    uint8_t computed_crc = crsf_crc8(&frame[2], frame_len - 3);
    if (computed_crc != frame[frame_len - 1]) {
        return false;
    }

    const uint8_t *payload = &frame[3];
    uint32_t bitbuf = 0;
    int bitcount = 0;
    size_t byte_idx = 0;
    crsf_rc_channels_t channels;
    for (int ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
        while (bitcount < 11) {
            bitbuf |= ((uint32_t)payload[byte_idx++]) << bitcount;
            bitcount += 8;
        }
        channels.raw[ch] = (uint16_t)(bitbuf & 0x7FF);
        bitbuf >>= 11;
        bitcount -= 11;
    }

    *out_channels = channels;
    return true;
}

// --- channel mapping -------------------------------------------------------------------------

static uint16_t channel_raw(const crsf_rc_channels_t *channels, uint8_t index)
{
    if (index >= CRSF_NUM_CHANNELS) {
        return 0;
    }
    return channels->raw[index];
}

static float unipolar_map(uint16_t raw, uint16_t raw_min, uint16_t raw_max)
{
    if (raw_max <= raw_min) {
        return 0.0f;
    }
    float t = ((float)raw - (float)raw_min) / ((float)raw_max - (float)raw_min);
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    return t;
}

static float bipolar_map(uint16_t raw, uint16_t raw_min, uint16_t raw_center, uint16_t raw_max)
{
    float value;
    if (raw >= raw_center) {
        if (raw_max <= raw_center) {
            return 0.0f;
        }
        value = ((float)raw - (float)raw_center) / ((float)raw_max - (float)raw_center);
    } else {
        if (raw_center <= raw_min) {
            return 0.0f;
        }
        value = ((float)raw - (float)raw_center) / ((float)raw_center - (float)raw_min);
    }
    if (value < -1.0f) {
        value = -1.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }
    return value;
}

static uint8_t quantize_positions(uint16_t raw, uint16_t raw_min, uint16_t raw_max,
                                   uint8_t positions)
{
    if (positions <= 1 || raw_max <= raw_min) {
        return 0;
    }
    float t = unipolar_map(raw, raw_min, raw_max);
    int32_t bucket = (int32_t)(t * (float)positions);
    if (bucket >= positions) {
        bucket = positions - 1;
    }
    if (bucket < 0) {
        bucket = 0;
    }
    return (uint8_t)bucket;
}

void crsf_channels_to_command(const crsf_rc_channels_t *channels, const crsf_channel_map_t *map,
                               crsf_command_t *out_command)
{
    if (channels == NULL || map == NULL || out_command == NULL) {
        return;
    }

    crsf_command_t command = { 0 };
    command.throttle =
        unipolar_map(channel_raw(channels, map->throttle_channel), map->raw_min, map->raw_max);
    command.roll = bipolar_map(channel_raw(channels, map->roll_channel), map->raw_min,
                                map->raw_center, map->raw_max);
    command.pitch = bipolar_map(channel_raw(channels, map->pitch_channel), map->raw_min,
                                 map->raw_center, map->raw_max);
    command.yaw = bipolar_map(channel_raw(channels, map->yaw_channel), map->raw_min,
                               map->raw_center, map->raw_max);
    command.arm = channel_raw(channels, map->arm_channel) >= map->arm_threshold;
    command.flight_mode = quantize_positions(channel_raw(channels, map->flight_mode_channel),
                                              map->raw_min, map->raw_max,
                                              map->flight_mode_positions);

    *out_command = command;
}

// --- link-quality estimate --------------------------------------------------------------------

void crsf_loss_tracker_reset(crsf_loss_tracker_t *tracker, int64_t nominal_frame_period_us)
{
    tracker->has_first_frame = false;
    tracker->first_frame_timestamp_us = 0;
    tracker->last_frame_timestamp_us = 0;
    tracker->received_count = 0;
    tracker->nominal_frame_period_us = nominal_frame_period_us;
}

void crsf_loss_tracker_record(crsf_loss_tracker_t *tracker, int64_t now_us)
{
    if (!tracker->has_first_frame) {
        tracker->has_first_frame = true;
        tracker->first_frame_timestamp_us = now_us;
    }
    tracker->last_frame_timestamp_us = now_us;
    tracker->received_count += 1;
}

float crsf_loss_tracker_loss_percent(const crsf_loss_tracker_t *tracker)
{
    if (!tracker->has_first_frame || tracker->nominal_frame_period_us <= 0) {
        return 0.0f;
    }
    int64_t elapsed_us = tracker->last_frame_timestamp_us - tracker->first_frame_timestamp_us;
    if (elapsed_us <= 0) {
        return 0.0f; // only one frame recorded so far; nothing to estimate an interval from yet
    }

    float expected = (float)elapsed_us / (float)tracker->nominal_frame_period_us + 1.0f;
    float received = (float)tracker->received_count;
    if (received >= expected) {
        return 0.0f;
    }
    return 100.0f * (1.0f - received / expected);
}
