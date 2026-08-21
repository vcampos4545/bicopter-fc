#include "radio_packet.h"

#include <math.h>
#include <string.h>

// --- little-endian byte-buffer helpers -------------------------------------------------------

static void put_u8(uint8_t *buf, size_t *pos, uint8_t v)
{
    buf[*pos] = v;
    *pos += 1;
}

static void put_u32(uint8_t *buf, size_t *pos, uint32_t v)
{
    buf[*pos + 0] = (uint8_t)(v >> 0);
    buf[*pos + 1] = (uint8_t)(v >> 8);
    buf[*pos + 2] = (uint8_t)(v >> 16);
    buf[*pos + 3] = (uint8_t)(v >> 24);
    *pos += 4;
}

static void put_i64(uint8_t *buf, size_t *pos, int64_t v)
{
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; i++) {
        buf[*pos + i] = (uint8_t)(u >> (8 * i));
    }
    *pos += 8;
}

static void put_f32(uint8_t *buf, size_t *pos, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put_u32(buf, pos, bits);
}

static uint8_t get_u8(const uint8_t *buf, size_t *pos)
{
    uint8_t v = buf[*pos];
    *pos += 1;
    return v;
}

static uint32_t get_u32(const uint8_t *buf, size_t *pos)
{
    uint32_t v = (uint32_t)buf[*pos + 0] | ((uint32_t)buf[*pos + 1] << 8) |
                 ((uint32_t)buf[*pos + 2] << 16) | ((uint32_t)buf[*pos + 3] << 24);
    *pos += 4;
    return v;
}

static int64_t get_i64(const uint8_t *buf, size_t *pos)
{
    uint64_t u = 0;
    for (int i = 0; i < 8; i++) {
        u |= ((uint64_t)buf[*pos + i]) << (8 * i);
    }
    *pos += 8;
    return (int64_t)u;
}

static float get_f32(const uint8_t *buf, size_t *pos)
{
    uint32_t bits = get_u32(buf, pos);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

// Simple additive checksum (sum of all preceding bytes, mod 256) -- not cryptographic, just a
// cheap guard against a foreign/corrupted packet passing the magic-byte check by coincidence. See
// radio_packet.h's file header for why this exists alongside ESP-NOW's own transport-level CRC.
static uint8_t checksum(const uint8_t *buf, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + buf[i]);
    }
    return sum;
}

// --- command packet --------------------------------------------------------------------------

size_t radio_command_packet_serialize(const radio_command_packet_t *packet, uint8_t *out_buf,
                                       size_t buf_len)
{
    if (packet == NULL || out_buf == NULL || buf_len < RADIO_COMMAND_PACKET_WIRE_SIZE) {
        return 0;
    }

    size_t pos = 0;
    put_u8(out_buf, &pos, RADIO_PACKET_MAGIC);
    put_u8(out_buf, &pos, RADIO_PACKET_VERSION);
    put_u32(out_buf, &pos, packet->sequence);
    put_i64(out_buf, &pos, packet->timestamp_us);
    put_f32(out_buf, &pos, packet->throttle);
    put_f32(out_buf, &pos, packet->roll);
    put_f32(out_buf, &pos, packet->pitch);
    put_f32(out_buf, &pos, packet->yaw);
    put_u8(out_buf, &pos, packet->arm ? 1 : 0);
    put_u8(out_buf, &pos, packet->flight_mode);
    put_u8(out_buf, &pos, checksum(out_buf, pos));

    return pos;
}

bool radio_command_packet_parse(const uint8_t *data, size_t len, radio_command_packet_t *out_packet)
{
    if (data == NULL || out_packet == NULL || len != RADIO_COMMAND_PACKET_WIRE_SIZE) {
        return false;
    }
    if (checksum(data, len - 1) != data[len - 1]) {
        return false;
    }

    size_t pos = 0;
    uint8_t magic = get_u8(data, &pos);
    uint8_t version = get_u8(data, &pos);
    if (magic != RADIO_PACKET_MAGIC || version != RADIO_PACKET_VERSION) {
        return false;
    }

    radio_command_packet_t packet = { 0 };
    packet.sequence = get_u32(data, &pos);
    packet.timestamp_us = get_i64(data, &pos);
    packet.throttle = get_f32(data, &pos);
    packet.roll = get_f32(data, &pos);
    packet.pitch = get_f32(data, &pos);
    packet.yaw = get_f32(data, &pos);
    packet.arm = get_u8(data, &pos) != 0;
    packet.flight_mode = get_u8(data, &pos);

    if (!isfinite(packet.throttle) || !isfinite(packet.roll) || !isfinite(packet.pitch) ||
        !isfinite(packet.yaw)) {
        return false;
    }

    *out_packet = packet;
    return true;
}

// --- telemetry packet -------------------------------------------------------------------------

size_t radio_telemetry_packet_serialize(const radio_telemetry_packet_t *packet, uint8_t *out_buf,
                                         size_t buf_len)
{
    if (packet == NULL || out_buf == NULL || buf_len < RADIO_TELEMETRY_PACKET_WIRE_SIZE) {
        return 0;
    }

    size_t pos = 0;
    put_u8(out_buf, &pos, RADIO_PACKET_MAGIC);
    put_u8(out_buf, &pos, RADIO_PACKET_VERSION);
    put_u32(out_buf, &pos, packet->sequence);
    put_i64(out_buf, &pos, packet->timestamp_us);
    put_u8(out_buf, &pos, packet->armed ? 1 : 0);
    put_u8(out_buf, &pos, packet->flight_mode);
    put_f32(out_buf, &pos, packet->packet_loss_percent);
    put_u8(out_buf, &pos, checksum(out_buf, pos));

    return pos;
}

bool radio_telemetry_packet_parse(const uint8_t *data, size_t len,
                                   radio_telemetry_packet_t *out_packet)
{
    if (data == NULL || out_packet == NULL || len != RADIO_TELEMETRY_PACKET_WIRE_SIZE) {
        return false;
    }
    if (checksum(data, len - 1) != data[len - 1]) {
        return false;
    }

    size_t pos = 0;
    uint8_t magic = get_u8(data, &pos);
    uint8_t version = get_u8(data, &pos);
    if (magic != RADIO_PACKET_MAGIC || version != RADIO_PACKET_VERSION) {
        return false;
    }

    radio_telemetry_packet_t packet = { 0 };
    packet.sequence = get_u32(data, &pos);
    packet.timestamp_us = get_i64(data, &pos);
    packet.armed = get_u8(data, &pos) != 0;
    packet.flight_mode = get_u8(data, &pos);
    packet.packet_loss_percent = get_f32(data, &pos);

    if (!isfinite(packet.packet_loss_percent)) {
        return false;
    }

    *out_packet = packet;
    return true;
}

// --- sequence / loss tracking -----------------------------------------------------------------

bool radio_seq_is_newer(uint32_t sequence, uint32_t last_sequence, bool has_last_sequence)
{
    if (!has_last_sequence) {
        return true;
    }
    return (int32_t)(sequence - last_sequence) > 0;
}

void radio_loss_tracker_reset(radio_loss_tracker_t *tracker)
{
    tracker->has_last_sequence = false;
    tracker->last_sequence = 0;
    tracker->received_count = 0;
    tracker->expected_count = 0;
}

void radio_loss_tracker_record(radio_loss_tracker_t *tracker, uint32_t sequence)
{
    uint32_t gap = tracker->has_last_sequence ? (sequence - tracker->last_sequence) : 1;

    tracker->received_count += 1;
    tracker->expected_count += gap;
    tracker->last_sequence = sequence;
    tracker->has_last_sequence = true;
}

float radio_loss_tracker_loss_percent(const radio_loss_tracker_t *tracker)
{
    if (tracker->expected_count == 0) {
        return 0.0f;
    }
    float received = (float)tracker->received_count;
    float expected = (float)tracker->expected_count;
    return 100.0f * (1.0f - received / expected);
}

// --- staleness -----------------------------------------------------------------------------

bool radio_is_stale(int64_t last_valid_rx_timestamp_us, int64_t now_us, int64_t timeout_us)
{
    return (now_us - last_valid_rx_timestamp_us) > timeout_us;
}
