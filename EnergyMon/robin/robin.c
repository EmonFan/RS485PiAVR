#include "robin.h"

static uint8_t pkt_buf[PKT_MAX_PLEN];
static uint8_t pkt_ibuf[PKT_MAX_PLEN];
static uint8_t pkt_send_buf[PKT_MAX_PLEN + PKT_N_LEADIN];

