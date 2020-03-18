#define PKT_OVERHEAD  5 /* 5 bytes of overhead for each packet */
#define PKT_MAX_PLEN 64 /* max packet length, give a data length of 59 */
#define PKT_MAX_DLEN (PKT_MAX_PLEN - PKT_OVERHEAD) /* max packet data length */

#define PKT_N_LEADIN    4     /* no. bytes of leadin */
#define PKT_LEADIN_BYTE 0x99
#define PKT_START_BYTE  0xaa

#define PKT_PREAMBLE    6   //The number of bytes from LEADIN to LENGTH

#define PKT_LEADIN      0x00
#define PKT_START       0x01
#define PKT_DEST        0x02
#define PKT_SOURCE      0x03
#define PKT_FLAG        0x04
#define PKT_LENGTH      0x05
#define PKT_DATA        0x06
#define PKT_CHECKSUM    0x07

#define CRC_ERROR       0
#define PROCESSING      1
#define DONE            2
#define ABORTED         3

#define DATA_REQUEST    0xFE
#define IDLE            0x00

#define FOO_BAR         0xC2

/* packet flags */
#define PKTF_ACK        0x01
#define PKTF_NACK       0x02
#define PKTF_REQACK     0x04
#define PKTF_REQID      0x08
#define PKTF_COMMAND    0x10
#define PKTF_RESERVED   0x20
#define PKTF_SENSOR_BAD 0x40
#define PKTF_USER2      0x80

