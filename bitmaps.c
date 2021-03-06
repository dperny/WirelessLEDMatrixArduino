/*
 * Most of the pixel art in this file was done by CJ Guttormsson
 * 
 * I took it and turned it all into xbm files, which are then crammed
 * into this file. XBM is a good format because it only uses 1 bit per
 * pixel, and is supported by the Adafruit graphics library. This means
 * I can cram a buttload of images into the progmem (like, over a hundred I think).
 */
#include <stdint.h>
#include <avr/pgmspace.h>

#define default_width 24
#define default_height 24

/*
 * Triangle with an exclamation mark in it
 */
#define error_width 24
#define error_height 16
static const uint8_t error_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0xc0, 0x04, 0x00, 0x30, 0x04,
   0x00, 0x0c, 0x04, 0x00, 0x03, 0x04, 0xc0, 0x00, 0x04, 0x20, 0xbe, 0x05,
   0x20, 0xbe, 0x05, 0xc0, 0x00, 0x04, 0x00, 0x03, 0x04, 0x00, 0x0c, 0x04,
   0x00, 0x30, 0x04, 0x00, 0xc0, 0x04, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00 };
   
/*
 * Two eighth notes
 */
#define music_width 24
#define music_height 16
static const uint8_t music_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0xf0, 0xff, 0x00, 0x30, 0xe0, 0x01, 0x30, 0xf0, 0x03,
   0x60, 0xf0, 0x03, 0x60, 0xe0, 0x01, 0x60, 0xc0, 0x00, 0xc0, 0x00, 0x00,
   0xc0, 0x00, 0x00, 0xc0, 0xff, 0x03, 0x00, 0x80, 0x07, 0x00, 0xc0, 0x0f,
   0x00, 0xc0, 0x0f, 0x00, 0x80, 0x07, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00 };

/*
 * Phone symbol
 */
#define phone_width 24
#define phone_height 16
static const uint8_t phone_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3f,
   0x00, 0x80, 0x3f, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x1f,
   0xc0, 0x00, 0x1f, 0xf0, 0x80, 0x0f, 0xfc, 0xe1, 0x07, 0xfc, 0xff, 0x03,
   0xf8, 0xff, 0x00, 0xf8, 0x3f, 0x00, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00 };
   
/*
 * Wireless symbol 33%
 */
#define wifi1_width 24
#define wifi1_height 16
static const uint8_t wifi1_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x07, 0x00, 0xc0, 0x0f, 0x00, 0xe0, 0x07, 0x00, 0xe0, 0x00,
   0x00, 0x70, 0x06, 0x00, 0x70, 0x0f, 0x00, 0x70, 0x0f, 0x00, 0x20, 0x06 };

/*
 * Wireless symbol 66%
 */
#define wifi2_width 24
#define wifi2_height 16
static const uint8_t wifi2_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x07, 0x00, 0xf0, 0x0f, 0x00, 0xfc, 0x07, 0x00, 0x7c, 0x00,
   0x00, 0x1e, 0x07, 0x00, 0xce, 0x0f, 0x00, 0xee, 0x07, 0x00, 0xe7, 0x00,
   0x00, 0x77, 0x06, 0x00, 0x77, 0x0f, 0x00, 0x77, 0x0f, 0x00, 0x22, 0x06 };

/*
 * Wireless symbol 100%
 */
#define wifi3_width 24
#define wifi3_height 16
static const uint8_t wifi3_bits[] PROGMEM = {
   0x00, 0xc0, 0x07, 0x00, 0xf8, 0x0f, 0x00, 0xfe, 0x07, 0x80, 0x3f, 0x00,
   0x80, 0x87, 0x07, 0xc0, 0xf1, 0x0f, 0xc0, 0xfd, 0x07, 0xe0, 0x7c, 0x00,
   0xe0, 0x1e, 0x07, 0xe0, 0xce, 0x0f, 0x70, 0xee, 0x07, 0x70, 0xe7, 0x00,
   0x70, 0x77, 0x06, 0x70, 0x77, 0x0f, 0x70, 0x77, 0x0f, 0x20, 0x22, 0x06 };

/*
 * Bluetooth symbol
 */
#define bluetooth_width 24
#define bluetooth_height 16
static const uint8_t bluetooth_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x01, 0xe0, 0x80, 0x03,
   0xb0, 0xc1, 0x06, 0x18, 0x63, 0x0c, 0x0c, 0x36, 0x18, 0xfe, 0xff, 0x3f,
   0xff, 0xff, 0x7f, 0x00, 0x36, 0x00, 0x00, 0x63, 0x00, 0x80, 0xc1, 0x00,
   0xc0, 0x80, 0x01, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*
 * Speaker with X through it
 */
#define mute_width 24
#define mute_height 16
static const uint8_t mute_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x60, 0x00, 0x06, 0xe0, 0x00, 0x07, 0xc0, 0x81, 0x03,
   0x80, 0xc3, 0x01, 0x00, 0xe7, 0x00, 0x00, 0x7e, 0x00, 0xf8, 0xbd, 0x1f,
   0x10, 0x3c, 0x08, 0x20, 0x7e, 0x04, 0x40, 0xe7, 0x02, 0x80, 0xc3, 0x01,
   0xc0, 0xbd, 0x03, 0xe0, 0x00, 0x07, 0x60, 0xff, 0x06, 0x00, 0x00, 0x00 };

/*
 * Speaker broadcasting sounds
 */
#define unmute_width 24
#define unmute_height 16
static const uint8_t unmute_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x48, 0x24, 0x12, 0x89, 0x24, 0x91,
   0x92, 0x00, 0x49, 0x04, 0x00, 0x20, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f,
   0x10, 0x00, 0x08, 0x20, 0x00, 0x04, 0x40, 0x00, 0x02, 0x80, 0x00, 0x01,
   0x00, 0xff, 0x00, 0x00, 0x81, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00 };

/*
 * Enveolope with an exclamation over it.
 */
#define newmsg_width 24
#define newmsg_height 16
static const uint8_t newmsg_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x80, 0x7f, 0x00, 0x80, 0x61, 0x00, 0x80, 0x52,
   0x00, 0x80, 0x4c, 0x38, 0x80, 0x4c, 0xc4, 0x98, 0x48, 0x02, 0xa5, 0x50,
   0x02, 0xa5, 0x50, 0xc4, 0x98, 0x48, 0x38, 0x80, 0x4c, 0x00, 0x80, 0x4c,
   0x00, 0x80, 0x52, 0x00, 0x80, 0x61, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00 };




