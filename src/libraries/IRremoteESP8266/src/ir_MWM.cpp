// Copyright 2018 Brett T. Warden

/// @file
/// @brief Disney Made With Magic (MWM) Support
/// derived from ir_Lasertag.cpp
/// @see https://github.com/crankyoldgit/IRremoteESP8266/pull/557

// Supports:
//   Brand: Disney,  Model: Made With Magic (Glow With The Show) wand

// #include <algorithm>
#include "IRrecv.h"
#include "IRsend.h"
#include "IRutils.h"

// Constants
const uint16_t kMWMMinSamples = 6;  // Msgs are >=3 bytes, bytes have >=2
                                    // samples
const uint16_t kMWMTick = 417;
const uint32_t kMWMMinGap = 30000;  // Typical observed delay b/w commands
const uint8_t kMWMTolerance = 0;    // Percentage error margin.
const uint16_t kMWMExcess = 0;      // See kMarkExcess.
const uint16_t kMWMDelta = 150;     // Use instead of Excess and Tolerance.
const uint8_t kMWMMaxWidth = 9;     // Maximum number of successive bits at a
                                    // single level - worst case
const int16_t kSpace = 1;
const int16_t kMark = 0;

#if SEND_MWM
/// Send a MWM packet/message.
/// Status: Implemented.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
/// @note This protocol is 2400 bps serial, 1 start bit (mark),
///   1 stop bit (space), no parity
void IRsend::sendMWM(const uint8_t data[], const uint16_t nbytes,
                     const uint16_t repeat) {
  if (nbytes < 3) return;  // Shortest possible message is 3 bytes

  // Set 38kHz IR carrier frequency & a 1/4 (25%) duty cycle.
  // NOTE: duty cycle is not confirmed. Just guessing based on RC5/6 protocols.
  enableIROut(38, 25);

  for (uint16_t r = 0; r <= repeat; r++) {
    // Data
    for (uint16_t i = 0; i < nbytes; i++) {
      uint8_t byte = data[i];

      // Start bit
      mark(kMWMTick);

      // LSB first, space=1
      for (uint8_t mask = 0x1; mask; mask <<= 1) {
        if (byte & mask) {  // 1
          space(kMWMTick);
        } else {  // 0
          mark(kMWMTick);
        }
      }
      // Stop bit
      space(kMWMTick);
    }
    // Footer
    space(kMWMMinGap);
  }
}
#endif  // SEND_MWM

#if DECODE_MWM
/// Decode the supplied MWM message.
/// Status: Implemented.
/// @param[in,out] results Ptr to the data to decode & where to store the result
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return True if it can decode it, false if it can't.
/// @note This protocol is 2400 bps serial, 1 start bit (mark),
///   1 stop bit (space), no parity
bool IRrecv::decodeMWM(decode_results *results, uint16_t offset,
                       const uint16_t nbits, const bool strict) {
  DPRINTLN("DEBUG: decodeMWM");

  // Compliance
  if (results->rawlen <= kMWMMinSamples + offset) {
    DPRINTLN("DEBUG: decodeMWM: too few samples");
    return false;
  }

  uint16_t used = 0;
  uint64_t data = 0;
  uint16_t frame_bits = 0;
  uint16_t data_bits = 0;
  char tmp[24]; //DEBUG
  // No Header

  // Data
  uint8_t bits_per_frame = 10;
  for (; offset < results->rawlen && results->bits < 8 * kStateSizeMax;
       frame_bits++) {
    
    DPRINT("DEBUG: decodeMWM: offset = ");
    DPRINTLN(itoa(offset,tmp,10));
    int16_t level = getRClevel(results, &offset, &used, kMWMTick, kMWMTolerance,
                               kMWMExcess, kMWMDelta, kMWMMaxWidth);
    if (level < 0) {
      DPRINTLN("DEBUG: decodeMWM: getRClevel returned error");
      break;
    }
    switch (frame_bits % bits_per_frame) {
      case 0:
        // Start bit
        if (level != kMark) {
          DPRINTLN("DEBUG: decodeMWM: framing error - invalid start bit");
          goto done;
        }
        break;
      case 9:
        // Stop bit
        if (level != kSpace) {
          DPRINTLN("DEBUG: decodeMWM: framing error - invalid stop bit");
          return false;
        } else {
          DPRINT("DEBUG: decodeMWM: data_bits = ");
          DPRINTLN(itoa(data_bits,tmp,10));
          DPRINT("DEBUG: decodeMWM: Finished byte: ");
          DPRINTLN(uint64ToString(data).c_str());
          results->state[data_bits / 8 - 1] = data & 0xFF;
          results->bits = data_bits;
          data = 0;
        }
        break;
      default:
        // Data bits
        DPRINT("DEBUG: decodeMWM: Storing bit: ");
        DPRINTLN(itoa((level == kSpace),tmp,10));
        // Transmission is LSB-first, space=1
        data |= ((level == kSpace)) << 8;
        data >>= 1;
        data_bits++;
        break;
    }
  }

done:
  // Footer (None)

  // Compliance
  DPRINT("DEBUG: decodeMWM: frame_bits = ");
  DPRINTLN(itoa(frame_bits,tmp,10));
  DPRINT("DEBUG: decodeMWM: data_bits = ");
  DPRINTLN(itoa(data_bits,tmp,10));
  if (data_bits < nbits) {
    DPRINT("DEBUG: decodeMWM: too few bits; expected ");
    DPRINTLN(itoa(nbits,tmp,10));
    return false;  // Less data than we expected.
  }

  uint16_t payload_length = 0;
  switch (results->state[0] & 0xf0) {
    case 0x90:
    case 0xf0:
      // Normal commands
      payload_length = results->state[0] & 0x0f;
      DPRINT("DEBUG: decodeMWM: payload_length = ");
      DPRINTLN(itoa(payload_length,tmp,10));
      break;
    default:
      if (strict) {
        // Show commands
        if (results->state[0] != 0x55 && results->state[1] != 0xAA) {
          return false;
        }
      }
      break;
  }
  if (data_bits < (payload_length + 3) * 8) {
    DPRINT("DEBUG: decodeMWM: too few bytes; expected ");
    DPRINTLN(itoa(payload_length + 3,tmp,10));
    return false;
  }
  if (strict) {
    if (payload_length && (data_bits > (payload_length + 3) * 8)) {
      DPRINT("DEBUG: decodeMWM: too many bytes; expected ");
      DPRINTLN(itoa(payload_length + 3,tmp,10));
      return false;
    }
  }

  // Success
  results->decode_type = MWM;
  results->repeat = false;
  return true;
}
#endif  // DECODE_MWM

// vim: et:ts=2:sw=2
