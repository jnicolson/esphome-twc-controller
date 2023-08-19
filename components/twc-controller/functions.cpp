#include <cstddef>

#include "functions.h"

namespace esphome {
    namespace twc_controller {
      uint8_t hexCharacterStringToBytes(uint8_t *byteArray, const uint8_t *hexString, size_t length)
      {
        //bool oddLength = strlen(hexString) & 1;
        bool oddLength = length & 1;

        std::uint8_t currentByte = 0;
        std::uint8_t byteIndex = 0;

        for (uint8_t charIndex = 0; charIndex < length; charIndex++)
        {
          bool oddCharIndex = charIndex & 1;

          if (oddLength)
          {
            // If the length is odd
            if (oddCharIndex)
            {
              // odd characters go in high nibble
              currentByte = nibble(hexString[charIndex]) << 4;
            }
            else
            {
              // Even characters go into low nibble
              currentByte |= nibble(hexString[charIndex]);
              byteArray[byteIndex++] = currentByte;
              currentByte = 0;
            }
          }
          else
          {
            // If the length is even
            if (!oddCharIndex)
            {
              // Odd characters go into the high nibble
              currentByte = nibble(hexString[charIndex]) << 4;
            }
            else
            {
              // Odd characters go into low nibble
              currentByte |= nibble(hexString[charIndex]);
              byteArray[byteIndex++] = currentByte;
              currentByte = 0;
            }
          }
        }
        return byteIndex;
      }

      uint8_t nibble(uint8_t c) {
        if (c >= '0' && c <= '9')
          return c - '0';

        if (c >= 'a' && c <= 'f')
          return c - 'a' + 10;

        if (c >= 'A' && c <= 'F')
          return c - 'A' + 10;

        return 0;  // Not a valid hexadecimal character
      }

      long random(long lower, long upper) {
          if(lower >= upper) {
              return lower;
          }
          return random_uint32()%(upper - lower) + lower;
      }


    }
}