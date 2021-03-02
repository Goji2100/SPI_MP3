#include <SPI.h>

#define   RESET false
#define   DUMP  false

void software_reset() {
  asm volatile ("  jmp 0");
}

// Green   - 10  CS   SDA
// Yellow  - 11  MOSI SCK
// Blue    - 12  MISO MOSI
// White   - 13  SCLK MISO

volatile uint8_t dump[1024], dumpp = 0, dumpg = 0;

volatile uint8_t strans = 0x00, state0 = 0x00, state1 = 0x00, skipn = 0, stat80 = 0;
volatile uint8_t stateFIFOres = 0, stateFIFOwait = 0, stateFIFOlen = 0, stateFIFOpos = 0;

// 12 26
// 12 30 01
// 12 30 02
// 12 39 02
// 12 60 00 FF FF FF FF 00 00 x1 x2 x3 x4

uint8_t stateFIFO[5][10] = {
  {  4, 0x44, 0x64, 2, 0x05, 0x00             },  // PICC_CMD_REQA          12 26
  {  4, 0x44, 0x64, 4, 0x15, 0xB2, 0xE9, 0x13 },  // PICC_CMD_MF_READ 01    12 30 01  UUID ?
  {  4, 0x04, 0x64, 4, 0x02, 0x00, 0x00, 0x00 },  // PICC_CMD_MF_READ 02    12 30 02  Track# nn 00 00 00
  {  4, 0x04, 0x66, 1, 0x05                   },  // ??                     12 39 02
  { 10, 0x04, 0x14, 0                         }   // PICC_CMD_MF_AUTH_KEY_A 12 60 ..
};

volatile uint8_t RegVal[64] = {
  //00    02    04    06    08    0A    0C    0E
  0x00, 0x20, 0x80, 0x00, 0x04, 0x00, 0x00, 0x21, // 0x
  0x00, 0x26, 0x00, 0x08, 0x10, 0x00, 0xA0, 0x00, // 1x
  0x00, 0x3F, 0x00, 0x80, 0x80, 0x00, 0x00, 0x84, // 2x
  0x84, 0x4D, 0x00, 0x00, 0x62, 0x00, 0x00, 0xEB, // 3x
  0x00, 0xFF, 0xFF, 0x88, 0x26, 0x87, 0x48, 0x88, // 4x
  0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 5x
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x40, 0x92, // 6x
  0x00, 0x03, 0x11, 0x88, 0xFF, 0x00, 0x03, 0x00  // 7x
};


void setup() {
  Serial.begin(115200 * 2);
  Serial.println("\n--");

  SPI.setDataMode(SPI_MODE1);

  pinMode(MISO, OUTPUT);
  SPCR |= (_BV(SPE) | _BV(SPIE));

  stateFIFO[2][4] = 1;     // Track #
}

volatile uint16_t intcnt = 0;

ISR (SPI_STC_vect) {
  static uint8_t temp;

  temp = SPDR;
  intcnt++;

#if (DUMP)
  dump[dumpp++] = temp;           // DEBUG
  if (dumpp >= sizeof dump) dumpp = 0;
#endif

  // First byte || Continuous Read FIFO(0x12 | 0x80)
  if ( (state0 == 0x00) || ((state0 == 0x92) && (temp == 0x92)) ) {
    if (temp & 0x80) {        // Read REG
      state0 = temp;
      if (strans == 0x00) {   // Not in FIFO
        SPDR = RegVal[(temp & 0x7F) >> 1];

      } else {
        switch (temp & 0x7F) {
          case 0x08:
            if (stat80 != 0) {
              SPDR = stat80;
              stat80 = 0;
            } else {
              if (stateFIFOwait) {
                stateFIFOwait--;
                SPDR = stateFIFO[stateFIFOres][1];
                if (stateFIFOres == 3 && stateFIFOwait == 0)
                  SPDR = 0x44;
                else
                  SPDR = stateFIFO[stateFIFOres][1];
              } else {
                SPDR = stateFIFO[stateFIFOres][2];
                RegVal[(0x94 & 0x7F) >> 1] = stateFIFOlen;                // Data count in FIFO
                if (strans == 0x60) strans = 0x00;
              }
            }
            break;

          case 0x0C:
            SPDR = 0x00;
            break;

          case 0x12:
            SPDR = stateFIFO[stateFIFOres][stateFIFOpos + 4];
            if (stateFIFOpos < stateFIFOlen) stateFIFOpos++; else strans = 0x00;
            break;

          case 0x14:
            SPDR = (stateFIFOlen - stateFIFOpos);
            break;

          case 0x18:
            SPDR = 0x10;
            break;

          case 0x54:
            SPDR = 0x80;
            break;

          default:
            SPDR = RegVal[(temp & 0x7F) >> 1];
            if (temp == 0x90) RegVal[(0x90 & 0x7F) >> 1] = 0x00;          // for 90
            break;
        }
      }
    } else {                  // Write REG
      state0 = temp;
      if (temp == 0x12) {
        state1 = 0;
        strans = 0;
      }
      SPDR = temp;            // DEBUG
    }

    // Second Byte
  } else {
    if (state0 & 0x80) {      // Read REG
      SPDR = strans;          // DEBUG
      state0 = 0;

    } else {                  // Write REG
      if (state0 == 0x12) {
        if (state1 == 0x00) {
          state1 = temp;
          switch (temp) {
            case 0x26:  skipn =  0; stateFIFOres = 0; break;
            case 0x30:  skipn =  1; stateFIFOres = 1; break;
            case 0x39:  skipn =  1; stateFIFOres = 3; break;
            case 0x60:  skipn = 11; stateFIFOres = 4; RegVal[0x10 >> 1] = 0x08; break;
            default:    skipn =  0; stateFIFOres = 0; break;
          }
          stateFIFOwait = stateFIFO[stateFIFOres][0];
          stateFIFOlen  = stateFIFO[stateFIFOres][3];
          stateFIFOpos  = 0;
        } else {
          if (skipn) {
            if (skipn == 1 && state1 == 0x30 && temp == 0x02)
              stateFIFOres++;                                            // PICC_CMD_MF_READ 02
            skipn--;
            SPDR = 0x08;
          }
        }
        if (skipn == 0) {
          strans = state1;
          state0 = 0x00;
          state1 = 0x00;
          SPDR = strans;  // 0x08; DEBUG = strans
        }
      } else {
        if (state0 == 0x02) {         // Write to CommandReg
          switch (temp) {
            case 0x0E:                                            break;  // PCD_MFAuthent
            case 0x0F:  strans = 0x00;                            break;  // PCD_SoftReset
            case 0x0C:  strans = 0x00;  RegVal[0x08 >> 1] = 0x04; break;  // PCD_Transceive
            case 0x20:  strans = 0x00;                            break;  // PCD_Idle(?)
          }
        } else {
          SPDR = RegVal[state0 >> 1];                             // DEBUG
          if ( (state0 == 0x08) && (temp == 0x04) ) stat80 = 0x04;
          RegVal[state0 >> 1] = temp;
        }
        state0 = 0x00;
      }
    }
  }
}

void loop() {
  static uint32_t pmillis = millis();
  static uint16_t pintcnt = 0;

#if (RESET)
  if (pintcnt == intcnt) {
    if ( millis() > (pmillis + 120) ) {
      Serial.print(pintcnt);  Serial.print(" - ");
      Serial.print(intcnt);   Serial.print(" - ");
      Serial.print(pmillis);  Serial.print(" - ");
      Serial.println(millis());
      delay(10);
      software_reset();
    }
  } else {
    pintcnt = intcnt;
    pmillis = millis();
  }
#endif

#if (DUMP)
  if (dumpg != dumpp) {
    if (dump[dumpg] == 0xA4) Serial.println();
    if (dump[dumpg] < 0x10) Serial.print("0");
    Serial.print(dump[dumpg], HEX);
    Serial.print(" ");
    if ((++dumpg) >= sizeof dump) dumpg = 0;
  }
#endif

  delay(1);
}
