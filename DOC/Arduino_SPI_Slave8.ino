#include <SPI.h>

// Green   - 10  UNO:CS    RC522:SDA   DOP:CS
// Yellow  - 11  UNO:MOSI  RC522:SCLK  DOP:SCLK
// Blue    - 12  UNO:MISO  RC522:MOSI  DOP:MOSI
// White   - 13  UNO:SCLK  RC552:MISO  DOP:MISO

#define   LOOP      1
#define   RESET     0 // 500

void reBOOT() {
  asm volatile (" jmp 0");
}

#define   DUMP_NONE 0
#define   DUMP_IN   1
#define   DUMP_WR   2
#define   DUMP_RD   4
#define   DUMP_CMD  8

#define   DUMPn     (DUMP_IN)
#define   DUMPn     (DUMP_WR)
#define   DUMPn     (DUMP_RD)
#define   DUMPn     (DUMP_RD | DUMP_WR)
#define   DUMPn     (DUMP_CMD)
#define   DUMPn     (DUMP_NONE)

#if (DUMPn)
#define wrRetREG(b)   { RetREG = (b); SPDR = (b); }
#else
#define wrRetREG(b)   { SPDR = (b); }
#endif

volatile uint8_t dumpp = 0, dumpg = 0, RetREG = 0, dump[256];

inline void dump_add(uint8_t b) {
  dump[dumpp] = b;
  if (++dumpp >= sizeof dump) dumpp = 0;
}

#define   DUMPx(n, data)  { if (DUMPn & (n)) dump_add(data); }

volatile uint8_t pageCnt = 0, loopCnt = 0;
volatile uint8_t temp = 0, strans = 0x00, state0 = 0x00, state1 = 0x00, skipn = 0, stat88 = 0;
volatile uint8_t stateFIFOres = 0, stateFIFOwait = 0, stateFIFOlen = 0, stateFIFOpos = 0;

const uint8_t stateFIFO[5][10] = {
  {  0,  4, 0x44, 0x64, 2, 0x05, 0x00             },  // PICC_CMD_REQA          12 26
  {  1, 12, 0x44, 0x64, 4, 0x15, 0xB2, 0xE9, 0x13 },  // PICC_CMD_MF_READ 01    12 30 01  Card seial # n1 n2 n3 n4
  {  1, 12, 0x04, 0x64, 4,    6,    0,    0,    0 },  // PICC_CMD_MF_READ 02    12 30 02  Track# nn 00 00 00
  {  1, 10, 0x04, 0x64, 2, 0x18, 0x00             },  // ??                     12 39 02
  { 11, 12, 0x04, 0x14, 0                         }   // PICC_CMD_MF_AUTH_KEY_A 12 60 00 FF FF FF FF 00 00 n1 n2 n3 n4
};

const uint8_t RegValInit[64] = {
  //00    02    04    06    08    0A    0C    0E
  0x00, 0x20, 0x80, 0x00, 0x14, 0x00, 0x00, 0x21, // 0x
  0x00, 0x8D, 0x00, 0x08, 0x10, 0x00, 0xA0, 0x00, // 1x
  0x00, 0x3B, 0x00, 0x00, 0x80, 0x00, 0x10, 0x84, // 2x
  0x84, 0x4D, 0x00, 0x00, 0x62, 0x00, 0x00, 0xEB, // 3x
  0x00, 0xFF, 0xFF, 0x88, 0x26, 0x87, 0x48, 0x88, // 4x
  0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, // 5x
  0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x40, 0x89, // 6x
  0x00, 0x16, 0x0C, 0x00, 0xFF, 0x00, 0x03, 0x00  // 7x
};

volatile uint8_t RegVal[64];

void setup() {
  Serial.begin(500000);
  Serial.println("\n--");

  //stateFIFO[2][5] = 11;     // Track #

  memcpy(RegVal, RegValInit, sizeof(RegValInit));
  SPI.setDataMode(SPI_MODE1);
  pinMode(MISO, OUTPUT);
  SPCR |= ( _BV(SPE) | _BV(SPIE) );
}

volatile uint16_t intcnt = 0;

ISR (SPI_STC_vect) {

  temp = SPDR;
#if (RESET)
  intcnt++;
#endif

#if (DUMPn & DUMP_IN)
  dump_add(temp);
#endif

  if ( (state0 == 0x00) || (temp == 0x92) ) {

    // First byte || Continuous Read FIFO(0x80 | 0x12)
    // -----------------------------------------------

    if (temp & 0x80) {                             // Read REG || Read REG continuous
      state0 = temp;
      if (strans != 0x00) {
        switch (temp) {
          case 0x88:
            if (stat88 != 0) {
              wrRetREG(stat88);
              stat88 = 0x00;
            } else {
              if (stateFIFOwait != 0) {
                stateFIFOwait--;
                wrRetREG( (stateFIFOres == 3 && stateFIFOwait == 0) ? 0x44 : stateFIFO[stateFIFOres][2] );
              } else {
                wrRetREG(stateFIFO[stateFIFOres][3]);
                RegVal[(0x88 & 0x7F) >> 1] = stateFIFO[stateFIFOres][3];
                RegVal[(0x94 & 0x7F) >> 1] = stateFIFOlen;                // Data count in FIFO
                if (strans == 0x60) strans = 0x00;
              }
            }
            break;

          case 0x8C:
            wrRetREG(0x00);
            break;

          case 0x92:

#if (LOOP == 0)
            wrRetREG(stateFIFO[stateFIFOres][stateFIFOpos + 5]);
            if (stateFIFOpos < stateFIFOlen) stateFIFOpos++; else strans = 0x00;
#endif

#if (LOOP == 1)
            if (strans == 0x39) {
              wrRetREG(pageCnt++);
              if (stateFIFOpos < stateFIFOlen) stateFIFOpos++; else strans = 0x00;
            } else {
              wrRetREG(stateFIFO[stateFIFOres][stateFIFOpos + 5]);
              if (stateFIFOpos < stateFIFOlen) stateFIFOpos++; else strans = 0x00;
            }
#endif

#if (LOOP == 2)
            if (strans == 0x39) {
              if (++loopCnt >= 32) {
                loopCnt = 0;
                if (++pageCnt >= 32) pageCnt = 0;
              }
              wrRetREG(pageCnt);
              if (stateFIFOpos < stateFIFOlen) stateFIFOpos++; else strans = 0x00;
            } else {
              wrRetREG(stateFIFO[stateFIFOres][stateFIFOpos + 5]);
              if (stateFIFOpos < stateFIFOlen) stateFIFOpos++; else strans = 0x00;
            }
#endif

#if (DUMPn & DUMP_RD)
            dump_add(state0);
            dump_add(RetREG);
#endif
            break;

          case 0x94:
            wrRetREG(stateFIFOlen);
            break;

          case 0x98:
            wrRetREG(0x10);
            break;

          case 0xD4:
            wrRetREG(0x80);
            break;

          default:
            wrRetREG(RegVal[(temp & 0x7F) >> 1]);
            break;
        }

      } else {
        wrRetREG(RegVal[(temp & 0x7F) >> 1]);
        if (temp == 0x90) RegVal[(0x90 & 0x7F) >> 1] = 0x00;          // for 90
      }

    } else {

      // Write REG
      // ---------

      state0 = temp;
      if (temp == 0x12) state1 = 0;
      if (temp == 0x08) SPDR = strans; else SPDR = 0x08;              // DEBUG
    }

  } else {

    // Second Byte
    // -----------

    if (state0 & 0x80) {      // Read REG

#if (DUMPn & DUMP_RD)
      dump_add(state0);
      dump_add(RetREG);
#endif

      SPDR = strans;          // DEBUG
      state0 = 0;

    } else {                  // Write REG

#if (DUMPn & DUMP_WR)
      if (state0 == 0x12) {
        if (state1 == 0x00) dump_add(state0);
      } else
        dump_add(state0);
      dump_add(temp);
#endif

      if (state0 == 0x12) {
        if (state1 == 0x00) {
          state1 = temp;
          switch (temp) {
            case 0x26:  stateFIFOres = 0; break;
            case 0x30:  stateFIFOres = 1; break;
            case 0x39:  stateFIFOres = 3; break;
            case 0x60:  stateFIFOres = 4; RegVal[0x10 >> 1] = 0x08; break;
            default:    stateFIFOres = 0; break;
          }
          skipn         = stateFIFO[stateFIFOres][0];
          stateFIFOwait = stateFIFO[stateFIFOres][1];
          stateFIFOlen  = stateFIFO[stateFIFOres][4];
          stateFIFOpos  = 0;
        } else {
          if (skipn) {
            if ( (skipn == 1) && (state1 == 0x30) && (temp == 0x02) )
              stateFIFOres++;                                            // PICC_CMD_MF_READ 02
            skipn--;
            SPDR = 0x08;
          }
        }

        if (skipn == 0) {
          strans = state1;
          state0 = 0x00;
          SPDR = 0x08;
        }

      } else {
        SPDR = RegVal[state0 >> 1];   // DEBUG
        if (state0 == 0x02) {                                     // Write to CommandReg
          switch (temp) {
            case 0x0E:                                            break;  // PCD_MFAuthent
            case 0x0F:                                                    // PCD_SoftReset
              strans = 0x00, state0 = 0x00, state1 = 0x00, skipn = 0, stat88 = 0;
              stateFIFOres = 0, stateFIFOwait = 0, stateFIFOlen = 0, stateFIFOpos = 0;
              // pageCnt = 0, loopCnt = 0;
              memcpy(RegVal, RegValInit, sizeof(RegValInit));
              break;
            case 0x0C:  strans = 0x00;  RegVal[0x08 >> 1] = 0x04; break;  // PCD_Transceive
            case 0x20:  strans = 0x00;                            break;  // PCD_Idle(?)
          }
        } else {
#if (0)
          if (state0 == 0x08 && temp != 0x7F) stat88 = temp = 0x04;
#endif
          if (state0 == 0x14)   temp = 0x00;
          if (state0 == 0x18)   RegVal[0x1A >> 1] &= 0x7F;
          if (state0 == 0x54) {
            RegVal[0x14 >> 1]  = 0x00;
            temp = 0x80;
          }
          RegVal[state0 >> 1] = temp;
        }
        state0 = 0x00;
      }
    }
  }
}

void loop() {

  while (1) {
#if (RESET)
    static uint32_t pmillis = millis();
    static uint16_t pintcnt = 0;

    if (pintcnt == intcnt) {
      if ( millis() > (pmillis + RESET) ) {
        Serial.print(pintcnt);  Serial.print(" - ");
        Serial.print(intcnt);   Serial.print(" - ");
        Serial.print(pmillis);  Serial.print(" - ");
        Serial.println(millis());
        delay(10);
        reBOOT();
      }
    } else {
      pintcnt = intcnt;
      pmillis = millis();
    }
#endif

#if (DUMPn)
    if (dumpg != dumpp) {
      uint8_t b = dump[dumpg];
      if (b == 0x12 || b == 0xD4 || b == 0xFF) Serial.println();
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
      if (++dumpg >= sizeof dump) dumpg = 0;
    } else
#endif

      delay(1);
  }
}
