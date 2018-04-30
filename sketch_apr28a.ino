// NOTE: If you change the pin assignments, you'll also have to change the assembler code below
static const int PIN_STEP = 5;
static const int PIN_DIR = 4;
static const int PIN_INDEX = 3;
static const int PIN_MOTOR_ENABLE = 2;
static const int PIN_READ_DATA = 8;
static const int PIN_TRACK00 = 9;
static const int PIN_WRITE_DATA = 6;
static const int PIN_WRITE_ENABLE = 7;

static const int BUFSIZE = 30*256;
uint8_t buf[BUFSIZE];

void byte_to_bits(char** buf, uint8_t b)
{
  *((*buf)++) = (b >> 7) & 1;
  *((*buf)++) = (b >> 6) & 1;
  *((*buf)++) = (b >> 5) & 1;
  *((*buf)++) = (b >> 4) & 1;
  *((*buf)++) = (b >> 3) & 1;
  *((*buf)++) = (b >> 2) & 1;
  *((*buf)++) = (b >> 1) & 1;
  *((*buf)++) = (b >> 0) & 1;
}

uint8_t bits_to_byte(char** buf)
{
  return (*((*buf)++) << 7) |
         (*((*buf)++) << 6) |
         (*((*buf)++) << 5) |
         (*((*buf)++) << 4) |
         (*((*buf)++) << 3) |
         (*((*buf)++) << 2) |
         (*((*buf)++) << 1) |
         (*((*buf)++) << 0);
}

void resetHead()
{
  Serial.print("Reset the head");
  digitalWrite(PIN_DIR, HIGH);
  while(digitalRead(PIN_TRACK00)) {
    Serial.write('.');
    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_STEP, HIGH);
    delay(5);
  }
  Serial.println();
}

void moveHeadTo(uint8_t track) // TODO: Could store current position
{
  resetHead();
  digitalWrite(PIN_DIR, LOW);
  for(int i = 0; i < track; i++) {
    digitalWrite(PIN_STEP, LOW);
    delay(10);
    digitalWrite(PIN_STEP, HIGH);
    delay(5);
  }
}

struct __attribute__((packed)) RecordedEntry {
  uint16_t note_delay;
  uint16_t note_period;
};

// TODO: These seem like delayMicroseconds is slower than it should
// This uses Moppy format for input
void record(struct RecordedEntry rec[], const uint16_t max_count, uint16_t* count)
{
  bool recording = true;
  uint16_t time_since_last = 0;
  uint16_t rec_idx = 0;
  digitalWrite(LED_BUILTIN, LOW);
  while(recording) {
    if (Serial.available() > 2) {
      if (Serial.peek() == 100) { // commmand?
        Serial.read(); // 100
        byte cmd = Serial.read();
        byte unused = Serial.read();
        switch(cmd) {
          case 0: break; // reset
          case 1: break; // connected
          case 2: break; // disconnected
          case 3: digitalWrite(LED_BUILTIN, HIGH); time_since_last = 0; break; // sequence start
          case 4: digitalWrite(LED_BUILTIN, LOW); recording = false; break; // sequence stop
        }
      } else {
        byte drive = Serial.read();
        uint16_t period = (Serial.read() << 8) | Serial.read();
        if (rec_idx < max_count) {
          rec[rec_idx].note_period = period;
          rec[rec_idx].note_delay = time_since_last;
          rec_idx++;
        } else {
          digitalWrite(LED_BUILTIN, LOW);
        }
        time_since_last = 0;
      }
    }
    delayMicroseconds(40);
    time_since_last++;
  }
  *count = rec_idx;
}

void playback(struct RecordedEntry rec[], uint16_t count)
{
  digitalWrite(LED_BUILTIN, HIGH);
  uint16_t period = 0;
  bool dir = false;
  bool st = false;
  uint8_t pos = 0;
  digitalWrite(PIN_DIR, LOW);
  for(uint16_t i = 0; i < count; i++) {
    for(uint16_t j = 0; j < rec[i].note_delay; j++) {
      if ((j%period) == 0) {
        if (pos >= 158) { digitalWrite(PIN_DIR, dir = !dir); pos = 0; }
        pos ++;
        digitalWrite(PIN_STEP, st = !st);
      }
      delayMicroseconds(40);
    }
    period = rec[i].note_period;
  }
  digitalWrite(LED_BUILTIN, LOW);
  resetHead();
}

void setup() {
  Serial.begin(230400);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_INDEX, INPUT_PULLUP);
  pinMode(PIN_TRACK00, INPUT_PULLUP);
  pinMode(PIN_READ_DATA, INPUT_PULLUP);
  pinMode(PIN_WRITE_DATA, OUTPUT);
  pinMode(PIN_WRITE_ENABLE, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_MOTOR_ENABLE, HIGH);
  digitalWrite(PIN_WRITE_ENABLE, HIGH);

  resetHead();

#if 0
  RecordedEntry rec[1000];
  uint16_t rec_count;
  record(rec, 1000, &rec_count);

  while(true) {
    byte c = Serial.read();
    if (c == 'a') {
      Serial.print("Entries: ");
      Serial.println(rec_count);
    }
    if (c == 'b') {
      Serial.println("Playing");
      playback(rec, rec_count);
    }
    if (c == 'c') {
      for(uint16_t i = 0; i < rec_count; i++) {
        Serial.print(i);
        Serial.print(' ');
        Serial.print(rec[i].note_delay);
        Serial.print(' ');
        Serial.println(rec[i].note_period);
      }
    }
  }

  return;
#endif

  moveHeadTo(20);
  delay(25);

  digitalWrite(PIN_MOTOR_ENABLE, LOW);
  delay(1000);

#if 0
  Serial.println("Recording");
  uint8_t* b = buf;
  for(int i = 0; i < 100; i++) *b++ = 0b10101001;
  //for(int i = 0; i < 1000; i++) { *b++ = 0; *b++ = -1; }
  uint16_t count;
  record((struct RecordedEntry*)(&buf[100]), 800, &count);

  moveHeadTo(20);
  delay(25);
  
  digitalWrite(PIN_WRITE_ENABLE, LOW);
  digitalWrite(PIN_WRITE_DATA, HIGH);

  asm volatile ("nop\n\t"
    "cli\n\t" // Disable interrupts because this loop is quite timing critical...
    // Wait for one loop to finish
    "read_loop0:\n\t"
    "sbic %[_PINE], 5\n\t"
    "rjmp read_loop0\n\t"
    "finish_loop0:\n\t"
    "sbis %[_PINE], 5\n\t"
    "rjmp finish_loop0\n\t"
    ""
    "lds r24, %[_PORTH]\n\t" // r24 = high
    "ldi r25, (1<<3)\n\t" // r25 = low
    "eor r25, r24\n\t"
    ""
    "clr r26\n\t" // r26 is the bit index
    "write_loop:\n\t"
    // Load in the next byte, if needed
    "cpse r26, 0\n\t"
    "rjmp no_load_needed\n\t"
    "ld __tmp_reg__, %a[buf]+\n\t"
    "no_load_needed:\n\t"
    ""
    "sts %[_PORTH], r25\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "sts %[_PORTH], r24\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"

    "sbrs __tmp_reg__, 7\n\t"
    "rjmp skip_delay\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "skip_delay:\n\t"

    // Increment the bit counter
    "inc r26\n\t"
    "andi r26, 7\n\t"
    // Shift the byte
    "lsl __tmp_reg__\n\t"
    
    "sbic %[_PINE], 5\n\t"
    "rjmp write_loop\n\t"
    
    "write_jumpout:\n\t"
    "sbis %[_PINE], 5\n\t"
    "rjmp write_jumpout\n\t"
    "sei\n\t" // Reenable interrupts
    :
    : [_PINE]    "I"  (_SFR_IO_ADDR(PINE)),
      [_PORTH]    "i"  (_SFR_MEM_ADDR(PORTH)),
      [buf] "e" (buf)
    : "r24", "r25", "r26"
  );
  digitalWrite(PIN_WRITE_ENABLE, HIGH);

  moveHeadTo(20);
  delay(25);
#endif

  Serial.println("Reading 8 times:");
  for(int k = 0; k < 8; k++) {
    for(int i = 0; i < BUFSIZE; i++) buf[i] = 0;
    asm volatile ("nop\n\t"
      "cli\n\t" // Disable interrupts because this loop is quite timing critical...
      "ldi r25, 0\n\t"
      "ldi r26, 30\n\t"
      "clr r23\n\t" // bit counter
      "clr r27\n\t" // temp byte
      // Wait for one loop to finish
      "read_loop1:\n\t"
      "sbic %[_PINE], 5\n\t"
      "rjmp read_loop1\n\t"
      "finish_loop1:\n\t"
      "sbis %[_PINE], 5\n\t"
      "rjmp finish_loop1\n\t"
      "\n\t"
      ""
      "read_loop:\n\t"
      "clr r24\n\t"
      "wait_for_rising_edge:\n\t"
      "lds __tmp_reg__, %[_PINH]\n\t"
      "sbrs __tmp_reg__, 5\n\t"
      "rjmp wait_for_rising_edge\n\t"
      
      "count_loop:\n\t"
      "inc r24\n\t"
      "lds __tmp_reg__, %[_PINH]\n\t"
      "sbrc __tmp_reg__, 5\n\t"
      "rjmp count_loop\n\t"

      "lsr r24\n\t"
      "lsr r24\n\t"
      //"lsr r24\n\t"
      //"andi r24, 1\n\t"
      "tst r24\n\t"
      "cpse r24, __zero_reg__\n\t"
      "ldi r24, 1\n\t"
      "lsl r27\n\t"
      "or r27, r24\n\t"
      "inc r23\n\t"
      //"cpse r23, 8\n\t"
      //"rjmp no_store_needed\n\t"
      "cpi r23, 8\n\t"
      "brne no_store_needed\n\t"
      
      "st %a[buf]+, r27\n\t"
      "clr r23\n\t"
      "clr r27\n\t"
      // Exit on buffer overflow
      "sez\n\t"
      "subi r25, 1\n\t"
      "sbc r26, __zero_reg__\n\t"
      "breq finish_loop\n\t"
      
      "no_store_needed:\n\t"

      // Repeat if no index hole still
      "sbic %[_PINE], 5\n\t"
      "rjmp read_loop\n\t"

      // Wait for the index hole end
      "finish_loop:\n\t"
      "sbis %[_PINE], 5\n\t"
      "rjmp finish_loop\n\t"
      "sei\n\t" // Reenable interrupts
      :
      : [_PINE]    "I"  (_SFR_IO_ADDR(PINE)),
        [_PINH]    "i"  (_SFR_MEM_ADDR(PINH)),
        [buf] "e" (buf)
      : "memory", "r24", "r25", "r26", "r27", "r23");
    // We now have to realign the bits in the buffer
    // TODO: Do this somehow during loading?
    for(int i = 0; i < BUFSIZE; i++) {
      int off = -1;
           if((buf[i]&0b11111111) == 0b10101001 && (buf[i+1]&0b00000000) == 0b00000000) off = 0;
      else if((buf[i]&0b01111111) == 0b01010100 && (buf[i+1]&0b10000000) == 0b10000000) off = 1;
      else if((buf[i]&0b00111111) == 0b00101010 && (buf[i+1]&0b11000000) == 0b01000000) off = 2;
      else if((buf[i]&0b00011111) == 0b00010101 && (buf[i+1]&0b11100000) == 0b00100000) off = 3;
      else if((buf[i]&0b00001111) == 0b00001010 && (buf[i+1]&0b11110000) == 0b10010000) off = 4;
      else if((buf[i]&0b00000111) == 0b00000101 && (buf[i+1]&0b11111000) == 0b01001000) off = 5;
      else if((buf[i]&0b00000011) == 0b00000010 && (buf[i+1]&0b11111100) == 0b10100100) off = 6;
      else if((buf[i]&0b00000001) == 0b00000001 && (buf[i+1]&0b11111110) == 0b01010010) off = 7;
      if (off == -1) continue;
      Serial.print("Offset is ");
      Serial.print(off);
      Serial.print(" starting at ");
      Serial.print(i);
      while ((uint8_t)((buf[i] << off) | (buf[i+1] >> (8-off))) == 0b10101001) i++;
      Serial.print(", but really at ");
      Serial.println(i);
      for(int j = 0; j < BUFSIZE; j++) {
        buf[j] = (buf[j+i] << off) | (buf[j+i+1] >> (8-off));
      }
      break;
    }
    for(uint8_t* b = buf; b < buf + BUFSIZE; ) {
      Serial.print(*b++, DEC); Serial.print(',');
      /*if(b[0] == 1 &&
         b[1] == 0 &&
         b[2] == 1 &&
         b[3] == 0 &&
         b[4] == 1 &&
         b[5] == 0 &&
         b[6] == 0 &&
         b[7] == 1 &&
         b[8] == 1 &&
         b[9] == 0 &&
         b[10] == 1 &&
         b[11] == 0 &&
         b[12] == 1 &&
         b[13] == 1 &&
         b[14] == 1 &&
         b[15] == 0) {
          b += 16;
          while(true) {
            uint8_t c = bits_to_byte(&b);
            if (c == 0)
              break;
            Serial.write(c);
          }
          Serial.println();
         } else b++;*/
    }
    Serial.println();
    playback((struct RecordedEntry*)(&buf[0]), 800);
  }

  digitalWrite(PIN_MOTOR_ENABLE, HIGH);
}

void loop() {
}
