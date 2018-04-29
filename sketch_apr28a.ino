static const int PIN_STEP = 5;
static const int PIN_DIR = 4;
static const int PIN_INDEX = 3;
static const int PIN_MOTOR_ENABLE = 2;
static const int PIN_READ_DATA = 8;
static const int PIN_TRACK00 = 9;
static const int PIN_WRITE_DATA = 6;
static const int PIN_WRITE_ENABLE = 7;

static const int BUFSIZE = 30*256;
char buf[BUFSIZE];

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

  Serial.print("Moving the head");
  digitalWrite(PIN_DIR, HIGH);
  while(digitalRead(PIN_TRACK00)) {
    Serial.write('.');
    digitalWrite(PIN_STEP, LOW);
    delay(10);
    digitalWrite(PIN_STEP, HIGH);
    delay(5);
  }
  Serial.println();

  digitalWrite(PIN_DIR, LOW);
  for(int i = 0; i < 20; i++) {
    digitalWrite(PIN_STEP, LOW);
    delay(10);
    digitalWrite(PIN_STEP, HIGH);
    delay(5);
  }

  delay(25);

  digitalWrite(PIN_MOTOR_ENABLE, LOW);
  delay(1000);

#if 1
  digitalWrite(PIN_WRITE_ENABLE, LOW);
  digitalWrite(PIN_WRITE_DATA, HIGH);

  Serial.print("Writing: ");
  uint8_t data[] = "Hello world!";
  Serial.write(data, sizeof(data));
  Serial.println();
  for(uint8_t* b = buf; b < buf+BUFSIZE - 1024;) {
    for(int i = 0; i < sizeof(data); i++) {
      byte_to_bits(&b, 0b10101001); byte_to_bits(&b, data[i]);
      //byte_to_bits(&b, 0b10101010);
    }
  }
  asm volatile ("nop\n\t"
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
    "write_loop:\n\t"
    "sts %[_PORTH], r25\n\t" // 2 cycles
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "nop; nop; nop; nop; nop; nop; nop; nop\n\t"
    "sbis %[_PINE], 5\n\t" // 2 if skipped and 1-word instr
    "rjmp write_jumpout\n\t" // not executed
    "nop\n\t" // 1 cycle, the other branch executes jmp normally, this one does not
    "nop\n\t" // 1 cycle, align to 8
    "sts %[_PORTH], r24\n\t" // 2 cycles
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
    "nop; nop; nop\n\t"

    "ld __tmp_reg__, %a[buf]+\n\t" // 2 cycles
    "tst __tmp_reg__\n\t" // 1 cycle
    "breq skip_delay\n\t" // 2 if jumped, 1 if not jumped
    "nop\n\t" // align after no jump
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
    
    "nop\n\t" // 1 cycle, align to 8
    "sbic %[_PINE], 5\n\t" // 1 cycle if not skipped
    "rjmp write_loop\n\t" // 2 cycles
    
    "write_jumpout:\n\t"
    "sbis %[_PINE], 5\n\t"
    "rjmp write_jumpout\n\t"
    :
    : [_PINE]    "I"  (_SFR_IO_ADDR(PINE)),
      [_PORTH]    "i"  (_SFR_MEM_ADDR(PORTH)),
      [buf] "e" (buf)
    : "r24", "r25"
  );
  digitalWrite(PIN_WRITE_ENABLE, HIGH);
#endif

  Serial.println("Reading 8 times:");
  for(int k = 0; k < 8; k++) {
    for(int i = 0; i < BUFSIZE; i++) buf[i] = 0;
    uint8_t aa = 0;
    uint8_t bb = 30;
    asm volatile ("nop\n\t"
      // Wait for one loop to finish
      "read_loop1:\n\t"
      "sbic %[_PINE], 5\n\t"
      "rjmp read_loop1\n\t"
      "finish_loop1:\n\t"
      "sbis %[_PINE], 5\n\t"
      "rjmp finish_loop1\n\t"
      ""
      "read_loop:\n\t"
      "clr r24\n\t" // 1 cycle
      "wait_for_rising_edge:\n\t"
      "lds __tmp_reg__, %[_PINH]\n\t"
      "sbrs __tmp_reg__, 5\n\t"
      "rjmp wait_for_rising_edge\n\t"
      
      "count_loop:\n\t"
      "inc r24\n\t" // 1 cycle
      "lds __tmp_reg__, %[_PINH]\n\t" // 2 cycles
      "sbrc __tmp_reg__, 5\n\t" // 1 if not skipped, 2 if skipped
      "rjmp count_loop\n\t" // 2 cycles

      "lsr r24\n\t" // 1 cycle
      "lsr r24\n\t" // 1 cycle
      "lsr r24\n\t" // 1 cycle
      "andi r24, 1\n\t" // 1 cycle
      "st %a[buf]+, r24\n\t" // 2 cycles
      
      "sez\n\t" // 1 cycle
      "subi %[aa], 1\n\t" // 1 cycle
      "sbc %[bb], __zero_reg__\n\t" // 1 cycle
      "breq finish_loop\n\t" // 1 cycle if not taken
      
      "sbic %[_PINE], 5\n\t" // 1 cycle if not skipped
      "rjmp read_loop\n\t" // 2 cycles
      
      "finish_loop:\n\t"
      "sbis %[_PINE], 5\n\t"
      "rjmp finish_loop\n\t"
      : [aa] "+&e" (aa),
        [bb] "+&e" (bb)
      : [_PINE]    "I"  (_SFR_IO_ADDR(PINE)),
        [_PINH]    "i"  (_SFR_MEM_ADDR(PINH)),
        [buf] "e" (buf)
      : "memory", "r24");
    uint16_t t = 0;
    for(uint8_t* b = buf; b < buf + BUFSIZE; ) {
      //Serial.print(*b++, DEC); Serial.print(',');
      if(b[0] == 1 &&
         b[1] == 0 &&
         b[2] == 1 &&
         b[3] == 0 &&
         b[4] == 1 &&
         b[5] == 0 &&
         b[6] == 0 &&
         b[7] == 1) {
          b += 8;
          Serial.write(bits_to_byte(&b));
          t++;
         } else b++;
    }
    Serial.println();
  }

  digitalWrite(PIN_MOTOR_ENABLE, HIGH);
}

void loop() {
}
