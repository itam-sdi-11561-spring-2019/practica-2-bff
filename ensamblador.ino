#include <math.h>

bool on = false;
int pin13 = 13;
int pin12 = 12;
int pin11 = 11;
int pinButton = 10;
int analogInput = A0;
unsigned int seconds = 0;
bool lights[] = { true, false, false };
int timer = 3026;
unsigned int cont = 0;

void setup()
{
  Serial.begin(9600);
  
  // Sets the ports
  DDRB = DDRB | B10000000;
  pinMode(pin13, OUTPUT);
  pinMode(pin12, OUTPUT);
  pinMode(pin11, OUTPUT);
  pinMode(pinButton, INPUT);
  pinMode(analogInput, INPUT);

  // Interrupciones
  /**
  cli();
  DDRD &= ~(1 << DDD1);
  PORTD |= (1 << PORTD1);
  EICRA |= (1 << ISC10);
  EIMSK |= (1 << INT1);
  sei();
  **/
  
  // Timer
  cli();
  TCCR1B = 0;
  TCCR1A = 0;
  TCCR1B |= (1 << CS12);
  TCNT1 = timer;
  TIMSK1 |= (1 << TOIE1);
  sei();

  
  // Timer CTC
  /*
  cli();
  TCCR1B = 0;
  TCCR1A = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  OCR1AH = 0x3D;
  OCR1AL = 0x09;
  TIMSK1 |= (1 << OCIE1A);
  sei();
  */
  // Runs functions
  Serial.println("Determinante");
  Serial.println(getDeterminant(10, 20, 30));
  Serial.println(getDeterminantAssembly(10, 20, 30));
  Serial.println("Promedio");
  Serial.println(getAverage(2, 4));
  //Serial.println(getAverageAssembly(2, 4));
  //blink();
}

ISR(INT1_vect)
{
  on = !on;
  //blinkAssembly(on);
  photodiode(on);
}

ISR(TIMER1_OVF_vect)
{
  //on = !on;
  //turnLED(on);

  // Restarts
  TCNT1 = timer;
  // Changes light
  // Green
  if (lights[0])
  {
    if (seconds < 15)
    {
      // Yellow
      if (seconds >= 15 - 3)
        lights[1] = true;
    }
    else
    {
      // Turns out green and yellow
      lights[0] = lights[1] = false;
      lights[2] = true;
      seconds = 0;
    }
  }
  // Red
  else if (lights[2])
  {
    if (seconds >= 10)
    {
      // Turns green and turns off red
      lights[0] = true;
      lights[2] = false;
      seconds = 0;
    }
  }

  trafficLights(lights);
  seconds++;
}

ISR(TIMER1_COMPA_vect)
{
  on = !on;
  turnLED(on);
}

/**
 * Gets determinant of a cuadratic equation.
 */
int getDeterminant(int a, int b, int c)
{
  return pow(b, 2) - 4 * a * c;
}

int getDeterminantAssembly(int a, int b, int c)
{
  int res = 0;
  
  asm volatile(
    "MOV r16, %1  \n\t" // a
    "MOV r17, %2  \n\t" // b
    "MOV r18, %3  \n\t" // c
    "LDI r19, 0x04  \n\t" // 4
    "MULS r17, r17  \n\t" // b²
    "MOV r20, r0  \n\t" // Saves b² LSB
    "MOV r21, r1  \n\t" // Saves b² MSB
    "LSL r16 \n\t"  // 2 * a
    "LSL r18 \n\t"  // 2 * c
    "MULS r16, r18  \n\t" // 2 * a * 2 * c
    "SUB r20, r0 \n\t" // Resta de LSB
    "SBC r21, r1 \n\t" // Resta de MSB
    "MOVW %0, r20  \n\t"
    : "=r" (res)
    : "r" (a), "r" (b), "r" (c)
  );

  Serial.println(res);

  return res;
}

/**
 * Gets the average.
 */
int getAverage(int a, int b)
{
  int sum = a + b;
  return sum / 2;
}

int getAverageAssembly(int a, int b)
{
  int res = 0;
  asm volatile(
    "MOV r18 0x00 \n\t" // i
    "MOV r19 0x02 \n\t" // size
    "FOR: \n\t"
    "ADD %1, %2 \n\t"
    "INC r18 \n\t" // i++
    "CPSE r18, r19 \n\t" // Compares
    "JMP FOR \n\t" // Returns
    // Divides
    "MOV r18, 0x00 \n\t" // i
    "FOR2: \n\t"
    "SUB %1, r19 \n\t" // Substract
    "BRMI 0x2 \n\t" // Compares
    "INC r17 \n\t" // i++
    "JMP FOR2 \n\t" // Returns
    "MOV %0, r17 \n\t"
    : "=r" (res)
    : "r" (a), "r" (b)
  );
  
  return res;
}

void blinkAssembly(bool bl)
{
  asm volatile(
    "LDI r16, 0x00 \n\t"
    "CPSE r16, %2 \n\t"
    "JMP OFF \n\t"
    "JMP ON \n\t"
    "OFF: CBI %0, %1 \n\t" // off
    "JMP EXIT \n\t"
    "ON: SBI %0, %1 \n\t" // on
    "EXIT: \n\t"
    :: "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB7), "r" (bl)
  );
}

void buttonAssembly()
{
  asm volatile(
    "SBI %0, %1 \n\t"
    "SBIS %2, %3 \n\t"
    "CBI %0, %1 \n\t"
    :: "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB7), "I" (_SFR_IO_ADDR(PINB)), "I" (PINB5)
  );
  
}

void blink()
{
  while(true)
  {
    digitalWrite(pin13, HIGH);
    delay(250);
    digitalWrite(pin13, LOW);
    delay(250);
  }
}

void button()
{
  digitalRead(pinButton); // ???
  if (digitalRead(pinButton))
  {
    cont++;
    Serial.print("Counter: ");
    Serial.print(cont);
    Serial.print('\n');
  }
}

void photodiode()
{
  int a = analogRead(analogInput);
  if (a < 1000)
  {
    cont++;
    Serial.println(cont);
  }
}

void photodiode(bool on)
{
  if (on) photodiode();
}

void turnLED(bool on)
{
  if (on) digitalWrite(pin13, HIGH);
  else digitalWrite(pin13, LOW);
}

void trafficLights(bool lights[])
{
  digitalWrite(pin11, lights[0] ? HIGH : LOW);
  digitalWrite(pin12, lights[1] ? HIGH : LOW);
  digitalWrite(pin13, lights[2] ? HIGH : LOW);
}

void loop()
{
  // put your main code here, to run repeatedly:
  //buttonAssembly();
  //button();
  //photodiode();
}
