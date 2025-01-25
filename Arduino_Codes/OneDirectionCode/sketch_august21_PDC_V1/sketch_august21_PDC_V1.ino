
/*
  Sketch: Node Code 
  Project: Real-Time Continuous Healthcare Monitoring System
  Engineer: Shahzad Muzaffar
  Institute: Masdar Institute of Science & Technology
*/

//#include <BMI160.h>
#include <CurieIMU.h>
#include <CurieTimerOne.h>
#include <MadgwickAHRS.h>
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions
#define SERIAL_ENABLE 1
//#define BOARD1 1
#define BOARD2 1

#define SEG_MASK        (0x000000FF)
#define SUB_SEG_MASK    (0x0F)
#define DATA_SIZE       (16)
#define SEG_SIZE        (8)
#define SUB_SEG_SIZE    (4)
#define STOP_PULSES     (2)
#define SEG_COUNT       ((DATA_SIZE / SUB_SEG_SIZE) + STOP_PULSES)
#define ON_BITS_LIMIT   (SEG_SIZE >> 1)
#define DELAY_PULSES    (4)
#define INI_FACTOR      (3)
#define INI_PULSES      (DELAY_PULSES * INI_FACTOR)

//#define PULSE_TIME_MS   (1) // Comment for Time usage in us
#define PULSE_MS        (1)
#define PULSE_US        (10)//(500) // largest value that will produce an accurate delay is 16383
#define STREAM_DELAY_US (500)

#define RX_MARGIN       (1) // in terms of Number of pulses
#define RX_US           (1000)
#define RX_DELAY_PULSES (DELAY_PULSES - RX_MARGIN)
#define RX_INI_PULSES   (INI_PULSES - INI_FACTOR)
#ifdef PULSE_TIME_MS
#define RX_DELAY        (RX_DELAY_PULSES * PULSE_MS * RX_US)
#define RX_INI_LENGTH   (RX_INI_PULSES * PULSE_MS * RX_US)
#else
#define RX_DELAY        (RX_DELAY_PULSES * PULSE_US)
#define RX_INI_LENGTH   (RX_INI_PULSES * PULSE_US)
#endif

//---------------------------------------------------------------------------------------------------
// Variable definitions

/* Time Defines */
volatile int timeInUsec = RX_DELAY; //100000;   // Mirco second unit.
bool toggle = 0;                  // The LED status toggle

/* IMU Defines */
Madgwick filter; // initialise Madgwick object
int ax, ay, az;
int gx, gy, gz;
int mx = 0.0f, my = 0.0f, mz = 0.0f;
float yaw;                       // YOu can say this z-axis - Normal to the chip
float pitch;                     // In the direction of the Width of Curie chip
float roll;                      // In the direction of the length of Curie chip
int factor = 80;                   // variable by which to divide gyroscope values, used to control sensitivity
                                   // note that an increased baud rate requires an increase in value of factor
int calibrateOffsets = 1;          // int to determine whether calibration takes place or not
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
const long interval = 1000;           // interval at which to blink (milliseconds)

//---------------------------------------------------------------------------------------------------
// Pin Defines

const byte ledPin      = 13;
const int buttonPin    = 7;
const byte SingleWireI = 2;
const byte SingleWireO = 4;
//---------------------------------------------------------------------------------------------------

volatile byte state       = LOW;
volatile byte count       = 0;
volatile byte Pstream     = 0;
unsigned int  Data        = 3;
unsigned int  SlvBoardIni = 0;
unsigned int  RxData      = 0;

// ====================================
// PDC Defines
volatile byte seg0 = 0;
volatile byte seg1 = 0;
volatile byte subSeg0 = 0;
volatile byte subSeg1 = 0;
volatile byte subSeg2 = 0;
volatile byte subSeg3 = 0;
volatile byte Flags0 = 0;
volatile byte Flags1 = 0;
volatile byte CFlags = 0;
// ====================================

/* Start of Main Code */
void setup() {

#ifdef SERIAL_ENABLE
  // initialize Serial communication
  Serial.begin(115200);
#endif
  
  /* Timer Initializations */
  //CurieTimerOne.start(timeInUsec, &timedIsr);  // set timer and callback
  //CurieTimerOne.kill();
  CurieTimerOne.attachInterrupt(&timedIsr); // set timer and callback
  
  // Initialize pin 2 as Input Pullup by default.
  pinMode(SingleWireI, INPUT); // ***Dont forget to attach a pull down resistor of 10K
  pinMode(SingleWireO, OUTPUT); // ***Dont forget to attach a pull down resistor of 10K
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(SingleWireO, LOW);

  attachInterrupt(digitalPinToInterrupt(SingleWireI), PDC_I_ISR, FALLING);

  digitalWrite(ledPin, LOW);
}

void loop()
{
#ifdef BOARD1
  
  /* Slave Boards initialization phase */
  // Send start pulse
  int buttonState = HIGH;

  if(SlvBoardIni == 0)
  {
    while (buttonState == HIGH)
    {
      buttonState = digitalRead(buttonPin);
    }
  
    buttonState = LOW;
    
    while (buttonState == LOW)
    {
      buttonState = digitalRead(buttonPin);
    }
#ifdef SERIAL_ENABLE
    Serial.println("Ini Sent!");
#endif
    PDC_SlvBoardsInit(INI_PULSES);
    digitalWrite(ledPin, HIGH);
    SlvBoardIni = 1;
  }
  else
  {
  
    // read the state of the pushbutton value
    buttonState = HIGH;
  
    while (buttonState == HIGH)
    {
      buttonState = digitalRead(buttonPin);
    }

    buttonState = LOW;
    
    while (buttonState == LOW)
    {
      buttonState = digitalRead(buttonPin);
    }
  
    PDC_Transmit(Data);
    //while (Pstream == 0){}

#ifdef SERIAL_ENABLE
    /*Serial.print(CFlags); Serial.print("-");
    Serial.print(Flags0); Serial.print("-");
    Serial.print(Flags1); Serial.print("-");
    Serial.print(subSeg0); Serial.print("-");
    Serial.print(subSeg1); Serial.print("-");
    Serial.print(subSeg2); Serial.print("-");
    Serial.print(subSeg3); Serial.print("-");*/
    Serial.println(Data);
#endif
    //Pstream = 0;
    Data++;
    //count = 0;
  }
#endif

#ifdef BOARD2
  if(SlvBoardIni == 0)
  {
    PDC_IniReceive();
    digitalWrite(ledPin, HIGH);
    SlvBoardIni = 1;
#ifdef SERIAL_ENABLE
    Serial.println("Ini Received!");
#endif
  }
  
  RxData = PDC_Receive();
#ifdef SERIAL_ENABLE
  /*Serial.print(CFlags); Serial.print("-");
  Serial.print(Flags0); Serial.print("-");
  Serial.print(Flags1); Serial.print("-");
  Serial.print(subSeg0); Serial.print("-");
  Serial.print(subSeg1); Serial.print("-");
  Serial.print(subSeg2); Serial.print("-");
  Serial.print(subSeg3); Serial.print("-");*/
  Serial.println(RxData);
#endif

#endif
}

/* ############################################################# */
/*                         PDC Code                              */
/* ############################################################# */

/* ============================================================= */
/* PDC Ini Transmitter */
/* ============================================================= */
void PDC_SlvBoardsInit(unsigned int IniPulseCount)
{
  /* Before Start : Select required setup */
  detachInterrupt(digitalPinToInterrupt(SingleWireI));
  digitalWrite(SingleWireO, LOW);
  
  /* Transmit */
  /* Send Pulse */
#ifdef PULSE_TIME_MS
    delay(PULSE_MS); // ms
#else
    delayMicroseconds(PULSE_US); // us
#endif
    digitalWrite(SingleWireO, HIGH);
#ifdef PULSE_TIME_MS
    delay(PULSE_MS*IniPulseCount); // ms
#else
    delayMicroseconds(PULSE_US*IniPulseCount); // us
#endif
    digitalWrite(SingleWireO, LOW);

  /* Send Delay */
  for(int i=0; i<DELAY_PULSES; i++)
  {
#ifdef PULSE_TIME_MS
    delay(PULSE_MS); // ms
#else
    delayMicroseconds(PULSE_US); // us
#endif
    digitalWrite(SingleWireO, LOW);
#ifdef PULSE_TIME_MS
    delay(PULSE_MS); // ms
#else
    delayMicroseconds(PULSE_US); // us
#endif
    digitalWrite(SingleWireO, LOW);
  }

   delayMicroseconds(STREAM_DELAY_US); // us
  /* Before Leaving : Place back in original state */
  attachInterrupt(digitalPinToInterrupt(SingleWireI), PDC_I_ISR, FALLING);
}

/* ============================================================= */
/* PDC Transmitter */
/* ============================================================= */
void PDC_Transmit(unsigned int TxData)
{
  byte sCount = 0;
  CFlags = 0;
  /* Before Start : Select required setup */
  detachInterrupt(digitalPinToInterrupt(SingleWireI));
  digitalWrite(SingleWireO, LOW);

  /* Segmentation */
  seg0 = TxData & SEG_MASK;
  seg1 = (TxData >> 8) & SEG_MASK;
  
  /* Encoding */
  seg0 = PDC_Encode(seg0, 0);
  seg1 = PDC_Encode(seg1, 1);

  /* Combine Flags */
  CFlags = ((Flags1 << 2) & 0x0C) | Flags0;

  /* Sub-Segmentation */
  subSeg0 = seg0 & SUB_SEG_MASK;
  subSeg1 = (seg0 >> 4) & SUB_SEG_MASK;
  subSeg2 = seg1 & SUB_SEG_MASK;
  subSeg3 = (seg1 >> 4) & SUB_SEG_MASK;
  
  /* Transmit */
  for(int sIt=0; sIt<SEG_COUNT; sIt++)
  {
    switch (sIt) {
      case 0: sCount = CFlags; break;
      case 1: sCount = subSeg0; break;
      case 2: sCount = subSeg1; break;
      case 3: sCount = subSeg2; break;
      case 4: sCount = subSeg3; break;
      case 5: sCount = (STOP_PULSES - 1); break;
      default: sCount = 0; break;
    }

    /* Send Pulse Stream */
    for(int i=0; i<=sCount; i++)
    {
#ifdef PULSE_TIME_MS
      delay(PULSE_MS); // ms
#else
      delayMicroseconds(PULSE_US); // us
#endif
      digitalWrite(SingleWireO, HIGH);
#ifdef PULSE_TIME_MS
      delay(PULSE_MS); // ms
#else
      delayMicroseconds(PULSE_US); // us
#endif
      digitalWrite(SingleWireO, LOW);
    }

    /* Send Delay */
    for(int i=0; i<DELAY_PULSES; i++)
    {
#ifdef PULSE_TIME_MS
      delay(PULSE_MS); // ms
#else
      delayMicroseconds(PULSE_US); // us
#endif
      digitalWrite(SingleWireO, LOW);
#ifdef PULSE_TIME_MS
      delay(PULSE_MS); // ms
#else
      delayMicroseconds(PULSE_US); // us
#endif
      digitalWrite(SingleWireO, LOW);
    }
  }

   delayMicroseconds(STREAM_DELAY_US); // us
  /* Before Leaving : Place back in original state */
  attachInterrupt(digitalPinToInterrupt(SingleWireI), PDC_I_ISR, FALLING);
}

/* ============================================================= */
/* PDC Ini Receiver */
/* ============================================================= */
void PDC_IniReceive()
{
  // Set to INI RX delay
  timeInUsec = RX_INI_LENGTH;

  while (Pstream == 0){}
  Pstream = 0;
  count = 0;

  // Restore to normal RX delay
  timeInUsec = RX_DELAY;
}

/* ============================================================= */
/* PDC Receiver */
/* ============================================================= */
unsigned int PDC_Receive()
{
  CFlags = 0;
  
  for(int sIt=0; sIt<SEG_COUNT; sIt++)
  {
    while (Pstream == 0){}
    count--;
    
    switch (sIt)
    {
      case 0: CFlags = count; break;
      case 1: subSeg0 = count; break;
      case 2: subSeg1 = count; break;
      case 3: subSeg2 = count; break;
      case 4: subSeg3 = count; break;
    }
    
    Pstream = 0;
    count = 0;
  }

  /* Sub-combination */
  seg0 = subSeg0 | (subSeg1 << 4);
  seg1 = subSeg2 | (subSeg3 << 4);

  /* Seperate Flags */
  Flags0 = CFlags & 0x03;
  Flags1 = ((CFlags >> 2) & 0x03);

  /* Decoding */
  seg0 = PDC_Decode(seg0, Flags0);
  seg1 = PDC_Decode(seg1, Flags1);

  /* Combination and Return */
  return (seg0 | (seg1 << 8));
  
}

/* ============================================================= */
/* PDC Encoder */
/* ============================================================= */
byte PDC_Encode(byte SegData, byte segNo)
{
  byte countOfOnes = 0;
  byte SegDataFlipped = 0;
  byte Flags = 0x00;

  countOfOnes = ((SegData>>7)&1)+((SegData>>6)&1)+((SegData>>5)&1)+((SegData>>4)&1)+
                ((SegData>>3)&1)+((SegData>>2)&1)+((SegData>>1)&1)+(SegData&1);
  if(countOfOnes > ON_BITS_LIMIT)
  {
    SegData = ~SegData;
    Flags = 0x02;
  }

  SegDataFlipped = reverse(SegData);
  if(SegDataFlipped < SegData)
  {
    SegData = SegDataFlipped;
    Flags = Flags | 0x01;
  }

  if(segNo == 0){
    Flags0 = Flags;
  } else {
    Flags1 = Flags;
  }
  return SegData;
}

byte reverse(byte b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

/* ============================================================= */
/* PDC Decoder */
/* ============================================================= */
byte PDC_Decode(byte SegData, byte Flags)
{
  if(Flags == 0x01){
    SegData = reverse(SegData);
  }
  else if(Flags == 0x02){
    SegData = ~SegData;
  }
  else if(Flags == 0x03){
    SegData = reverse(SegData);
    SegData = ~SegData;
  }
  
  return SegData;
}

/* ============================================================= */
/* PDC Input ISR */
/* ============================================================= */
void PDC_I_ISR()
{
  CurieTimerOne.pause();  // Pause Timer
  count++;
  CurieTimerOne.restart(timeInUsec);  // Restarts Timer
}

/* ============================================================= */
/* Timer ISR */
/* ============================================================= */
void timedIsr()
{
  CurieTimerOne.pause();  // Pause Timer
  Pstream = 1;
  //Serial.println("In Time Int");
  //CurieTimerOne.restart(timeInUsec);  // Restarts Timer
}
