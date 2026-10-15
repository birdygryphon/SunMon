

#include "LPC23xx.h"


#include "FreeRTOS.h"
#include "semphr.h" 
#include "task.h"
#include "serial.h"
#include "MetersIncludes.h"


static void advanceHistoryArray(void);


void crc16_ccitt(char ser_data);

#define solarSTACK_SIZE		configMINIMAL_STACK_SIZE+200

#define SOLAR_READ_DELAY         500

/* after what inactivity time (ms) do we determine the solar FSM as stuck? */
#define SOLAR_FSM_STUCK_TIMEOUT  2000

/* how often (in ms) do we query the timer? */
#define SOLAR_TIME_CHECK_INTERVAL (60 * 1000)


/* means "unknown value", converted to "U" on rrdtool script side */
#define SOLAR_FSM_INVALID_VALUE  0x4000 /* 16384 */



int SolarHistory[SOLAR_WATT_HISTORYSIZE][3]; /* three watt values: P1, P2, P3 */
int historyRemainTime = SOLAR_WATT_HISTORY_STEPSIZE; /* how many ticks have we left until the next history entry must be written? */


unsigned long correctVoltWattRx = 0; /* counter of correctly received wattage/voltage packets */
unsigned long failedVoltWattRx = 0;  /* counter of incorrectly received wattage/voltage packets */
unsigned long queryTimeCounter = 0;  /* counter attempted queries for the time */

int resultValueRaw = 0; /* last yield result value (raw) */ 

static unsigned short staticCRC;

static unsigned short calculatedCrc2Byte; /* last calculated CRC (by checkRxTwoByteCheckSum() ) */
static unsigned char calculatedCrc1Byte; /* last calculated CRC (by checkRxOneByteCheckSum() ) */

// Pin I/O LED Control Maskbit
#define  DIR_RS485   0x00080000   		// P1.19(0000 0000 0000 x000 0000 0000 0000 0000)


#define RX_UART3_BUF_SIZE 100
#if RX_UART3_BUF_SIZE > 254
# error "RX_UART3_BUF_SIZE too large!"
#endif


#define SOL_NO_INIT   0 /* no init yet */
#define SOL_SENT_INIT 1 /* sent init string */
#define SOL_CHAN     2 /* querying channel currentChannel */
#define SOL_CHAN_WAIT_REPLY 3 /* waiting for reply on channel currentChannel = 0; */
#define SOL_QUERY_TIME 4 /* querying the time on the SC */
#define SOL_WAIT_FOR_TIME_RSP 5 /* waiting for reply on time query */
#define SOL_QUERY_LASTDAY_YIELD 6 /* querying yield data */
#define SOL_QUERY_LASTDAY_YIELD_WAIT_REPLY 7 /* waiting for yield data reply */
#define SOL_QUERY_LASTHOUR_YIELD 8 /* querying yield data */
#define SOL_QUERY_LASTHOUR_YIELD_WAIT_REPLY 9 /* waiting for yield data reply */
#define SOL_FINISHED 0xA /* finished */
//#define SOL_
//#define SOL_


// UART/FIFO Register Bit Masks

#define UIIR_PEND         1U
#define UIIR_RLS_INT      (3U << 1)
#define UIIR_RDA_INT      (2U << 1)
#define UIIR_CTI_INT      (6U << 1)
#define UIIR_THRE_INT     (1U << 1)

#define ULSR_RDR          1U


/* Handle to the com port (UART3) */
  xComPortHandle xPort;

unsigned int uart3TxRunning = 0; /* flag to tell if we are currently sending a datagram */


unsigned int rxBuf3NextFreeRxPos = 0;

unsigned char currentChannel; /* the SolarCount channel we currently query (here: 0 to 3) */

static int solarState = SOL_QUERY_TIME; /* state of the SolarCount communication state machine */


portTickType xReadRate, xLastReadTime;


unsigned char rxUART3[RX_UART3_BUF_SIZE];
int uart3_buf[50];

unsigned char scDate[6]; /* date received from solar count */


int dayRxYield[NUMBER_OF_SOLAR_CHANNELS]; /* daily yield data (last day) */
int hourRxYield[NUMBER_OF_SOLAR_CHANNELS]; /* hourly yield data (last hour) */

const unsigned char query_sc_time[] = "\x02\xB0\xF0\x03\x00\x04\xAB\xB9\x03"; // Zeit abfragen
const unsigned char query_sc_time_rsp[] = "\x02\xF0\xB0\x08\x00\x84\x00\x00\x00\x00\x00\x00\x00\x03"; // Zeit abfragen
const unsigned char query_sc_time_r_m[] = "\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // Zeit abfragen

// Query month
// Time Stamp:Year=10:Month=1
// ---> 01-Jun-2010 14:41:21 : 02 31 3c 4d 30 31 30 30 30 31 b0 03
// <--- 01-Jun-2010 14:41:21 : 02 31 4d 30 31 30 30 30 31 30 30 30 30 31 34 38 36 aa 03 03

// tt (time) values: m=minute , h = hour, D = day, M = month (\x31), Y = year
// cc (channel) values: x30 + channel number
// m1: month BCD msb, m2: month BCD lsb
// ck: checksum
// ad: unit address
//                                       ad      tt  cc  m1  m2  ck
//specification: unsigned char req_month_yield[] = "\x02\x31\x3c\x4d\x30\x31\x30\x30\x03"; // Frame for querying the yield. Will be filled in by solQueryYield()

//                                  
unsigned char req_month_yield[] = "\x02\x31\x3C\x6d\x30\x32\x30\x30\x30\x32\xff\x03"; // Frame for querying the monthly yield. Will be filled in by solQueryYield()


unsigned char req_minute_yield[] = "\x02\x31\x3C\x6D\x30\x32\x30\x30\x30\x32\x00\x03"; // Frame for querying the minutely yield. Will be filled in by solQueryYield()
const unsigned char yield_minute_rsp[] = "\x02\x31\x6D\x30\x31\x30\x30\x30\x32\x30\x30\x30\x30\x30\x32\x41\x44\xC8\x03\x03";
const unsigned char yield_minute_r_m[] = "\xFF\xFF\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF";


unsigned char req_day_yield[]    =  "\x02\x31\x3c\x44\x30\x32\x30\x30\x30\x32\x00\x03"; // Frame for querying the minutely yield. Will be filled in by solQueryYield()
const unsigned char yield_day_rsp[]     = "\x02\x31\x44\x30\x31\x30\x30\x30\x32\x30\x30\x30\x30\x30\x32\x41\x44\xC8\x03\x03";
const unsigned char yield_day_r_m[]     = "\xFF\xFF\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF";


unsigned char req_hour_yield[]    = "\x02\x31\x3c\x68\x30\x32\x30\x30\x30\x32\x00\x03"; // Frame for querying the minutely yield. Will be filled in by solQueryYield()
const unsigned char yield_hour_rsp[]    = "\x02\x31\x68\x30\x31\x30\x30\x30\x32\x30\x30\x30\x30\x30\x30\x30\x30\x30\x03\x03";
const unsigned char yield_hour_r_m[]    = "\xFF\xFF\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF";






// tt (time) values: m=minute , h = hour, D = day, M = month (\x31), Y = year
// cc (channel) values: x30 + channel number
// m1: month BCD msb, m2: month BCD lsb
// ck: checksum
// ad: unit address
// r1 to r7: result binary coded hex


//                                   02  31  4d  30  30  30  30  30  31  30  30  30  30  31  35  44  43
//                                   00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16
unsigned char yield_month_rsp[] = "\x02\x31\x4d\x33\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03"; /* exp. query yield response. Will be filled in by solQueryYield() */
unsigned char yield_month_r_m[] = "\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff"; /* exp. query yield seq response mask. Will be filled in by solQueryYield() */


//char init1[]  = "abcdef";
const unsigned char init1[]     = "\x02\xB0\xF0\x04\x00\x21\x00\xCC\x35\x03"; // 1. init seq to SolarCount
const unsigned char init1_rsp[] = "\x02\xf0\xb0\x08\x00\x84\x09\x0c\x04\x0a\x26\x86\xcd\x00\x03"; /* exp. init seq response */
const unsigned char init1_r_m[] = "\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff"; /* exp. init seq response mask */
/* init1_r_m: response after first init: \x02\xf0\xb0\x08 , later: \x02\xf0\xb0\x09 */


const unsigned char reqchan0[] = "\x02\xB0\xF0\x04\x00\x20\x00\xFD\x06\x03"; // request channel 0
const unsigned char reqchan1[] = "\x02\xB0\xF0\x04\x00\x20\x01\xDC\x16\x03"; // request channel 1
const unsigned char reqchan2[] = "\x02\xB0\xF0\x04\x00\x20\x02\xBF\x26\x03"; // request channel 2
const unsigned char reqchan3[] = "\x02\xB0\xF0\x04\x00\x20\x03\x9E\x36\x03"; // request channel 3
const unsigned char reqchan4[] = "\x02\xB0\xF0\x04\x00\x20\x04\x79\x46\x03"; // request channel 4
const unsigned char reqchan5[] = "\x02\xB0\xF0\x04\x00\x20\x05\x58\x56\x03"; // request channel 5
const unsigned char init3[] = "\x02\xF0\xB0\x0A\x00"; // 3. init seq. to SolarCount


const unsigned char init_after_all_chan[]     = "\x02\xb0\xf0\x04\x00\x21\x00\xcc\x35\x03"; // send to SC after all channels have been scanned
// ^^ equals init1[]

const unsigned char init_after_all_chan_rsp[] = "\x02\xf0\xb0\x09\x00\xa1\xfd\x03\x00\x00\x00\x00\x07\x21\x03"; // response from SC to init_after_all_chan[]
const unsigned char init_after_all_chan_r_m[] = "\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // mask for init_after_all_chan_rsp

const unsigned char reqchan0_rsp[] = "\x02\xf0\xb0\x0a\x00\xa0\x00\x87\x00\x00\x00\xec\x00\xf2\x3f\x03"; // response to request channel 0
const unsigned char reqchan0_r_m[] = "\xff\xff\xff\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // response to request channel 0: mask
const unsigned char reqchan1_rsp[] = "\x02\xf0\xb0\x0a\x00\xa0\x01\x87\x00\x00\x00\xec\x00\xf2\x3f\x03"; // response to request channel 1
const unsigned char reqchan1_r_m[] = "\xff\xff\xff\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // response to request channel 1: mask
const unsigned char reqchan2_rsp[] = "\x02\xf0\xb0\x0a\x00\xa0\x02\x87\x00\x00\x00\xec\x00\xf2\x3f\x03"; // response to request channel 2
const unsigned char reqchan2_r_m[] = "\xff\xff\xff\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // response to request channel 2: mask
const unsigned char reqchan3_rsp[] = "\x02\xf0\xb0\x0a\x00\xa0\x03\x87\x00\x00\x00\xec\x00\xf2\x3f\x03"; // response to request channel 3
const unsigned char reqchan3_r_m[] = "\xff\xff\xff\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // response to request channel 3: mask
const unsigned char reqchan4_rsp[] = "\x02\xf0\xb0\x0a\x00\xa0\x04\x87\x00\x00\x00\xec\x00\xf2\x3f\x03"; // response to request channel 4:
const unsigned char reqchan4_r_m[] = "\xff\xff\xff\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // response to request channel 4: mask
const unsigned char reqchan5_rsp[] = "\x02\xf0\xb0\x0a\x00\xa0\x05\x87\x00\x00\x00\xec\x00\xf2\x3f\x03"; // response to request channel 5
const unsigned char reqchan5_r_m[] = "\xff\xff\xff\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\xff"; // response to request channel 5: mask



unsigned char chanVolt[6]; /* holds the voltages measured on channel 0 to 5 */
int chanWatt[6]; /* holds the wattages measured on channel 0 to 5 */


portTickType xLastSolarStateChangeTime;

portTickType xLastTimeCheckTime;


 /* time when the elements of the history array were last moved on one step */
portTickType xLastSolarHistoryAdvance;



int solarReadErrors = 0;

int solarReadCounter = 0;


int solarStateWhenFail = 0xFF;
int currentChannelWhenFail = 0xFF;

/* pototype  section */

void init_serial3 (void); 									// Init UART-3


void putchar3(unsigned char ch);                                                // Put Char To UART-3
unsigned char getchar3(unsigned char * rxTrue);  				// Get Char From UART-3
void send_uart3(char * data, int length);					// Print String to UART3
void delay(unsigned long int);							// Delay Time Function
void solarFSM(void);
unsigned char checkUart3Received(
     const unsigned char * compareBuf, 
     const unsigned char * compareBufMask, 
     int lengthCompare);


void RXD_RS485(void)
{
  IOCLR1 = DIR_RS485;		// RS485 Direction = 0 (Receive)
} /* RXD_RS485 */

void TXD_RS485(void) 
{
  //delay(500000);
  IOSET1 = DIR_RS485;		// RS485 Direction = 1 (Transmit)
} /* TXD_RS485 */


void initSolarFSM(void)
{
  int i;
  int k;

  xLastSolarStateChangeTime = xTaskGetTickCount();
  xLastTimeCheckTime = xTaskGetTickCount();
  xLastReadTime = xTaskGetTickCount();
  solarState = SOL_QUERY_TIME;
  uart3TxRunning = 0; /* reset flag to tell if we are currently sending a datagram */
  rxBuf3NextFreeRxPos = 0;
  currentChannel = 0;

  /* set recent data item to invalid value */
  chanWatt[0] = SOLAR_FSM_INVALID_VALUE;
  chanWatt[1] = SOLAR_FSM_INVALID_VALUE;
  chanWatt[2] = SOLAR_FSM_INVALID_VALUE;

  /* do not invalidate history here so we do not lose all data */

  /* init daily rx yield array with 0 */
  for (i = 0; i < NUMBER_OF_SOLAR_CHANNELS; i++)
  {
      dayRxYield[i] = 65535; /* default: invalid/unset value */
  } /* for */


} /* initSolarFSM */



/* Clear Uart3 RxBuf and pointer to next Rx element */
void resetUart3RxBuf(void)
{
  int i;
  for (i = 0; i < RX_UART3_BUF_SIZE; i++)
  {
    rxUART3[i] = 0; // clear byte
  }
  rxBuf3NextFreeRxPos = 0; // buffer empty
}


/* Get data from Uart3 RxBuf if there is any, copy it to rxUART3 buffer */
void getUart3RxData(void)
{
  signed portBASE_TYPE byteReceived; /* did we receive a byte? */

  const portTickType xBlockTime = ( portTickType ) 0; /* ( portTickType ) 0xffff / portTICK_RATE_MS; */
  unsigned char RxBuf; // one rx char, or 0x100 if no data
  //unsigned char rxTrue; // did we receive a byte?

  while (rxBuf3NextFreeRxPos + 1 < RX_UART3_BUF_SIZE)
  {
    /* while not overrun */

    //RxBuf = getchar3(&rxTrue);

    byteReceived = xSerialGetChar( xPort, &RxBuf, xBlockTime );


    if ((rxBuf3NextFreeRxPos == 0) && (byteReceived == pdTRUE) && (RxBuf == 0x00))
    {
      /* we are at position 0, and we have received a 0x00 byte. ignore this leading 0. */
     ; /* do nothing */
    }
    else if (byteReceived == pdTRUE)
    {
       // if data available
       rxUART3[rxBuf3NextFreeRxPos] = RxBuf;
       rxBuf3NextFreeRxPos++;
    } /* if */
    else
    {
      break; /* no more data. break the while loop, finished. */
    } /* else */
  } /* while */
} /* resetUart3RxBuf */


/* Remove the first n bytes of Uart3 RxBuf, also adjust pointer to next Rx element */
void chopUart3RxBuf(int n)
{
  int i;

  // for now, clear the whole buffer
  resetUart3RxBuf();


  // begin quickfix (TODO)



  //if (n > RX_UART3_BUF_SIZE)
  //{
  //  /* n too large */
  
  
  //resetUart3RxBuf ();



  //  return;
  //}
  /* move end part of the buffer (after n) to front */
  //for (i = 0; i < n; i++)
  //{
  //  rxUART3[i] = rxUART3[i+n]; // move byte
  //}
  /* write 0 to now unused elements */
  //for (i = n; i < RX_UART3_BUF_SIZE; i++)
  //{
  //  rxUART3[i] = 0; // clear byte
  //}

  /* clear all elements */

 


  /* adjust rxBuf3NextFreeRxPos */ 
  //if (n >= rxBuf3NextFreeRxPos)
  //{
  //  /* remove more elements than are in the buffer */
  //  rxBuf3NextFreeRxPos = 0; // buffer empty
  //}
  //else
  //{
  //  rxBuf3NextFreeRxPos -= n;
  //}
 // end quickfix (TODO)

}/* chopUart3RxBuf */




/*
 * checkUart3Received
 *
 * This routine checks if the bit pattern specified in compareBuf was received. Only bytes with the value
 * 0xff in the corresponding index of compareBufMask are compared, others are don't care.
 * lengthCompare bytes are compared. Comparison is always done from byte 0 of compareBuf and CompareBufMask.
 *
 *
 */
unsigned char checkUart3Received(const unsigned char * compareBuf, const unsigned char * compareBufMask, int lengthCompare)
{

  /* first check if we received enough bytes */
  int numRxBytes; 
  int k;
  int result;
  volatile unsigned char * testRx;
  volatile unsigned const char * testCmp;
  volatile unsigned const char * testMask;

  numRxBytes = rxBuf3NextFreeRxPos;

  result = 0; /* begin: no mismatch */
  if (numRxBytes >= lengthCompare )
  {
    /* we have received enough bytes */
    for (k = 0; k < lengthCompare; k++)
    {
      if ( (compareBufMask[k] == 0xff) && (compareBuf[k] != rxUART3[k]) )
      {
        testRx = &(rxUART3[0]);
        testCmp = &(compareBuf[0]);
        testMask = &(compareBufMask[0]);
        result = 1; /* we compare this bit (mask 0xff), and a mismatch occurred */
        break;
      }
      else
      {
        /* nothing, compare further */ ;
      } /* else */
    } /* for */
    if (result == 1)
    {
      return k; /* mismatch, return first index that did not match */
    }
    else
    {
      return 0; /* no mismatch, we got the desired byte pattern */
    }
  
  } /* if */
  else
  {
    /* we have not received enough bytes (yet) */
    return 0xff;
  } /* else */

} /* checkUart3Received */


void solarFSM(void)
{
  unsigned char i;
  int cmpResult;
  unsigned char power; /* variable for conversions */
  unsigned char checkDate; /* temp variable for day of month */
  unsigned char checkHour; /* temp variable for hour */
  unsigned char rxYieldChkSum; /* temp checksum for rx yield */


  /* always get Rx data */
  getUart3RxData();

  /* did the SolarFSM get stuck during reading? */
  if (xTaskGetTickCount() > (xLastSolarStateChangeTime + SOLAR_FSM_STUCK_TIMEOUT))
  {
    /* something has gone wrong (possible Rx Timeout). restart, state SOL_QUERY_TIME */
    xLastSolarStateChangeTime = xTaskGetTickCount();

    /* save error info */
    solarStateWhenFail = solarState;
    currentChannelWhenFail = currentChannel;
    
    initSolarFSM();
    solarReadErrors++;

  }
  else if (xTaskGetTickCount() < xLastSolarStateChangeTime)
  {
    /* overflow of timer, just recover. delay will be wrong. */
    xLastSolarStateChangeTime = xTaskGetTickCount();
  }


  if (xTaskGetTickCount() > (xLastSolarHistoryAdvance + SOLAR_WATT_HISTORY_STEPSIZE))
  {
    advanceHistoryArray();
    xLastSolarHistoryAdvance = xTaskGetTickCount(); /* update timer */
  }
  else if (xTaskGetTickCount() < xLastSolarHistoryAdvance)
  {
    /* overflow of timer, just recover. delay will be wrong. */
    xLastSolarHistoryAdvance = xTaskGetTickCount();
  }


  switch (solarState)
  {
    case SOL_NO_INIT:
      resetUart3RxBuf(); /* init */
      currentChannel = 0; /* the channel we query first */

       /* send init 1 */
      send_uart3((unsigned char *)init1, sizeof(init1));
      xLastSolarStateChangeTime = xTaskGetTickCount();
      solarState = SOL_SENT_INIT; /* now wait for reply */
    break;

    case SOL_SENT_INIT:
    /* wait for reply to init, check if full response has arrived */
    cmpResult = checkUart3Received(init1_rsp, init1_r_m,  sizeof(init1_rsp)-1);
    if (cmpResult == 0)
    {
      /* we have got a match! */
      
      solarReadCounter++;
      /* copy Rx data */
      /* TODO */


      /* finally, discard data from buffer */
      resetUart3RxBuf();

      /* now ready to query channel 0 data in next state */
      xLastSolarStateChangeTime = xTaskGetTickCount();
      solarState = SOL_CHAN;
    }
    else
    {
      /* no match (yet), stay in state */
    }
    break;


    case SOL_QUERY_TIME:

      resetUart3RxBuf(); /* init */

       /* send query_sc_time */
      send_uart3((unsigned char *)query_sc_time, sizeof(query_sc_time));
      xLastSolarStateChangeTime = xTaskGetTickCount();
      queryTimeCounter++;
      solarState = SOL_WAIT_FOR_TIME_RSP; /* query sent wait for response */
    break;


    case SOL_WAIT_FOR_TIME_RSP:


      cmpResult = checkUart3Received(query_sc_time_rsp, query_sc_time_r_m,  sizeof(query_sc_time_rsp)-1);
      if (cmpResult == 0)
      {
          /* we have got a match! */
        
          if (checkRxTwoByteCheckSum(&(rxUART3[0]), 1, 10, 11) == 1) /* checksum in bytes 11 and 12, check bytes 1 to 10 */
          {

            /* use data only if checksum correct */
    
            scDate[0] = rxUART3[6]; /* year */
            scDate[1] = rxUART3[7]; /* month */
            scDate[2] = rxUART3[8]; /* day */
            scDate[3] = rxUART3[9]; /* hour */
            scDate[4] = rxUART3[10]; /* min */

          }
          else
          {
            /* time rx checksum error */ ;
          }

        /* finally, discard data from buffer */
        resetUart3RxBuf();

        xLastSolarStateChangeTime = xTaskGetTickCount();

        currentChannel = 0; /* start yield scan on channel 0 */
        solarState = SOL_QUERY_LASTDAY_YIELD; //xxxxxxx SOL_NO_INIT; //SOL_QUERY_YIELD; /* change state: time response received, now init and query the channels */
      }
      else
      {
        /* stay in state */
      }
    
    break;

////// begin day queries

    case SOL_QUERY_LASTDAY_YIELD:

        xLastSolarStateChangeTime = xTaskGetTickCount();

        if (currentChannel >= 6)
        {
          currentChannel = 0;
          solarState = SOL_QUERY_LASTHOUR_YIELD; /* we are done with all the channels. go ahead */
        }
        else
        {
          /* query the next channel */

          checkDate = scDate[2]-1; /* we query the previous day */

          if (checkDate == 0)
          {
            /* first day of month */

            /* check new month */
            if (scDate[1] == 3)
            {
              /* looking for end of february? handle leap years */
              if ((scDate[0] % 4) == 0)
              {
                checkDate = 29; /* leap year */
              }
              else
              {
                checkDate = 28; /* not a leap year */
              }
            }
            else if 
                 ((scDate[1] == 1) ||
                 (scDate[1] == 1+1) ||
                 (scDate[1] == 3+1) ||
                 (scDate[1] == 5+1) ||
                 (scDate[1] == 7+1) ||
                 (scDate[1] == 8+1) ||
                 (scDate[1] == 10+1))
                 { 
                    checkDate = 31;
                 }
                 else
                 {
                    checkDate = 30;
                 }
          }

          send_yield_query_day(/* channel: */ currentChannel, /* day: (yesterday) */ checkDate );
          xLastSolarStateChangeTime = xTaskGetTickCount();
          solarState = SOL_QUERY_LASTDAY_YIELD_WAIT_REPLY; /* query next channel. change state: now wait for response */

        }

    break; /* end of case SOL_QUERY_LASTDAY_YIELD */

    case SOL_QUERY_LASTDAY_YIELD_WAIT_REPLY:

       cmpResult = checkUart3Received(yield_day_rsp, yield_day_r_m,  sizeof(yield_day_rsp)-1);

      /* wait for reply to request, check if full response has arrived */
      if (cmpResult == 0)
      {
          /* we have got a match (i.e. expected frame received) ! */
          
          /* calculate the checksum of the received data */

          rxYieldChkSum = calculate_crc8(  (&(rxUART3[0])), (sizeof(yield_day_rsp)-2));

          if (rxYieldChkSum == rxUART3[17])
          {
            /* checksum is correct! */

            if (currentChannel <= 5)
            {
  
                /* convert and copy value */
                dayRxYield[currentChannel] =
                  extractBinaryHexDigitString( &(rxUART3[0]), 13, 16 );
  
            } /* if */
            else
            {
              /* all channels done. nothing to do, we leave via SOL_QUERY_LASTDAY_YIELD */
            }
          }
          else if (currentChannel <= 5)
          {
            /* valid channel index. but checksum is incorrect ! */
            dayRxYield[currentChannel] = 65535; /* invalid value */
          }
          else
          {
            /* checksum incorrect and channel index out of range. do nothing. */
          }

          /* always move on to the next value... */

          currentChannel++; /* increment channel counter so we query the next channel in the next round */
          xLastSolarStateChangeTime = xTaskGetTickCount();
          solarState = SOL_QUERY_LASTDAY_YIELD; /* go ahead */
     }

    break; /* end of case SOL_QUERY_LASTDAY_YIELD_WAIT_REPLY */

////// end day queries


////// begin hour queries

   case SOL_QUERY_LASTHOUR_YIELD:

        xLastSolarStateChangeTime = xTaskGetTickCount();

        if (currentChannel >= 6)
        {
          currentChannel = 0;
          solarState = SOL_NO_INIT; /* we are done with all the channels. go ahead */
        }
        else
        {
          /* query the next channel */

          checkHour = scDate[3]; /* we query the previous hour. From doc: 22 means 21h to 22h */
          
          if (checkHour == 0)
          {
              checkHour = 23; /* in case of hour 0: we want to get the 23h to midnight yield */ 
          }

          send_yield_query_hour(/* channel: */ currentChannel, /* hour: last hour */ checkHour-1 );
          xLastSolarStateChangeTime = xTaskGetTickCount();
          solarState = SOL_QUERY_LASTHOUR_YIELD_WAIT_REPLY; /* query next channel. change state: now wait for response */

        }

    break; /* end of case SOL_QUERY_LASTDAY_YIELD */

    case SOL_QUERY_LASTHOUR_YIELD_WAIT_REPLY:

       cmpResult = checkUart3Received(yield_hour_rsp, yield_hour_r_m,  sizeof(yield_hour_rsp)-1);

      /* wait for reply to request, check if full response has arrived */
      if (cmpResult == 0)
      {
          /* we have got a match (i.e. expected frame received) ! */
          
          /* calculate the checksum of the received data */

          rxYieldChkSum = calculate_crc8(  (&(rxUART3[0])), (sizeof(yield_hour_rsp)-2));

          if (rxYieldChkSum == rxUART3[17])
          {
            /* checksum is correct! */

            if (currentChannel <= 5)
            {

                /* convert and copy value. hourly values must be divided by 60 before feeding it to a DB */
                hourRxYield[currentChannel] = 
                   extractBinaryHexDigitString( &(rxUART3[0]), 10, 16 );
  
            } /* if */
            else
            {
              /* all channels done. nothing to do, we leave via SOL_QUERY_LASTHOUR_YIELD */
            }
          }
          else if (currentChannel <= 5)
          {
            /* valid channel index. but checksum is incorrect ! */
            hourRxYield[currentChannel] = 65535; /* invalid value */
          }
          else
          {
            /* checksum incorrect and channel index out of range. do nothing. */
          }

          /* always move on to the next value... */

          currentChannel++; /* increment channel counter so we query the next channel in the next round */
          xLastSolarStateChangeTime = xTaskGetTickCount();
          solarState = SOL_QUERY_LASTHOUR_YIELD; /* go ahead */
     }

    break; /* end of case SOL_QUERY_LASTHOUR_YIELD_WAIT_REPLY */


///// end hour queries












    case SOL_CHAN:
    /* send query for channel number currentChannel */

     vTaskDelayUntil( &xLastReadTime, SOLAR_READ_DELAY );
     switch ( currentChannel )
     {
      case 0:
        send_uart3((unsigned char *) reqchan0, sizeof(reqchan0));
        break;
      case 1:
        send_uart3((unsigned char *) reqchan1, sizeof(reqchan1));
        break;
      case 2:
        send_uart3((unsigned char *) reqchan2, sizeof(reqchan2));
        break;
      case 3:
        send_uart3((unsigned char *) reqchan3, sizeof(reqchan3));
        break;
      case 4:
        send_uart3((unsigned char *) reqchan4, sizeof(reqchan4));
        break;
      case 5:
        send_uart3((unsigned char *) reqchan5, sizeof(reqchan5));
        break;
      case 6:
        /* we are through with all the channels, ready to send next init for next query round */
        xLastSolarStateChangeTime = xTaskGetTickCount();
        solarState = SOL_FINISHED;
        currentChannel = 0;

        /* new duplicated code: */

        resetUart3RxBuf();


        if (xTaskGetTickCount() > (xLastTimeCheckTime + SOLAR_TIME_CHECK_INTERVAL))
        {
           /* time to get the time info again! */
           xLastTimeCheckTime = xTaskGetTickCount();
           solarState = SOL_QUERY_TIME; /* restart the measurement cycle (with time check at first) */
        }
        else if (xTaskGetTickCount() < xLastTimeCheckTime)
        {
          /* overflow of timer, just recover. delay will be wrong. */
          xLastTimeCheckTime = xTaskGetTickCount();
          solarState = SOL_NO_INIT; /* restart the measurement cycle (without time check at first) */
        }
        else
        {
          /* not time to get the time info. leave state at SOL_FINISHED. */
        }



        break;
      default:
        currentChannel = 0; /* current channel not 0 to 6, reset it. */
       /* nothing else to do */

     } /* switch */
     if ((solarState != SOL_FINISHED) && (solarState != SOL_QUERY_TIME))
     {
      xLastSolarStateChangeTime = xTaskGetTickCount();
      solarState = SOL_CHAN_WAIT_REPLY; /* wait for reply if not finished */
     } /* if */
    break;

    case SOL_CHAN_WAIT_REPLY:

      switch ( currentChannel )
      {
        case 0:
          cmpResult = checkUart3Received(reqchan0_rsp, reqchan0_r_m,  sizeof(reqchan0_rsp)-1);
          break;
        case 1:
          cmpResult = checkUart3Received(reqchan1_rsp, reqchan1_r_m,  sizeof(reqchan1_rsp)-1);
          break;
        case 2:
          cmpResult = checkUart3Received(reqchan2_rsp, reqchan2_r_m,  sizeof(reqchan2_rsp)-1);
          break;
        case 3:
          cmpResult = checkUart3Received(reqchan3_rsp, reqchan3_r_m,  sizeof(reqchan3_rsp)-1);
          break;
        case 4:
          cmpResult = checkUart3Received(reqchan4_rsp, reqchan4_r_m,  sizeof(reqchan4_rsp)-1);
          break;
        case 5:
          cmpResult = checkUart3Received(reqchan5_rsp, reqchan5_r_m,  sizeof(reqchan5_rsp)-1);
          break;
         case 6:
          /* wait for the response to the finishing init */
          // cmpResult = checkUart3Received(init_after_all_chan_rsp, init_after_all_chan_r_m,  sizeof(init_after_all_chan_rsp)-1);
          break;
        default:
          /* unknown channel */
          break;
      } /* switch */


      /* wait for reply to request, check if full response has arrived */
      if (cmpResult == 0)
      {
        /* we have got a match! */
        
        /* copy Rx data */
        /* TODO */
        if ((currentChannel >= 0) && (currentChannel < 6))
        {

          if (checkRxTwoByteCheckSum(&(rxUART3[0]), 1, 12, 13) == 1) /* checksum in bytes 13 and 14, check bytes 1 to 12 */
          {
            /* checksum correct */
            portENTER_CRITICAL();
            chanVolt[currentChannel] = rxUART3[11];
            chanWatt[currentChannel] = (int)(rxUART3[7]) | (((int)(rxUART3[8])) << 8) /* index 7: LSB, index 8: MSB */;
            correctVoltWattRx++;
            portEXIT_CRITICAL();
          }
          else
          {
            /* checksum incorrect */
            portENTER_CRITICAL();
            /* mark values as invalid */
            //chanVolt[currentChannel] = 1;
            chanWatt[currentChannel] = SOLAR_FSM_INVALID_VALUE;
            failedVoltWattRx++;
            portEXIT_CRITICAL();
          }

        } /* if */

        if (currentChannel != 6)
        {
          /* we received input from channel 0 to 5 */

          /* finally, discard data from buffer */
   
  
          /* switch to next channel */
          currentChannel++;

          /* finally, discard data from buffer */
          resetUart3RxBuf();

          /* now ready to query channel data for next channel */
          xLastSolarStateChangeTime = xTaskGetTickCount();
          solarState = SOL_CHAN;
        }
        else
        {

          /* I think we never go here ... */

          /* we received the finishing sequence */
          xLastSolarStateChangeTime = xTaskGetTickCount();

          resetUart3RxBuf();

          // may be duplicated
          if (xTaskGetTickCount() > (xLastTimeCheckTime + SOLAR_TIME_CHECK_INTERVAL))
          {
            /* time to get the time info again! */
            xLastTimeCheckTime = xTaskGetTickCount();
            solarState = SOL_QUERY_TIME; /* restart the measurement cycle (with time check at first) */
          }
          else if (xTaskGetTickCount() < xLastTimeCheckTime)
          {
            /* overflow of timer, just recover. delay will be wrong. */
            xLastTimeCheckTime = xTaskGetTickCount();
            solarState = SOL_NO_INIT; /* restart the measurement cycle (without time check at first) */
          }
          else
          {
            /* not time to get the time info. do a regular reading cycle without time */
            solarState = SOL_NO_INIT; /* restart the measurement cycle (without time check at first) */
          }
        } /* else */

      }
      else
      {
        /* no match (yet), stay in state */
      }
    break;



  } /* switch */


} /* solarFSM */


/*
 * Check if we have to save the current data to the next history array set, and do it
 * if necessary.
 *
 */
static void advanceHistoryArray(void)
{
  int i;

  /* advance entries by 1. we do not care about data locking here. */
  for (i = SOLAR_WATT_HISTORYSIZE-1; i > 0; i--)
  {
    portENTER_CRITICAL();
    SolarHistory[i][0] = SolarHistory[i-1][0];
    SolarHistory[i][1] = SolarHistory[i-1][1];
    SolarHistory[i][2] = SolarHistory[i-1][2];
    portEXIT_CRITICAL();
  } /* for */
  /* get the most recent values to index 0 */
  portENTER_CRITICAL();
  SolarHistory[0][0] = chanWatt[0];
  SolarHistory[0][1] = chanWatt[1];
  SolarHistory[0][2] = chanWatt[2];
  portEXIT_CRITICAL();

} /* advanceHistoryArray() */



void init_serial3 (void)  
{

  unsigned int baud = 57600;
  unsigned int divisor = get_uart_clk(3, OSCILLATOR_CLOCK_FREQUENCY) / (16 * baud);

  int i;

  for (i = 0; i < RX_UART3_BUF_SIZE; i++)
  {
    rxUART3[i] = 0;
  }

 /* init serial port */
 xPort = xSerialPortInitMinimal( baud /* baud rate */, 80 /* buffer length */ );
//vStartComTestTasks();

  // xxxx xxxx xxxx xxxx xxxx xxxx xxxx 1010
  PINSEL0 &= 0xFFFFFFF0;									// Reset P0.0,P0.1 Pin Config
  PINSEL0 |= 0x00000002;									// Select P0.0 = TxD(UART3)
  PINSEL0 |= 0x00000008;									// Select P0.1 = RxD(UART3)

  // Config P1.19 = Output Control Direction RS485
  // P1.19 = "0" = Receive  RS485
  // P1.19 = "1" = Transmit	RS485
  // xxxx xxxx xxxx xxxx 00xx xxxx xxxx xxxx
  PINSEL3 &= 0xFFFF3FFF;									// P1.19 = GPIO
  IODIR1 = DIR_RS485;										// Pin Control Direction RS485 = Output
  //RXD_RS485();												// Default RS485 Direction

   //PCLKSEL1 = 0;  /* UART3 frequency */


  // xxxx xx1x xxxx xxxx xxxx xxxx xxxx xxxx
  PCONP   |= 0x02000000;									// UART3 Power-ON



/* Configure UART */


 U3LCR = 0x83;                            					// 8 bits, no Parity, 1 Stop bit

 U3DLL = divisor & 0xFF;

 U3DLM = (divisor >> 8) & 0xFF;
 U3LCR = 0x03;                           	 				// DLAB = 0

// Baud Rate Calculator: http://prototalk.net/forums/showthread.php?t=11


}




/***************************/
/* Print String to UART[3] */
/***************************/
void send_uart3(char * data, int length)
{


  int i = 0;				// Pointer Buffer

  TXD_RS485();												
  //delay(50000);

  uart3TxRunning = 1; // sending in progress
  for (i = 0; i < length-1; i++)	// Get char & Print Until null
  {
   
    /* go only through length-1 because we do not send the terminating \x00 of the c-string in data */
    putchar3(data[i]); // Write char to UART
  }
  uart3TxRunning = 0; // done sending this datagram. The UART interrupt can switch to Rx as soon as FIFO is empty


  // we send directly. unused.
  // vSerialPutString( xPort, &(data[0]), length-1 );


  return;
}


/***********************/
/* Delay Time Function */
/*    1-4294967296     */
/***********************/
void delay(unsigned long int count1)
{
  volatile unsigned long int countVol;
  countVol = count1;
  while(countVol > 0) {countVol--;}								// Loop Decrease Counter	
}

/******************************/
/* Write Character To UART[3] */
/******************************/
void putchar3 (unsigned char ch)  
{  

  while (!(U3LSR & 0x20));									// Wait TXD Buffer Empty
  U3THR = ch;
}


/******************************/
/* Get character From UART[3] */
/******************************/
unsigned char getchar3(unsigned char * rxTrue)  
{
  
  if (!(U3LSR & 0x01))    // Check RXD Receive Data Ready
  {
    *rxTrue = 0; /* no data received */
    return 0; // no data available                                                                    
  }
  else
  {
    *rxTrue = 1; /* a byte was received */
    return (U3RBR);    // Get Receive Data & Return
  }
}



static portTASK_FUNCTION( vSolar_Task, pvParameters )
{
  int i;

  init_serial3();

  /* invalidate history */
  for (i = 0; i < SOLAR_WATT_HISTORYSIZE; i++)
  {
            SolarHistory[i][0] = SOLAR_FSM_INVALID_VALUE;
            SolarHistory[i][1] = SOLAR_FSM_INVALID_VALUE;
            SolarHistory[i][2] = SOLAR_FSM_INVALID_VALUE;
  }

  xLastSolarHistoryAdvance = xTaskGetTickCount();

  initSolarFSM();

  for( ;; )
  {



    //portENTER_CRITICAL();

    solarFSM();

    //portEXIT_CRITICAL();


   if (solarState == SOL_FINISHED)
   {
      solarState = SOL_NO_INIT;
   } /* if */
   
  }
}



void vStartSolarTask( unsigned portBASE_TYPE uxPriority )
{

  /* Spawn the task. */
  xTaskCreate( vSolar_Task, ( signed portCHAR * ) "Solar", solarSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}




portCHAR checkRxOneByteCheckSum(unsigned char* dataStart, unsigned char firstByte, unsigned char lastByte, unsigned char CheckSumStartByte)
{
  portCHAR retVal; /* return value */

// "\xf0\xb0\x0a\x00\xa0\x00\x87\x00\x00\x00\xec\x00\xf2\

  int x;

  unsigned long crcLONG;
  unsigned char crcCHAR;
  unsigned char crcCHARinv;
  unsigned char carry;

  crcLONG = 0;
  crcCHAR = 0;
  crcCHARinv = 0;
  carry = 0;


  /* use bytes 1 to 13 for the checksum */
  for (x = firstByte; x <= lastByte; x++)
  {

    crcLONG = (crcLONG + dataStart[x]); /* add and truncate to 8 bit */
  }

  carry = (unsigned char)(((crcLONG & 0xFF00) >> 8));

  crcCHAR = ((unsigned char)crcLONG) + carry;
  crcCHARinv = (unsigned char)(~crcCHAR);


  calculatedCrc1Byte = crcCHARinv; /* save calculated CRC (for use in case of checksum generation) */

  if (crcCHARinv == dataStart[CheckSumStartByte])
  {
    retVal = 1; /* checksum OK, data is valid */
  }
  else
  {
    retVal = 0; /* checksum error, data is invalid */
  }
  return retVal;
} /* checkRxOneByteCheckSum */


portCHAR checkRxTwoByteCheckSum(unsigned char* dataStart, unsigned char firstByte, unsigned char lastByte, unsigned char CheckSumStartByte )
{
  unsigned short cmpValue;
  portCHAR retVal; /* return value */
  portCHAR x;

  staticCRC = 0xFFFF; /* init 16 bit checksum CRC start value */

  /* use bytes first to last for the checksum */
  for (x = firstByte; x <= lastByte; x++)
  {

     crc16_ccitt(dataStart[x]);
  }



  cmpValue = ((((unsigned short)dataStart[CheckSumStartByte+1])<<8) | dataStart[CheckSumStartByte]);

  calculatedCrc2Byte = staticCRC; /* save calculated CRC (for use in case of checksum generation) */

  if (cmpValue == (staticCRC & 0xFFFF))
  {
    retVal = 1; /* checksum OK, data is valid */
  }
  else
  {
    retVal = 0; /* checksum error, data is invalid */
  }
  return retVal;

} /* portCHAR checkRxTwoByteCheckSum() */




void crc16_ccitt(char ser_data)
{
   staticCRC  = (unsigned char)(staticCRC >> 8) | (staticCRC << 8);
   staticCRC ^= ser_data;
   staticCRC ^= (unsigned char)(staticCRC & 0xff) >> 4;
   staticCRC ^= (staticCRC << 8) << 4;
   staticCRC ^= ((staticCRC & 0xff) << 4) << 1;
}




int extractBinaryHexDigitString(unsigned char* srcDataStart, 
                                    unsigned char firstSrcByte, 
                                    unsigned char lastSrcByte
                                    )
{
  unsigned char currentChar;
  unsigned char conversionError = 0;

  unsigned char x;
  int resultValue = 0; /* initial value */
  unsigned char round = 0; /* which digit? 0 = LSB */
  int currentCharValue;

  /* go through the bytes backwards (least significant to most significant) */
  for (x = lastSrcByte; x >= firstSrcByte; x--)
  {
    currentChar = srcDataStart[x];
    if ((currentChar < 0x30) || (currentChar > 0x46))
    {
      conversionError = 1;
    }
    else if (currentChar <= 0x39)
    {
      // 0x0 to 0x9
      currentCharValue = (srcDataStart[x]) - 0x30;
    }
    else
    {
      // 0xA to 0xF
      currentCharValue = 0xA + ((srcDataStart[x]) - 0x41);
    }
    resultValue = resultValue + (currentCharValue << (4 * round));

    round++;
  } /* for */

  /* if a conversion error occurred, nullify the result */
  if (conversionError == 1)
  {
      resultValue = 0xFFFF;
  }

  return resultValue;

 } /* extractBinaryHexDigitString() */


static unsigned char calculate_crc8(unsigned char * d_serdata_tuc, unsigned char nbytes)
{
   unsigned char i;
   unsigned char j;
   unsigned char d_currbyte_tuc;
   unsigned char d_crc_tuc;
   
   /* start with BISISTX and repeat the checksum calculation for all bytes until CHKS8 (excl.) */
    d_crc_tuc = 0; //   init the checksum

    for( j = 1; j < (nbytes-2); j ++  )
    {

      d_currbyte_tuc = d_serdata_tuc[ j ];

      for( i = 8; i > 0; i --  )
      {
         /*  if  current bit != 0 */
         if (( (d_crc_tuc ^ d_currbyte_tuc) & BIT0  ) != 0)
         {
            d_crc_tuc ^= 0x18;
            d_crc_tuc >>= 1;
            d_crc_tuc |= BIT7;
         }
         else
         {
          d_crc_tuc >>= 1;
         }
         d_currbyte_tuc >>= 1;   //       prepare for next databit
      } /* for */
    } // this byte is finished

  /* checksum is complete now - result always ORed with 0x80  and then compare */
  /* calculated and received checksum */
  d_crc_tuc |= BIT7;
  return d_crc_tuc;
}


void send_yield_query_day(unsigned char channel /* 0 to 5 */, unsigned char day /* 1 to 31 */)
{
        unsigned char newChkSum;

        req_day_yield[5] = 0x30 + channel; // channel ( 0 to 5 )


        convert_uchar_to_2_byte_hexencoding(day, &req_day_yield[8], &req_day_yield[9]);




        newChkSum = calculate_crc8(  (&(req_day_yield[0])), (sizeof(req_day_yield)-1));
        req_day_yield[10] = newChkSum; /* fill in checksum */

        resetUart3RxBuf();
        send_uart3((unsigned char *)req_day_yield, sizeof(req_day_yield));

} /* prepare_yield_query_day() */


void send_yield_query_hour(unsigned char channel /* 0 to 5 */, unsigned char hour /* 0 to 23 */)
{

        unsigned char newChkSum;
        unsigned char hourMSB;
        unsigned char hourLSB;

        req_hour_yield[5] = 0x30 + channel; // channel ( 0 to 5 )


        convert_uchar_to_2_byte_hexencoding(hour, &req_hour_yield[8], &req_hour_yield[9]);





        newChkSum = calculate_crc8(  (&(req_hour_yield[0])), (sizeof(req_hour_yield)-1));

        // crc
        req_hour_yield[10] = newChkSum; /* fill in checksum */

        resetUart3RxBuf();
        send_uart3((unsigned char *)req_hour_yield, sizeof(req_hour_yield));



} /* prepare_yield_query_hour() */


void send_yield_query_minute(unsigned char channel /* 0 to 5 */, unsigned char minute /* 0 to 59 */)
{
        unsigned char newChkSum;
        unsigned char minuteMSB;
        unsigned char minuteLSB;

        req_minute_yield[5] = 0x30 + channel; // channel ( 0 to 5 )

     
        minuteMSB = minute % 16;
        minuteLSB = minute - minuteMSB;
        
        req_minute_yield[6] = 0x30 + minuteMSB; // minute MSB (binary coded hex) 
        req_minute_yield[7] = 0x30 + minuteLSB; // minute LSB (binary coded hex)

        newChkSum = calculate_crc8(  (&(req_minute_yield[0])), (sizeof(req_minute_yield)-1));

        // crc
        req_minute_yield[10] = newChkSum; /* fill in checksum */

        resetUart3RxBuf();
        send_uart3((unsigned char *)req_minute_yield, sizeof(req_minute_yield));


} /* prepare_yield_query_minute() */

void convert_uchar_to_2_byte_hexencoding(unsigned char inValue, unsigned char * outMSB, unsigned char * outLSB)
{

  *outLSB = convertSingleDigitHex(inValue & 0xF);
  *outMSB = convertSingleDigitHex(((inValue >> 4) & 0xF));
}


void convert_uchar_to_2_byte_decencoding(unsigned char inValue, unsigned char * outMSB, unsigned char * outLSB)
{

  unsigned char msb;
  unsigned char lsb;

  lsb = (inValue % 10); /* remainder of division by 10 */
  msb = (inValue - lsb) / 10;

  /* now convert to ASCII chars */
  *outLSB = 0x30 + lsb;
  *outMSB = 0x30 + msb;
}


unsigned char convertSingleDigitHex(unsigned char inValue)
{
  unsigned char retVal;

  switch (inValue)
  {
    case 0x0:
      retVal = 0x30 + 0;
      break;
    case 0x1:
      retVal = 0x30 + 1;
      break;
    case 0x2:
      retVal = 0x30 + 2;
      break;
    case 0x3:
      retVal = 0x30 + 3;
      break;
    case 0x4:
      retVal = 0x30 + 4;
      break;
    case 0x5:
      retVal = 0x30 + 5;
      break;
    case 0x6:
      retVal = 0x30 + 6;
      break;
    case 0x7:
      retVal = 0x30 + 7;
      break;
    case 0x8:
      retVal = 0x30 + 8;
      break;
    case 0x9:
      retVal = 0x30 + 9;
      break;

    case 0xA:
      retVal = 0x41 + 0;
      break;
    case 0xB:
      retVal = 0x41 + 1;
      break;
    case 0xC:
      retVal = 0x41 + 2;
      break;
    case 0xD:
      retVal = 0x41 + 3;
      break;
    case 0xE:
      retVal = 0x41 + 4;
      break;
    case 0xF:
      retVal = 0x41 + 5;
      break;

    default:
      retVal = 0x78; /* erro: 'x' */

  }
  return retVal;

} /* convertSingleDigitHex() */