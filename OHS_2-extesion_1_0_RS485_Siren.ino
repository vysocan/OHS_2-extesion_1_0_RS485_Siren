/*
 * Open Home Security - Extension node with RS485, Power 12V
 * 8 Analog and 1 Digital zones
 * Realay for Siren/Horn, AC_OFF, BATTERY_OK signals.
 * 
 * Board v.1.00
 */

#include <LibRS485v2.h>
#include <avr/eeprom.h> // Global configuration for in chip EEPROM

// Enable 1 or diable 0 SoftwareSerial.
// SoftwareSerial can internaly break some interrupts, timings ...
#define DEBUG_ON 0
#if DEBUG_ON
#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 7); // RX, TX
#endif

// Node settings
#define MY_ADDRESS       5    // 0 is gateway, 15 is multicast
#define VERSION          100  // Version of EEPROM struct
#define PING_DELAY       1200000 // In milliseconds, 20 minutes
// Constants
#define REG_LEN          21   // Size of one conf. element
#define NODE_NAME_SIZE   16   // As defined in gateway
#define ALARM_ZONES      9    // Fixed number by hardware
#define RS485_REPEAT     3    // Repeat sending
// Pins
#define DE               3    // RS 485 DE pin
#define BOX_TAMPER       4    // 
#define AC_OFF           5    // 
#define BATTERY_OK       6    // 
#define RELAY            9    //
// Alarm definition constants
#define ALARM_PIR_LOW    560
#define ALARM_PIR        715
#define ALARM_PIR_HI     900
#define ALARM_OK_LOW     270
#define ALARM_OK         380
#define ALARM_OK_HI      490
#define ALARM_UNBALANCED 100
#define ALARM_TAMPER     0
// Some macro helpers
#define GET_ZONE_SENT(x)             ((x) & 0b1)
#define GET_ZONE_SENT_TAMPER(x)      ((x >> 1U) & 0b1)
#define SET_ZONE_SENT(x)             x |= 1
#define SET_ZONE_SENT_TAMPER(x)      x |= (1 << 1U)
#define CLEAR_ZONE_SENT(x)           x &= ~1
#define CLEAR_ZONE_SENT_TAMPER(x)    x &= ~(1 << 1U)
#define GET_CONF_ZONE_ENABLED(x)     ((x) & 0b1)
#define GET_CONF_ZONE_PIR_AS_TMP(x)  ((x >> 1U) & 0b1)
#define GET_CONF_ZONE_BALANCED(x)    ((x >> 2U) & 0b1)
#define GET_CONF_ZONE_TYPE(x)        ((x >> 7U) & 0b1)

// Hardware pin to software zone number map
static const uint8_t zonePins[] = {A5, A4, A3, A2, A1, A0, A7, A6};

// RS485 message buffers
RS485_msg in_msg, out_msg; 

// Global variables
int8_t   resp;
uint8_t  mode = 0;
uint8_t  pos;
uint8_t  toSend = 0;
uint16_t val;
unsigned long zoneMillis;
unsigned long aliveMillis;
uint16_t pirDelay = 60000; // 60 seconds to allow PIR to settle

// Configuration struct
struct config_t {
  uint16_t version;
  char     reg[REG_LEN * 10]; // Number of elements on this node
} conf;

// Zone runtime variables
typedef struct {
  char    lastEvent = 'U';
  uint8_t setting = 0;
} zone_t;
zone_t zone[ALARM_ZONES];
/* 
 * Registration 
 */
void sendConf(){
  int8_t result;
  uint16_t count = 0;

  // Wait some time to avoid contention
  delay(MY_ADDRESS * 1000);   
  out_msg.address = 0;
  out_msg.ack = FLAG_ACK;
  out_msg.ctrl = FLAG_DTA;
  out_msg.data_length = REG_LEN + 1; // Add 'R'

  #if DEBUG_ON    
    mySerial.print(F("Conf: "));    
  #endif    
  while (count < sizeof(conf.reg)) {
    out_msg.buffer[0] = 'R'; // Registration flag
    memcpy(&out_msg.buffer[1], &conf.reg[count], REG_LEN);
    result = RS485.sendMsgWithAck(&out_msg, RS485_REPEAT);
    #if DEBUG_ON    
      mySerial.print(count, DEC);
      mySerial.print(F("="));
      mySerial.print(result, DEC);
      mySerial.print(F(", "));
    #endif
    count += REG_LEN;
  }
  #if DEBUG_ON    
    mySerial.println();
  #endif
}
/*
 * Set defaults
 */
void setDefault(){
  conf.version = VERSION;   // Change VERSION to take effect
  conf.reg[0+(REG_LEN*0)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*0)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*0)]  = 12;        // Zone index
  conf.reg[3+(REG_LEN*0)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*0)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*0)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*1)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*1)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*1)]  = 13;        // Zone index
  conf.reg[3+(REG_LEN*1)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*1)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*1)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*2)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*2)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*2)]  = 14;        // Zone index
  conf.reg[3+(REG_LEN*2)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*2)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*2)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*3)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*3)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*3)]  = 15;        // Zone index
  conf.reg[3+(REG_LEN*3)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*3)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*3)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*4)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*4)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*4)]  = 16;        // Zone index
  conf.reg[3+(REG_LEN*4)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*4)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*4)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*5)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*5)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*5)]  = 17;        // Zone index
  conf.reg[3+(REG_LEN*5)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*5)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*5)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*6)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*6)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*6)]  = 18;        // Zone index
  conf.reg[3+(REG_LEN*6)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*6)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*6)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*7)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*7)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*7)]  = 19;        // Zone index
  conf.reg[3+(REG_LEN*7)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*7)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*7)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*8)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*8)]  = 'D';       // Digital
  conf.reg[2+(REG_LEN*8)]  = 20;        // Zone index
  conf.reg[3+(REG_LEN*8)]  = B00000010; // Digital, unbalanced, PIR as tamper
  conf.reg[4+(REG_LEN*8)]  = B00011111; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*8)], 0, NODE_NAME_SIZE);
  strcpy(&conf.reg[5+(REG_LEN*8)], "Box tamper"); // Set default name
  conf.reg[0+(REG_LEN*9)]  = 'H';       // Horn/Siren 
  conf.reg[1+(REG_LEN*9)]  = 'D';       // Digital 
  conf.reg[2+(REG_LEN*9)]  = 1;         // Local index, !Used in code bellow to match!
  conf.reg[3+(REG_LEN*9)]  = B00000000; // Default setting
  conf.reg[4+(REG_LEN*9)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*9)], 0, NODE_NAME_SIZE);
  strcpy(&conf.reg[5+(REG_LEN*9)], "Remote siren"); // Set default name
}
/*
 * Send ping command to gateway 
 */
void sendPing(void) {
  out_msg.address = 0;
  out_msg.ctrl = FLAG_CMD;
  out_msg.data_length = 2; // PING = 2
  RS485.sendMsgWithAck(&out_msg, RS485_REPEAT);
}
/*
 * Setup
 */
void setup() {
  pinMode(A0, INPUT); pinMode(A1, INPUT);
  pinMode(A2, INPUT); pinMode(A3, INPUT);
  pinMode(A4, INPUT); pinMode(A5, INPUT);
  pinMode(A6, INPUT); pinMode(A7, INPUT);
  pinMode(BOX_TAMPER, INPUT);
  pinMode(DE, OUTPUT);
  pinMode(AC_OFF, INPUT);
  pinMode(BATTERY_OK, INPUT);
  pinMode(RELAY, OUTPUT); digitalWrite(RELAY, LOW);
  // Free IO ports
  //pinMode(7, OUTPUT); digitalWrite(7, LOW); 
  //pinMode(8, OUTPUT); digitalWrite(8, LOW); 
    
  #if DEBUG_ON
    mySerial.begin(19200);
    mySerial.println(F("Start"));
  #endif

  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  if (conf.version != VERSION) setDefault();

  // Delay 10 seconds to send conf
  delay(10000);
  RS485.begin(19200, MY_ADDRESS);
  sendConf();

  zoneMillis  = millis();
  aliveMillis = millis();
}
/*
 * Main loop
 */
void loop() {
  // Check battery state
  // The signal is "Low" when the voltage of battery is under 11V
  if (digitalRead(BATTERY_OK) == LOW) {    
    return; // Go to loop(), and do nothing
  }

  // Check AC state
  // The signal turns to be "High" when the power supply turns OFF
  if (digitalRead(AC_OFF) == HIGH) {
    // Not sure what to do yet
  }
  
  // Look for incomming transmissions
  resp = RS485.readMsg(&in_msg);
  if (resp == 1) {
    // Commands from gateway
    if (in_msg.ctrl == FLAG_CMD) {
      switch (in_msg.data_length) {
        case 1: // Request for registration
          sendConf(); 
          // Clear sent zones, maybe GW was restarted
          for(uint8_t ii = 0; ii < ALARM_ZONES; ii++) {
            CLEAR_ZONE_SENT(zone[ii].setting);
            SET_ZONE_SENT_TAMPER(zone[ii].setting); // To send OK for balanced
          }
          break;
        default: break;
      }
    }
    // Configuration change 
    if (in_msg.ctrl == FLAG_DTA && in_msg.buffer[0] == 'R') {
      // Replace part of conf with new data.
      pos = 0; 
      #if DEBUG_ON
        mySerial.print(F("Data: "));
        for (uint8_t ii=0; ii < in_msg.data_length-1; ii++){ mySerial.print((char)in_msg.buffer[1+ii]); }
        mySerial.println();
      #endif
      while (((conf.reg[pos] != in_msg.buffer[1]) || (conf.reg[pos+1] != in_msg.buffer[2]) ||
              (conf.reg[pos+2] != in_msg.buffer[3])) && (pos < sizeof(conf.reg))) {
        pos += REG_LEN; // size of one conf. element
      }      
      if (pos < sizeof(conf.reg)) {
        //* for (uint8_t ii=0; ii < in_msg.data_length-1; ii++){ conf.reg[pos+ii]=in_msg.buffer[1+ii]; }
        memcpy(&conf.reg[pos], &in_msg.buffer[1], REG_LEN);
        // Save it to EEPROM
        conf.version = VERSION;
        eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration      
      }
    }
    // Siren/Horn relay
    if (in_msg.ctrl == FLAG_DTA && in_msg.buffer[0] == 'H') {
      // Number matches local conf.freg.
      if (in_msg.buffer[1] == conf.reg[2+(REG_LEN*9)]) digitalWrite(RELAY, in_msg.buffer[2]);
    }
  } // RS485 message

  // Zone
  if ((unsigned long)(millis() - zoneMillis) > pirDelay) {
    zoneMillis = millis(); 
    pirDelay = 250; // PIR settled
    // Clear toSend
    toSend = 0;
    // Cycle trough zones
    for(uint8_t zoneNum = 0; zoneNum < ALARM_ZONES; zoneNum++) {
      if (GET_CONF_ZONE_ENABLED(conf.reg[4+(REG_LEN*zoneNum)])) {
        // Digital 0, Analog 1
        if (GET_CONF_ZONE_TYPE(conf.reg[3+(REG_LEN*zoneNum)])) {
          // Do ADC
          val = analogRead(zonePins[zoneNum]);;
          // Force unbalanced for analog zones
          if (!GET_CONF_ZONE_BALANCED(conf.reg[3+(REG_LEN*zoneNum)])){
            if (val < ALARM_UNBALANCED) val = ALARM_OK;
            else                        val = ALARM_PIR;
          }
        } else {
          // We have just one digital zone
          if (digitalRead(BOX_TAMPER)) val = ALARM_PIR;
          else                         val = ALARM_OK;
        }

        // alarm as tamper && is PIR, then make it tamper
        if ((GET_CONF_ZONE_PIR_AS_TMP(conf.reg[3+(REG_LEN*zoneNum)])) && 
            (val >= ALARM_PIR_LOW && val <= ALARM_PIR_HI)) val = ALARM_TAMPER;

        // Decide zone state
        switch ((uint16_t)(val)) {
          case ALARM_OK_LOW ... ALARM_OK_HI:
            // Do not send OK for balanced zones, gateway swithes them to OK automatically 
            if ((zone[zoneNum].lastEvent == 'O') && !GET_ZONE_SENT(zone[zoneNum].setting) &&
                (!GET_CONF_ZONE_BALANCED(conf.reg[3+(REG_LEN*zoneNum)]) || GET_ZONE_SENT_TAMPER(zone[zoneNum].setting))) {
              SET_ZONE_SENT(zone[zoneNum].setting);
              CLEAR_ZONE_SENT_TAMPER(zone[zoneNum].setting);
              out_msg.buffer[toSend*2+1] = conf.reg[2+(REG_LEN*zoneNum)]; // Zone number as in cof.reg
              out_msg.buffer[toSend*2+2] = zone[zoneNum].lastEvent;
              toSend++;
            }
            if ((zone[zoneNum].lastEvent != 'O') && 
              (!GET_CONF_ZONE_BALANCED(conf.reg[3+(REG_LEN*zoneNum)]) || GET_ZONE_SENT_TAMPER(zone[zoneNum].setting))) {
              CLEAR_ZONE_SENT(zone[zoneNum].setting);
            }
            zone[zoneNum].lastEvent = 'O';
            break;
          case ALARM_PIR_LOW ... ALARM_PIR_HI:
            if ((zone[zoneNum].lastEvent == 'P') && !GET_ZONE_SENT(zone[zoneNum].setting)) {
              SET_ZONE_SENT(zone[zoneNum].setting);
              CLEAR_ZONE_SENT_TAMPER(zone[zoneNum].setting);
              out_msg.buffer[toSend*2+1] = conf.reg[2+(REG_LEN*zoneNum)]; // Zone number as in cof.reg
              out_msg.buffer[toSend*2+2] = zone[zoneNum].lastEvent;
              toSend++;
            }
            if (zone[zoneNum].lastEvent != 'P') CLEAR_ZONE_SENT(zone[zoneNum].setting);
            zone[zoneNum].lastEvent = 'P';
            break;
          default: // Line is cut or short or tamper, no difference to alarm event
            if ((zone[zoneNum].lastEvent == 'T') && !GET_ZONE_SENT(zone[zoneNum].setting)) {
              SET_ZONE_SENT(zone[zoneNum].setting);
              SET_ZONE_SENT_TAMPER(zone[zoneNum].setting);
              out_msg.buffer[toSend*2+1] = conf.reg[2+(REG_LEN*zoneNum)]; // Zone number as in cof.reg
              out_msg.buffer[toSend*2+2] = zone[zoneNum].lastEvent;
              toSend++;
            }
            if (zone[zoneNum].lastEvent != 'T') CLEAR_ZONE_SENT(zone[zoneNum].setting);
            zone[zoneNum].lastEvent = 'T';
            break;
        }
      } // zone enabled
    } // for each alarm zone

    // Send zones if any
    if (toSend > 0) {    
      out_msg.address = 0;
      out_msg.ctrl = FLAG_DTA;
      out_msg.data_length = 1 + (toSend*2);
      out_msg.buffer[0] = 'Z'; // Zone
      resp = RS485.sendMsgWithAck(&out_msg, RS485_REPEAT);
      #if DEBUG_ON
        mySerial.print(F("Sent: ("));
        mySerial.print(toSend);     
        mySerial.print(F(") "));
        mySerial.print(resp);     
        mySerial.print(F(". Data: "));
        for (uint8_t ii=0; ii < toSend; ii++){
          mySerial.print(out_msg.buffer[(ii*2)+1], DEC);
          mySerial.print(F("="));
          mySerial.write((char)out_msg.buffer[(ii*2)+2]);
          mySerial.print(F(", "));
        }      
        mySerial.println();
      #endif 
    }
  }

  // Send alive packet every PING_DELAY
  if ((unsigned long)(millis() - aliveMillis) > PING_DELAY){
    aliveMillis = millis();
    sendPing();
  }

} // End main loop
