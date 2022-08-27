////////////////////////////////////////////////////////////////////////////////
//                                                             
//                            `/o++///////:-`                         
//                         .+yNMNNNNMMMMMMMMNmdhhhhhyo/`              
//                    /sydNMMMMMMMMMMMMMMMMMMMMMMMMMMMMd-             
//                  `mMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMs:+:`        
//                 -yMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMm`       
//               -hNMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMo       
//             `+mMMMMMMMMMMMMNMMMMMMmddh:/hhMMMMMMMMMMMMMMMMMN`      
//             sMMMMMMMMMMMMMMMMNhso-````  ``./oosdmMNMNMMMMMMs       
//             hMMMMMMMMMMMMMMMMo                 ``.....:yMh-        
//             hMMMMMMMMMMMMMMMN.                         `om-        
//             hMMMMMMMMMMMMMMMMy`                          yy        
//             yMMMMMMMMMMMMMMMMMo                          .m        
//             yMMMMMMMMMMMMMMMMM+                           s.       
//             hMMMMMMMMMMMMMMMNs`                           -h`      
//             oMMMMMMMMMMMMMMh-     `:sso++++/:-`        .:sdN+      
//             -MmhdNMMMMMMMMm`    -omMMMMMMMMMMNmh+` ``:hmNd/o.      
//              ohhs+osmMMMMNh``` `yNMMMMNmdmmNMMMMmo`/hyhmmms+..-:   
//              yMMNMd+dMMMMMs     `/yddNo+oNMMMMMNh. /NosNNm+`  -o   
//              oMM+NdmMdoNMMy       `..-..-hdhhy/+d` `h+dmo.:   `.   
//               oyoMMMM: sMMs ``                 ``     ..  ---:/    
//                -:+hdmy +Nmh::+:-:            `-.          ````     
//                 :.  `- .shsh/syys           :dm.```       `        
//                 :h`     :hNo+-::`` :```    :mNNhdddh-    :`        
//                  y- `// ./dsy+:```   `.``  ..:mMMMMMd.  ./         
//                  `.   .  -:-so-/.``        `.+NMMMMMMN+ +`         
//                   `      .`./hs:/--`     `sdmMMMMMMMMMMy:          
//                   `        ---+/s/:.    `yMMmdmMMhhNNddy`          
//                   `        ..`../s. `` `hNMs` -mNNm+-  -           
//                   .          ````+o.-``.+Nmo:  `/MM+   `           
//                   .         `..` ` -./oohmyo:::/+MMN: :`           
//                  `         `+os++/-/hMMMMMMMMMMMMMMNyNy           
//                             .yyNNmmNNNMMMMMMMMMMMMMMMMM:           
//                              ``-:omMMMMMMMMMMMMMMMMMMMy            
//                                    .-/+ydNMMNNMm+--:+:`            
//
////////////////////////////////////////////////////////////////////////////////
//
// Module Name:   Array
// File Name:     Array.ino
// Version:       000.002.000
// Created On:    July 31, 2022
// Created By:    Bud Wandinger
// Modified:      August 10, 2022
// Modified By:   Bud Wandinger
//
// Description:
//    This module runs on an Arduino UNO and is responsible for detecting
//    phyisical presence within a specified range and sending OSC commands
//    via ethernet to light-control software.
//
//    The physical presence detection sensor used is the A01NYUB ultrasonic
//    proximity sensor by DFRobot. This module interfaces with the sensor via
//    software driven a-syncronis serial.
//
//    This module utilizes the KeyStudio W5100 Ethernet Shield to accomplish
//    ethernet networking communications with the computer running the
//    light-control software.
//
// Disclaimer:
//    THIS CODE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
//    INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
//    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PAGERDUTY OR
//    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//    PROFITS; OR BUSINESS INTERRUPTION) SUSTAINED BY YOU OR A THIRD PARTY,
//    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//    LIABILITY, OR TORT ARISING IN ANY WAY OUT OF THE USE OF THIS SAMPLE CODE,
//    EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
// PRIVATE DEFINITIONS
////////////////////////////////////////////////////////////////////////////////
#define DEBUG                       true    // Enable or disable serial debugging messages
#define DEBUG_DISTANCE              true    // Enable or disable the proximity measurement via serial messages
#define DEBUG_BAUD                  115200  // The baud rate of the debug serial
#define BYTE_LEN                    8       // The number of bits in a byte
#define EnTxPin                     6       // IO Pin used to enable(HIGH) or disable (LOW) the MAX485 driver outputs
//#define EnRxPin                     5
#define ETH_RESET_PIN               7       // The GPIO connected to hardware reset of the ethernet shield
#define ETH_RESET_DELAY             500     // Milliseconds - The delay for the hardware reset pin on the ethernet shield
#define PRESENCE_DISTANCE           180     // Centimeters - The distance which within presence should be detected
#define PRESENCE_HYSTERESIS         5       // Centimeters - The hysteresis for the presence detection calculation 
#define MAC_LEN                     6       // The number of bytes in a MAC address
#define SENSOR_TX                   8       // The physical TX pin on the Arduino connected to the proximity sensor
#define SENSOR_RX                   9       // The physical RX pin on the Arduino connected to the proximity sensor
#define SENSOR_BAUD                 19200   // The baud rate of the sensor
#define SENSOR_READ_PERIOD          150     // Milliseconds - The proximity measurement period of the proximity sensor
#define SENSOR_RESP_LEN             8       // The number if bytes in the sensor's distance response frame
#define SENSOR_DIST_CMD_LEN         6       // The number of bytes in the sensor's distance command frame
#define SENSOR_DIST_CMD_HDR_H       0x55    // The most significant byte of the sensor distance command header frame
#define SENSOR_DIST_CMD_HDR_L       0xAA    // The least significant byte of the sensor distance command header frame
#define SENSOR_DEVICE_ADDR          0x11    // The sensor's modbus address
#define SENSOR_DIST_CMD_DATA_LEN    0x00    // The sensor's distance command data length - There is no data reqiured for this command
#define SENSOR_DIST_CMD_OP          0x02    // The sensor's distance command opcode
#define SENSOR_DIST_CMD_CHECKSUM    (uint8_t) \
    (SENSOR_DIST_CMD_HDR_H + \
     SENSOR_DIST_CMD_HDR_L + \
     SENSOR_DEVICE_ADDR + \
     SENSOR_DIST_CMD_DATA_LEN + \
     SENSOR_DIST_CMD_OP)
#define DISTANCE_FILTER_SIZE        8       // The number of smaples to average out the distance measurement from the sensor
#define DISTANCE_FILTER_SIZE_TWO    3       // 2 to the power of this number equals the DISTANCE_FILTER_SIZE - used for efficient binary division


////////////////////////////////////////////////////////////////////////////////
// PRIVATE CONSTANTS
////////////////////////////////////////////////////////////////////////////////
enum SENSOR_PACKET_BYTE {
    HEADER,
    DATA_H,
    DATA_L,
    CHK_SUM
};

enum PRESENCE_STATES {
    NO_PRESENCE,
    PRESENCE,
}; 

const unsigned int IN_PORT = 8000;
const unsigned int OUT_PORT = 8000;
byte HOST_MAC[MAC_LEN] = {0x98, 0x76, 0xB6, 0x11, 0xEF, 0x86};
IPAddress HOST_IP(192, 168, 1, 2);
IPAddress DEST_IP(192, 168, 1, 240);
EthernetUDP UDP;
char OSC_PRESENCE_STR[] = "/composition/columns/1/connect";
char OSC_NO_PRESENCE_STR[] = "/composition/columns/2/connect";
const uint8_t SENSOR_DISTANCE_CMD[SENSOR_DIST_CMD_LEN] = {
    SENSOR_DIST_CMD_HDR_H,
    SENSOR_DIST_CMD_HDR_L,
    SENSOR_DEVICE_ADDR,
    SENSOR_DIST_CMD_DATA_LEN,
    SENSOR_DIST_CMD_OP,
    SENSOR_DIST_CMD_CHECKSUM};


////////////////////////////////////////////////////////////////////////////////
// PRIVATE VARIABLES
////////////////////////////////////////////////////////////////////////////////
static SoftwareSerial sensor_serial(SENSOR_RX, SENSOR_TX);
static PRESENCE_STATES presence_state;
static uint16_t distance;
static uint16_t distance_filter[DISTANCE_FILTER_SIZE];
static uint8_t distance_filter_idx;
static uint8_t sensor_resp[SENSOR_RESP_LEN];


////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////////////////////
static void measure_distance(void);
static void run_presence_state(void);
static void send_osc(OSCMessage msg);


////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
	pinMode(EnTxPin, OUTPUT );  
	digitalWrite(EnTxPin, LOW);

  //pinMode(EnRxPin, OUTPUT);
  //digitalWrite(EnRxPin, LOW);
 
#ifdef DEBUG
    Serial.begin(DEBUG_BAUD);
#endif
    
    sensor_serial.begin(SENSOR_BAUD);

#if DEBUG
    Serial.println("\tCOMPLETE");
    Serial.println("Initializing ethernet");
    Serial.print("\t- Host MAC   : ");
    for (uint8_t i = 0; i < MAC_LEN; i++)
    {
        Serial.print(HOST_MAC[i], HEX);
        if (i < MAC_LEN - 1)
        {
            Serial.print("-");
        }
    }
    Serial.println();
    Serial.print("\t- Host IP    : ");
    Serial.println(HOST_IP);
    Serial.print("\t- Dest IP    : ");
    Serial.println(DEST_IP);
    Serial.print("\t- In Port    : ");
    Serial.println(IN_PORT);
    Serial.print("\t- Out Port   : ");
    Serial.println(OUT_PORT);
#endif

    // Ethernet shield requires a hard reset after the external power supply comes online
    pinMode(ETH_RESET_PIN, OUTPUT);
    digitalWrite(ETH_RESET_PIN, LOW);
    delay(ETH_RESET_DELAY);
    digitalWrite(ETH_RESET_PIN, HIGH);
    
    Ethernet.begin(HOST_MAC, HOST_IP);

#if DEBUG
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        Serial.println("ERROR: NO ETHERNET HARDWARE DETECTED");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5100)
    {
        Serial.println("\t- Hardware   : W5100");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5200)
    {
        Serial.println("\t- Hardware   : W5100");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5500)
    {
        Serial.println("\t- Hardware   : W5100");
    }
    else
    {
        Serial.print("ERROR: UNKOWN ETHERNET HARWARE DETECTED");
    }
#endif
    
    UDP.begin(IN_PORT);

#if DEBUG
    Serial.println("\tCOMPLETE");
    Serial.println("Setup complete. Starting main loop...");
#endif
}

void loop(void)
{
    measure_distance();
    run_presence_state();
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS 
////////////////////////////////////////////////////////////////////////////////
static void measure_distance(void)
{
    digitalWrite(EnTxPin, HIGH);       // Tx Enable (HIGH)
    delay(1);
    uint8_t i;
    uint32_t distance_filter_sum;
    for (i = 0; i < SENSOR_DIST_CMD_LEN; i++)
    {
		
        sensor_serial.write(SENSOR_DISTANCE_CMD[i]);
        
    }
    delay(1);
    digitalWrite(EnTxPin, LOW);           // Tx Enable (LOW)
    
    delay(SENSOR_READ_PERIOD);
    
    i = 0;
	
	  
    while (sensor_serial.available())
    {
        //sensor_resp[i++] = (sensor_serial.read()^0xFF);        //add bit mask here.
//        Serial.print("A");
//       Serial.println(sensor_serial.read());
        
        sensor_resp[i++] = sensor_serial.read();                 //OG code
        
 
    }

    distance_filter[distance_filter_idx++] = ((sensor_resp[5] << 8) | sensor_resp[6]);

    if (distance_filter_idx >= DISTANCE_FILTER_SIZE)
    {
        distance_filter_idx = 0;
    }

    distance_filter_sum = 0;

    for (i = 0; i < DISTANCE_FILTER_SIZE; i++)
    {
        distance_filter_sum += distance_filter[i];
    }

   distance = distance_filter_sum >> DISTANCE_FILTER_SIZE_TWO;

#if DEBUG_DISTANCE
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println("cm");
#endif

    return;
}

static void run_presence_state(void)
{
    switch (presence_state)
    {
        case NO_PRESENCE:
            if (distance < PRESENCE_DISTANCE - PRESENCE_HYSTERESIS)
            {
#if DEBUG
                Serial.println("PRESENCE");
#endif
                presence_state = PRESENCE;
                send_osc(OSC_NO_PRESENCE_STR);
            }
            break;
        case PRESENCE:
            if (distance > PRESENCE_DISTANCE + PRESENCE_HYSTERESIS)
            {
#if DEBUG
                Serial.println("NO PRESENCE");
#endif
                presence_state = NO_PRESENCE;
                send_osc(OSC_PRESENCE_STR);
            }
            break;
    }
}

static void send_osc(char* msg_str)
{
#if DEBUG
    Serial.print("Sending OSC: ");
    Serial.println(msg_str);
#endif
   
    OSCMessage msg(msg_str); 
    UDP.beginPacket(DEST_IP, OUT_PORT);
    msg.send(UDP);
    UDP.endPacket();
    msg.empty();

#if DEBUG
    Serial.println("COMPLETE");
#endif
}
