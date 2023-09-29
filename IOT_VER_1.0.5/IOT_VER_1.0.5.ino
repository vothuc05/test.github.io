// ESP32dev Signal  Wired to LCD        Wired to VS1053      SDCARD   Wired to the rest                     LAN 8720
// -------- ------  --------------      -------------------  ------   ---------------                  -------------
// GPIO32           -                    pin 1 XDCS            -       -                               -
// GPIO13            -                   pin 2 XCS             -       -                               -
// GPIO14            -                   pin 4 DREQ            -       -                               -
// GPIO12            pin 3 D/C or A0     -                     -       -                               -
// GPIO22           -                   -                     CS      -                               -
// GPIO16   RXD2    -                   -                     -       TX of NEXTION (if in use)       -
// GPIO17   TXD2    -                   -                     -       RX of NEXTION (if in use)       -
// GPIO18   SCK     pin 5 CLK or SCK    pin 5 SCK             CLK     -                               -
// GPIO33   MISO    -                   pin 7 MISO            MISO    -                               -
// GPIO23   MOSI    pin 4 DIN or SDA    pin 6 MOSI            MOSI    -                               -
// GPIO15           pin 2 CS            -                     -       -                               -
// GPI03    RXD0    -                   -                     -       Reserved serial input           -
// GPIO1    TXD0    -                   -                     -       Reserved serial output          -
// GPIO34   -       -                   -                     -       Optional pull-up resistor       -
// GPIO35   -       -                   -                     -       Infrared receiver VS1838B       -voi
// GPIO39   -       -                   -                     -       Rotary encoder CLK              -
// GPIO32   -       -                   -                     -       Rotary encoder DT               -
// GPIO00   -       -                   -                     -       Rotary encoder SW               -
// GPIO17                                                                                             - PHY_POWER   : NC - Osc. Enable - 4k7 Pulldown
// GPIO22                                                                                             - EMAC_TXD1   : TX1
// GPIO19                                                                                             - EMAC_TXD0   : TX0
// GPIO21                                                                                             - EMAC_TX_EN  : TX_EN
// GPIO26                                                                                             - EMAC_RXD1   : RX1
// GPIO25                                                                                             - EMAC_RXD0   : RX0
// GPIO27                                                                                             - EMAC_RX_DV  : CRS
// GPIO17                                                                                             - EMAC_TX_CLK : nINT/REFCLK (50MHz) - 4k7 Pullup
// GPIO15                                                                                             - SMI_MDC     : MDC
// GPIO02                                                                                             - SMI_MDIO    : MDIO
// GND                  : GND
//3V3                  : VCC
// -------  ------  ---------------     -------------------  ------   ----------------
// GND      -       pin 8 GND           pin 8 GND                     Power supply GND
// VCC 5 V  -       pin 7 BL            -                             Power supply
// VCC 5 V  -       pin 6 VCC           pin 9 5V                      Power supply
// EN       -       pin 1 RST           pin 3 XRST                    -
//
//

#define __FIRMWARE_NAME__    "NM:VNA_1101E"
#define __FIRMWARE_VERSION__ "FW:IOT_1.0.5"
#define __HARDWARE_VERSION__ "HW:V2.3"
#define VERSION     "IOT_1.0.5"
// ESP32-Radio can be updated (OTA) to the latest version from a remote server.
// The download uses the following server and files:
#define UPDATEHOST  "ota.mysignage.vn"                           // Host for software updates
#define BINFILE     "/VNR_1101E/IOT/Karadio_update.bin"      // Binary file name for update software
#define TFTFILE     "/VNR_1101E/IOT/Karadio_update.tft"      // Binary file name for update NEXTION image
//#define TFTFILE     "/ESP32-Radio.tft"          // Binary file name for update NEXTION image
//
// Define (just one) type of display.  See documentation.
#define BLUETFT                        // Works also for RED TFT 128x160
//#define OLED                         // 64x128 I2C OLED
//#define DUMMYTFT                     // Dummy display
//#define LCD1602I2C                   // LCD 1602 display with I2C backpack
//#define ILI9341                      // ILI9341 240*320
//#define NEXTION                      // Nextion display. Uses UART 2 (pin 16 and 17)
//
#include <ETH.h> // quản lí kết nối ethernet
#include <nvs.h>// Lưu dữ liệu khi ngắt nguồn
#include <PubSubClient.h>// giao tiếp máy chủ mqtt
#include <WiFiMulti.h>//kết nối nhiều magnj wifi khác nhau
#include <ESPmDNS.h>// các thiết bị trên mạng cục bộ tự tìm thấy, liên kết với nhau qua tên máy chủ thay vì IP
#include <time.h>// quản lí và đồng bộ hóa thời gian trên vi điều khiển
#include <stdio.h>// nhập xuất dữ liệu
#include <string.h>
#include <FS.h>// đọc, ghi và xóa tệp ghi
#include <SD.h>// truy cập và làm việc với thẻ nhớ SD để lưu trữ và truy xuất dữ liệu
#include <SPI.h>
#include <ArduinoOTA.h>// caaph nhật ota từ xa qua wifi
#include <freertos/queue.h>// quản lí các tác vụ trong FreRtos
#include <freertos/task.h>
#include <esp_task_wdt.h>// sử dụng bộ đếm watchdog của tác vụ 
#include <esp_partition.h>// làm việc với phân vùng trên VĐK
#include <driver/adc.h>//
#include <Update.h>// cập nhật firmware từ thẻ nhớ hoặc ngoại vi
#include <base64.h>// mã hóa dữ liệu nhị phân để truyền tải qua email hoặc http
#include <WiFi.h> // for WiFi shield
#include <WiFiUdp.h>// UDP qua wifi
#include <NTPClient.h>// lấy time từ máy chủ NTP qua mạng,đồng bộ thời gian thực cho VĐK
//-----------------------------------------Config PIN Ngoai------------------------
//---------------------------------------------------------------------------------
#ifdef ETH_CLK_MODE// kiểm tra macro ETH_CLK_MODE có tồn tại 
#undef ETH_CLK_MODE// loại bỏ định ETH_CLK_MODE trc đó
#endif
#define ETH_CLK_MODE    ETH_CLOCK_GPIO16_OUT

// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN   -1

// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720

// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        0

// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     15

// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    2

#define RS_VS1053 4
#define MUTE 12
//GPIO LED OUTPUT
#define led_do 5
#define led_xanh 17
#define power_4g 14
#define mic_det 35
#define vol_down 34
#define vol_up 0
//----------------------------------------------------------------------------------
// Number of entries in the queue
#define QSIZ 200
// Debug buffer size
#define DEBUG_BUFFER_SIZE 200
#define NVSBUFSIZE 150
// Access point name if connection to WiFi network fails.  Also the hostname for WiFi and OTA.
// Note that the password of an AP must be at least as long as 8 characters.
// Also used for other naming.
#define NAME "Vietbroadcast"
// Max number of presets in preferences
#define MAXPRESETS 200
// Maximum number of MQTT reconnects before give-up
#define MAXMQTTCONNECTS 120
// Adjust size of buffer to the longest expected string for nvsgetstr
#define NVSBUFSIZE 150
// Position (column) of time in topline relative to end
#define TIMEPOS -52
// SPI speed for SD card
#define SDSPEED 1000000
// Size of metaline buffer
#define METASIZ 1024
// Max. number of NVS keys in table
#define MAXKEYS 200
// Time-out [sec] for blanking TFT display (BL pin)
#define BL_TIME 45

//-------------------------Config MQTTT------------------------------
#define MQTT_SUBTOPIC     "STD"           // Command to receive from MQTT
#define MQTT_PUBTOPIC     "DTS"           // Command to receive from MQTT

//--------------------------------------------------------------------
//                       Mang base 64                                 
static const char* encoding = "34865D785498MNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
//--------------------------------------------------------------------
//--------------------------------------------------------------------

 
//ip broker sever
const char* mqtt_ip_manufacture = "1101m.vietbroadcast.vn";
uint16_t    mqtt_port_manufacture = 1883;
String      mqtt_mqttuser_manufacture = "vietbroadcast" ;                           // User for MQTT authentication
String      mqtt_mqttpasswd_manufacture = "Viet@123456" ;                         // Password for MQTT authentication
char* topic_manager = "MANAGER/DTS";
char* topic_manu = "MANUFACTURE/DTS";
//---------------------------------------------------------------------

//-------------------------Config OTA----------------------------------
#define otaclient mp3client                   // OTA uses mp3client for connection to host

//-------------------------Config Time Zone----------------------------
WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP);
String formattedTime;
String timeupdate;
String dayStamp;
String timeStamp;

//------------------------Config led.tick()----------------------------
unsigned int time_led_xanh_on;
unsigned int time_led_xanh_off;
unsigned int time_led_do;
unsigned long time1 = 0;
unsigned long time2 = 0;

//**************************************************************************************************
// Forward declaration and prototypes of various functions.                                        *
//**************************************************************************************************
void        displaytime ( const char* str, uint16_t color = 0xFFFF ) ;
void        showstreamtitle ( const char* ml, bool full = false ) ;
void        handlebyte_ch ( uint8_t b ) ;
void        handleFSf ( const String& pagename ) ;
void        handleCmd()  ;
char*       dbgprint( const char* format, ... ) ;
const char* analyzeCmd ( const char* str ) ;
const char* analyzeCmd ( const char* par, const char* val ) ;
const char* analyzeCmd_manu_broker ( const char* str ) ;
const char* analyzeCmd_manu_broker ( const char* par, const char* val ) ;
void        chomp ( String &str ) ;
String      httpheader ( String contentstype ) ;
bool        nvssearch ( const char* key ) ;
void        mp3loop() ;
void        tftlog ( const char *str ) ;
void        playtask ( void * parameter ) ;       // Task to play the stream
void        spftask ( void * parameter ) ;        // Task for special functions
void        gettime() ;
void        reservepin ( int8_t rpinnr ) ;


//**************************************************************************************************
// Several structs.                                                                                *
//**************************************************************************************************
//

struct scrseg_struct                                  // For screen segments
{
  bool     update_req ;                               // Request update of screen
  uint16_t color ;                                    // Textcolor
  uint16_t y ;                                        // Begin of segment row
  uint16_t height ;                                   // Height of segment
  String   str ;                                      // String to be displayed
} ;

enum qdata_type { QDATA, QSTARTSONG, QSTOPSONG } ;    // datatyp in qdata_struct
struct qdata_struct
{
  int datatyp ;                                       // Identifier
  __attribute__((aligned(4))) uint8_t buf[32] ;       // Buffer for chunk
} ;

struct ini_struct
{ 

  String         mqttbroker ;                         // The name of the MQTT broker server
  String         mqttprefix ;                         // Prefix to use for topics
  String         mqttowner;
  uint16_t       mqttport ;                           // Port, default 1883
  String         mqttuser ;                           // User for MQTT authentication
  String         mqttpasswd ;                         // Password for MQTT authentication
  unsigned int        reqvol ;                             // Requested volume
  unsigned int        micvol;                              // Mic Volume
  unsigned int        streamvol;
  uint16_t       frq;                                 //
  uint8_t        rtone[4] ;                           // Requested bass/treble settings
  int16_t        newpreset ;                          // Requested preset
  String         clk_server ;                         // Server to be used for time of day clock
  int8_t         clk_offset ;                         // Offset in hours with respect to UTC
  int8_t         clk_dst ;                            // Number of hours shift during DST
  int8_t         ir_pin ;                             // GPIO connected to output of IR decoder
  int8_t         enc_clk_pin ;                        // GPIO connected to CLK of rotary encoder
  int8_t         enc_dt_pin ;                         // GPIO connected to DT of rotary encoder
  int8_t         enc_sw_pin ;                         // GPIO connected to SW of rotary encoder
  int8_t         tft_cs_pin ;                         // GPIO connected to CS of TFT screen
  int8_t         tft_dc_pin ;                         // GPIO connected to D/C or A0 of TFT screen
  int8_t         tft_scl_pin ;                        // GPIO connected to SCL of i2c TFT screen
  int8_t         tft_sda_pin ;                        // GPIO connected to SDA of I2C TFT screen
  int8_t         tft_bl_pin ;                         // GPIO to activate BL of display
  int8_t         tft_blx_pin ;                        // GPIO to activate BL of display (inversed logic)
  int8_t         sd_cs_pin ;                          // GPIO connected to CS of SD card
  int8_t         vs_cs_pin ;                          // GPIO connected to CS of VS1053
  int8_t         vs_dcs_pin ;                         // GPIO connected to DCS of VS1053
  int8_t         vs_dreq_pin ;                        // GPIO connected to DREQ of VS1053
  int8_t         vs_shutdown_pin ;                    // GPIO to shut down the amplifier
  int8_t         vs_shutdownx_pin ;                   // GPIO to shut down the amplifier (inversed logic)
  int8_t         spi_sck_pin ;                        // GPIO connected to SPI SCK pin
  int8_t         spi_miso_pin ;                       // GPIO connected to SPI MISO pin
  int8_t         spi_mosi_pin ;                       // GPIO connected to SPI MOSI pin
  uint16_t       bat0 ;                               // ADC value for 0 percent battery charge
  uint16_t       bat100 ;                             // ADC value for 100 percent battery charge
  String         hengio;
} ;

struct WifiInfo_t                                     // For list with WiFi info
{
  uint8_t inx ;                                       // Index as in "wifi_00"
  char * ssid ;                                       // SSID for an entry
  char * passphrase ;                                 // Passphrase for an entry
} ;

struct nvs_entry
{
  uint8_t  Ns ;                                       // Namespace ID
  uint8_t  Type ;                                     // Type of value
  uint8_t  Span ;                                     // Number of entries used for this item
  uint8_t  Rvs ;                                      // Reserved, should be 0xFF
  uint32_t CRC ;                                      // CRC
  char     Key[16] ;                                  // Key in Ascii
  uint64_t Data ;                                     // Data in entry
} ;

struct nvs_page                                       // For nvs entries
{ // 1 page is 4096 bytes
  uint32_t  State ;
  uint32_t  Seqnr ;
  uint32_t  Unused[5] ;
  uint32_t  CRC ;
  uint8_t   Bitmap[32] ;
  nvs_entry Entry[126] ;
} ;

struct keyname_t                                      // For keys in NVS
{
  char      Key[16] ;                                 // Max length is 15 plus delimeter
} ;

//**************************************************************************************************
// Global data section.                                                                            *
//**************************************************************************************************
// There is a block ini-data that contains some configuration.  Configuration data is              *
// saved in the preferences by the webinterface.  On restart the new data will                     *
// de read from these preferences.                                                                 *
// Items in ini_block can be changed by commands from webserver/MQTT/Serial.                       *
//**************************************************************************************************

enum display_t { T_UNDEFINED, T_BLUETFT, T_OLED,         // Various types of display
                 T_DUMMYTFT, T_LCD1602I2C, T_ILI9341,
                 T_NEXTION
               } ;

enum datamode_t { INIT = 1, HEADER = 2, DATA = 4,        // State for datastream
                  METADATA = 8, PLAYLISTINIT = 16,
                  PLAYLISTHEADER = 32, PLAYLISTDATA = 64,
                  STOPREQD = 128, STOPPED = 256
                } ;

//----------------------------Variable Status-------------------------------
uint8_t           sts_net;
uint8_t           oldsts=0;
uint16_t          try_connect;
uint32_t          pmp=5000;
int               cv_timeupdate;
int               count_update;
uint16_t          bau = 115200;                          // toc do baud
uint8_t           stt_ply=1;
int               state_vs = 0;                          // trang thai VS 1053
char              key_pub[30] ;
char              key_mac[6];
String            key_str;
String            mac_id;
String            key_remote;
char              test_key[6];
char              rt_recive[20];
String            id = "1a2b3c";
uint8_t           mode_led;
uint8_t           accept_try = 0;
String            dsp_info;
bool              flag_mac=true;
bool              flag_ip=false;
bool              flag_status=false;
uint8_t           flag_dsp=0;
String            dsp_mac;
String            dsp_ip;
String            dsp_signal;
static uint32_t   tmpVolUpCnt = 0;
static uint32_t   tmpVolDownCnt = 0;
static uint32_t   tmpMicOnCnt = 0;
static uint32_t   tmpMicOffCnt = 0;
static uint8_t    LastMicOnStatus = 0;
static uint8_t    MicOnStatus = 0xFF;
bool              sts_4g =  false;
bool              sts_AP = false;
bool              sts_stop_done = true;
bool              cam_mic = false;
uint32_t          time_button=0x3FF;
int               LEDState = HIGH;

//----------------------------Parameter Manufacture-------------------------------------

int8_t            ban;                                 // Ban thiet bi
uint32_t          dur=1;                                 // thoi gian thiet bi chay
int8_t            ota;                                   // cai dat cap nhat
String            otf;
String            fip;
String            fnm;
String            fpw;
String            sip;
uint32_t          otc;                                   // check size;
String            mip = "mqtt.mysignage.vn";
String            ctp;
uint8_t           dfc;
uint8_t           crq;
//-----------------------------Parameter Manager------------------------------------------
String            ert;                                 // Thoi gian loi
String            imp;
uint8_t           err;
uint16_t          sam;                                   // toc do lay mau encoder
int               realtime;                              //real time
String            dlk;                                 //Link Ic Cast
String            wap;
String            pr0;
String            pr1;
String            pr2;
String            pr3;
String            pr4;
String            pr5;
String            pr6;
String            pr7;
String            pr8;
String            pr9;

String            rcv;
String            snd;
uint8_t           cod;                                   // trang thai thu
uint8_t           linein =1;
//----------------------------Parameter Both server---------------------------------------
String            dnm;                                 // The name of device
String            did;                                 // id device
String            mac_pub = "";                        // dia chi mac
String            ver;                                 // version
String            gps;                                 // GPS
String            tmz;                                 // timezone
String            reg;                                 // Registation date
String            epd;                                 // Expire date
uint8_t           mod=0;                               // chon che do play
uint8_t           sts;                                 // trang thai hien tai
uint16_t          zip=000000;
char              version_firmware[60];
char              ota_info[60];
//----------------------------------------------------------------------------------------

// Global variablesu
uint8_t           reload_accept=0;
uint32_t          qspace ;                               // Free space in data queue
int               DEBUG = 1 ;                            // Debug on/off
int               numSsid ;                              // Number of available WiFi networks
WiFiMulti         wifiMulti ;                            // Possible WiFi networks
ini_struct        ini_block ;                            // Holds configurable data
WiFiServer        cmdserver ( 80 ) ;                     // Instance of embedded webserver, port 80
WiFiClient        mp3client ;                            // An instance of the mp3 client, also used for OTA
WiFiClient        cmdclient ;                            // An instance of the client for commands
WiFiClient        wmqttclient ;                          // An instance for mqtt
PubSubClient      mqttclient ( wmqttclient ) ;           // Client for MQTT subscriber
WiFiClient        espClient;
PubSubClient      mqttclient1 (espClient);
HardwareSerial*   nxtserial = NULL ;                     // Serial port for NEXTION (if defined)
TaskHandle_t      maintask ;                             // Taskhandle for main task
TaskHandle_t      xplaytask ;                            // Task handle for playtask
TaskHandle_t      xspftask ;                             // Task handle for special functions
SemaphoreHandle_t SPIsem = NULL ;                        // For exclusive SPI usage
hw_timer_t*       timer = NULL ;                         // For timer
char              timetxt[9] ;                           // Converted timeinfo
char              datetxt[3];
char              cmd[130] ;                             // Command from MQTT or Serial
uint8_t           tmpbuff[6000] ;                        // Input buffer for mp3 or data stream
QueueHandle_t     dataqueue ;                            // Queue for mp3 datastream
QueueHandle_t     spfqueue ;                             // Queue for special functions
qdata_struct      outchunk ;                             // Data to queue
qdata_struct      inchunk ;                              // Data from queue
uint8_t*          outqp = outchunk.buf ;                 // Pointer to buffer in outchunk
uint32_t          totalcount = 0 ;                       // Counter mp3 data
datamode_t        datamode ;                             // State of datastream
int               metacount ;                            // Number of bytes in metadata
int               datacount ;                            // Counter databytes before metadata
char              metalinebf[METASIZ + 1] ;              // Buffer for metaline/ID3 tags
int16_t           metalinebfx ;                          // Index for metalinebf
String            icystreamtitle ;                       // Streamtitle from metadata
String            icyname ;                              // Icecast station name
String            ipaddress ;                            // Own IP-address
String            ETHspeed;
int               bitrate ;                              // Bitrate in kb/sec
int               mbitrate ;                             // Measured bitrate
int               metaint = 0 ;                          // Number of databytes between metadata
int16_t           currentpreset = -1 ;                   // Preset station playing
String            host ;                                 // The URL to connect to or file to play
String            playlist ;                             // The URL of the specified playlist
bool              hostreq = false ;                      // Request for new hostoa
bool              reqtone = false ;                      // New tone setting requested
bool              muteflag = false ;                     // Mute output
bool              resetreq = false ;                     // Request to reset the ESP32
bool              updatereq = false ;                    // Request to update software from remote host
bool              NetworkFound = false ;                 // True if WiFi network connected
bool              mqtt_on = false ;                      // MQTT in use
bool              mqtt_on_manu = false;
String            networks ;                             // Found networks in the surrounding
uint16_t          mqttcount = 0 ;                        // Counter
uint16_t          mqttcount_manu_broker = 0 ;            // Counter
int8_t            playingstat = 0 ;                      // 1 if radio is playing (for MQTT)
int16_t           playlist_num = 0 ;                     // Nonzero for selection from playlist
File              mp3file ;                              // File containing mp3 on SD card
uint32_t          mp3filelength ;                        // File length
bool              localfile = false ;                    // Play from local mp3-file or not
bool              chunked = false ;                      // Station provides chunked transfer
int               chunkcount = 0 ;                       // Counter for chunked transfer
String            http_getcmd ;                          // Contents of last GET command
String            http_rqfile ;                          // Requested file
bool              http_response_flag = false ;           // Response required
uint16_t          ir_value = 0 ;                         // IR code
uint32_t          ir_0 = 550 ;                           // Average duration of an IR short pulse
uint32_t          ir_1 = 1650 ;                          // Average duration of an IR long pulse
struct tm         timeinfo ;                             // Will be filled by NTP server
bool              time_req = false ;                     // Set time requested
bool              SD_okay = false ;                      // True if SD card in place and readable
String            SD_nodelist ;                          // Nodes of mp3-files on SD
int               SD_nodecount = 0 ;                     // Number of nodes in SD_nodelist
String            SD_currentnode = "" ;                  // Node ID of song playing ("0" if random)
uint16_t          adcval ;                               // ADC value (battery voltage)
uint32_t          clength ;                              // Content length found in http header
uint32_t          max_mp3loop_time = 0 ;                 // To check max handling time in mp3loop (msec)
int16_t           scanios ;                              // TEST*TEST*TEST
int16_t           scaniocount ;                          // TEST*TEST*TEST
uint16_t          bltimer = 0 ;                          // Backlight time-out counter
display_t         displaytype = T_UNDEFINED ;            // Display type
std::vector<WifiInfo_t> wifilist ;                       // List with wifi_xx info
// nvs stuff
nvs_page                nvsbuf ;                         // Space for 1 page of NVS info
const esp_partition_t*  nvs ;                            // Pointer to partition struct
esp_err_t               nvserr ;                         // Error code from nvs functions
uint32_t                nvshandle = 0 ;                  // Handle for nvs access
uint8_t                 namespace_ID ;                   // Namespace ID found
char                    nvskeys[MAXKEYS][16] ;           // Space for NVS keys
std::vector<keyname_t> keynames ;                        // Keynames in NVS
// Rotary encoder stuff
#define sv DRAM_ATTR static volatile
sv uint16_t       clickcount = 0 ;                       // Incremented per encoder click
sv int16_t        rotationcount = 0 ;                    // Current position of rotary switch
sv uint16_t       enc_inactivity = 0 ;                   // Time inactive
sv bool           singleclick = false ;                  // True if single click detected
sv bool           doubleclick = false ;                  // True if double click detected
sv bool           tripleclick = false ;                  // True if triple click detected
sv bool           longclick = false ;                    // True if longclick detected
enum enc_menu_t { VOLUME, PRESET, TRACK } ;              // State for rotary encoder menu
enc_menu_t        enc_menu_mode = VOLUME ;               // Default is VOLUME mode

//
struct progpin_struct                                    // For programmable input pins
{
  int8_t         gpio ;                                  // Pin number
  bool           reserved ;                              // Reserved for connected devices
  bool           avail ;                                 // Pin is available for a command
  String         command ;                               // Command to execute when activated
  // Example: "uppreset=1"
  bool           cur ;                                   // Current state, true = HIGH, false = LOW
} ;

progpin_struct   progpin[] =                             // Input pins and programmed function
{
  {  0, false, true,  "uvol=2", false },
  //int pin_number: Đây là số của pin đầu vào (hoặc đầu ra) tương ứng.
  //bool enable_internal_pullup: Biến boolean xác định xem có sử dụng pull-up (kéo lên nội bộ) trên pin đó hay không.
  //bool is_input: Biến boolean xác định xem pin có phải là đầu vào (input) hay không.
  //String function: Một chuỗi (String) mô tả chức năng được lập trình cho pin đó.
  //bool reserved: Biến boolean xác định xem pin này đã được dành riêng cho một mục đích khác hay không.
  //{  1, true,  false,  "", false },                    // Reserved for TX Serial output
  {  2, false, false,  "", false },
  //{  3, true,  false,  "", false },                    // Reserved for RX Serial input
  {  4, false, false,  "", false },
  //{  5, true, false,  "", false },
  //{  6, true,  false,  "", false },                    // Reserved for FLASH SCK
  //{  7, true,  false,  "", false },                    // Reserved for FLASH D0
  //{  8, true,  false,  "", false },                    // Reserved for FLASH D1
  //{  9, true,  false,  "", false },                    // Reserved for FLASH D2
  //{ 10, true,  false,  "", false },                    // Reserved for FLASH D3
  //{ 11, true,  false,  "", false },                    // Reserved for FLASH CMD
  //{ 12, false, false,  "", false },
  { 13, false, false,  "", false },
 // { 14, false, false,  "", false },
  { 15, false, false,  "", false },
  { 16, false, false,  "", false },                      // May be UART 2 RX for Nextion
  { 17, false, false,  "", false },                      // May be UART 2 TX for Nextion
  { 18, false, false,  "", false },                      // Default for SPI CLK
  { 19, false, false,  "", false },                      // Default for SPI MISO
  //{ 20, true,  false,  "", false },                    // Not exposed on DEV board
  { 21, false, false,  "", false },                      // Also Wire SDA
  { 22, false, false,  "", false },                      // Also Wire SCL
  { 23, false, false,  "", false },                      // Default for SPI MOSI
  //{ 24, true,  false,  "", false },                    // Not exposed on DEV board
  { 25, false, false,  "", false },
  { 26, false, false,  "", false },
  { 27, false, false,  "", false },
  //{ 28, true,  false,  "", false },                    // Not exposed on DEV board
  //{ 29, true,  false,  "", false },                    // Not exposed on DEV board
  //{ 30, true,  false,  "", false },                    // Not exposed on DEV board
  //{ 31, true,  false,  "", false },                    // Not exposed on DEV board
  { 32, false, false,  "", false },
  { 33, false, false,  "", false },
  { 34, false, true,  "dvol=2", false },                      // Note, no internal pull-up
  { 35, false, false,  "", false },                      // Note, no internal pull-up
  { 36, false, false,  "", false },                    // Reserved for ADC battery level
  { 39, false, false,  "", false },                     // Note, no internal pull-up
  { -1, false, false,  "", false }                       // End of list
} ;

struct touchpin_struct                                   // For programmable input pins
{
  int8_t         gpio ;                                  // Pin number GPIO
  bool           reserved ;                              // Reserved for connected devices
  bool           avail ;                                 // Pin is available for a command
  String         command ;                               // Command to execute when activated
  // Example: "uppreset=1"
  bool           cur ;                                   // Current state, true = HIGH, false = LOW
  int16_t        count ;                                 // Counter number of times low level
} ;
touchpin_struct   touchpin[] =                           // Touch pins and programmed function
{
  {   4, false, false, "", false, 0 },                   // TOUCH0
  {   0, true,  false, "", false, 0 },                   // TOUCH1, reserved for BOOT button
  {   2, false, false, "", false, 0 },                   // TOUCH2
  {  15, false, false, "", false, 0 },                   // TOUCH3
  {  13, false, false, "", false, 0 },                   // TOUCH4
  {  12, false, false, "", false, 0 },                   // TOUCH5
  {  14, false, false, "", false, 0 },                   // TOUCH6
  {  27, false, false, "", false, 0 },                   // TOUCH7
  {  33, false, false, "", false, 0 },                   // TOUCH8
  {  32, false, false, "", false, 0 },                   // TOUCH9
  {  -1, false, false, "", false, 0 }                    // End of list
  // End of table
} ;


//**************************************************************************************************
// Pages, CSS and data for the webinterface.                                                       *
//**************************************************************************************************
#include "about_html.h"
#include "config_html.h"// tệp html cho trang config 
#include "index_html.h"
#include "mp3play_html.h"
#include "radio_css.h"
#include "favicon_ico.h"//chứa biểu tượng của trang web khi người dùng truy cập 
#include "defaultprefs.h"// tệp chưa các gtri mặc định/ cài đặt ban đầu 

//**************************************************************************************************
// End of global data section.                                                                     *
//**************************************************************************************************




//**************************************************************************************************
//                                     M Q T T P U B _ C L A S S                                   *
//**************************************************************************************************
// ID's for the items to publish to MQTT.  Is index in amqttpub[]
enum { MQTT_DNM, MQTT_DID, MQTT_VER, MQTT_MAC, MQTT_GPS, MQTT_TMZ , MQTT_REG,
       MQTT_EPD, MQTT_BAN, MQTT_DUR, MQTT_OTA, MQTT_OTF, MQTT_OTC, MQTT_CIP, MQTT_CPR ,
       MQTT_CNM, MQTT_CPW, MQTT_CTP, MQTT_VOLUME, MQTT_MOD, MQTT_PRESET, MQTT_STS,
       MQTT_ERR, MQTT_ERT, MQTT_BAU, MQTT_SAM, MQTT_DLK ,MQTT_IMP, MQTT_ICY , MQTT_PLY,
       MQTT_FRQ, MQTT_COD, MQTT_IP, MQTT_STREAMTITLE, MQTT_PLAYLISTPOS, MQTT_RCV, MQTT_PLK,MQTT_PMP,
       MQTT_ZIP, MQTT_CRQ, MQTT_PR0, MQTT_PR1, MQTT_PR2, MQTT_PR3, MQTT_PR4, MQTT_PR5, MQTT_PR6, MQTT_PR7, MQTT_PR8,MQTT_PR9,
     } ;
enum { MQSTRING, MQINT8, MQINT16, MQINT32 } ;                     // Type of variable to publish

class mqttpubc                                           // For MQTT publishing
{
    struct mqttpub_struct
    {
      const char*    topic ;                             // Topic as partial string (without prefix)
      uint8_t        type ;                              // Type of payload
      void*          payload ;                           // Payload for this topic
      bool           topictrigger ;                      // Set to true to trigger MQTT publish
    } ;
    // Publication topics for MQTT.  The topic will be pefixed by "PREFIX/", where PREFIX is replaced
    // by the the mqttprefix in the preferences.
  protected:
    mqttpub_struct amqttpub[53] =                   // Definitions of various MQTT topic to publish1
    { // Index is equal to enum above
      { "DNM",             MQSTRING, &dnm,              false }, // Defimotion for MQTT_DNM
      { "DID",             MQSTRING, &did,              false }, // Defimotion for MQTT_DID
      { "VER",             MQSTRING, &ver,              false }, // Defimotion for MQTT_VER
      { "MAC",             MQSTRING, &mac_pub,          false }, // Defimotion for MQTT_MAC
      { "GPS",             MQSTRING, &gps,              false }, // Defimotion for MQTT_VER
      { "TMZ",             MQSTRING, &ini_block.clk_offset,  false }, // Defimotion for MQTT_TMZ
      { "REG",             MQSTRING, &reg,              false }, // Defimotion for MQTT_REG
      { "EXP",             MQSTRING, &epd,              false }, // Defimotion for MQTT_EPD
      { "BAN",             MQINT8,   &ban,              false }, // Defimotion for MQTT_BAN
      { "DUR",             MQINT32,  &dur,              false }, // Defimotion for MQTT_DUR
      { "OTA",             MQINT8,   &ota,              false }, // Defimotion for MQTT_OTA
      { "OTF",             MQSTRING, &otf,              false }, // Defimotion for MQTT_EPD
      { "OTC",             MQINT32,  &otc,              false }, // Defimotion for MQTT_OTA
      { "CIP",             MQSTRING, &ini_block.mqttbroker,       false }, // Defimotion for MQTT_CIP
      { "CPR",             MQINT16,  &ini_block.mqttport,         false }, // Defimotion for MQTT_CPR
      { "CNM",             MQSTRING, &ini_block.mqttuser,         false }, // Defimotion for MQTT_CNM
      { "CPW",             MQSTRING, &ini_block.mqttpasswd,       false }, // Defimotion for MQTT_CPW
      { "CTP",             MQSTRING, &ctp,               false }, // Defimotion for MQTT_CTP
      { "VOL",             MQINT8,   &ini_block.reqvol, false }, // Definition for MQTT_VOLUME
      { "MOD",             MQINT8,   &mod,              false }, // Definition for MQTT_MOD
      { "PRE",             MQINT8,   &currentpreset,    false }, // Definition for MQTT_PRE
      { "STS",             MQINT8,   &sts,              false }, // Definition for MQTT_STS
      { "ERR",             MQINT8,   &err,              false }, // Definition for MQTT_ERR
      { "ERT",             MQSTRING, &ert,              false }, // Definition for MQTT_ERT
      { "BAU",             MQINT16,  &bau,              false }, // Definition for MQTT_BAU
      { "SAM",             MQINT16,  &sam,              false }, // Definition for MQTT_SAM
      { "DLK",             MQSTRING, &dlk,              false }, // Definition for MQTT_DLK
      { "IMP",             MQSTRING, &imp,              false }, // Definition for MQTT_DLK
      { "ICY",             MQSTRING, &icyname,          false }, // Definition for MQTT_ICY
      { "PLY",             MQINT8,   &playingstat,      false }, // Definition for MQTT_PLY
      { "FRQ",             MQINT16,  &ini_block.frq,    false }, // Definition for MQTT_FRQ
      { "COD",             MQINT8,   &cod,              false }, // Definition for MQTT_COD
      { "IP",              MQSTRING, &ipaddress,        false }, // Definition for MQTT_IP
      { "ICY",             MQSTRING, &icystreamtitle,   false }, // Definition for MQTT_STREAMTITLE
      { "POS",             MQINT16,  &playlist_num,     false }, // Definition for MQTT_PLAYLISTPOS
      { "RCV",             MQSTRING, &rcv,              false }, // Definition for MQTT_ST1
      { "PLK",             MQSTRING, &key_str,          false }, // Definition for MQTT_ST1
      { "PMP",             MQINT32,  &pmp,              false }, // Definition for MQTT_PMP
      { "ZIP",             MQINT16,  &zip,              false }, // Definition for MQTT_ZIP
      { "CRQ",             MQINT8,   &crq,              false }, // Definition for MQTT_CRQ
      { "PR0",             MQSTRING, &pr0,              false }, // Definition for MQTT_PR0
      { "PR1",             MQSTRING, &pr1,              false }, // Definition for MQTT_CRQ
      { "PR2",             MQSTRING, &pr2,              false }, // Definition for MQTT_CRQ
      { "PR3",             MQSTRING, &pr3,              false }, // Definition for MQTT_CRQ
      { "PR4",             MQSTRING, &pr4,              false }, // Definition for MQTT_CRQ
      { "PR5",             MQSTRING, &pr5,              false }, // Definition for MQTT_CRQ
      { "PR6",             MQSTRING, &pr6,              false }, // Definition for MQTT_CRQ
      { "PR7",             MQSTRING, &pr7,              false }, // Definition for MQTT_CRQ
      { "PR8",             MQSTRING, &pr8,              false }, // Definition for MQTT_CRQ
      { "PR9",             MQSTRING, &pr9,              false }, // Definition for MQTT_CRQ
      { NULL,              0,        NULL,              false }  // End of definitions
    } ;
  public:
    void          trigger ( uint8_t item ) ;                      // Trigger publishig for one item
    void          publishtopic() ;                                // Publish triggerer items
} ;


//**************************************************************************************************
// MQTTPUB  class implementation.                                                                  *
//**************************************************************************************************

//**************************************************************************************************
//                                            T R I G G E R                                        *
//**************************************************************************************************
// Set request for an item to publish to MQTT.                                                     *
//**************************************************************************************************
void mqttpubc::trigger ( uint8_t item )                    // Trigger publishig for one item
{
  amqttpub[item].topictrigger = true ;                     // Request re-publish for an item
}

//**************************************************************************************************
//                                     P U B L I S H T O P I C                                     *
//**************************************************************************************************
// Publish a topic to MQTT broker.                                                                 *
//**************************************************************************************************
void mqttpubc::publishtopic()
{
  int         i = 0 ;                                         // Loop control
  char        topic[80] ;                                     // Topic to send
  char        topic_manu[80] ;                                     // Topic to send
  const char* payload ;                                       // Points to payload
  const char* convert;
  char        intvar[10] ;                                    // Space for integer parameter
  char        strvar[60];
  while ( amqttpub[i].topic )
  {
    if ( amqttpub[i].topictrigger )                           // Topic ready to send?
    {
      amqttpub[i].topictrigger = false ;                      // Success or not: clear trigger
       sprintf ( topic, "%s/%s",ini_block.mqttprefix.c_str(),MQTT_PUBTOPIC) ; //tạo chuỗi pubtopic với tiền tố được định nghĩa                        // Add prefix to topic
       sprintf ( topic_manu, "MANUFACTURE/DTS") ;
     switch ( amqttpub[i].type )                             // Select conversion method
      {
        case MQSTRING :
          convert = ((String*)amqttpub[i].payload)->c_str() ;// chuyển đổi con troe payload từ kiểu string --> con trỏ const char*
          sprintf ( strvar, "%s=%s", amqttpub[i].topic,
                    convert);
          // payload = intvar1->c_str() ;                           // Get pointer to payload
          payload = strvar;
          break ;
        case MQINT8 :
          sprintf ( intvar, "%s=%d", amqttpub[i].topic,
                    *(int8_t*)amqttpub[i].payload ) ;         // Convert to array of char
          payload = intvar ;                                  // Point to this array
          break ;
        case MQINT16 :
          sprintf ( intvar, "%s=%d",amqttpub[i].topic,
                    *(int16_t*)amqttpub[i].payload ) ; 
          payload = intvar ;
          break ;
        case MQINT32 :
          sprintf ( intvar, "%s=%d", amqttpub[i].topic,
                    *(int32_t*)amqttpub[i].payload ) ; 
          payload = intvar ;
          break ;
        default :
          continue ;                                          // Unknown data type
      }
      dbgprint ( "Publish to topic %s : %s",                  // Show for debug
                 topic, payload ) ;
      if ( !mqttclient.publish ( topic, payload ))           // Publish!
      {
        dbgprint ( "MQTT publish failed!" ) ;                 // Failed
      }
      return ;                                                // Do the rest later
    }
    i++ ;                                                     // Next entry
  }
}

mqttpubc         mqttpub ;                                    // Instance for mqttpubc

//
//**************************************************************************************************
// VS1053 stuff.  Based on maniacbug library.                                                      *
//**************************************************************************************************
// VS1053 class definition.                                                                        *
//**************************************************************************************************
class VS1053
{
  private:
    int8_t        cs_pin ;                         // Pin where CS line is connected
    int8_t        dcs_pin ;                        // Pin where DCS line is connected
    int8_t        dreq_pin ;                       // Pin where DREQ line is connected
    int8_t        shutdown_pin ;                   // Pin where the shutdown line is connected
    int8_t        shutdownx_pin ;                  // Pin where the shutdown (inversed) line is connected
    uint8_t       curvol ;                         // Current volume setting 0..100%
    const uint8_t vs1053_chunk_size = 32 ;
    // SCI Register
    const uint8_t SCI_MODE          = 0x0 ;
    const uint8_t SCI_STATUS        = 0x1 ;
    const uint8_t SCI_BASS          = 0x2 ;
    const uint8_t SCI_CLOCKF        = 0x3 ;
    const uint8_t SCI_AUDATA        = 0x5 ;
    const uint8_t SCI_WRAM          = 0x6 ;
    const uint8_t SCI_WRAMADDR      = 0x7 ;
    const uint8_t SCI_AIADDR        = 0xA ;
    const uint8_t SCI_VOL           = 0xB ;
    const uint8_t SCI_AICTRL0       = 0xC ;
    const uint8_t SCI_AICTRL1       = 0xD ;
    const uint8_t SCI_num_registers = 0xF ;
    // SCI_MODE bits
    const uint8_t SM_SDINEW         = 11 ;        // Bitnumber in SCI_MODE always on
    const uint8_t SM_RESET          = 2 ;         // Bitnumber in SCI_MODE soft reset
    const uint8_t SM_CANCEL         = 3 ;         // Bitnumber in SCI_MODE cancel song
    const uint8_t SM_TESTS          = 5 ;         // Bitnumber in SCI_MODE for tests
    const uint8_t SM_ADPCM          = 12 ;        // Bitnumber in SCI_MODE for MIC
    const uint8_t SM_LINE1          = 14 ;        // Bitnumber in SCI_MODE for Line input
    SPISettings   VS1053_SPI ;                    // SPI settings for this slave
    uint8_t       endFillByte ;                   // Byte to send when stopping song
    bool          okay              = true ;      // VS1053 is working
  protected:
    inline void await_data_request() const
    {
      while ( ( dreq_pin >= 0 ) &&
              ( !digitalRead ( dreq_pin ) ) )
      {
        NOP() ;                                   // Very short delay
      }
    }

    inline void control_mode_on() const
    {
      SPI.beginTransaction ( VS1053_SPI ) ;       // Prevent other SPI users
      digitalWrite ( cs_pin, LOW ) ;
    }

    inline void control_mode_off() const
    {
      digitalWrite ( cs_pin, HIGH ) ;             // End control mode
      SPI.endTransaction() ;                      // Allow other SPI users
    }

    inline void data_mode_on() const
    {
      SPI.beginTransaction ( VS1053_SPI ) ;       // Prevent other SPI users
      //digitalWrite ( cs_pin, HIGH ) ;           // Bring slave in data mode
      digitalWrite ( dcs_pin, LOW ) ;
    }

    inline void data_mode_off() const
    {
      digitalWrite ( dcs_pin, HIGH ) ;            // End data mode
      SPI.endTransaction() ;                      // Allow other SPI users
    }

    uint16_t    read_register ( uint8_t _reg ) const ;
    void        write_register ( uint8_t _reg, uint16_t _value ) const ;
    inline bool sdi_send_buffer ( uint8_t* data, size_t len ) ;
    void        sdi_send_fillers ( size_t length ) ;
    void        wram_write ( uint16_t address, uint16_t data ) ;
    uint16_t    wram_read ( uint16_t address ) ;
    void        output_enable ( bool ena ) ;             // Enable amplifier through shutdown pin(s)

  public:
    // Constructor.  Only sets pin values.  Doesn't touch the chip.  Be sure to call begin()!
    VS1053 ( int8_t _cs_pin, int8_t _dcs_pin, int8_t _dreq_pin,
             int8_t _shutdown_pin, int8_t _shutdownx_pin ) ;
    void     begin() ;                                   // Begin operation.  Sets pins correctly,
    // and prepares SPI bus.
    void     startSong() ;                               // Prepare to start playing. Call this each
    // time a new song starts.
    inline bool playChunk ( uint8_t* data,               // Play a chunk of data.  Copies the data to
                            size_t len ) ;               // the chip.  Blocks until complete.
    // Returns true if more data can be added
    // to fifo
    void     stopSong() ;                                // Finish playing a song. Call this after
    // the last playChunk call.
    void     setVolume ( uint8_t vol ) ;                 // Set the player volume.Level from 0-100,
    // higher is louder.
    void     setTone ( uint8_t* rtone ) ;                // Set the player baas/treble, 4 nibbles for
    // treble gain/freq and bass gain/freq
    inline uint8_t  getVolume() const                    // Get the current volume setting.
    { // higher is louder.
      return curvol ;
    }
    void     printDetails ( const char *header ) ;       // Print config details to serial output
    void     softReset() ;                               // Do a soft reset
    bool     testComm ( const char *header ) ;           // Test communication with module
    inline bool data_request() const
    {
      return ( digitalRead ( dreq_pin ) == HIGH ) ;
    }
    void     AdjustRate ( long ppm2 ) ;                  // Fine tune the datarate
    void     SwitchToMic();
    void     SwitchToStream();
} ;

//**************************************************************************************************
// VS1053 class implementation.                                                                    *
//**************************************************************************************************

VS1053::VS1053 ( int8_t _cs_pin, int8_t _dcs_pin, int8_t _dreq_pin,
                 int8_t _shutdown_pin, int8_t _shutdownx_pin ) :
  cs_pin(_cs_pin), dcs_pin(_dcs_pin), dreq_pin(_dreq_pin), shutdown_pin(_shutdown_pin),
  shutdownx_pin(_shutdownx_pin)
{
}

uint16_t VS1053::read_register ( uint8_t _reg ) const
{
  uint16_t result ;

  control_mode_on() ;
  SPI.write ( 3 ) ;                                // Read operation
  SPI.write ( _reg ) ;                             // Register to write (0..0xF)
  // Note: transfer16 does not seem to work
  result = ( SPI.transfer ( 0xFF ) << 8 ) |        // Read 16 bits data
           ( SPI.transfer ( 0xFF ) ) ;
  await_data_request() ;                           // Wait for DREQ to be HIGH again
  control_mode_off() ;
  return result ;
}

void VS1053::write_register ( uint8_t _reg, uint16_t _value ) const
{
  control_mode_on( );
  SPI.write ( 2 ) ;                                // Write operation
  SPI.write ( _reg ) ;                             // Register to write (0..0xF)
  SPI.write16 ( _value ) ;                         // Send 16 bits data
  await_data_request() ;
  control_mode_off() ;
}

bool VS1053::sdi_send_buffer ( uint8_t* data, size_t len )
{
  size_t chunk_length ;                            // Length of chunk 32 byte or shorter

  data_mode_on() ;
  while ( len )                                    // More to do?
  {
    chunk_length = len ;
    if ( len > vs1053_chunk_size )
    {
      chunk_length = vs1053_chunk_size ;
    }
    len -= chunk_length ;
    await_data_request() ;                         // Wait for space available
    SPI.writeBytes ( data, chunk_length ) ;
    data += chunk_length ;
  }
  data_mode_off() ;
  return data_request() ;                          // True if more data can de stored in fifo
}

void VS1053::sdi_send_fillers ( size_t len )
{
  size_t chunk_length ;                            // Length of chunk 32 byte or shorter

  data_mode_on() ;
  while ( len )                                    // More to do?
  {
    await_data_request() ;                         // Wait for space available
    chunk_length = len ;
    if ( len > vs1053_chunk_size )
    {
      chunk_length = vs1053_chunk_size ;
    }
    len -= chunk_length ;
    while ( chunk_length-- )
    {
      SPI.write ( endFillByte ) ;
    }
  }
  data_mode_off();
}

void VS1053::wram_write ( uint16_t address, uint16_t data )
{
  write_register ( SCI_WRAMADDR, address ) ;
  write_register ( SCI_WRAM, data ) ;
}

uint16_t VS1053::wram_read ( uint16_t address )
{
  write_register ( SCI_WRAMADDR, address ) ;            // Start reading from WRAM
  return read_register ( SCI_WRAM ) ;                   // Read back result
}

bool VS1053::testComm ( const char *header )
{
  // Test the communication with the VS1053 module.  The result wille be returned.
  // If DREQ is low, there is problably no VS1053 connected.  Pull the line HIGH
  // in order to prevent an endless loop waiting for this signal.  The rest of the
  // software will still work, but readbacks from VS1053 will fail.
  int            i ;                                    // Loop control
  uint16_t       r1, r2, cnt = 0 ;
  uint16_t       delta = 300 ;                          // 3 for fast SPI
  const uint16_t vstype[] = { 1001, 1011, 1002, 1003,   // Possible chip versions
                              1053, 1033, 1063, 1103
                            } ;

  dbgprint ( header ) ;                                 // Show a header
  if ( !digitalRead ( dreq_pin ) )
  {
 //   Serial.println(dreq_pin);
    dbgprint ( "VS1053 not properly installed!" ) ;
    // Allow testing without the VS1053 module
    pinMode ( dreq_pin,  INPUT_PULLUP ) ;               // DREQ is now input with pull-up
    return false ;                                      // Return bad result
  }
  // Further TESTING.  Check if SCI bus can write and read without errors.
  // We will use the volume setting for this.
  // Will give warnings on serial output if DEBUG is active.
  // A maximum of 20 errors will be reported.
  if ( strstr ( header, "Fast" ) )
  {
    delta = 3 ;                                         // Fast SPI, more loops
  }
  for ( i = 0 ; ( i < 0xFFFF ) && ( cnt < 20 ) ; i += delta )
  {
    write_register ( SCI_VOL, i ) ;                     // Write data to SCI_VOL
    r1 = read_register ( SCI_VOL ) ;                    // Read back for the first time
    r2 = read_register ( SCI_VOL ) ;                    // Read back a second time
    if  ( r1 != r2 || i != r1 || i != r2 )              // Check for 2 equal reads
    {
      dbgprint ( "VS1053 SPI error. SB:%04X R1:%04X R2:%04X", i, r1, r2 ) ;
      cnt++ ;
      delay ( 10 ) ;
      okay = false;
    }
  }
  okay = ( cnt == 0 ) ;                                 // True if working correctly
  // Further testing: is it the right chip?
  r1 = ( read_register ( SCI_STATUS ) >> 4 ) & 0x7 ;    // Read status to get the version
  dbgprint ("%d",r1);
  if ( r1 !=  4 && r1 !=  6)                                       // Version 4 is a genuine VS1053
  {
    dbgprint ( "This is not a VS1053, "                 // Report the wrong chip
               "but a VS%d instead!",
               vstype[r1] ) ;
    okay = false ;
  }
  return ( okay ) ;                                     // Return the result
}

void VS1053::begin()
{
  pinMode      ( RS_VS1053, OUTPUT ) ;
  digitalWrite (RS_VS1053,  LOW);
  delay(10);
  digitalWrite ( RS_VS1053, HIGH ) ;
  pinMode      ( dreq_pin,  INPUT ) ;                   // DREQ is an input
  pinMode      ( cs_pin,    OUTPUT ) ;                  // The SCI and SDI signals
  pinMode      ( dcs_pin,   OUTPUT ) ;
  digitalWrite ( dcs_pin,   HIGH ) ;                    // Start HIGH for SCI en SDI
  digitalWrite ( cs_pin,    HIGH ) ;

  // digitalWrite (RS_VS1053,  LOW);

  // delay(10);
  //  digitalWrite (RS_VS1053,  LOW);
 
  if ( shutdown_pin >= 0 )                              // Shutdown in use?
  {
    pinMode ( shutdown_pin,   OUTPUT ) ;
  }
  if ( shutdownx_pin >= 0 )                            // Shutdown (inversed logic) in use?
  {
    pinMode ( shutdownx_pin,   OUTPUT ) ;// shutdownx 
  }
  output_enable ( false ) ;                            // Disable amplifier through shutdown pin(s)
  delay ( 100 ) ;
  // Init SPI in slow mode ( 0.2 MHz )
  VS1053_SPI = SPISettings ( 200000, MSBFIRST, SPI_MODE0 ) ;
  SPI.setDataMode ( SPI_MODE0 ) ;
  SPI.setBitOrder ( MSBFIRST ) ;
  //printDetails ( "Right after reset/startup" ) ;
  delay ( 20 ) ;
  //printDetails ( "20 msec after reset" ) ;
  if ( testComm ( "Slow SPI, Testing VS1053 read/write registers..." ) )
  {
    // Most VS1053 modules will start up in midi mode.  The result is that there is no audio
    // when playing MP3.  You can modify the board, but there is a more elegant way: 
    wram_write ( 0xC017, 3 ) ;                            // GPIO DDR = 3
    wram_write ( 0xC019, 0 ) ;                            // GPIO ODATA = 0
    delay ( 100 ) ;
    //printDetails ( "After test loop" ) ;
    softReset() ;                                         // Do a soft reset
    // Switch on the analog parts
    write_register ( SCI_AUDATA, 44100 + 1 ) ;            // 44.1kHz + stereo
    // The next clocksetting allows SPI clocking at 5 MHz, 4 MHz is safe then.
    write_register ( SCI_CLOCKF, 6 << 12 ) ;              // Normal clock settings
    // multiplyer 3.0 = 12.2 MHz
    //SPI Clock to 4 MHz. Now you can set high speed SPI clock.
    VS1053_SPI = SPISettings ( 5000000, MSBFIRST, SPI_MODE0 ) ;
    write_register ( SCI_MODE, _BV ( SM_SDINEW ) | _BV ( SM_LINE1 ) ) ;
//    write_register ( SCI_MODE, _BV ( SM_ADPCM ) | _BV ( SM_SDINEW )) ;   
    testComm ( "Fast SPI, Testing VS1053 read/write registers again..." ) ;
    delay ( 10 ) ;
    await_data_request() ;
    endFillByte = wram_read ( 0x1E06 ) & 0xFF ;
    dbgprint ( "endFillByte is %X", endFillByte ) ;
    state_vs = 1;
    //printDetails ( "After last clocksetting" ) ;
    delay ( 100 ) ;
  }
}

void VS1053::setVolume ( uint8_t vol )
{
  // Set volume.  Both left and right.
  // Input value is 0..100.  100 is the loudest.
  // Clicking reduced by using 0xf8 to 0x00 as limits.
  uint16_t value ;                                      // Value to send to SCI_VOL

  if ( vol != curvol )
  {
    curvol = vol ;                                      // Save for later use
    value = map ( vol, 0, 100, 0xF8, 0x00 ) ;           // 0..100% to one channel
    value = ( value << 8 ) | value ;
    write_register ( SCI_VOL, value ) ;                 // Volume left and right
    output_enable ( vol != 0 ) ;                        // Enable/disable amplifier through shutdown pin(s)
  }
}

void VS1053::setTone ( uint8_t *rtone )                 // Set bass/treble (4 nibbles)
{
  // Set tone characteristics.  See documentation for the 4 nibbles.
  uint16_t value = 0 ;                                  // Value to send to SCI_BASS
  int      i ;                                          // Loop control

  for ( i = 0 ; i < 4 ; i++ ) // 
  {  
    value = ( value << 4 ) | rtone[i] ;                 // Shift next nibble in
  }
  write_register ( SCI_BASS, value ) ;                  // Volume left and right
}

void VS1053::startSong()
{
  sdi_send_fillers ( 10 ) ;
  output_enable ( true ) ;                              // Enable amplifier through shutdown pin(s)
}

bool VS1053::playChunk ( uint8_t* data, size_t len )
{
  return okay && sdi_send_buffer ( data, len ) ;        // True if more data can be added to fifo
}

void VS1053::stopSong()
{
  uint16_t modereg ;                                    // Read from mode register
  int      i ;                                          // Loop control
  
  sdi_send_fillers ( 2052 ) ;
  output_enable ( false ) ;                             // Disable amplifier through shutdown pin(s)
  delay ( 10 ) ;
  write_register ( SCI_MODE, _BV ( SM_SDINEW ) | _BV ( SM_CANCEL ) ) ;
//  sts_stop_done =true;
  for ( i = 0 ; i < 20 ; i++ )
  {
    sdi_send_fillers ( 32 ) ;
    modereg = read_register ( SCI_MODE ) ;              // Read mode status
    if ( ( modereg & _BV ( SM_CANCEL ) ) == 0 )         // SM_CANCEL will be cleared when finished
    {
      sdi_send_fillers ( 2052 ) ;
      dbgprint ( "Song stopped correctly after %d msec", i * 10 ) ;
      return ;
    }
    delay ( 10 ) ;

  }
  printDetails ( "Song stopped incorrectly!" ) ;
}

void VS1053::softReset()
{
  write_register ( SCI_MODE, _BV ( SM_SDINEW ) | _BV ( SM_RESET ) ) ;
  delay ( 10 ) ;
  await_data_request() ;
}

void VS1053::printDetails ( const char *header )
{
  uint16_t     regbuf[16] ;
  uint8_t      i ;

  dbgprint ( header ) ;
  dbgprint ( "REG   Contents" ) ;
  dbgprint ( "---   -----" ) ;
  for ( i = 0 ; i <= SCI_num_registers ; i++ )
  {
    regbuf[i] = read_register ( i ) ;
  }
  for ( i = 0 ; i <= SCI_num_registers ; i++ )
  {
    delay ( 5 ) ;
    dbgprint ( "%3X - %5X", i, regbuf[i] ) ;
  }
}

void  VS1053::output_enable ( bool ena )               // Enable amplifier through shutdown pin(s)
{
  if ( shutdown_pin >= 0 )                             // Shutdown in use?
  {
    digitalWrite ( shutdown_pin, !ena ) ;              // Shut down or enable audio output
  }
  if ( shutdownx_pin >= 0 )                            // Shutdown (inversed logic) in use?
  {
    digitalWrite ( shutdownx_pin, ena ) ;              // Shut down or enable audio output
  }
}


void VS1053::AdjustRate ( long ppm2 )                  // Fine tune the data rate
{
  write_register ( SCI_WRAMADDR, 0x1e07 ) ;
  write_register ( SCI_WRAM,     ppm2 ) ;
  write_register ( SCI_WRAM,     ppm2 >> 16 ) ;
  // oldClock4KHz = 0 forces  adjustment calculation when rate checked.
  write_register ( SCI_WRAMADDR, 0x5b1c ) ;
  write_register ( SCI_WRAM,     0 ) ;
  // Write to AUDATA or CLOCKF checks rate and recalculates adjustment.
  write_register ( SCI_AUDATA,   read_register ( SCI_AUDATA ) ) ;
}
void VS1053::SwitchToMic(void){
      dbgprint("Vao den day la stream to mic");
      write_register(SCI_AICTRL1, 4096); //1024     
      write_register(SCI_MODE, 0x1804); 
//      write_register ( SCI_MODE, _BV ( SM_SDINEW ) | _BV ( SM_LINE1 )| _BV ( SM_RESET )) ;
      delay ( 10 ) ;
      await_data_request() ;
        /* Switch to Mic Line in: 0x5804 Mic - in: 0x1804*/
}
void VS1053::SwitchToStream(void){
            /*Set volumme -> Mic volumme*/
    write_register ( SCI_MODE, _BV ( SM_SDINEW ) | _BV ( SM_LINE1 )| _BV ( SM_RESET )) ;
    delay ( 10 ) ;
    await_data_request() ;
    sts_stop_done == false;
    dbgprint("Vao den day la mic to stream");
    
        /* Switch to Mic Line in: 0x5804 Mic - in: 0x1804*/
}


// The object for the MP3 player
VS1053* vs1053player ;
void resetvs1053()
{
  if (state_vs == 0) {
    datamode = STOPREQD ;
    vs1053player->begin() ;
    hostreq = true ;
  }
}
//**************************************************************************************************
// End VS1053 stuff.                                                                               *
//**************************************************************************************************

// Include software for the right display
#ifdef BLUETFT
#include "bluetft.h"                                     // For ILI9163C or ST7735S 128x160 display
#endif
#ifdef ILI9341
#include "ILI9341.h"                                     // For ILI9341 320x240 display
#endif
#ifdef OLED
#include "SSD1306.h"                                     // For OLED I2C SD1306 64x128 display
#endif
#ifdef LCD1602I2C
#include "LCD1602.h"                                     // For LCD 1602 display (I2C)
#endif
#ifdef DUMMYTFT
#include "Dummytft.h"                                    // For Dummy display
#endif
#ifdef NEXTION
#include "NEXTION.h"                                     // For NEXTION display
#endif


//**************************************************************************************************
//                                           B L S E T                                             *
//**************************************************************************************************
// Enable or disable the TFT backlight if configured.                                              *
// May be called from interrupt level.                                                             *
//**************************************************************************************************
void IRAM_ATTR blset ( bool enable )
{
  if ( ini_block.tft_bl_pin >= 0 )                       // Backlight for TFT control?
  {
    digitalWrite ( ini_block.tft_bl_pin, enable ) ;      // Enable/disable backlight
  }
  if ( ini_block.tft_blx_pin >= 0 )                      // Backlight for TFT (inversed logic) control?
  {
    digitalWrite ( ini_block.tft_blx_pin, !enable ) ;    // Enable/disable backlight
  }
  if ( enable )
  {
    bltimer = 0 ;                                        // Reset counter backlight time-out
  }
}



//**************************************************************************************************
//                                      N V S O P E N                                              *
//**************************************************************************************************
// Open Preferences with my-app namespace. Each application module, library, etc.                  *
// has to use namespace name to prevent key name collisions. We will open storage in               *
// RW-mode (second parameter has to be false).                                                     *
//**************************************************************************************************
void nvsopen()
{
  if ( ! nvshandle )                                         // Opened already?
  {
    nvserr = nvs_open ( NAME, NVS_READWRITE, &nvshandle ) ;  // No, open nvs
    if ( nvserr )
    {
      dbgprint ( "nvs_open failed!" ) ;
    }
  }
}


//**************************************************************************************************
//                                      N V S C L E A R                                            *
//**************************************************************************************************
// Clear all preferences.                                                                          *
//**************************************************************************************************
esp_err_t nvsclear()
{
  nvsopen() ;                                         // Be sure to open nvs
  return nvs_erase_all ( nvshandle ) ;                // Clear all keys
}

esp_err_t nvserase(char* key_erase)
{
  nvsopen();
  return nvs_erase_key (nvshandle,key_erase );
}


//**************************************************************************************************
//                                      N V S G E T S T R                                          *
//**************************************************************************************************
// Read a string from nvs.                                                                         *
//**************************************************************************************************
String nvsgetstr ( const char* key )
{
  static char   nvs_buf[NVSBUFSIZE] ;       // Buffer for contents
  size_t        len = NVSBUFSIZE ;          // Max length of the string, later real length

  nvsopen() ;                               // Be sure to open nvs
  nvs_buf[0] = '\0' ;                       // Return empty string on error
  nvserr = nvs_get_str ( nvshandle, key, nvs_buf, &len ) ;
  if ( nvserr )
  {
    dbgprint ( "nvs_get_str failed %X for key %s, keylen is %d, len is %d!",
               nvserr, key, strlen ( key), len ) ;
    dbgprint ( "Contents: %s", nvs_buf ) ;
  }
  return String ( nvs_buf ) ;
}


//**************************************************************************************************
//                                      N V S S E T S T R                                          *
//**************************************************************************************************
// Put a key/value pair in nvs.  Length is limited to allow easy read-back.                        *
// No writing if no change.                                                                        *
//**************************************************************************************************
esp_err_t nvssetstr ( const char* key, String val )
{
  String curcont ;                                         // Current contents
  bool   wflag = true  ;                                   // Assume update or new key

  //dbgprint ( "Setstring for %s: %s", key, val.c_str() ) ;
  if ( val.length() >= NVSBUFSIZE )                        // Limit length of string to store
  {
    dbgprint ( "nvssetstr length failed!" ) ;
    return ESP_ERR_NVS_NOT_ENOUGH_SPACE ;
  }
  if ( nvssearch ( key ) )                                 // Already in nvs?
  {
    curcont = nvsgetstr ( key ) ;                          // Read current value
    wflag = ( curcont != val ) ;                           // Value change?
  }
  if ( wflag )                                             // Update or new?
  {
    //dbgprint ( "nvssetstr update value" ) ;
    nvserr = nvs_set_str ( nvshandle, key, val.c_str() ) ; // Store key and value
    if ( nvserr )                                          // Check error
    {
      dbgprint ( "nvssetstr failed!" ) ;
    }
  }
  return nvserr ;
}


//**************************************************************************************************
//                                      N V S C H K E Y                                            *
//**************************************************************************************************
// Change a keyname in in nvs.                                                                     *
//**************************************************************************************************
void nvschkey ( const char* oldk, const char* newk )
{
  String curcont ;                                         // Current contents

  if ( nvssearch ( oldk ) )                                // Old key in nvs?
  {
    curcont = nvsgetstr ( oldk ) ;                         // Read current value
    nvs_erase_key ( nvshandle, oldk ) ;                    // Remove key
    nvssetstr ( newk, curcont ) ;                          // Insert new
  }
}


//**************************************************************************************************
//                                      C L A I M S P I                                            *
//**************************************************************************************************
// Claim the SPI bus.  Uses FreeRTOS semaphores.                                                   *
// If the semaphore cannot be claimed within the time-out period, the function continues without   *
// claiming the semaphore.  This is incorrect but allows debugging.                                *
//**************************************************************************************************
void claimSPI ( const char* p )
{
  const              TickType_t ctry = 10 ;                 // Time to wait for semaphore
  uint32_t           count = 0 ;                            // Wait time in ticks
  static const char* old_id = "none" ;                      // ID that holds the bus

  while ( xSemaphoreTake ( SPIsem, ctry ) != pdTRUE  )      // Claim SPI bus
  {
    if ( count++ > 25 )
    {
      dbgprint ( "SPI semaphore not taken within %d ticks by CPU %d, id %s",
                 count * ctry,
                 xPortGetCoreID(),
                 p ) ;
      dbgprint ( "Semaphore is claimed by %s", old_id ) ;
    }
    if ( count >= 100 )
    {
      return ;                                               // Continue without semaphore
    }
  }
  old_id = p ;                                               // Remember ID holding the semaphore
}


//**************************************************************************************************
//                                   R E L E A S E S P I                                           *
//**************************************************************************************************
// Free the the SPI bus.  Uses FreeRTOS semaphores.                                                *
//**************************************************************************************************
void releaseSPI()
{
  xSemaphoreGive ( SPIsem ) ;                            // Release SPI bus
}


//**************************************************************************************************
//                                      Q U E U E F U N C                                          *
//**************************************************************************************************
// Queue a special function for the play task.                                                     *
//**************************************************************************************************
void queuefunc ( int func )
{
  qdata_struct     specchunk ;                          // Special function to queue

  specchunk.datatyp = func ;                            // Put function in datatyp
  xQueueSend ( dataqueue, &specchunk, 200 ) ;           // Send to queue
}


//**************************************************************************************************
//                                      N V S S E A R C H                                          *
//**************************************************************************************************
// Check if key exists in nvs.                                                                     *
//**************************************************************************************************
bool nvssearch ( const char* key )
{
  size_t        len = NVSBUFSIZE ;                      // Length of the string

  nvsopen() ;                                           // Be sure to open nvs
  nvserr = nvs_get_str ( nvshandle, key, NULL, &len ) ; // Get length of contents
  return ( nvserr == ESP_OK ) ;                         // Return true if found
}


//**************************************************************************************************
//                                      T F T S E T                                                *
//**************************************************************************************************
// Request to display a segment on TFT.  Version for char* and String parameter.                   *
//**************************************************************************************************
void tftset ( uint16_t inx, const char *str )
{
  if ( inx < TFTSECS )                                  // Segment available on display
  {
    if ( str )                                          // String specified?
    {
      tftdata[inx].str = String ( str ) ;               // Yes, set string
    }
    tftdata[inx].update_req = true ;                    // and request flag
  }
}

void tftset ( uint16_t inx, String& str )
{
  if ( inx < TFTSECS )                                  // Segment available on display
  {
    tftdata[inx].str = str ;                            // Set string
    tftdata[inx].update_req = true ;                    // and request flag
  }
}


//**************************************************************************************************
//                                      U T F 8 A S C I I                                          *
//**************************************************************************************************
// UTF8-Decoder: convert UTF8-string to extended ASCII.                                            *
// Convert a single Character from UTF8 to Extended ASCII.                                         *
// Return "0" if a byte has to be ignored.                                                         *
//**************************************************************************************************
byte utf8ascii ( byte ascii )
{
  static const byte lut_C3[] =
  { "AAAAAAACEEEEIIIIDNOOOOO#0UUUU###aaaaaaaceeeeiiiidnooooo##uuuuyyy" } ;
  static byte       c1 ;              // Last character buffer
  byte              res = 0 ;         // Result, default 0

  if ( ascii <= 0x7F )                // Standard ASCII-set 0..0x7F handling
  {
    c1 = 0 ;
    res = ascii ;                     // Return unmodified
  }
  else
  {
    switch ( c1 )                     // Conversion depending on first UTF8-character
    {
      case 0xC2: res = '~' ;
        break ;
      case 0xC3: res = lut_C3[ascii - 128] ;
        break ;
      case 0x82: if ( ascii == 0xAC )
        {
          res = 'E' ;                 // Special case Euro-symbol
        }
    }
    c1 = ascii ;                      // Remember actual character
  }
  return res ;                        // Otherwise: return zero, if character has to be ignored
}


//**************************************************************************************************
//                                      U T F 8 A S C I I                                          *
//**************************************************************************************************
// In Place conversion UTF8-string to Extended ASCII (ASCII is shorter!).                          *
//**************************************************************************************************
void utf8ascii ( char* s )
{
  int  i, k = 0 ;                     // Indexes for in en out string
  char c ;

  for ( i = 0 ; s[i] ; i++ )          // For every input character
  {
    c = utf8ascii ( s[i] ) ;          // Translate if necessary
    if ( c )                          // Good translation?
    {
      s[k++] = c ;                    // Yes, put in output string
    }
  }
  s[k] = 0 ;                          // Take care of delimeter
}


//**************************************************************************************************
//                                      U T F 8 A S C I I                                          *
//**************************************************************************************************
// Conversion UTF8-String to Extended ASCII String.                                                *
//**************************************************************************************************
String utf8ascii ( const char* s )
{
  int  i ;                            // Index for input string
  char c ;
  String res = "" ;                   // Result string

  for ( i = 0 ; s[i] ; i++ )          // For every input character
  {
    c = utf8ascii ( s[i] ) ;          // Translate if necessary
    if ( c )                          // Good translation?
    {
      res += String ( c ) ;           // Yes, put in output string
    }
  }
  return res ;
}


//**************************************************************************************************
//                                          D B G P R I N T                                        *
//**************************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks the DEBUG flag.        *
// Print only if DEBUG flag is true.  Always returns the formatted string.                         *
//**************************************************************************************************
char* dbgprint ( const char* format, ... )
{
  static char sbuf[DEBUG_BUFFER_SIZE] ;                // For debug lines
  va_list varArgs ;                                    // For variable number of params

  va_start ( varArgs, format ) ;                       // Prepare parameters
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;  // Format the message
  va_end ( varArgs ) ;                                 // End of using parameters
  if ( DEBUG )                                         // DEBUG on?
  {
    Serial.print ( "D: " ) ;                           // Yes, print prefix
    Serial.println ( sbuf ) ;                          // and the info
  }
  return sbuf ;                                        // Return stored string
}




//**************************************************************************************************
//                                  S E L E C T N E X T S D N O D E                                *
//**************************************************************************************************
// Select the next or previous mp3 file from SD.  If the last selected song was random, the next   *
// track is a random one too.  Otherwise the next/previous node is choosen.                        *
// If nodeID is "0" choose a random nodeID.                                                        *
// Delta is +1 or -1 for next or previous track.                                                   *
// The nodeID will be returned to the caller.                                                      *
//**************************************************************************************************
String selectnextSDnode ( String curnod, int16_t delta )
{
  int16_t        inx, inx2 ;                           // Position in nodelist

  if ( hostreq )                                       // Host request already set?
  {
    return "" ;                                        // Yes, no action
  }
  dbgprint ( "SD_currentnode is %s, "
             "curnod is %s, "
             "delta is %d",
             SD_currentnode.c_str(),
             curnod.c_str(),
             delta ) ;
  if ( SD_currentnode == "0" )                         // Random playing?
  {
    return SD_currentnode ;                            // Yes, return random nodeID
  }
  else
  {
    inx = SD_nodelist.indexOf ( curnod ) ;             // Get position of current nodeID in list
    if ( delta > 0 )                                   // Next track?
    {
      inx += curnod.length() + 1 ;                     // Get position of next nodeID in list
      if ( inx >= SD_nodelist.length() )               // End of list?
      {
        inx = 0 ;                                      // Yes, wrap around
      }
    }
    else
    {
      if ( inx == 0 )                                  // At the begin of the list?
      {
        inx = SD_nodelist.length()  ;                  // Yes, goto end of list
      }
      inx-- ;                                          // Index of delimeter of previous node ID
      while ( ( inx > 0 ) &&
              ( SD_nodelist[inx - 1] != '\n' ) )
      {
        inx-- ;
      }
    }
    inx2 = SD_nodelist.indexOf ( "\n", inx ) ;         // Find end of node ID
  }
  return SD_nodelist.substring ( inx, inx2 ) ;         // Return nodeID
}


//**************************************************************************************************
//                                      G E T S D F I L E N A M E                                  *
//**************************************************************************************************
// Translate the nodeID of a track to the full filename that can be used as a station.             *
// If nodeID is "0" choose a random nodeID.                                                        *
//**************************************************************************************************
String getSDfilename ( String nodeID )
{
  String          res ;                                    // Function result
  File            root, file ;                             // Handle to root and directory entry
  uint16_t        n, i ;                                   // Current seqnr and counter in directory
  int16_t         inx ;                                    // Position in nodeID
  const char*     p = "/" ;                                // Points to directory/file
  uint16_t        rndnum ;                                 // Random index in SD_nodelist
  int             nodeinx = 0 ;                            // Points to node ID in SD_nodecount
  int             nodeinx2 ;                               // Points to end of node ID in SD_nodecount

  SD_currentnode = nodeID ;                                // Save current node
  if ( nodeID == "0" )                                     // Empty parameter?
  {
    dbgprint ( "getSDfilename random choice" ) ;
    rndnum = random ( SD_nodecount ) ;                     // Yes, choose a random node
    for ( i = 0 ; i < rndnum ; i++ )                       // Find the node ID
    {
      // Search to begin of the random node by skipping lines
      nodeinx = SD_nodelist.indexOf ( "\n", nodeinx ) + 1 ;
    }
    nodeinx2 = SD_nodelist.indexOf ( "\n", nodeinx ) ;     // Find end of node ID
    nodeID = SD_nodelist.substring ( nodeinx,
                                     nodeinx2 ) ;          // Get node ID
  }
  dbgprint ( "getSDfilename requested node ID is %s",      // Show requeste node ID
             nodeID.c_str() ) ;
  while ( ( n = nodeID.toInt() ) )                         // Next sequence in current level
  {
    inx = nodeID.indexOf ( "," ) ;                         // Find position of comma
    if ( inx >= 0 )
    {
      nodeID = nodeID.substring ( inx + 1 ) ;              // Remove sequence in this level from nodeID
    }
    claimSPI ( "sdopen" ) ;                                // Claim SPI bus
    root = SD.open ( p ) ;                                 // Open the directory (this level)
    releaseSPI() ;                                         // Release SPI bus
    for ( i = 1 ; i <=  n ; i++ )
    {
      claimSPI ( "sdopenxt" ) ;                            // Claim SPI bus
      file = root.openNextFile() ;                         // Get next directory entry
      releaseSPI() ;                                       // Release SPI bus
      delay ( 5 ) ;                                        // Allow playtask
    }
    p = file.name() ;                                      // Points to directory- or file name
  }
  res = String ( "localhost" ) + String ( p ) ;            // Format result
  dbgprint ( "Selected file is %s", res.c_str() ) ;        // Show result
  return res ;                                             // Return full station spec
}


//**************************************************************************************************
//                                      L I S T S D T R A C K S                                    *
//**************************************************************************************************
// Search all MP3 files on directory of SD card.  Return the number of files found.                *
// A "node" of max. 4 levels ( the subdirectory level) will be generated for every file.           *
// The numbers within the node-array is the sequence number of the file/directory in that          *
// subdirectory.                                                                                   *
// A node ID is a string like "2,1,4,0", which means the 4th file in the first directory           *
// of the second directory.                                                                        *
// The list will be send to the webinterface if parameter "send"is true.                           *
//**************************************************************************************************
int listsdtracks ( const char * dirname, int level = 0, bool send = true )
{
  const  uint16_t SD_MAXDEPTH = 4 ;                     // Maximum depts.  Note: see mp3play_html.
  static uint16_t fcount, oldfcount ;                   // Total number of files
  static uint16_t SD_node[SD_MAXDEPTH + 1] ;            // Node ISs, max levels deep
  static String   SD_outbuf ;                           // Output buffer for cmdclient
  uint16_t        ldirname ;                            // Length of dirname to remove from filename
  File            root, file ;                          // Handle to root and directory entry
  String          filename ;                            // Copy of filename for lowercase test
  uint16_t        i ;                                   // Loop control to compute single node id
  String          tmpstr ;                              // Tijdelijke opslag node ID

  if ( strcmp ( dirname, "/" ) == 0 )                   // Are we at the root directory?
  {
    fcount = 0 ;                                        // Yes, reset count
    memset ( SD_node, 0, sizeof(SD_node) ) ;            // And sequence counters
    SD_outbuf = String() ;                              // And output buffer
    SD_nodelist = String() ;                            // And nodelist
    if ( !SD_okay )                                     // See if known card
    {
      if ( send )
      {
        cmdclient.println ( "0/No tracks found" ) ;     // No SD card, emppty list
      }
      return 0 ;
    }
  }
  oldfcount = fcount ;                                  // To see if files found in this directory
  dbgprint ( "SD directory is %s", dirname ) ;          // Show current directory
  ldirname = strlen ( dirname ) ;                       // Length of dirname to remove from filename
  claimSPI ( "sdopen2" ) ;                              // Claim SPI bus
  root = SD.open ( dirname ) ;                          // Open the current directory level
  releaseSPI() ;                                        // Release SPI bus
  if ( !root || !root.isDirectory() )                   // Success?
  {
    dbgprint ( "%s is not a directory or not root",     // No, print debug message
               dirname ) ;
    return fcount ;                                     // and return
  }
  while ( true )                                        // Find all mp3 files
  {
    claimSPI ( "opennextf" ) ;                          // Claim SPI bus
    file = root.openNextFile() ;                        // Try to open next
    releaseSPI() ;                                      // Release SPI bus
    if ( !file )
    {
      break ;                                           // End of list
    }
    SD_node[level]++ ;                                  // Set entry sequence of current level
    if ( file.name()[0] == '.' )                        // Skip hidden directories
    {
      continue ;
    }
    if ( file.isDirectory() )                           // Is it a directory?
    {
      if ( level < SD_MAXDEPTH )                        // Yes, dig deeper
      {
        listsdtracks ( file.name(), level + 1, send ) ; // Note: called recursively
        SD_node[level + 1] = 0 ;                        // Forget counter for one level up
      }
    }
    else
    {
      filename = String ( file.name() ) ;               // Copy filename
      filename.toLowerCase() ;                          // Force lowercase
      if ( filename.endsWith ( ".mp3" ) )               // It is a file, but is it an MP3?
      {
        fcount++ ;                                      // Yes, count total number of MP3 files
        tmpstr = String() ;                             // Empty
        for ( i = 0 ; i < SD_MAXDEPTH ; i++ )           // Add a line containing the node to SD_outbuf
        {
          if ( i )                                      // Need to add separating comma?
          {
            tmpstr += String ( "," ) ;                  // Yes, add comma
          }
          tmpstr += String ( SD_node[i] ) ;             // Add sequence number
        }
        if ( send )                                     // Need to add to string for webinterface?
        {
          SD_outbuf += tmpstr +                         // Form line for mp3play_html page
                       utf8ascii ( file.name() +        // Filename starts after directoryname
                                   ldirname ) +
                       String ( "\n" ) ;
        }
        SD_nodelist += tmpstr + String ( "\n" ) ;       // Add to nodelist
        //dbgprint ( "Track: %s",                       // Show debug info
        //           file.name() + ldirname ) ;
        if ( SD_outbuf.length() > 1000 )                // Buffer full?
        {
          cmdclient.print ( SD_outbuf ) ;               // Yes, send it
          SD_outbuf = String() ;                        // Clear buffer
        }
      }
      else if ( filename.endsWith ( ".m3u" ) )          // Is it an .m3u file?
      {
        dbgprint ( "Playlist %s", file.name() ) ;       // Yes, show
      }
    }
    if ( send )
    {
      mp3loop() ;                                       // Keep playing
    }
  }
  if ( fcount != oldfcount )                            // Files in this directory?
  {
    SD_outbuf += String ( "-1/ \n" ) ;                  // Spacing in list
  }
  if ( SD_outbuf.length() )                             // Flush buffer if not empty
  {
    cmdclient.print ( SD_outbuf ) ;                     // Filled, send it
    SD_outbuf = String() ;                              // Continue with empty buffer
  }
  return fcount ;                                       // Return number of MP3s (sofar)
}


//**************************************************************************************************
//                                     G E T E N C R Y P T I O N T Y P E                           *
//**************************************************************************************************
// Read the encryption type of the network and return as a 4 byte name                             *
//**************************************************************************************************
const char* getEncryptionType ( wifi_auth_mode_t thisType )
{
  switch ( thisType )
  {
    case WIFI_AUTH_OPEN:
      return "OPEN" ;
    case WIFI_AUTH_WEP:
      return "WEP" ;
    case WIFI_AUTH_WPA_PSK:
      return "WPA_PSK" ;
    case WIFI_AUTH_WPA2_PSK:
      return "WPA2_PSK" ;
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "WPA_WPA2_PSK" ;
    case WIFI_AUTH_MAX:
      return "MAX" ;
    default:
      break ;
  }
  return "????" ;
}


//**************************************************************************************************
//                                        L I S T N E T W O R K S                                  *
//**************************************************************************************************
// List the available networks.                                                                    *
// Acceptable networks are those who have an entry in the preferences.                             *
// SSIDs of available networks will be saved for use in webinterface.                              *
//**************************************************************************************************
void listNetworks()
{
  WifiInfo_t       winfo ;            // Entry from wifilist
  wifi_auth_mode_t encryption ;       // TKIP(WPA), WEP, etc.
  const char*      acceptable ;       // Netwerk is acceptable for connection
  int              i, j ;             // Loop control

  dbgprint ( "Scan Networks" ) ;                         // Scan for nearby networks
  numSsid = WiFi.scanNetworks() ;
  dbgprint ( "Scan completed" ) ;
  if ( numSsid <= 0 )
  {
    dbgprint ( "Couldn't get a wifi connection" ) ;
    return ;
  }
  // print the list of networks seen:
  dbgprint ( "Number of available networks: %d",
             numSsid ) ;
  // Print the network number and name for each network found and
  for ( i = 0 ; i < numSsid ; i++ )
  {
    acceptable = "" ;                                    // Assume not acceptable
    for ( j = 0 ; j < wifilist.size() ; j++ )            // Search in wifilist
    {
      winfo = wifilist[j] ;                              // Get one entry
      if ( WiFi.SSID(i).indexOf ( winfo.ssid ) == 0 )    // Is this SSID acceptable?
      {
        acceptable = "Acceptable" ;
        break ;
      }
    }
    encryption = WiFi.encryptionType ( i ) ;
    dbgprint ( "%2d - %-25s Signal: %3d dBm, Encryption %4s, %s",
               i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
               getEncryptionType ( encryption ),
               acceptable ) ;
    // Remember this network for later use
    networks += WiFi.SSID(i) + String ( "|" ) ;
  }
  dbgprint ( "End of list" ) ;
}


//**************************************************************************************************
//                                          T I M E R 1 0 S E C                                    *
//**************************************************************************************************
// Extra watchdog.  Called every 10 seconds.                                                       *
// If totalcount has not been changed, there is a problem and playing will stop.                   *
// Note that calling timely procedures within this routine or in called functions will             *
// cause a crash!                                                                                  *
//**************************************************************************************************
void IRAM_ATTR timer10sec()
{
  static uint32_t oldtotalcount = 7321 ;          // Needed for change detection
  static uint8_t  morethanonce = 0 ;              // Counter for succesive fails
  uint32_t        bytesplayed ;                   // Bytes send to MP3 converter

  if ( datamode & ( INIT | HEADER | DATA |        // Test op playing
                    METADATA | PLAYLISTINIT |
                    PLAYLISTHEADER |
                    PLAYLISTDATA ) )
  {
    bytesplayed = totalcount - oldtotalcount ;    // Nunber of bytes played in the 10 seconds
    oldtotalcount = totalcount ;                  // Save for comparison in next cycle
    if ( bytesplayed == 0 )                       // Still playing?
    {
      
      if ( morethanonce > 250 )                    // No! Happened too many times?
      {
        morethanonce = 0;
//        datamode = STOPREQD ;  
//        reload_accept=0;
//        accept_try =0;
//        ESP.restart() ;                           // Reset the CPU, probably no return
      }
      if ( datamode & ( PLAYLISTDATA |            // In playlist mode?
                        PLAYLISTINIT |
                        PLAYLISTHEADER ) )
      {
        playlist_num = 0 ;                        // Yes, end of playlist
      }
      if ( ( morethanonce > 0 ) ||                // Happened more than once?
           ( playlist_num > 0 ) )                 // Or playlist active?
      {
       datamode = STOPREQD ;                     // Stop player
       retry();
      }
      morethanonce++ ;                            // Count the fails
      dbgprint ( "[Try %d]",morethanonce) ;
      sts=2;
      stt_ply=0;
      digitalWrite(MUTE,LOW);
    }
    else
    {
      //                                          // Data has been send to MP3 decoder
      // Bitrate in kbits/s is bytesplayed / 10 / 1000 * 8
      mbitrate = ( bytesplayed + 625 ) / 1250 ;   // Measured bitrate
      morethanonce = 0 ;                          // Data seen, reset failcounter
      stt_ply=1;
    }
       if (  stt_ply == 1 && playingstat==0)
    {
      sts = 0;
    }
    else if (stt_ply == 1 && playingstat==1)
    {
      sts = 1;
      digitalWrite(MUTE,HIGH);
    }
    else if ( accept_try == 0 && stt_ply == 0)
    {
      sts = 0;
      err=2;
     // get_time_ert (); 
    }
//    else if (stt_ply == 0)
//    {
//      sts=2;
//    }
  }
}


//**************************************************************************************************
//                                          T I M E R 1 0 0                                        *
//**************************************************************************************************
// Called every 100 msec on interrupt level, so must be in IRAM and no lengthy operations          *
// allowed.                                                                                        *
//**************************************************************************************************
void IRAM_ATTR timer100()
{
  sv int16_t   count10sec = 0 ;                   // Counter for activatie 10 seconds process
  sv int16_t   eqcount = 0 ;                      // Counter for equal number of clicks
  sv int16_t   oldclickcount = 0 ;                // To detect difference

  if ( ++count10sec == 20  )                     // 10 seconds passed?
  {
    timer10sec() ;                                // Yes, do 10 second procedure
    count10sec = 0 ;                              // Reset count
  }
  if ( ( count10sec % 10 ) == 0 )                 // One second over?
  {
    scaniocount = scanios ;                       // TEST*TEST*TEST
    scanios = 0 ;
    if ( ++timeinfo.tm_sec >= 60 )                // Yes, update number of seconds
    {
      timeinfo.tm_sec = 0 ;                       // Wrap after 60 seconds
      if ( ++timeinfo.tm_min >= 60 )
      {
        timeinfo.tm_min = 0 ;                     // Wrap after 60 minutes
        if ( ++timeinfo.tm_hour >= 24 )
        {
          timeinfo.tm_hour = 0 ;                  // Wrap after 24 hours
        }
      }
    }
    time_req = true ;                             // Yes, show current time request
    if ( ++bltimer == BL_TIME )                   // Time to blank the TFT screen?
    {
      bltimer = 0 ;                               // Yes, reset counter
      blset ( false ) ;                           // Disable TFT (backlight)
    }
  }
  // Handle rotary encoder. Inactivity counter will be reset by encoder interrupt
  if ( ++enc_inactivity == 36000 )                // Count inactivity time
  {
    enc_inactivity = 1000 ;                       // Prevent wrap
  }
  // Now detection of single/double click of rotary encoder switch
  if ( clickcount )                               // Any click?
  {
    if ( oldclickcount == clickcount )            // Yes, stable situation?
    {
      if ( ++eqcount == 4 )                       // Long time stable?
      {
        eqcount = 0 ;
        if ( clickcount > 2 )                     // Triple click?
        {
        //  tripleclick = true ;                    // Yes, set result
        }
        else if ( clickcount == 2 )               // Double click?
        {
          doubleclick = true ;                    // Yes, set result
        }
        else
        {
          singleclick = true ;                    // Just one click seen
        }
        clickcount = 0 ;                          // Reset number of clicks
      }
    }
    else
    {
      oldclickcount = clickcount ;                // To detect change
      eqcount = 0 ;                               // Not stable, reset count
    }
  }
}


//**************************************************************************************************
//                                          I S R _ I R                                            *
//**************************************************************************************************
// Interrupts received from VS1838B on every change of the signal.                                 *
// Intervals are 640 or 1640 microseconds for data.  syncpulses are 3400 micros or longer.         *
// Input is complete after 65 level changes.                                                       *
// Only the last 32 level changes are significant and will be handed over to common data.          *
//**************************************************************************************************
void IRAM_ATTR isr_IR()
{
  sv uint32_t      t0 = 0 ;                          // To get the interval
  sv uint32_t      ir_locvalue = 0 ;                 // IR code
  sv int           ir_loccount = 0 ;                 // Length of code
  uint32_t         t1, intval ;                      // Current time and interval since last change
  uint32_t         mask_in = 2 ;                     // Mask input for conversion
  uint16_t         mask_out = 1 ;                    // Mask output for conversion

  t1 = micros() ;                                    // Get current time
  intval = t1 - t0 ;                                 // Compute interval
  t0 = t1 ;                                          // Save for next compare
  if ( ( intval > 300 ) && ( intval < 800 ) )        // Short pulse?
  {
    ir_locvalue = ir_locvalue << 1 ;                 // Shift in a "zero" bit
    ir_loccount++ ;                                  // Count number of received bits
    ir_0 = ( ir_0 * 3 + intval ) / 4 ;               // Compute average durartion of a short pulse
  }
  else if ( ( intval > 1400 ) && ( intval < 1900 ) ) // Long pulse?
  {
    ir_locvalue = ( ir_locvalue << 1 ) + 1 ;         // Shift in a "one" bit
    ir_loccount++ ;                                  // Count number of received bits
    ir_1 = ( ir_1 * 3 + intval ) / 4 ;               // Compute average durartion of a short pulse
  }
  else if ( ir_loccount == 65 )                      // Value is correct after 65 level changes
  {
    while ( mask_in )                                // Convert 32 bits to 16 bits
    {
      if ( ir_locvalue & mask_in )                   // Bit set in pattern?
      {
        ir_value |= mask_out ;                       // Set set bit in result
      }
      mask_in <<= 2 ;                                // Shift input mask 2 positions
      mask_out <<= 1 ;                               // Shift output mask 1 position
    }
    ir_loccount = 0 ;                                // Ready for next input
  }
  else
  {
    ir_locvalue = 0 ;                                // Reset decoding
    ir_loccount = 0 ;
  }
}


//**************************************************************************************************
//                                          I S R _ E N C _ S W I T C H                            *
//**************************************************************************************************
// Interrupts received from rotary encoder switch.                                                 *
//**************************************************************************************************
void IRAM_ATTR isr_enc_switch()
{
  sv uint32_t     oldtime = 0 ;                            // Time in millis previous interrupt
  sv bool         sw_state ;                               // True is pushed (LOW)
  bool            newstate ;                               // Current state of input signal
  uint32_t        newtime ;                                // Current timestamp

  // Read current state of SW pin
  newstate = ( digitalRead ( ini_block.enc_sw_pin ) == LOW ) ;
  newtime = millis() ;
  if ( newtime == oldtime )                                // Debounce
  {
    return ;
  }
  if ( newstate != sw_state )                              // State changed?
  {
    sw_state = newstate ;                                  // Yes, set current (new) state
    if ( !sw_state )                                       // SW released?
    {
      if ( ( newtime - oldtime ) > 5000 )                  // More than 1 second?
      {
        longclick = true ;                                 // Yes, register longclick
      }
      else
      {
        clickcount++ ;                                     // Yes, click detected
      }
      enc_inactivity = 0 ;                                 // Not inactive anymore
    }
  }
  oldtime = newtime ;                                      // For next compare
}


//**************************************************************************************************
//                                          I S R _ E N C _ T U R N                                *
//**************************************************************************************************
// Interrupts received from rotary encoder (clk signal) knob turn.                                 *
// The encoder is a Manchester coded device, the outcomes (-1,0,1) of all the previous state and   *
// actual state are stored in the enc_states[].                                                    *
// Full_status is a 4 bit variable, the upper 2 bits are the previous encoder values, the lower    *
// ones are the actual ones.                                                                       *
// 4 bits cover all the possible previous and actual states of the 2 PINs, so this variable is     *
// the index enc_states[].                                                                         *
// No debouncing is needed, because only the valid states produce values different from 0.         *
// Rotation is 4 if position is moved from one fixed position to the next, so it is devided by 4.  *
//**************************************************************************************************
void IRAM_ATTR isr_enc_turn()
{
  sv uint32_t     old_state = 0x0001 ;                          // Previous state
  sv int16_t      locrotcount = 0 ;                             // Local rotation count
  uint8_t         act_state = 0 ;                               // The current state of the 2 PINs
  uint8_t         inx ;                                         // Index in enc_state
  sv const int8_t enc_states [] =                               // Table must be in DRAM (iram safe)
  { 0,                    // 00 -> 00
    -1,                   // 00 -> 01                           // dt goes HIGH
    1,                    // 00 -> 10
    0,                    // 00 -> 11
    1,                    // 01 -> 00                           // dt goes LOW
    0,                    // 01 -> 01
    0,                    // 01 -> 10
    -1,                   // 01 -> 11                           // clk goes HIGH
    -1,                   // 10 -> 00                           // clk goes LOW
    0,                    // 10 -> 01
    0,                    // 10 -> 10
    1,                    // 10 -> 11                           // dt goes HIGH
    0,                    // 11 -> 00
    1,                    // 11 -> 01                           // clk goes LOW
    -1,                   // 11 -> 10                           // dt goes HIGH
    0                     // 11 -> 11
  } ;
  // Read current state of CLK, DT pin. Result is a 2 bit binary number: 00, 01, 10 or 11.
  act_state = ( digitalRead ( ini_block.enc_clk_pin ) << 1 ) +
              digitalRead ( ini_block.enc_dt_pin ) ;
  inx = ( old_state << 2 ) + act_state ;                        // Form index in enc_states
  locrotcount += enc_states[inx] ;                              // Get delta: 0, +1 or -1
  if ( locrotcount == 4 )
  {
    rotationcount++ ;                                           // Divide by 4
    locrotcount = 0 ;
  }
  else if ( locrotcount == -4 )
  {
    rotationcount-- ;                                           // Divide by 4
    locrotcount = 0 ;
  }
  old_state = act_state ;                                       // Remember current status
  enc_inactivity = 0 ;
}


//**************************************************************************************************
//                                S H O W S T R E A M T I T L E                                    *
//**************************************************************************************************
// Show artist and songtitle if present in metadata.                                               *
// Show always if full=true.                                                                       *
//**************************************************************************************************
void showstreamtitle ( const char *ml, bool full )
{
  char*             p1 ;
  char*             p2 ;
  char              streamtitle[150] ;           // Streamtitle from metadata

  if ( strstr ( ml, "StreamTitle=" ) )
  {
    dbgprint ( "Streamtitle found, %d bytes", strlen ( ml ) ) ;
    dbgprint ( ml ) ;
    p1 = (char*)ml + 12 ;                       // Begin of artist and title
    if ( ( p2 = strstr ( ml, ";" ) ) )          // Search for end of title
    {
      if ( *p1 == '\'' )                        // Surrounded by quotes?
      {
        p1++ ;
        p2-- ;
      }
      *p2 = '\0' ;                              // Strip the rest of the line
    }
    // Save last part of string as streamtitle.  Protect against buffer overflow
    strncpy ( streamtitle, p1, sizeof ( streamtitle ) ) ;
    streamtitle[sizeof ( streamtitle ) - 1] = '\0' ;
  }
  else if ( full )
  {
    // Info probably from playlist
    strncpy ( streamtitle, ml, sizeof ( streamtitle ) ) ;
    streamtitle[sizeof ( streamtitle ) - 1] = '\0' ;
  }
  else
  {
    icystreamtitle = "" ;                       // Unknown type
    return ;                                    // Do not show
  }
  // Save for status request from browser and for MQTT
  icystreamtitle = streamtitle ;
  if ( ( p1 = strstr ( streamtitle, " - " ) ) ) // look for artist/title separator
  {
    p2 = p1 + 3 ;                               // 2nd part of text at this position
    if ( displaytype == T_NEXTION )
    {
      *p1++ = '\\' ;                            // Found: replace 3 characters by "\r"
      *p1++ = 'r' ;                             // Found: replace 3 characters by "\r"
    }
    else
    {
      *p1++ = '\n' ;                            // Found: replace 3 characters by newline
    }
    if ( *p2 == ' ' )                           // Leading space in title?
    {
      p2++ ;
    }
    strcpy ( p1, p2 ) ;                         // Shift 2nd part of title 2 or 3 places
  }
  tftset ( 1, streamtitle ) ;                   // Set screen segment text middle part
}


//**************************************************************************************************
//                                    S T O P _ M P 3 C L I E N T                                  *
//**************************************************************************************************
// Disconnect from the server.                                                                     *
//**************************************************************************************************
void stop_mp3client ()
{

  while ( mp3client.connected() )
  {
    dbgprint ( "Stopping client" ) ;               // Stop connection to host
    mp3client.flush() ;
    mp3client.stop() ;
    delay ( 500 ) ;
  }
  mp3client.flush() ;                              // Flush stream client
  mp3client.stop() ;                               // Stop stream client
}


//**************************************************************************************************
//                                    C O N N E C T T O H O S T                                    *
//**************************************************************************************************
// Connect to the Internet radio server specified by newpreset.                                    *
//**************************************************************************************************
bool connecttohost()
{
  int         inx ;                                 // Position of ":" in hostname
  uint16_t    port = 80 ;                           // Port number for host
  String      extension = "/" ;                     // May be like "/mp3" in "skonto.ls.lv:8002/mp3"
  String      hostwoext = host ;                    // Host without extension and portnumber
  String      auth  ;                               // For basic authentication

  stop_mp3client() ;                                // Disconnect if still connected
  dbgprint ( "Connect to new host %s", host.c_str() ) ;
  tftset ( 0, "Internet-Radio" ) ;                     // Set screen segment text top line
  tftset ( 1, "" ) ;                                // Clear song and artist
  displaytime ( "" ) ;                              // Clear time on TFT screen
  datamode = INIT ;                                 // Start default in metamode
  chunked = false ;                                 // Assume not chunked
  if ( host.endsWith ( ".m3u" ) )                   // Is it an m3u playlist?
  {
    playlist = host ;                               // Save copy of playlist URL
    datamode = PLAYLISTINIT ;                       // Yes, start in PLAYLIST mode
    if ( playlist_num == 0 )                        // First entry to play?
    {
      playlist_num = 1 ;                            // Yes, set index
    }
    dbgprint ( "Playlist request, entry %d", playlist_num ) ;
  }
  // In the URL there may be an extension, like noisefm.ru:8000/play.m3u&t=.m3u
  inx = host.indexOf ( "/" ) ;                      // Search for begin of extension
  if ( inx > 0 )                                    // Is there an extension?
  {
    extension = host.substring ( inx ) ;            // Yes, change the default
    hostwoext = host.substring ( 0, inx ) ;         // Host without extension
  }
  // In the host there may be a portnumber
  inx = hostwoext.indexOf ( ":" ) ;                 // Search for separator
  if ( inx >= 0 )                                   // Portnumber available?
  {
    port = host.substring ( inx + 1 ).toInt() ;     // Get portnumber as integer
    hostwoext = host.substring ( 0, inx ) ;         // Host without portnumber
  }
  dbgprint ( "Connect to %s on port %d, extension %s",
             hostwoext.c_str(), port, extension.c_str() ) ;
  if ( mp3client.connect ( hostwoext.c_str(), port ) )
  {
   // digitalWrite(MUTE, HIGH);
    dbgprint ( "Connected to server" ) ;
    if ( nvssearch ( "basicauth" ) )                // Does "basicauth" exists?
    {
      auth = nvsgetstr ( "basicauth" ) ;            // Use basic authentication?
      if ( auth != "" )                             // Should be user:passwd
      {
        auth = base64::encode ( auth.c_str() ) ;   // Encode
        auth = String ( "Authorization: Basic " ) +
               auth + String ( "\r\n" ) ;
      }
    }
    mp3client.print ( String ( "GET " ) +
                      extension +
                      String ( " HTTP/1.1\r\n" ) +
                      String ( "Host: " ) +
                      hostwoext +
                      String ( "\r\n" ) +
                      String ( "Icy-MetaData:1\r\n" ) +
                      auth +
                      String ( "Connection: close\r\n\r\n" ) ) ;
    return true ;
  }
  dbgprint ( "Request %s failed!", host.c_str() ) ;
  digitalWrite(MUTE, LOW);
  return false ;
}


//**************************************************************************************************
//                                      S S C O N V                                                *
//**************************************************************************************************
// Convert an array with 4 "synchsafe integers" to a number.                                       *
// There are 7 bits used per byte.                                                                 *
//**************************************************************************************************
uint32_t ssconv ( const uint8_t* bytes )
{
  uint32_t res = 0 ;                                      // Result of conversion
  uint8_t  i ;                                            // Counter number of bytes to convert

  for ( i = 0 ; i < 4 ; i++ )                             // Handle 4 bytes
  {
    res = res * 128 + bytes[i] ;                          // Convert next 7 bits
  }
  return res ;                                            // Return the result
}


//**************************************************************************************************
//                                  H A N D L E _ I D 3                                            *
//**************************************************************************************************
// Check file on SD card for ID3 tags and use them to display some info.                           *
// Extended headers are not parsed.                                                                *
//**************************************************************************************************
void handle_ID3 ( String &path )
{
  char*  p ;                                                // Pointer to filename
  struct ID3head_t                                          // First part of ID3 info
  {
    char    fid[3] ;                                        // Should be filled with "ID3"
    uint8_t majV, minV ;                                    // Major and minor version
    uint8_t hflags ;                                        // Headerflags
    uint8_t ttagsize[4] ;                                   // Total tag size
  } ID3head ;
  uint8_t  exthsiz[4] ;                                     // Extended header size
  uint32_t stx ;                                            // Ext header size converted
  uint32_t sttg ;                                           // Total tagsize converted
  uint32_t stg ;                                            // Size of a single tag
  struct ID3tag_t                                           // Tag in ID3 info
  {
    char    tagid[4] ;                                      // Things like "TCON", "TYER", ...
    uint8_t tagsize[4] ;                                    // Size of the tag
    uint8_t tagflags[2] ;                                   // Tag flags
  } ID3tag ;
  uint8_t  tmpbuf[4] ;                                      // Scratch buffer
  uint8_t  tenc ;                                           // Text encoding
  String   albttl = String() ;                              // Album and title

  tftset ( 2, "Playing from local file" ) ;                 // Assume no ID3
  p = (char*)path.c_str() + 1 ;                             // Point to filename (after the slash)
  showstreamtitle ( p, true ) ;                             // Show the filename as title (middle part)
  mp3file = SD.open ( path ) ;                              // Open the file
  if ( path.endsWith ( ".mu3" ) )                           // Is it a playlist?
  {
    return ;                                                // Yes, no ID's, but leave file open
  }
  mp3file.read ( (uint8_t*)&ID3head, sizeof(ID3head) ) ;    // Read first part of ID3 info
  if ( strncmp ( ID3head.fid, "ID3", 3 ) == 0 )
  {
    sttg = ssconv ( ID3head.ttagsize ) ;                    // Convert tagsize
    dbgprint ( "Found ID3 info" ) ;
    if ( ID3head.hflags & 0x40 )                            // Extended header?
    {
      stx = ssconv ( exthsiz ) ;                            // Yes, get size of extended header
      while ( stx-- )
      {
        mp3file.read () ;                                   // Skip next byte of extended header
      }
    }
    while ( sttg > 10 )                                     // Now handle the tags
    {
      sttg -= mp3file.read ( (uint8_t*)&ID3tag,
                             sizeof(ID3tag) ) ;             // Read first part of a tag
      if ( ID3tag.tagid[0] == 0 )                           // Reached the end of the list?
      {
        break ;                                             // Yes, quit the loop
      }
      stg = ssconv ( ID3tag.tagsize ) ;                     // Convert size of tag
      if ( ID3tag.tagflags[1] & 0x08 )                      // Compressed?
      {
        sttg -= mp3file.read ( tmpbuf, 4 ) ;               // Yes, ignore 4 bytes
        stg -= 4 ;                                         // Reduce tag size
      }
      if ( ID3tag.tagflags[1] & 0x044 )                     // Encrypted or grouped?
      {
        sttg -= mp3file.read ( tmpbuf, 1 ) ;               // Yes, ignore 1 byte
        stg-- ;                                            // Reduce tagsize by 1
      }
      if ( stg > ( sizeof(metalinebf) + 2 ) )               // Room for tag?
      {
        break ;                                             // No, skip this and further tags
      }
      sttg -= mp3file.read ( (uint8_t*)metalinebf,
                             stg ) ;                        // Read tag contents
      metalinebf[stg] = '\0' ;                              // Add delimeter
      tenc = metalinebf[0] ;                                // First byte is encoding type
      if ( tenc == '\0' )                                   // Debug all tags with encoding 0
      {
        dbgprint ( "ID3 %s = %s", ID3tag.tagid,
                   metalinebf + 1 ) ;
      }
      if ( ( strncmp ( ID3tag.tagid, "TALB", 4 ) == 0 ) ||  // Album title
           ( strncmp ( ID3tag.tagid, "TPE1", 4 ) == 0 ) )   // or artist?
      {
        albttl += String ( metalinebf + 1 ) ;               // Yes, add to string
        if ( displaytype == T_NEXTION )                     // NEXTION display?
        {
          albttl += String ( "\\r" ) ;                      // Add code for newline (2 characters)
        }
        else
        {
          albttl += String ( "\n" ) ;                       // Add newline (1 character)
        }
      }
      if ( strncmp ( ID3tag.tagid, "TIT2", 4 ) == 0 )       // Songtitle?
      {
        tftset ( 2, metalinebf + 1 ) ;                      // Yes, show title
      }
    }
    tftset ( 1, albttl ) ;                                  // Show album and title
  }
  mp3file.close() ;                                         // Close the file
  mp3file = SD.open ( path ) ;                              // And open the file again
}


//**************************************************************************************************
//                                       C O N N E C T T O F I L E                                 *
//**************************************************************************************************
// Open the local mp3-file.                                                                        *
//**************************************************************************************************
bool connecttofile()
{
  String path ;                                           // Full file spec

  stop_mp3client() ;                                      // Disconnect if still connected
  tftset ( 0, "ESP32 MP3 Player" ) ;                      // Set screen segment top line
  displaytime ( "" ) ;                                    // Clear time on TFT screen
  datamode = DATA ;                                       // Assume to start in datamode
  if ( host.endsWith ( ".m3u" ) )                         // Is it an m3u playlist?
  {
    playlist = host ;                                     // Save copy of playlist URL
    datamode = PLAYLISTINIT ;                             // Yes, start in PLAYLIST mode
    if ( playlist_num == 0 )                              // First entry to play?
    {
      playlist_num = 1 ;                                  // Yes, set index
    }
    dbgprint ( "Playlist request, entry %d", playlist_num ) ;
  }
  if ( mp3file )
  {
    while ( mp3file.available() )
    {
      mp3file.read() ;
    }
    dbgprint ( "Closed SD file" ) ;                       // TEST*TEST*TEST
    mp3file.close() ;                                     // Be sure to close current file
  }
  path = host.substring ( 9 ) ;                           // Path, skip the "localhost" part
  claimSPI ( "sdopen3" ) ;                                // Claim SPI bus
  handle_ID3 ( path ) ;                                   // See if there are ID3 tags in this file
  releaseSPI() ;                                          // Release SPI bus
  if ( !mp3file )
  {
    dbgprint ( "Error opening file %s", path.c_str() ) ;  // No luck
    return false ;
  }
  mp3filelength = mp3file.available() ;                   // Get length
  //mqttpub.trigger ( MQTT_STREAMTITLE ) ;                  // Request publishing to MQTT
  //icyname = "" ;                                          // No icy name yet
  chunked = false ;                                       // File not chunked
  metaint = 0 ;                                           // No metadata
  return true ;
}
//**************************************************************************************************
//                                       C O N N E C T LAN                                         *
//**************************************************************************************************
static bool eth_connected = false;
uint8_t state_lan = 0;
int count = 0;
int LAN_IP_STATE = 0;
void WiFiEvent(WiFiEvent_t event) {
char*      pfs ;                                      // Pointer to formatted string
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      LAN_IP_STATE = 1;
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      ipaddress =ETH.localIP().toString() ;             // Form IP address
      dsp_ip =  ipaddress;
      dsp_signal = "SIGNAL:100Mbs";
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      state_lan = 1;
      NetworkFound = true;
      WiFi.disconnect(true) ;                               // After restart the router could
      WiFi.softAPdisconnect(true) ;
      if (sts_4g == true)
      {
//        Serial.println("Mode led: 1");
        mode_led = 1;
      }
      
      else
      {
//        Serial.println("Mode led: 4");
        mode_led = 3;
      }
    
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      dsp_ip = "0.0.0.0";
      dsp_signal = "SIGNAL:0Mbs";
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }

}

//**************************************************************************************************
//                                       C O N N E C T W I F I                                     *
//**************************************************************************************************
// Connect to WiFi using the SSID's available in wifiMulti.                                        *
// If only one AP if found in preferences (i.e. wifi_00) the connection is made without            *
// using wifiMulti.                                                                                *
// If connection fails, an AP is created and the function returns false.                           *
//************************************************************************************************
bool status_wifi = false;
bool connectwifi()
{
  char*      pfs ;                                      // Pointer to formatted string
  char*      pfs2 ;                                     // Pointer to formatted string
  bool       localAP = false ;                          // True if only local AP is left
  int        count_try =0 ;
  WifiInfo_t winfo ;                                    // Entry from wifilist

  WiFi.disconnect ( true ) ;                            // After restart the router could
  WiFi.softAPdisconnect ( true ) ;                      // still keep the old connection
  vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;            // Silly things to start connection
  WiFi.mode ( WIFI_STA ) ;
  vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;
  if ( wifilist.size()  )                               // Any AP defined?
  {
    if ( wifilist.size() == 1 )                         // Just one AP defined in preferences?
    {
      winfo = wifilist[0] ;                             // Get this entry
      WiFi.begin ( winfo.ssid, winfo.passphrase ) ;     // Connect to single SSID found in wifi_xx
      dbgprint ( "Try WiFi %s", winfo.ssid ) ;          // Message to show during WiFi connect
    }
    else                                                // More AP to try
    {
      wifiMulti.run() ;                                 // Connect to best network
    }
    while (  WiFi.waitForConnectResult() != WL_CONNECTED ) // Try to connect
    {
      delay(500);
      Serial.print(".");                                 // Error, setup own AP
      count_try++;
      if ( count_try > 1)
      {
        localAP = true ;
        break;
      }
    }
  }
  else
  {
    localAP = true ;                                    // Not even a single AP defined
  }
  if ( localAP )                                        // Must setup local AP?
  {
      digitalWrite(power_4g,HIGH);
      Serial.println("Bat module 4G");
      sts_4g  = true;
      
  }
  else
  {
    ipaddress = WiFi.localIP().toString() ;             // Form IP address
    mode_led = 2;
//    Serial.println ("Mode led: 2");
    status_wifi = true;
    eth_connected = true;
    pfs2 = dbgprint ( "Connected to %s", WiFi.SSID().c_str() ) ;
    tftlog ( pfs2 ) ;
    pfs = dbgprint ( "IP = %s", ipaddress.c_str() ) ;   // String to dispay on TFT
  }
  tftlog ( pfs ) ;                                      // Show IP
  delay ( 3000 ) ;                                      // Allow user to read this
  tftlog ( "\f" ) ;                                     // Select new page if NEXTION
  return ( localAP == false ) ;                         // Return result of connection
}


//**************************************************************************************************
//                                           O T A S T A R T                                       *
//**************************************************************************************************
// Update via WiFi has been started by Arduino IDE or update request.                              *
//**************************************************************************************************
void otastart()
{
  char* p ;

  p = dbgprint ( "OTA update Started" ) ;
  tftset ( 2, p ) ;                                   // Set screen segment bottom part
}


//**************************************************************************************************
//                                D O _ N E X T I O N _ U P D A T E                                *
//**************************************************************************************************
// Update NEXTION image from OTA stream.                                                           *
//**************************************************************************************************
bool do_nextion_update ( uint32_t clength )
{
  bool     res = false ;                                       // Update result
  uint32_t k ;
  int      c ;                                                 // Reply from NEXTION

  if ( nxtserial )                                             // NEXTION active?
  {
    vTaskDelete ( xspftask ) ;                                 // Prevent output to NEXTION
    delay ( 1000 ) ;
    nxtserial->printf ( "\xFF\xFF\xFF" ) ;                     // Empty command
    for ( int i = 0 ; i < 100 ; i++ )                          // Any input seen?
    {
      if ( nxtserial->available() )
      {
        c =  nxtserial->read() ;                               // Read garbage
      }
      delay ( 20 ) ;
    }
    nxtserial->printf ( "whmi-wri %d,9600,0\xFF\xFF\xFF",      // Start upload
                        clength ) ;
    while ( !nxtserial->available() )                          // Any input seen?
    {
      delay ( 20 ) ;
    }
    c =  nxtserial->read() ;                                   // Yes, read the 0x05 ACK
    while ( clength )                                          // Loop for the transfer
    {
      k = clength ;
      if ( k > 4096 )
      {
        k = 4096 ;
      }
      k = otaclient.read ( tmpbuff, k ) ;                      // Read a number of bytes from the stream
      dbgprint ( "TFT file, read %d bytes", k ) ;
      nxtserial->write ( tmpbuff, k ) ;
      while ( !nxtserial->available() )                        // Any input seen?
      {
        delay ( 20 ) ;
      }
      c =  (char)nxtserial->read() ;                           // Yes, read the 0x05 ACK
      if ( c != 0x05 )
      {
        break ;
      }
      clength -= k ;
    }
    otaclient.flush() ;
    if ( clength == 0 )
    {
      dbgprint ( "Update successfully completed" ) ;
      res = true ;
    }
  }
  return res ;
}


//**************************************************************************************************
//                                D O _ S O F T W A R E _ U P D A T E                              *
//**************************************************************************************************
// Update software from OTA stream.                                                                *
//**************************************************************************************************
bool do_software_update ( uint32_t clength )
{
  bool res = false ;                                          // Update result

  if ( Update.begin ( clength ) )                             // Update possible?
  {
    dbgprint ( "Begin OTA update, length is %d",
               clength ) ;
    if ( Update.writeStream ( otaclient ) == clength )        // writeStream is the real download
    {
      dbgprint ( "Written %d bytes successfully", clength ) ;
    }
    else
    {
      dbgprint ( "Write failed!" ) ;
    }
    if ( Update.end() )                                       // Check for successful flash
    {
      dbgprint( "OTA done" ) ;
      if ( Update.isFinished() )
      {
        dbgprint ( "Update successfully completed" ) ;
        res = true ;                                          // Positive result
      }
      else
      {
        dbgprint ( "Update not finished!" ) ;
      }
    }
    else
    {
      dbgprint ( "Error Occurred. Error %s", Update.getError() ) ;
    }
  }
  else
  {
    // Not enough space to begin OTA
    dbgprint ( "Not enough space to begin OTA" ) ;
    otaclient.flush() ;
  }
  return res ;
}


//**************************************************************************************************
//                                        U P D A T E _ S O F T W A R E                            *
//**************************************************************************************************
// Update software by download from remote host.                                                   *
//**************************************************************************************************
void update_software ( const char* lstmodkey, const char* updatehost, const char* binfile )
{
  uint32_t    timeout = millis() ;                              // To detect time-out
  String      line ;                                            // Input header line
  String      lstmod = "" ;                                     // Last modified timestamp in NVS
  String      newlstmod ;                                       // Last modified from host

  updatereq = false ;                                           // Clear update flag
  otastart() ;                                                  // Show something on screen
  stop_mp3client () ;                                           // Stop input stream
  lstmod = nvsgetstr ( lstmodkey ) ;                            // Get current last modified timestamp
  dbgprint ( "Connecting to %s for %s",
             updatehost, binfile ) ;
  if ( !otaclient.connect ( updatehost, 80 ) )                  // Connect to host
  {
    dbgprint ( "Connect to updatehost failed!" ) ;
    return ;
  }
  otaclient.printf ( "GET %s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Cache-Control: no-cache\r\n"
                     "Connection: close\r\n\r\n",
                     binfile,
                     updatehost ) ;
  while ( otaclient.available() == 0 )                          // Wait until response appears
  {
    if ( millis() - timeout > 5000 )
    {
      dbgprint ( "Connect to Update host Timeout!" ) ;
      otaclient.stop() ;
      return ;
    }
  }
  // Connected, handle response
  while ( otaclient.available() )
  {
    line = otaclient.readStringUntil ( '\n' ) ;                 // Read a line from response
    line.trim() ;                                               // Remove garbage
    dbgprint ( line.c_str() ) ;                                 // Debug info
    if ( !line.length() )                                       // End of headers?
    {
      break ;                                                   // Yes, get the OTA started
    }
    // Check if the HTTP Response is 200.  Any other response is an error.
    if ( line.startsWith ( "HTTP/1.1" ) )                       //
    {
      if ( line.indexOf ( " 200 " ) < 0 )
      {
        dbgprint ( "Got a non 200 status code from server!" ) ;
        return ;
      }
    }
    scan_content_length ( line.c_str() ) ;                      // Scan for content_length
    if ( line.startsWith ( "Last-Modified: " ) )                // Timestamp of binary file
    {
      newlstmod = line.substring ( 15 ) ;                       // Isolate timestamp
    }
  }
  if ( newlstmod == lstmod )                                    // Need for update?
  {
    dbgprint ( "No new version available" ) ;                   // No, show reason
    otaclient.flush() ;
    return ;
  }
  if ( clength > 0 )
  {
    if ( strstr ( binfile, ".bin" ) )                           // Update of the sketch?
    {
      if ( do_software_update ( clength ) )                     // Flash updated sketch
      {
        nvssetstr ( lstmodkey, newlstmod ) ;                    // Update Last Modified in NVS
      }
    }
    if ( strstr ( binfile, ".tft" ) )                           // Update of the NEXTION image?
    {
      if ( do_nextion_update ( clength ) )                      // Flash updated NEXTION
      {
        nvssetstr ( lstmodkey, newlstmod ) ;                    // Update Last Modified in NVS
      }
    }
  }
  else
  {
    dbgprint ( "There was no content in the response" ) ;
    otaclient.flush() ;
  }
}
  // End of headers reached
//   if ( newlstmod == lstmod )                                    // Need for update?
//   {
//     count_update = 1;
//     nvssetstr ("count_update", String(count_update));
//     dbgprint ( "No new version available" ) ;                   // No, show reason
//     otaclient.flush() ;
//     return ;
//   }
//   if ( clength != otc )                                    // Need for update?
//   {
//     count_update = 1;
//     nvssetstr ("count_update", String(count_update));
//     dbgprint ( "Size file doe not fit" ) ;                   // No, show reason
//     otaclient.flush() ;
//     return ;
//   }
//   if ( clength == otc )
//   {
//     if ( strstr ( binfile, ".bin" ) )                           // Update of the sketch?
//     {
//       if ( do_software_update ( clength ) )                     // Flash updated sketch
//       {
//         nvssetstr ( lstmodkey, newlstmod ) ;                    // Update Last Modified in NVS
//       }
//     }
//     if ( strstr ( binfile, ".tft" ) )                           // Update of the NEXTION image?
//     {
//       if ( do_nextion_update ( clength ) )                      // Flash updated NEXTION
//       {
//         nvssetstr ( lstmodkey, newlstmod ) ;                    // Update Last Modified in NVS
//       }
//     }
//     count_update = 0;
//     nvssetstr ("count_update", String(count_update));
//     ota = 0;
//     nvssetstr ("ota", String (ota));
//   }
//   else
//   {
//     dbgprint ( "There was no content in the response" ) ;
//     otaclient.flush() ;
//   }
// }


//**************************************************************************************************
//                                  R E A D H O S T F R O M P R E F                                *
//**************************************************************************************************
// Read the mp3 host from the preferences specified by the parameter.                              *
// The host will be returned.                                                                      *
// We search for "preset_x" or "preset_xx" or "preset_xxx".                       *
//**************************************************************************************************
String readhostfrompref ( int16_t preset )
{
  char           tkey[12] ;                            // Key as an array of char

  sprintf ( tkey, "preset_%d", preset ) ;              // Form the search key
  if ( !nvssearch ( tkey ) )                           // Does _x[x[x]] exists?
  {
    sprintf ( tkey, "preset_%03d", preset ) ;          // Form new search key
    if ( !nvssearch ( tkey ) )                         // Does _xxx exists?
    {
      sprintf ( tkey, "preset_%02d", preset ) ;        // Form new search key
    }
    if ( !nvssearch ( tkey ) )                         // Does _xx exists?
    {
      return String ( "" ) ;                           // Not found
    }
  }
  // Get the contents
  return nvsgetstr ( tkey ) ;                          // Get the station (or empty sring)
}


//**************************************************************************************************
//                                  R E A D H O S T F R O M P R E F                                *
//**************************************************************************************************
// Search for the next mp3 host in preferences specified newpreset.                                *
// The host will be returned.  newpreset will be updated                                           *
//**************************************************************************************************
String readhostfrompref()
{
  String contents = "" ;                                // Result of search
  int    maxtry = 0 ;                                   // Limit number of tries
  //   ini_block.newpreset--;
  while ( ( contents = readhostfrompref ( ini_block.newpreset ) ) == "" )
  {
    if ( ++maxtry >= MAXPRESETS )
    {
      return "" ;
    }
    if ( ++ini_block.newpreset >= MAXPRESETS )          // Next or wrap to 0
    {
      ini_block.newpreset = 0 ;
    }
  }
  // Get the contents
  return contents ;                                     // Return the station
}


//**************************************************************************************************
//                                       R E A D P R O G B U T T O N S                             *
//**************************************************************************************************
// Read the preferences for the programmable input pins and the touch pins.                        *
//**************************************************************************************************
void readprogbuttons()
{
  char        mykey[20] ;                                   // For numerated key
  int8_t      pinnr ;                                       // GPIO pinnumber to fill
  int         i ;                                           // Loop control
  String      val ;                                         // Contents of preference entry

  for ( i = 0 ; ( pinnr = progpin[i].gpio ) >= 0 ; i++ )    // Scan for all programmable pins
  {
    sprintf ( mykey, "gpio_%02d", pinnr ) ;                 // Form key in preferences
    if ( nvssearch ( mykey ) )
    {
      val = nvsgetstr ( mykey ) ;                           // Get the contents
      if ( val.length() )                                   // Does it exists?
      {
        if ( !progpin[i].reserved )                         // Do not use reserved pins
        {
          progpin[i].avail = true ;                         // This one is active now
          progpin[i].command = val ;                        // Set command
          dbgprint ( "gpio_%02d will execute %s",           // Show result
                     pinnr, val.c_str() ) ;
        }
      }
    }
  }
  // Now for the touch pins 0..9, identified by their GPIO pin number
  for ( i = 0 ; ( pinnr = touchpin[i].gpio ) >= 0 ; i++ )   // Scan for all programmable pins
  {
    sprintf ( mykey, "touch_%02d", i ) ;                    // Form key in preferences
    if ( nvssearch ( mykey ) )
    {
      val = nvsgetstr ( mykey ) ;                           // Get the contents
      if ( val.length() )                                   // Does it exists?
      {
        if ( !touchpin[i].reserved )                        // Do not use reserved pins
        {
          touchpin[i].avail = true ;                        // This one is active now
          touchpin[i].command = val ;                       // Set command
          //pinMode ( touchpin[i].gpio,  INPUT ) ;          // Free floating input
          dbgprint ( "touch_%02d will execute %s",          // Show result
                     i, val.c_str() ) ;
          dbgprint ( "Level is now %d",
                     touchRead ( pinnr ) ) ;                // Sample the pin
        }
        else
        {
          dbgprint ( "touch_%02d pin (GPIO%02d) is reserved for I/O!",
                     i, pinnr ) ;
        }
      }
    }
  }
}


//**************************************************************************************************
//                                       R E S E R V E P I N                                       *
//**************************************************************************************************
// Set I/O pin to "reserved".                                                                      *
// The pin is than not available for a programmable function.                                      *
//**************************************************************************************************
void reservepin ( int8_t rpinnr )
{
  uint8_t i = 0 ;                                           // Index in progpin/touchpin array
  int8_t  pin ;                                             // Pin number in progpin array

  while ( ( pin = progpin[i].gpio ) >= 0 )                  // Find entry for requested pin
  {
    if ( pin == rpinnr )                                    // Entry found?
    {
      if ( progpin[i].reserved )                            // Already reserved?
      {
        dbgprint ( "Pin %d is already reserved!", rpinnr ) ;
      }
      //dbgprint ( "GPIO%02d unavailabe for 'gpio_'-command", pin ) ;
      progpin[i].reserved = true ;                          // Yes, pin is reserved now
      break ;                                               // No need to continue
    }
    i++ ;                                                   // Next entry
  }
  // Also reserve touchpin numbers
  i = 0 ;
  while ( ( pin = touchpin[i].gpio ) >= 0 )                 // Find entry for requested pin
  {
    if ( pin == rpinnr )                                    // Entry found?
    {
      //dbgprint ( "GPIO%02d unavailabe for 'touch'-command", pin ) ;
      touchpin[i].reserved = true ;                         // Yes, pin is reserved now
      break ;                                               // No need to continue
    }
    i++ ;                                                   // Next entry
  }
}


//**************************************************************************************************
//                                       R E A D I O P R E F S                                     *
//**************************************************************************************************
// Scan the preferences for IO-pin definitions.                                                    *
//**************************************************************************************************
void readIOprefs()
{
  struct iosetting
  {
    const char* gname ;                                   // Name in preferences
    int8_t*     gnr ;                                     // GPIO pin number
    int8_t      pdefault ;                                // Default pin
  };
  struct iosetting klist[] = {                            // List of I/O related keys
     { "pin_ir",        &ini_block.ir_pin,          -1 },
    { "pin_enc_clk",   &ini_block.enc_clk_pin,      -1 },
    { "pin_enc_dt",    &ini_block.enc_dt_pin,       -1 },
    { "pin_enc_sw",    &ini_block.enc_sw_pin,       -1 },
    { "pin_tft_cs",    &ini_block.tft_cs_pin,       -1 }, // Display SPI version
    { "pin_tft_dc",    &ini_block.tft_dc_pin,       -1 }, // Display SPI version
    { "pin_tft_scl",   &ini_block.tft_scl_pin,      -1 }, // Display I2C version
    { "pin_tft_sda",   &ini_block.tft_sda_pin,      -1 }, // Display I2C version
    { "pin_tft_bl",    &ini_block.tft_bl_pin,       -1 }, // Display backlight
    { "pin_tft_blx",   &ini_block.tft_blx_pin,      -1 }, // Display backlight (inversed logic)
    { "pin_sd_cs",     &ini_block.sd_cs_pin,        -1 },
    { "pin_vs_cs",     &ini_block.vs_cs_pin,        13 },
    { "pin_vs_dcs",    &ini_block.vs_dcs_pin,       32 },
    { "pin_vs_dreq",   &ini_block.vs_dreq_pin,      36 },
    { "pin_shutdown",  &ini_block.vs_shutdown_pin,  -1 }, // Amplifier shut-down pin
    { "pin_shutdownx", &ini_block.vs_shutdownx_pin, -1 }, // Amplifier shut-down pin (inversed logic)
    { "pin_spi_sck",   &ini_block.spi_sck_pin,      18 },
    { "pin_spi_miso",  &ini_block.spi_miso_pin,     33 },
    { "pin_spi_mosi",  &ini_block.spi_mosi_pin,     23 },
    { NULL,            NULL,                        0  }  // End of list
  } ;
  int         i ;                                         // Loop control
  int         count = 0 ;                                 // Number of keys found
  String      val ;                                       // Contents of preference entry
  int8_t      ival ;                                      // Value converted to integer
  int8_t*     p ;                                         // Points to variable

  for ( i = 0 ; klist[i].gname ; i++ )                    // Loop trough all I/O related keys
  {
    p = klist[i].gnr ;                                    // Point to target variable
    ival = klist[i].pdefault ;                            // Assume pin number to be the default
    if ( nvssearch ( klist[i].gname ) )                   // Does it exist?
    {
      val = nvsgetstr ( klist[i].gname ) ;                // Read value of key
      if ( val.length() )                                 // Parameter in preference?
      {
        count++ ;                                         // Yes, count number of filled keys
        ival = val.toInt() ;                              // Convert value to integer pinnumber
        reservepin ( ival ) ;                             // Set pin to "reserved"
      }
    }
    *p = ival ;                                           // Set pinnumber in ini_block
    dbgprint ( "%s set to %d",                            // Show result
               klist[i].gname,
               ival ) ;
  }
}


//**************************************************************************************************
//                                       R E A D P R E F S                                         *
//**************************************************************************************************
// Read the preferences and interpret the commands.                                                *
// If output == true, the key / value pairs are returned to the caller as a String.                *
//**************************************************************************************************
String readprefs ( bool output )
{
  uint16_t    i ;                                           // Loop control
  String      val ;                                         // Contents of preference entry
  String      cmd ;                                         // Command for analyzCmd
  String      outstr = "" ;                                 // Outputstring
  char*       key ;                                         // Point to nvskeys[i]
  uint8_t     winx ;                                        // Index in wifilist
  uint16_t    last2char = 0 ;                               // To detect paragraphs

  i = 0 ;
  while ( *( key = nvskeys[i] ) )                           // Loop trough all available keys
  {
    val = nvsgetstr ( key ) ;                               // Read value of this key
    cmd = String ( key ) +                                  // Yes, form command
          String ( " = " ) +
          val ;
    if ( strstr ( key, "wifi_"  ) )                         // Is it a wifi ssid/password?
    {
      winx = atoi ( key + 5 ) ;                             // Get index in wifilist
      if ( ( winx < wifilist.size() ) &&                    // Existing wifi spec in wifilist?
           ( val.indexOf ( wifilist[winx].ssid ) == 0 ) )
      {
        val = String ( wifilist[winx].ssid ) +              // Yes, hide password
              String ( "/*******" ) ;
      }
      cmd = String ( "" ) ;                                 // Do not analyze this

    }
    else if ( strstr ( key, "mqttpasswd"  ) )               // Is it a MQTT password?
    {
      val = String ( "*******" ) ;                          // Yes, hide it
    }
    if ( output )
    {
      if ( ( i > 0 ) &&
           ( *(uint16_t*)key != last2char )  )               // New paragraph?
      {
        //        outstr += String ( "#\n" ) ;                        // Yes, add separator
      }
      last2char = *(uint16_t*)key ;                         // Save 2 chars for next compare
      String x = String (key);
      if ( strstr ( key, "dnm") || strstr ( key, "did") || strstr ( key, "epd") || strstr ( key, "reg") || strstr ( key, "ban") ||
           strstr ( key, "gps") || strstr ( key, "cpr") || strstr ( key, "cnm") || strstr ( key, "cip") || strstr ( key, "pmp") ||
           strstr ( key, "cpw") || strstr ( key, "frq") || strstr ( key, "mod") || strstr ( key, "ctp") ||
           strstr ( key, "sc1") || strstr ( key, "st1") || strstr ( key, "sl1") || strstr ( key, "sa1") ||
           strstr ( key, "sc2") || strstr ( key, "st2") || strstr ( key, "sl2") || strstr ( key, "sa2") ||
           strstr ( key, "sc3") || strstr ( key, "st3") || strstr ( key, "sl3") || strstr ( key, "sa3") ||
           strstr ( key, "sc4") || strstr ( key, "st4") || strstr ( key, "sl4") || strstr ( key, "sa4") ||
           strstr ( key, "err") || strstr ( key, "ert") || strstr ( key, "dlk") ||strstr ( key, "dlk")  || 
           strstr ( key, "ota") || strstr ( key, "otc") || strstr ( key, "debug")|| strstr ( key, "pr0")|| 
           strstr ( key, "pr1") || strstr ( key, "pr2") || strstr ( key, "pr3") || strstr ( key, "pr4") || 
           strstr ( key, "pr5") || strstr ( key, "pr6") || strstr ( key, "pr7") || strstr ( key, "pr8") || 
           strstr ( key, "pr9") || strstr ( key, "micvol")|| strstr ( key, "streamvol")
         )
      {

      }
      else
      {
        outstr += String ( key ) +                            // Add to outstr
                  String ( " = " ) +
                  val +
                  String ( "\n" ) ;                           // Add newline
      }
    }
    else
    {
      analyzeCmd ( cmd.c_str() ) ;                          // Analyze it
      analyzeCmd_manu_broker (cmd.c_str());
    }
    i++ ;                                                   // Next key
  }
  if ( i == 0 )
  {
    outstr = String ( "No preferences found.\n"
                      "Use defaults or run Esp32_radio_init first.\n" ) ;
  }
  return outstr ;
}


//**************************************************************************************************
//                                    M Q T T R E C O N N E C T                                    *
//**************************************************************************************************
// Reconnect to broker.                                                                            *
//**************************************************************************************************
bool mqttreconnect()
{
  static uint32_t retrytime = 0 ;                         // Limit reconnect interval
  bool            res = false ;                           // Connect resultresult
  char            clientid[20] ;                          // Client ID
  char            subtopic[60] ;                          // Topic to subscribe
  char            subtopic_mqttowner[60];

  if ( ( millis() - retrytime ) < 5000 )                  // Don't try to frequently
  {
    return res ;
  }
  retrytime = millis() ;                                  // Set time of last try
 if ( mqttcount > MAXMQTTCONNECTS )                      // Tried too much?
 {
   mqtt_on = false ;                                     // Yes, switch off forever
   resetreq =true;
   return res ;                                          // and quit
 }
 if (cam_mic == false) mqttcount++ ;                                           // Count the retries

 if (mqttcount % 5 == 0 && (dsp_ip == "0.0.0.0" || dsp_ip == ""))
 {
   vs1053player->begin() ; 
 }
 if (mqttcount > 12 && (dsp_ip == "0.0.0.0" || dsp_ip == ""))
 {
    resetreq =true;
 }
  dbgprint ( "(Re)connecting number %d to MQTT %s",       // Show some debug info
             mqttcount,
             ini_block.mqttbroker.c_str() ) ;
  sprintf ( clientid, "%s-%s-%04d",                          // Generate client ID
            NAME,ini_block.mqttprefix.c_str(), (int) random ( 10000 ) % 10000 ) ;
  res = mqttclient.connect ( clientid,                    // Connect to broker
                             ini_block.mqttuser.c_str(),
                             ini_block.mqttpasswd.c_str()
                           );
  if ( res )
  {
    mqttcount = 0;
    sprintf ( subtopic, "%s/%s",                          // Add prefix to subtopic
              ini_block.mqttprefix.c_str(),MQTT_SUBTOPIC ) ;
    res = mqttclient.subscribe ( subtopic ) ;             // Subscribe to MQTT
    if ( !res )
    {
      dbgprint ( "MQTT subscribe failed!" ) ;             // Failure
    }
    mqttpub.trigger ( MQTT_VOLUME ) ;                         // Publish own IP
  }
  else
  {
    dbgprint ( "MQTT connection failed, rc=%d",
               mqttclient.state() ) ;
  }
  return res ;
}
bool mqttreconnect1()
{
  static uint32_t retrytime = 0 ;                         // Limit reconnect interval
  bool            res1 = false ;                           // Connect result
  char            clientid[20] ;                          // Client ID
  char            subtopic[60] ;                          // Topic to subscribe

  if ( ( millis() - retrytime ) < 5000 )                  // Don't try to frequently
  {
    return res1 ;
  }
  retrytime = millis() ;                                  // Set time of last try
 if ( mqttcount_manu_broker > MAXMQTTCONNECTS )                      // Tried too much?
 {
   resetreq =true;
   mqtt_on_manu = false ;                                     // Yes, switch off forever
   return res1 ;                                          // and quit
 }
  //mqttcount_manu_broker++ ;                                           // Count the retries
  sprintf ( clientid, "%s-%04d",                          // Generate client ID
            NAME, (int) random ( 10000 ) % 10000 ) ;
  dbgprint ( "(Re)connecting number %d to MQTT %s",       // Show some debug info
             mqttcount_manu_broker,
             mqtt_ip_manufacture )  ;
  res1 = mqttclient1.connect ( clientid,                    // Connect to broker
                               ini_block.mqttuser.c_str(),
                               ini_block.mqttpasswd.c_str()
                             );
  if ( res1 )
  {
    sprintf ( subtopic, "%s/%s",ini_block.mqttprefix.c_str(),                          // Add prefix to subtopic
              MQTT_SUBTOPIC ) ;
    res1 = mqttclient1.subscribe ( subtopic ) ;             // Subscribe to MQTT

    if ( !res1 )
    {
      dbgprint ( "MQTT subscribe failed!" ) ;             // Failure
    }
  }
  else
  {
    dbgprint ( "MQTT connection failed, rc=%d",
               mqttclient.state() ) ;

  }
  return res1;
}

//**************************************************************************************************
//                                    O N M Q T T M E S S A G E                                    *
//**************************************************************************************************
// Executed when a subscribed message is received.                                                 *
// Note that message is not delimited by a '\0'.                                                   *
// Note that cmd buffer is shared with serial input.                                               *
// //**************************************************************************************************
// void onMqttMessage ( char* topic, byte* payload, unsigned int len )
// {
//   const char*  reply ;                                // Result from analyzeCmd

//   if ( strstr ( topic, MQTT_SUBTOPIC ) )              // Check on topic, maybe unnecessary
//   {
//     if ( len >= sizeof(cmd) )                         // Message may not be too long
//     {
//       len = sizeof(cmd) - 1 ;
//     }
//     strncpy ( cmd, (char*)payload, len ) ;            // Make copy of message
//     char tach_mang[130];
//     char tat_ca[100];
//     for (int  i = 15 ; i < len-1 ; i++)
//     {
//       tach_mang[i - 15] = cmd[i];
//     }
//     for (int  i = 9 ; i < len-1 ; i++)
//     {
//       tat_ca[i - 9] = cmd[i];
//     }
//     String cmd_str = String(cmd);
//     String id_sub = cmd_str.substring(0, 14);
//     String id_all = cmd_str.substring(0, 8);
//     if ( id_sub == ini_block.mqttprefix)
//     {
//       tach_mang[len - 16] = '\0' ;                               // Take care of delimeter
//       dbgprint ( "MQTT message arrived [%s], lenght = %d, %s", topic, (len - 16) , tach_mang ) ;
//       reply = analyzeCmd ( tach_mang ) ;                      // Analyze command and handle it
//       dbgprint ( reply ) ;                              // Result for debugging
//     }
//     if ( id_all == "[AAA555]")
//     {
//       tat_ca[len-8] = '\0' ;                                 // Take care of delimeter
//       dbgprint ( "MQTT message arrived [%s], lenght = %d, %s", topic, len , tat_ca ) ;
//       dbgprint ( reply ) ;                              // Result for debugging
//     }
//   }
// }
// void onMqttMessage1 ( char* topic, byte* payload, unsigned int len )
// {
//   const char*  reply ;                                // Result from analyzeCmd

//   if ( strstr ( topic, MQTT_SUBTOPIC) )              // Check on topic, maybe unnecessary
//   {
//     if ( len >= sizeof(cmd) )                         // Message may not be too long
//     {
//       len = sizeof(cmd) - 1 ;
//     }
//     strncpy ( cmd, (char*)payload, len ) ;            // Make copy of message
//     char tach_mang[130];
//     for (int  i = 15 ; i < len-1; i++)
//     {
//       tach_mang[i - 15] = cmd[i];                              
//     }
//     String cmd_str = String(cmd);
//     char   cmd_char[130];
//     String id_sub = cmd_str.substring(0, 14);
//     if ( id_sub == ini_block.mqttprefix)
//     {
//       tach_mang[len - 16] = '\0' ;                                // Take care of delimeter
//       dbgprint ( "MQTT message arrived [%s], lenght = %d, %s", topic, (len - 16) , tach_mang) ;
//       reply = analyzeCmd_manu_broker ( tach_mang ) ;             // Analyze command and handle it
//       dbgprint ( reply ) ;                                       // Result for debugging
//     }
//   }
// }
void onMqttMessage ( char* topic, byte* payload, unsigned int len )
{
  const char*  reply ;                                // Result from analyzeCmd

  if ( strstr ( topic, MQTT_SUBTOPIC ) )              // Check on topic, maybe unnecessary
  {
    if ( len >= sizeof(cmd) )                         // Message may not be too long
    {
      len = sizeof(cmd) - 1 ;
    }
    strncpy ( cmd, (char*)payload, len ) ;            // Make copy of message
    cmd[len] = '\0' ;                                 // Take care of delimeter
    dbgprint ( "MQTT message arrived [%s], lenght = %d, %s", topic, len, cmd ) ;
    reply = analyzeCmd ( cmd ) ;                      // Analyze command and handle it
    dbgprint ( reply ) ;                              // Result for debugging
  }
}
void onMqttMessage1 ( char* topic, byte* payload, unsigned int len )
{
  const char*  reply ;                                // Result from analyzeCmd

  if ( strstr ( topic, MQTT_SUBTOPIC ) )              // Check on topic, maybe unnecessary
  {
    if ( len >= sizeof(cmd) )                         // Message may not be too long
    {
      len = sizeof(cmd) - 1 ;
    }
    strncpy ( cmd, (char*)payload, len ) ;            // Make copy of message
    cmd[len] = '\0' ;                                 // Take care of delimeter
    dbgprint ( "MQTT message arrived [%s], lenght = %d, %s", topic, len, cmd ) ;
    reply = analyzeCmd_manu_broker ( cmd ) ;                      // Analyze command and handle it
    dbgprint ( reply ) ;                              // Result for debugging
  }
}


//**************************************************************************************************
//                                     S C A N S E R I A L                                         *
//**************************************************************************************************
// Listen to commands on the Serial inputline.                                                     *
//**************************************************************************************************
void scanserial()
{
  static String serialcmd ;                      // Command from Serial input
  char          c ;                              // Input character
  const char*   reply = "" ;                     // Reply string from analyzeCmd
  uint16_t      len ;                            // Length of input string

  while ( Serial.available() )                   // Any input seen?
  {
    c =  (char)Serial.read() ;                   // Yes, read the next input character
    //Serial.write ( c ) ;                       // Echo
    len = serialcmd.length() ;                   // Get the length of the current string
    if ( ( c == '\n' ) || ( c == '\r' ) )
    {
      if ( len )
      {
        strncpy ( cmd, serialcmd.c_str(), sizeof(cmd) ) ;
        if ( nxtserial )                         // NEXTION test possible?
        {
          if ( serialcmd.startsWith ( "N:" ) )   // Command meant to test Nextion display?
          {
            nxtserial->printf ( "%s\xFF\xFF\xFF", cmd + 2 ) ;
          }
        }
        reply = analyzeCmd ( cmd ) ;             // Analyze command and handle it
        reply = analyzeCmd_manu_broker ( cmd ) ;             // Analyze command and handle it
        dbgprint ( reply ) ;                     // Result for debugging
        serialcmd = "" ;                         // Prepare for new command
      }
    }
    if ( c >= ' ' )                              // Only accept useful characters
    {
      serialcmd += c ;                           // Add to the command
    }
    if ( len >= ( sizeof(cmd) - 2 )  )           // Check for excessive length
    {
      serialcmd = "" ;                           // Too long, reset
    }
  }
}


//**************************************************************************************************
//                                     S C A N S E R I A L 2                                       *
//**************************************************************************************************
// Listen to commands on the 2nd Serial inputline (NEXTION).                                       *
//**************************************************************************************************
void scanserial2()
{
  static String  serialcmd ;                       // Command from Serial input
  char           c ;                               // Input character
  const char*    reply = "" ;                      // Reply string from analyzeCmd
  uint16_t       len ;                             // Length of input string
  static uint8_t ffcount = 0 ;                     // Counter for 3 tmes "0xFF"

  if ( nxtserial )                                 // NEXTION active?
  {
    while ( nxtserial->available() )               // Yes, any input seen?
    {
      c =  (char)nxtserial->read() ;               // Yes, read the next input character
      len = serialcmd.length() ;                   // Get the length of the current string
      if ( c == 0xFF )                             // End of command?
      {
        if ( ++ffcount < 3 )                       // 3 times FF?
        {
          continue ;                               // No, continue to read
        }
        ffcount = 0 ;                              // For next command
        if ( len )
        {
          strncpy ( cmd, serialcmd.c_str(), sizeof(cmd) ) ;
          dbgprint ( "NEXTION command seen %02X %s",
                     cmd[0], cmd + 1 ) ;
          if ( cmd[0] == 0x70 )                    // Button pressed?
          {
            reply = analyzeCmd ( cmd + 1 ) ;       // Analyze command and handle it
            dbgprint ( reply ) ;                   // Result for debugging
          }
          serialcmd = "" ;                         // Prepare for new command
        }
      }
      else if ( c >= ' ' )                         // Only accept useful characters
      {
        serialcmd += c ;                           // Add to the command
      }
      if ( len >= ( sizeof(cmd) - 2 )  )           // Check for excessive length
      {
        serialcmd = "" ;                           // Too long, reset
      }
    }
  }
}


//**************************************************************************************************
//                                     S C A N D I G I T A L                                       *
//**************************************************************************************************
// Scan digital inputs.                                                                            *
//**************************************************************************************************
void  scandigital()
{
  static uint32_t oldmillis = 5000 ;                        // To compare with current time
  int             i ;                                       // Loop control
  int8_t          pinnr ;                                   // Pin number to check
  bool            level ;                                   // Input level
  const char*     reply ;                                   // Result of analyzeCmd
  int16_t         tlevel ;                                  // Level found by touch pin
  const int16_t   THRESHOLD = 30 ;                          // Threshold or touch pins

  if ( ( millis() - oldmillis ) < 100 )                     // Debounce
  {
    return ;
  }
  scanios++ ;                                               // TEST*TEST*TEST
  oldmillis = millis() ;                                    // 100 msec over
  for ( i = 0 ; ( pinnr = progpin[i].gpio ) >= 0 ; i++ )    // Scan all inputs
  {
    if ( !progpin[i].avail || progpin[i].reserved )         // Skip unused and reserved pins
    {
      continue ;
    }
    level = ( digitalRead ( pinnr ) == HIGH ) ;             // Sample the pin
    if ( level != progpin[i].cur )                          // Change seen?
    {
      progpin[i].cur = level ;                              // And the new level
      if ( !level )                                         // HIGH to LOW change?
      {
        dbgprint ( "GPIO_%02d is now LOW, execute %s",
                   pinnr, progpin[i].command.c_str() ) ;
        reply = analyzeCmd ( progpin[i].command.c_str() ) ; // Analyze command and handle it
        dbgprint ( reply ) ;                                // Result for debugging
      }
    }
  }
  // Now for the touch pins
  for ( i = 0 ; ( pinnr = touchpin[i].gpio ) >= 0 ; i++ )   // Scan all inputs
  {
    if ( !touchpin[i].avail || touchpin[i].reserved )       // Skip unused and reserved pins
    {
      continue ;
    }
    tlevel = ( touchRead ( pinnr ) ) ;                      // Sample the pin
    level = ( tlevel >= THRESHOLD ) ;                       // True if below threshold
    if ( level )                                            // Level HIGH?
    {
      touchpin[i].count = 0 ;                               // Reset count number of times
    }
    else
    {
      if ( ++touchpin[i].count < 3 )                        // Count number of times LOW
      {
        level = true ;                                      // Not long enough: handle as HIGH
      }
    }
    if ( level != touchpin[i].cur )                         // Change seen?
    {
      touchpin[i].cur = level ;                             // And the new level
      if ( !level )                                         // HIGH to LOW change?
      {
        dbgprint ( "TOUCH_%02d is now %d ( < %d ), execute %s",
                   pinnr, tlevel, THRESHOLD,
                   touchpin[i].command.c_str() ) ;
        reply = analyzeCmd ( touchpin[i].command.c_str() ); // Analyze command and handle it
        dbgprint ( reply ) ;                                // Result for debugging
      }
    }
  }
}


//**************************************************************************************************
//                                     S C A N I R                                                 *
//**************************************************************************************************
// See if IR input is available.  Execute the programmed command.                                  *
//**************************************************************************************************
void scanIR()
{
  char        mykey[20] ;                                   // For numerated key
  String      val ;                                         // Contents of preference entry
  const char* reply ;                                       // Result of analyzeCmd

  if ( ir_value )                                           // Any input?
  {
    sprintf ( mykey, "ir_%04X", ir_value ) ;                // Form key in preferences
    if ( nvssearch ( mykey ) )
    {
      val = nvsgetstr ( mykey ) ;                           // Get the contents
      dbgprint ( "IR code %04X received. Will execute %s",
                 ir_value, val.c_str() ) ;
      reply = analyzeCmd ( val.c_str() ) ;                  // Analyze command and handle it
      dbgprint ( reply ) ;                                  // Result for debugging
    }
    else
    {
      dbgprint ( "IR code %04X received, but not found in preferences!  Timing %d/%d",
                 ir_value, ir_0, ir_1 ) ;
    }
    ir_value = 0 ;                                          // Reset IR code received
  }
}


//**************************************************************************************************
//                                           M K _ L S A N                                         *
//**************************************************************************************************
// Make al list of acceptable networks in preferences.                                             *
// Will be called only once by setup().                                                            *
// The result will be stored in wifilist.                                                          *
// Not that the last found SSID and password are kept in common data.  If only one SSID is         *
// defined, the connect is made without using wifiMulti.  In this case a connection will           *
// be made even if de SSID is hidden.                                                              *
//**************************************************************************************************
void  mk_lsan()
{
  uint8_t     i ;                                        // Loop control
  char        key[10] ;                                  // For example: "wifi_03"
  String      buf ;                                      // "SSID/password"
  String      lssid, lpw ;                               // Last read SSID and password from nvs
  int         inx ;                                      // Place of "/"
  WifiInfo_t  winfo ;                                    // Element to store in list

  dbgprint ( "Create list with acceptable WiFi networks" ) ;
  for ( i = 0 ; i < 100 ; i++ )                          // Examine wifi_00 .. wifi_99
  {
    sprintf ( key, "wifi_%02d", i ) ;                    // Form key in preferences
    if ( nvssearch ( key  ) )                            // Does it exists?
    {
      buf = nvsgetstr ( key ) ;                          // Get the contents
      inx = buf.indexOf ( "/" ) ;                        // Find separator between ssid and password
      if ( inx > 0 )                                     // Separator found?
      {
        lpw = buf.substring ( inx + 1 ) ;                // Isolate password
        lssid = buf.substring ( 0, inx ) ;               // Holds SSID now
        dbgprint ( "Added %s to list of networks",
                   lssid.c_str() ) ;
        winfo.inx = i ;                                  // Create new element for wifilist ;
        winfo.ssid = strdup ( lssid.c_str() ) ;          // Set ssid of element
        winfo.passphrase = strdup ( lpw.c_str() ) ;
        wifilist.push_back ( winfo ) ;                   // Add to list
        wifiMulti.addAP ( winfo.ssid,                    // Add to wifi acceptable network list
                          winfo.passphrase ) ;
      }
    }
  }
  dbgprint ( "End adding networks" ) ; ////
}


//**************************************************************************************************
//                                     G E T R A D I O S T A T U S                                 *
//**************************************************************************************************
// Return preset-, tone- and volume status.                                                        *
// Included are the presets, the current station, the volume and the tone settings.                *
//**************************************************************************************************
String getradiostatus()
{
  char                pnr[3] ;                           // Preset as 2 character, i.e. "03"

  sprintf ( pnr, "%02d", ini_block.newpreset ) ;         // Current preset
  return String ( "preset=" ) +                          // Add preset setting
         String ( pnr ) +
         String ( "\nvolume=" ) +                        // Add volume setting
         String ( String ( ini_block.reqvol ) ) +
         String ( "\ntoneha=" ) +                        // Add tone setting HA
         String ( ini_block.rtone[0] ) +
         String ( "\ntonehf=" ) +                        // Add tone setting HF
         String ( ini_block.rtone[1] ) +
         String ( "\ntonela=" ) +                        // Add tone setting LA
         String ( ini_block.rtone[2] ) +
         String ( "\ntonelf=" ) +                        // Add tone setting LF
         String ( ini_block.rtone[3] ) ;
}


//**************************************************************************************************
//                                     G E T S E T T I N G S                                       *
//**************************************************************************************************
// Send some settings to the webserver.                                                            *
// Included are the presets, the current station, the volume and the tone settings.                *
//**************************************************************************************************
void getsettings()
{
  String              val ;                              // Result to send
  String              statstr ;                          // Station string
  int                 inx ;                              // Position of search char in line
  int16_t             i ;                                // Loop control, preset number

  for ( i = 0 ; i < MAXPRESETS ; i++ )                   // Max number of presets
  {
    statstr = readhostfrompref ( i ) ;                   // Get the preset from NVS
    if ( statstr != "" )                                 // Preset available?
    {
      // Show just comment if available.  Otherwise the preset itself.
      inx = statstr.indexOf ( "#" ) ;                    // Get position of "#"
      if ( inx > 0 )                                     // Hash sign present?
      {
        statstr.remove ( 0, inx + 1 ) ;                  // Yes, remove non-comment part
      }
      chomp ( statstr ) ;                                // Remove garbage from description
      dbgprint ( "statstr is %s", statstr.c_str() ) ;
      val += String ( "preset_" ) +
             String ( i ) +
             String ( "=" ) +
             statstr +
             String ( "\n" ) ;                           // Add delimeter
      if ( val.length() > 1000 )                         // Time to flush?
      {
        cmdclient.print ( val ) ;                        // Yes, send
        val = "" ;                                       // Start new string
      }
    }
  }
  val += getradiostatus() +                              // Add radio setting
         String ( "\n\n" ) ;                             // End of reply
  cmdclient.print ( val ) ;                              // And send
}


//**************************************************************************************************
//                                           T F T L O G                                           *
//**************************************************************************************************
// Log to TFT if enabled.                                                                          *
//**************************************************************************************************
void tftlog ( const char *str )
{
  if ( tft )                                           // TFT configured?
  {
    dsp_println ( str ) ;                              // Yes, show error on TFT
    dsp_update() ;                                     // To physical screen
  }
}


//**************************************************************************************************
//                                   F I N D N S I D                                               *
//**************************************************************************************************
// Find the namespace ID for the namespace passed as parameter.                                    *
//**************************************************************************************************
uint8_t FindNsID ( const char* ns )
{
  esp_err_t                 result = ESP_OK ;                 // Result of reading partition
  uint32_t                  offset = 0 ;                      // Offset in nvs partition
  uint8_t                   i ;                               // Index in Entry 0..125
  uint8_t                   bm ;                              // Bitmap for an entry
  uint8_t                   res = 0xFF ;                      // Function result

  while ( offset < nvs->size )
  {
    result = esp_partition_read ( nvs, offset,                // Read 1 page in nvs partition
                                  &nvsbuf,
                                  sizeof(nvsbuf) ) ;
    if ( result != ESP_OK )
    {
      dbgprint ( "Error reading NVS!" ) ;
      break ;
    }
    i = 0 ;
    while ( i < 126 )
    {

      bm = ( nvsbuf.Bitmap[i / 4] >> ( ( i % 4 ) * 2 ) ) ;    // Get bitmap for this entry,
      bm &= 0x03 ;                                            // 2 bits for one entry
      if ( ( bm == 2 ) &&
           ( nvsbuf.Entry[i].Ns == 0 ) &&
           ( strcmp ( ns, nvsbuf.Entry[i].Key ) == 0 ) )
      {
        res = nvsbuf.Entry[i].Data & 0xFF ;                   // Return the ID
        offset = nvs->size ;                                  // Stop outer loop as well
        break ;
      }
      else
      {
        if ( bm == 2 )
        {
          i += nvsbuf.Entry[i].Span ;                         // Next entry
        }
        else
        {
          i++ ;
        }
      }
    }
    offset += sizeof(nvs_page) ;                              // Prepare to read next page in nvs
  }
  return res ;
}


//**************************************************************************************************
//                            B U B B L E S O R T K E Y S                                          *
//**************************************************************************************************
// Bubblesort the nvskeys.                                                                         *
//**************************************************************************************************
void bubbleSortKeys ( uint16_t n )
{
  uint16_t i, j ;                                             // Indexes in nvskeys
  char     tmpstr[16] ;                                       // Temp. storage for a key

  for ( i = 0 ; i < n - 1 ; i++ )                             // Examine all keys
  {
    for ( j = 0 ; j < n - i - 1 ; j++ )                       // Compare to following keys
    {
      if ( strcmp ( nvskeys[j], nvskeys[j + 1] ) > 0 )        // Next key out of order?
      {
        strcpy ( tmpstr, nvskeys[j] ) ;                       // Save current key a while
        strcpy ( nvskeys[j], nvskeys[j + 1] ) ;               // Replace current with next key
        strcpy ( nvskeys[j + 1], tmpstr ) ;                   // Replace next with saved current
      }
    }
  }
}


//**************************************************************************************************
//                                      F I L L K E Y L I S T                                      *
//**************************************************************************************************
// File the list of all relevant keys in NVS.                                                      *
// The keys will be sorted.                                                                        *
//**************************************************************************************************
void fillkeylist()
{
  esp_err_t    result = ESP_OK ;                                // Result of reading partition
  uint32_t     offset = 0 ;                                     // Offset in nvs partition
  uint16_t     i ;                                              // Index in Entry 0..125.
  uint8_t      bm ;                                             // Bitmap for an entry
  uint16_t     nvsinx = 0 ;                                     // Index in nvskey table

  keynames.clear() ;                                            // Clear the list
  while ( offset < nvs->size )
  {
    result = esp_partition_read ( nvs, offset,                  // Read 1 page in nvs partition
                                  &nvsbuf,
                                  sizeof(nvsbuf) ) ;
    if ( result != ESP_OK )
    {
      dbgprint ( "Error reading NVS!" ) ;
      break ;
    }
    i = 0 ;
    while ( i < 126 )
    {
      bm = ( nvsbuf.Bitmap[i / 4] >> ( ( i % 4 ) * 2 ) ) ;      // Get bitmap for this entry,
      bm &= 0x03 ;                                              // 2 bits for one entry
      if ( bm == 2 )                                            // Entry is active?
      {
        if ( nvsbuf.Entry[i].Ns == namespace_ID )               // Namespace right?
        {
          strcpy ( nvskeys[nvsinx], nvsbuf.Entry[i].Key ) ;     // Yes, save in table
          if ( ++nvsinx == MAXKEYS )
          {
            nvsinx-- ;                                          // Prevent excessive index
          }
        }
        i += nvsbuf.Entry[i].Span ;                             // Next entry
      }
      else
      {
        i++ ;
      }
    }
    offset += sizeof(nvs_page) ;                                // Prepare to read next page in nvs
  }
  nvskeys[nvsinx][0] = '\0' ;                                   // Empty key at the end
  dbgprint ( "Read %d keys from NVS", nvsinx ) ;
  bubbleSortKeys ( nvsinx ) ;                                   // Sort the keys
}


//**************************************************************************************************
//                                           S E T U P                                             *
//**************************************************************************************************
// Setup for the program.                                                                          *
//**************************************************************************************************
void setup()
{
  int                       i ;                          // Loop control
  int                       pinnr ;                      // Input pinnumber
  const char*               p ;
  byte                      mac[6] ;                     // WiFi mac address
  char                      tmpstr[30] ;                 // For version and Mac address
  char                      tmpstr1[30] ;                 // For version and Mac address
  char                      tmpstr_pub[20] ;                 // For version and Mac address
  const char*               partname = "nvs" ;           // Partition with NVS info
  char                      random_key[6];
  uint8_t                   volume_d;
  uint32_t                  delay_lan=2000;
  char*                     pfs ;  
  static char               reply_signal[10];
  char buffer_mac[20];
  // For version and Mac address

  esp_partition_iterator_t  pi ;                         // Iterator for find
  const char*               dtyp = "Display type is %s" ;
  const char*               wvn = "Include file %s_html has the wrong version number! "
                                  "Replace header file." ;

  Serial.begin (115200) ;                              // For debug
  pinMode(3,INPUT_PULLUP);
 // pin_pullup();
  // Version tests for some vital include files
  if ( about_html_version   < 170626 ) dbgprint ( wvn, "about" ) ;
  if ( config_html_version  < 180806 ) dbgprint ( wvn, "config" ) ;
  if ( index_html_version   < 180102 ) dbgprint ( wvn, "index" ) ;
  if ( mp3play_html_version < 180918 ) dbgprint ( wvn, "mp3play" ) ;
  if ( defaultprefs_version < 180816 ) dbgprint ( wvn, "defaultprefs" ) ;
  // Print some memory and sketch info
  dbgprint ( "Starting ESP32-radio running on CPU %d at %d MHz.  Version %s.  Free memory %d",
             xPortGetCoreID(),
             ESP.getCpuFreqMHz(),
             VERSION,
             ESP.getFreeHeap() ) ;                       // Normally about 170 kB
//#if defined ( BLUETFT )                                // Report display option
//  dbgprint ( dtyp, "BLUETFT" ) ;
//#endif
//#if defined ( ILI9341 )                                // Report display option
//  dbgprint ( dtyp, "ILI9341" ) ;
//#endif
//#if defined ( OLED )
//  dbgprint ( dtyp, "OLED" ) ;
//#endif
//#if defined ( DUMMYTFT )
//  dbgprint ( dtyp, "DUMMYTFT" ) ;
//#endif
//#if defined ( LCD1602I2C )
//  dbgprint ( dtyp, "LCD1602" ) ;
//#endif
//#if defined ( NEXTION )
//  dbgprint ( dtyp, "NEXTION" ) ;
//#endif
  maintask = xTaskGetCurrentTaskHandle() ;               // My taskhandle
  SPIsem = xSemaphoreCreateMutex(); ;                    // Semaphore for SPI bus
  pi = esp_partition_find ( ESP_PARTITION_TYPE_DATA,     // Get partition iterator for
                            ESP_PARTITION_SUBTYPE_ANY,   // the NVS partition
                            partname ) ;
  if ( pi )
  {
    nvs = esp_partition_get ( pi ) ;                     // Get partition struct
    esp_partition_iterator_release ( pi ) ;              // Release the iterator
    dbgprint ( "Partition %s found, %d bytes",
               partname,
               nvs->size ) ;
  }
  else
  {
    dbgprint ( "Partition %s not found!", partname ) ;   // Very unlikely...
    while ( true ) ;                                     // Impossible to continue
  }
  namespace_ID = FindNsID ( NAME ) ;                     // Find ID of our namespace in NVS
  fillkeylist() ;                                        // Fill keynames with all keys
  memset ( &ini_block, 0, sizeof(ini_block) ) ;          // Init ini_block
  ini_block.mqttport = 1883 ;                            // Default port for MQTT
  ini_block.mqttuser = "vietbroadcast" ;
  ini_block.mqttpasswd = "Viet@123456" ;
  ini_block.mqttbroker = "1101c.vietbroadcast.vn";
  ini_block.mqttowner = "all";
  ini_block.clk_server = "pool.ntp.org" ;                // Default server for NTP
  ini_block.reqvol = 80;
  ini_block.clk_offset = 7 ;                             // Default Amsterdam time zone
  ini_block.clk_dst = 0 ;                                // DST is +1 hour
  ini_block.bat0 = 0 ;                                   // Battery ADC levels not yet defined
  ini_block.bat100 = 0 ;
  //  nvssetstr (count_update ,"0");
  readIOprefs() ;                                        // Read pins used for SPI, TFT, VS1053, IR,
  // Rotary encoder

  for ( i = 0 ; (pinnr = progpin[i].gpio) >= 0 ; i++ )   // Check programmable input pins
  {
    pinMode ( pinnr, INPUT_PULLUP ) ;                    // Input for control button
    delay ( 10 ) ;
    // Check if pull-up active
    if ( ( progpin[i].cur = digitalRead ( pinnr ) ) == HIGH )
    {
      p = "HIGH" ;
    }
    else
    {
      p = "LOW, probably no PULL-UP" ;                   // No Pull-up
    }
    dbgprint ( "GPIO%d is %s", pinnr, p ) ;
  }
  WiFi.macAddress ( mac ) ;                        // Get mac-adress
  sprintf ( key_mac, "%02X%02X%02X%02X%02X%02X",           // Generate string from last part
                  mac[0], mac[1],
                  mac[2], mac[3],
                  mac[4], mac[5]) ;
                  
        mac_id = String (key_mac);
        dsp_mac = "MAC:" + mac_id;
  ini_block.mqttprefix = String ( key_mac ) ;       // Save for further use

//  String auth = base64::encode (ini_block.mqttprefix);
//  Serial.println(auth);
  
  pinMode(led_do,OUTPUT);
  pinMode(led_do,LOW);
  pinMode(led_xanh,OUTPUT);
  digitalWrite(led_xanh,LEDState);
  pinMode(MUTE,OUTPUT);
  pinMode(power_4g,OUTPUT);
  
  sprintf (buffer_mac,"MAC=%s\n\r",ini_block.mqttprefix.c_str());
  sprintf (version_firmware,"%s .%s .%s",__FIRMWARE_NAME__,__FIRMWARE_VERSION__,__HARDWARE_VERSION__);
  ver = String(version_firmware);
  Serial.print(buffer_mac);
  //sprintf (ota_info,"OTF=%s",BINFILE);
  sprintf (ota_info,"%s",BINFILE);
  otf = String(ota_info);
  readprogbuttons() ;                                    // Program the free input pins
  SPI.begin ( ini_block.spi_sck_pin,                     // Init VSPI bus with default or modified pins
              ini_block.spi_miso_pin,
              ini_block.spi_mosi_pin ) ;
  vs1053player = new VS1053 ( ini_block.vs_cs_pin,       // Make instance of player
                              ini_block.vs_dcs_pin,
                              ini_block.vs_dreq_pin,
                              ini_block.vs_shutdown_pin,
                              ini_block.vs_shutdownx_pin ) ;
  if ( ini_block.ir_pin >= 0 )
  {
    dbgprint ( "Enable pin %d for IR",
               ini_block.ir_pin ) ;
    pinMode ( ini_block.ir_pin, INPUT ) ;                // Pin for IR receiver VS1838B
    attachInterrupt ( ini_block.ir_pin,                  // Interrupts will be handle by isr_IR
                      isr_IR, CHANGE ) ;
  }
  if ( ( ini_block.tft_cs_pin >= 0  ) ||                 // Display configured?
       ( ini_block.tft_scl_pin >= 0 ) )
  {
    dbgprint ( "Start display" ) ;
    if ( dsp_begin() )                                   // Init display
    {
      dsp_setRotation() ;                                // Use landscape format
      dsp_erase() ;                                      // Clear screen
      dsp_setTextSize ( 1 ) ;                            // Small character font
      dsp_setTextColor ( WHITE ) ;                       // Info in white
      dsp_setCursor ( 2, 3 ) ;                           // Top of screen
      dsp_print ( "Starting..." "\n" "Version:" ) ;
      strncpy ( tmpstr, VERSION, 16 ) ;                  // Limit version length
      dsp_println ( tmpstr ) ;
      dsp_println ( "By CTIN" ) ;
      dsp_update() ;                                     // Show on physical screen
    }
  }
  if ( ini_block.tft_bl_pin >= 0 )                       // Backlight for TFT control?
  {
    pinMode ( ini_block.tft_bl_pin, OUTPUT ) ;           // Yes, enable output
  }
  if ( ini_block.tft_blx_pin >= 0 )                      // Backlight for TFT (inversed logic) control?
  {
    pinMode ( ini_block.tft_blx_pin, OUTPUT ) ;          // Yes, enable output
  }
  blset ( true ) ;                                       // Enable backlight (if configured)
  if ( ini_block.sd_cs_pin >= 0 )                        // SD configured?
  {
    if ( !SD.begin ( ini_block.sd_cs_pin, SPI,           // Yes,
                     SDSPEED ) )                         // try to init SD card driver
    {
      p = dbgprint ( "SD Card Mount Failed!" ) ;         // No success, check formatting (FAT)
      tftlog ( p ) ;                                     // Show error on TFT as well
    }
    else
    {
      SD_okay = ( SD.cardType() != CARD_NONE ) ;         // See if known card
      if ( !SD_okay )
      {
        p = dbgprint ( "No SD card attached" ) ;         // Card not readable
        tftlog ( p ) ;                                   // Show error on TFT as well
      }
      else
      {
        dbgprint ( "Locate mp3 files on SD, may take a while..." ) ;
        tftlog ( "Read SD card" ) ;
        SD_nodecount = listsdtracks ( "/", 0, false ) ;  // Build nodelist
        p = dbgprint ( "%d tracks on SD", SD_nodecount ) ;
        tftlog ( p ) ;                                   // Show number of tracks on TFT
      }
    }
  }
  vs1053player->begin() ;                                // Initialize VS1053 player
  WiFi.onEvent(WiFiEvent);                               // in preferences.
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // For debug
  mk_lsan() ;                                            // Make a list of acceptable networks
  WiFi.mode ( WIFI_STA ) ;                               // This ESP is a station
  WiFi.persistent ( false ) ;                            // Do not save SSID and password
  WiFi.disconnect() ;                                    // After restart router could still
  delay ( 100 ) ;                                        // keep old connection
  listNetworks() ;                                       // Find WiFi networks
  DEBUG = 0;
  readprefs ( false ) ;                                  // Read preferences
  tcpip_adapter_set_hostname ( TCPIP_ADAPTER_IF_STA,
                               NAME ) ;

  delay(3000);
  volume_d = digitalRead(vol_down);
  if (volume_d== LOW)
     {
        dbgprint ( "WiFi Failed!  Trying to setup AP with name %s and password %s.", NAME, NAME ) ;
        WiFi.softAP ( NAME, NAME ) ;                        // This ESP will be an AP
        dbgprint ( "Start server for commands" ) ;
        mode_led = 0;
     }
   else
   {
     while(delay_lan--);
     if (!eth_connected )
      {
            NetworkFound = connectwifi() ;                         // Connect to WiFi network
       }
   }
  dbgprint ( "Start server for commands" ) ;
  cmdserver.begin() ;

//  WiFi.macAddress ( mac ) ;                        // Get mac-adress
//  sprintf ( key_mac, "%02X%02X%02X%02X%02X%02X",           // Generate string from last part
//                  mac[0], mac[1],
//                  mac[2], mac[3],
//                  mac[4], mac[5]) ;
//                  
//        mac_id = String (key_mac);
//        dsp_mac = "MAC:" + mac_id;
//  ini_block.mqttprefix = String ( key_mac ) ;       // Save for further use
//  if (eth_connected )                                    // OTA and MQTT only if Wifi network found
//  {
      if (state_lan == 1)
      {
        ipaddress =ETH.localIP().toString() ;             // Form IP address
        pfs = dbgprint ( "IP_LAN = %s", ipaddress.c_str() ) ;   // String to dispay on TFT
        dsp_ip = ipaddress;
        tftlog(pfs);
        ETHspeed = ETH.linkSpeed();
        pfs = dbgprint ("SPEED = %s Mbps",ETHspeed.c_str());
        tftlog(pfs);
        dsp_signal = "SIGNAL:100Mbs";
      }
      else if (state_lan == 2)
      {
        ipaddress = WiFi.localIP().toString() ;             // Form IP address
        dsp_ip =  ipaddress;
        sprintf(reply_signal,"%3d",WiFi.RSSI());
        dsp_signal = "SIGNAL:" + String (reply_signal) + " db" ;
      }

    dbgprint ( "Network found. Starting mqtt and OTA" ) ;
    mqtt_on = ( ini_block.mqttbroker.length() > 0 ) &&   // Use MQTT if broker specified
              ( ini_block.mqttbroker != "none" ) ;
    mqtt_on_manu = (mip.length() > 0);
    ArduinoOTA.setHostname ( NAME ) ;                    // Set the hostname
    ArduinoOTA.onStart ( otastart ) ;
    ArduinoOTA.begin() ;                                 // Allow update over the air
    if ( mqtt_on || mqtt_on_manu)                                       // Broker specified?
    {
        for  ( int i = 0; i < 6; i++)
        {
          random_key[i] = random(0x00, 0xff);
          test_key[i] = (random_key[i] + mac[i]) % 256;
          String stringOne = String (test_key[i], HEX);
          String stringOne1 = String (random_key[i], HEX);
          key_remote += stringOne1;
          sprintf ( tmpstr, "%02X%02X%02X%02X%02X%02X",           // Generate string from last part
                    test_key[0], test_key[1],
                    test_key[2], test_key[3],
                    test_key[4], test_key[5]) ;
          sprintf ( key_pub, "%02X%02X%02X%02X%02X%02X",           // Generate string from last part
                    random_key[0], random_key[1],
                    random_key[2], random_key[3],
                    random_key[4], random_key[5]) ;
          key_str = String ( key_pub);
         
          
        }
        Serial.print(key_remote);
        Serial.println("");
        sprintf ( rt_recive, "[%02X%02X%02X%02X%02X%02X][OK]",           // Generate string from last part
                  mac[0], mac[1],
                  mac[2], mac[3],
                  mac[4], mac[5]) ;
      mac_pub = WiFi.macAddress();
      dbgprint ( "MQTT uses prefix %s", ini_block.mqttprefix.c_str() ) ;
      dbgprint ( "Init MQTT" ) ;
      mqttclient.setServer(ini_block.mqttbroker.c_str(), // Specify the broker
                           ini_block.mqttport ) ;        // And the port
      mqttclient.setCallback ( onMqttMessage ) ;         // Set callback on receive
      mqttclient1.setServer(mqtt_ip_manufacture, mqtt_port_manufacture);          // Srecify the broker 2
      mqttclient1.setCallback( onMqttMessage1 ) ;         // Set callback on receive
    }
    if ( MDNS.begin ( NAME ) )                           // Start MDNS transponder
    {
      dbgprint ( "MDNS responder started" ) ;
    }
    else
    {
      dbgprint ( "Error setting up MDNS responder!" ) ;
    }
//  }
//  else
//  {
//    currentpreset = ini_block.newpreset ;                // No network: do not start radio
//  }
  timer = timerBegin ( 0, 80, true ) ;                   // User 1st timer with prescaler 80
  timerAttachInterrupt ( timer, &timer100, true ) ;      // Call timer100() on timer alarm
  timerAlarmWrite ( timer, 100000, true ) ;              // Alarm every 100 msec
  timerAlarmEnable ( timer ) ;                           // Enable the timer
  delay ( 1000 ) ;                                       // Show IP for a while
  configTime ( ini_block.clk_offset * 3600,
               ini_block.clk_dst * 3600,
               ini_block.clk_server.c_str() ) ;          // GMT offset, daylight offset in seconds
  timeinfo.tm_year = 0 ;                                 // Set TOD to illegal
  // Init settings for rotary switch (if existing).
  if ( ( ini_block.enc_clk_pin + ini_block.enc_dt_pin + ini_block.enc_sw_pin ) > 2 )
  {
    attachInterrupt ( ini_block.enc_clk_pin, isr_enc_turn,   CHANGE ) ;
    attachInterrupt ( ini_block.enc_dt_pin,  isr_enc_turn,   CHANGE ) ;
    attachInterrupt ( ini_block.enc_sw_pin,  isr_enc_switch, CHANGE ) ;
    dbgprint ( "Rotary encoder is enabled" ) ;
  }
  else
  {
    dbgprint ( "Rotary encoder is disabled (%d/%d/%d)",
               ini_block.enc_clk_pin,
               ini_block.enc_dt_pin,
               ini_block.enc_sw_pin) ;
  }
  if ( NetworkFound )
  {
    gettime() ;                                           // Sync time
  }

  if ( tft )
  {
    dsp_fillRect ( 0, 8,                                  // Clear most of the screen
                   dsp_getwidth(),
                   dsp_getheight() - 8, BLACK ) ;
  }
  outchunk.datatyp = QDATA ;                              // This chunk dedicated to QDATA
  adc1_config_width ( ADC_WIDTH_12Bit ) ;
  adc1_config_channel_atten ( ADC1_CHANNEL_3, ADC_ATTEN_0db ) ;
  dataqueue = xQueueCreate ( QSIZ,                        // Create queue for communication
                             sizeof ( qdata_struct ) ) ;
  xTaskCreatePinnedToCore (
    playtask,                                             // Task to play data in dataqueue.
    "Playtask",                                           // name of task.
    1600,                                                 // Stack size of task
    NULL,                                                 // parameter of the task
    2,                                                    // priority of the task
    &xplaytask,                                           // Task handle to keep track of created task
    0 ) ;                                                 // Run on CPU 0
  xTaskCreate (
    spftask,                                              // Task to handle special functions.
    "Spftask",                                            // name of task.
    2048,                                                 // Stack size of task
    NULL,                                                 // parameter of the task
    1,                                                    // priority of the task
    &xspftask ) ;                                         // Task handle to keep track of created task
}


//**************************************************************************************************
//                                        R I N B Y T                                              *
//**************************************************************************************************
// Read next byte from http inputbuffer.  Buffered for speed reasons.                              *
//**************************************************************************************************
uint8_t rinbyt ( bool forcestart )
{
  static uint8_t  buf[1024] ;                           // Inputbuffer
  static uint16_t i ;                                   // Pointer in inputbuffer
  static uint16_t len ;                                 // Number of bytes in buf
  uint16_t        tlen ;                                // Number of available bytes
  uint16_t        trycount = 0 ;                        // Limit max. time to read

  if ( forcestart || ( i == len ) )                     // Time to read new buffer
  {
    while ( cmdclient.connected() )                     // Loop while the client's connected
    {
      tlen = cmdclient.available() ;                    // Number of bytes to read from the client
      len = tlen ;                                      // Try to read whole input
      if ( len == 0 )                                   // Any input available?
      {
        if ( ++trycount > 3 )                           // Not for a long time?
        {
          dbgprint ( "HTTP input shorter than expected" ) ;
          return '\n' ;                                 // Error! No input
        }
        delay ( 10 ) ;                                  // Give communication some time
        continue ;                                      // Next loop of no input yet
      }
      if ( len > sizeof(buf) )                          // Limit number of bytes
      {
        len = sizeof(buf) ;
      }
      len = cmdclient.read ( buf, len ) ;               // Read a number of bytes from the stream
      i = 0 ;                                           // Pointer to begin of buffer
      break ;
    }
  }
  return buf[i++] ;
}


//**************************************************************************************************
//                                        W R I T E P R E F S                                      *
//**************************************************************************************************
// Update the preferences.  Called from the web interface.                                         *
//**************************************************************************************************
void writeprefs()
{
  //  readprefs();
  int        inx ;                                            // Position in inputstr
  uint8_t    winx ;                                           // Index in wifilist
  char       c ;                                              // Input character
  String     inputstr = "" ;                                  // Input regel
  String     key, contents ;                                  // Pair for Preferences entry
  String     dstr ;                                           // Contents for debug

  timerAlarmDisable ( timer ) ;                               // Disable the timer
  // nvsclear() ;                                                // Remove all preferences
  while ( true )
  {
    c = rinbyt ( false ) ;                                    // Get next inputcharacter
    if ( c == '\n' )                                          // Newline?
    {
      if ( inputstr.length() == 0 )
      {
        dbgprint ( "End of writing preferences" ) ;
        break ;                                               // End of contents
      }
      if ( !inputstr.startsWith ( "#" ) )                     // Skip pure comment lines
      {
        inx = inputstr.indexOf ( "=" ) ;
        if ( inx >= 0 )                                       // Line with "="?
        {
          key = inputstr.substring ( 0, inx ) ;               // Yes, isolate the key
          key.trim() ;
          contents = inputstr.substring ( inx + 1 ) ;         // and contents
          contents.trim() ;
          dstr = contents ;                                   // Copy for debug
          if ( ( key.indexOf ( "wifi_" ) == 0 ) )             // Sensitive info?
          {
            winx = key.substring(5).toInt() ;                 // Get index in wifilist
            if ( ( winx < wifilist.size() ) &&                // Existing wifi spec in wifilist?
                 ( contents.indexOf ( wifilist[winx].ssid ) == 0 ) &&
                 ( contents.indexOf ( "/****" ) > 0 ) )       // Hidden password?
            {
              contents = String ( wifilist[winx].ssid ) +     // Retrieve ssid and password
                         String ( "/" ) +
                         String ( wifilist[winx].passphrase ) ;
              dstr = String ( wifilist[winx].ssid ) +
                     String ( "/*******" ) ;                  // Hide in debug line
            }
          }
          if ( ( key.indexOf ( "mqttpasswd" ) == 0 ) )        // Sensitive info?
          {
            if ( contents.indexOf ( "****" ) == 0 )           // Hidden password?
            {
              contents = ini_block.mqttpasswd ;               // Retrieve mqtt password
            }
            dstr = String ( "*******" ) ;                     // Hide in debug line
          }
          dbgprint ( "writeprefs setstr %s = %s",
                     key.c_str(), dstr.c_str() ) ;
          nvssetstr ( key.c_str(), contents ) ;               // Save new pair
        }
      }
      inputstr = "" ;
    }
    else
    {
      if ( c != '\r' )                                        // Not newline.  Is is a CR?
      {
        inputstr += String ( c ) ;                            // No, normal char, add to string
      }
    }
  }
  timerAlarmEnable ( timer ) ;                                // Enable the timer
  fillkeylist() ;                                             // Update list with keys
}


//**************************************************************************************************
//                                        H A N D L E H T T P R E P L Y                            *
//**************************************************************************************************
// Handle the output after an http request.                                                        *
//**************************************************************************************************
void handlehttpreply()
{
  const char*   p ;                                         // Pointer to reply if command
  String        sndstr = "" ;                               // String to send
  int           n ;                                         // Number of files on SD card

  if ( http_response_flag )
  {
    http_response_flag = false ;
    if ( cmdclient.connected() )
    {
      if ( http_rqfile.length() == 0 &&                     // An empty "GET"?
           http_getcmd.length() == 0 )
      {
        if ( NetworkFound )                                 // Yes, check network
        {
          handleFSf ( String( "index.html") ) ;             // Okay, send the startpage
        }
        else
        {
          handleFSf ( String( "config.html") ) ;            // Or the configuration page if in AP mode
        }
      }
      else
      {
        if ( http_getcmd.length() )                         // Command to analyze?
        {
          dbgprint ( "Send reply for %s", http_getcmd.c_str() ) ;
          sndstr = httpheader ( String ( "text/html" ) ) ;  // Set header
          if ( http_getcmd.startsWith ( "getprefs" ) )      // Is it a "Get preferences"?
          {
            if ( datamode != STOPPED )                      // Still playing?
            {
              datamode = STOPREQD ;                         // Stop playing
            }
            sndstr += readprefs ( true ) ;                  // Read and send
          }
          else if ( http_getcmd.startsWith ( "getdefs" ) )  // Is it a "Get default preferences"?
          {
            sndstr += String ( defprefs_txt + 1 ) ;         // Yes, read initial values
          }
          else if ( http_getcmd.startsWith ("saveprefs") )  // Is is a "Save preferences"
          {
            writeprefs() ;                                  // Yes, handle it
            sndstr += String ( "Config saved" ) ;           // Give reply
          }
          else if ( http_getcmd.startsWith ( "mp3list" ) )  // Is is a "Get SD MP3 tracklist"?
          {
            if ( datamode != STOPPED )                      // Still playing?
            {
              datamode = STOPREQD ;                         // Stop playing
            }
            cmdclient.print ( sndstr ) ;                    // Yes, send header
            n = listsdtracks ( "/" ) ;                      // Handle it
            dbgprint ( "%d tracks found on SD card", n ) ;
            return ;                                        // Do not send empty line
          }
          else if ( http_getcmd.startsWith ( "settings" ) ) // Is is a "Get settings" (like presets and tone)?
          {
            cmdclient.print ( sndstr ) ;                    // Yes, send header
            getsettings() ;                                 // Handle settings request
            return ;                                        // Do not send empty line
          }
          else
          {
            p = analyzeCmd ( http_getcmd.c_str() ) ;        // Yes, do so
            sndstr += String ( p ) ;                        // Content of HTTP response follows the header
          }
          sndstr += String ( "\n" ) ;                       // The HTTP response ends with a blank line
          cmdclient.print ( sndstr ) ;
        }
        else if ( http_rqfile.length() )                    // File requested?
        {
          dbgprint ( "Start file reply for %s",
                     http_rqfile.c_str() ) ;
          handleFSf ( http_rqfile ) ;                       // Yes, send it
        }
        else
        {
          httpheader ( String ( "text/html" ) ) ;           // Send header
          // the content of the HTTP response follows the header:
          cmdclient.println ( "Dummy response\n" ) ;        // Text ending with double newline
          dbgprint ( "Dummy response sent" ) ;
        }
      }
    }
  }
}


//**************************************************************************************************
//                                        H A N D L E H T T P                                      *
//**************************************************************************************************
// Handle the input of an http request.                                                            *
//**************************************************************************************************
void handlehttp()
{
  bool        first = true ;                                 // First call to rinbyt()
  char        c ;                                            // Next character from http input
  int         inx0, inx ;                                    // Pos. of search string in currenLine
  String      currentLine = "" ;                             // Build up to complete line
  bool        reqseen = false ;                              // No GET seen yet

  if ( !cmdclient.connected() )                              // Action if client is connected
  {
    return ;                                                 // No client active
  }
  dbgprint ( "handlehttp started" ) ;
  while ( true )                                             // Loop till command/file seen
  {
    c = rinbyt ( first ) ;                                   // Get a byte
    first = false ;                                          // No more first call
    if ( c == '\n' )
    {
      // If the current line is blank, you got two newline characters in a row.
      // that's the end of the client HTTP request, so send a response:
      if ( currentLine.length() == 0 )
      {
        http_response_flag = reqseen ;                       // Response required or not
        break ;
      }
      else
      {
        // Newline seen, remember if it is like "GET /xxx?y=2&b=9 HTTP/1.1"
        if ( currentLine.startsWith ( "GET /" ) )            // GET request?
        {
          inx0 = 5 ;                                         // Start search at pos 5
        }
        else if ( currentLine.startsWith ( "POST /" ) )      // POST request?
        {
          inx0 = 6 ;
        }
        else
        {
          inx0 = 0 ;                                         // Not GET nor POST
        }
        if ( inx0 )                                          // GET or POST request?
        {
          reqseen = true ;                                   // Request seen
          inx = currentLine.indexOf ( "&" ) ;                // Search for 2nd parameter
          if ( inx < 0 )
          {
            inx = currentLine.indexOf ( " HTTP" ) ;          // Search for end of GET command
          }
          // Isolate the command
          http_getcmd = currentLine.substring ( inx0, inx ) ;
          inx = http_getcmd.indexOf ( "?" ) ;                // Search for command
          if ( inx == 0 )                                    // Arguments only?
          {
            http_getcmd = http_getcmd.substring ( 1 ) ;      // Yes, get rid of question mark
            http_rqfile = "" ;                               // No file
          }
          else if ( inx > 0 )                                // Filename present?
          {
            http_rqfile = http_getcmd.substring ( 0, inx ) ; // Remember filename
            http_getcmd = http_getcmd.substring ( inx + 1 ) ; // Remove filename from GET command
          }
          else
          {
            http_rqfile = http_getcmd ;                      // No parameters, set filename
            http_getcmd = "" ;
          }
          if ( http_getcmd.length() )
          {
            dbgprint ( "Get command is: %s",                 // Show result
                       http_getcmd.c_str() ) ;
          }
          if ( http_rqfile.length() )
          {
            dbgprint ( "Filename is: %s",                    // Show requested file
                       http_rqfile.c_str() ) ;
          }
        }
        currentLine = "" ;
      }
    }
    else if ( c != '\r' )                                    // No LINFEED.  Is it a CR?
    {
      currentLine += c ;                                     // No, add normal char to currentLine
    }
  }
  //cmdclient.stop() ;
}


//**************************************************************************************************
//                                          X M L P A R S E                                        *
//**************************************************************************************************
// Parses line with XML data and put result in variable specified by parameter.                    *
//**************************************************************************************************
void xmlparse ( String &line, const char *selstr, String &res )
{
  String sel = "</" ;                                  // Will be like "</status-code"
  int    inx ;                                         // Position of "</..." in line

  sel += selstr ;                                      // Form searchstring
  if ( line.endsWith ( sel ) )                         // Is this the line we are looking for?
  {
    inx = line.indexOf ( sel ) ;                       // Get position of end tag
    res = line.substring ( 0, inx ) ;                  // Set result
  }
}


//**************************************************************************************************
//                                          X M L G E T H O S T                                    *
//**************************************************************************************************
// Parses streams from XML data.                                                                   *
// Example URL for XML Data Stream:                                                                *
// http://playerservices.streamtheworld.com/api/livestream?version=1.5&mount=IHR_TRANAAC&lang=en   *
//**************************************************************************************************
String xmlgethost  ( String mount )
{
  const char* xmlhost = "playerservices.streamtheworld.com" ;  // XML data source
  const char* xmlget =  "GET /api/livestream"                  // XML get parameters
                        "?version=1.5"                         // API Version of IHeartRadio
                        "&mount=%sAAC"                         // MountPoint with Station Callsign
                        "&lang=en" ;                           // Language

  String   stationServer = "" ;                     // Radio stream server
  String   stationPort = "" ;                       // Radio stream port
  String   stationMount = "" ;                      // Radio stream Callsign
  uint16_t timeout = 0 ;                            // To detect time-out
  String   sreply = "" ;                            // Reply from playerservices.streamtheworld.com
  String   statuscode = "200" ;                     // Assume good reply
  char     tmpstr[200] ;                            // Full GET command, later stream URL
  String   urlout ;                                 // Result URL

  stop_mp3client() ; // Stop any current wificlient connections.
  dbgprint ( "Connect to new iHeartRadio host: %s", mount.c_str() ) ;
  datamode = INIT ;                                   // Start default in metamode
  chunked = false ;                                   // Assume not chunked
  sprintf ( tmpstr, xmlget, mount.c_str() ) ;         // Create a GET commmand for the request
  dbgprint ( "%s", tmpstr ) ;
  if ( mp3client.connect ( xmlhost, 80 ) )            // Connect to XML stream
  {
    dbgprint ( "Connected to %s", xmlhost ) ;
    mp3client.print ( String ( tmpstr ) + " HTTP/1.1\r\n"
                      "Host: " + xmlhost + "\r\n"
                      "User-Agent: Mozilla/5.0\r\n"
                      "Connection: close\r\n\r\n" ) ;
    while ( mp3client.available() == 0 )
    {
      delay ( 200 ) ;                                 // Give server some time
      if ( ++timeout > 25 )                           // No answer in 5 seconds?
      {
        dbgprint ( "Client Timeout !" ) ;
      }
    }
    dbgprint ( "XML parser processing..." ) ;
    while ( mp3client.available() )
    {
      sreply = mp3client.readStringUntil ( '>' ) ;
      sreply.trim() ;
      // Search for relevant info in in reply and store in variable
      xmlparse ( sreply, "status-code", statuscode ) ;
      xmlparse ( sreply, "ip",          stationServer ) ;
      xmlparse ( sreply, "port",        stationPort ) ;
      xmlparse ( sreply, "mount",       stationMount ) ;
      if ( statuscode != "200" )                      // Good result sofar?
      {
        dbgprint ( "Bad xml status-code %s",         // No, show and stop interpreting
                   statuscode.c_str() ) ;
        tmpstr[0] = '\0' ;                           // Clear result
        break ;
      }
    }
    if ( ( stationServer != "" ) &&                   // Check if all station values are stored
         ( stationPort != "" ) &&
         ( stationMount != "" ) )
    {
      sprintf ( tmpstr, "%s:%s/%s_SC",                // Build URL for ESP-Radio to stream.
                stationServer.c_str(),
                stationPort.c_str(),
                stationMount.c_str() ) ;
      dbgprint ( "Found: %s", tmpstr ) ;
    }
  }
  else
  {
    dbgprint ( "Can't connect to XML host!" ) ;       // Connection failed
    tmpstr[0] = '\0' ;
  }
  mp3client.stop() ;
  return String ( tmpstr ) ;                          // Return final streaming URL.
}


//**************************************************************************************************
//                                      H A N D L E S A V E R E Q                                  *
//**************************************************************************************************
// Handle save volume/preset/tone.  This will save current settings every 10 minutes to            *
// the preferences.  On the next restart these values will be loaded.                              *
// Note that saving prefences will only take place if contents has changed.                        *
//**************************************************************************************************
void handleSaveReq()
{
  static uint32_t savetime = 0 ;                          // Limit save to once per 10 minutes

  if ( ( millis() - savetime ) < 10000 )                 // 600 sec is 10 minutes
  {
    return ;
  }
  savetime = millis() ;                                   // Set time of last save
  nvssetstr ( "preset", String ( currentpreset )  ) ;     // Save current preset
  nvssetstr ( "volume", String ( ini_block.reqvol ) );    // Save current volue
  nvssetstr ( "volmic", String ( ini_block.micvol ) );    // Save current volue
  nvssetstr ( "volstream", String ( ini_block.streamvol ) );    // Save current volue

}


//**************************************************************************************************
//                                      H A N D L E I P P U B                                      *
//**************************************************************************************************
// Handle publish op IP to MQTT.  This will happen every 10 minutes.                               *
//**************************************************************************************************
void handleIpPub()
{
  static uint32_t pubtime = 300000 ;                       // Limit save to once per 10 minutes

  if ( ( millis() - pubtime ) < 600000 )                   // 600 sec is 10 minutes
  {
    return ;
  }
  pubtime = millis() ;                                     // Set time of last publish
  mqttpub.trigger ( MQTT_IP ) ;                            // Request re-publish IP
}


//**************************************************************************************************
//                                      H A N D L E V O L P U B                                    *
//**************************************************************************************************
// Handle publish of Volume to MQTT.  This will happen max every 10 seconds.                       *
//**************************************************************************************************
void handleVolPub()
{
  static uint8_t  oldvol = -1 ;                            // For comparison
  const char*        payload ;                                       // Points to payload
  char            strvar[120];
  if ( ini_block.reqvol != oldvol )                        // Volume change?
  {
    sprintf ( strvar, "%s_VOL=%d", ini_block.mqttprefix.c_str(), ini_block.reqvol);
    payload = strvar;
    mqttclient.publish("SN/DTS", payload);
    mqttpub.trigger ( MQTT_VOLUME ) ;                      // Request publish VOLUME
    oldvol = ini_block.reqvol ;                            // Remember publishe volume
  }
}



//**************************************************************************************************
//                                           C H K _ E N C                                         *
//**************************************************************************************************
// See if rotary encoder is activated and perform its functions.                                   *
//**************************************************************************************************
void chk_enc()
{
  static int8_t  enc_preset ;                                 // Selected preset
  static String  enc_nodeID ;                                 // Node of selected track
  static String  enc_filename ;                               // Filename of selected track
  String         tmp ;                                        // Temporary string
  int16_t        inx ;                                        // Position in string

  if ( enc_menu_mode != VOLUME )                              // In default mode?
  {
    if ( enc_inactivity > 40 )                                // No, more than 4 seconds inactive
    {
      enc_inactivity = 0 ;
      enc_menu_mode = VOLUME ;                                // Return to VOLUME mode
      dbgprint ( "Encoder mode back to VOLUME" ) ;
      tftset ( 2, (char*)NULL ) ;                             // Restore original text at bottom
    }
  }
  if ( singleclick || doubleclick ||                          // Any activity?
       tripleclick || longclick ||
       ( rotationcount != 0 ) )
  {
    blset ( true ) ;                                          // Yes, activate display if needed
  }
  else
  {
    return ;                                                  // No, nothing to do
  }
  if ( tripleclick )                                          // First handle triple click
  {
    dbgprint ( "Triple click") ;
    tripleclick = false ;
    if ( SD_nodecount )                                       // Tracks on SD?
    {
      enc_menu_mode = TRACK ;                                 // Swich to TRACK mode
      dbgprint ( "Encoder mode set to TRACK" ) ;
      tftset ( 3, "Turn to select track\n"                    // Show current option
               "Press to confirm" ) ;
      enc_nodeID = selectnextSDnode ( SD_currentnode, +1 ) ;  // Start with next file on SD
      if ( enc_nodeID == "" )                                 // Current track available?
      {
        inx = SD_nodelist.indexOf ( "\n" ) ;                  // No, find first
        enc_nodeID = SD_nodelist.substring ( 0, inx ) ;
      }
      // Stop playing as reading filenames saturates SD I/O.
      if ( datamode != STOPPED )
      {
        datamode = STOPREQD ;                                 // Request STOP
      }
    }
  }
  if ( doubleclick )                                          // Handle the doubleclick
  {
    dbgprint ( "Double click") ;
    doubleclick = false ;
    enc_menu_mode = PRESET ;                                  // Swich to PRESET mode
    dbgprint ( "Encoder mode set to PRESET" ) ;
    tftset ( 3, "Xoay de tron kenh\n"                    // Show current option
             "Nhan de xac nhan" ) ;       
    enc_preset = ini_block.newpreset + 1 ;                    // Start with current preset + 1
  }
  if ( singleclick )
  {
    dbgprint ( "Single click") ;
    singleclick = false ;
    switch ( enc_menu_mode )                                  // Which mode (VOLUME, PRESET, TRACK)?
    {
      case VOLUME :
        // if ( muteflag )
        // {
        //   tftset ( 3, "" ) ;                                  // Clear text
        // }
        // else
        // {
        //   tftset ( 3, "Mute" ) ;
        // }
        // muteflag = !muteflag ;                                // Mute/unmute
        if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                          PLAYLISTHEADER | PLAYLISTDATA ) )

        {
          digitalWrite (MUTE, LOW);
          datamode = STOPREQD ;                           // Request STOP
          playingstat=0;
          reload_accept=0;
          accept_try = 0;
          tftset ( 3, "STOP" ) ;
        }
        else
        {
          tftset ( 3, "" ) ;                                  // Clear text
          hostreq = true ;                                // Request UNSTOP
          playingstat=1;
          reload_accept = 1;
          accept_try = 1;
        }
        break ;
      case PRESET :
        currentpreset = -1 ;                                  // Make sure current is different
        ini_block.newpreset = enc_preset ;                    // Make a definite choice
        enc_menu_mode = VOLUME ;                              // Back to default mode
        tftset ( 3, "" ) ;                                    // Clear text
        break ;
      case TRACK :
        host = enc_filename ;                                 // Selected track as new host
        hostreq = true ;                                      // Request this host
        enc_menu_mode = VOLUME ;                              // Back to default mode
        tftset ( 3, "" ) ;                                    // Clear text
        break ;
    }
  }
  if ( longclick )                                            // Check for long click
  {
    crq=1;
    mqttpub.trigger (MQTT_CRQ);
    longclick=false;
    // dbgprint ( "Long click") ;
    // if ( datamode != STOPPED )
    // {
    //   datamode = STOPREQD ;                                   // Request STOP, do not touch logclick flag
    // }
    // else
    // {
    //   longclick = false ;                                     // Reset condition
    //   dbgprint ( "Long click detected" ) ;
    //   if ( SD_nodecount )                                     // Tracks on SD?
    //   {
    //     host = getSDfilename ( "0" ) ;                        // Get random track
    //     hostreq = true ;                                      // Request this host
    //   }
      muteflag = false ;                                      // Be sure muteing is off
    // }
  }
  if ( rotationcount == 0 )                                   // Any rotation?
  {
    return ;                                                  // No, return
  }
  dbgprint ( "Rotation count %d", rotationcount ) ;
  switch ( enc_menu_mode )                                    // Which mode (VOLUME, PRESET, TRACK)?
  {
    case VOLUME :
      if ( ( ini_block.reqvol + rotationcount ) < 0 )         // Limit volume
      {
        ini_block.reqvol = 0 ;                                // Limit to normal values
      }
      else if ( ( ini_block.reqvol + rotationcount ) > 100 )
      {
        ini_block.reqvol = 100 ;                              // Limit to normal values
      }
      else
      {
        ini_block.reqvol += rotationcount ;
      }
      muteflag = false ;                                      // Mute off
      break ;
    case PRESET :
      if ( ( enc_preset + rotationcount ) < 0 )               // Negative not allowed
      {
        enc_preset = 0 ;                                      // Stay at 0
      }
      else
      {
        enc_preset += rotationcount ;                         // Next preset
      }
      tmp = readhostfrompref ( enc_preset ) ;                 // Get host spec and possible comment
      if ( tmp == "" )                                        // End of presets?
      {
        enc_preset = 0 ;                                      // Yes, wrap
        tmp = readhostfrompref ( enc_preset ) ;               // Get host spec and possible comment
      }
      dbgprint ( "Preset is %d", enc_preset ) ;
      // Show just comment if available.  Otherwise the preset itself.
      inx = tmp.indexOf ( "#" ) ;                             // Get position of "#"
      if ( inx > 0 )                                          // Hash sign present?
      {
        tmp.remove ( 0, inx + 1 ) ;                           // Yes, remove non-comment part
      }
      chomp ( tmp ) ;                                         // Remove garbage from description
      tftset ( 3, tmp ) ;                                     // Set screen segment bottom part
      break ;
    case TRACK :
      enc_nodeID = selectnextSDnode ( enc_nodeID,
                                      rotationcount ) ;       // Select the next file on SD
      enc_filename = getSDfilename ( enc_nodeID ) ;           // Set new filename
      tmp = enc_filename ;                                    // Copy for display
      dbgprint ( "Select %s", tmp.c_str() ) ;
      while ( ( inx = tmp.indexOf ( "/" ) ) >= 0 )            // Search for last slash
      {
        tmp.remove ( 0, inx + 1 ) ;                           // Remove before the slash
      }
      dbgprint ( "Simplified %s", tmp.c_str() ) ;
      tftset ( 3, tmp ) ;
    // Set screen segment bottom part
    default :
      break ;
  }
  rotationcount = 0 ;                                         // Reset
}


//**************************************************************************************************
//                                           M P 3 L O O P                                         *
//**************************************************************************************************
// Called from the mail loop() for the mp3 functions.                                              *
// A connection to an MP3 server is active and we are ready to receive data.                       *
// Normally there is about 2 to 4 kB available in the data stream.  This depends on the sender.    *
//**************************************************************************************************
void mp3loop()
{
  int             count_rs = 0;
  uint32_t        maxchunk ;                             // Max number of bytes to read
  int             res = 0 ;                              // Result reading from mp3 stream
  uint32_t        av = 0 ;                               // Available in stream
  String          nodeID ;                               // Next nodeID of track on SD
  uint32_t        timing ;                               // Startime and duration this function
  int             state_mk = 0;                           //
  int             inx ;                                  // Indexe of "#" in station name
  // Try to keep the Queue to playtask filled up by adding as much bytes as possible
  if ( datamode & ( INIT | HEADER | DATA |               // Test op playing
                    METADATA | PLAYLISTINIT |
                    PLAYLISTHEADER |
                    PLAYLISTDATA ) )
  {
    
    timing = millis() ;                                  // Start time this function
    maxchunk = sizeof(tmpbuff) ;                         // Reduce byte count for this mp3loop()
    qspace = uxQueueSpacesAvailable( dataqueue ) *       // Compute free space in data queue
             sizeof(qdata_struct) ;
    delay(50);

    if ( localfile )                                     // Playing file from SD card?
    {
      av = mp3filelength ;                               // Bytes left in file
      if ( av < maxchunk )                               // Reduce byte count for this mp3loop()
      {
        maxchunk = av ;
      }
      if ( maxchunk > qspace )                           // Enough space in queue?
      {
        maxchunk = qspace ;                              // No, limit to free queue space
      }
      if ( maxchunk )                                    // Anything to read?
      {
        claimSPI ( "sdread" ) ;                          // Claim SPI bus
        res = mp3file.read ( tmpbuff, maxchunk ) ;       // Read a block of data
        releaseSPI() ;                                   // Release SPI bus
        mp3filelength -= res ;                           // Number of bytes left
      }
    }
    else
    {
      av = mp3client.available() ;                       // Available from stream
      if ( av < maxchunk )                               // Limit read size
      {
        maxchunk = av ;
      }
      if ( maxchunk > qspace )                           // Enough space in queue?
      {
        maxchunk = qspace ;                              // No, limit to free queue space
      }
      if ( maxchunk )                                    // Anything to read?
      {
        res = mp3client.read ( tmpbuff, maxchunk ) ;     // Read a number of bytes from the stream
      }
    }
    if ( maxchunk == 0 )
    {
      if ( datamode == PLAYLISTDATA )                    // End of playlist
      {
        playlist_num = 1 ;                               // Yes, restart playlist
        dbgprint ( "End of playlist seen" ) ;
        datamode = STOPPED ;
        ini_block.newpreset++ ;
      }
    }

    for ( int i = 0 ; i < res ; i++ )
    {
      handlebyte_ch ( tmpbuff[i] ) ;                     // Handle one byte
    }
    timing = millis() - timing ;                         // Duration this function
    if ( timing > max_mp3loop_time )                     // New maximum found?
    {
      max_mp3loop_time = timing ;                        // Yes, set new maximum
      dbgprint ( "Duration mp3loop %d", timing ) ;       // and report it
    }
  }
  if ( datamode == STOPREQD )                            // STOP requested?
  {
    dbgprint ( "STOP requested" ) ;
    if (reload_accept == 0 )
    {
      sts=0;
      digitalWrite (MUTE, LOW);
    }
    if ( localfile )
    {
      claimSPI ( "close" ) ;                             // Claim SPI bus
      mp3file.close() ;
      releaseSPI() ;                                     // Release SPI bus
    }
    else
    {
      stop_mp3client() ;                                 // Disconnect if still connected
    }
    chunked = false ;                                    // Not longer chunked
    datacount = 0 ;                                      // Reset datacount
    outqp = outchunk.buf ;                               // and pointer
    queuefunc ( QSTOPSONG ) ;                            // Queue a request to stop the song
    metaint = 0 ;                                        // No metaint known now
    datamode = STOPPED ;                                 // Yes, state becomes STOPPED
    return ;
  }
  if ( localfile )                                       // Playing from SD?
  {
    if ( datamode & DATA )                               // Test op playing
    {
      if ( av == 0 )                                     // End of mp3 data?
      {
        datamode = STOPREQD ;                            // End of local mp3-file detected
        if ( playlist_num )                              // Playing from playlist?
        {
          playlist_num++ ;                               // Yes, goto next item in playlist
          datamode = PLAYLISTINIT ;
          host = playlist ;
        }
        else
        {
          nodeID = selectnextSDnode ( SD_currentnode,
                                      +1 ) ;             // Select the next file on SD
          host = getSDfilename ( nodeID ) ;
        }
        hostreq = true ;                                 // Request this host
      }
    }
  }
if(reload_accept==1)
{
  if ( hostreq )                                          // New preset or station?
  {
    hostreq = false ;
    currentpreset = ini_block.newpreset ;                 // Remember current preset
 //   mqttpub.trigger ( MQTT_PRESET ) ;                     // Request publishing to MQTT
    // Find out if this URL is on localhost (SD).
    localfile = ( host.indexOf ( "localhost/" ) >= 0 ) ;
    if ( localfile )                                      // Play file from localhost?
    {
      if ( ! connecttofile() )                            // Yes, open mp3-file
      {
        datamode = STOPPED ;                              // Start in DATA mode
      }
    }
    else
    {
      if ( host.startsWith ( "ihr/" ) )                   // iHeartRadio station requested?
      {
        host = host.substring ( 4 ) ;                     // Yes, remove "ihr/"
        host = xmlgethost ( host ) ;                      // Parse the xml to get the host
      }
      connecttohost() ;                                   // Switch to new host
    }
  }
  if ( ini_block.newpreset != currentpreset )            // New station or next from playlist requested?
  {
    if ( datamode != STOPPED )                           // Yes, still busy?
    {
      datamode = STOPREQD ;                              // Yes, request STOP
    }
    else
    {
      if ( playlist_num )                                 // Playing from playlist?
      { // Yes, retrieve URL of playlist
        playlist_num += ini_block.newpreset -
                        currentpreset ;                   // Next entry in playlist

        ini_block.newpreset = currentpreset ;             // Stay at current preset
        // Serial.println("oke");
      }
      else
      {
        host = readhostfrompref() ;                       // Lookup preset in preferences
        icyname = host ;                                  // First guess station name
        inx = icyname.indexOf ( "#" ) ;                   // Get position of "#"
        if ( inx > 0 )                                    // Hash sign present?
        {
          icyname.remove ( 0, inx + 1 ) ;                 // Yes, remove non-comment part
        }
        chomp ( icyname ) ;                               // Remove garbage from description
        tftset ( 2, icyname ) ;                           // Set screen segment bottom part
        chomp ( host ) ;                                  // Get rid of part after "#"
        //   Serial.println("oke");
      }
      dbgprint ( "New preset/file requested (%d/%d) from %s",
                 ini_block.newpreset, playlist_num, host.c_str() ) ;
      if ( host != ""  )                                  // Preset in ini-file?
      {
        hostreq = true ;                                  // Force this station as new preset
        // Serial.println ( "oke");
      }
      else
      {
        // This preset is not available, return to preset 0, will be handled in next mp3loop()
        dbgprint ( "No host for this preset" ) ;
        ini_block.newpreset = 0 ;                         // Wrap to first station
      }
    }
  }
}
}


//**************************************************************************************************
//                                           L O O P                                               *
//**************************************************************************************************
// Main loop of the program.                                                                       *
//**************************************************************************************************
void loop()
{
  if ( updatereq )                                  // Software update requested?
  {
    if ( displaytype == T_NEXTION )                 // NEXTION in use?
    {
      update_software ( "lstmodn",                  // Yes, update NEXTION image from remote image
                        UPDATEHOST, TFTFILE ) ;
    }
    update_software ( "lstmods",                    // Update sketch from remote file
                      UPDATEHOST, BINFILE ) ;
    resetreq = true ;                               // And reset
  }
  if ( resetreq )                                   // Reset requested?
  {
    datamode = STOPREQD ;
    digitalWrite (MUTE, LOW);
    delay ( 1000 ) ;                                // Yes, wait some time
    ESP.restart() ;                                 // Reboot
  }
  scanserial() ;                                    // Handle serial input
  scandigital() ;                                   // Scan digital inputs
  scanIR() ;                                        // See if IR input
  ArduinoOTA.handle() ;                             // Check for OTA
  if(eth_connected)
  {
    if ( ban == 0 )
    {
       mp3loop() ;
     }
  }
  // Do more mp3 related actions
  handlehttpreply() ;
  cmdclient = cmdserver.available() ;               // Check Input from client?
  if ( cmdclient )                                  // Client connected?
  {
    dbgprint ( "Command client available" ) ;
    handlehttp() ;
  }
  // Handle MQTT.
  if ( mqtt_on || mqtt_on_manu )
  {
    mqttclient.loop() ;                             // Handling of MQTT connection
    mqttclient1.loop();
  }
  handleVolPub();
  handle_spec() ;
  handleSTSPub();
  handleSaveReq() ;                                 // See if time to save settings
  Sts_Pub_Change();
  resetvs1053();
  wait_reset();
  time_online();
  led_tick();
//  IO_Tick();
  Mic_tick();
  if(dur <3)
  {
    GetMac();
  }

}
//**************************************************************************************************
//                             D E C O D E _ S P E C _ C H A R S                                   *
//**************************************************************************************************
// Decode special characters like "&#39;".                                                         *
//**************************************************************************************************
String decode_spec_chars ( String str )
{
  int    inx, inx2 ;                                // Indexes in string
  char   c ;                                        // Character from string
  char   val ;                                      // Converted character
  String res = str ;

  while ( ( inx = res.indexOf ( "&#" ) ) >= 0 )     // Start sequence in string?
  {
    inx2 = res.indexOf ( ";", inx ) ;               // Yes, search for stop character
    if ( inx2 < 0 )                                 // Stop character found
    {
      break ;                                       // Malformed string
    }
    res = str.substring ( 0, inx ) ;                // First part
    inx += 2 ;                                      // skip over start sequence
    val = 0 ;                                       // Init result of 
    while ( ( c = str[inx++] ) != ';' )             // Convert character
    {
      val = val * 10 + c - '0' ;
    }
    res += ( String ( val ) +                       // Add special char to string
             str.substring ( ++inx2 ) ) ;           // Add rest of string
  }
  return res ;
}

//**************************************************************************************************
//                                    C H K H D R L I N E                                          *
//**************************************************************************************************
// Check if a line in the header is a reasonable headerline.                                       *
// Normally it should contain something like "icy-xxxx:abcdef".                                    *
//**************************************************************************************************
bool chkhdrline ( const char* str )
{
  char    b ;                                         // Byte examined
  int     len = 0 ;                                   // Lengte van de string

  while ( ( b = *str++ ) )                            // Search to end of string
  {
    len++ ;                                           // Update string length
    if ( ! isalpha ( b ) )                            // Alpha (a-z, A-Z)
    {
      if ( b != '-' )                                 // Minus sign is allowed
      {
        if ( b == ':' )                               // Found a colon?
        {
          return ( ( len > 5 ) && ( len < 50 ) ) ;    // Yes, okay if length is okay
        }
        else
        {
          return false ;                              // Not a legal character
        }
      }
    }
  }
  return false ;                                      // End of string without colon
}


//**************************************************************************************************
//                            S C A N _ C O N T E N T _ L E N G T H                                *
//**************************************************************************************************
// If the line contains content-length information: set clength (content length counter).          *
//**************************************************************************************************
void scan_content_length ( const char* metalinebf )
{
  if ( strstr ( metalinebf, "Content-Length" ) )        // Line contains content length
  {
    clength = atoi ( metalinebf + 15 ) ;                // Yes, set clength
    dbgprint ( "Content-Length is %d", clength ) ;      // Show for debugging purposes
  }
}


//**************************************************************************************************
//                                   H A N D L E B Y T E _ C H                                     *
//**************************************************************************************************
// Handle the next byte of data from server.                                                       *
// Chunked transfer encoding aware. Chunk extensions are not supported.                            *
//**************************************************************************************************
void handlebyte_ch ( uint8_t b )
{
  static int       chunksize = 0 ;                      // Chunkcount read from stream
  static uint16_t  playlistcnt ;                        // Counter to find right entry in playlist
  static int       LFcount ;                            // Detection of end of header
  static bool      ctseen = false ;                     // First line of header seen or not

  if ( chunked &&
       ( datamode & ( DATA |                           // Test op DATA handling
                      METADATA |
                      PLAYLISTDATA ) ) )
  {
    if ( chunkcount == 0 )                             // Expecting a new chunkcount?
    {
      if ( b == '\r' )                                 // Skip CR
      {
        return ;
      }
      else if ( b == '\n' )                            // LF ?
      {
        chunkcount = chunksize ;                       // Yes, set new count
        chunksize = 0 ;                                // For next decode
        return ;
      }
      // We have received a hexadecimal character.  Decode it and add to the result.
      b = toupper ( b ) - '0' ;                        // Be sure we have uppercase
      if ( b > 9 )
      {
        b = b - 7 ;                                    // Translate A..F to 10..15
      }
      chunksize = ( chunksize << 4 ) + b ;
      return  ;
    }
    chunkcount-- ;                                     // Update count to next chunksize block
  }
  if ( datamode == DATA )                              // Handle next byte of MP3/Ogg data
  {
    *outqp++ = b ;
    if ( outqp == ( outchunk.buf + sizeof(outchunk.buf) ) ) // Buffer full?
    {
      // Send data to playtask queue.  If the buffer cannot be placed within 200 ticks,
      // the queue is full, while the sender tries to send more.  The chunk will be dis-
      // carded it that case.
      xQueueSend ( dataqueue, &outchunk, 200 ) ;       // Send to queue
      outqp = outchunk.buf ;                           // Item empty now
    }
    if ( metaint )                                     // No METADATA on Ogg streams or mp3 files
    {
      if ( --datacount == 0 )                          // End of datablock?
      {
        datamode = METADATA ;
        metalinebfx = -1 ;                             // Expecting first metabyte (counter)
      }
    }
    return ;
  }
  if ( datamode == INIT )                              // Initialize for header receive
  {
    ctseen = false ;                                   // Contents type not seen yet
    metaint = 0 ;                                      // No metaint found
    LFcount = 0 ;                                      // For detection end of header
    bitrate = 0 ;                                      // Bitrate still unknown
    dbgprint ( "Switch to HEADER" ) ;
    datamode = HEADER ;                                // Handle header
    totalcount = 0 ;                                   // Reset totalcount
    metalinebfx = 0 ;                                  // No metadata yet
    metalinebf[0] = '\0' ;
  }
  if ( datamode == HEADER )                            // Handle next byte of MP3 header
  {
    if ( ( b > 0x7F ) ||                               // Ignore unprintable characters
         ( b == '\r' ) ||                              // Ignore CR
         ( b == '\0' ) )                               // Ignore NULL
    {
      // Yes, ignore
    }
    else if ( b == '\n' )                              // Linefeed ?
    {
      LFcount++ ;                                      // Count linefeeds
      metalinebf[metalinebfx] = '\0' ;                 // Take care of delimiter
      if ( chkhdrline ( metalinebf ) )                 // Reasonable input?
      {
        dbgprint ( "Headerline: %s",                   // Show headerline
                   metalinebf ) ;
        String metaline = String ( metalinebf ) ;      // Convert to string
        String lcml = metaline ;                       // Use lower case for compare
        lcml.toLowerCase() ;
        if ( lcml.startsWith ( "location: http://" ) ) // Redirection?
        {
          host = metaline.substring ( 17 ) ;           // Yes, get new URL
          hostreq = true ;                             // And request this one
        }
        if ( lcml.indexOf ( "content-type" ) >= 0)     // Line with "Content-Type: xxxx/yyy"
        {
          ctseen = true ;                              // Yes, remember seeing this
          String ct = metaline.substring ( 13 ) ;      // Set contentstype. Not used yet
          ct.trim() ;
          dbgprint ( "%s seen.", ct.c_str() ) ;
        }
        if ( lcml.startsWith ( "icy-br:" ) )
        {
          bitrate = metaline.substring(7).toInt() ;    // Found bitrate tag, read the bitrate
          if ( bitrate == 0 )                          // For Ogg br is like "Quality 2"
          {
            bitrate = 87 ;                             // Dummy bitrate
          }
        }
        else if ( lcml.startsWith ("icy-metaint:" ) )
        {
          metaint = metaline.substring(12).toInt() ;   // Found metaint tag, read the value
        }
        else if ( lcml.startsWith ( "icy-name:" ) )
        {
          icyname = metaline.substring(9) ;            // Get station name
          icyname = decode_spec_chars ( icyname ) ;    // Decode special characters in name
          icyname.trim() ;                             // Remove leading and trailing spaces
          tftset ( 2, icyname ) ;                      // Set screen segment bottom part
        //  mqttpub.trigger ( MQTT_ICY ) ;           // Request publishing to MQTT
        }
        else if ( lcml.startsWith ( "transfer-encoding:" ) )
        {
          // Station provides chunked transfer
          if ( lcml.endsWith ( "chunked" ) )
          {
            chunked = true ;                           // Remember chunked transfer mode
            chunkcount = 0 ;                           // Expect chunkcount in DATA
          }
        }
      }
      metalinebfx = 0 ;                                // Reset this line
      if ( ( LFcount == 2 ) && ctseen )                // Content type seen and a double LF?
      {
        dbgprint ( "Switch to DATA, bitrate is %d"     // Show bitrate
                   ", metaint is %d",                  // and metaint
                   bitrate, metaint ) ;

        datamode = DATA ;                              // Expecting data now
        datacount = metaint ;                          // Number of bytes before first metadata
        queuefunc ( QSTARTSONG ) ;                     // Queue a request to start song
      }
    }
    else
    {
      metalinebf[metalinebfx++] = (char)b ;            // Normal character, put new char in metaline
      if ( metalinebfx >= METASIZ )                    // Prevent overflow
      {
        metalinebfx-- ;
      }
      LFcount = 0 ;                                    // Reset double CRLF detection
    }
    return ;
  }
  if ( datamode == METADATA )                          // Handle next byte of metadata
  {
    if ( metalinebfx < 0 )                             // First byte of metadata?
    {
      metalinebfx = 0 ;                                // Prepare to store first character
      metacount = b * 16 + 1 ;                         // New count for metadata including length byte
      if ( metacount > 1 )
      {
        dbgprint ( "Metadata block %d bytes",
                   metacount - 1 ) ;                   // Most of the time there are zero bytes of metadata
      }
    }
    else
    {
      metalinebf[metalinebfx++] = (char)b ;            // Normal character, put new char in metaline
      if ( metalinebfx >= METASIZ )                    // Prevent overflow
      {
        metalinebfx-- ;
      }
    }
    if ( --metacount == 0 )
    {
      metalinebf[metalinebfx] = '\0' ;                 // Make sure line is limited
      if ( strlen ( metalinebf ) )                     // Any info present?
      {
        // metaline contains artist and song name.  For example:
        // "StreamTitle='Don McLean - American Pie';StreamUrl='';"
        // Sometimes it is just other info like:
        // "StreamTitle='60s 03 05 Magic60s';StreamUrl='';"
        // Isolate the StreamTitle, remove leading and trailing quotes if present.
        showstreamtitle ( metalinebf ) ;               // Show artist and title if present in metadata
       // mqttpub.trigger ( MQTT_STREAMTITLE ) ;         // Request publishing to MQTT
      }
      if ( metalinebfx  > ( METASIZ - 10 ) )           // Unlikely metaline length?
      {
        dbgprint ( "Metadata block too long! Skipping all Metadata from now on." ) ;
        metaint = 0 ;                                  // Probably no metadata
      }
      datacount = metaint ;                            // Reset data count
      //bufcnt = 0 ;                                     // Reset buffer count
      datamode = DATA ;                                // Expecting data
    }
  }
  if ( datamode == PLAYLISTINIT )                      // Initialize for receive .m3u file
  {
    // We are going to use metadata to read the lines from the .m3u file
    // Sometimes this will only contain a single line
    metalinebfx = 0 ;                                  // Prepare for new line
    LFcount = 0 ;                                      // For detection end of header
    datamode = PLAYLISTHEADER ;                        // Handle playlist header
    if ( localfile )                                   // SD-card mode?
    {
      datamode = PLAYLISTDATA ;                        // Yes, no header here
    }
    playlistcnt = 1 ;                                  // Reset for compare
    totalcount = 0 ;                                   // Reset totalcount
    clength = 0xFFFFFFFF ;                             // Content-length unknown
    dbgprint ( "Read from playlist" ) ;
  }
  if ( datamode == PLAYLISTHEADER )                    // Read header
  {
    if ( ( b > 0x7F ) ||                               // Ignore unprintable characters
         ( b == '\r' ) ||                              // Ignore CR
         ( b == '\0' ) )                               // Ignore NULL
    {
      return ;                                         // Quick return
    }
    else if ( b == '\n' )                              // Linefeed ?
    {
      LFcount++ ;                                      // Count linefeeds
      metalinebf[metalinebfx] = '\0' ;                 // Take care of delimeter
      dbgprint ( "Playlistheader: %s",                 // Show playlistheader
                 metalinebf ) ;
      scan_content_length ( metalinebf ) ;             // Check if it is a content-length line
      metalinebfx = 0 ;                                // Ready for next line
      if ( LFcount == 2 )
      {
        dbgprint ( "Switch to PLAYLISTDATA, "          // For debug
                   "search for entry %d",
                   playlist_num ) ;
        datamode = PLAYLISTDATA ;                      // Expecting data now
        mqttpub.trigger ( MQTT_PLAYLISTPOS ) ;         // Playlistposition to MQTT
        return ;
      }
    }
    else
    {
      metalinebf[metalinebfx++] = (char)b ;            // Normal character, put new char in metaline
      if ( metalinebfx >= METASIZ )                    // Prevent overflow
      {
        metalinebfx-- ;
      }
      LFcount = 0 ;                                    // Reset double CRLF detection
    }
  }
  if ( datamode == PLAYLISTDATA )                      // Read next byte of .m3u file data
  {
    clength-- ;                                        // Decrease content length by 1
    if ( ( b > 0x7F ) ||                               // Ignore unprintable characters
         ( b == '\r' ) ||                              // Ignore CR
         ( b == '\0' ) )                               // Ignore NULL
    {
      // Yes, ignore
    }
    if ( b != '\n' )                                   // Linefeed?
    { // No, normal character in playlistdata,
      metalinebf[metalinebfx++] = (char)b ;            // add it to metaline
      if ( metalinebfx >= METASIZ )                    // Prevent overflow
      {
        metalinebfx-- ;
      }
    }
    if ( ( b == '\n' ) ||                              // linefeed ?
         ( clength == 0 ) )                            // Or end of playlist data contents
    {
      int inx ;                                        // Pointer in metaline
      metalinebf[metalinebfx] = '\0' ;                 // Take care of delimeter
      dbgprint ( "Playlistdata: %s",                   // Show playlistheader
                 metalinebf ) ;
      if ( strlen ( metalinebf ) < 5 )                 // Skip short lines
      {
        metalinebfx = 0 ;                              // Flush line
        metalinebf[0] = '\0' ;
        return ;
      }
      String metaline = String ( metalinebf ) ;        // Convert to string
      if ( metaline.indexOf ( "#EXTINF:" ) >= 0 )      // Info?
      {
        if ( playlist_num == playlistcnt )             // Info for this entry?
        {
          inx = metaline.indexOf ( "," ) ;             // Comma in this line?
          if ( inx > 0 )
          {
            // Show artist and title if present in metadata
            showstreamtitle ( metaline.substring ( inx + 1 ).c_str(), true ) ;
           // mqttpub.trigger ( MQTT_STREAMTITLE ) ;     // Request publishing to MQTT
          }
        }
      }
      if ( metaline.startsWith ( "#" ) )               // Commentline?
      {
        metalinebfx = 0 ;                              // Yes, ignore
        return ;                                       // Ignore commentlines
      }
      // Now we have an URL for a .mp3 file or stream.  Is it the rigth one?
      dbgprint ( "Entry %d in playlist found: %s", playlistcnt, metalinebf ) ;
      if ( playlist_num == playlistcnt  )
      {
        inx = metaline.indexOf ( "http://" ) ;         // Search for "http://"
        if ( inx >= 0 )                                // Does URL contain "http://"?
        {
          host = metaline.substring ( inx + 7 ) ;      // Yes, remove it and set host
        }
        else
        {
          host = metaline ;                            // Yes, set new host
        }
        if ( localfile )                               // SD card mode?
        {
          if ( ! metaline.startsWith ( "localhost" ) ) // Prepend "localhost" if missing
          {
            host = String ( "localhost/" ) + metaline ;
          }
          if ( ! connecttofile() )                     // Yes, connect to file
          {
            datamode = STOPPED ;                       // Error, stop!
          }
        }
        else
        {
          connecttohost() ;                           // Connect to stream host
        }
      }
      metalinebfx = 0 ;                                // Prepare for next line
      host = playlist ;                                // Back to the .m3u host
      playlistcnt++ ;                                  // Next entry in playlist
    }
  }
}


//**************************************************************************************************
//                                     G E T C O N T E N T T Y P E                                 *
//**************************************************************************************************
// Returns the contenttype of a file to send.                                                      *
//**************************************************************************************************
String getContentType ( String filename )
{
  if      ( filename.endsWith ( ".html" ) ) return "text/html" ;
  else if ( filename.endsWith ( ".png"  ) ) return "image/png" ;
  else if ( filename.endsWith ( ".gif"  ) ) return "image/gif" ;
  else if ( filename.endsWith ( ".jpg"  ) ) return "image/jpeg" ;
  else if ( filename.endsWith ( ".ico"  ) ) return "image/x-icon" ;
  else if ( filename.endsWith ( ".css"  ) ) return "text/css" ;
  else if ( filename.endsWith ( ".zip"  ) ) return "application/x-zip" ;
  else if ( filename.endsWith ( ".gz"   ) ) return "application/x-gzip" ;
  else if ( filename.endsWith ( ".mp3"  ) ) return "audio/mpeg" ;
  else if ( filename.endsWith ( ".pw"   ) ) return "" ;              // Passwords are secret
  return "text/plain" ;
}


//**************************************************************************************************
//                                        H A N D L E F S F                                        *
//**************************************************************************************************
// Handling of requesting pages from the PROGMEM. Example: favicon.ico                             *
//**************************************************************************************************
void handleFSf ( const String& pagename )
{
  String                 ct ;                           // Content type
  const char*            p ;
  int                    l ;                            // Size of requested page
  int                    TCPCHUNKSIZE = 1024 ;          // Max number of bytes per write

  dbgprint ( "FileRequest received %s", pagename.c_str() ) ;
  ct = getContentType ( pagename ) ;                    // Get content type
  if ( ( ct == "" ) || ( pagename == "" ) )             // Empty is illegal
  {
    cmdclient.println ( "HTTP/1.1 404 Not Found" ) ;
    cmdclient.println ( "" ) ;
    return ;
  }
  else
  {
    if ( pagename.indexOf ( "index.html" ) >= 0 )       // Index page is in PROGMEM
    {
      p = index_html ;
      l = sizeof ( index_html ) ;
    }
    else if ( pagename.indexOf ( "radio.css" ) >= 0 )   // CSS file is in PROGMEM
    {
      p = radio_css + 1 ;
      l = sizeof ( radio_css ) ;
    }
    else if ( pagename.indexOf ( "config.html" ) >= 0 ) // Config page is in PROGMEM
    {
      p = config_html ;
      l = sizeof ( config_html ) ;
    }
    else if ( pagename.indexOf ( "mp3play.html" ) >= 0 ) // Mp3player page is in PROGMEM
    {
      p = mp3play_html ;
      l = sizeof ( mp3play_html ) ;
    }
    else if ( pagename.indexOf ( "about.html" ) >= 0 )  // About page is in PROGMEM
    {
      p = about_html ;
      l = sizeof ( about_html ) ;
    }
    else if ( pagename.indexOf ( "favicon.ico" ) >= 0 ) // Favicon icon is in PROGMEM
    {
      p = (char*)favicon_ico ;
      l = sizeof ( favicon_ico ) ;
    }
    else
    {
      p = index_html ;
      l = sizeof ( index_html ) ;
    }
    if ( *p == '\n' )                                   // If page starts with newline:
    {
      p++ ;                                             // Skip first character
      l-- ;
    }
    dbgprint ( "Length of page is %d", strlen ( p ) ) ;
    cmdclient.print ( httpheader ( ct ) ) ;             // Send header
    // The content of the HTTP response follows the header:
    if ( l < 10 )
    {
      cmdclient.println ( "Testline<br>" ) ;
    }
    else
    {
      while ( l )                                       // Loop through the output page
      {
        if ( l <= TCPCHUNKSIZE )                        // Near the end?
        {
          cmdclient.write ( p, l ) ;                    // Yes, send last part
          l = 0 ;
        }
        else
        {
          cmdclient.write ( p, TCPCHUNKSIZE ) ;         // Send part of the page
          p += TCPCHUNKSIZE ;                           // Update startpoint and rest of bytes
          l -= TCPCHUNKSIZE ;
        }
      }
    }
    // The HTTP response ends with another blank line:
    cmdclient.println() ;
    dbgprint ( "Response send" ) ;
  }
}


//**************************************************************************************************
//                                         C H O M P                                               *
//**************************************************************************************************
// Do some filtering on de inputstring:                                                            *
//  - String comment part (starting with "#").                                                     *
//  - Strip trailing CR.                                                                           *
//  - Strip leading spaces.                                                                        *
//  - Strip trailing spaces.                                                                       *
//**************************************************************************************************
void chomp ( String &str )
{
  int   inx ;                                         // Index in de input string

  if ( ( inx = str.indexOf ( "#" ) ) >= 0 )           // Comment line or partial comment?
  {
    str.remove ( inx ) ;                              // Yes, remove
  }
  str.trim() ;                                        // Remove spaces and CR
}


//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient, Serial or MQTT.                         *
// Version for handling string with: <parameter>=<value>                                           *
//**************************************************************************************************
const char* analyzeCmd ( const char* str )
{
  char*        value ;                           // Points to value after equalsign in command
  const char*  res ;                             // Result of analyzeCmd

  value = strstr ( str, "=" ) ;                  // See if command contains a "="
  if ( value )
  {
    *value = '\0' ;                              // Separate command from value
    res = analyzeCmd ( str, value + 1 ) ;        // Analyze command and handle it
    *value = '=' ;                               // Restore equal sign
  }
  else
  {
    res = analyzeCmd ( str, "0" ) ;              // No value, assume zero
  }
  return res ;
}

const char* analyzeCmd ( const char* par, const char* val )
{
  String             argument ;                       // Argument as string
  String             value ;                          // Value of an argument as a string
  int                ivalue ;                         // Value of argument as an integer
  static char        reply[180] ;                     // Reply to client, will be returned
  uint8_t            oldvol ;                         // Current volume
  bool               relative ;                       // Relative argument (+ or -)
  String             tmpstr ;                         // Temporary for value
  uint32_t           av ;                             // Available in stream/file
  blset ( true ) ;                                    // Enable backlight of TFT
  strcpy ( reply, "Command accepted" ) ;              // Default reply
  argument = String ( par ) ;                         // Get the argument
  chomp ( argument ) ;                                // Remove comment and useless spaces
  if ( argument.length() == 0 )                       // Lege commandline (comment)?
  {
    return reply ;                                    // Ignore
  }
  argument.toLowerCase() ;                            // Force to lower case
  value = String ( val ) ;                            // Get the specified value
  chomp ( value ) ;                                   // Remove comment and extra spaces
  ivalue = value.toInt() ;                            // Also as an integer
  ivalue = abs ( ivalue ) ;                           // Make positive
  relative = argument.indexOf ( "u" ) == 0 ;         // + relative setting?
  if ( argument.indexOf ( "d" ) == 0 )             // - relative setting?
  {
    relative = true ;                                 // It's relative
    ivalue = - ivalue ;                               // But with negative value
  }
  if ( value.startsWith ( "http://" ) )               // Does (possible) URL contain "http://"?
  {
    value.remove ( 0, 7 ) ;                           // Yes, remove it
  }
  if ( value.length() )
  {
    tmpstr = value ;                                  // Make local copy of value
    if ( argument.indexOf ( "passw" ) >= 0 )          // Password in value?
    {
      tmpstr = String ( "*******" ) ;                 // Yes, hide it
    }
    dbgprint ( "Command: %s with parameter %s",
               argument.c_str(), tmpstr.c_str() ) ;
  }
  else
  {
    dbgprint ( "Command: %s (without parameter)",
               argument.c_str() ) ;
  }
  if ( argument.indexOf ( "vol" ) >= 0 )
  {
    if ( value == "?")
    {
      mqttpub.trigger ( MQTT_VOLUME );
    }
    else
    {
      oldvol = vs1053player->getVolume() ;
      if ( relative )                                   // + relative setting?
      {
        ini_block.reqvol = oldvol + ivalue ;            // Up/down by 0.5 or more dB
      }
      else
      {
        ini_block.reqvol = ivalue ;                     // Absolue setting

        mqttpub.trigger ( MQTT_VOLUME );
      }
      if ( ini_block.reqvol > 127 )                     // Wrapped around?
      {
        ini_block.reqvol = 0 ;                          // Yes, keep at zero
        mqttpub.trigger ( MQTT_VOLUME );
      }
      if ( ini_block.reqvol > 100 )
      {
        ini_block.reqvol = 100 ;                        // Limit to normal values
        mqttpub.trigger ( MQTT_VOLUME );
      }
      muteflag = false ;                                // Stop possibly muting
      sprintf ( reply, "Volume is now %d",              // Reply new volume
                ini_block.reqvol ) ;
    }
  }
else if (argument.indexOf ( "ply" ) >= 0)
  {
    if ( value == "?")
    {
      mqttpub.trigger ( MQTT_PLY );
    }
    if (ban == 0)
    {
      if (value == "0")
      {
        sts=0;
        if(cam_mic == false)
        {
          if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                            PLAYLISTHEADER | PLAYLISTDATA ) )
    
          {
             datamode = STOPREQD ;                           // Request STOP
             playingstat=0;
             reload_accept=0;
             accept_try =0;
          }
          else
          {
             reload_accept=0;
             accept_try =0;
          }
          digitalWrite (MUTE, LOW);
        }
      }
      else if (value == "1")
      {
        if(cam_mic == false)
        {
          stt_ply = 1;
          if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                            PLAYLISTHEADER | PLAYLISTDATA ) )
    
          {
          }
          else
          {
            hostreq = true ;                                // Request UNSTOP
            playingstat=1;
            reload_accept = 1;
            accept_try=1;
          }
         }
       }
    }
  }
  else if ( argument.indexOf ( "pre" ) >= 0 )      // (UP/DOWN)Preset station?
  {

    if ( value == "?")
    {
      mqttpub.trigger ( MQTT_PRESET );
    }
    // If MP3 player is active: change track
    else if ( localfile && ( ( datamode & DATA ) != 0 ) && relative )            // MP# player active?
    {
      datamode = STOPREQD ;                           // Force stop MP3 player
      if ( playlist_num )                             // In playlist mode?
      {
        playlist_num += ivalue ;                      // Set new entry number
        if ( playlist_num <= 0 )                      // Limit number
        {
          playlist_num = 1 ;
        }
        host = playlist ;                             // Yes, prepare to read playlist
      }
      else
      {
        tmpstr = selectnextSDnode ( SD_currentnode,
                                    ivalue ) ;        // Select the next or previous file on SD
        host = getSDfilename ( tmpstr ) ;
        sprintf ( reply, "Playing %s",                // Reply new filename
                  host.c_str() ) ;
      }
      hostreq = true ;                                // Request this host
    }
    else
    {
      if ( relative )                                 // Relative argument?
      {
        currentpreset = ini_block.newpreset ;         // Remember currentpreset
        ini_block.newpreset += ivalue ;               // Yes, adjust currentpreset
      }
      else
      {
        ini_block.newpreset = ivalue ;                // Otherwise set station
        playlist_num = 0 ;                            // Absolute, reset playlist
        currentpreset = -1 ;                          // Make sure current is different
      }
      datamode = STOPREQD ;                           // Force stop MP3 player
      sprintf ( reply, "Preset is now %d",            // Reply new preset
                ini_block.newpreset ) ;

    }
  }
  else if (argument.indexOf ( "sts") >= 0)
  {
    if (value == "?")
    {
        mqttpub.trigger (MQTT_STS);
     }
  }
  else if (argument.indexOf ( "volmic") >= 0)
  {
  }
  else if (argument.indexOf ( "volstream") >= 0)
  {
  }
  else if ( argument.indexOf( "err") >=0 )
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_ERR);
    }
    else if (ivalue < 9 && ivalue >=0 )
    {
      err = ivalue;
      nvssetstr ( "err" , String( err));
    } 
  }
  else if ( argument.indexOf( "ert") >=0 )
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_ERT);
    }
    else
    {
      ert = value;
      nvssetstr ( "ert" , String( ert));
    } 
  }
  
  else if ( ( value.length() > 0 ) &&
            ( ( argument == "mp3track" ) ||           // Select a track from SD card?
              ( argument == "dlk" ) ) )           // Station in the form address:port
  {
    if ( argument.startsWith ( "mp3" ) )              // MP3 track to search for
    {
      if ( !SD_okay )                                 // SD card present?
      {
        strcpy ( reply, "Command not accepted!" ) ;   // Error reply
        return reply ;
      }
      value = getSDfilename ( value ) ;               // like "localhost/........"
    }

    if ( value == "?")
    {
      mqttpub.trigger (MQTT_DLK);
    }
//    else if (value == dlk && host == value)
//    {
//        Serial.println("Tiep tuc Play");
//    }
    else if (value == "0.0.0.0:0/")
    {        
      
      dlk = value;
      host = value ; 
      if (ban == 0)
      {
        sts=0;
        digitalWrite (MUTE, LOW);
      }
      if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                          PLAYLISTHEADER | PLAYLISTDATA ) )
  
        {
           datamode = STOPREQD ;                           // Request STOP
           playingstat=0;
           reload_accept=0;
           accept_try =0;
        }
       else accept_try = 0; 
    }
    else
    {
      if(cam_mic == false)
      {
        if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                          PLAYLISTHEADER | PLAYLISTDATA ) )
        {
          datamode = STOPREQD ;                           // Request STOP
        }
        accept_try = 1 ;
        reload_accept = 1;
        delay(1000);
        dlk = value;
        host = value ;                                    // Save it for storage and selection later
        vs1053player->getVolume() ;
        ini_block.reqvol = ini_block.reqvol + 1 ; 
        ini_block.reqvol = ini_block.reqvol - 1 ; 
        hostreq = true ;                                  // Force this station as new preset
        sprintf ( reply,
                  "Playing %s",                           // Format reply
                  host.c_str() ) ;
        utf8ascii ( reply ) ;                             // Remove possible strange characters
      }
    }
  }
  else if (argument.indexOf ( "wap" ) >= 0)
  {
    wap = value;
    nvssetstr ( "wifi_00",    String ( wap )  ) ; // Save device name

  }
  else if (argument.indexOf ("pr0") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR0);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_00" ) ;                    // Remove key
    }
    else 
    {
      pr0 = value;
      nvssetstr ( "preset_00",    String ( pr0 )  ) ; // Save device name
      nvssetstr ( "pr0",    String ( pr0 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr1") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR1);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_01" ) ;                    // Remove key
    }
    else 
    {
      pr1 = value;
      nvssetstr ( "preset_01",    String ( pr1 )  ) ; // Save device name
      nvssetstr ( "pr1",    String ( pr1 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr2") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR2);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_02" ) ;                    // Remove key
    }
    else 
    {
      pr2 = value;
      nvssetstr ( "preset_02",    String ( pr2 )  ) ; // Save device name
      nvssetstr ( "pr2",    String ( pr2 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr3") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR3);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_03" ) ;                    // Remove key
    }
    else 
    {
      pr3 = value;
      nvssetstr ( "preset_03",    String ( pr3 )  ) ; // Save device name
      nvssetstr ( "pr3",    String ( pr3 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr4") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR4);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_04" ) ;                    // Remove key
    }
    else 
    {
      pr4 = value;
      nvssetstr ( "preset_04",    String ( pr4 )  ) ; // Save device name
      nvssetstr ( "pr4",    String ( pr4 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr5") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR5);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_05" ) ;                    // Remove key
    }
    else 
    {
      pr5 = value;
      nvssetstr ( "preset_05",    String ( pr5 )  ) ; // Save device name
      nvssetstr ( "pr5",    String ( pr5 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr6") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR6);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_06" ) ;                    // Remove key
    }
    else 
    {
      pr6 = value;
      nvssetstr ( "preset_06",    String ( pr6 )  ) ; // Save device name
      nvssetstr ( "pr6",    String ( pr6 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr7") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR7);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_07" ) ;                    // Remove key
    }
    else 
    {
      pr7 = value;
      nvssetstr ( "preset_07",    String ( pr7 )  ) ; // Save device name
      nvssetstr ( "pr7",    String ( pr7 )  ) ; // Save device na
    }
  }
  else if (argument.indexOf ("pr8") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_PR8);     
    }
    else if (value == "0")
    {
      nvs_erase_key ( nvshandle, "preset_08" ) ;                    // Remove key
    }
    else 
    {
      pr8 = value;
      nvssetstr ( "preset_08",    String ( pr8 )  ) ; // Save device name
      nvssetstr ( "pr8",    String ( pr8 )  ) ; // Save device name
    }
  }
  else if (argument.indexOf ("pr9") >= 0)
  {
    if (value == "?")
    {
       mqttpub.trigger (MQTT_PR9);     
    }
    else if (value == "0")
    {
         nvs_erase_key ( nvshandle, "preset_09" ) ;                    // Remove key
    }
    else 
    {
      pr9 = value;
      nvssetstr ( "preset_09",    String ( pr9 )  ) ; // Save device name
      nvssetstr ( "pr9",    String ( pr9 )  ) ; // Save device name
    }

  }
  else if (argument.indexOf ("preset_09") >= 0)
  {
    pr9 = value;
  }

  else if (argument.indexOf ( "rst" ) >= 0)
  {
    if ( value == "1")
    {
      resetreq = true ;                                 // Reset all
    }
  }

  else if ( argument.indexOf ("plk") >= 0)
  {
    if ( value == "?")
    {
      mqttpub.trigger (MQTT_PLK);
    }
  }
  else if( argument.indexOf ("inf") >=0)
  {
    if ( value == "?")
    {
          mqttpub.trigger (MQTT_STREAMTITLE); 
          mqttpub.trigger (MQTT_ICY);
    }

  }
  else if ( argument.indexOf( "snd") >=0 )
  {
      snd = value;
      Serial.println(snd);
  } 
  else if ( argument == "debug" )                     // debug on/off request?
  {
    DEBUG = ivalue ;                                  // Yes, set flag accordingly
    nvssetstr ( "debug",    String ( DEBUG )  ) ; // Save device name
  }  
  else if (argument.indexOf ("linein") > 0)
  {
    linein = ivalue;
  }

  //-------------------------------Bo sung MANAGER-----------------------------
  else if (argument.indexOf ( "dnm" ) >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_DNM);
    }
    else
    {
      dnm = value;
      nvssetstr ( "dnm",    String ( dnm )  ) ; // Save device name
    }
  }


  else if (argument.indexOf ( "mac" ) >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_MAC);
    }
  }
  else if (argument.indexOf ( "ver" ) >= 0)
  {
    if (value == "?")
    {
//      mqttclient.publish("return_status", version_firmware);
        mqttpub.trigger (MQTT_VER);
      //  mqttclient.publish(topic, version_firmware);
    }
  }
      else if ( argument.startsWith ( "ota" ) )        // Update request
  {
    if ( value == "1")
    {
      datamode = STOPREQD ;                           // Request STOP
      delay(2000);
      updatereq = true ;                                // Reset all
    }
  }
     else if (argument.indexOf ( "otf" ) >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_OTF);
    }
  }
    else if (argument. indexOf ( "ban") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger ( MQTT_BAN);
    }

    else if ( value == "0")
    {
      ban = ivalue;
      nvssetstr( "ban", String (ban)) ;
      datamode = STOPREQD ;
      sts=0;
      mqttreconnect() ; 
    }
    else if (value == "1")
    {
      if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                        PLAYLISTHEADER | PLAYLISTDATA ) )

      {
         datamode = STOPREQD ;                           // Request STOP
         playingstat=0;
         reload_accept=0;
         accept_try =0;
      }
      else
      {
         accept_try =0;
      }
      digitalWrite (MUTE, LOW);
      ban = ivalue;
      nvssetstr( "ban", String (ban)) ;
      sts=99;
    }
  }
  else if (argument.indexOf ( "rst" ) >= 0)
  {
    if ( value == "1")
    {
      resetreq = true ;                                 // Reset all
    }
  }
  else if ( argument.indexOf("dur") >=0)
  {
    if (value ="?")
    {
      mqttpub.trigger (MQTT_DUR);
    }
  }
  //-------------------------------------------------------------------
  else
  {
    rcv = argument.c_str();
    mqttpub.trigger ( MQTT_RCV);
    sprintf ( reply, "%s called with illegal parameter: %s",
              NAME, argument.c_str() ) ;
  }
  return reply ;                                      // Return reply to the caller
}
//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient, Serial or MQTT.                         *
// Version for handling string with: <parameter>=<value>                                           *
//**************************************************************************************************
const char* analyzeCmd_manu_broker ( const char* str )
{
  char*        value ;                           // Points to value after equalsign in command
  const char*  res ;                             // Result of analyzeCmd
  value = strstr ( str, "=" ) ;                  // See if command contains a "="
  if ( value )
  {
    *value = '\0' ;                              //
    res = analyzeCmd_manu_broker ( str, value + 1 ) ;        // Analyze command and handle it
    *value = '=' ;                               // Restore equal sign
  }
  else
  {
    res = analyzeCmd_manu_broker ( str, "0" ) ;              // No value, assume zero
  }
  return res ;
}

const char* analyzeCmd_manu_broker ( const char* par, const char* val )
{

  String             argument ;                       // Argument as string
  String             value ;                          // Value of an argument as a string
  int                ivalue ;                         // Value of argument as an integer
  static char        reply[180] ;                     // Reply to client, will be returned
  uint8_t            oldvol ;                         // Current volume
  bool               relative ;                       // Relative argument (+ or -)
  String             tmpstr ;                         // Temporary for value
  uint32_t           av ;                             // Available in stream/file
  const char*        payload ;                                       // Points to payload
  char               strvar[120];
  blset ( true ) ;                                    // Enable backlight of TFT
  strcpy ( reply, "Command accepted" ) ;              // Default reply
  argument = String ( par ) ;                         // Get the argument
  chomp ( argument ) ;                                // Remove comment and useless spaces
  if ( argument.length() == 0 )                       // Lege commandline (comment)?
  {
    return reply ;                                    // Ignore
  }
  if ( argument.length() > 120 )                       // Lege commandline (comment)?
  {
    return reply ;                                    // Ignore
  }
  argument.toLowerCase() ;                            // Force to lower case
  value = String ( val ) ;                            // Get the specified value
  chomp ( value ) ;                                   // Remove comment and extra spaces
  ivalue = value.toInt() ;                            // Also as an integer
  ivalue = abs ( ivalue ) ;                           // Make positive
  //**************************************************************************************
  //                               Manufacture                                           *
  //**************************************************************************************
  if (argument.indexOf ( "dnm" ) >= 0)
  {  
    
    if (value == "?")
    {
      sprintf ( strvar, "%s_DNM=%s", ini_block.mqttprefix.c_str(), dnm);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      dnm = value;
      nvssetstr ( "dnm",    String ( dnm )  ) ; // Save device name
      sprintf ( strvar, "%s_DNM=%s", ini_block.mqttprefix.c_str(), dnm);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }

  else if (argument.indexOf ( "did" ) >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_DID=%s", ini_block.mqttprefix.c_str(), did);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      did = value;
      nvssetstr ( "did",    String ( did )  ) ; // Save device ID
      sprintf ( strvar, "%s_DID=%s", ini_block.mqttprefix.c_str(), did);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
  else if (argument.indexOf ( "ver" ) >= 0)
  {
    if (value == "?")
    {
      mqttclient1.publish(topic_manu, version_firmware);
    }
  }
  else if (argument.indexOf ("gps") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_GPS);
    }
    else
    {
      gps = value;
      nvssetstr( "gps",  String(gps));  // Save time zone
      mqttclient1.publish(topic_manu, rt_recive);
    }
  }
  else if (argument.indexOf ("tmz") >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_TMZ);
    }
    else
    {
      ini_block.clk_offset = ivalue;
      nvssetstr( "tmz",  String(ini_block.clk_offset));  // Save time zone
      mqttclient1.publish(topic_manu, rt_recive);

    }
  }
  else if (argument.indexOf ( "reg" ) >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_REG);
    }
    else
    {
      reg = value;
      nvssetstr ( "reg",    String ( reg )  ) ; // Save current preset
      mqttclient1.publish(topic_manu, rt_recive);

    }
  }
  else if (argument.indexOf ( "epd" ) >= 0)
  {
    if (value == "?")
    {
      mqttpub.trigger (MQTT_EPD);
    }
    else
    {
      epd = value;
      nvssetstr ( "epd",    String ( epd )  ) ; // Save current preset
      mqttclient1.publish(topic_manu, rt_recive);

    }
  }
  else if (argument. indexOf ( "ban") >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_BAN=%d", ini_block.mqttprefix.c_str(), ban);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else if ( value == "0")
    {
      ban = ivalue;
      nvssetstr( "ban", String (ban)) ;
      datamode = STOPREQD ;
      sts=0;
      mqttreconnect() ;
      sprintf ( strvar, "%s_BAN=%d", ini_block.mqttprefix.c_str(), ban);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);

    }
    else if ( value == "1" )
    {
      ban = ivalue;
      nvssetstr( "ban", String (ban)) ;
      sprintf ( strvar, "%s_BAN=%d", ini_block.mqttprefix.c_str(), ban);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);

    }
  }
  else if ( argument.indexOf("dur") >=0)
  {
    if (value ="?")
    {
      sprintf ( strvar, "%s_DUR=%d", ini_block.mqttprefix.c_str(), dur);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
    else if ( argument.startsWith ( "ota" ) )        // Update request
  {
    if ( value == "1")
    {
      datamode = STOPREQD ;                           // Request STOP
      delay(2000);
      updatereq = true ;                                // Reset all
    }
  }
  else if ( argument.indexOf("otc") >= 0)
  {
    if ( value == "?")
    {
      mqttpub.trigger(MQTT_OTC);
    }
    else
    {
      otc = ivalue;
      nvssetstr ("otc", String (otc));
      mqttclient1.publish(topic_manu, rt_recive);
    }
  }
  else if (argument.indexOf("pmp") >=0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_PMP=%d", ini_block.mqttprefix.c_str(), pmp);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      pmp = ivalue;
      nvssetstr ("pmp", String (pmp));
      sprintf ( strvar, "%s_PMP=%d", ini_block.mqttprefix.c_str(), pmp);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
  else if (argument.indexOf ( "cip" ) >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_CIP=%s", ini_block.mqttprefix.c_str(), ini_block.mqttbroker.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      ini_block.mqttbroker = value ;                  // Yes, set broker accordingly
      nvssetstr ( "cip",    String ( ini_block.mqttbroker )  ) ; // Save current mqttbroker
      sprintf ( strvar, "%s_CIP=%s", ini_block.mqttprefix.c_str(), ini_block.mqttbroker.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
  else if (argument.indexOf ( "cpr" ) >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_CPR=%d", ini_block.mqttprefix.c_str(), ini_block.mqttport);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      ini_block.mqttport = ivalue ;                  // Yes, set broker accordingly
      nvssetstr ( "cpr",    String ( ini_block.mqttport )  ) ; // Save current mqttbroker
      sprintf ( strvar, "%s_CPR=%d", ini_block.mqttprefix.c_str(), ini_block.mqttport);
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
  else if (argument.indexOf ( "cnm" ) >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_CNM=%s", ini_block.mqttprefix.c_str(), ini_block.mqttuser.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      ini_block.mqttuser = value ;                  // Yes, set broker accordingly
      nvssetstr ( "cnm",    String ( ini_block.mqttuser )  ) ; // Save current mqttbroker
      sprintf ( strvar, "%s_CNM=%s", ini_block.mqttprefix.c_str(), ini_block.mqttuser.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
  else if (argument.indexOf ( "cpw" ) >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_CPW=%s", ini_block.mqttprefix.c_str(), ini_block.mqttpasswd.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      ini_block.mqttpasswd = value.c_str() ;                  // Yes, set broker accordingly
      nvssetstr ( "cpw",    String ( ini_block.mqttpasswd )  ) ; // Save current mqttbroker
      sprintf ( strvar, "%s_CPW=%s", ini_block.mqttprefix.c_str(), ini_block.mqttpasswd.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
  else if (argument.indexOf ( "ctp" ) >= 0)
  {
    if (value == "?")
    {
      sprintf ( strvar, "%s_CTP=%s", ini_block.mqttprefix.c_str(), ctp.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
    else
    {
      ctp = value ;                  // Yes, set broker accordingly
      nvssetstr ( "ctp",    String ( ctp )  ) ; // Save current mqttbroker
      sprintf ( strvar, "%s_CTP=%s", ini_block.mqttprefix.c_str(), ctp.c_str());
      payload = strvar;
      mqttclient1.publish(topic_manu, payload);
    }
  }
    else if (argument.indexOf ( "dfc" ) >= 0)
  {
    if ( value == "1")
    {
      Default_config ();
      mqttclient1.publish(topic_manu, rt_recive);
    }
  }
  else if (argument.indexOf ( "rst" ) >= 0)
  {
    if ( value == "1")
    {
      resetreq = true ;                                 // Reset all
    }
  }

  else
  {
    sprintf ( reply, "%s called with illegal parameter: %s",
              NAME, argument.c_str() ) ;
  }
  return reply ;
}

//**************************************************************************************************
//                                     H T T P H E A D E R                                         *
//**************************************************************************************************
// Set http headers to a string.                                                                   *
//**************************************************************************************************
String httpheader ( String contentstype )
{
  return String ( "HTTP/1.1 200 OK\nContent-type:" ) +
         contentstype +
         String ( "\n"
                  "Server: " NAME "\n"
                  "Cache-Control: " "max-age=3600\n"
                  "Last-Modified: " VERSION "\n\n" ) ;
}


//**************************************************************************************************
//* Function that are called from spftask.                                                         *
//* Note that some device dependent function are place in the *.h files.                           *
//**************************************************************************************************

//**************************************************************************************************
//                                      D I S P L A Y I N F O                                      *
//**************************************************************************************************
// Show a string on the LCD at a specified y-position (0..2) in a specified color.                 *
// The parameter is the index in tftdata[].                                                        *
//**************************************************************************************************
void displayinfo ( uint16_t inx )
{
  uint16_t       width = dsp_getwidth() ;                  // Normal number of colums
  scrseg_struct* p = &tftdata[inx] ;
  uint16_t len ;                                           // Length of string, later buffer length

  if ( inx == 0 )                                          // Topline is shorter
  {
    width += TIMEPOS ;                                     // Leave space for time
  }
  if ( tft )                                               // TFT active?
  {
    dsp_fillRect ( 0, p->y, width, p->height, BLACK ) ;    // Clear the space for new info
    if ( ( dsp_getheight() > 64 ) && ( p->y > 1 ) )        // Need and space for divider?
    {
      dsp_fillRect ( 0, p->y - 4, width, 1, GREEN ) ;      // Yes, show divider above text
    }
    len = p->str.length() ;                                // Required length of buffer
    if ( len++ )                                           // Check string length, set buffer length
    {
      char buf [ len ] ;                                   // Need some buffer space
      p->str.toCharArray ( buf, len ) ;                    // Make a local copy of the string
      utf8ascii ( buf ) ;                                  // Convert possible UTF8
      dsp_setTextColor ( p->color ) ;                      // Set the requested color
      dsp_setCursor ( 2, p->y ) ;                          // Prepare to show the info
      dsp_println ( buf ) ;                                // Show the string
    }
  }
}


//**************************************************************************************************
//                                         G E T T I M E                                           *
//**************************************************************************************************
// Retrieve the local time from NTP server and convert to string.                                  *
// Will be called every second.                                                                    *
//**************************************************************************************************
void gettime()
{
  static int16_t delaycount = 0 ;                           // To reduce number of NTP requests
  static int16_t retrycount = 100 ;

  if ( tft )                                                // TFT used?
  {
    if ( timeinfo.tm_year )                                 // Legal time found?
    {
      sprintf ( timetxt, "%02d:%02d:%02d",                  // Yes, format to a string
                timeinfo.tm_hour,
                timeinfo.tm_min,
                timeinfo.tm_sec ) ;
    }
    if ( --delaycount <= 0 )                                // Sync every few hours
    {
      delaycount = 7200 ;                                   // Reset counter
      if ( timeinfo.tm_year )                               // Legal time found?
      {
        dbgprint ( "Sync TOD, old value is %s", timetxt ) ;
      }
      dbgprint ( "Sync TOD" ) ;
      if ( !getLocalTime ( &timeinfo ) )                    // Read from NTP server
      {
        dbgprint ( "Failed to obtain time!" ) ;             // Error
        timeinfo.tm_year = 0 ;                              // Set current time to illegal
        if ( retrycount )                                   // Give up syncing?
        {
          retrycount-- ;                                    // No try again
          delaycount = 5 ;                                  // Retry after 5 seconds
        }
      }
      else
      {
        //        sprintf ( timetxt, "%02d:%02d:%02d",                // Format new time to a string
        //                  timeinfo.tm_hour,
        //                  timeinfo.tm_min,
        //                  timeinfo.tm_sec ) ;
        sprintf ( timetxt, "%02d:%02d:%02d",                // Format new time to a string
                  timeinfo.tm_mday,
                  timeinfo.tm_mon,
                  timeinfo.tm_year ) ;


        dbgprint ( "Sync TOD, new value is %s", timetxt ) ;
      }
    }
  }
}


//**************************************************************************************************
//                                H A N D L E _ T F T _ T X T                                      *
//**************************************************************************************************
// Check if tft refresh is requested.                                                              *
//**************************************************************************************************
bool handle_tft_txt()
{
  for ( uint16_t i = 0 ; i < TFTSECS ; i++ )              // Handle all sections
  {
    if ( tftdata[i].update_req )                          // Refresh requested?
    {
      displayinfo ( i ) ;                                 // Yes, do the refresh
      dsp_update() ;                                      // Updates to the screen
      tftdata[i].update_req = false ;                     // Reset request
      return true ;                                       // Just handle 1 request
    }
  }
  return false ;                                          // Not a single request
}


//**************************************************************************************************
//                                     P L A Y T A S K                                             *
//**************************************************************************************************
// Play stream data from input queue.                                                              *
// Handle all I/O to VS1053B during normal playing.                                                *
// Handles display of text, time and volume on TFT as well.                                        *
//**************************************************************************************************
void playtask ( void * parameter )
{
  while ( true )
  {
    if ( xQueueReceive ( dataqueue, &inchunk, 5 ) )
    {
      while ( !vs1053player->data_request() )                       // If FIFO is full..
      {
        vTaskDelay ( 10 ) ;                                          // Yes, take a break
      }
      switch ( inchunk.datatyp )                                    // What kind of chunk?
      {
        case QDATA:
          claimSPI ( "chunk" ) ;                                    // Claim SPI bus
          vs1053player->playChunk ( inchunk.buf,                    // DATA, send to player
                                    sizeof(inchunk.buf) ) ;
          releaseSPI() ;                                            // Release SPI bus
          totalcount += sizeof(inchunk.buf) ;                       // Count the bytes
          break ;
        case QSTARTSONG:
           playingstat = 1 ; 
           vs1053player->getVolume() ;
          ini_block.reqvol = ini_block.reqvol + 1 ; 
          ini_block.reqvol = ini_block.reqvol - 1 ; 
         if (stt_ply == 1 && playingstat==1)
          {
            sts = 1;
            sts_stop_done =false;
          }
          claimSPI ( "startsong" ) ;                                // Claim SPI bus
          vs1053player->startSong() ;                               // START, start player
          releaseSPI() ;                                            // Release SPI bus
          break ;
        case QSTOPSONG:
          playingstat = 0 ;                                         // Status for MQTT
          if (  stt_ply == 1 && playingstat==0)
          {
            sts = 0;
          }
          claimSPI ( "stopsong" ) ;                                 // Claim SPI bus
          vs1053player->setVolume ( 0 ) ;                           // Mute
          vs1053player->stopSong() ;                                // STOP, stop player
          releaseSPI() ;                                            // Release SPI bus
          sts_stop_done = true;
//          dbgprint("Stop ben nay");
          vTaskDelay ( 500 / portTICK_PERIOD_MS ) ;                 // Pause for a short time
          break ;
        default:
          break ;
      }
    }
    //esp_task_wdt_reset() ;                                        // Protect against idle cpu
  }
  //vTaskDelete ( NULL ) ;                                          // Will never arrive here
}


//**************************************************************************************************
//                                   H A N D L E _ S P E C                                         *
//**************************************************************************************************
// Handle special (non-stream data) functions for spftask.                                         *
//**************************************************************************************************

void handle_spec()
{
  // Do some special function if necessary
  if ( dsp_usesSPI() )                                        // Does display uses SPI?
  {
    claimSPI ( "hspectft" ) ;                                 // Yes, claim SPI bus
  }
  if ( tft )                                                  // Need to update TFT?
  {
    handle_tft_txt() ;                                        // Yes, TFT refresh necessary
    dsp_update() ;                                            // Be sure to paint physical screen
  }
  if ( dsp_usesSPI() )                                        // Does display uses SPI?
  {
    releaseSPI() ;                                            // Yes, release SPI bus
  }
  if ( time_req && NetworkFound )                             // Time to refresh time?
  {
    gettime() ;                                               // Yes, get the current time
  }
  claimSPI ( "hspec" ) ;                                      // Claim SPI bus
  if ( muteflag )                                             // Mute or not?
  {
    vs1053player->setVolume ( 0 ) ;                           // Mute
  }
  else
  {
    vs1053player->setVolume ( ini_block.reqvol ) ;            // Unmute
  }
  if ( reqtone )                                              // Request to change tone?
  {
    reqtone = false ;
    vs1053player->setTone ( ini_block.rtone ) ;               // Set SCI_BASS to requested value
  }
  if ( time_req )                                             // Time to refresh timetxt?
  {
    time_req = false ;                                        // Yes, clear request
    if ( NetworkFound  )                                      // Time available?
    {
      displaytime ( timetxt ) ;                               // Write to TFT screen
      displayvolume() ;                                       // Show volume on display
     // displaybattery() ;                                      // Show battery charge on display

    }
  }
  releaseSPI() ;                                              // Release SPI bus
  if ( mqtt_on )
  {
    if ( !mqttclient.connected() )                            // See if connected
    {
      mqttreconnect() ;                                       // No, reconnect
    }
    else
    {
      mqttpub.publishtopic() ;                                // Check if any publishing to do
    }
  }
   if ( mqtt_on_manu)
  {
    if ( !mqttclient1.connected() )                            // See if connected
    {
      mqttreconnect1() ;
    }
    else
    {
      mqttpub.publishtopic() ;                                // Check if any publishing to do
    }
  }
}


//**************************************************************************************************
//                                     S P F T A S K                                               *
//**************************************************************************************************
// Handles display of text, time and volume on TFT.                                                *
// Handles ADC meassurements.                                                                      *
// This task runs on a low priority.                                                               *
//**************************************************************************************************
void spftask ( void * parameter )
{
  while ( true )
  {
    //handle_spec() ;                                                 // Maybe some special funcs?
    vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;                       // Pause for a short time
    adcval = ( 15 * adcval +                                        // Read ADC and do some filtering
               adc1_get_raw ( ADC1_CHANNEL_3) ) / 16 ;
  }
  //vTaskDelete ( NULL ) ;                                          // Will never arrive here
}


void  time_online ()
{
 static uint32_t     oldtime = 60000 ;                            // Time in millis previous interrupt
  if ((millis() - oldtime) < 60000)
  {
   return ;
  }
  oldtime = millis();
    dur ++;
    
}

//------------------------------------retry conect--------------------------------------
void retry()
{
  if ( accept_try == 1)
  {
    hostreq = true;
  }

}
//----------------------------------------------------------------------------------------

void Sts_Pub_Change()
{
 
  if ( oldsts!= sts)                        // Volume change?
  {
    mqttpub.trigger ( MQTT_STS ) ;                      // Request publish VOLUME
      oldsts = sts ;                            // Remember publishe volume
  }

}
//**************************************************************************************************
//                                      H A N D L E I P P U B                                      *
//**************************************************************************************************
// Handle publish op STS to MQTT.  This will happen every 5 minutes.                               *
//**************************************************************************************************
void handleSTSPub()
{
  char buffer_mac[20];
  static uint32_t pubtime = 300000 ;                       // Limit save to once per 10 minutes

  if ( ( millis() - pubtime ) < pmp )                   // 600 sec is 10 minutes
  {
    return ;
  }
  pubtime = millis() ;                                     // Set time of last publish
  mqttpub.trigger ( MQTT_STS) ;                            // Request re-publish IP``
}


//***************************************************************************************************
//                                    WAIT TO RESET                                                 *
//***************************************************************************************************
void wait_reset()
{
  static uint32_t pubtime = 0 ;                         // Limit save to once per 10 seconds
  static char     reply_signal[180];
  if(sts_net == 1){
    
    if ( millis() >600000 )                    // 5  minute
    {
        resetreq =true;
    }
    
  }

}

void Default_config ()
{  
   dnm="Vietbroadcast.vn";
   nvssetstr ( "dnm" , String(dnm));
   did="";
   nvssetstr ( "did" , String( did));
   gps="";
   nvssetstr ( "gps" , String( gps));
   reg="01/01/2021";
   nvssetstr ( "reg" , String( reg));
   epd="01/01/2022";
   ban = 0;
   nvssetstr ( "ban" , String( ban));
   ota=0;
   nvssetstr ( "ota" , String( ota));
   pmp=5000;
   nvssetstr ( "pmp" , String( pmp));
   ini_block.mqttbroker="mqtt.mysignage.vn";
   nvssetstr ( "cip" , String( ini_block.mqttbroker));
   ini_block.mqttport=1883;
   nvssetstr ( "cpr" , String(ini_block.mqttport));
   ini_block.mqttuser="vietbroadcast";
   nvssetstr ( "cnm" , String( ini_block.mqttuser));
   ini_block.mqttpasswd="Viet@123456";
   nvssetstr ( "cpw" , String( ini_block.mqttpasswd));
   ctp="return_status";
   nvssetstr ( "ctp" , String( ctp)); 
   ini_block.reqvol=90;
   nvssetstr ( "volume", String ( ini_block.reqvol ) );    // Save current volue
   ini_block.micvol=90;
   nvssetstr ( "volmic", String ( ini_block.micvol ) );    // Save current volue
   ini_block.streamvol=90;
   nvssetstr ( "volstream", String ( ini_block.streamvol ) );    // Save current volue
   mod=0;
   nvssetstr ( "mod" , String( mod));
   dlk="";
   nvssetstr ( "dlk" , String( dlk));
}

void get_inf5s()
{
  if ( (unsigned long) (millis() - time1) > 5000  )
    {
        if (flag_dsp == 0 )
        {
          flag_dsp=1;
          tftset ( 0, "" ) ;    
          tftset ( 0, dsp_mac ) ;    
        }
        else if (flag_dsp == 1)
        {
          flag_dsp=2;
          tftset ( 0, "" ) ;    
          tftset ( 0, dsp_ip ) ; 
        }
        else
        {
          flag_dsp=0;
          tftset ( 0, "" ) ;    
          tftset ( 0, dsp_signal ) ; 
        }
      time1 = millis();
    }
}
void get_signage()
{
  static uint32_t pubtime = 5000 ;                         // Limit save to once per 10 seconds
  static char     reply_signal[10];
  if ( ( millis() - pubtime ) < 5000)                    // 10 seconds
  {
    return ;
  }
  pubtime = millis() ;                                     // Set time of last publish
  if (state_lan==2)
  {
        sprintf(reply_signal,"%3d",WiFi.RSSI());
        dsp_signal = "SIGNAL:" + String (reply_signal) + " db";
        if (String (reply_signal).toInt() == 0 ) resetreq =true; 
  }
}

uint8_t Check_MicOn_Pin(void){
    return digitalRead(mic_det);
}
uint8_t Check_VolUp_Pin(void){
    return digitalRead(vol_up);
}

uint8_t Check_VolDown_Pin(void){
    return digitalRead(vol_down);
}
void IO_Tick(void)
{
  uint8_t            oldvol ;                         // Current volume
  bool               relative ;                       // Relative argument (+ or -)
  static char        reply[180] ;                     // Reply to client, will be returned

  
        /*Polling vol up / vol down button Press: 1 Release: 0*/

        
    if(Check_VolUp_Pin() == 0){
        tmpVolUpCnt = (tmpVolUpCnt << 1) | 1;
     // nwy_ext_echo("\rVol up button Press");
    }
    else tmpVolUpCnt = 0;

    if(tmpVolUpCnt >= time_button)
    {
        dbgprint("Confirm up button Press");
        tmpVolUpCnt = 0;
        oldvol = vs1053player->getVolume() ;
        if(MicOnStatus == 0){
        ini_block.streamvol = oldvol;
        ini_block.streamvol ++;           // Up/down by 0.5 or more dB
            if ( ini_block.streamvol > 100 ) ini_block.streamvol = 100 ;                    // Limit to normal values
            sprintf ( reply, "Volume stream is now %d",              // Reply new volume
                      ini_block.streamvol ) ;
            ini_block.reqvol = ini_block.streamvol;
          }
         else{
            ini_block.micvol = oldvol;
            ini_block.micvol ++;                               // Up/down by 0.5 or more dB
            if ( ini_block.micvol > 100 ) ini_block.micvol = 100 ;                       // Limit to normal values
              sprintf ( reply, "Volume mic is now %d",           // Reply new volume
                        ini_block.micvol ) ;
            ini_block.reqvol = ini_block.micvol;         
          }
    }
    if(Check_VolDown_Pin() == 0){
      tmpVolDownCnt = (tmpVolDownCnt << 1) | 1;
       // dbgprint("\rVol down button Press");
    }
    else
    {
      tmpVolDownCnt = 0;
    }
    if(tmpVolDownCnt >= time_button){
      dbgprint("Confirm Vol down button Press");
      tmpVolDownCnt = 0;
      oldvol = vs1053player->getVolume() ;
     if(MicOnStatus == 0){
       ini_block.streamvol = oldvol;
      if(ini_block.streamvol > 0) ini_block.streamvol --;           // Up/down by 0.5 or more dB
            if ( ini_block.streamvol < 2 ) ini_block.streamvol = 0 ;                    // Limit to normal values
            sprintf ( reply, "Volume stream is now %d",              // Reply new volume
                      ini_block.streamvol ) ;
            ini_block.reqvol = ini_block.streamvol;     
          }
         else{
          ini_block.micvol = oldvol;
          if (ini_block.micvol>0) ini_block.micvol --;                               // Up/down by 0.5 or more dB
            if ( ini_block.micvol < 2 ) ini_block.micvol = 0 ;                       // Limit to normal values
              sprintf ( reply, "Volume mic is now %d",           // Reply new volume
                        ini_block.micvol ) ;
            ini_block.reqvol = ini_block.micvol;  
          }
    }
}
void Mic_tick()
{      
  if(Check_MicOn_Pin() == 0){
        tmpMicOnCnt = (tmpMicOnCnt << 1) | 1;
        tmpMicOffCnt = 0;
    }
    else{
        tmpMicOffCnt = (tmpMicOffCnt << 1) | 1;
        tmpMicOnCnt = 0;
    }
    if(tmpMicOnCnt >= 0x3FF){
      //  nwy_ext_echo("\rMIC ON");
        MicOnStatus = 1;
        tmpMicOnCnt = 0;
    }
    if(tmpMicOffCnt >= 0x3FF){
        // nwy_ext_echo("\rMIC OFF");
        MicOnStatus = 0;
        tmpMicOffCnt = 0;
    }
    if(MicOnStatus == 0xFF) return;

    /*MicOnStatus doi trang thai 0->1 */
    if(LastMicOnStatus != MicOnStatus && MicOnStatus == 1){
      if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                        PLAYLISTHEADER | PLAYLISTDATA ) )
      {
         datamode = STOPREQD ;                           // Request STOP
         playingstat=0;
         reload_accept=0;
         accept_try =0;
      }
      dbgprint("\rMic Detect cam mic");
      cam_mic = true;
      digitalWrite (MUTE, HIGH);
    /*MicOnSttus doi trang thai 1->0 *       
     * 
     */
        delay(1000);
        if (sts_stop_done == true)
        {      
               dbgprint("\rChuan bi stream to mic");
               vs1053player -> SwitchToMic();
               LastMicOnStatus = MicOnStatus;
        }
    }
     if(LastMicOnStatus != MicOnStatus && MicOnStatus == 0){
        digitalWrite (MUTE, LOW);
        dbgprint("\rMic Detect rut mic");
        cam_mic = false;
        mqttreconnect() ;
        vs1053player -> SwitchToStream();
        LastMicOnStatus = MicOnStatus;
     }
}
void led_tick()
{
    switch (mode_led){
      case 0:
           time_led_do = 4000;        // Che do diem truy cap
           break;
      case 1:
           time_led_do = 2000;       // Che do 4G
           break;
      case 2:
           time_led_do = 3000;      // Che do Wifi
           break;
      case 3: 
           time_led_do = 1000;      // Che do LAN
           break;
      default:
      break;
    }
    switch (sts){
      case 0:
           time_led_xanh_on = 100;
           time_led_xanh_off = 900 ;                   
           break;
      case 1:
           time_led_xanh_on = 100;
           time_led_xanh_off = 100;    
           break;
      case 2:
           time_led_xanh_on = 900;
           time_led_xanh_off = 100;  
           break;
      case 99: 
           time_led_xanh_on = 3000;
           time_led_xanh_off = 3000;  
           break;
      default:
      break;
    }
    if ( (unsigned long) (millis() - time1) > time_led_do )
    {
        if ( digitalRead(led_do) == LOW )
        {
            digitalWrite(led_do, HIGH);
        } else {
            digitalWrite(led_do, LOW );
        }
        time1 = millis();
    }

   if(LEDState == HIGH)
   {
       if ( (unsigned long) (millis() - time2) >= time_led_xanh_on )
       {
          LEDState = LOW ;
          time2 = millis();
       }
   }
   else
   {
        if ( (unsigned long) (millis() - time2) >= time_led_xanh_off )
       {
          LEDState = HIGH ;
          time2 = millis();
       }
   }
  digitalWrite(led_xanh,LEDState);

}
//**************************************************************************************************
//                                      H A N D L E I P P U B                                      *
//**************************************************************************************************
// Handle publish op STS to MQTT.  This will happen every 5 minutes.                               *
//**************************************************************************************************
void GetMac()
{
  char buffer_mac[20];
  static uint32_t pubtime = 0 ;                       // Limit save to once per 10 minutes

  if ( ( millis() - pubtime ) < 5000 )                   // 600 sec is 10 minutes
  {
    return ;
  }
  pubtime = millis() ;                                     // Set time of last publish
  sprintf (buffer_mac,"MAC=%s\n\r",ini_block.mqttprefix.c_str());
  Serial.print(buffer_mac);
}
