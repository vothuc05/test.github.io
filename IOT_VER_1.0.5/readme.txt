/// ESP32dev Signal  Wired to LCD        Wired to VS1053      SDCARD   Wired to the rest                     LAN 8720
// -------- ------  --------------      -------------------  ------   ---------------                  -------------
// GPIO32           -                    pin 1 XDCS            -       -                               -
// GPIO13            -                   pin 2 XCS             -       -                               -
// GPIO14            -                   pin 4 DREQ            -       -                               -

// GPIO22           -                   -                     CS      -                               -
// GPIO16   RXD2    -                   -                     -       TX of NEXTION (if in use)       -
// GPIO17   TXD2    -                   -                     -       RX of NEXTION (if in use)       -
// GPIO18   SCK     pin 5 CLK or SCK    pin 5 SCK             CLK     -                               -
// GPIO33   MISO    -                   pin 7 MISO            MISO    -                               -
// GPIO23   MOSI    pin 4 DIN or SDA    pin 6 MOSI            MOSI    -                               -
// GPIO15           pin 2 CS            -                     -       -                               -
// GPI03    RXD0    -                   -                     -       Reserved serial input           -
// GPIO1    TXD0    -                   -                     -       Reserved serial output          -
// GPIO34   -       -                   -                     -       Down volume      -
// GPIO35   -       -                   -                     -       Mic Detec			      -
// GPIO39   -       -                   -                     -       NUll             		      -
// GPIO00   -       -                   -                     -       Up volume                       -
// GPIO12           -                   -       	      -       MUTE                            -
// GPIO22                                                                                             - EMAC_TXD1   : TX1
// GPIO19                                                                                             - EMAC_TXD0   : TX0
// GPIO21                                                                                             - EMAC_TX_EN  : TX_EN
// GPIO26                                                                                             - EMAC_RXD1   : RX1
// GPIO25                                                                                             - EMAC_RXD0   : RX0
// GPIO27                                                                                             - EMAC_RX_DV  : CRS
// GPIO16                                                                                             - EMAC_TX_CLK : nINT/REFCLK (50MHz) - 4k7 Pullup
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

__FIRMWARE_NAME__    "NM:VNR_1101E"
__FIRMWARE_VERSION__ "FW:TU_1.0.2"
__HARDWARE_VERSION__ "HW:V2.3"
Hướng dẫn sử dụng
Kết nối điểm truy cập với name:  Vietbroadcast
			  passs: Vietbroadcast
Cấu hình wifi

B1: Truy cập vào địa chỉ IP "192.168.4.1"
B2: Chuyến sang trang config
B3: Sửa điểm wifi cần kết nối được dạng (name/pass)
B4: Nhấn Save
B5: Nhấn restart

Sử dụng trên website
2 broker
Broker 1:
MQTT : mqtt.mysignage.vn
USER : vietbroadcast
PASS : Viet@123456
Broker 2:
MQTT : net-radio.vov.link
USER : vietbroadcast
PASS : Viet@12345656
OTA Server  : ota.mysignage.vn 
Path OTA: "VNR_1101E/Radio/Karadio_update.bin"
MQTT TOPIC
	- Thiết bị pub topic :  return_status 
		       message: MAC_item=value
	- Thiết bị sub topic :  MAC/command
		       message: item=value
Ngày 20/11/2021
Update:
	- Sửa lỗi không mở điểm truy cập khi nhấn giảm âm lượng (done)
	- Cập nhật ban thiết bị gửi STS=99 (done)
	- Update file bin lên server(done)
Việc cần làm
	- Sửa trạng thái đèn nháy theo LAN, Wifi
	
Ngày 22/11/2021
Update:
	- Sửa lỗi bật module 4G (done)
	- Sửa lỗi Ban=0 nhưng STS vẫn bằng 99 (done)
	- Sửa trạng thái đèn đỏ nháy theo LAN (1s), Wifi(3s),4g(2s),AP(4s) (done)
Việc cần làm
	- Mode mic chưa hoạt động
Ngày 24/11/2021
	- Mic mic đã hoạt động, rút mic đưa về trạng thái stop
	- 4 chế độ nháy led xanh on/off ms: play(100/100), stop(100/900), reconnect(900/100), ban(3000/3000)
	- Nút nhấn ấn từng lần 1 không nhấn giữ (done)
Ngày 06/12/2021 
Update
	- Gửi MAC 5s 1 lần trong vòng 1p
Ngày 24/05/2021
Update
	- Sửa lỗi không mute khi gửi DLK=0.0.0.0:0/
	- Sửa lỗi không mute khi STS=2
Ngày 25/05/2021
Update
	- Sửa lôi không lấy đc DHCP từ module 4G
Ngày 02/11/2022
Update
	- Mã hóa Base64 với MAC đưa vào account mqtt


	