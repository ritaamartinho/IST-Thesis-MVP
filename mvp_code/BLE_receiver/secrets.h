/***** Wi-Fi Credentials *****/
#define STA_SSID "name"
#define STA_PASS "password"


/***** The ThingsBoard parameters *****/

// if cloud/professional edition 
#define THINGSBOARD_SERVER  "thingsboard.cloud"

// if community edition live demo
// #define THINGSBOARD_SERVER  "demo.thingsboard.io"

//BLE Reveiver Device Token
#define TOKEN "xxxxxxxxxxxxxxxxxxxx"


/***** BLE System UUID *****/
// same for all beacons and all BLE Receivers
uint8_t uuid_system[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };