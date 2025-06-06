void send_over_lora_ttn(double avg);

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xF8, 0x9B};
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0xC9, 0x30, 0x9E, 0x8F, 0xBE, 0x25, 0xEA, 0xCD, 0x28, 0x1A, 0x53, 0x4F, 0x1F, 0x98, 0xE6, 0xC4};

/* ABP para*/
uint8_t nwkSKey[] = { 0x8B, 0x83, 0xD1, 0x87, 0xD8, 0x47, 0x63, 0x8D, 0xC9, 0xEB, 0x08, 0x35, 0xA1, 0xE5, 0xC6, 0x51 };
uint8_t appSKey[] = { 0x6D, 0xEE, 0xFE, 0x6A, 0xC2, 0x24, 0xCA, 0x63, 0x6E, 0x6E, 0xD6, 0xB8, 0x40, 0x3F, 0xAB, 0xCC};
uint32_t devAddr =  ( uint32_t )0x260B84A5;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;