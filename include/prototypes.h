/*
 * function prototypes for w8an-wspr
 */
void savePortalData(void);
void launchSettingsPortal(void);
void callbackSaveParams(void);
void encode(unsigned long);
void calibrate(unsigned long, int);
void set_tx_buffer(void);
void sendNTPpacket(IPAddress &address);
time_t getNtpTime(void);
void getTime(void);
void printTime(void);
void setFreqbuf(int);
void initFilterPins(void);
void lowpass(uint);            
void blinkLed(int);              
