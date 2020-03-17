#ifndef __NORDIC_52832_NFC_
#define	__NORDIC_52832_NFC_

enum {
  READSTATE_IDLE,
  READSTATE_READ_INFO,
  READSTATE_READ_DATA,
};

enum {
  NFCSTATE_IDLE,
  NFCSTATE_READ_INFO,
  NFCSTATE_READ_DATA,
};

extern bool data_recived_flag;
extern uint8_t data_recived_buf[1024];
extern uint32_t data_recived_len;

bool i2c_master_write(uint8_t *buf,uint32_t len);
bool i2c_master_read(void);

int nfc_init(void);
int twi_master_init(void);
void nfc_poll(void);
#endif
