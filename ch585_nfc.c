#include "ch32fun.h"
#include "NFCA_LIB/CH58x_NFCA_LIB.h"
#include <stdio.h>

#define LED PA8

#define NFCA_PCD_DATA_BUF_SIZE                   32
#define NFCA_PCD_MAX_SEND_NUM                    (NFCA_PCD_DATA_BUF_SIZE)
#define NFCA_PCD_MAX_RECV_NUM                    (NFCA_PCD_DATA_BUF_SIZE * 16 / 9)
#define NFCA_PCD_MAX_PARITY_NUM                  (NFCA_PCD_MAX_RECV_NUM)
#define NFCA_PCD_LPCD_THRESHOLD_PERMIL           5
#define NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL 20
#define NFCA_PCD_WAIT_MAX_MS                     1000

#define PICC_REQALL            0x52
#define PICC_ANTICOLL1         0x93
#define PICC_ANTICOLL2         0x95
#define PICC_HALT              0x50

#define NAK_INVALID_ARG        0x00
#define NAK_CRC_ERROR          0x01
#define NAK_NOT_AUTHED         0x04
#define NAK_EEPROM_ERROR       0x05
#define NAK_OTHER_ERROR        0x06
#define PICC_NAK_HEAD          0x0200

#define PCD_ANTICOLL_OVER_TIME 10
#define PCD_SELECT_OVER_TIME   10

#define ISO14443A_CHECK_BCC(B) ((B[0] ^ B[1] ^ B[2] ^ B[3]) == B[4])

__attribute__((aligned(4))) static uint16_t gs_nfca_pcd_data_buf[NFCA_PCD_DATA_BUF_SIZE];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_parity_buf[NFCA_PCD_MAX_PARITY_NUM];
__attribute__((aligned(4))) static uint16_t gs_lpcd_adc_filter_buf[8];

static uint16_t gs_lpcd_adc_base_value;
uint16_t g_nfca_pcd_recv_buf_len;
uint32_t g_nfca_pcd_recv_bits;

typedef enum {
	SampleFreq_8 = 0,
	SampleFreq_8_or_4,
	SampleFreq_5_33_or_2_67,
	SampleFreq_4_or_2,
} ADC_SampClkTypeDef;

typedef enum {
	ADC_PGA_1_4 = 0,
	ADC_PGA_1_2,
	ADC_PGA_0,
	ADC_PGA_2,
	ADC_PGA_2_ = 0x10,
	ADC_PGA_4,
	ADC_PGA_8,
	ADC_PGA_16,
} ADC_SignalPGATypeDef;

typedef enum {
	CH_EXTIN_0 = 0,
	CH_EXTIN_1,
	CH_EXTIN_2,
	CH_EXTIN_3,
	CH_EXTIN_4,
	CH_EXTIN_5,
	CH_EXTIN_6,
	CH_EXTIN_7,
	CH_EXTIN_8,
	CH_EXTIN_9,
	CH_EXTIN_10,
	CH_EXTIN_11,
	CH_EXTIN_12,
	CH_EXTIN_13,
	
	CH_INTE_VBAT = 14,
	CH_INTE_VTEMP = 15,
	CH_INTE_NFC = 16,
} ADC_SingleChannelTypeDef;

enum {
	PCD_NO_ERROR              = 0,
	PCD_COMMUNICATE_ERROR     = 1,
	PCD_ODD_PARITY_ERROR      = 2,
	PCD_UNKNOWN_ERROR         = 3,

	PCD_OVERTIME_ERROR        = 0x0100,
	PCD_FRAME_ERROR           = 0x0101,
	PCD_BCC_ERROR             = 0x0102,
	PCD_CRC_ERROR             = 0x0103,
	PCD_AUTH_ERROR            = 0x0104,
	PCD_DECRYPT_ERROR         = 0x0105,
	PCD_VALUE_BLOCK_INVALID   = 0x0106,

	PICC_NAK_INVALID_ARG      = (PICC_NAK_HEAD | NAK_INVALID_ARG),
	PICC_NAK_CRC_ERROR        = (PICC_NAK_HEAD | NAK_CRC_ERROR),
	PICC_NAK_NOT_AUTHED       = (PICC_NAK_HEAD | NAK_NOT_AUTHED),
	PICC_NAK_EEPROM_ERROR     = (PICC_NAK_HEAD | NAK_EEPROM_ERROR),
	PICC_NAK_OTHER_ERROR      = (PICC_NAK_HEAD | NAK_OTHER_ERROR),
};

extern void NFC_IRQLibHandler(void);
__INTERRUPT
void NFC_IRQHandler(void) {
	NFC_IRQLibHandler();
}

uint16_t sys_get_vdd(void) {
	uint8_t  sensor, channel, config, tkey_cfg;
	uint16_t adc_data;
	
	tkey_cfg = R8_TKEY_CFG;
	sensor = R8_TEM_SENSOR;
	channel = R8_ADC_CHANNEL;
	config = R8_ADC_CFG;
	
	R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
	R8_ADC_CHANNEL = CH_INTE_VBAT;
	R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (0 << 4);
	R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
	R8_ADC_CONVERT |= (3 << 4);
	R8_ADC_CONVERT |= RB_ADC_START;
	while (R8_ADC_CONVERT & RB_ADC_START);
	adc_data = R16_ADC_DATA;
	
	R8_TEM_SENSOR = sensor;
	R8_ADC_CHANNEL = channel;
	R8_ADC_CFG = config;
	R8_TKEY_CFG = tkey_cfg;
	return (adc_data);
}

int ADC_VoltConverSignalPGA_MINUS_12dB(uint16_t adc_data) {
	return (((int)adc_data*1050+256)/512 - 3*1050);
}

// NFCA functions
void nfca_pcd_init() {
	nfca_pcd_config_t cfg;
	uint8_t res;
	
	funPinMode( (PB8 | PB9 | PB16 | PB17), GPIO_CFGLR_IN_FLOAT );
	
	R32_PIN_IN_DIS |= (((PB8 | PB9) & ~PB)<< 16);
	R16_PIN_CONFIG |= (((PB16 | PB17) & ~PB) >> 8);
	
	cfg.pcd_end_cb = NULL;
	
	cfg.data_buf = gs_nfca_pcd_data_buf;
	cfg.data_buf_size = NFCA_PCD_DATA_BUF_SIZE;
	
	cfg.send_buf = g_nfca_pcd_send_buf;
	cfg.send_buf_size = NFCA_PCD_MAX_SEND_NUM;
	
	cfg.recv_buf = g_nfca_pcd_recv_buf;
	cfg.recv_buf_size = NFCA_PCD_MAX_RECV_NUM;
	
	cfg.parity_buf = g_nfca_pcd_parity_buf;
	cfg.parity_buf_size = NFCA_PCD_MAX_PARITY_NUM;
	
	res = nfca_pcd_lib_init(&cfg);
	if(res) {
		printf("nfca pcd lib init error\n");
		while(1);
	}
}

void nfca_pcd_start(void) {
	nfca_pcd_lib_start();
	NVIC_ClearPendingIRQ(NFC_IRQn);
	NVIC_EnableIRQ(NFC_IRQn);
}

void nfca_pcd_stop(void) {
	nfca_pcd_lib_stop();
	NVIC_DisableIRQ(NFC_IRQn);
}

void nfca_pcd_lpcd_calibration() {
	uint8_t sensor, channel, config, tkey_cfg;
	uint32_t adc_all;
	uint16_t adc_max, adc_min, adc_value;
	
	adc_all = 0;
	adc_max = 0;
	adc_min = 0xffff;
	
	nfca_pcd_start();
	
	Delay_Ms(200);
	
	tkey_cfg = R8_TKEY_CFG;
	sensor = R8_TEM_SENSOR;
	channel = R8_ADC_CHANNEL;
	config = R8_ADC_CFG;
	
	R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
	R8_ADC_CHANNEL = CH_INTE_NFC;
	R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_8_or_4 << 6) | (ADC_PGA_1_4 << 4);
	R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
	R8_ADC_CONVERT &= ~(3 << 4);
	
	Delay_Ms(100);
	
	for(int i = 0; i < 10; i++) {
		R8_ADC_CONVERT |= RB_ADC_START;
		while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
		adc_value = R16_ADC_DATA;
		
		if(adc_value > adc_max) {
			adc_max = adc_value;
		}

		if(adc_value < adc_min) {
			adc_min = adc_value;
		}
		adc_all = adc_all + adc_value;
	}

	R8_TEM_SENSOR = sensor;
	if(channel == CH_INTE_NFC) {
		R8_ADC_CHANNEL = CH_INTE_VBAT;
	}
	else {
		R8_ADC_CHANNEL = channel;
	}
	R8_ADC_CFG = config;
	R8_TKEY_CFG = tkey_cfg;
	
	adc_all = adc_all - adc_max - adc_min;
	
	gs_lpcd_adc_base_value = adc_all >> 3;
	
	printf("gs_lpcd_adc_base_value:%d\n", gs_lpcd_adc_base_value);
	
	for(int i = 0; i < 8; i++) {
		gs_lpcd_adc_filter_buf[i] = gs_lpcd_adc_base_value;
	}
	
	nfca_pcd_stop();
}

uint16_t nfca_adc_get_ant_signal(void) {
	uint8_t  sensor, channel, config, tkey_cfg;
	uint16_t adc_data;
	uint32_t adc_data_all;
	
	tkey_cfg = R8_TKEY_CFG;
	sensor = R8_TEM_SENSOR;
	channel = R8_ADC_CHANNEL;
	config = R8_ADC_CFG;
	
	R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
	R8_ADC_CHANNEL = CH_INTE_NFC;
	R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_8_or_4 << 6) | (ADC_PGA_1_4 << 4);
	R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
	R8_ADC_CONVERT &= ~(3 << 4);
	
	adc_data_all = 0;
	
	for(uint8_t i = 0; i < 2; i++) {
		R8_ADC_CONVERT |= RB_ADC_START;
		while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
		adc_data_all = adc_data_all + R16_ADC_DATA;
	}
	
	adc_data = adc_data_all / 2;
	
	if(channel == CH_INTE_NFC) {
		R8_ADC_CHANNEL = CH_INTE_VBAT;
	}
	else {
		R8_ADC_CHANNEL = channel;
	}

	R8_TEM_SENSOR = sensor;
	R8_ADC_CFG = config;
	R8_TKEY_CFG = tkey_cfg;
	return (adc_data);
}

nfca_pcd_controller_state_t nfca_pcd_wait_communicate_end(void) {
	nfca_pcd_controller_state_t status;
	uint32_t overtimes;
	
	overtimes = 0;
	
	while (1) {
		status = nfca_pcd_get_communicate_status();
		if ((status != 0) || (overtimes > (NFCA_PCD_WAIT_MAX_MS * 10))) {
			break;
		}
		
		Delay_Ms(100);
		overtimes++;
	}
	
	g_nfca_pcd_recv_buf_len = nfca_pcd_get_recv_data_len();
	g_nfca_pcd_recv_bits = nfca_pcd_get_recv_bits();
	
	return status;
}

uint16_t nfca_pcd_lpcd_adc_filter_buf_add(uint16_t lpcd_adc) {
	uint32_t lpcd_adc_all = 0;
	for(uint8_t i = 0; i < 7; i++) {
		gs_lpcd_adc_filter_buf[i] = gs_lpcd_adc_filter_buf[i + 1];
		lpcd_adc_all = lpcd_adc_all + gs_lpcd_adc_filter_buf[i];
	}
	gs_lpcd_adc_filter_buf[7] = lpcd_adc;
	lpcd_adc_all = lpcd_adc_all + gs_lpcd_adc_filter_buf[7];
	lpcd_adc_all = (lpcd_adc_all >> 3);
	return (uint16_t)lpcd_adc_all;
}

uint8_t nfca_pcd_lpcd_check(void) {
	uint32_t adc_value_diff;
	uint16_t adc_value;
	uint8_t res = 0;
	
	adc_value = nfca_adc_get_ant_signal();
	printf("adc_value:%d\n", adc_value);
	if(adc_value > gs_lpcd_adc_base_value) {
		adc_value_diff = adc_value - gs_lpcd_adc_base_value;
	}
	else {
		adc_value_diff = gs_lpcd_adc_base_value - adc_value;
	}
	adc_value_diff = (adc_value_diff * 1000) / gs_lpcd_adc_base_value;
	
	if((adc_value > gs_lpcd_adc_base_value) && (adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL)) {
		adc_value = ((uint32_t)gs_lpcd_adc_base_value * (1000 + NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL) / 1000);
	}
	else {
		if(adc_value_diff >= NFCA_PCD_LPCD_THRESHOLD_PERMIL) {
			res = 1;
		}

		if(adc_value < gs_lpcd_adc_base_value) {
			adc_value = gs_lpcd_adc_base_value - 1;
		}
	}
	
	gs_lpcd_adc_base_value = nfca_pcd_lpcd_adc_filter_buf_add(adc_value);
	return res;
}

// ISO 14443-3A functions
uint16_t ISO14443_CRCA(uint8_t *buf, uint8_t len) {
	uint8_t *data = buf;
	uint16_t crc = 0x6363;
	uint8_t ch;
	while (len--) {
		ch = *data++ ^ crc;
		ch = ch ^ (ch << 4);
		crc = (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4);
	}
	return crc;
}

uint16_t ISO14443AAppendCRCA(void *buf, uint16_t len) {
	uint16_t crc = 0x6363;
	uint8_t *data = (uint8_t *) buf;
	uint8_t byte8 = 0;
	
	while (len--) {
		byte8 = *data++;
		byte8 ^= crc & 0xFF;
		byte8 ^= byte8 << 4;

		crc = ((((uint16_t)byte8 << 8) | (((crc) >> 8) & 0xFF))
				^ (uint8_t)(byte8 >> 4)
				^ ((uint16_t)byte8 << 3));
	}
	
	data[0] = (crc >> 0) & 0x00FF;
	data[1] = (crc >> 8) & 0x00FF;
	
	return crc;
}

uint8_t ISO14443ACheckOddParityBit(uint8_t *data, uint8_t *parity, uint16_t len) {
	for (int i = 0; i < len; i++) {
		if ((byteParityBitsTable[data[i]]) != parity[i]) {
			return 0;
		}
	}
	return 1;
}

void ISO14443ACalOddParityBit(uint8_t *data, uint8_t *out_parity, uint16_t len) {
	for (int i = 0; i < len; i++) {
		out_parity[i] = byteParityBitsTable[data[i]];
	}
}



// PCD functions
uint16_t PcdRequest(uint8_t req_code) {
	nfca_pcd_controller_state_t status;
	
	g_nfca_pcd_send_buf[0] = req_code;
	nfca_pcd_set_wait_us(200);

	if(nfca_pcd_communicate(7, NFCA_PCD_REC_MODE_COLI, 0) == 0) {
	status = nfca_pcd_wait_communicate_end();
	
	if((status == NFCA_PCD_CONTROLLER_STATE_DONE) || (status == NFCA_PCD_CONTROLLER_STATE_COLLISION)) {
		if(g_nfca_pcd_recv_bits == (2 * 9)) {
			if(ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 2)) {
				printf("ATQA:0x%04x\r\n", ((uint16_t *)(g_nfca_pcd_recv_buf))[0]);
				return ((uint16_t *)(g_nfca_pcd_recv_buf))[0];
			}
			else {
				printf("ODD BIT ERROR\r\n");
				printf("data:0x%02x 0x%02x\r\n", ((uint16_t *)(g_nfca_pcd_recv_buf))[0], ((uint16_t *)(g_nfca_pcd_recv_buf))[1]);
				printf("parity:%d %d\r\n", g_nfca_pcd_parity_buf[0], g_nfca_pcd_parity_buf[1]);
			}
		}
		else {
			printf("BITS NUM ERROR: %ld, 0x%04x\r\n", g_nfca_pcd_recv_bits, ((uint16_t *)(g_nfca_pcd_recv_buf))[0]);
		}
	}
	else {
		printf("STATUS ERROR: %d\r\n", status);
	}
	
	}
	else {
		printf("COMMUNICATE ERROR\r\n");
	}
	
	return 0;
}

uint16_t PcdAnticoll(uint8_t cmd) {
	nfca_pcd_controller_state_t status;
	uint16_t res = PCD_COMMUNICATE_ERROR;
	
	g_nfca_pcd_send_buf[0] = cmd;
	g_nfca_pcd_send_buf[1] = 0x20;
	
	nfca_pcd_set_wait_ms(PCD_ANTICOLL_OVER_TIME);
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 2);

	if (nfca_pcd_communicate(16, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) {
		status = nfca_pcd_wait_communicate_end();
		
		if(status == NFCA_PCD_CONTROLLER_STATE_DONE) {
			if (g_nfca_pcd_recv_bits == (5 * 9)) {
				if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 5)) {
					if (ISO14443A_CHECK_BCC(g_nfca_pcd_recv_buf)) {
						res = PCD_NO_ERROR;
					}
					else {
						res = PCD_BCC_ERROR;
						printf("check bcc error\n");
					}
				}
				else {
					res = PCD_ODD_PARITY_ERROR;
					printf("ODD BIT ERROR\r\n");
				}
			}
		}
	}
	
	return res;
}

uint16_t PcdSelect(uint8_t cmd, uint8_t *pSnr)
{
	nfca_pcd_controller_state_t status;
	uint16_t res = PCD_COMMUNICATE_ERROR;
	
	g_nfca_pcd_send_buf[0] = cmd;
	g_nfca_pcd_send_buf[1] = 0x70;
	g_nfca_pcd_send_buf[6] = 0;
	nfca_pcd_set_wait_ms(PCD_SELECT_OVER_TIME);
	for (res = 0; res < 4; res++) {
		g_nfca_pcd_send_buf[res + 2] = *(pSnr + res);
		g_nfca_pcd_send_buf[6] ^= *(pSnr + res);
	}
	ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 7);
	
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 9);
	
	if (nfca_pcd_communicate(9 * 8, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) {
		status = nfca_pcd_wait_communicate_end();
		
		if(status == NFCA_PCD_CONTROLLER_STATE_DONE) {
			if (g_nfca_pcd_recv_bits == (3 * 9)) {
				if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 3)) {
					if (ISO14443_CRCA((uint8_t *)g_nfca_pcd_recv_buf, 3) == 0) {
						// g_m1_crypto1_cipher.is_encrypted = 0;
						res = PCD_NO_ERROR;
					}
					else {
						res = PCD_CRC_ERROR;
						printf("check crc error\n");
					}
				}
				else {
					res = PCD_ODD_PARITY_ERROR;
					printf("ODD BIT ERROR\r\n");
				}
			}
		}
	}
	
	return res;
}

void PcdHalt(void) {
	nfca_pcd_set_wait_ms(5);
	
	g_nfca_pcd_send_buf[0] = PICC_HALT;
	g_nfca_pcd_send_buf[1] = 0;
	
	g_nfca_pcd_send_buf[2] = 0x57;
	g_nfca_pcd_send_buf[3] = 0xcd;
	
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);
	
	nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);
	nfca_pcd_wait_communicate_end();
}

// test
void nfca_pcd_test() {
	uint16_t res;
	uint8_t picc_uid[7];

	int vdd_value = ADC_VoltConverSignalPGA_MINUS_12dB( sys_get_vdd() );
	printf("vdd_value: %d\n", vdd_value);
	if(vdd_value > 3400) {
		nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL0);
		printf("NFC DRV LVL0\n");
	}
	else if(vdd_value > 3000) {
		nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL1);
		printf("NFC DRV LVL1\n");
	}
	else if(vdd_value > 2600) {
		nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL2);
		printf("NFC DRV LVL2\n");
	}
	else {
		nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL3);
		printf("NFC DRV LVL3\n");
	}

	while(1) {
		nfca_pcd_start();

		if(nfca_pcd_lpcd_check()) {
			printf("CARD DETECT\n");
			
			Delay_Ms(5);
	
			res = PcdRequest(PICC_REQALL);
			if(res == 0x0004) { /* Mifare Classic */
				res = PcdAnticoll(PICC_ANTICOLL1);
				if (res == PCD_NO_ERROR) {
					picc_uid[0] = g_nfca_pcd_recv_buf[0];
					picc_uid[1] = g_nfca_pcd_recv_buf[1];
					picc_uid[2] = g_nfca_pcd_recv_buf[2];
					picc_uid[3] = g_nfca_pcd_recv_buf[3];
					printf("uid: %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1], picc_uid[2], picc_uid[3]);
	
					res = PcdSelect(PICC_ANTICOLL1, picc_uid);
					if (res == PCD_NO_ERROR) {
						printf("\nselect OK, SAK:%02x\n", g_nfca_pcd_recv_buf[0]);
						PcdHalt();
					}
				}
			}
			else if(res == 0x0044) { /* Mifare Ultralight, NFC Forum Type2 */
				res = PcdAnticoll(PICC_ANTICOLL1);
				if (res == PCD_NO_ERROR) {
					if(g_nfca_pcd_recv_buf[0] == 0x88) {
						picc_uid[0] = g_nfca_pcd_recv_buf[1];
						picc_uid[1] = g_nfca_pcd_recv_buf[2];
						picc_uid[2] = g_nfca_pcd_recv_buf[3];
	
						res = PcdSelect(PICC_ANTICOLL1, g_nfca_pcd_recv_buf);
						if (res == PCD_NO_ERROR) {
							res = PcdAnticoll(PICC_ANTICOLL2);
							if (res == PCD_NO_ERROR) {
								picc_uid[3] = g_nfca_pcd_recv_buf[0];
								picc_uid[4] = g_nfca_pcd_recv_buf[1];
								picc_uid[5] = g_nfca_pcd_recv_buf[2];
								picc_uid[6] = g_nfca_pcd_recv_buf[3];
								printf("uid: %02x %02x %02x %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1],
								picc_uid[2], picc_uid[3], picc_uid[4], picc_uid[5], picc_uid[6]);
				
								res = PcdSelect(PICC_ANTICOLL2, g_nfca_pcd_recv_buf);
								if (res == PCD_NO_ERROR) {
									printf("SELECT OK, SAK: %02x\n", g_nfca_pcd_recv_buf[0]);
									PcdHalt();
								}
							}
							else {
								printf("ERROR ANTICOLL2\n");
							}
						}
						else {
							printf("CARD PcdSelect error: %d\n", res);
						}
					}
					else {
						printf("ERROR UID0\n");
					}
				}
			}
		}
		else {
			printf("NO CARD\n");
		}

		nfca_pcd_stop();
		Delay_Ms(500);
	}
}

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}

int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

	printf("~ ch585 NFC ~\n");
	blink(5);
	
	nfca_pcd_init();
	nfca_pcd_lpcd_calibration();
	nfca_pcd_test();

	while(1);
}
