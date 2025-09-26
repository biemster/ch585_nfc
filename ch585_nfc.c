#include "ch32fun.h"
#include <stdio.h>

#define LED PA8

#define R8_NFC_CMD                    (*(vu8*)0x4000E000)
#define R8_NFC_STATUS                 (*(vu8*)0x4000E001)
#define R16_NFC_INTF_STATUS           (*(vu16*)0x4000E004)
#define R16_NFC_RXTX_LEN              (*(vu16*)0x4000E008)
#define R16_NFC_FIFO                  (*(vu16*)0x4000E00C)
#define R16_NFC_TMR                   (*(vu16*)0x4000E010)
#define R32_NFC_DRV                   (*(vu32*)0x4000E014)

#define BSS_PCD_END_CB                (*(vu32*)0x200000EC)
#define BSS_PCD_DATA_BUF              (*(vu32*)0x200000F0)
#define BSS_PCD_SEND_BUF              (*(vu32*)0x200000F4)
#define BSS_PCD_RECV_BUF              (*(vu32*)0x200000F8)
#define BSS_PCD_PARITY_BUF            (*(vu32*)0x200000FC)
#define BSS_PCD_DATA_BUF_SIZE         (*(vu16*)0x20000100)
#define BSS_PCD_SEND_BUF_SIZE         (*(vu16*)0x20000102)
#define BSS_PCD_RECV_BUF_SIZE         (*(vu16*)0x20000104)
#define BSS_PCD_PARITY_BUF_SIZE       (*(vu16*)0x20000106)
#define BSS_PCD_TX_FIFO_BYTES         (*(vu16*)0x20000108)
#define BSS_PCD_TX_TOTAL_BYTES        (*(vu16*)0x2000010A)
#define BSS_PCD_WORD_IDX              (*(vu16*)0x2000010C)
#define BSS_R16_NFC_RECV_BITS         (*(vu16*)0x2000010E)
#define BSS_R32_NFC_RECV_LEN          (*(vu32*)0x20000114)
#define BSS_R8_NFC_BUF_OFFSET         (*(vu8*)0x2000011A)
#define BSS_R8_NFC_INTF_MODE          (*(vu8*)0x2000011B)
#define BSS_R8_NFC_COMM_STATUS        (*(vu8*)0x2000011C)

#define NFCA_PCD_DATA_BUF_SIZE                   32
#define NFCA_PCD_MAX_SEND_NUM                    (NFCA_PCD_DATA_BUF_SIZE)
#define NFCA_PCD_MAX_RECV_NUM                    (NFCA_PCD_DATA_BUF_SIZE * 16 / 9)
#define NFCA_PCD_MAX_PARITY_NUM                  (NFCA_PCD_MAX_RECV_NUM)
#define NFCA_PCD_LPCD_THRESHOLD_PERMIL           5
#define NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL 20
#define NFCA_PCD_WAIT_MAX_MS                     1000
#define NFCA_PCD_TICKS_PER_MILLISECOND           1725

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

#define ISO14443A_CHECK_BCC(B)    ((B[0] ^ B[1] ^ B[2] ^ B[3]) == B[4])
#define NFCA_PCD_SET_OUT_DRV(lvl) (R32_NFC_DRV = ((R32_NFC_DRV & 0x9ff) | lvl))

__attribute__((aligned(4))) static uint16_t gs_nfca_pcd_data_buf[NFCA_PCD_DATA_BUF_SIZE];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_parity_buf[NFCA_PCD_MAX_PARITY_NUM];
__attribute__((aligned(4))) static uint16_t gs_lpcd_adc_filter_buf[8];

extern uint8_t nfca_available;
static uint16_t gs_lpcd_adc_base_value;
uint16_t g_nfca_pcd_recv_buf_len;
uint32_t g_nfca_pcd_recv_bits;

typedef enum {
	NFCA_PCD_CONTROLLER_STATE_FREE = 0,
	NFCA_PCD_CONTROLLER_STATE_SENDING,
	NFCA_PCD_CONTROLLER_STATE_RECEIVING,
	NFCA_PCD_CONTROLLER_STATE_COLLISION,
	NFCA_PCD_CONTROLLER_STATE_OVERTIME,
	NFCA_PCD_CONTROLLER_STATE_DONE,
	NFCA_PCD_CONTROLLER_STATE_ERR,
} nfca_pcd_controller_state_t;

typedef enum {
	NFCA_PCD_DRV_CTRL_LEVEL0 = (0x00 << 13),
	NFCA_PCD_DRV_CTRL_LEVEL1 = (0x01 << 13),
	NFCA_PCD_DRV_CTRL_LEVEL2 = (0x02 << 13),
	NFCA_PCD_DRV_CTRL_LEVEL3 = (0x03 << 13),
} NFCA_PCD_DRV_CTRL_Def;

typedef enum {
	NFCA_PCD_REC_MODE_NONE   = 0,
	NFCA_PCD_REC_MODE_NORMAL = 1,
	NFCA_PCD_REC_MODE_COLI   = 0x10,
} NFCA_PCD_REC_MODE_Def;

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

const uint8_t byteParityBitsTable[256] = {
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}

extern void NFC_IRQLibHandler(void);
__INTERRUPT
void NFC_IRQHandler(void) {
	NFC_IRQLibHandler();
	return;

	// --- State 1: TRANSMITTING ---
	if (BSS_R8_NFC_COMM_STATUS == 1) {
		// Check if the TX FIFO is ready for more data (TX FIFO Empty flag)
		if ((R16_NFC_INTF_STATUS & 8) && (BSS_PCD_TX_FIFO_BYTES < BSS_PCD_TX_TOTAL_BYTES)) {
			// Refill the FIFO with up to 5 words
			for (int i = 0; i < 5; i++) {
				R16_NFC_FIFO = gs_nfca_pcd_data_buf[BSS_PCD_TX_FIFO_BYTES];
				BSS_PCD_TX_FIFO_BYTES++;
				if (BSS_PCD_TX_FIFO_BYTES >= BSS_PCD_TX_TOTAL_BYTES) {
					break;
				}
			}
		}

		// Check if the transmission has completed (TX Complete flag)
		if (R16_NFC_INTF_STATUS & 1) {
			R8_NFC_CMD &= 0xfe; // Clear the Start TX bit

			// If mode is Transceive, switch to receive mode
			if (BSS_R8_NFC_INTF_MODE) {
				funDigitalWrite( PA4, FUN_HIGH );
				BSS_R16_NFC_RECV_BITS = 0;
				BSS_PCD_WORD_IDX = 0;

				// Set control register based on mode (e.g., enable parity)
				R8_NFC_STATUS = (BSS_R8_NFC_INTF_MODE & 0x10) ? 0x36 : 0x26;
	
				BSS_R8_NFC_COMM_STATUS = 2; // Change state to Receiving
				R8_NFC_CMD |= 0x18; // Enable RX
				funDigitalWrite( PA4, FUN_LOW );
			} 
			// If mode is Transmit-only, the operation is complete
			else {
				R8_NFC_STATUS = 0;
				BSS_R8_NFC_COMM_STATUS = 5; // Status: Success
			}
		}
	}
	// --- State 2: RECEIVING ---
	else if (BSS_R8_NFC_COMM_STATUS == 2) {
		// Check if there is data in the RX FIFO (FIFO Not Empty flag)
		if (R16_NFC_INTF_STATUS & 4) { // Note: OV flag is used for Not Empty
			// Drain up to 5 words from the FIFO
			for (int i = 0; i < 5; i++) {
				gs_nfca_pcd_data_buf[BSS_PCD_WORD_IDX++] = R16_NFC_FIFO;
			}
		}

		// Check if the reception has completed (RX Complete flag)
		if (R16_NFC_INTF_STATUS & 1) {
			BSS_R16_NFC_RECV_BITS = R16_NFC_RXTX_LEN;
			uint16_t received_words = (BSS_R16_NFC_RECV_BITS + 15) / 16;

			// Drain any final words left in the FIFO
			if (BSS_PCD_WORD_IDX < received_words) {
				uint16_t words_to_drain = received_words - BSS_PCD_WORD_IDX;
				for (int i = 0; i < words_to_drain; i++) {
					gs_nfca_pcd_data_buf[BSS_PCD_WORD_IDX++] = R16_NFC_FIFO;
				}
			}
			BSS_R8_NFC_COMM_STATUS = 5; // Status: Success
		}
		// Check for error flags
		else if (R16_NFC_INTF_STATUS & 0x10) {
			BSS_R16_NFC_RECV_BITS = R16_NFC_RXTX_LEN;
			// Drain FIFO on error
			uint16_t received_words = (BSS_R16_NFC_RECV_BITS + 15) / 16;
			if (BSS_PCD_WORD_IDX < received_words) {
				uint16_t words_to_drain = received_words - BSS_PCD_WORD_IDX;
				for (int i = 0; i < words_to_drain; i++) {
					gs_nfca_pcd_data_buf[BSS_PCD_WORD_IDX++] = R16_NFC_FIFO;
				}
			}
			BSS_R8_NFC_COMM_STATUS = 3; // Status: Parity Error
		} 
		else if (R16_NFC_INTF_STATUS & 0x20) {
			BSS_R8_NFC_COMM_STATUS = 4; // Status: CRC Error
		}
	}
	// --- Any Other State: ERROR ---
	else {
		BSS_R8_NFC_COMM_STATUS = 6; // Status: General Error
	}
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
	funPinMode( (PB8 | PB9 | PB16 | PB17), GPIO_CFGLR_IN_FLOAT );
	
	R32_PIN_IN_DIS |= (((PB8 | PB9) & ~PB)<< 16);
	R16_PIN_CONFIG |= (((PB16 | PB17) & ~PB) >> 8);

	BSS_PCD_END_CB = 0;
	
	BSS_PCD_DATA_BUF = (uint32_t)gs_nfca_pcd_data_buf;
	BSS_PCD_SEND_BUF = (uint32_t)g_nfca_pcd_send_buf;
	BSS_PCD_RECV_BUF = (uint32_t)g_nfca_pcd_recv_buf;
	BSS_PCD_PARITY_BUF = (uint32_t)g_nfca_pcd_parity_buf;
	
	BSS_PCD_DATA_BUF_SIZE = NFCA_PCD_DATA_BUF_SIZE;
	BSS_PCD_SEND_BUF_SIZE = NFCA_PCD_MAX_SEND_NUM;
	BSS_PCD_RECV_BUF_SIZE = NFCA_PCD_MAX_RECV_NUM; 
	BSS_PCD_PARITY_BUF_SIZE = NFCA_PCD_MAX_PARITY_NUM;

	nfca_available = 1;
}

void nfca_pcd_start(void) {
	if(nfca_available) {
		R8_NFC_CMD = 0x24;
		R32_NFC_DRV &= 0xe7ff;
	}
	NVIC_ClearPendingIRQ(NFC_IRQn);
	NVIC_EnableIRQ(NFC_IRQn);
}

void nfca_pcd_stop(void) {
	if(nfca_available) {
		R8_NFC_STATUS = 0;
		R8_NFC_CMD = 0;
	}
	NVIC_DisableIRQ(NFC_IRQn);
}

void nfca_pcd_wait_ms(uint32_t milliseconds) {
	uint32_t ticks = milliseconds * NFCA_PCD_TICKS_PER_MILLISECOND;
	
	if (ticks > 0xFFFF) {
		ticks = 0xFFFF;
	}
	
	R16_NFC_TMR = (uint16_t)ticks;
}

void nfca_pcd_wait_us(uint32_t microseconds) {
	uint64_t ticks = ((uint64_t)microseconds * NFCA_PCD_TICKS_PER_MILLISECOND) / 1000;
	
	if (ticks > 0xFFFF) {
		ticks = 0xFFFF;
	}
	
	R16_NFC_TMR = (uint16_t)ticks;
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

uint32_t nfca_pcd_separate_recv_data(uint16_t data_buf[], uint16_t num_bits, uint8_t bss_11a, uint8_t recv_buf[], uint8_t parity_buf[], uint16_t output_len) {
	// Current position in the bitstream, combining word index and bit offset.
	uint32_t current_total_bit_pos = bss_11a; //bit_offset;
	uint32_t bits_remaining = num_bits;
	int32_t bytes_processed;

	// The main loop processes one output byte per iteration.
	// This corresponds to the loop between LAB_00001068 and LAB_000010b6.
	for (bytes_processed = 0; bytes_processed < output_len; ++bytes_processed) {
		// Stop if there aren't enough bits left for a full 9-bit packet.
		// The assembly checks this with 'num_bits' (bss_10e).
		if (bits_remaining < 9) {
			break;
		}

		// Determine which word(s) in data_buf hold the next 9 bits.
		uint32_t word_index = current_total_bit_pos / 16;
		uint32_t bit_pos_in_word = current_total_bit_pos % 16;

		// Combine two consecutive 16-bit words into a 32-bit integer to safely
		// extract 9 bits that may cross a word boundary. This is a clean C
		// equivalent of the assembly's more complex boundary-checking logic.
		uint32_t combined_words = data_buf[word_index];
		if (bit_pos_in_word > (16 - 9)) {
			// The 9-bit chunk crosses from data_buf[word_index] to data_buf[word_index + 1].
			combined_words |= (uint32_t)data_buf[word_index + 1] << 16;
		}

		// Shift to the correct starting bit and mask to extract the 9-bit packet.
		// The mask 0x1FF is binary 1 1111 1111.
		uint16_t nine_bit_packet = (combined_words >> bit_pos_in_word) & 0x1FF;

		// The lower 8 bits are the data byte.
		// Corresponds to 'sra' and 'andi' at 0x1072-0x1076.
		recv_buf[bytes_processed] = (uint8_t)(nine_bit_packet & 0xFF);

		// The 9th bit is the parity bit.
		// Corresponds to the 'sbext' instruction at 0x1090.
		parity_buf[bytes_processed] = (uint8_t)((nine_bit_packet >> 8) & 0x01);

		// Advance the bit position by 9 for the next iteration.
		current_total_bit_pos += 9;
		bits_remaining -= 9;
	}

	// The function returns the number of bytes processed, which is in register a0
	// at the end of the assembly code.
	return bytes_processed;
}

nfca_pcd_controller_state_t nfca_pcd_get_comm_status() {
	if(BSS_R8_NFC_COMM_STATUS > 2) {
		if(BSS_R8_NFC_INTF_MODE) {
			R8_NFC_CMD &= 0xef;
			BSS_R32_NFC_RECV_LEN = nfca_pcd_separate_recv_data(gs_nfca_pcd_data_buf, BSS_R16_NFC_RECV_BITS, BSS_R8_NFC_BUF_OFFSET, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, NFCA_PCD_MAX_RECV_NUM);
			printf("r: %ld [%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x]\n", BSS_R32_NFC_RECV_LEN, gs_nfca_pcd_data_buf[0], gs_nfca_pcd_data_buf[1], gs_nfca_pcd_data_buf[2], gs_nfca_pcd_data_buf[3],
					gs_nfca_pcd_data_buf[4], gs_nfca_pcd_data_buf[5], gs_nfca_pcd_data_buf[6], gs_nfca_pcd_data_buf[7],
					gs_nfca_pcd_data_buf[8], gs_nfca_pcd_data_buf[9], gs_nfca_pcd_data_buf[10], gs_nfca_pcd_data_buf[11]);
			printf("      [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", g_nfca_pcd_recv_buf[0], g_nfca_pcd_recv_buf[1], g_nfca_pcd_recv_buf[2], g_nfca_pcd_recv_buf[3],
					g_nfca_pcd_recv_buf[4], g_nfca_pcd_recv_buf[5], g_nfca_pcd_recv_buf[6], g_nfca_pcd_recv_buf[7],
					g_nfca_pcd_recv_buf[8], g_nfca_pcd_recv_buf[9], g_nfca_pcd_recv_buf[10], g_nfca_pcd_recv_buf[11]);
		}
		return BSS_R8_NFC_COMM_STATUS;
	}
	return 0;
}

uint32_t nfca_pcd_prepare_send_data(uint8_t send_buf[], uint16_t num_bits, uint8_t parity_buf[], uint16_t data_buf[], uint32_t max_out_len) {
	if (num_bits <= 3) {
		return 0;
	}

	int processed_bytes = 0;
	int num_chunks = 0;

	// Process the data in 8-byte chunks.
	while ((num_bits - processed_bytes) >= 8) {
		if ((processed_bytes + 8) > max_out_len) {
			// How can I just break here, and still everything goes fine??
			break;
		}

		for (int i = 0; i < 8; ++i) {
			int current_index = processed_bytes + i;

			// Read a byte from each input buffer using the calculated index.
			uint8_t data_byte = send_buf[current_index];
			uint8_t parity_byte = parity_buf[current_index];

			// Combine data and parity into a 16-bit word.
			// If the parity byte is non-zero, the 9th bit is set.
			data_buf[current_index] = (uint16_t)data_byte | (parity_byte > 0 ? 0x0100 : 0);
		}
		processed_bytes += 8;
		num_chunks++;
	}

	int remaining_len = num_bits - processed_bytes;

	// Process any remaining bytes that did not form a full 8-byte chunk.
	if (remaining_len > 0 && processed_bytes < max_out_len) {
		data_buf[num_chunks] = send_buf[num_chunks] | (parity_buf[num_chunks] > 0 ? 0x0100 : 0);
	}

	return ((num_bits / 8) *9) + (num_bits % 8); // this should reflect num_chunks??
}

uint8_t nfca_pcd_comm(uint16_t data_bits_num, NFCA_PCD_REC_MODE_Def mode, uint8_t offset) {
	if (!nfca_available) {
		BSS_R8_NFC_INTF_MODE = 0;
		BSS_R8_NFC_COMM_STATUS = 6; // Status: Error/Unavailable
		return 2;
	}

	// Check for a minimum number of bits.
	if (data_bits_num <= 3) {
		BSS_R8_NFC_INTF_MODE = 0;
		BSS_R8_NFC_COMM_STATUS = 6; // Status: Error/Invalid Param
		return 1;
	}

	// Prepare the raw data buffers into a single packed buffer for the FIFO.
	int prepared_byte_len = nfca_pcd_prepare_send_data(g_nfca_pcd_send_buf, data_bits_num, g_nfca_pcd_parity_buf, gs_nfca_pcd_data_buf, NFCA_PCD_DATA_BUF_SIZE);
	printf("s: %d [%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x]\n", prepared_byte_len, gs_nfca_pcd_data_buf[0], gs_nfca_pcd_data_buf[1], gs_nfca_pcd_data_buf[2], gs_nfca_pcd_data_buf[3],
			gs_nfca_pcd_data_buf[4], gs_nfca_pcd_data_buf[5], gs_nfca_pcd_data_buf[6], gs_nfca_pcd_data_buf[7],
			gs_nfca_pcd_data_buf[8], gs_nfca_pcd_data_buf[9], gs_nfca_pcd_data_buf[10], gs_nfca_pcd_data_buf[11]);
	printf("      [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", g_nfca_pcd_send_buf[0], g_nfca_pcd_send_buf[1], g_nfca_pcd_send_buf[2], g_nfca_pcd_send_buf[3],
			g_nfca_pcd_send_buf[4], g_nfca_pcd_send_buf[5], g_nfca_pcd_send_buf[6], g_nfca_pcd_send_buf[7],
			g_nfca_pcd_send_buf[8], g_nfca_pcd_send_buf[9], g_nfca_pcd_send_buf[10], g_nfca_pcd_send_buf[11]);

	// --- Configure NFC Hardware ---
	R8_NFC_STATUS = 0; // Clear control register

	// Clear command bits 0 and 4 (CMD_IDLE and CMD_MF_AUTH)
	R8_NFC_CMD &= ~(0x11);

	// Store current communication parameters in global state
	BSS_R8_NFC_INTF_MODE = mode;
	BSS_R8_NFC_BUF_OFFSET = offset;

	// Set the total number of bytes to be transmitted
	R16_NFC_RXTX_LEN = (uint16_t)prepared_byte_len;

	// Calculate the number of 9-bit words to transmit.
	// The hardware appears to send data in 9-bit frames (8 data + 1 parity).
	int num_words = (prepared_byte_len + 8) / 9;

	// --- Load Data into FIFO ---
	if (prepared_byte_len > 72) {
		// For large transfers, load the first 8 words (16 bytes) into the FIFO.
		// The hardware might use DMA for the rest of the data.
		for (int i = 0; i < 8; i++) {
			R16_NFC_FIFO = gs_nfca_pcd_data_buf[i];
		}
		BSS_PCD_TX_FIFO_BYTES = 8;
	}
	else {
		// For smaller transfers, load all the words into the FIFO.
		for (int i = 0; i < num_words; i++) {
			R16_NFC_FIFO = gs_nfca_pcd_data_buf[i];
		}
		BSS_PCD_TX_FIFO_BYTES = num_words;
	}

	// --- Start Transmission ---
	BSS_PCD_TX_TOTAL_BYTES = num_words;
	BSS_R8_NFC_COMM_STATUS = 1; // Status: Transmitting

	// Set registers to indicate 9-bit transmission format
	R16_NFC_INTF_STATUS = 9;
	R8_NFC_STATUS = 9;

	R8_NFC_CMD |= (mode ? 9 : 1);

	return 0; // Success
}

nfca_pcd_controller_state_t nfca_pcd_wait_comm_end(void) {
	nfca_pcd_controller_state_t status;
	uint32_t overtimes;
	
	overtimes = 0;
	
	while (1) {
		status = nfca_pcd_get_comm_status();
		if ((status != 0) || (overtimes > (NFCA_PCD_WAIT_MAX_MS * 10))) {
			break;
		}
		
		Delay_Us(100);
		overtimes++;
	}
	
	g_nfca_pcd_recv_buf_len = BSS_R32_NFC_RECV_LEN;
	g_nfca_pcd_recv_bits = BSS_R16_NFC_RECV_BITS;
	
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
	nfca_pcd_wait_us(200);

	if(nfca_pcd_comm(7, NFCA_PCD_REC_MODE_COLI, 0) == 0) {
		status = nfca_pcd_wait_comm_end();
		
		if((status == NFCA_PCD_CONTROLLER_STATE_DONE) || (status == NFCA_PCD_CONTROLLER_STATE_COLLISION)) {
			if(g_nfca_pcd_recv_bits == (2 * 9)) {
				if(ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 2)) {
					return ((uint16_t *)(g_nfca_pcd_recv_buf))[0];
				}
				else {
					printf("ODD BIT ERROR\n");
					printf("data: 0x%02x 0x%02x\n", ((uint16_t *)(g_nfca_pcd_recv_buf))[0], ((uint16_t *)(g_nfca_pcd_recv_buf))[1]);
					printf("parity: %d %d\n", g_nfca_pcd_parity_buf[0], g_nfca_pcd_parity_buf[1]);
				}
			}
			else {
				printf("BITS NUM ERROR: %ld, 0x%04x\n", g_nfca_pcd_recv_bits, ((uint16_t *)(g_nfca_pcd_recv_buf))[0]);
			}
		}
		else {
			printf("STATUS ERROR %d:", status);
			switch(status) {
			case NFCA_PCD_CONTROLLER_STATE_OVERTIME:
				printf(" OVERTIME\n");
				break;
			case NFCA_PCD_CONTROLLER_STATE_ERR:
				printf(" ERR\n");
				break;
			default:
				printf(" UNKNOWN\n");
				break;
			}
		}
	}
	else {
		printf("COMMUNICATE ERROR\n");
	}
	
	return 0;
}

uint16_t PcdAnticoll(uint8_t cmd) {
	nfca_pcd_controller_state_t status;
	uint16_t res = PCD_COMMUNICATE_ERROR;
	
	g_nfca_pcd_send_buf[0] = cmd;
	g_nfca_pcd_send_buf[1] = 0x20;
	
	nfca_pcd_wait_ms(PCD_ANTICOLL_OVER_TIME);
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 2);

	if (nfca_pcd_comm(16, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) {
		status = nfca_pcd_wait_comm_end();
		
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
					printf("ODD BIT ERROR\n");
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
	nfca_pcd_wait_ms(PCD_SELECT_OVER_TIME);
	for (res = 0; res < 4; res++) {
		g_nfca_pcd_send_buf[res + 2] = *(pSnr + res);
		g_nfca_pcd_send_buf[6] ^= *(pSnr + res);
	}
	ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 7);
	
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 9);
	
	if (nfca_pcd_comm(9 * 8, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) {
		status = nfca_pcd_wait_comm_end();
		
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
					printf("ODD BIT ERROR\n");
				}
			}
		}
	}
	
	return res;
}

void PcdHalt(void) {
	nfca_pcd_wait_ms(5);
	
	g_nfca_pcd_send_buf[0] = PICC_HALT;
	g_nfca_pcd_send_buf[1] = 0;
	
	g_nfca_pcd_send_buf[2] = 0x57;
	g_nfca_pcd_send_buf[3] = 0xcd;
	
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);
	
	nfca_pcd_comm((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);
	nfca_pcd_wait_comm_end();
}

// test
void nfca_pcd_test() {
	uint16_t res;
	uint8_t picc_uid[7];

	int vdd_value = ADC_VoltConverSignalPGA_MINUS_12dB( sys_get_vdd() );
	if(vdd_value > 3400) {
		NFCA_PCD_SET_OUT_DRV(NFCA_PCD_DRV_CTRL_LEVEL0);
		printf("NFC drive lvl0\n");
	}
	else if(vdd_value > 3000) {
		NFCA_PCD_SET_OUT_DRV(NFCA_PCD_DRV_CTRL_LEVEL1);
		printf("NFC drive lvl1\n");
	}
	else if(vdd_value > 2600) {
		NFCA_PCD_SET_OUT_DRV(NFCA_PCD_DRV_CTRL_LEVEL2);
		printf("NFC drive lvl2\n");
	}
	else {
		NFCA_PCD_SET_OUT_DRV(NFCA_PCD_DRV_CTRL_LEVEL3);
		printf("NFC drive lvl3\n");
	}

	while(1) {
		nfca_pcd_start();

		if(nfca_pcd_lpcd_check()) {
			printf("* card detected\n");
			
			//either blink(...) or Delay_Ms(5)
			blink(2);
			// Delay_Ms(5);
	
			res = PcdRequest(PICC_REQALL);
			if(res == 0x0004) { /* Mifare Classic */
				res = PcdAnticoll(PICC_ANTICOLL1);
				if (res == PCD_NO_ERROR) {
					picc_uid[0] = g_nfca_pcd_recv_buf[0];
					picc_uid[1] = g_nfca_pcd_recv_buf[1];
					picc_uid[2] = g_nfca_pcd_recv_buf[2];
					picc_uid[3] = g_nfca_pcd_recv_buf[3];
					printf("Mifare Classic uid: %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1], picc_uid[2], picc_uid[3]);
	
					res = PcdSelect(PICC_ANTICOLL1, picc_uid);
					if (res == PCD_NO_ERROR) {
						printf("select OK, SAK:%02x\n", g_nfca_pcd_recv_buf[0]);
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
								printf("Mifare Ultralight uid: %02x %02x %02x %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1],
										picc_uid[2], picc_uid[3], picc_uid[4], picc_uid[5], picc_uid[6]);
				
								res = PcdSelect(PICC_ANTICOLL2, g_nfca_pcd_recv_buf);
								if (res == PCD_NO_ERROR) {
									printf("select OK, SAK: %02x\n", g_nfca_pcd_recv_buf[0]);
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
			else {
				printf("unknown type (ATQA: 0x%04x)\n", res);
			}
		}

		nfca_pcd_stop();
		Delay_Ms(500);
	}
}

int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

	printf("~ ch585 NFC ~\n");
	blink(5);
	
	funPinMode( PA4, GPIO_CFGLR_OUT_2Mhz_PP ); // Used in the IRQHandler
	nfca_pcd_init();
	nfca_pcd_lpcd_calibration();
	nfca_pcd_test();

	while(1);
}
