#include "ch32fun.h"
#include <stdio.h>

#define LED PA8

#define R8_NFC_CMD                    (*(vu8*)0x4000E000)
#define  RB_NFC_ANTENNA_ON            0x4
#define R8_NFC_STATUS                 (*(vu8*)0x4000E001)
#define R16_NFC_INTF_STATUS           (*(vu16*)0x4000E004)
#define R16_NFC_RXTX_LEN              (*(vu16*)0x4000E008)
#define R16_NFC_FIFO                  (*(vu16*)0x4000E00C)
#define R16_NFC_TMR                   (*(vu16*)0x4000E010)
#define R32_NFC_DRV                   (*(vu32*)0x4000E014)

#define PLUSMIN_7(t,r)                     ((t >> 3) == (r >> 3))
#define PLUSMIN_15(t,r)                    ((t >> 4) == (r >> 4))

#define RB_TMR_FREQ_13_56             0x20
#define TMR0_NFCA_PICC_ETU            128 // For NFC at 106 kbps, 1 ETU  is ~128 / 13.56 MHz = ~9.44 us.
#define TMR0_NFCA_PICC_IS_1ETU(p)     PLUSMIN_7(p, TMR0_NFCA_PICC_ETU)
#define TMR0_NFCA_PICC_IS_1_5ETU(p)   PLUSMIN_7(p, ((TMR0_NFCA_PICC_ETU /2) *3))
#define TMR0_NFCA_PICC_IS_2ETU(p)     PLUSMIN_7(p, (TMR0_NFCA_PICC_ETU *2))
#define TMR3_NFCA_PICC_CNT_END        18 // picc -> pcd freq is 13.56 / 16, but 18 - 20 correspond better to the ch585 nfc freq (18 is used in the blob)
#define TMR3_NFCA_PICC_FTD            (1236 / (TMR3_NFCA_PICC_CNT_END *4)) // Frame Delay Time (~91.15us, divided by PWM period (=CNT_END*4)), wait time for sending response
#define TMR3_CTRL_PWM_ON              ((TMR3_NFCA_PICC_CNT_END /2) -1) // blob uses 8 with CNT_END 18
#define TMR3_CTRL_PWM_OFF             0

#define NFCA_DATA_BUF_SIZE                       512
#define NFCA_PICC_SIGNAL_BUF_SIZE                64
#define NFCA_PICC_TX_BUF_SIZE                    512
#define NFCA_PICC_RESP_SIZE                      128
#define NFCA_PCD_MAX_SEND_NUM                    (NFCA_DATA_BUF_SIZE)
#define NFCA_PCD_MAX_RECV_NUM                    (NFCA_DATA_BUF_SIZE * 16 / 9)
#define NFCA_MAX_PARITY_NUM                      (NFCA_PCD_MAX_RECV_NUM)
#define NFCA_PCD_LPCD_THRESHOLD_PERMIL           5
#define NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL 20
#define NFCA_PCD_WAIT_MAX_MS                     1000
#define NFCA_PCD_TICKS_PER_MILLISECOND           1725

#define PICC_REQA              0x26
#define PICC_WUPA              0x52
#define PICC_ANTICOLL1         0x93
#define PICC_ANTICOLL2         0x95
#define PICC_ANTICOLL3         0x97
#define PICC_HALT              0x50
#define PICC_READ              0x30

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

__attribute__((aligned(4))) static uint16_t gs_nfca_data_buf[NFCA_DATA_BUF_SIZE];
__attribute__((aligned(4))) static uint8_t g_nfca_parity_buf[NFCA_MAX_PARITY_NUM];
__attribute__((aligned(4))) static uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) static uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) static uint32_t gs_nfca_picc_signal_buf[NFCA_PICC_SIGNAL_BUF_SIZE];
__attribute__((aligned(4))) static uint32_t gs_nfca_picc_tx_buf[NFCA_PICC_TX_BUF_SIZE];
__attribute__((aligned(4))) static uint16_t gs_lpcd_adc_filter_buf[8];
__attribute__((aligned(4))) static uint8_t ATQA[2];
__attribute__((aligned(4))) static uint8_t picc_uid[10];
__attribute__((aligned(4))) static uint8_t picc_ultralight_pages[6 *4]; // 6 pages of 4 bytes
__attribute__((aligned(4))) static uint8_t gs_picc_req_resp[NFCA_PICC_RESP_SIZE];

static uint16_t gs_lpcd_adc_base_value;
static uint16_t g_nfca_pcd_recv_buf_len;
static uint32_t g_nfca_pcd_recv_bits;
static uint16_t g_nfca_pcd_recv_word_idx;
static uint16_t g_nfca_pcd_send_fifo_bytes;
static uint16_t g_nfca_pcd_send_total_bytes;
static uint8_t g_nfca_pcd_buf_offset;
static uint8_t g_nfca_pcd_intf_mode;
static uint8_t g_nfca_pcd_comm_status;

static int gs_picc_last_processed_dma_idx; // to .sbss, which is initialized to 0
static int gs_picc_data_idx; // to .sbss, which is initialized to 0
static int gs_picc_anticoll_cmd_guess; // to .sbss, which is initialized to 0
static int gs_picc_uid_len; // to .sbss, which is initialized to 0

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
	PICC_STATE_FREE = 0,
	PICC_STATE_WAIT_FOR_SEND_RESP,
	PICC_STATE_SEND_RESP,
} nfca_picc_controller_state_t;
static nfca_picc_controller_state_t gs_picc_state; // to .sbss, which is initialized to 0

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

typedef enum {
	High_Level = 0,
	Low_Level,
} PWMX_PolarTypeDef;

typedef enum {
	PWM_Times_1 = 0,
	PWM_Times_4,
	PWM_Times_8,
	PWM_Times_16,
} PWM_RepeatTsTypeDef;

typedef enum {
	CAP_NULL = 0,
	Edge_To_Edge,
	FallEdge_To_FallEdge,
	RiseEdge_To_RiseEdge,
} CapModeTypeDef;

static const uint8_t byteParityBitsTable[256] = {
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

__HIGH_CODE
void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}
static volatile int gs_debug_blink; // to .sbss, which is initialized to 0

__INTERRUPT
__HIGH_CODE
void NFC_IRQHandler(void) {
	// Read the interrupt status and write it back to clear the flags.
	uint16_t intf_status = R16_NFC_INTF_STATUS;
	R16_NFC_INTF_STATUS = intf_status;

	// --- State 1: TRANSMITTING ---
	if (g_nfca_pcd_comm_status == 1) {
		// Check if the TX FIFO is ready for more data (TX FIFO Empty flag)
		if ((intf_status & 8) && (g_nfca_pcd_send_fifo_bytes < g_nfca_pcd_send_total_bytes)) {
			// Refill the FIFO with up to 5 words
			for (int i = 0; i < 5; i++) {
				R16_NFC_FIFO = gs_nfca_data_buf[g_nfca_pcd_send_fifo_bytes++];
				if (g_nfca_pcd_send_fifo_bytes >= g_nfca_pcd_send_total_bytes) {
					break;
				}
			}
		}

		// Check if the transmission has completed (TX Complete flag)
		if (intf_status & 1) {

			// If mode is Transceive, switch to receive mode
			R8_NFC_CMD &= ~(0x01); // Clear the Start TX bit
			if (g_nfca_pcd_intf_mode) {
				g_nfca_pcd_recv_bits = 0;
				g_nfca_pcd_recv_word_idx = 0;

				// Set control register based on mode (e.g., enable parity)
				R8_NFC_STATUS = (g_nfca_pcd_intf_mode & 0x10) ? 0x36 : 0x26;

				g_nfca_pcd_comm_status = 2; // Change state to Receiving
				R8_NFC_CMD |= 0x18; // Enable RX
			} 
			// If mode is Transmit-only, the operation is complete
			else {
				R8_NFC_STATUS = 0;
				g_nfca_pcd_comm_status = 5; // Status: Success
			}
		}
	}
	// --- State 2: RECEIVING ---
	else if (g_nfca_pcd_comm_status == 2) {
		// Check if there is data in the RX FIFO (FIFO Not Empty flag)
		if (intf_status & 4) { // Note: OV flag is used for Not Empty
			// Drain up to 5 words from the FIFO
			for (int i = 0; i < 5; i++) {
				gs_nfca_data_buf[g_nfca_pcd_recv_word_idx++] = R16_NFC_FIFO;
			}
		}

		// Check if the reception has completed (RX Complete flag)
		if (intf_status & 2) {
			g_nfca_pcd_recv_bits = R16_NFC_RXTX_LEN;
			uint16_t received_words = (g_nfca_pcd_recv_bits + 15) / 16;

			// Drain any final words left in the FIFO
			if (g_nfca_pcd_recv_word_idx < received_words) {
				uint16_t words_to_drain = received_words - g_nfca_pcd_recv_word_idx;
				for (int i = 0; i < words_to_drain; i++) {
					gs_nfca_data_buf[g_nfca_pcd_recv_word_idx++] = R16_NFC_FIFO;
				}
			}
			g_nfca_pcd_comm_status = 5; // Status: Success
		}
		// Check for error flags
		else if (intf_status & 0x10) {
			g_nfca_pcd_recv_bits = R16_NFC_RXTX_LEN;
			// Drain FIFO on error
			uint16_t received_words = (g_nfca_pcd_recv_bits + 15) / 16;
			if (g_nfca_pcd_recv_word_idx < received_words) {
				uint16_t words_to_drain = received_words - g_nfca_pcd_recv_word_idx;
				for (int i = 0; i < words_to_drain; i++) {
					gs_nfca_data_buf[g_nfca_pcd_recv_word_idx++] = R16_NFC_FIFO;
				}
			}
			g_nfca_pcd_comm_status = 3; // Status: Parity Error
		}
		else if (intf_status & 0x20) {
			g_nfca_pcd_comm_status = 4; // Status: CRC Error
		}
	}
	// --- Any Other State: ERROR ---
	else {
		g_nfca_pcd_comm_status = 6; // Status: General Error
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

void nfca_init() {
	funPinMode( PA7, GPIO_CFGLR_IN_FLOAT ); // NFC CTR
	R32_PIN_IN_DIS |= PA7; // disable PA7 digital input

	funPinMode( (PB8 | PB9 | PB16 | PB17), GPIO_CFGLR_IN_FLOAT );
	R32_PIN_IN_DIS |= (((PB8 | PB9) & ~PB) << 16); // disable PB8 PB9 digital input
	R16_PIN_CONFIG |= (((PB16 | PB17) & ~PB) >> 8); // gpio alt func
}

// NFCA PCD functions
void nfca_pcd_start(void) {
	R8_NFC_CMD = 0x24;
	R32_NFC_DRV &= 0xe7ff;
	NVIC_ClearPendingIRQ(NFC_IRQn);
	NVIC_EnableIRQ(NFC_IRQn);
}

void nfca_pcd_stop(void) {
	R8_NFC_STATUS = 0;
	R8_NFC_CMD = 0;
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

uint32_t nfca_pcd_separate_recv_data(uint16_t data_buf[], uint16_t num_bits, uint8_t buf_offset, uint8_t recv_buf[], uint8_t parity_buf[], uint16_t output_len) {
	// Current position in the bitstream, combining word index and bit offset.
	uint32_t current_total_bit_pos = buf_offset;
	uint32_t bits_remaining = num_bits;
	int32_t bytes_processed;

	// The main loop processes one output byte per iteration.
	for (bytes_processed = 0; bytes_processed < output_len; ++bytes_processed) {
		// Stop if there aren't enough bits left for a full 9-bit packet.
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
		recv_buf[bytes_processed] = (uint8_t)(nine_bit_packet & 0xFF);

		// The 9th bit is the parity bit.
		parity_buf[bytes_processed] = (uint8_t)((nine_bit_packet >> 8) & 0x01);

		// Advance the bit position by 9 for the next iteration.
		current_total_bit_pos += 9;
		bits_remaining -= 9;
	}

	return bytes_processed;
}

nfca_pcd_controller_state_t nfca_pcd_get_comm_status() {
	if(g_nfca_pcd_comm_status > 2) {
		if(g_nfca_pcd_intf_mode) {
			R8_NFC_CMD &= 0xef;
			g_nfca_pcd_recv_buf_len = nfca_pcd_separate_recv_data(gs_nfca_data_buf, g_nfca_pcd_recv_bits, g_nfca_pcd_buf_offset, g_nfca_pcd_recv_buf, g_nfca_parity_buf, NFCA_PCD_MAX_RECV_NUM);
		}
		return g_nfca_pcd_comm_status;
	}
	return 0;
}

uint32_t nfca_pcd_prepare_send_data(uint8_t send_buf[], uint16_t num_bits, uint8_t parity_buf[], uint16_t data_buf[], uint32_t output_len) {
	if (num_bits <= 3) {
		return 0;
	}

	int processed_bytes = 0;
	int num_chunks = 0;

	// Process the data in 8-byte chunks.
	while ((num_bits - processed_bytes) >= 8) {
		if ((processed_bytes + 8) > output_len) {
			break;
		}

		for (int i = 0; i < 8; ++i) {
			// Combine data and parity into a 16-bit word.
			// If the parity byte is non-zero, the 9th bit is set.
			data_buf[processed_bytes +i] = (uint16_t)send_buf[processed_bytes +i] | (parity_buf[processed_bytes +i] > 0 ? 0x0100 : 0);
		}
		processed_bytes += 8;
		num_chunks++;
	}

	int remaining_len = num_bits - processed_bytes;

	// Process any remaining bytes that did not form a full 8-byte chunk.
	if (remaining_len > 0 && processed_bytes < output_len) {
		data_buf[num_chunks] = (uint16_t)send_buf[num_chunks] | (parity_buf[num_chunks] > 0 ? 0x0100 : 0);
	}

	return ((num_bits / 8) *9) + (num_bits % 8);
}

uint8_t nfca_pcd_comm(uint16_t data_bits_num, NFCA_PCD_REC_MODE_Def mode, uint8_t offset) {
	// Check for a minimum number of bits.
	if (data_bits_num <= 3) {
		g_nfca_pcd_intf_mode = 0;
		g_nfca_pcd_comm_status = 6; // Status: Error/Invalid Param
		return 1;
	}

	// Prepare the raw data buffers into a single packed buffer for the FIFO.
	int prepared_byte_len = nfca_pcd_prepare_send_data(g_nfca_pcd_send_buf, data_bits_num, g_nfca_parity_buf, gs_nfca_data_buf, NFCA_DATA_BUF_SIZE);

	// --- Configure NFC Hardware ---
	R8_NFC_STATUS = 0; // Clear control register

	// Clear command bits 0 and 4 (CMD_IDLE and CMD_MF_AUTH)
	R8_NFC_CMD &= ~(0x11);

	// Store current communication parameters in global state
	g_nfca_pcd_intf_mode = mode;
	g_nfca_pcd_buf_offset = offset;

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
			R16_NFC_FIFO = gs_nfca_data_buf[i];
		}
		g_nfca_pcd_send_fifo_bytes = 8;
	}
	else {
		// For smaller transfers, load all the words into the FIFO.
		for (int i = 0; i < num_words; i++) {
			R16_NFC_FIFO = gs_nfca_data_buf[i];
		}
		g_nfca_pcd_send_fifo_bytes = num_words;
	}

	// --- Start Transmission ---
	g_nfca_pcd_send_total_bytes = num_words;
	g_nfca_pcd_comm_status = 1; // Status: Transmitting

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
		if ((status != NFCA_PCD_CONTROLLER_STATE_FREE) || (overtimes > (NFCA_PCD_WAIT_MAX_MS * 10))) {
			break;
		}

		Delay_Us(100);
		overtimes++;
	}

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

// NFCA PICC functions
void nfca_picc_start() {
	R8_NFC_CMD = 0x48;
	R32_NFC_DRV =  (R32_NFC_DRV & 0xf9cf) | 0x620;

	// TMR0 will do rx from PCD, capturing the pulse trains
	R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
	R32_TMR0_DMA_BEG = (uint32_t)gs_nfca_picc_signal_buf;
	R32_TMR0_DMA_END = (uint32_t)&(gs_nfca_picc_signal_buf[NFCA_PICC_SIGNAL_BUF_SIZE]);
	R8_TMR0_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;

	R8_TMR0_INTER_EN = RB_TMR_IE_DATA_ACT;
	R8_TMR0_INT_FLAG = R8_TMR0_INT_FLAG; // clear interrupt flags
	R8_TMR0_CTRL_MOD = (RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
	NVIC_EnableIRQ(TMR0_IRQn);

	// TMR3 does PWM output for the tx to PCD, and also abuse the DMA PWM output mode to time the reponses on the FDT (Frame Delay Timing)
	R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

	R32_TMR3_CNT_END = TMR3_NFCA_PICC_CNT_END;
	R32_TMR3_FIFO = TMR3_CTRL_PWM_OFF;
	gs_nfca_picc_tx_buf[0] = TMR3_CTRL_PWM_OFF;
	R32_TMR3_DMA_BEG = (uint32_t)gs_nfca_picc_tx_buf;
	R32_TMR3_DMA_END = (uint32_t)&gs_nfca_picc_tx_buf[1];

	R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END; // clear flag
	R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;

	// Start the timer, which starts the whole DMA-driven transmission process
	R8_TMR3_CTRL_DMA = RB_TMR_DMA_ENABLE;
	R8_TMR3_CTRL_MOD = (RB_TMR_OUT_EN | (High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
	NVIC_EnableIRQ(TMR3_IRQn); // for tx
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
				if(ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_parity_buf, 2)) {
					return ((uint16_t *)(g_nfca_pcd_recv_buf))[0];
				}
				else {
					printf("ODD BIT ERROR\n");
					printf("data: 0x%02x 0x%02x\n", ((uint16_t *)(g_nfca_pcd_recv_buf))[0], ((uint16_t *)(g_nfca_pcd_recv_buf))[1]);
					printf("parity: %d %d\n", g_nfca_parity_buf[0], g_nfca_parity_buf[1]);
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
	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_parity_buf, 2);

	if (nfca_pcd_comm(16, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) {
		status = nfca_pcd_wait_comm_end();
		
		if(status == NFCA_PCD_CONTROLLER_STATE_DONE) {
			if (g_nfca_pcd_recv_bits == (5 * 9)) {
				if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_parity_buf, 5)) {
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

uint16_t PcdSelect(uint8_t cmd, uint8_t *pSnr) {
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

	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_parity_buf, 9);

	if (nfca_pcd_comm(9 * 8, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) {
		status = nfca_pcd_wait_comm_end();
		
		if(status == NFCA_PCD_CONTROLLER_STATE_DONE) {
			if (g_nfca_pcd_recv_bits == (3 * 9)) {
				if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_parity_buf, 3)) {
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

	ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_parity_buf, 4);

	nfca_pcd_comm((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);
	nfca_pcd_wait_comm_end();
}

// test
void nfca_pcd_test() {
	uint16_t res;

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

			res = PcdRequest(PICC_WUPA);
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

									printf("* Found an Ultralight, switching from PCD to PICC emulation.\n");
									nfca_pcd_stop();
									break;
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

static inline uint8_t check_parity(uint8_t bit_str[]) {
	char expected = '1';
	for(int i = 0; i < 8; i++) {
		if(bit_str[i] == '1') {
			expected = (expected == '1') ? '0' : '1';
		}
	}
	return (bit_str[8] == expected) ? 1 : 0;
}

static inline uint8_t bits_to_byte(uint8_t bit_str[]) {
	uint8_t byte_val = 0;
	for (int i = 0; i < 8; ++i) {
		if (bit_str[i] == '1') {
			byte_val |= (1 << i);
		}
	}
	return byte_val;
}

static inline uint8_t bits_to_frame(uint8_t bit_str[], int len) {
	// this is reusing (overwriting!) the input bit string!
	uint8_t num_bytes = 0;
	int has_eof = (bit_str[len -1] == '0') ? 1 : 0;

	if(bit_str[0] != '0') {
		// printf("* ERROR in bits_to_frame: SOF not zero\n");
	}
	else if((len -1 -has_eof) % 9) { // frame len is 9 bits per byte, + SOF,EOF
		// printf("* ERROR in bits_to_frame: bit string is incomplete\n");
	}
	else {
		for(int i = 1; i < (len -has_eof); i += 9) {
			if(check_parity(&bit_str[i])) {
				bit_str[num_bytes++] = bits_to_byte(&bit_str[i]);
			}
			else {
				num_bytes = 0;
				break;
				// printf("* ERROR in bits_to_frame: wrong parity of byte at index %d\n", i);
			}
		}
	}

	return num_bytes;
}

__HIGH_CODE
int decode_pulses_to_bits(uint16_t pulses[], int len, uint8_t *result_buf) {
	int bit_idx = 0;
	result_buf[bit_idx++] = '0'; // Initial state starts with a '0' bit

	for (int i = 0; i < len; ++i) {
		uint8_t last_bit = result_buf[bit_idx - 1];
		uint16_t pulse = pulses[i];

		if (last_bit == '0') {
			if (TMR0_NFCA_PICC_IS_1ETU(pulse)) {
				result_buf[bit_idx++] = '0';
			}
			else if (TMR0_NFCA_PICC_IS_1_5ETU(pulse)) {
				result_buf[bit_idx++] = '1';
			}
			else {
				return -1;
			}
		}
		else { // last_bit == '1'
			if (TMR0_NFCA_PICC_IS_1ETU(pulse)) {
				result_buf[bit_idx++] = '1';
			}
			else if (TMR0_NFCA_PICC_IS_1_5ETU(pulse)) {
				result_buf[bit_idx++] = '0';
				result_buf[bit_idx++] = '0';
			}
			else if (TMR0_NFCA_PICC_IS_2ETU(pulse)) {
				result_buf[bit_idx++] = '0';
				result_buf[bit_idx++] = '1';
			}
			else {
				return -1;
			}
		}
	}

	return bit_idx; // Return the number of bits decoded
}

__HIGH_CODE
uint16_t nfca_manchester_encode(int frame_delay_time, const uint8_t *data, uint8_t num_bytes) {
	uint16_t dma_idx = 0;
	uint8_t parity;

	gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
	for(int i = 0; i < frame_delay_time -1; i++) {
		gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
	}

	// Start of Frame (SOF)
	gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_ON;
	gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;

	for (uint8_t i = 0; i < num_bytes; i++) {
		uint8_t current_byte = data[i];
		parity = 0;

		// 8 data bits
		for (uint8_t j = 0; j < 8; j++) {
			uint8_t bit = (current_byte >> j) & 1;
			parity ^= bit;
			if (bit) { // '1' bit (High then Low subcarrier)
				gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_ON;
				gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
			}
			else { // '0' bit (Low then High subcarrier)
				gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
				gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_ON;
			}
		}

		// Parity bit (odd parity)
		if (parity) { // Parity is 1, send a '0' bit to make total odd
			gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
			gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_ON;
		} else { // Parity is 0, send a '1' bit
			gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_ON;
			gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
		}
	}

	// End of Frame
	gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;
	gs_nfca_picc_tx_buf[dma_idx++] = TMR3_CTRL_PWM_OFF;

	return dma_idx;
}

__HIGH_CODE
void wch_nfca_picc_prepare_tx_dma(int silence, uint8_t *data, uint8_t num_bytes) {
	// Encode the data into the DMA buffer format
	uint16_t dma_len = nfca_manchester_encode(silence, data, num_bytes);
	R32_TMR3_DMA_BEG = (uint32_t)gs_nfca_picc_tx_buf;
	R32_TMR3_DMA_END = (uint32_t)&gs_nfca_picc_tx_buf[dma_len];
}

__HIGH_CODE
int wch_nfca_standard_frame_response(uint8_t *data, uint8_t num_bytes, int req_resp_debug) {
	// Create response for PCD request
	int res = 0;
	int page_start = 0;

	if(req_resp_debug) {
		printf("req (%d): [0x%02x", num_bytes, data[0]);
		for(int i = 1; i < num_bytes; i++) {
			printf(" 0x%02x", data[i]);
		}
		printf("]\n");
	}

	switch(data[0]) {
	case PICC_READ:
		page_start = data[1] *4;
		for(int i = 0; i < 16; i++) {
			data[i] = picc_ultralight_pages[i +page_start];
			ISO14443AAppendCRCA(data, 16);
			res = 18;
		}
		break;
	default:
		res = 0; // no response
		break;
	}

	if(req_resp_debug) {
		printf("resp (%d): [0x%02x", res, data[0]);
		for(int i = 1; i < res; i++) {
			printf(" 0x%02x", data[i]);
		}
		printf("]\n");
	}

	return res;
}

__HIGH_CODE
int wch_nfca_anticoll_response(uint8_t *data, uint8_t num_bytes, int req_resp_debug) {
	// Create response for PCD request
	int i = 0;
	int res = 0;

	if(req_resp_debug) {
		printf("req (%d): [0x%02x", num_bytes, data[0]);
		for(int i = 1; i < num_bytes; i++) {
			printf(" 0x%02x", data[i]);
		}
		printf("]\n");
	}

	switch(data[0]) {
	case PICC_ANTICOLL1:
		switch(data[1]) {
		case 0x20: // requested 4 bytes
			if(gs_picc_uid_len > 4) {
				data[i++] = 0x88;
			}
			else {
				data[3] = picc_uid[3];
			}
			data[i++] = picc_uid[0];
			data[i++] = picc_uid[1];
			data[i++] = picc_uid[2];

			data[4] = data[0] ^ data[1] ^ data[2] ^ data[3]; // Block Check Character
			res = 5;
			break;
		case 0x70: // full frame
			if((gs_picc_uid_len > 4) && (data[3] == picc_uid[0]) && (data[4] == picc_uid[1]) && (data[5] == picc_uid[2])) {
				// acknowledge
				data[0] = 0x04;
				ISO14443AAppendCRCA(data, 1);
				res = 3;
			}
			else if((data[2] == picc_uid[0]) && (data[3] == picc_uid[1]) && (data[4] == picc_uid[2]) && (data[5] == picc_uid[3])) {
				// acknowledge
				data[0] = 0x00;
				ISO14443AAppendCRCA(data, 1);
				res = 3;
			}
			else {
				// not ours, don't respond
				res = 0;
			}
			break;
		}
		break;
	case PICC_ANTICOLL2:
		switch(data[1]) {
		case 0x20: // requested 4 bytes
			if(gs_picc_uid_len > 7) {
				data[i++] = 0x88;
			}
			else {
				data[3] = picc_uid[6];
			}
			data[i++] = picc_uid[3];
			data[i++] = picc_uid[4];
			data[i++] = picc_uid[5];

			data[4] = data[0] ^ data[1] ^ data[2] ^ data[3]; // Block Check Character
			res = 5;
			break;
		case 0x70: // full frame
			if((gs_picc_uid_len > 7) && (data[3] == picc_uid[3]) && (data[4] == picc_uid[4]) && (data[5] == picc_uid[5])) {
				// acknowledge
				data[0] = 0x04;
				ISO14443AAppendCRCA(data, 1);
				res = 3;
			}
			else if((data[2] == picc_uid[3]) && (data[3] == picc_uid[4]) && (data[4] == picc_uid[5]) && (data[5] == picc_uid[6])) {
				// acknowledge
				data[0] = 0x00;
				ISO14443AAppendCRCA(data, 1);
				res = 3;
			}
			else {
				// not ours, don't respond
				res = 0;
			}
			break;
		}
		break;
	case PICC_ANTICOLL3:
		switch(data[1]) {
		case 0x20: // requested 4 bytes
			data[0] = picc_uid[6];
			data[1] = picc_uid[7];
			data[2] = picc_uid[8];
			data[3] = picc_uid[9];
			data[4] = data[0] ^ data[1] ^ data[2] ^ data[3]; // Block Check Character
			res = 5;
			break;
		case 0x70: // full frame
			if((data[2] == picc_uid[6]) && (data[3] == picc_uid[7]) && (data[4] == picc_uid[8]) && (data[5] == picc_uid[9])) {
				// acknowledge
				data[0] = 0x00;
				ISO14443AAppendCRCA(data, 1);
				res = 3;
			}
			else {
				// not ours, don't respond
				res = 0;
			}
			break;
		}
		break;
	case PICC_HALT: // Fall-through
	default:
		res = 0; // no response
		break;
	}

	if(req_resp_debug) {
		printf("resp (%d): [0x%02x", res, data[0]);
		for(int i = 1; i < res; i++) {
			printf(" 0x%02x", data[i]);
		}
		printf("]\n");
	}

	return res;
}

__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void) {
	R8_TMR0_INT_FLAG = R8_TMR0_INT_FLAG;	// Acknowledge

	// Get the index of the NEXT free spot in the DMA buffer.
	int current_dma_idx = (R32_TMR0_DMA_NOW - R32_TMR0_DMA_BEG) / sizeof(uint32_t);
	while(gs_picc_last_processed_dma_idx != current_dma_idx) {
		uint16_t value_from_dma = (uint16_t)gs_nfca_picc_signal_buf[gs_picc_last_processed_dma_idx];

		// rise to rise timings are always 1, 1.5 or 2 et_u (128 ticks in our case)
		if(TMR0_NFCA_PICC_IS_1ETU(value_from_dma) ||
			TMR0_NFCA_PICC_IS_1_5ETU(value_from_dma) ||
			TMR0_NFCA_PICC_IS_2ETU(value_from_dma)) {
			gs_nfca_data_buf[gs_picc_data_idx++] = value_from_dma;
		}
		else {
			// noise, restart pulse train capture
			gs_picc_data_idx = 0;
			gs_picc_anticoll_cmd_guess = 0;
		}

		// Advance our tracking index, wrapping around the circular DMA buffer.
		gs_picc_last_processed_dma_idx = (gs_picc_last_processed_dma_idx + 1) % NFCA_PICC_SIGNAL_BUF_SIZE;
	}

	if(gs_picc_data_idx > 4) {
		uint16_t pulse_sum = 0;
		for(int i = 0; i < 5 && !gs_picc_anticoll_cmd_guess; i++) {
			// WUPA 0x26, sum([128, 192, 192, 192, 256]) = 960
			// REQA 0x52, sum([128, 192, 128, 192, 192, ...]) = 832
			// HALT 0x50, sum([128, 128, 128, 128, 192, ...]) = 704
			// SEL1 0x93, sum([192, 128, 192, 192, 192, ...]) = 896
			// SEL2 0x95, sum([192, 256, 256, 192, 192, ...]) = 1088
			// SEL3 0x97, sum([192, 128, 128, 256, 192, ...]) = 896
			// READ 0x30, sum([128, 128, 128, 128, 192, ...]) = 704
			pulse_sum += gs_nfca_data_buf[i];
		}

		if(!gs_picc_anticoll_cmd_guess) {
			switch(gs_picc_data_idx) {
			case 5: // WUPA 0x26, [128, 192, 192, 192, 256]
				if(PLUSMIN_7(pulse_sum, 960)) { // WUPA
					gs_picc_anticoll_cmd_guess = PICC_WUPA;
				}
				break;
			case 6: // REQA 0x52, [128, 192, 128, 192, 192, 192]
				if(PLUSMIN_7(pulse_sum, 832)) { // REQA
					gs_picc_anticoll_cmd_guess = PICC_REQA;
				}
				break;
			case 7: // HALT 0x50, SEL1 0x93, SEL1 0x95, SEL1 0x97
				if(PLUSMIN_7(pulse_sum, 704)) { // HALT or READ
					switch(gs_nfca_data_buf[5] >> 3) {
					case 256 >> 3:
						gs_picc_anticoll_cmd_guess = PICC_HALT;
						break;
					case 128 >> 3:
						gs_picc_anticoll_cmd_guess = PICC_READ;
						break;
					}
				}
				else if(PLUSMIN_7(pulse_sum, 896) || // SEL1,3
						PLUSMIN_7(pulse_sum, 1088)) { // SEL2
					switch(gs_nfca_data_buf[2] >> 3) { // >> 3 = +/-7
					case 192 >> 3:
						gs_picc_anticoll_cmd_guess = PICC_ANTICOLL1;
						break;
					case 256 >> 3:
						gs_picc_anticoll_cmd_guess = PICC_ANTICOLL2;
						break;
					case 128 >> 3:
						gs_picc_anticoll_cmd_guess = PICC_ANTICOLL3;
						break;
					}
				}
				break;
			}
		}

		// set TMR3 timer for preparing response
		if(gs_picc_state != PICC_STATE_SEND_RESP) {
			int req_len = decode_pulses_to_bits(gs_nfca_data_buf, /*len=*/gs_picc_data_idx, gs_picc_req_resp);
			uint8_t cmd = 0;
			uint8_t frame_len = 0;
			uint8_t resp_len = 0;
			uint8_t preptime = 0;
			uint16_t crc = 0;

			if(req_len > 0) {
				switch(gs_picc_anticoll_cmd_guess) {
				case PICC_REQA:
				case PICC_WUPA:
					cmd = bits_to_byte(gs_picc_req_resp) >> 1; // >> 1 to discard EOF 0 last bit
					if(cmd == gs_picc_anticoll_cmd_guess) {
						// gs_debug_blink = 1;
						gs_picc_state = PICC_STATE_SEND_RESP;

						R32_TMR3_FIFO = TMR3_CTRL_PWM_OFF;
						gs_picc_req_resp[0] = ATQA[0];
						gs_picc_req_resp[1] = ATQA[1];
						wch_nfca_picc_prepare_tx_dma(/*silence=*/TMR3_NFCA_PICC_FTD, gs_picc_req_resp, 2);
				
						R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END; // clear flag
						R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;
				
						// Start the timer, which starts the whole DMA-driven transmission process
						R8_TMR3_CTRL_DMA = RB_TMR_DMA_ENABLE;
					}
					else {
						gs_picc_anticoll_cmd_guess = 0;
						gs_picc_state = PICC_STATE_FREE;
						printf("guess was wrong: 0x%02x != 0x%02x", cmd, gs_picc_anticoll_cmd_guess);
					}
					break;
				case PICC_HALT:
				case PICC_ANTICOLL1:
				case PICC_ANTICOLL2:
				case PICC_ANTICOLL3:
				case PICC_READ:
					frame_len = bits_to_frame(gs_picc_req_resp, req_len); // gs_picc_req_resp is reused as output
					if(frame_len) {
						if(gs_picc_req_resp[0] == gs_picc_anticoll_cmd_guess) {
							crc = ISO14443_CRCA(gs_picc_req_resp, frame_len -2);
							if(((frame_len == 2) && (gs_picc_req_resp[1] == 0x20)) ||
									((crc >> 8 == gs_picc_req_resp[frame_len -1]) && ((crc & 0xff) == gs_picc_req_resp[frame_len -2]))) {

								if(gs_picc_anticoll_cmd_guess == PICC_READ) {
									resp_len = wch_nfca_standard_frame_response(gs_picc_req_resp, frame_len, /*debug print=*/0);
								}
								else {
									resp_len = wch_nfca_anticoll_response(gs_picc_req_resp, frame_len, /*debug print=*/0);
								}

								if(resp_len) {
									gs_picc_state = PICC_STATE_SEND_RESP;

									R32_TMR3_FIFO = TMR3_CTRL_PWM_OFF;
									preptime = (frame_len + resp_len); // !!! TWEAK THE TIMING HERE !!!
									wch_nfca_picc_prepare_tx_dma(/*silence=*/TMR3_NFCA_PICC_FTD -preptime, gs_picc_req_resp, resp_len);

									R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END; // clear flag
									R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;

									// Start the timer, which starts the whole DMA-driven transmission process
									R8_TMR3_CTRL_DMA = RB_TMR_DMA_ENABLE;
								}
							}
						}
						else {
							gs_picc_anticoll_cmd_guess = 0;
							gs_picc_state = PICC_STATE_FREE;
							printf("guess was wrong: 0x%02x != 0x%02x", gs_picc_req_resp[0], gs_picc_anticoll_cmd_guess);
						}
					}
					break;
				default:
					break;
				}
			}
			else {
				gs_picc_anticoll_cmd_guess = 0;
			}
		}
	}

	if(gs_picc_state == PICC_STATE_SEND_RESP) {
		NVIC_ClearPendingIRQ(TMR0_IRQn);
	}
}

__INTERRUPT
__HIGH_CODE
void TMR3_IRQHandler(void) {
	R8_TMR3_INT_FLAG = R8_TMR3_INT_FLAG; // Acknowledge

	R8_TMR3_CTRL_DMA = 0;
	R8_TMR3_INTER_EN = 0;
	gs_picc_state = PICC_STATE_FREE;

	NVIC_ClearPendingIRQ(TMR3_IRQn);
}

int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

	printf("~ ch585 NFC ~\n");
	printf("* Waiting for Mifare Ultralight.\n");
	blink(5);
	
	nfca_init();

#ifdef CLONE_ULTRALIGHT
	nfca_pcd_lpcd_calibration();
	nfca_pcd_test(); // handles nfca_pcd_start() and _stop()
#else
	uint8_t ultralight_ID[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
	memcpy(picc_uid, ultralight_ID, sizeof(ultralight_ID));
#endif
	
	uint8_t ultralight_ATQA[] = {0x44, 0x00};
	memcpy(ATQA, ultralight_ATQA, sizeof(ultralight_ATQA));
	
	printf("* Emulating Ultralight with uid %02x %02x %02x %02x %02x %02x %02x\n",
			picc_uid[0], picc_uid[1], picc_uid[2], picc_uid[3], picc_uid[4], picc_uid[5], picc_uid[6]);
	gs_picc_uid_len = 7;

	int i = 0;
	// [ID0, ID1, ID2, BCC1]
	picc_ultralight_pages[i++] = picc_uid[0];
	picc_ultralight_pages[i++] = picc_uid[1];
	picc_ultralight_pages[i++] = picc_uid[2];
	picc_ultralight_pages[i++] = (0x88 ^ picc_uid[0] ^ picc_uid[1] ^ picc_uid[2]); // ultralight uid is 7-bit

	// [ID3, ID4, ID5, ID6]]
	picc_ultralight_pages[i++] = picc_uid[3];
	picc_ultralight_pages[i++] = picc_uid[4];
	picc_ultralight_pages[i++] = picc_uid[5];
	picc_ultralight_pages[i++] = picc_uid[6];

	// [BCC2, 0x48, Lock0, Lock1] (0x48 is Internal/Manufacturer data)
	picc_ultralight_pages[i++] = (picc_uid[0] ^ picc_uid[0] ^ picc_uid[0] ^ picc_uid[0]);
	picc_ultralight_pages[i++] = 0x48;
	picc_ultralight_pages[i++] = 0x00; // 0x00: writable
	picc_ultralight_pages[i++] = 0x00; // 0x00: writable

	// [OTP0, OTP1, OTP2, OTP3] (One Time Programmable bytes).
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;

	// User memory
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;

	// User memory
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	picc_ultralight_pages[i++] = 0x00;
	
	nfca_picc_start();

	while(1) {
		if(gs_debug_blink) {
			blink(gs_debug_blink);
			gs_debug_blink = 0;
		}
	}
}
