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

#define RB_TMR_FREQ_13_56             0x20
#define TMR0_NFCA_PICC_CNT_END        288
#define TMR3_NFCA_PICC_CNT_END        18


#define NFCA_DATA_BUF_SIZE                       512
#define NFCA_PICC_SIGNAL_BUF_SIZE                512
#define NFCA_PCD_MAX_SEND_NUM                    (NFCA_DATA_BUF_SIZE)
#define NFCA_PCD_MAX_RECV_NUM                    (NFCA_DATA_BUF_SIZE * 16 / 9)
#define NFCA_MAX_PARITY_NUM                      (NFCA_PCD_MAX_RECV_NUM)
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

__attribute__((aligned(4))) static uint16_t gs_nfca_data_buf[NFCA_DATA_BUF_SIZE];
__attribute__((aligned(4))) uint8_t g_nfca_parity_buf[NFCA_MAX_PARITY_NUM];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) static uint32_t gs_nfca_picc_signal_buf[NFCA_PICC_SIGNAL_BUF_SIZE];
__attribute__((aligned(4))) static uint16_t gs_lpcd_adc_filter_buf[8];
uint8_t picc_uid[7];

static uint16_t gs_lpcd_adc_base_value;
uint16_t g_nfca_pcd_recv_buf_len;
uint32_t g_nfca_pcd_recv_bits;
uint16_t g_nfca_pcd_recv_word_idx;
uint16_t g_nfca_pcd_send_fifo_bytes;
uint16_t g_nfca_pcd_send_total_bytes;
uint8_t g_nfca_pcd_buf_offset;
uint8_t g_nfca_pcd_intf_mode;
uint8_t g_nfca_pcd_comm_status;

volatile uint32_t g_picc_data_idx = 0;
volatile uint32_t g_picc_last_processed_dma_idx = 0;

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

__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void) {
	R8_TMR0_INT_FLAG = R8_TMR0_INT_FLAG; // Acknowledge

	// Get the index of the NEXT free spot in the DMA buffer.
	uint32_t current_dma_idx = (R32_TMR0_DMA_NOW - R32_TMR0_DMA_BEG) / sizeof(uint32_t);

	while (g_picc_last_processed_dma_idx != current_dma_idx) {
		if (g_picc_data_idx < NFCA_DATA_BUF_SIZE) {
			uint32_t value_from_dma = gs_nfca_picc_signal_buf[g_picc_last_processed_dma_idx];
			// rise to rise timings are never below one et_u (128 ticks in our case) or more than 2 et_u (101)
			if(96 < value_from_dma && value_from_dma < 288) {
				gs_nfca_data_buf[g_picc_data_idx++] = (uint16_t)value_from_dma;
			}
			else if(gs_nfca_data_buf[g_picc_data_idx -1] != 0xFFFF) {
				// don't write pulse trains less than 7 bits, overwrite that last one on next trigger
				if(gs_nfca_data_buf[g_picc_data_idx -2] == 0xFFFF) {
					g_picc_data_idx -= 1;
				}
				else if(gs_nfca_data_buf[g_picc_data_idx -3] == 0xFFFF) {
					g_picc_data_idx -= 2;
				}
				else if(gs_nfca_data_buf[g_picc_data_idx -4] == 0xFFFF) {
					g_picc_data_idx -= 3;
				}
				else if(gs_nfca_data_buf[g_picc_data_idx -5] == 0xFFFF) {
					g_picc_data_idx -= 4;
				}
				else if(gs_nfca_data_buf[g_picc_data_idx -6] == 0xFFFF) {
					g_picc_data_idx -= 5;
				}
				else if(gs_nfca_data_buf[g_picc_data_idx -7] == 0xFFFF) {
					g_picc_data_idx -= 6;
				}
				else {
					gs_nfca_data_buf[g_picc_data_idx++] = 0xFFFF;
				}
			}
		}

		// Advance our tracking index, wrapping around the circular DMA buffer.
		g_picc_last_processed_dma_idx = (g_picc_last_processed_dma_idx + 1) % NFCA_PICC_SIGNAL_BUF_SIZE;
	}

	NVIC_ClearPendingIRQ(TMR0_IRQn);
}

__INTERRUPT
__HIGH_CODE
void TMR3_IRQHandler(void){
	R8_TMR3_CTRL_DMA = 0;
	R32_TMR3_DMA_END = R32_TMR3_DMA_NOW + 0x100;
	R32_TMR0_CNT_END = 0x120;
	R8_TMR0_CTRL_DMA = 5;
	R8_TMR0_CTRL_MOD = 0xe5;

	NVIC_ClearPendingIRQ(TMR0_IRQn);
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
	funPinMode( (PB8 | PB9 | PB16 | PB17), GPIO_CFGLR_IN_FLOAT );
	
	R32_PIN_IN_DIS |= (((PB8 | PB9) & ~PB)<< 16);
	R16_PIN_CONFIG |= (((PB16 | PB17) & ~PB) >> 8);
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

	R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

	R8_TMR3_CTRL_MOD = (Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;

	R32_TMR3_CNT_END = TMR3_NFCA_PICC_CNT_END;
	R32_TMR3_FIFO = 0;
	R32_TMR3_DMA_END = R32_TMR3_DMA_NOW + 0x100;
	R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END;
	R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;

	R8_TMR3_CTRL_MOD = ((Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);

	R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
	R8_TMR0_CTRL_MOD = RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56;
	R32_TMR0_CNT_END = TMR0_NFCA_PICC_CNT_END;

	R32_TMR0_DMA_END = R32_TMR0_DMA_NOW + 0x100;
	R8_TMR0_INT_FLAG = RB_TMR_IF_DMA_END;

	R32_TMR0_DMA_BEG = (uint32_t)gs_nfca_picc_signal_buf;
	R32_TMR0_DMA_END = (uint32_t)&(gs_nfca_picc_signal_buf[NFCA_PICC_SIGNAL_BUF_SIZE]);
	R8_TMR0_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;

	R8_TMR0_INT_FLAG = RB_TMR_IF_DATA_ACT;
	R8_TMR0_INTER_EN = RB_TMR_IE_DATA_ACT;

	R8_TMR0_CTRL_MOD = (RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);

	NVIC_EnableIRQ(TMR3_IRQn);
	NVIC_EnableIRQ(TMR0_IRQn);
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

// For NFC at 106 kbps, 1 ETU is ~128 / 13.56 MHz = ~9.44 us.
#define ETU           128
#define ETU_TOLERANCE 16

static inline int is_close(uint16_t val, uint16_t target, uint16_t tol) {
	uint16_t diff = (val > target) ? (val - target) : (target - val);
	return diff < tol;
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
	uint8_t num_bytes = 0;
	if(bit_str[0] != '0' || bit_str[len -1] != '0') {
		printf("* ERROR in bits_to_frame: SOF/EOF not zero\n");
	}
	else if((len -2) % 9) { // frame len is 9 bits per byte, + SOF,EOF
		printf("* ERROR in bits_to_frame: bit string is incomplete\n");
	}
	else {
		for(int i = 1; i < (len -1); i += 9) {
			if(check_parity(&bit_str[i])) {
				bit_str[num_bytes++] = bits_to_byte(&bit_str[i]);
			}
			else {
				printf("* ERROR in bits_to_frame: wrong parity of byte at index %d\n", i);
			}
		}
	}

	return num_bytes;
}

int8_t decode_pulses_to_bits(uint16_t pulses[], int len, uint8_t *result_buf) {
	int8_t bit_idx = 0;
	result_buf[bit_idx++] = '0'; // Initial state starts with a '0' bit

	for (int i = 0; i < len; ++i) {
		uint8_t last_bit = result_buf[bit_idx - 1];
		uint16_t pulse = pulses[i];

		if (last_bit == '0') {
			if (is_close(pulse, ETU, ETU_TOLERANCE)) {
				result_buf[bit_idx++] = '0';
			}
			else if (is_close(pulse, (ETU * 3 / 2), ETU_TOLERANCE)) { // 1.5 * ETU
				result_buf[bit_idx++] = '1';
			}
			else {
				return -1;
			}
		}
		else { // last_bit == '1'
			if (is_close(pulse, ETU, ETU_TOLERANCE)) {
				result_buf[bit_idx++] = '1';
			}
			else if (is_close(pulse, (ETU * 3 / 2), ETU_TOLERANCE)) { // 1.5 * ETU
				result_buf[bit_idx++] = '0';
				result_buf[bit_idx++] = '0';
			}
			else if (is_close(pulse, (ETU * 2), ETU_TOLERANCE)) { // 2.0 * ETU
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

int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

	printf("~ ch585 NFC ~\n");
	printf("* Waiting for Mifare Ultralight.\n");
	blink(5);
	
	// nfca_init();
	// nfca_pcd_lpcd_calibration();
	// nfca_pcd_test(); // handles nfca_pcd_start() and _stop()

	printf("* Emulating Ultralight with uid %02x %02x %02x %02x %02x %02x %02x\n",
			picc_uid[0], picc_uid[1], picc_uid[2], picc_uid[3], picc_uid[4], picc_uid[5], picc_uid[6]);

	uint16_t pcd_pulses[NFCA_DATA_BUF_SIZE] = {0};
	uint8_t pcd_req[NFCA_DATA_BUF_SIZE] = {0};
	nfca_picc_start();

	while(1) {
		if(g_picc_data_idx > 7) {
			uint32_t npulses = g_picc_data_idx;
			Delay_Us(50); // wait 5 et_u to see if there are more pulses incoming
			if (g_picc_data_idx > npulses) {
				continue;
			}

			NVIC_DisableIRQ(TMR0_IRQn);
			int buf_copy_idx = 0;
			for(int i = 0; i < g_picc_data_idx; i++) {
				if(gs_nfca_data_buf[i] < 0xFFFF) {
					pcd_pulses[buf_copy_idx++] = gs_nfca_data_buf[i];
				}
			}

			memset(pcd_req, 0, sizeof(pcd_req));
			// first pulse is some initializer, discard that
			int req_len = decode_pulses_to_bits(&pcd_pulses[1], /*len=*/buf_copy_idx -1, pcd_req);
			printf("req(%d): %s\n", req_len, pcd_req);

			if(pcd_req[0] != '0' || pcd_req[req_len -1] != '0') {
				printf("* ERROR: SOF/EOF not zero\n");
			}
			else if(req_len == 9) { // short frame
				uint8_t b = bits_to_byte(pcd_req);
				printf("cmd: 0x%02x\n", b >> 1); // discard EOF 0 last bit
			}
			else { // normal frame, need to check parity bits
				uint8_t frame_len = bits_to_frame(pcd_req, req_len); // pcd_req is reused as output
				printf("frame: [%02x", pcd_req[0]);
				for(int i = 1; i < frame_len; i++) {
					printf(" %02x", pcd_req[i]);
				}
				printf("]\n");
			}

			g_picc_data_idx = 0;
			NVIC_EnableIRQ(TMR0_IRQn);
		}
	}
}
