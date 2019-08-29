/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t packet_length;
static uint8_t rx_buffer[16];
static void (*handler)(uint8_t byte_count, uint8_t bytes[]);

volatile uint8_t pending_int = 0;
volatile uint8_t startup = 1;

#define SPI1_CS	GPIO_PIN_4
#define SPI2_CS	GPIO_PIN_12

#define RF_RECEIVE_BUS	GPIOB

#define MAX_NOOP        0x00    /**< No operation adresa. */
#define MAX_DIGIT0      0x01    /**< Adresa cifre 0 ili reda 0. */
#define MAX_DIGIT1      0x02    /**< Adresa cifre 1 ili reda 1. */
#define MAX_DIGIT2      0x03    /**< Adresa cifre 2 ili reda 2. */
#define MAX_DIGIT3      0x04    /**< Adresa cifre 3 ili reda 3. */
#define MAX_DIGIT4      0x05    /**< Adresa cifre 4 ili reda 4. */
#define MAX_DIGIT5      0x06    /**< Adresa cifre 5 ili reda 5. */
#define MAX_DIGIT6      0x07    /**< Adresa cifre 6 ili reda 6. */
#define MAX_DIGIT7      0x08    /**< Adresa cifre 7 ili reda 7. */
#define MAX_DECODEMODE  0x09    /**< Adresa registra za izbor moda dekodiranja. Mi koristimo NO DECODE MODE(0x00). */
#define MAX_INTENSITY   0x0A    /**< Adresa registra za kontrolu intenziteta osvetljaja dioda(od 0x00 do 0x0F). */
#define MAX_SCANLIMIT   0x0B    /**< Adresa registra za kontrolu broja prikazanih cifara. Mi prikazujemo sve cifre(0x07), odnosno sve clanove niza. */
#define MAX_SHUTDOWN    0x0C    /**< Adresa registra koji ukljucuje/iskljucuje kontroler. */
#define MAX_DISPLAYTEST 0x0F    /**< Adresa registra za izbor normalnog moda rada(0x00) ili display test moda(0x01). */


// Access modes for the configuration and multi-byte registers. Add/or them to a register name to set the appropriate bits.
#define WRITE_BYTE		0x00
#define WRITE_BURST		0x40
#define READ_BYTE			0x80
#define READ_BURST		0xC0

// Configuration registers.
#define IOCFG2			0x00	/* pg61,53	GDO2 output pin config */
#define IOCFG1			0x01	/* pg61,53	GDO1 output pin config */
#define IOCFG0			0x02	/* pg61,53	GDO0 output pin config */
#define FIFOTHR			0x03	/* pg62		RX FIFO and TX FIFO thresholds */
#define SYNC1				0x04	/* pg62		sync word, high byte */
#define SYNC0				0x05	/* pg62		sync word, low byte */
#define PKTLEN			0x06	/* pg62		packet length */
#define PKTCTRL1		0x07	/* pg63		packet automation control */
#define PKTCTRL0		0x08	/* pg64		packet automation control */
#define ADDR				0x09	/* pg64		device address */
#define CHANNR			0x0A	/* pg64		channel number */
#define FSCTRL1			0x0B	/* pg65		frequency synthesizer control */
#define FSCTRL0			0x0C	/* pg65		frequency synthesizer control */
#define FREQ2				0x0D	/* pg65		frequency control word, high byte */
#define FREQ1				0x0E	/* pg65		frequency control word, middle byte */
#define FREQ0				0x0F	/* pg65		frequency control word, low byte */
#define MDMCFG4			0x10	/* pg66		modem configuration */
#define MDMCFG3			0x11	/* pg66		modem configuration */
#define MDMCFG2			0x12	/* pg67		modem configuration */
#define MDMCFG1			0x13	/* pg68		modem configuration */
#define MDMCFG0			0x14	/* pg68		modem configuration */
#define DEVIATN			0x15	/* pg69		modem deviation setting */
#define MCSM2				0x16	/* pg70		main radio control state machine config */
#define MCSM1				0x17	/* pg71		main radio control state machine config */
#define MCSM0				0x18	/* pg72		main radio control state machine config */
#define FOCCFG			0x19	/* pg73		frequency offset compensation config */
#define BSCFG				0x1A	/* pg74		bit synchronization config */
#define AGCCTRL2		0x1B	/* pg75		agc control */
#define AGCCTRL1		0x1C	/* pg76		agc control */
#define AGCCTRL0		0x1D	/* pg77		agc control */
#define WOREVT1			0x1E	/* pg77		event0 timeout, high byte */
#define WOREVT0			0x1F	/* pg78		event0 timeout, low byte */
#define WORCTRL			0x20	/* pg78		wake on radio control */
#define FREND1			0x21	/* pg78		front end rx config */
#define FREND0			0x22	/* pg79		front end tx config */
#define FSCAL3			0x23	/* pg79		frequency synthesizer calibration */
#define FSCAL2			0x24	/* pg79		frequency synthesizer calibration */
#define FSCAL1			0x25	/* pg80		frequency synthesizer calibration */
#define FSCAL0			0x26	/* pg80		frequency synthesizer calibration */
#define RCCTRL1			0x27	/* pg80		rc oscillator config */
#define RCCTRL0			0x28	/* pg80		rc oscillator config */
#define FSTEST			0x29	/* pg80		frequency synthesizer calibration control */
#define PTEST				0x2A	/* pg80		production test */
#define AGCTEST			0x2B	/* pg81		agc test */
#define TEST2				0x2C	/* pg82		various test settings */
#define TEST1				0x2D	/* pg82		various test settings */
#define TEST0				0x2E	/* pg82		various test settings */

// Multi-byte registers.
#define PATABLE			0x3E	/* pg46		output power setting */
#define FIFO				0x3F	/* pg43		read the RX FIFO or write the TX FIFO one byte at a time */

// Command strobes. Will be written as individual bytes (not bursts)
#define SRES				(0x30 + WRITE_BYTE)		/* pg57		reset chip */
#define SFSTXON			(0x31 + WRITE_BYTE)		/* pg57		if MCSM0.FS_AUTOCAL=1: enable and calibrate frequency synthesizer
															if RX with CCA: go to wait state (only synth. running) for quick TX/RX turnaround */
#define SXOFF				(0x32 + WRITE_BYTE)		/* pg57		turn off crystal oscillator */
#define SCAL				(0x33 + WRITE_BYTE)		/* pg57		calibrate frequency synthesizer and turn it off */
#define SRX					(0x34 + WRITE_BYTE)		/* pg57		enable RX. will also perform calibration if coming from idle and MCSM0.FS_AUTOCAL=1 */
#define STX					(0x35 + WRITE_BYTE)		/* pg57		enable TX. will also perform calibration if coming from idle and MCSM0.FS_AUTOCAL=1
															if RX with CCA: only go to TX mode if channel clear */
#define SIDLE				(0x36 + WRITE_BYTE)		/* pg57		exit TX/RX mode. turns off frequency synthesizer and exits WOR mode if applicable. */
#define SWOR				(0x38 + WRITE_BYTE)		/* pg57		start automatic RX polling if WORCTRL.RC_PD=1 */
#define SPWD				(0x39 + WRITE_BYTE)		/* pg57		enter power down mode when CS goes high */
#define SFRX				(0x3A + WRITE_BYTE)		/* pg57		flush the RX FIFO. only use when in IDLE or RXFIFO_OVERFLOW states */
#define SFTX				(0x3B + WRITE_BYTE)		/* pg57		flush the TX FIFO. only use when in IDLE or TXFIFO_UNDERFLOW states */
#define SWORRST			(0x3C + WRITE_BYTE)		/* pg57		reset RTC to Event1 value */
#define SNOP				(0x3D + WRITE_BYTE)		/* pg57		no operation */

// Status registers. Will be read as individual bytes (NOT actually a burst as listed below.)
#define PARTNUM					(0x30 + READ_BURST)		/* pg81		chip part number, = 0x80 */
#define VERSION					(0x31 + READ_BURST)		/* pg81		chip version number, = 0x03 */
#define FREQEST					(0x32 + READ_BURST)		/* pg81		frequency offset estimate from demodulator */
#define LQI							(0x33 + READ_BURST)		/* pg82		link quality estimate from demodulator */
#define RSSI						(0x34 + READ_BURST)		/* pg82		received signal strength indication */
#define MARCSTATE				(0x35 + READ_BURST)		/* pg82		main radio control state machine state */
#define WORTIME1				(0x36 + READ_BURST)		/* pg83		WOR time, high byte */
#define WORTIME0				(0x37 + READ_BURST)		/* pg83		WOR time, low byte */
#define PKTSTATUS				(0x38 + READ_BURST)		/* pg83		current GDOx status and packet status */
#define VCO_VC_DAC			(0x39 + READ_BURST)		/* pg83		current setting from PLL calibration module */
#define TXBYTES					(0x3A + READ_BURST)		/* pg83		underflow and number of bytes in TX FIFO */
#define RXBYTES					(0x3B + READ_BURST)		/* pg84		overflow and number of bytes in RX FIFO */
#define RCCTRL1_STATUS	(0x3C + READ_BURST)		/* pg85		last rc oscillator calibration result */
#define RCCTRL0_STATUS	(0x3D + READ_BURST)		/* pg85		last rc oscillator calibration result */

const uint8_t ucNumber[50][8] = {
	{0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 000 (.)
	{0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 001 (.)
	{0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 002 (.)
	{0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 003 (.)
	{0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 004 (.)
	{0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 005 (.)
	{0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 006 (.)
	{0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 007 (.)
	{0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 008 (.)
	{0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 009 (.)
	{0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 010 (.)
	{0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 011 (.)
	{0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 012 (.)
	{0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 013 (.)
	{0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00},	// Char 014 (.)
	{0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00},	// Char 015 (.)
	{0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00},	// Char 016 (.)
	{0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},	// Char 017 (.)
	{0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00},	// Char 018 (.)
	{0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00},	// Char 019 (.)
	{0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00},	// Char 020 (.)
	{0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00},	// Char 021 (.)
	{0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},	// Char 022 (.)
	{0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00},	// Char 023 (.)
	{0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00},	// Char 024 (.)
	{0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00},	// Char 025 (.)
	{0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00},	// Char 026 (.)
	{0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00},	// Char 027 (.)
	{0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00},	// Char 028 (.)
	{0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00},	// Char 029 (.)
	{0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00},	// Char 030 (.)
	{0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00},	// Char 031 (.)
	{0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00},	// Char 032 ( )
	{0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00},	// Char 033 (!)
	{0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00},	// Char 034 (")
	{0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00},	// Char 035 (#)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00},	// Char 036 ($)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00},	// Char 037 (%)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00},	// Char 038 (&)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00},	// Char 039 (')
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00},	// Char 040 (()
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00},	// Char 041 ())
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0},	// Char 042 (*)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60},	// Char 043 (+)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30},	// Char 044 (,)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18},	// Char 045 (-)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},	// Char 046 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06},	// Char 047 (/)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03},	// Char 048 (0)
	{0xFF, 0xC3, 0xA5, 0x99, 0x99, 0xA5, 0xC3, 0xFF},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t cc2500_get_status(SPI_HandleTypeDef,uint16_t, GPIO_TypeDef*);
void cc2500_set_channel(SPI_HandleTypeDef, uint8_t, uint16_t, GPIO_TypeDef*);
uint8_t cc2500_get_channel(SPI_HandleTypeDef, uint16_t, GPIO_TypeDef*);
uint8_t cc2500_get_rssi(SPI_HandleTypeDef, uint16_t, GPIO_TypeDef* );
void cc2500_flush_rx_fifo(SPI_HandleTypeDef, uint16_t, GPIO_TypeDef*);
void cc2500_enter_rx_mode(SPI_HandleTypeDef, uint16_t, GPIO_TypeDef*);
void cc2500_flush_tx_fifo(SPI_HandleTypeDef, uint16_t, GPIO_TypeDef*);
void cc2500_enter_tx_mode(SPI_HandleTypeDef, uint16_t, GPIO_TypeDef*);
void cc2500_transmit_packet(SPI_HandleTypeDef, uint8_t bytes[],uint16_t, GPIO_TypeDef* );
void cc2500_setup_r(SPI_HandleTypeDef , uint16_t, GPIO_TypeDef*, uint8_t packet_size, void (*packet_handler)(uint8_t byte_count, uint8_t bytes[]));
/* USER CODE END PFP */


void sendPacket(uint8_t reg, uint8_t data)
{
	// CS
	HAL_GPIO_WritePin(GPIOA, SPI1_CS, GPIO_PIN_RESET);

	//uint16_t packet = (reg << 8) | data;
	uint8_t packet[2];
	packet[0] = reg;
	packet[1] = data;

	HAL_SPI_Transmit(&hspi1, (uint8_t*)&packet, 2, 100);

	// CS
	HAL_GPIO_WritePin(GPIOA, SPI1_CS, GPIO_PIN_SET);
}

void matrix_init()
{
	
    // Initialise MAX7219 with 8x8 led matrix
    sendPacket(MAX_NOOP, 0x00);           // NO OP (seems needed after power on)
    sendPacket(MAX_SCANLIMIT, 0x07);      // Enable all digits (always needed for current/8 per row)
    sendPacket(MAX_INTENSITY, 0x07);      // Display intensity (0x00 to 0x0F)
    sendPacket(MAX_DECODEMODE, 0);        // No BCD decoding for led matrix

    // Clear all rows/digits
    sendPacket(MAX_DIGIT0, 0);
    sendPacket(MAX_DIGIT1, 0);
    sendPacket(MAX_DIGIT2, 0);
    sendPacket(MAX_DIGIT3, 0);
    sendPacket(MAX_DIGIT4, 0);
    sendPacket(MAX_DIGIT5, 0);
    sendPacket(MAX_DIGIT6, 0);
    sendPacket(MAX_DIGIT7, 0);
    sendPacket(MAX_SHUTDOWN, 1); // Wake oscillators/display up
}

void DisplayDigit(uint8_t ucDigit){

    uint8_t ucRow;
    for(ucRow=0;ucRow<8;ucRow++)
        sendPacket(MAX_DIGIT0+ucRow, ucNumber[ucDigit][ucRow]);

}
void process_new_packet(uint8_t byte_count, uint8_t bytes[]) {

	int16_t data = (bytes[1] << 8) | bytes[0];

}

static uint8_t cc2500_write_register(SPI_HandleTypeDef hspi, uint8_t reg, uint8_t val, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	uint8_t status = 0;
	
	HAL_GPIO_WritePin(GPIObus, cs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status1 = HAL_SPI_Transmit(&hspi, (uint8_t*)&reg, 1, 100);
	HAL_StatusTypeDef status2 = HAL_SPI_TransmitReceive(&hspi, (uint8_t*)&val, &status, 1, 100);
	HAL_GPIO_WritePin(GPIObus, cs_pin, GPIO_PIN_SET);
	
	return status;
}

static uint8_t cc2500_send_strobe(SPI_HandleTypeDef hspi,uint8_t reg, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	uint8_t status = 0;
	
	HAL_GPIO_WritePin(GPIObus, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi, (uint8_t*)&reg, &status, 1, 100);
	HAL_GPIO_WritePin(GPIObus, cs_pin, GPIO_PIN_SET);
	
	return status;
}

uint8_t cc2500_get_status(SPI_HandleTypeDef hspi,uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	return cc2500_send_strobe(hspi,SNOP,cs_pin, GPIObus);

}

void cc2500_set_channel(SPI_HandleTypeDef hspi, uint8_t channel, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	cc2500_write_register(hspi, CHANNR, channel, cs_pin, GPIObus);

}

uint8_t cc2500_get_channel(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	return cc2500_write_register(hspi, CHANNR | READ_BYTE, 0x00, cs_pin, GPIObus);

}

uint8_t cc2500_get_rssi(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	int8_t rawRssi = cc2500_write_register(hspi, RSSI, 0x00, cs_pin, GPIObus);
	uint8_t normalized = rawRssi + 108; // (-128 to 127) -> (0 to 255) -> -20 to remove some offset
	return normalized;

}

void cc2500_flush_rx_fifo(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	cc2500_send_strobe(hspi, SFRX, cs_pin, GPIObus);
}

void cc2500_enter_rx_mode(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	cc2500_send_strobe(hspi, SRX, cs_pin, GPIObus);

}


void cc2500_flush_tx_fifo(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	cc2500_send_strobe(hspi,SFTX, cs_pin, GPIObus);

}

void cc2500_enter_tx_mode(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	cc2500_send_strobe(hspi,STX, cs_pin, GPIObus);

}

void cc2500_transmit_packet(SPI_HandleTypeDef hspi,uint8_t bytes[],uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	// wait for empty TX FIFO
	while(cc2500_write_register(hspi,TXBYTES | READ_BYTE, 0x00, cs_pin, GPIObus) != 0);

	// send the address
	cc2500_write_register(hspi,FIFO, 0x69, cs_pin, GPIObus);

	// send the payload
	for(uint8_t i = 0; i < packet_length; i++)
		cc2500_write_register(hspi,FIFO, bytes[i],cs_pin, GPIObus);

	// check for RX FIFO underflow
	uint8_t status = cc2500_get_status(hspi,cs_pin,GPIObus);
	if((status >> 4) == 7) {
		cc2500_flush_tx_fifo(hspi,cs_pin,GPIObus);
		cc2500_enter_tx_mode(hspi,cs_pin,GPIObus);
	}
}

static void receiver_handler(SPI_HandleTypeDef hspi,uint16_t cs_pin, GPIO_TypeDef* GPIObus ) {

	uint8_t byte_count = cc2500_write_register(hspi,RXBYTES | READ_BYTE, 0x00, cs_pin, GPIObus);

	// check for RX FIFO overflow
	if(byte_count >> 7) {
		cc2500_flush_rx_fifo(hspi,cs_pin, GPIObus);
		cc2500_enter_rx_mode(hspi,cs_pin, GPIObus);
		return;
	}

	// receive the bytes
	for(uint8_t i = 0; i < byte_count; i++)
		rx_buffer[i] = cc2500_write_register(hspi, FIFO | READ_BYTE, 0x00, cs_pin, GPIObus);

	// re-enter RX mode
	cc2500_enter_rx_mode(hspi, cs_pin, GPIObus);

	// call the user's packet handler function, skipping past byte 0 (address byte)
	handler(packet_length, &rx_buffer[1]);
	
	pending_int = 0;

}

void cc2500_setup_r(SPI_HandleTypeDef hspi, uint16_t cs_pin, GPIO_TypeDef* GPIObus , uint8_t packet_size, void (*packet_handler)(uint8_t byte_count, uint8_t bytes[])) {

	packet_length = packet_size;
	handler = packet_handler;


	HAL_GPIO_WritePin(GPIObus, cs_pin, GPIO_PIN_SET);

	// ensure cc2500 registers contain their reset values
	cc2500_send_strobe(hspi, SRES, cs_pin, GPIObus);

	// waste time for reset to complete
	for(volatile uint32_t i = 0; i < 99999; i++);

	// write to the registers that need different values
	cc2500_write_register(hspi, IOCFG0,	0x01, cs_pin, GPIObus);	// GDO0 as interrupt: asserts on sync, deasserts on end of packet
	cc2500_write_register(hspi, SYNC1,	0xBE, cs_pin, GPIObus);	// Sync word: 0xBEEF
	cc2500_write_register(hspi, SYNC0,	0xEF, cs_pin, GPIObus);	// Sync word: 0xBEEF
	cc2500_write_register(hspi, PKTLEN,	packet_size+1, cs_pin, GPIObus); // Packet length +1 for the address at byte 0
	cc2500_write_register(hspi, PKTCTRL1,	0x0D, cs_pin, GPIObus);	// Packet control: CRC autoflush, append status bytes, strict address check
	cc2500_write_register(hspi, PKTCTRL0,	0x44, cs_pin, GPIObus);	// Packet control: data whitening, CRC enabled, fixed packet length mode
	cc2500_write_register(hspi, ADDR,		0x69, cs_pin, GPIObus);	// Address: 0x69
	cc2500_write_register(hspi, CHANNR,	0x7F, cs_pin, GPIObus);	// Channel: 127
	cc2500_write_register(hspi, FSCTRL1,	0x0C, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FREQ2,	0x5C, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FREQ1,	0xF6, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FREQ0,	0x27, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, MDMCFG4,	0x0E, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, MDMCFG3,	0x3B, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, MDMCFG2,	0x73, cs_pin, GPIObus);	// Modem configuration: MSK, 30/32 sync bits detected
	cc2500_write_register(hspi, MDMCFG1,	0xC2, cs_pin, GPIObus);	// Modem configuration: FEC, 8 byte minimum preamble
	cc2500_write_register(hspi, DEVIATN,	0x00, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, MCSM1,	0x0E, cs_pin, GPIObus);	// Radio state machine: stay in TX mode after sending a packet, stay in RX mode after receiving
	cc2500_write_register(hspi, MCSM0,	0x18, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FOCCFG,	0x1D, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, BSCFG,	0x1C, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, AGCCTRL2,	0xC7, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, AGCCTRL0,	0xB0, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FREND1,	0xB6, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FSCAL3,	0xEA, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FSCAL1,	0x00, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, FSCAL0,	0x19, cs_pin, GPIObus);	//
	cc2500_write_register(hspi, PATABLE,	0xFF, cs_pin, GPIObus);	// Output power: +1dBm (the maximum possible)
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
	HAL_GPIO_WritePin(GPIOA, SPI1_CS, GPIO_PIN_RESET);
	matrix_init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	cc2500_setup_r(hspi2, SPI2_CS, RF_RECEIVE_BUS, 1,&process_new_packet);
	
	cc2500_enter_rx_mode(hspi2, SPI2_CS, RF_RECEIVE_BUS);
	startup=0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	if(pending_int == 1){
		receiver_handler(hspi2, SPI2_CS, RF_RECEIVE_BUS);
		DisplayDigit(rx_buffer[1]);
	}
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : exti0_receiver_Pin */
  GPIO_InitStruct.Pin = exti0_receiver_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(exti0_receiver_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin LD2_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
