/*
 * radio.c
 *
 *  Created on: 24-Jan-2009
 *      Author: Neil MacMillan
 *  Ported to Linux by Doug and Will Szumski 11-01-12
 */
#include "radio.h"
#include <stdlib.h>

// non-public constants and macros

#define CHANNEL 112
#define ADDRESS_LENGTH 5

#define _BV(bit) (1 << (bit)) 


// Flag which denotes that the radio is currently transmitting
static volatile uint8_t transmit_lock;
// tracks the payload widths of the Rx pipes
static volatile uint8_t rx_pipe_widths[6] = {32, 32, 0, 0, 0, 0};
// holds the transmit address (Rx pipe 0 is set to this address when transmitting with auto-ack enabled).
static volatile uint8_t tx_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };
// holds the receiver address for Rx pipe 0 (the address is overwritten when transmitting with auto-ack enabled).
static volatile uint8_t rx_pipe0_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };
// the driver keeps track of the success status for the last 16 transmissions
static volatile uint16_t tx_history = 0xFF;

static volatile RADIO_TX_STATUS tx_last_status = RADIO_TX_SUCCESS;

extern void radio_rxhandler(uint8_t pipenumber);


static void free_tx_rx(uint8_t *tx, uint8_t *rx);
static void allocate_tx_rx(uint8_t *tx, uint8_t *rx, uint8_t len);

/**
 * Retrieve the status register.
 */
static uint8_t get_status()
{
	uint8_t status = 0;
	status = SPI_Write_Byte(NOP);
	//printf("nrf24l01 status: %d\n",status);
	return status;
}
/**
 * Set a register in the radio
 * \param reg The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, &c.).
 * \param value The value to write to the given register (the whole register is overwritten).
 * \return The status register.
 */
static uint8_t set_register(radio_register_t reg, uint8_t* value, uint8_t len)
{
	//uint8_t *tx;
    //uint8_t *rx;
	uint8_t rtn;

	//allocate_tx_rx(tx,rx,(len+1));

	//FIX:
	//uint8_t tx[len+1] = {0, };
	//uint8_t rx[len+1] = {0, };
	uint8_t tx[60] = {0, };
	uint8_t rx[60] = {0, };
	uint8_t i;
	tx[0] = (W_REGISTER | (REGISTER_MASK & reg));
	for (i = 1; i < len+1; i++) {
		tx[i] = value[i-1];
	}
	SPI_ReadWrite_Block(tx, rx, len+1);
	//printf("nrf24l01 status: %d\n",rx[0]);
	
	rtn = rx[0];
	//free_tx_rx(tx,rx);

	return rtn;

}

/**
 * Retrieve a register value from the radio.
 * \param reg The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, &c.).
 * \param buffer A contiguous memory block into which the register contents will be copied.  If the buffer is too long for the
 * 		register contents, then the remaining bytes will be overwritten with 0xFF.
 * \param len The length of the buffer.
 */
static uint8_t get_register(radio_register_t reg, uint8_t* buffer, uint8_t len)
{
	
	//uint8_t *tx;
   // uint8_t *rx;
	uint8_t rtn;

	//allocate_tx_rx(tx,rx,(len+1));

	//uint8_t tx[len+1] = {0, };
	uint8_t tx[60] = {0, };
	uint8_t rx[60] = {0, };
	uint8_t i;
	tx[0] = (R_REGISTER | (REGISTER_MASK & reg));
	for (i = 1; i < len+1; i++) {
		tx[i] = 0xFF;
	}
	SPI_ReadWrite_Block(tx, tx, len+1);
	
	for (i = 1; i < len+1; i++) {
		buffer[i-1] = tx[i];
	}

	rtn = tx[0];

	//free_tx_rx(tx,rx);

	return rtn;
	
}


static void allocate_tx_rx(uint8_t *tx, uint8_t *rx, uint8_t len) {

	int i;

	tx = (uint8_t *) malloc( (len ) * sizeof (uint8_t));

	if (tx == NULL) {
			printf("unable to allocate memory in function send_instruction\n");
			printf("exiting\n");		
			exit(-1);
	}

    rx = (uint8_t *) malloc( (len ) * sizeof (uint8_t));

	if (rx == NULL) {
			printf("unable to allocate memory in function send_instruction\n");
			printf("exiting\n");		
			exit(-1);
	}

	for (i = 0 ; i< len ; i++) {
		tx[i] = NOP;
		rx[i] = NOP;
	}

	printf("done a malloc\n");
}

static void free_tx_rx(uint8_t *tx, uint8_t *rx) {
	printf("in free_tx_rx\n");	
	free(tx);
	tx = NULL;
	free(rx);
	rx= NULL;
	printf("freed the memory\n");
}



/**
 * Send an instruction to the nRF24L01.
 * \param instruction The instruction to send (see the bottom of nRF24L01.h)
 * \param data An array of argument data to the instruction.  If len is 0, then this may be NULL.
 * \param buffer An array for the instruction's return data.  This can be NULL if the instruction has no output.
 * \param len The length of the data and buffer arrays.
 */
static void send_instruction(uint8_t instruction, uint8_t* data, uint8_t* buffer, uint8_t len)
{
	//uint8_t *tx;
    //uint8_t *rx;
	uint8_t tx[60] = {0, };
	uint8_t rx[60] = {0, };

	//allocate_tx_rx(tx,rx,(len+1));
	
	//uint8_t tx[len+1] = {0, };
	//uint8_t rx[len+1] = {0, };
	uint8_t i;
	tx[0] = instruction;
	for (i = 1; i < len+1; i++) {
		tx[i] = data[i-1];
	}


	SPI_ReadWrite_Block(tx, rx, len+1);

	if (len > 0) 
	{
		if (buffer == NULL)	;
			//return;
		else {
			for (i = 1; i < len+1; i++) {
				buffer[i-1] = rx[i];
			}
		}

	}

}

/**
 * Switch the radio to receive mode.  If the radio is already in receive mode, this does nothing.
 */
static void set_rx_mode()
{
	uint8_t config;
	struct timespec ts;
	get_register(CONFIG, &config, 1);
	if ((config & _BV(PRIM_RX)) == 0)
	{
		config |= _BV(PRIM_RX);
		set_register(CONFIG, &config, 1);
		// the radio takes 130 us to power up the receiver.
		ts.tv_sec = 0;
        ts.tv_nsec = 130000000;
        nanosleep (&ts, NULL);

	}
}

/**
 * Switch the radio to transmit mode.  If the radio is already in transmit mode, this does nothing.
 */
static void set_tx_mode()
{
	uint8_t config;
	struct timespec ts;
	get_register(CONFIG, &config, 1);
	if ((config & _BV(PRIM_RX)) != 0)
	{
		config &= ~_BV(PRIM_RX);
		set_register(CONFIG, &config, 1);
		// The radio takes 130 us to power up the transmitter
		// You can delete this if you're sending large packets (I'm thinking > 25 bytes, but I'm not sure) because it
		// sending the bytes over SPI can take this long.
		ts.tv_sec = 0;
        ts.tv_nsec = 130000000;
        nanosleep (&ts, NULL);
	}
}

/**
 * Reset the pipe 0 address if pipe 0 is enabled.  This is necessary when the radio is using Enhanced Shockburst, because
 * the pipe 0 address is set to the transmit address while the radio is transmitting (this is how the radio receives
 * auto-ack packets).
 */
static void reset_pipe0_address()
{
	if (rx_pipe_widths[RADIO_PIPE_0] != 0)
	{
		// reset the pipe 0 address if pipe 0 is enabled.
		set_register(RX_ADDR_P0, (uint8_t*)rx_pipe0_address, ADDRESS_LENGTH);
	}
}

/**
 * Configure radio defaults and turn on the radio in receive mode.
 * This configures the radio to its max-power, max-packet-header-length settings.  If you want to reduce power consumption
 * or increase on-air payload bandwidth, you'll have to change the config.
 */
static void configure_registers()
{
	uint8_t value;

	//SPI_Init(device);
	// set address width to 5 bytes.
	value = ADDRESS_LENGTH - 2;			// 0b11 for 5 bytes, 0b10 for 4 bytes, 0b01 for 3 bytes
	set_register(SETUP_AW, &value, 1);
	// set Enhanced Shockburst retry to every 586 us, up to 5 times.  If packet collisions are a problem even with AA enabled,
	// then consider changing the retry delay to be different on the different stations so that they do not keep colliding on each retry.
	value = 0x15;
	//value = 0x10;
	set_register(SETUP_RETR, &value, 1);

	// Set to use 2.4 GHz channel 110.
	value = CHANNEL;
	set_register(RF_CH, &value, 1);

	// Set radio to 2 Mbps and high power.  Leave LNA_HCURR at its default.
	value = _BV(RF_DR) | _BV(LNA_HCURR);
	set_register(RF_SETUP, &value, 1);

	// Enable 2-byte CRC and power up in receive mode.
	value = _BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP) | _BV(PRIM_RX);
	set_register(CONFIG, &value, 1);

	// clear the interrupt flags in case the radio's still asserting an old unhandled interrupt
    value = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);
    set_register(STATUS, &value, 1);

    // flush the FIFOs in case there are old data in them.
	send_instruction(FLUSH_TX, NULL, NULL, 0);
	send_instruction(FLUSH_RX, NULL, NULL, 0);
}

void Radio_Init()
{
	transmit_lock = 0;
	struct timespec ts;
	// disable radio during config
	CE_LOW();

	// A 10.3 ms delay is required between power off and power on states (controlled by 3.3 V supply).
	ts.tv_sec = 0;
    ts.tv_nsec = 11000000;
    nanosleep (&ts, NULL);

	// Configure the radio registers that are not application-dependent.
	configure_registers();

	// A 1.5 ms delay is required between power down and power up states (controlled by PWR_UP bit in CONFIG)
	ts.tv_sec = 0;
    ts.tv_nsec = 2000000;
    nanosleep (&ts, NULL);

	// enable radio as a receiver
	CE_HIGH();
}

// default address for pipe 0 is 0xe7e7e7e7e7
// default address for pipe 1 is 0xc2c2c2c2c2
// default address for pipe 2 is 0xc2c2c2c2c3 (disabled)
// default address for pipe 3 is 0xc2c2c2c2c4 (disabled)
// default address for pipe 4 is 0xc2c2c2c2c5 (disabled)
// default address for pipe 5 is 0xc2c2c2c2c6 (disabled)

void Radio_Configure_Rx(RADIO_PIPE pipe, uint8_t* address, ON_OFF enable)
{
	uint8_t value;
	uint8_t use_aa = 1;
	uint8_t payload_width = 32;
	//if (payload_width < 1 || payload_width > 32 || pipe < RADIO_PIPE_0 || pipe > RADIO_PIPE_5) return;

	// store the pipe 0 address so that it can be overwritten when transmitting with auto-ack enabled.
	if (pipe == RADIO_PIPE_0)
	{
		rx_pipe0_address[0] = address[0];
		rx_pipe0_address[1] = address[1];
		rx_pipe0_address[2] = address[2];
		rx_pipe0_address[3] = address[3];
		rx_pipe0_address[4] = address[4];
	}

	// Set the address.  We set this stuff even if the pipe is being disabled, because for example the transmitter
	// needs pipe 0 to have the same address as the Tx address for auto-ack to work, even if pipe 0 is disabled.
	set_register(RX_ADDR_P0 + pipe, address, pipe > RADIO_PIPE_1 ? 1 : ADDRESS_LENGTH);

	// Set auto-ack.
	get_register(EN_AA, &value, 1);
	if (use_aa)
		value |= _BV(pipe);
	else
		value &= ~_BV(pipe);
	set_register(EN_AA, &value, 1);

	// Set the pipe's payload width.  If the pipe is being disabled, then the payload width is set to 0.
	value = enable ? payload_width : 0;
	set_register(RX_PW_P0 + pipe, &value, 1);
	rx_pipe_widths[pipe] = value;

	// Enable or disable the pipe.
	get_register(EN_RXADDR, &value, 1);
	if (enable)
		value |= _BV(pipe);
	else
		value &= ~_BV(pipe);
	set_register(EN_RXADDR, &value, 1);

}

// default transmitter address is 0xe7e7e7e7e7.
void Radio_Set_Tx_Addr(uint8_t* address)
{
	tx_address[0] = address[0];
	tx_address[1] = address[1];
	tx_address[2] = address[2];
	tx_address[3] = address[3];
	tx_address[4] = address[4];
	set_register(TX_ADDR, address, ADDRESS_LENGTH);
}

void Radio_Configure(RADIO_DATA_RATE dr, RADIO_TX_POWER power)
{
	uint8_t value;

	//if (power < RADIO_LOWEST_POWER || power > RADIO_HIGHEST_POWER || dr < RADIO_1MBPS || dr > RADIO_2MBPS) return;

	// set the address
	//Radio_Set_Tx_Addr(address);

	// set the data rate and power bits in the RF_SETUP register
	get_register(RF_SETUP, &value, 1);

	value |= 3 << RF_PWR;			// set the power bits so that the & will mask the power value in properly.
	value &= power << RF_PWR;		// mask the power value into the RF status byte.

	if (dr)
		value |= _BV(RF_DR);
	else
		value &= ~_BV(RF_DR);

	set_register(RF_SETUP, &value, 1);
}

uint8_t Radio_Transmit(radiopacket_t* payload, RADIO_TX_WAIT wait)
{
	//pthread_mutex_t * mutex =  isr_get_mutex();
	//pthread_mutex_lock( mutex );	
	//if (block && transmit_lock) while (transmit_lock);
	//if (!block && transmit_lock) return 0;
	uint8_t len = 32;

	// indicate that the driver is transmitting.
    transmit_lock = 1;

	// disable the radio while writing to the Tx FIFO.
    CE_LOW();

	set_tx_mode();

    // for auto-ack to work, the pipe0 address must be set to the Tx address while the radio is transmitting.
    // The register will be set back to the original pipe 0 address when the TX_DS or MAX_RT interrupt is asserted.
    set_register(RX_ADDR_P0, (uint8_t*)tx_address, ADDRESS_LENGTH);

    // transfer the packet to the radio's Tx FIFO for transmission
	// TODO: Does typecasting fix warning? 
	// TODO: send_instruction(W_TX_PAYLOAD, (uint8_t*)payload, NULL, len);
	
    send_instruction(W_TX_PAYLOAD,  (uint8_t*)payload, NULL, len);
	

    // start the transmission.
    CE_HIGH();

    if (wait == RADIO_WAIT_FOR_TX)
    {
    	while (transmit_lock);
    	return tx_last_status;
    }
	//pthread_mutex_unlock( mutex );
    return RADIO_TX_SUCCESS;
}

RADIO_RX_STATUS Radio_Receive(radiopacket_t* buffer)
{
	uint8_t len = 32;
	uint8_t status;
	uint8_t pipe_number;
	uint8_t doMove = 1;
	RADIO_RX_STATUS result;

	transmit_lock = 0;

	CE_LOW();

    status = get_status();
	pipe_number =  (status & 0xE) >> 1;

	if (pipe_number == RADIO_PIPE_EMPTY)
	{
		result = RADIO_RX_FIFO_EMPTY;
		doMove = 0;
	}

	if (rx_pipe_widths[pipe_number] > len)
	{
		// the buffer isn't big enough, so don't copy the data.
		result = RADIO_RX_INVALID_ARGS;
		doMove = 0;
	}

	if (doMove)
	{
		// Move the data payload into the local
		send_instruction(R_RX_PAYLOAD, (uint8_t*)buffer, (uint8_t*)buffer, rx_pipe_widths[pipe_number]);
		int i;
		status = get_status();
		pipe_number =  (status & 0xE) >> 1;

		if (pipe_number != RADIO_PIPE_EMPTY)
			result = RADIO_RX_MORE_PACKETS;
		else
			result = RADIO_RX_SUCCESS;
	}

	CE_HIGH();

	transmit_lock = 0;

	//release_radio();

	return result;
}

// This is only accurate if all the failed packets were sent using auto-ack.
uint8_t Radio_Success_Rate()
{
	uint16_t wh = tx_history;
	uint8_t weight = 0;
	while (wh != 0)
	{
		if ((wh & 1) != 0) weight++;
		wh >>= 1;
	}
	wh = (16 - weight) * 100;
	wh /= 16;
	return wh;
}

void Radio_Flush()
{
	send_instruction(FLUSH_TX, NULL, NULL, 0);
	send_instruction(FLUSH_RX, NULL, NULL, 0);
}


void interrupt_handler(unsigned interrupts) {

	//printf("entering interupt-handler: %d\n", interrupts);
	
    uint8_t status;
    uint8_t pipe_number;


    CE_LOW();

    status = get_status();

    if (status & _BV(RX_DR))

    {
    	pipe_number =  (status & 0xE) >> 1;
    	radio_rxhandler(pipe_number);
    }
    // We can get the TX_DS or the MAX_RT interrupt, but not both.

    if (status & _BV(TX_DS))
    {
        // if there's nothing left to transmit, switch back to receive mode.
        transmit_lock = 0;

        reset_pipe0_address();
        set_rx_mode();

    	// indicate in the history that a packet was transmitted successfully by appending a 1.
    	tx_history <<= 1;

    	tx_history |= 1;

    	tx_last_status = RADIO_TX_SUCCESS;
    }
    else if (status & _BV(MAX_RT))

    {
        send_instruction(FLUSH_TX, NULL, NULL, 0);

    	transmit_lock = 0;

    	reset_pipe0_address();
		set_rx_mode();
    	// indicate in the history that a packet was dropped by appending a 0.
    	tx_history <<= 1;


    	tx_last_status = RADIO_TX_MAX_RT;
    }

    // clear the interrupt flags.
	status = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);

	set_register(STATUS, &status, 1);

    CE_HIGH();


	//printf("exiting interupt-handler\n");
}
