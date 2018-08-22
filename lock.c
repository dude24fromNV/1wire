

#define F_CPU   16000000UL
#define BUFFER_LEN (64)

#include "ds18b20.h"
#include "onewire.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
/* Power management. For more see: https://www.nongnu.org/avr-libc/user-manual/group__avr__power.html */
#include <avr/power.h>
/* Atomic operations */
#include <util/atomic.h>
#include <stdbool.h>
#include <avr/eeprom.h> 

static const uint32_t uart_baudrate = 19200;	/* Baud rate (Baud / second) */
/* Value to be placed in UBRR register. See datasheet for more */
static const uint16_t ubrr_val = 51;
/* Read and write buffers */
static uint8_t	rdbuff[BUFFER_LEN] = {'\0'},
		wrbuff[BUFFER_LEN] = {'\0'};
static uint8_t rdind = 0, wrind = 0;	/* Indices */

/* Indicates transfer & reception completion */
volatile bool txcflag = true;
volatile bool rxcflag = false;

/*==================uart===========================*/
static void uart_init(void)
{
	/* To use USART module, we need to power it on first */
	power_usart0_enable();

	/* Configure prescaler */
	UBRR0L = ubrr_val & 0xFF; /* Lower byte */
	UBRR0H = (ubrr_val >> 8) & 0xFF;   /* Higher byte */
	/* Or we could use UBRR0 (16-bit defined) instead */

	/* Set operation mode */
	/* Asynchronous mode, even parity, 2 stop bits, 8 data bits */
	UCSR0C = (1 << UPM01) | (1 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);

	/* Continue setup & enable USART in one operation */
	/* RX & TX complete, Data reg empty interrupts enabled */
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}


static void uart_put(uint8_t *str, uint8_t size)
{
	/* If buffer contents have not been transfered yet -- put MCU to sleep */
	while(!txcflag)
		sleep_cpu();
        if (size >= BUFFER_LEN) return;
	/* No interrupts can occur while this block is executed */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < size; i++) {
			wrbuff[i] = str[i];
		}
                wrbuff[size]='\n';
		wrind = 0;
		txcflag = false; /* programmatic transfer complete flag */
		/* Enable transmitter interrupts */
		UCSR0B |= (1 << TXCIE0) | (1 << UDRIE0);
	}
}


void segm_bcd(uint8_t number, uint8_t *res)
{
	do {
		*res++ = number % 10+0x30;
		number /= 10;
	} while (number);
}


uint8_t EEMEM serial_num [10][8];
uint8_t EEMEM index_sn_ee = 0;

int main()
{
	/* We use internal pullup resitor for 1-wire line */
	DDRB = (1 << 0) | (1 << 1);
	PORTB |= (1 << 0)| (1 << 1);
        PORTB &= ~(1<<2);

	_delay_ms(2000);

	uint8_t ibutton_id[8];
        uint8_t scrpad[9];
	uint8_t *crc = &ibutton_id[7];
        uint8_t str[]={"1Wire"};
        uint8_t tempr[1]={0x00, 0x00}; 
        uint8_t temp =0;
        uint8_t calccrc = 0;
        uint8_t index_sn = 0;//eeprom_read_byte(&index_sn_ee);    
 
        uart_init();
        set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
        
	uart_put(str, sizeof(str)/sizeof(str[0]));
       
        /* save serial number 1wire device if press button before star device*/ 
        if (!((PINB) & (1 << 2))) {
		_delay_us(500);
		if (!((PINB) & (1 << 2))){
                        if (index_sn < 10){
			        init_termometr();
                                read_rom_termometr(ibutton_id);
                                calccrc = ow_crc8_fast_arr(ibutton_id, 7);
                                if (*crc == calccrc){
                                        eeprom_write_block(ibutton_id, serial_num[1], 8);
                                        index_sn++;
                                        eeprom_update_byte(index_sn_ee, index_sn);
                                        
                                }
                        }
                }
        }
               
        sei();
	while (1) {
                init_termometr();
                read_rom_termometr(ibutton_id);
                uint8_t calccrc = ow_crc8_fast_arr(ibutton_id, 7);
                if (*crc == calccrc) {PORTB &= ~(1 << 1);
                        
                        init_termometr();
                        skip_rom();
                        convert_tempr ();
                        init_termometr();
                        skip_rom();
                        read_scrpad(scrpad);
                } else {
                        PORTB |=(1 << 1);
                }
                
                _delay_ms(2000);
                /*read temperature & output*/
                temp = scrpad[0]>>4 ;
                temp |= scrpad[1] <<4;
                segm_bcd(temp, tempr);
                uart_put(&tempr[1],1);
                uart_put(&tempr[0],1);
                /* test corect save serial number in eeprom*/
                _delay_ms(2000);
                uint8_t StringOfData[8];
                eeprom_read_block((void*)&StringOfData, serial_num[0], 8); 
                uart_put(StringOfData,8);
                _delay_ms(2000);
                StringOfData[8];
                eeprom_read_block((void*)&StringOfData, serial_num[1], 8); 
                uart_put(StringOfData,8);
	}
}



/* USART TX Complete interrupt handler */
ISR(USART_TX_vect, ISR_BLOCK)
{
	/* When data register is empty and after the shift register contents */
	/* are already transfered, this interrupt is raised */
	UCSR0B &= ~(1 << TXCIE0);
}


/* USART Data Register Empty interrupt handler */
ISR(USART_UDRE_vect, ISR_BLOCK)
{
	if (('\n' == wrbuff[wrind]) || txcflag) {
		/* If nothing to transmit - disable this interrupt and exit */
		UCSR0B &= ~(1 << UDRIE0);
		txcflag = true;
		return;
	}

	UDR0 = wrbuff[wrind++];	

	/* Really we don't need this as long as every string ends with '\0' */
	if (wrind >= BUFFER_LEN)
		wrind = 0;
}
