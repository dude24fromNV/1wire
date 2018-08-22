/*
 * ds18b20.c
 *
 * Created: 07.08.2017 10:57:59
 * Author : Valera
 */ 

#include <avr/io.h>
#include <util/atomic.h>
#include "ds18b20.h"
void output_1wire(){
	ddr_termometr |= 1<<termometr;
	port_termometr &= ~(1<<termometr);
}

void input_1wire(){
	ddr_termometr &= ~(1<<termometr);
	port_termometr &= ~(1<<termometr);
}

void writebit1(){
	cli();
	output_1wire();
	_delay_us(6);
	input_1wire();
	_delay_us(64);
	sei();
}

void writebit0(){
	cli();
	output_1wire();
	_delay_us(60);
	input_1wire();
	_delay_us(10);
	sei();
}

uint8_t readbit(){
	cli();
	uint8_t bit=0;
	output_1wire();
	_delay_us(3);
	input_1wire();
	_delay_us(12);
	bit = pin_termometr;
	_delay_us(55);
	bit >>=termometr;
	bit &= 0x01;
	sei();
	return bit;	
}

void init_termometr(){
	cli();
	uint8_t state =0;
	uint8_t not_device = 0;
	do 
	{
		output_1wire();
		_delay_us(480);
		input_1wire();
		_delay_us(70);
		state = pin_termometr;
		_delay_us(410);
		state>>=termometr;
		state &=0x01;
		if (state ==1)
		{
			not_device ++;
		} else{
			not_device = 10;
		}
	} while ( not_device !=10);
	sei();
}

void write_byte_termometr(uint8_t byte){
	cli();
	uint8_t bit;
	for (int i = 0; i< 8; ++i )
	{
		bit  = byte & 0x01;
		switch(bit){
			case 0x00:
				writebit0();
				break;
			case 0x01:
				writebit1();
				break;
		}
		byte >>=1;
	}
	sei();
}

uint8_t read_byte_termometr(){
	uint8_t byte=0,bit=0;
	cli();
	for(int i=0; i<8; ++i){
		byte >>=1;
		bit = readbit();
		
		if (bit == 1)
		{
			byte |= 0x80;
		} 
		else
		{
			byte  &= ~(0x80);
		}
	}
	sei();
	return byte;
}

void read_rom_termometr(uint8_t *mas){
        cli();
        write_byte_termometr(0x33);
        for( uint8_t i =0; i<8; i++){
                mas[i]= read_byte_termometr();
        }
        sei();
}

void convert_tempr (){
        write_byte_termometr(0x44);
        uint8_t bit = 0;
        
        do{     
                bit = readbit();
                _delay_ms(10);
        }while(!bit);
}

void write_scrpad(uint8_t th, uint8_t tl, uint8_t config){
        cli();
        write_byte_termometr(0x4e);
        write_byte_termometr(th);
        write_byte_termometr(tl);
        write_byte_termometr(config);
        sei();
}

void read_scrpad(uint8_t *data){
        cli();
        write_byte_termometr(0xBE);
        for( uint8_t i =0; i<9; i++){
                data[i]= read_byte_termometr();
        }
        sei();
}

void skip_rom(){
        write_byte_termometr(0xcc);
}
