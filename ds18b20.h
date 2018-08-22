/*
 * ds18b20.h
 *
 * Created: 07.08.2017 10:59:07
 *  Author: Valera
 */ 


#ifndef DS18B20_H_
#define DS18B20_H_

#define F_CPU 16000000UL

#include <stdio.h>
#include <util/delay.h>

#define port_termometr PORTB
#define ddr_termometr DDRB
#define pin_termometr PINB
#define termometr 0
void input_1wire();
void output_1wire();
void init_termometr();
void write_byte_termometr(uint8_t);
void writebit1();
void writebit0();
uint8_t readbit();
void read_rom_termometr(uint8_t *mas);
uint8_t read_byte_termometr();
void convert_tempr ();
void write_scrpad(uint8_t th, uint8_t tl, uint8_t config); 
void read_scrpad(uint8_t *data); 
void skip_rom();

#endif /* DS18B20_H_ */