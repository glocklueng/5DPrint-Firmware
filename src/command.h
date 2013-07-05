/* command.h - host commands */

/* TODO:  insert BSD license here. */

#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>


struct command {
	uint32_t seqnbr;
	uint8_t type;
	uint16_t code;
	uint16_t has_X : 1;
	uint16_t has_Y : 1;
	uint16_t has_Z : 1;
	uint16_t has_E : 1;
	uint16_t has_F : 1;
	uint16_t has_I : 1;
	uint16_t has_J : 1;
	uint16_t has_P : 1;
	uint16_t has_S : 1;
	uint16_t has_T : 1;
	uint16_t has_D : 1;
	float X;
	float Y;
	float Z;
	float E;
	float F;
	float I;
	float J;
	int32_t P;
	int32_t S;
	int32_t T;
	int32_t D;
};

#endif
