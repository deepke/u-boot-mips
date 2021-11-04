#include "ls1a500.h"
typedef struct pin_cfgs {
	int pin_num;
	int model;
} pin_cfgs_t;

pin_cfgs_t default_pin_cfgs[];
#define set_pin_mode(pin_num, mode)				\
do {								\
	volatile unsigned int *addr = (unsigned int *)		\
		(LS1A500_GPIO_MULTI_CFG + (pin_num) / 8 * 4);	\
	int bit = (pin_num) % 8 * 4;				\
	*addr &= ~(7 << bit);					\
	*addr |= ((mode) << bit);				\
} while(0)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define cfg_all_pin_multi(cfgs)					\
do {								\
	int i;							\
	for (i = 0; i < ARRAY_SIZE(cfgs); i++) {		\
		set_pin_mode(cfgs[i].pin_num, cfgs[i].model);	\
	}							\
} while(0)

