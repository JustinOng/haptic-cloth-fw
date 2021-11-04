#ifndef HELPERS_H
#define HELPERS_H

#define _bv(bit) (1 << (bit))
#define sbi(reg, bit) (reg |= _bv(bit))
#define cbi(reg, bit) (reg &= ~_bv(bit))

#endif
