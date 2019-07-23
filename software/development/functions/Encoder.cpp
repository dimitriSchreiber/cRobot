#include "Encoder.hpp"

Encoder::Encoder(){
	m_encoderAddr = 0;

}

bool Encoder::init(void **virtual_base, unsigned long PIO_BASE){
	m_encoderAddr = &virtual_base + ( (unsigned long)(ALT_LWFPGASLVS_OFST + &PIO_BASE) & ( unsigned long)( HW_REGS_MASK ) );
	return true;
}