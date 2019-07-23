#ifndef ENCODER_HPP
#define ENCODER_HPP

class Encoder{
	public:
		Encoder();
		~Encoder();
		bool init(void **virtual_base, unsigned long PIO_BASE);
		int32_t getEncoderAddr();
	private:
		int32_t m_encoderAddr;
};



#endif