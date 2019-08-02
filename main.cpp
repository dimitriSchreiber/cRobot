#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <ctime>
#include <string>
#include <cmath>
#include <csignal>
#include <sys/types.h>
#include <sys/mman.h>
#include <cinttypes>


extern "C"{
#include <rc/math/filter.h>
}


//#include "rc/time.h"


#include "main.h"
#include "motorController.h"
#include "fpgaAddressComm.h"
#include "fpgaEncoder.h"




void signalHandler(int sigNum){
    exit_flag = 1;
    std::cout << "setting exit_flag to 1"<< std:: endl;
}

int main(){
    signal(SIGINT, signalHandler);


    int fd;
    void* virtual_base;

    exit_flag = 0;

    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }
    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
    if( virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap() failed...\n" );
        close( fd );
        return(1);
    }

    h2p_lw_heartbeat_addr = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HEARTBEAT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[0] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[1] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[2] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[3] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[4] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[5] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[6] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[7] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_gpio1_addr=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    for(int k=0;k<8;k++){
        alt_write_word(h2p_lw_pwm_values_addr[k], 0);
    }

    h2p_lw_quad_addr[0]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[1]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[2]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[3]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[4]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[5]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[6]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[7]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    h2p_lw_quad_reset_addr = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_RESET_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    uint32_t reset_mask = 255 | 255<<20;

    alt_write_word(h2p_lw_quad_reset_addr, 0);


    auto heartBeatFunc = [](){
        CTRobot::fpgaHeartBeat heartBeat(h2p_lw_heartbeat_addr);
        std::cout<< "heartbeat on"<<std::endl;
        while (exit_flag == 0) {
                heartBeat.changeState();
                usleep(0.1 * 10000);//1.1 seconds
            }
        std::cout<< "heartbeat shutting down"<<std::endl;
    };

    std::thread heartBeatThread(heartBeatFunc);

    auto motorFunc = [](){
        CTRobot::motorController motor(h2p_lw_pwm_values_addr[0],h2p_lw_gpio1_addr,7);
        motor.attachEncoder(h2p_lw_quad_addr[0],h2p_lw_quad_reset_addr,7);
        motor.resetEncoder();

        int32_t setPosition = 8000;


        std::cout<<"motor on"<<std::endl;
        try {
            while (exit_flag == 0) {
                motor.setPIDValue();
                motor.runPID(setPosition);
                std::cout<< motor.readEncoder()<<std::endl;
                usleep(10000);
            }
        }
        catch(const char* errMsg){
            std::cout<< errMsg << std::endl;
        }
    };
    std::thread motor1(motorFunc);


    heartBeatThread.join();
    usleep(1000);//1.1 seconds
    motor1.join();


    return 0;
}
