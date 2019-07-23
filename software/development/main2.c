#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/mman.h>
#include <netinet/in.h>
#include <inttypes.h>


#include <rc/math.h>
#include <rc/time.h>

#include "main2.h"

int E_STATE = 0;
int ERR_RESET = 1;
int CONNECTED = 0;  //global flag to indicate if a connection has been made
int32_t beat = 0;
uint8_t switch_states[8];
int32_t internal_encoders[8] = {0};
float avg_current_array[8]={0};
int32_t arm_encoders1=0,arm_encoders2=0,arm_encoders3=0,arm_encoders4=0;
int exit_flag = 0;
int socket_error = 0;
int system_state = 1;

uint64_t global_loop_start_time = 0;
uint64_t loopEndAfterSleep = 0;

int CURRENT_FLAG = 0;
int TRAVEL_FLAG = 0;
int ETSOP_FLAG = 0;

/*------------------------------------------
FPGA PID Gains
-----------------------------------------*/
uint8_t P=150;
uint8_t P2=150;
uint8_t I=0;
uint8_t D=0;
float controllerGain = 0.02;
float avg_current = 0;
//float dt = 1 / (float)SAMPLE_RATE;
float dt = 0.001;

/*------------------------------------------
C side PID filter setup
-----------------------------------------*/

#define P_float 0.000003
#define I_float 0.000000
#define D_float 0.00000001
//double ARM_P[8] = {0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.0002};
double ARM_P[8] = {P_float, P_float, P_float, P_float, P_float, P_float, P_float, P_float};
double ARM_I[8] = {I_float, I_float, I_float, I_float, I_float, I_float, I_float, I_float};
double ARM_D[8] = {D_float, D_float, D_float, D_float, D_float, D_float, D_float, D_float};



int main(int argc, char **argv)
{
	uint64_t loopStartTime = 0;
	uint64_t loopEndTime = 0;

	//pthread_t pth, pth_heartbeat;	// this is our thread identifier

/*--------------------------------
ctrl-c catcher
--------------------------------*/
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
   	sigemptyset(&sigIntHandler.sa_mask);
  	sigIntHandler.sa_flags = 0;
  	sigaction(SIGINT, &sigIntHandler, NULL);

/*------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------
Setup FPGA communication
------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------*/
	void *virtual_base;
	int fd;
	int i,j,k;
	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
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
	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	// GPIO0 (1<<0 nsleep HIGH) (1<<1 disable HIGH)
	h2p_lw_gpio0_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_gpio1_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_heartbeat_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HEARTBEAT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_reset_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_RESET_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_values_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_VALUES_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_limit_switch_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LIMIT_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_e_stop = virtual_base + ( ( unsigned long )(ALT_LWFPGASLVS_OFST + E_STOP_BASE) & ( unsigned long)( HW_REGS_MASK ) );

	h2p_lw_pwm_values_addr[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[4] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[5] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[6] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[7] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	//Motor bank encoder counts
	h2p_lw_quad_addr[0]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[1]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[2]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[3]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[4]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[5]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[6]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[7]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	//External encoders for robot arm
	h2p_lw_quad_addr_external[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_8_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr_external[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_9_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr_external[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_10_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr_external[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_11_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	
	h2p_lw_pid_input_addr[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[4] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[5] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[6] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[7] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	
	h2p_lw_pid_output_addr[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[4] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[5] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[6] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[7] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	h2p_lw_adc = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	pthread_create(&pth_heartbeat,NULL,heartbeat_func,NULL);
	pthread_create(&pth,NULL,threadFunc,NULL);
/*------------------------------------------
Run controller
------------------------------------------*/
	int dir_bitmask;
	int en_bitmask;
	long myCounter = 0;
	int32_t e_stop = 0;
	int max_val = 0;
	int min_val = 0;
	dir_bitmask = 0x00;
	double duty = 0.0;
	int32_t enc_val[8] = {1};
	int q;
	//pull disable low
	alt_write_word(h2p_lw_gpio0_addr, (1<<1));
	//set pwm 0
	for(k=0;k<8;k++){
		alt_write_word(h2p_lw_pwm_values_addr[k], 0);
	}
	//begin loop
	while(exit_flag == 0)
	{
		//direction reverse
		if(1 && myCounter%1500 == 0 && myCounter > 500){
			alt_write_word(h2p_lw_gpio1_addr, 255);
			printf("Changing direction \n");
		}
		if(1 && myCounter%1000 == 0 && myCounter > 500){
			alt_write_word(h2p_lw_gpio1_addr, 0);
			printf("Changing direction \n");
		}
		if(1 && myCounter%100 == 0 && myCounter > 500){
			//read all encoders
			for(q = 0; q < 8; q++){
				printf("%d ", alt_read_word(h2p_lw_quad_addr[q]));
			}
			printf("\n");
		}
		if(0 && myCounter%100 == 0 && myCounter > 100){
			//pwm specific pin
			alt_write_word(h2p_lw_pwm_values_addr[1], (1<<9));
		}
		if(1 && myCounter%10 == 0 && myCounter > 100){
			//pwm whole register
			for(i = 0; i < 8; i++){
				alt_write_word(h2p_lw_pwm_values_addr[i], myCounter%4096);
			}
		}
		myCounter++;
		loopStartTime = rc_nanos_since_epoch();
		global_loop_start_time = loopStartTime;

		loopEndTime = rc_nanos_since_epoch();
		int uSleepTime = (dt*pow(10,6) - (int)(loopEndTime - loopStartTime)/1000);
		//rc_usleep(1000);
		if(uSleepTime > 0){
			rc_usleep(uSleepTime);
			//printf("We good");
		}
		else{
			rc_usleep(10);
			printf("Overran in main control loop!!! %" PRIu64 ", %" PRIu64 ", %" PRIu64 "\n", loopStartTime, loopEndTime, uSleepTime);
		}

		loopEndAfterSleep = rc_nanos_since_epoch();
	}
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	//Cleanup
	pthread_cancel(pth);
	printf("Exiting tcp thread\n");
	pthread_join(pth, NULL);
    pthread_join(pth_heartbeat, NULL);
	printf("\nExiting safely\n");
	sleep(1);
	close( fd );
	return 0;
}

