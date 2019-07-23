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

#define P_float 0.0005
#define I_float 0.005
#define D_float 0.0000006
//double ARM_P[8] = {0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.0002};
double ARM_P[8] = {P_float, P_float, P_float, P_float, P_float, P_float, P_float, P_float};
double ARM_I[8] = {I_float, I_float, I_float, I_float, I_float, I_float, I_float, I_float};
double ARM_D[8] = {D_float, D_float, D_float, D_float, D_float, D_float, D_float, D_float};



int main(int argc, char **argv)
{
	uint64_t loopStartTime = 0;
	uint64_t loopEndTime = 0;

	rc_filter_t ARM_PID0 = rc_filter_empty();

	//pthread_t pth, pth_heartbeat;	// this is our thread identifier

/*------------------------------------------
Generic setup below for port communication
-----------------------------------------*/

    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }	
   	portnumber_global = atoi(argv[1]);
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


	rc_filter_pid(&ARM_PID0, ARM_P[0], ARM_I[0], ARM_D[0], 1*dt, dt);


/*------------------------------------------
Run controller
------------------------------------------*/

	for(i=0;i<8;i++){
			position_setpoints[i] = 0;
	}

	int adc_data;

	int dir_bitmask;
	int en_bitmask;
	int currentPosition = 0;
	long myCounter = 0;
	int32_t e_stop = 0;
	int max_val = 0;
	int min_val = 0;
	dir_bitmask = 0x00;
	double duty = 0.0;
	int32_t enc_val[8] = {1};
	int q;
	double ez_p = 1;
	//pull disable low
	alt_write_word(h2p_lw_gpio0_addr, (1<<1));
	//set pwm 0
	for(k=0;k<8;k++){
		alt_write_word(h2p_lw_pwm_values_addr[k], 0);
	}
	//begin loop
	while(exit_flag == 0)
	{
		if(1 && myCounter%10 == 0 && myCounter > 100){

		*(h2p_lw_adc) = 0; //write starts adc read
		//ADC_Controller_for_DE_Series_Boards.pdf pg3 reference we might have old data spi to read different channels
		//reference Using_DE_Series_ADC.pdf page 12 for example

		for (i=0; i<7; i++){
			adc_data = *(h2p_lw_adc + i); //read, pointer addition increments address by parent datat type size, in this case i*32 bits
			float current = (adc_data)* 0.001; // 4096 for ADC, 4.096v reference, unit of Volts and 1A/1V for current sensor --> A units too
			current = current * (current > 0);
			avg_current_array[i] = 0.2 * current + 0.8 * avg_current_array[i];
		}

			alt_write_word(h2p_lw_quad_reset_addr, 0);
			//int32_t setpoint = 2000*10*0.1*sin(rc_nanos_since_epoch()/pow(10,9)*1*3.14);
			//int32_t setpoint = 0;			
//printf("%d\n", setpoint);
			internal_encoders[0] = alt_read_word(h2p_lw_quad_addr[0]);
			internal_encoders[1] = alt_read_word(h2p_lw_quad_addr[1]);
			int32_t error =  internal_encoders[0] - position_setpoints[0];

			alt_write_word(h2p_lw_pid_input_addr[1], error);
			int32_t check_error = (int32_t)(*h2p_lw_quad_addr[0]);
			
			double pid_output_ARM;
			pid_output_ARM = rc_filter_march(&ARM_PID0, error) * 2048;

			int32_t pid_output = (int32_t)pid_output_ARM;
			int32_t positive_pid_output = (pid_output>=0);
			int32_t pid_output_cutoff = fabs(pid_output)*(fabs(pid_output) <= 2047) + 2047*(fabs(pid_output) > 2047);

			alt_write_word(h2p_lw_pwm_values_addr[0], (pid_output_cutoff));
					
			dir_bitmask = alt_read_word(h2p_lw_gpio1_addr);
			if(!positive_pid_output)
				dir_bitmask |= (1<<7);
			else
				dir_bitmask &= ~(1<<7);
			alt_write_word(h2p_lw_gpio1_addr, dir_bitmask);
			//printf("Plastic:%d\n Wood:%d\n", alt_read_word(h2p_lw_quad_addr[0]), alt_read_word(h2p_lw_quad_addr[1]));
		}
		if(0 && myCounter%10 == 0 && myCounter > 100){
			currentPosition = alt_read_word(h2p_lw_quad_addr[1]);
			printf("%d: %d\n", currentPosition, abs(currentPosition * ez_p));
			if (currentPosition < 0){
				alt_write_word(h2p_lw_gpio1_addr, 0);
			}else{
				alt_write_word(h2p_lw_gpio1_addr, 1<<7);
			}
			for(k=0;k<8;k++){
				alt_write_word(h2p_lw_pwm_values_addr[0], abs(currentPosition * ez_p));
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

