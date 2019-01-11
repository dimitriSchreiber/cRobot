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

#include <rc/math.h>
#include <rc/time.h>

#include "main.h"

int E_STATE = 0;
int ERR_RESET = 1;
int CONNECTED = 0;  //global flag to indicate if a connection has been made
int32_t beat = 0;
uint8_t switch_states[8];
int32_t internal_encoders[8];
int32_t arm_encoders1=0,arm_encoders2=0,arm_encoders3=0,arm_encoders4=0;
int exit_flag = 0;
int socket_error = 0;
int system_state = 1;

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
float dt = 1 / (float)SAMPLE_RATE;

/*------------------------------------------
C side PID filter setup
-----------------------------------------*/
//double ARM_P[8] = {0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.0002};
double ARM_P[8] = {0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001};
double ARM_I[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double ARM_D[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



int main(int argc, char **argv)
{

	rc_filter_t ARM_PID0 = rc_filter_empty();
	rc_filter_t ARM_PID1 = rc_filter_empty();
	rc_filter_t ARM_PID2 = rc_filter_empty();
	rc_filter_t ARM_PID3 = rc_filter_empty();
	rc_filter_t ARM_PID4 = rc_filter_empty();
	rc_filter_t ARM_PID5 = rc_filter_empty();
	rc_filter_t ARM_PID6 = rc_filter_empty();
	rc_filter_t ARM_PID7 = rc_filter_empty();

	int i;

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
	int j;
	int k;
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
	h2p_lw_gpio_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
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

/*--------------------------------
initial setpoints to zero
--------------------------------*/
	for(i=0;i<8;i++){
			position_setpoints[i] = 0;
	}

/*--------------------------------
set PWM values to zero
--------------------------------*/
	for(k=0;k<8;k++){
		alt_write_word(h2p_lw_pwm_values_addr[j], 0);
	}
	
/*--------------------------------
calculate FPGA PID values
--------------------------------*/

	//calculate PID values
	uint32_t PID_values = 0;
	PID_values = PID_values | (P+I+D);
	//Question for this
	PID_values = PID_values | ((P+2*D) << 8);
	//PID_values = PID_values | (10 << 8);
	PID_values = PID_values | (D << 16);
	PID_values = PID_values | ((P2+I+D) << 24);
	alt_write_word(h2p_lw_pid_values_addr, PID_values);
	uint32_t pid_values_read = *(uint32_t*)h2p_lw_pid_values_addr;
	printf("\nSent: P: %d, I: %d, D: %d, P2: %d\n", P, I, D, P2);
	printf("Received: P: %d, I: %d, D: %d, P2: %d\n\n", (uint8_t)(pid_values_read & (0x000000FF)), (uint8_t)((pid_values_read & (0x0000FF00))>>8), (uint8_t)((pid_values_read & (0x00FF0000))>>16)), (uint8_t)((pid_values_read & (0xFF000000))>>24);
	

/*--------------------------------
calibrate current sense
--------------------------------*/
	int current_offset;
	printf("Calculating current offset please wait...\n");
	current_offset = calc_current_offset(h2p_lw_adc);
	printf("Current offset is: %d\n", current_offset);

/*--------------------------------
setup local PID controllers
--------------------------------*/
	rc_filter_pid(&ARM_PID0, ARM_P[0], ARM_I[0], ARM_D[0], dt, dt);
	rc_filter_pid(&ARM_PID1, ARM_P[1], ARM_I[1], ARM_D[1], dt, dt);
	rc_filter_pid(&ARM_PID2, ARM_P[2], ARM_I[2], ARM_D[2], dt, dt);
	rc_filter_pid(&ARM_PID3, ARM_P[3], ARM_I[3], ARM_D[3], dt, dt);
	rc_filter_pid(&ARM_PID4, ARM_P[4], ARM_I[4], ARM_D[4], dt, dt);
	rc_filter_pid(&ARM_PID5, ARM_P[5], ARM_I[5], ARM_D[5], dt, dt);
	rc_filter_pid(&ARM_PID6, ARM_P[6], ARM_I[6], ARM_D[6], dt, dt);
	rc_filter_pid(&ARM_PID7, ARM_P[7], ARM_I[7], ARM_D[7], dt, dt);


/*------------------------------------------
Run controller
------------------------------------------*/
	int dir_bitmask;
	long myCounter = 0;
	int32_t e_stop=0;
	struct timeval timer_usec;
	int max_val = 0;
	int min_val = 0;
	double tracking_error[8];

	int adc_data;
	float current;

	for(k = 0; k<8; k++){
		tracking_error[k] = 0;
	}

	while(exit_flag == 0)
	{
		//On motor bank limit switches
		uint32_t switches = alt_read_word(h2p_lw_limit_switch_addr);
		switch_states[0] = (switches&1<<0)==0;
		switch_states[1] = (switches&1<<3)==0;
		switch_states[2] = (switches&1<<2)==0;
		switch_states[3] = (switches&1<<1)==0;
		switch_states[4] = (switches&1<<7)>0;
		switch_states[5] = (switches&1<<6)>0;
		switch_states[6] = (switches&1<<5)==0;
		switch_states[7] = (switches&1<<4)==0;

		//ADC read
		*(h2p_lw_adc) = 0; //write starts adc read
		adc_data = *(h2p_lw_adc); //read
		current = (adc_data - current_offset)* 0.001;
		current = current * (current > 0);
		avg_current = 0.3 * current + 0.7 * avg_current;
		//avg_current = avg_current * (avg_current > 0);

		//Read encoder positions
		for(j = 0; j<8; j++){
			int32_t output = alt_read_word(h2p_lw_quad_addr[j]);
			internal_encoders[j] = output;
			
			//read external joint encoders
			if(j==7){
				arm_encoders1 = alt_read_word(h2p_lw_quad_addr_external[0]);
				arm_encoders2 = alt_read_word(h2p_lw_quad_addr_external[1]);
				arm_encoders3 = alt_read_word(h2p_lw_quad_addr_external[2]);
				arm_encoders4 = alt_read_word(h2p_lw_quad_addr_external[3]);
			}


			e_stop = alt_read_word(h2p_lw_pid_e_stop);
			//printf("e_stop value %d", e_stop);
			//E stop state checking
			if (abs(internal_encoders[j]) > MAX_TRAVEL_RANGE || abs(position_setpoints[j]) > MAX_TRAVEL_RANGE || ERR_RESET || e_stop || avg_current > MAX_CURRENT){
				E_STATE = 1;
				ERR_RESET = 1;
				//printf("e_stop value: %d\n", e_stop);
				//Reset pwm and pid loops, [0:7] and [20:27]
				int32_t reset_mask = 255 | 255<<20;
				alt_write_word(h2p_lw_quad_reset_addr, reset_mask);

				if(avg_current > MAX_CURRENT){
					CURRENT_FLAG = 1;
				}
				if(abs(internal_encoders[j]) > MAX_TRAVEL_RANGE){
					TRAVEL_FLAG = 1;
				}
				if(e_stop){
					ETSOP_FLAG = 1;
				}
			}
			//normal operation
			else{

				//IF I HAVE  TIME! ADD INTERPOLATION OR VELOCITY LIMITS HERE!!!! FOR NOW IT SEEMS TO BE PRETTY GOOD THOUGH.
				alt_write_word(h2p_lw_quad_reset_addr, 0);
				int32_t error = internal_encoders[j] - position_setpoints[j];
				tracking_error[j] = tracking_error[j]*0.99 + error*.01; //only used in printout

				alt_write_word(h2p_lw_pid_input_addr[j], error);
				int32_t check_error = (int32_t)(*h2p_lw_quad_addr[j]);// - position_setpoints[j];
				
				usleep(10);
				//int32_t pid_output = (int32_t)(alt_read_word(h2p_lw_pid_output_addr[j])) * controllerGain;
				//EDIT: trying to use jame's code
				double error_degrees = (double)error / 2000.0 * 360;
				double pid_output_ARM;

				switch(j){
					case 0:
						pid_output_ARM = rc_filter_march(&ARM_PID0, error) * 2048;
				        break;
						//printf("1");
					case 1:
						pid_output_ARM = rc_filter_march(&ARM_PID1, error) * 2048;
						break;
						//printf("2");
					case 2:
						pid_output_ARM = rc_filter_march(&ARM_PID2, error) * 2048;
						break;
						//printf("3");
					case 3:
						pid_output_ARM = rc_filter_march(&ARM_PID3, error) * 2048;
						break;
						//printf("4");
					case 4:
						pid_output_ARM = rc_filter_march(&ARM_PID4, error) * 2048;
						break;
						//printf("5");
					case 5:
						pid_output_ARM = rc_filter_march(&ARM_PID5, error) * 2048;
						break;
						//printf("6");
					case 6:
						pid_output_ARM = rc_filter_march(&ARM_PID6, error) * 2048;
						break;
						//printf("7");
					case 7:
						pid_output_ARM = rc_filter_march(&ARM_PID7, error) * 2048;
						break;
						//printf("8");
				}
				int32_t pid_output = (int32_t)pid_output_ARM;
				int32_t positive_pid_output = (pid_output>=0);
				int32_t pid_output_cutoff = fabs(pid_output)*(fabs(pid_output) <= 2047) + 2047*(fabs(pid_output) > 2047);	
				
				//For direction
				if(j < 4 ){
					alt_write_word(h2p_lw_pwm_values_addr[j], (pid_output_cutoff));
					
					dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
					if(positive_pid_output)
						dir_bitmask |= (1<<(j+4));
					else
						dir_bitmask &= ~(1<<(j+4));
					alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
				}
				else{
					if (j < 8){
					alt_write_word(h2p_lw_pwm_values_addr[j], (pid_output_cutoff));
					dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
					if(positive_pid_output == 0)
						dir_bitmask |= (1<<(j-4));
					else
						dir_bitmask &= ~(1<<(j-4));
					alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
					}
				}
				

				if(0 && myCounter%500 == 0 && j == 7){
					printf("Raw value read from the ADC is : %d\n", adc_data);
					printf("Average current value is: %f\n", avg_current);

					//External encoder stuff -> this is currently the linear encoder on stage
					printf("Linear encoder counts is: %d\n", arm_encoders1);
					float inches = M_PI*1.141*arm_encoders1*0.15/360*1.029; //1.029 is a calibration constant
					printf("Linear encoder value in inches is: %f\n", inches);

					// int dval = max_val - min_val;
					// printf("Axis: %d; AVG tracking error: %lf Position Setpoint: %d; Error: %d; Current PID output, unsigned: %d; Cutoff output: %d\n", j, tracking_error[j], position_setpoints[j], error, pid_output, pid_output_cutoff);
					// printf("Heartbeat: %d; Error: %d; Error read: %d; PID out: %d\n", myCounter, error, check_error, pid_output);
					// printf("value range: %d\n", dval);

					printf("Motor encoder counts: %d,%d,%d,%d,%d,%d,%d,%d\n", internal_encoders[0], internal_encoders[1], internal_encoders[2], internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7]);
					
					// printf("External encoder counts: %d, %d, %d, %d\n", arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4);
					printf("%d,%d,%d,%d,%d,%d,%d,%d\n", switch_states[0],switch_states[1],switch_states[2],switch_states[3],switch_states[4],switch_states[5],switch_states[6],switch_states[7]);
					// printf("%d,%d,%d,%d,%d,%d,%d,%d\n", position_setpoints[0],position_setpoints[1],position_setpoints[2],position_setpoints[3],position_setpoints[4],position_setpoints[5],position_setpoints[6],position_setpoints[7]);

					printf("\n\n");
				}
			}
		}

		myCounter++;
		usleep(1000);

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

