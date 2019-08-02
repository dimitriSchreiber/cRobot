//
// Created by bunny on 7/28/19.
//

#ifndef UNTITLED_MAIN_H
#define UNTITLED_MAIN_H

#include "fpga_headers/hps_0.h"
#include "fpga_headers/hwlib.h"
#include "fpga_headers/socal.h"
#include "fpga_headers/hps.h"
#include "fpga_headers/alt_gpio.h"

#include <thread>


//Globals
uint32_t currentStatus = 0;
volatile unsigned long *h2p_lw_led_addr;
volatile unsigned long *h2p_lw_gpio_addr;
unsigned char* h2p_lw_heartbeat_addr;
volatile unsigned long *h2p_lw_pid_values_addr;
unsigned char* h2p_lw_quad_reset_addr;
volatile unsigned long *h2p_lw_limit_switch_addr;
volatile unsigned long *h2p_lw_pid_e_stop;
unsigned char* h2p_lw_quad_addr[8];
volatile unsigned long *h2p_lw_quad_addr_external[4];
volatile unsigned long *h2p_lw_pid_input_addr[8];
volatile unsigned long *h2p_lw_pid_output_addr[8];
//unsigned char* *h2p_lw_pwm_values_addr[8];
unsigned char* h2p_lw_pwm_values_addr[8];
volatile unsigned long *h2p_lw_adc;
unsigned char* h2p_lw_gpio1_addr;
volatile int32_t position_setpoints[8];


//MACROS
#define SAMPLE_RATE 1000
#define SYNC_TOLERANCE 10
//#define dt (1.0/(float)SAMPLE_RATE)
#define interval_time_us ((int)(dt * 1000000))
#define update_dt 0.01
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define MAX_TRAVEL_RANGE 10000000
#define MAX_CURRENT 2.5 //amps

int E_STATE;
int ERR_RESET;
int CONNECTED;//global flag to indicate if a connection has been made

int32_t beat;
uint8_t switch_states[8];
int32_t internal_encoders[8];
int32_t arm_encoders1,arm_encoders2,arm_encoders3,arm_encoders4;

int exit_flag;

float dt;

int portnumber_global;
int socket_error;
int system_state;
int sockfd, newsockfd; //global socket value such that it can be called in signal catcher to close ports

int CURRENT_FLAG;
int TRAVEL_FLAG;
int ETSOP_FLAG;

//uint8_t P=150;
//uint8_t P2=150;
//uint8_t I=0;
//uint8_t D=0;
#define P_float 0.0005
#define I_float 0.00005
#define D_float 0.0000006
float controllerGain;
float avg_current;
float avg_current_array[8];
uint64_t global_loop_start_time;

pthread_t pth, pth_heartbeat;	// this is our thread identifier

//Struct -> not used?
struct axis_motor{
    double accGoal;
    double velGoal;
    double posGoal;
    double accCurrent;
    double velCurrent;
    double posCurrent;
    double posGoalCurrent;
    double dutyCyle;
    int setpointUpdated;
    int startupFlag;
};


#endif //UNTITLED_MAIN_H
