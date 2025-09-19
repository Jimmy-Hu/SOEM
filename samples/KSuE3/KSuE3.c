#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <math.h>
#include <stdint.h>

// Platform-specific headers for timing and to prevent WinSock redefinition errors
#ifdef _MSC_VER
// WIN32_LEAN_AND_MEAN excludes older Windows headers (like winsock.h) to prevent
// conflicts with networking libraries like SOEM which use winsock2.h.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif

#include "soem/soem.h"

// Global flag to handle graceful shutdown on Ctrl+C
atomic_bool keep_running = true;

// EtherCAT context
ecx_contextt ec_context = {0};
// Global variable for the expected Working Counter
int g_expected_wkc = 0;

// --- Motion & Drive Constants ---
const int SLAVE_ID = 1; // We only have one slave
const int CSP_MODE = 8; // Cyclic Synchronous Position Mode
const double COUNTS_PER_REVOLUTION = 2097152.0; // From drive manual (Object 608Fh:01h, 2^21)
const double COUNTS_PER_DEGREE = 5825.422222222222;//COUNTS_PER_REVOLUTION / 360.0;
const int CYCLE_TIME_MS = 2;
const double CYCLE_TIME_S = 0.002;//CYCLE_TIME_MS / 1000.0;
const long long CYCLE_TIME_NS = 2 * 1000000LL;

// --- Motion Profile State Machine ---
typedef enum {
    MOTION_IDLE,
    MOTION_ACCELERATING,
    MOTION_CRUISING,
    MOTION_DECELERATING
} motion_state_t;

// --- Global Motion Profile Variables ---
// These are atomic or volatile to ensure safe access between the main and RT threads.
atomic_int_fast64_t g_target_pos_counts = 0;
volatile double g_current_pos_counts = 0.0;
volatile double g_current_vel_cps = 0.0; // Counts per second
volatile double g_max_vel_cps = 0.0;
volatile double g_accel_cps2 = 0.0;      // Counts per second^2
atomic_int g_motion_state = MOTION_IDLE;


// Define structs that match the slave's PDO mapping from the manual.
// This provides a clean, type-safe way to access the process data.
#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif

// RxPDO: Master -> Slave (Outputs from Master perspective)
typedef struct
#if defined(__GNUC__) || defined(__clang__)
__attribute__((packed))
#endif
{
    uint16_t control_word;
    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
    int8_t mode_of_operation;
    int32_t velocity_offset;
} out_PDO_t;

// TxPDO: Slave -> Master (Inputs to Master perspective)
typedef struct
#if defined(__GNUC__) || defined(__clang__)
__attribute__((packed))
#endif
{
    uint16_t status_word;
    int32_t position_actual_value;
    int32_t velocity_actual_value;
    int16_t torque_actual_value;
    int32_t following_error_actual_value;
    int8_t mode_of_operation_display;
    uint16_t touch_probe_status;
    int32_t touch_probe_pos1_pos_value;
} in_PDO_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

out_PDO_t* output_data;
in_PDO_t* input_data;


void signal_handler(int signum)
{
    (void)signum;
    keep_running = false;
}

bool check_state(const uint16_t statusword, const uint16_t expected_mask, const uint16_t expected_state)
{
    return (statusword & expected_mask) == expected_state;
}

/**
 * @brief Sets the parameters for a new trapezoidal motion profile.
 * @param target_deg Target angle in degrees.
 * @param speed_dps Maximum speed in degrees per second.
 * @param accel_dps2 Acceleration in degrees per second squared.
 */
void set_target_motion(double target_deg, double speed_dps, double accel_dps2)
{
    printf("\nNew move requested: %.2f degrees at %.2f deg/s.\n", target_deg, speed_dps);
    g_target_pos_counts = (int64_t)(target_deg * COUNTS_PER_DEGREE);
    g_max_vel_cps = speed_dps * COUNTS_PER_DEGREE;
    g_accel_cps2 = accel_dps2 * COUNTS_PER_DEGREE;
    
    // Initialize motion state. The RT thread will pick this up.
    g_current_pos_counts = input_data->position_actual_value; // Start from current actual position
    g_current_vel_cps = 0.0;
    g_motion_state = MOTION_ACCELERATING;
}


/**
 * @brief The main real-time thread for cyclic EtherCAT communication.
 */
void* ec_thread_func(void* arg)
{
    (void)arg; // Unused parameter

#ifndef _MSC_VER
    pthread_setname_np(pthread_self(), "ec_thread");
#endif
    
    bool is_bus_operational = false;
    bool is_drive_operational = false;
    int wkc;
    int op_request_cycle_count = 0;
    const int INITIAL_CYCLES = 50; // Run for 100ms to establish communication

    while (keep_running)
    {
        // Use Distributed Clocks to synchronize the loop
        ec_dcsync0(&ec_context, SLAVE_ID, CYCLE_TIME_NS, 0);

        ecx_send_processdata(&ec_context);
        wkc = ecx_receive_processdata(&ec_context, EC_TIMEOUTRET);

        if (wkc < g_expected_wkc)
        {
            printf("Working Counter less than expected. WKC: %d, Expected: %d\n", wkc, g_expected_wkc);
        }

        op_request_cycle_count++;
        
        // --- EtherCAT State Machine Handling ---
        if (!is_bus_operational)
        {
            // After a few cycles to satisfy the watchdog, request OP state
            if (op_request_cycle_count == INITIAL_CYCLES)
            {
                printf("Requesting OPERATIONAL state for all slaves...\n");
                ec_context.slavelist[0].state = EC_STATE_OPERATIONAL;
                ecx_writestate(&ec_context, 0);
            }
            // Check if the state transition was successful
            else if (op_request_cycle_count > INITIAL_CYCLES)
            {
                ecx_readstate(&ec_context);
                if (ec_context.slavelist[SLAVE_ID].state == EC_STATE_OPERATIONAL)
                {
                     printf("All slaves reached OPERATIONAL state.\n");
                     is_bus_operational = true;
                }
            }
        }
        else // Bus is operational, now handle CiA 402 drive state machine
        {
            uint16_t current_status = input_data->status_word;

            if (!is_drive_operational)
            {
                if (check_state(current_status, 0x4F, 0x40)) { output_data->control_word = 0x06; }
                else if (check_state(current_status, 0x6F, 0x21)) { output_data->control_word = 0x07; }
                else if (check_state(current_status, 0x6F, 0x23)) { output_data->control_word = 0x0F; }
                else if (check_state(current_status, 0x6F, 0x27))
                {
                    if (!is_drive_operational)
                    {
                        printf("Drive state: Operation Enabled. Ready for motion commands.\n");
                        is_drive_operational = true;
                        g_current_pos_counts = input_data->position_actual_value;
                        output_data->target_position = input_data->position_actual_value;
                    }
                }
            }
            else // Drive is operational, execute motion profile
            {
                motion_state_t current_motion_state = g_motion_state;
                int64_t target_pos = g_target_pos_counts;
                double distance_to_target = (double)target_pos - g_current_pos_counts;
                int direction = (distance_to_target > 0) ? 1 : -1;

                double decel_dist = (g_current_vel_cps * g_current_vel_cps) / (2.0 * g_accel_cps2);
                
                if (current_motion_state == MOTION_ACCELERATING)
                {
                    if (fabs(distance_to_target) <= decel_dist) { current_motion_state = MOTION_DECELERATING; }
                    else if (fabs(g_current_vel_cps) >= g_max_vel_cps) { current_motion_state = MOTION_CRUISING; }
                }
                else if (current_motion_state == MOTION_CRUISING)
                {
                    if (fabs(distance_to_target) <= decel_dist) { current_motion_state = MOTION_DECELERATING; }
                }
                else if (current_motion_state == MOTION_DECELERATING)
                {
                    if (fabs(distance_to_target) < 1.0 || (fabs(g_current_vel_cps) < 100.0 && fabs(distance_to_target) < 1000.0))
                    {
                        g_current_vel_cps = 0;
                        g_current_pos_counts = target_pos;
                        current_motion_state = MOTION_IDLE;
                    }
                }

                switch (current_motion_state)
                {
                    case MOTION_ACCELERATING:
                        g_current_vel_cps += direction * g_accel_cps2 * CYCLE_TIME_S;
                        if (fabs(g_current_vel_cps) > g_max_vel_cps) { g_current_vel_cps = direction * g_max_vel_cps; }
                        break;
                    case MOTION_CRUISING:
                        g_current_vel_cps = direction * g_max_vel_cps;
                        break;
                    case MOTION_DECELERATING:
                        g_current_vel_cps -= direction * g_accel_cps2 * CYCLE_TIME_S;
                        if ((direction == 1 && g_current_vel_cps < 0) || (direction == -1 && g_current_vel_cps > 0)) { g_current_vel_cps = 0; }
                        break;
                    case MOTION_IDLE: break;
                }
                g_motion_state = current_motion_state;
                
                if (current_motion_state != MOTION_IDLE)
                {
                    g_current_pos_counts += g_current_vel_cps * CYCLE_TIME_S;
                }
                
                output_data->target_position = (int32_t)g_current_pos_counts;
            
                printf("Target: %-9lld | Actual: %-9d | State: %-12s | Status: 0x%04X\r", 
                       (long long)target_pos,
                       input_data->position_actual_value, 
                       (current_motion_state == MOTION_ACCELERATING) ? "Accelerating" :
                       (current_motion_state == MOTION_CRUISING) ? "Cruising" :
                       (current_motion_state == MOTION_DECELERATING) ? "Decelerating" : "Idle",
                       current_status);
                fflush(stdout);
            }
        }

        // The old Sleep()/nanosleep() is no longer needed, as ec_dcsync0() handles the timing.
    }
    return NULL;
}


int main(int argc, char* argv[])
{
    if (argc < 4)
    {
        fprintf(stderr, "Usage: %s <ifname> <angle_deg> <speed_dps>\n", argv[0]);
        fprintf(stderr, "Example: %s eth0 360 180\n", argv[0]);
        return EXIT_FAILURE;
    }
    
    const char* const ifname = argv[1];
    double target_angle = atof(argv[2]);
    double target_speed = atof(argv[3]);
    double target_accel = 360.0; // Default acceleration of 1 rev/s^2

    signal(SIGINT, signal_handler);
    pthread_t ec_thread;

    if (ecx_init(&ec_context, ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        if (ecx_config_init(&ec_context) > 0)
        {
            printf("%d slaves found and configured.\n", ec_context.slavecount);
            printf("Slave 1 Name: %s\n", ec_context.slavelist[SLAVE_ID].name);

            char IOmap[4096];
            ecx_config_map_group(&ec_context, &IOmap, 0);

            printf("Configuring Distributed Clocks...\n");
            ecx_configdc(&ec_context);
            printf("DC configuration complete.\n");
            
            g_expected_wkc = (ec_context.grouplist[0].outputsWKC * 2) + ec_context.grouplist[0].inputsWKC;
            printf("Calculated Expected WKC: %d\n", g_expected_wkc);

            output_data = (out_PDO_t*)ec_context.slavelist[SLAVE_ID].outputs;
            input_data = (in_PDO_t*)ec_context.slavelist[SLAVE_ID].inputs;
            
            printf("Configuring SDOs...\n");
            int wkc = 0;
            int8_t mode = CSP_MODE;
            wkc = ecx_SDOwrite(&ec_context, SLAVE_ID, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
            if (wkc == 0) {
                 fprintf(stderr, "Error: Failed to set Mode of Operation via SDO.\n");
                 ecx_close(&ec_context);
                 return EXIT_FAILURE;
            }
            printf("Mode of Operation set to CSP (8).\n");

            printf("Requesting SAFE-OPERATIONAL state for all slaves...\n");
            ec_context.slavelist[0].state = EC_STATE_SAFE_OP;
            ecx_writestate(&ec_context, 0);
            
            int chk = ecx_statecheck(&ec_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            if (chk != EC_STATE_SAFE_OP)
            {
                 fprintf(stderr, "Error: Not all slaves reached SAFE-OP state. Current state: 0x%04X\n", chk);
                 ecx_close(&ec_context);
                 return EXIT_FAILURE;
            }
            printf("All slaves reached SAFE-OPERATIONAL state.\n");
            
            pthread_create(&ec_thread, NULL, &ec_thread_func, NULL);
            
            // Wait a moment for thread to initialize and reach Operation Enabled state
#ifdef _MSC_VER
            Sleep(500);
#else
            struct timespec sleep_time = {0, 500 * 1000000};
            nanosleep(&sleep_time, NULL);
#endif
            
            // Set the target motion after the thread is running
            set_target_motion(target_angle, target_speed, target_accel);


            while (keep_running)
            {
#ifdef _MSC_VER
                Sleep(100);
#else
                struct timespec sleep_time = {0, 100 * 1000000};
                nanosleep(&sleep_time, NULL);
#endif
            }
            
            pthread_join(ec_thread, NULL);
        }
        else
        {
            fprintf(stderr, "No slaves found!\n");
        }

        printf("\nRequesting INIT state for all slaves...\n");
        ec_context.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&ec_context, 0);
        
        ecx_close(&ec_context);
        printf("EtherCAT socket closed.\n");
    }
    else
    {
        fprintf(stderr, "ec_init on %s failed.\n", ifname);
        return EXIT_FAILURE;
    }
    
    printf("Shutdown complete.\n");
    return EXIT_SUCCESS;
}

