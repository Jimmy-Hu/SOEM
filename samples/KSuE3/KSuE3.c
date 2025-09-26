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
const int CSV_MODE = 9; // Cyclic Synchronous Velocity Mode
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

// --- Global Motion Profile & Status Variables ---
// These are atomic or volatile to ensure safe access between the main and RT threads.
atomic_int_fast64_t g_target_pos_counts = 0;
volatile double g_current_pos_counts = 0.0;
volatile double g_current_vel_cps = 0.0; // Counts per second
volatile double g_max_vel_cps = 0.0;
volatile double g_accel_cps2 = 0.0;      // Counts per second^2
atomic_int g_motion_state = MOTION_IDLE;
volatile atomic_bool g_is_bus_operational = false;
volatile atomic_bool g_is_drive_operational = false;
volatile atomic_bool g_fault_detected = false;
volatile atomic_uint_fast16_t g_current_status_word = 0;
volatile atomic_uint_fast16_t g_current_control_word = 0;
volatile atomic_uint_fast16_t g_last_error_code = 0;
volatile atomic_uint_fast16_t g_driver_status = 0; // For object 0x3C13:D5
volatile atomic_int_fast32_t g_actual_position = 0;


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

    // Calculate final target position in encoder counts
    long long start_pos = g_actual_position;
    g_target_pos_counts = start_pos + (long long)(target_deg * COUNTS_PER_DEGREE);

    g_max_vel_cps = speed_dps * COUNTS_PER_DEGREE;
    g_accel_cps2 = accel_dps2 * COUNTS_PER_DEGREE;

    // Initialize motion state. The RT thread will pick this up.
    g_current_pos_counts = g_actual_position;
    g_current_vel_cps = 0.0;
    g_motion_state = MOTION_ACCELERATING;
}

#ifndef _MSC_VER
// Helper function for POSIX sleep logic
void add_timespec(struct timespec *ts, int64 nsec)
{
   ts->tv_nsec += nsec;
   while (ts->tv_nsec >= 1000000000)
   {
      ts->tv_nsec -= 1000000000;
      ts->tv_sec++;
   }
}
#endif

/**
 * @brief The main real-time thread for cyclic EtherCAT communication.
 */
void* ec_thread_func(void* arg)
{
    (void)arg; // Unused parameter

#ifndef _MSC_VER
    pthread_setname_np(pthread_self(), "ec_thread");
#endif

    bool is_dc_synced = false;
    bool op_request_sent = false;
    bool new_setpoint_toggle = false;

    // Initialize all output data to zero to prevent sending
    // garbage values to the drive on startup.
    memset(output_data, 0, sizeof(out_PDO_t));
    output_data->mode_of_operation = CSP_MODE;

    // --- Timing variables for DC synchronization ---
#ifdef _MSC_VER
    LARGE_INTEGER a, b;
    int64_t to_time;
    QueryPerformanceFrequency(&a);
    QueryPerformanceCounter(&b);
    to_time = b.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif

    while (keep_running)
    {
        // --- DC Synchronization Logic ---
#ifdef _MSC_VER
        to_time += (a.QuadPart / 1000) * CYCLE_TIME_MS;
        QueryPerformanceCounter(&b);
        int64_t sleep_time_ticks = to_time - b.QuadPart;
        if (sleep_time_ticks > 0) {
            Sleep((DWORD)(sleep_time_ticks * 1000 / a.QuadPart));
        }
#else
        add_timespec(&ts, CYCLE_TIME_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
#endif

        ecx_send_processdata(&ec_context);
        int wkc = ecx_receive_processdata(&ec_context, EC_TIMEOUTRET);

        if (wkc < g_expected_wkc && g_is_bus_operational) {
            // Handle error, maybe set a global error flag
        }

        // Update global status variables for the main thread to read
        g_current_status_word = input_data->status_word;
        g_actual_position = input_data->position_actual_value;
        g_current_control_word = output_data->control_word;


        // --- EtherCAT State Machine Handling ---
        if (!g_is_bus_operational)
        {
            if (!is_dc_synced)
            {
                if (ec_context.slavelist[SLAVE_ID].hasdc && (ec_context.DCtime > 0))
                {
                    is_dc_synced = true;
                }
            }

            if (is_dc_synced && !op_request_sent)
            {
                ec_context.slavelist[0].state = EC_STATE_OPERATIONAL;
                ecx_writestate(&ec_context, 0);
                op_request_sent = true;
            }

            if (op_request_sent)
            {
                ecx_readstate(&ec_context);
                if (ec_context.slavelist[SLAVE_ID].state == EC_STATE_OPERATIONAL)
                {
                     g_is_bus_operational = true;
                }
                else if (ec_context.slavelist[SLAVE_ID].state & EC_STATE_ERROR)
                {
                    fprintf(stderr, "\nError: Slave %d is in ERROR state 0x%04X, AL status code: 0x%04X (%s)\n",
                           SLAVE_ID, ec_context.slavelist[SLAVE_ID].state,
                           ec_context.slavelist[SLAVE_ID].ALstatuscode,
                           ec_ALstatuscode2string(ec_context.slavelist[SLAVE_ID].ALstatuscode));
                    keep_running = false; // Stop the application on error
                }
            }
        }
        else // Bus is operational, now handle CiA 402 drive state machine
        {
            if (!g_is_drive_operational)
            {
                // Always command the drive to hold its current position during state transitions
                output_data->target_position = input_data->position_actual_value;
                uint16_t current_status = g_current_status_word;

                // State: Fault (0x...8) -> Send Fault Reset
                if ((current_status & 0x08) != 0)
                {
                    g_fault_detected = true;
                    output_data->control_word = 0x80;
                }
                else
                {
                    g_fault_detected = false;
                    // State: Switch on Disabled (0x..40) -> Send Shutdown
                    if (check_state(current_status, 0x4F, 0x40))
                    {
                        output_data->control_word = 0x06;
                    }
                    // State: Ready to Switch On (0x..21) -> Send Switch On
                    else if (check_state(current_status, 0x6F, 0x21))
                    {
                        output_data->control_word = 0x07;
                    }
                    // State: Switched On (0x..23) -> Send Enable Operation
                    else if (check_state(current_status, 0x6F, 0x23))
                    {
                        output_data->control_word = 0x0F;
                    }
                    // State: Operation Enabled (0x..27) -> Drive is ready
                    else if (check_state(current_status, 0x6F, 0x27))
                    {
                        if (!g_is_drive_operational)
                        {
                            g_is_drive_operational = true;
                            g_current_pos_counts = input_data->position_actual_value;
                            output_data->target_position = input_data->position_actual_value;
                        }
                    }
                }
            }
            else // Drive is operational, execute motion profile
            {
                uint16_t base_control_word = 0x0F; // Command: Enable Operation
                
                motion_state_t current_motion_state = g_motion_state;
                
                // If idle, hold position. Otherwise, calculate next position from profiler.
                if (current_motion_state == MOTION_IDLE)
                {
                     output_data->target_position = input_data->position_actual_value;
                }
                else
                {
                    // Toggle the "new set-point" bit (bit 4) to make
                    // the drive accept the new target_position on each cycle.
                    new_setpoint_toggle = !new_setpoint_toggle;
                    if (new_setpoint_toggle)
                    {
                        base_control_word |= 0x10; // Set bit 4
                    }

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
                        // Condition to end the move
                        if ( (direction == 1 && g_current_pos_counts >= target_pos) ||
                             (direction == -1 && g_current_pos_counts <= target_pos) ||
                             (fabs(distance_to_target) < 100.0) ) // Small tolerance band
                        {
                            g_current_vel_cps = 0;
                            g_current_pos_counts = target_pos;
                            current_motion_state = MOTION_IDLE;
                        }
                    }

                    // Update velocity based on state
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

                    // Update position based on new velocity
                    if (current_motion_state != MOTION_IDLE)
                    {
                        g_current_pos_counts += g_current_vel_cps * CYCLE_TIME_S;
                    }
                    
                    output_data->target_position = (int32_t)g_current_pos_counts;
                }
                output_data->control_word = base_control_word;
            }
        }
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

        // For TI ESCs, overlapped mode is required for correct WKC in OP state
        ec_context.overlappedMode = TRUE;

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

            int timeout_ms = 5000;
            bool motion_started = false;

            // --- Main thread becomes the status printing and timeout loop ---
            while (keep_running)
            {
                if (g_is_drive_operational)
                {
                    if (!motion_started)
                    {
                        // Set the target motion now that the drive is confirmed ready
                        set_target_motion(target_angle, target_speed, target_accel);
                        motion_started = true;
                    }

                    motion_state_t current_motion_state = g_motion_state;
                    printf("Target: %-9lld | Actual: %-9d | State: %-12s | Status: 0x%04X | Control: 0x%04X\n",
                       (long long)g_target_pos_counts,
                       (int)g_actual_position,
                       (current_motion_state == MOTION_ACCELERATING) ? "Accelerating" :
                       (current_motion_state == MOTION_CRUISING) ? "Cruising" :
                       (current_motion_state == MOTION_DECELERATING) ? "Decelerating" : "Idle",
                       (unsigned int)g_current_status_word,
                       (unsigned int)g_current_control_word);
                    fflush(stdout);
                }
                else
                {
                    if (g_fault_detected && g_last_error_code == 0)
                    {
                        uint16_t error_code = 0;
                        int size = sizeof(error_code);
                        int wkc_sdo = ecx_SDOread(&ec_context, SLAVE_ID, 0x3C13, 0x84, FALSE, &size, &error_code, EC_TIMEOUTRXM);
                        if (wkc_sdo > 0)
                        {
                            g_last_error_code = error_code;
                        }
                    }

                     // Print status while waiting for the drive to become operational
                    printf("Waiting... Bus: %s | Drv Status: 0x%04X | Ctrl Sent: 0x%04X | Last Err: 0x%04X | Drv Stat: 0x%04X\r",
                        g_is_bus_operational ? "OP" : "INIT",
                        (unsigned int)g_current_status_word,
                        (unsigned int)g_current_control_word,
                        (unsigned int)g_last_error_code,
                        (unsigned int)g_driver_status);
                    fflush(stdout);

                    timeout_ms -= 100;
                    if (timeout_ms <= 0)
                    {
                        if ((g_current_status_word & 0x08) != 0)
                        {
                            fprintf(stderr, "\nError: Drive timed out in FAULT state (0x%04X). Last Error Code: 0x%04X\n", g_current_status_word, g_last_error_code);
                            fprintf(stderr, "This is likely a hardware issue. Please check:\n");
                            fprintf(stderr, "1. 24-48V Motor Power Supply is ON.\n");
                            fprintf(stderr, "2. Motor and Encoder cables are securely connected.\n");
                            fprintf(stderr, "3. The motor is not physically jammed.\n");
                        }
                        else
                        {
                            fprintf(stderr, "\nError: Drive did not become operational within the timeout period. Final status: 0x%04X\n", g_current_status_word);
                        }
                        keep_running = false;
                    }
                }

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

