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

// --- Global Status & Command Variables ---
volatile atomic_bool g_is_bus_operational = false;
volatile atomic_bool g_is_drive_operational = false;
volatile atomic_bool g_fault_detected = false;
volatile atomic_uint_fast16_t g_current_status_word = 0;
volatile atomic_uint_fast16_t g_current_control_word = 0;
volatile atomic_uint_fast16_t g_last_error_code = 0;
volatile atomic_int_fast32_t g_actual_velocity = 0; // In counts per second
volatile double g_target_velocity_dps = 0.0;     // In degrees per second


// Define structs that match the slave's PDO mapping.
#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif

// RxPDO: Master -> Slave (Outputs from Master)
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

// TxPDO: Slave -> Master (Inputs to Master)
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

#ifndef _MSC_VER
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
 * The main real-time thread for cyclic EtherCAT communication.
 */
void* ec_thread_func(void* arg)
{
    (void)arg;

#ifndef _MSC_VER
    pthread_setname_np(pthread_self(), "ec_thread");
#endif

    bool is_dc_synced = false;
    bool op_request_sent = false;

    memset(output_data, 0, sizeof(out_PDO_t));
    output_data->mode_of_operation = CSV_MODE;

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

        if (wkc < g_expected_wkc && g_is_bus_operational) {}

        g_current_status_word = input_data->status_word;
        g_actual_velocity = input_data->velocity_actual_value;
        g_current_control_word = output_data->control_word;

        if (!g_is_bus_operational)
        {
            if (!is_dc_synced)
            {
                if (ec_context.slavelist[SLAVE_ID].hasdc && (ec_context.DCtime > 0)) { is_dc_synced = true; }
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
                if (ec_context.slavelist[SLAVE_ID].state == EC_STATE_OPERATIONAL) { g_is_bus_operational = true; }
                else if (ec_context.slavelist[SLAVE_ID].state & EC_STATE_ERROR)
                {
                    fprintf(stderr, "\nError: Slave %d is in ERROR state 0x%04X, AL status code: 0x%04X (%s)\n",
                           SLAVE_ID, ec_context.slavelist[SLAVE_ID].state,
                           ec_context.slavelist[SLAVE_ID].ALstatuscode,
                           ec_ALstatuscode2string(ec_context.slavelist[SLAVE_ID].ALstatuscode));
                    keep_running = false;
                }
            }
        }
        else // Bus is operational, handle CiA 402 drive state machine
        {
            if (!g_is_drive_operational)
            {
                uint16_t current_status = g_current_status_word;
                if ((current_status & 0x08) != 0)
                {
                    g_fault_detected = true;
                    output_data->control_word = 0x80;
                }
                else
                {
                    g_fault_detected = false;
                    if (check_state(current_status, 0x4F, 0x40)) { output_data->control_word = 0x06; }
                    else if (check_state(current_status, 0x6F, 0x21)) { output_data->control_word = 0x07; }
                    else if (check_state(current_status, 0x6F, 0x23)) { output_data->control_word = 0x0F; }
                    else if (check_state(current_status, 0x6F, 0x27))
                    {
                        if (!g_is_drive_operational) { g_is_drive_operational = true; }
                    }
                }
            }
            else // Drive is operational, send velocity command
            {
                output_data->control_word = 0x0F;
                int32_t target_velocity_cps = (int32_t)(g_target_velocity_dps * COUNTS_PER_DEGREE);
                output_data->target_velocity = target_velocity_cps;
            }
        }
    }
    return NULL;
}


int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        fprintf(stderr, "Usage: %s <ifname> <speed_dps>\n", argv[0]);
        fprintf(stderr, "Example: %s eth0 90\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* const ifname = argv[1];
    double target_speed_dps = atof(argv[2]);

    signal(SIGINT, signal_handler);
    pthread_t ec_thread;

    if (ecx_init(&ec_context, ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
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
            int8_t mode = CSV_MODE;
            wkc = ecx_SDOwrite(&ec_context, SLAVE_ID, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
            if (wkc == 0) {
                 fprintf(stderr, "Error: Failed to set Mode of Operation via SDO.\n");
                 ecx_close(&ec_context);
                 return EXIT_FAILURE;
            }
            printf("Mode of Operation set to CSV (9).\n");

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
            
            while (keep_running)
            {
                if (g_is_drive_operational)
                {
                    // Set target velocity once the drive is confirmed ready
                    g_target_velocity_dps = target_speed_dps;

                    double actual_dps = (double)g_actual_velocity / COUNTS_PER_DEGREE;
                    printf("Target Vel: %-7.2f dps | Actual Vel: %-7.2f dps | Status: 0x%04X | Control: 0x%04X\r\n",
                           target_speed_dps,
                           actual_dps,
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
                        if (ecx_SDOread(&ec_context, SLAVE_ID, 0x3C13, 0x84, FALSE, &size, &error_code, EC_TIMEOUTRXM) > 0)
                        {
                            g_last_error_code = error_code;
                        }
                    }

                    printf("Waiting for drive... Bus State: %s | Drive Status: 0x%04X | Control Sent: 0x%04X | Last Error: 0x%04X\r\n",
                        g_is_bus_operational ? "OPERATIONAL" : "INITIALIZING",
                        (unsigned int)g_current_status_word,
                        (unsigned int)g_current_control_word,
                        (unsigned int)g_last_error_code);
                    fflush(stdout);

                    timeout_ms -= 100;
                    if (timeout_ms <= 0)
                    {
                        if ((g_current_status_word & 0x08) != 0)
                        {
                            fprintf(stderr, "\nError: Drive timed out in FAULT state (0x%04X). Last Error Code: 0x%04X\n", g_current_status_word, g_last_error_code);
                            fprintf(stderr, "This is likely a hardware issue. Please check physical setup.\n");
                        }
                        else
                        {
                            fprintf(stderr, "\nError: Drive did not become operational. Final status: 0x%04X\n", g_current_status_word);
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
