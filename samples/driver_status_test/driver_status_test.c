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

const int SLAVE_ID = 1;

void signal_handler(int signum)
{
    (void)signum;
    keep_running = false;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <ifname>\n", argv[0]);
        fprintf(stderr, "Example: %s eth0\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* const ifname = argv[1];
    signal(SIGINT, signal_handler);

    // Initialize SOEM, bind to physical NIC
    if (ecx_init(&ec_context, ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        // Find and configure slaves
        if (ecx_config_init(&ec_context) > 0)
        {
            printf("%d slaves found and configured.\n", ec_context.slavecount);
            
            // A valid process data map must be configured for the slave
            // to be willing to transition to SAFE-OP.
            char IOmap[4096];
            ecx_config_map_group(&ec_context, &IOmap, 0);


            if (ec_context.slavecount >= SLAVE_ID)
            {
                 // Bring all slaves to SAFE-OPERATIONAL state to enable SDO communication
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
                printf("All slaves reached SAFE-OPERATIONAL state. Ready to read SDOs.\n\n");

                // Main diagnostic loop
                while (keep_running)
                {
                    // **CRITICAL FIX**: SDOs are transported over EtherCAT frames. We must
                    // maintain the cyclic frame exchange for mailbox communication to work.
                    ecx_send_processdata(&ec_context);
                    ecx_receive_processdata(&ec_context, EC_TIMEOUTRET);

                    uint16_t driver_status = 0;
                    int size = sizeof(driver_status);
                    
                    // SDO Read of "Driver Status" (Object 0x3C13, Sub-index 0xD5)
                    int wkc_sdo = ecx_SDOread(&ec_context, SLAVE_ID, 0x3C13, 0xD5, FALSE, &size, &driver_status, EC_TIMEOUTRXM);

                    if (wkc_sdo > 0)
                    {
                        printf("Driver Status (0x3C13:D5): 0x%04X\r", (unsigned int)driver_status);
                        fflush(stdout);
                    }
                    else
                    {
                        // This error is now more meaningful, as we know the bus is active.
                        // It indicates the slave is genuinely busy or has a mailbox issue.
                        fprintf(stderr, "\nWarning: Failed to read SDO (WKC=%d).\n", wkc_sdo);
                    }
                    
                    // Wait before next read
#ifdef _MSC_VER
                    Sleep(500); // 500ms delay
#else
                    struct timespec sleep_time = {0, 500 * 1000000};
                    nanosleep(&sleep_time, NULL);
#endif
                }
            }
            else
            {
                fprintf(stderr, "Error: Slave %d not found on the bus.\n", SLAVE_ID);
            }
        }
        else
        {
            fprintf(stderr, "No slaves found!\n");
        }

        // Clean shutdown
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

