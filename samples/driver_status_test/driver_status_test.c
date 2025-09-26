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
    if (argc < 4)
    {
        fprintf(stderr, "Usage: %s <ifname> <index> <subindex>\n", argv[0]);
        fprintf(stderr, "Example: %s eth0 0x3C13 0xD5\n", argv[0]);
        fprintf(stderr, "         Index and subindex can be in hex (0x...) or decimal.\n");
        return EXIT_FAILURE;
    }

    const char* const ifname = argv[1];
    // Convert command-line arguments to numbers. strtol with base 0 auto-detects hex.
    uint16_t object_index = (uint16_t)strtol(argv[2], NULL, 0);
    uint8_t object_subindex = (uint8_t)strtol(argv[3], NULL, 0);
    
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
                printf("Reading Object 0x%04X:%02X...\n", object_index, object_subindex);

                // Main diagnostic loop
                while (keep_running)
                {
                    // SDOs are transported over EtherCAT frames. We must
                    // maintain the cyclic frame exchange for mailbox communication to work.
                    ecx_send_processdata(&ec_context);
                    ecx_receive_processdata(&ec_context, EC_TIMEOUTRET);

                    // We will read into a 32-bit integer, as it can hold 8, 16, and 32-bit values.
                    uint32_t sdo_value = 0;
                    int size = sizeof(sdo_value);
                    
                    // SDO Read of the user-specified object
                    int wkc_sdo = ecx_SDOread(&ec_context, SLAVE_ID, object_index, object_subindex, FALSE, &size, &sdo_value, EC_TIMEOUTRXM);

                    if (wkc_sdo > 0)
                    {
                        // Display the value based on the number of bytes read back
                        if (size == 1) // 8-bit
                        {
                            printf("Object 0x%04X:%02X (8-bit):  0x%02X (%u)\r", object_index, object_subindex, (uint8_t)sdo_value, (uint8_t)sdo_value);
                        }
                        else if (size == 2) // 16-bit
                        {
                            printf("Object 0x%04X:%02X (16-bit): 0x%04X (%u)\r", object_index, object_subindex, (uint16_t)sdo_value, (uint16_t)sdo_value);
                        }
                        else // 32-bit
                        {
                             printf("Object 0x%04X:%02X (32-bit): 0x%08X (%u)\r", object_index, object_subindex, sdo_value, sdo_value);
                        }
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