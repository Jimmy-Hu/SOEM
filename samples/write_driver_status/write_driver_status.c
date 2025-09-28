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

// EtherCAT context
ecx_contextt ec_context = {0};

const int SLAVE_ID = 1;

int main(int argc, char* argv[])
{
    if (argc < 6)
    {
        fprintf(stderr, "Usage: %s <ifname> <index> <subindex> <value> <size_bits>\n", argv[0]);
        fprintf(stderr, "Example: %s eth0 0x3413 0 0x0D000000 32\n", argv[0]);
        fprintf(stderr, "         All numeric values can be in hex (0x...) or decimal.\n");
        fprintf(stderr, "         <size_bits> must be 8, 16, or 32.\n");
        return EXIT_FAILURE;
    }

    const char* const ifname = argv[1];
    uint16_t object_index = (uint16_t)strtol(argv[2], NULL, 0);
    uint8_t object_subindex = (uint8_t)strtol(argv[3], NULL, 0);
    uint32_t value_to_write = (uint32_t)strtol(argv[4], NULL, 0);
    int size_in_bits = atoi(argv[5]);

    // Initialize SOEM, bind to physical NIC
    if (ecx_init(&ec_context, ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        // Find and configure slaves
        if (ecx_config_init(&ec_context) > 0)
        {
            printf("%d slaves found and configured.\n", ec_context.slavecount);
            
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
                printf("All slaves reached SAFE-OPERATIONAL state. Ready for SDO communication.\n\n");

                printf("Attempting to write 0x%X to object 0x%04X:%02X...\n", value_to_write, object_index, object_subindex);
                
                int wkc_sdo = 0;
                
                // Perform SDO write based on the specified size
                switch(size_in_bits)
                {
                    case 8:
                    {
                        uint8_t val8 = (uint8_t)value_to_write;
                        wkc_sdo = ecx_SDOwrite(&ec_context, SLAVE_ID, object_index, object_subindex, FALSE, sizeof(val8), &val8, EC_TIMEOUTRXM);
                        break;
                    }
                    case 16:
                    {
                        uint16_t val16 = (uint16_t)value_to_write;
                        wkc_sdo = ecx_SDOwrite(&ec_context, SLAVE_ID, object_index, object_subindex, FALSE, sizeof(val16), &val16, EC_TIMEOUTRXM);
                        break;
                    }
                    case 32:
                    {
                        uint32_t val32 = value_to_write;
                        wkc_sdo = ecx_SDOwrite(&ec_context, SLAVE_ID, object_index, object_subindex, FALSE, sizeof(val32), &val32, EC_TIMEOUTRXM);
                        break;
                    }
                    default:
                        fprintf(stderr, "Error: Invalid size specified. Must be 8, 16, or 32.\n");
                        wkc_sdo = -1; // Mark as error
                        break;
                }

                if (wkc_sdo > 0)
                {
                    printf("SUCCESS: SDO write completed.\n");
                }
                else
                {
                    fprintf(stderr, "FAILURE: SDO write failed (WKC=%d).\n", wkc_sdo);
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
