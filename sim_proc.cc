#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_proc.h"

/*  argc holds the number of command line arguments
    argv[] holds the commands themselves

    Example:-
    sim 256 32 4 gcc_trace.txt
    argc = 5
    argv[0] = "sim"
    argv[1] = "256"
    argv[2] = "32"
    ... and so on
*/
int main (int argc, char* argv[]){

    FILE *FP;               // File handler
    char *trace_file;       // Variable that holds trace file name;
    proc_params params;       // look at sim_bp.h header file for the the definition of struct proc_params
    
    if (argc != 5)
    {
        printf("Error: Wrong number of inputs:%d\n", argc-1);
        exit(EXIT_FAILURE);
    }
    
    params.rob_size     = strtoul(argv[1], NULL, 10);
    params.iq_size      = strtoul(argv[2], NULL, 10);
    params.width        = strtoul(argv[3], NULL, 10);
    trace_file          = argv[4];


    FP = fopen(trace_file, "r");
    if(FP == NULL){
        // Throw error and exit if fopen() failed
        printf("Error: Unable to open file %s\n", trace_file);
        exit(EXIT_FAILURE);
    }

    SuperscalarOOOPipeline superscal_pip = SuperscalarOOOPipeline(params.rob_size, params.iq_size, params.width);

    superscal_pip.process_trace_file(FP);
    printf("# === Simulator Command =========\n");
    printf("# %s %s %s %s %s\n", argv[0], argv[1], argv[2], argv[3], argv[4]);
    printf("# === Processor Configuration ===\n");
    printf("# ROB_SIZE = %lu\n", params.rob_size);
    printf("# IQ_SIZE  = %lu\n", params.iq_size);
    printf("# WIDTH    = %lu\n", params.width);
    superscal_pip.print_sim_result();   


    return 0;
}
