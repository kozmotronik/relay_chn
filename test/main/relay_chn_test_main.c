#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "unity_test_runner.h"


static void print_banner(const char*);

void app_main(void) {

    print_banner("Starting interactive test menu");
    /* This function will not return, and will be busy waiting for UART input.
     * Make sure that task watchdog is disabled if you use this function.
     */
    unity_run_menu();
}

static void print_banner(const char* text)
{
    printf("\n##### %s #####\n\n", text);
}