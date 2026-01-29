#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>

extern void controlTask_create(void);
extern void serverTask_create(void);
//extern void statsTask_create(void); //deprecated process used for debug

/*FOR DEBUG ONLY*/
//static uint64_t start_time = 0;
//functions used by FreeRTOS to mark start time of a count and detect how long it's been. Uses POSIX calls.
//NOTE: this was my first try at these, but I have been convinced not to use gettimeofday() because NTP sync messes with the output unpredictably
//see: https://blog.habets.se/2010/09/gettimeofday-should-never-be-used-to-measure-time.html
//TODO: change to use clock_gettime(CLOCK_MONOTONIC, &tv) instead
/* void vConfigureTimerForRunTimeStats(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    start_time = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;

    printf("runtime counter init at %llu us\n", (unsigned long long)start_time);
}

unsigned long ulGetRunTimeCounterValue(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    uint64_t current_time = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
    uint64_t elapsed_time = current_time - start_time;

    return (unsigned long)elapsed_time;
}  */
/* static void print_stats(void){
    char buffer[2048];
    memset(buffer, 0, sizeof(buffer)); // clear buffer

    printf("Calling vTaskGetRunTimeStats...\n");
    vTaskGetRunTimeStats(buffer);

    printf("buffer length: %zu\n", strlen(buffer));
    printf("buffer contents (raw): \n%s\n", buffer);

    if (strlen(buffer) == 0){
        printf("the buffer is empty :(");
    }

} */
/* void test_port_runtime_counter(void){
    printf("Testing runtime counter:\n");

    extern uint32_t ulPortGetRunTime(void);

    uint32_t start = ulPortGetRunTime();
    printf("Start at: %lu\n", (unsigned long)start);

    //wait
    for (volatile int i = 0; i < 50000000; i++);

    uint32_t end = ulPortGetRunTime();
    printf("End at: %lu\n", (unsigned long)end);

} */

int main(void){
    printf("Controller starting...\n");

    controlTask_create();
    serverTask_create();
    //statsTask_create(); //deprecated, used for debug

    vTaskStartScheduler();

    //Should never get here
    printf("Scheduler Failed :(\n");
    return 1;
    
}
