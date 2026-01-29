/*
SERVERTASK, COMPRISES:
            COMMAND HANDLER (updates shared memory based on data received from python UI), 
            SERVER TASK (opens and configures a socket, and listens for connections)
            SERVER TASK CREATOR
*/

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

#include "shared.h"

#include "FreeRTOS.h"
#include "task.h"

#define PORT 8888
#define CMD_UPDATE_PID 0x01
#define CMD_UPDATE_PLANT 0x02

#define SERVER_TASK_STACK_SIZE 4096
#define SERVER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

//initialize staging structs and atomic swap flags
volatile pid_params_t pid_staged_params = {0};
atomic_uint_fast8_t pid_ready = 0;

volatile vehicle_plant_t plant_staged_params = {0};
atomic_uint_fast8_t plant_ready = 0;

//read a known number of bytes from the client
static ssize_t read_n_bytes(int client_fd, void *buffer, size_t n) {
    size_t total_bytes_read = 0;
    uint8_t *ptr = (uint8_t *)buffer; //byte pointer to traverse the buffer

    while (total_bytes_read < n) {
        ssize_t bytes = read(client_fd, ptr + total_bytes_read, n - total_bytes_read);

        if (bytes < 0) {
            if(errno == EINTR){
                continue; //expected interrupt error, try again
            }
            return -1; //a different error
        }

        if (bytes == 0) {
            return 0; //connection closed
        }

        total_bytes_read += bytes;
    }

    return total_bytes_read;
}

//write a known number of bytes to the client
static ssize_t write_n_bytes(int client_fd, const void *buffer, size_t n){
    size_t total_bytes_written = 0;
    const uint8_t *ptr = (const uint8_t *)buffer;

    while (total_bytes_written < n){
        ssize_t bytes = write(client_fd, ptr + total_bytes_written, n - total_bytes_written);

        if (bytes < 0){
            if (errno == EINTR){
                continue; //handle expected exception
            }
            return -1; //a real error
        }

        total_bytes_written += bytes;
    }

    return total_bytes_written;
}

static void send_response(int client_fd, char msg){
    write_n_bytes(client_fd, &msg, 1); //write a 1 byte response
}

//TODO: handle setpoint and slope separately since they would probably need to be changed more often than the other parameters
static void handle_pid_update(int client_fd){
    pid_params_t pid_new_params;

    ssize_t bytes = read_n_bytes(client_fd, &pid_new_params, sizeof(pid_params_t));

    if (bytes != sizeof(pid_params_t)) {
        printf("Failed to read PID parameters\n");
        send_response(client_fd, 'N'); //NACK
        return;
    }

    if (pid_new_params.kp < 0.0f || pid_new_params.ki < 0.0f || pid_new_params.kd < 0.0f){
        printf("Invalid: PID parameters cannot be negative\n");
        send_response(client_fd, 'N'); //NACK
        return;
    }

    //update shared memory: copy new param struct to staging struct
    memcpy((void*)&pid_staged_params, &pid_new_params, sizeof(pid_params_t));
    atomic_store_explicit(&pid_ready, 1, memory_order_release);

    printf("PID parameters updated to: Kp = %.3f, Ki = %.3f, Kd = %.3f, setpoint = %.3f m/s\n", pid_new_params.kp, pid_new_params.ki, pid_new_params.kd, pid_new_params.setpoint);
    send_response(client_fd, 'A'); //ACK
}

static void handle_plant_update(int client_fd){
    vehicle_plant_t plant_new_params;

    ssize_t bytes = read_n_bytes(client_fd, &plant_new_params, sizeof(vehicle_plant_t));

    if (bytes != sizeof(vehicle_plant_t)){
        printf("Failed to read plant parameters\n");
        send_response(client_fd, 'N');
        return;
    }

    if (plant_new_params.mass <= 0.0f || plant_new_params.drag_coeff < 0.0f || plant_new_params.maxf <= 0.0f){
        printf("Invalid: plant parameters cannot be negative\n");
        send_response(client_fd, 'N');
        return;
    }

    memcpy((void*)&plant_staged_params, &plant_new_params, sizeof(vehicle_plant_t));
    atomic_store_explicit(&plant_ready, 1, memory_order_release);

    printf("Plant parameters updated to: Mass = %.0f kg, Drag = %.3f, Engine Force = %.0f N, Slope = %.1f deg\n", 
            plant_new_params.mass, plant_new_params.drag_coeff, plant_new_params.maxf, plant_new_params.slope);

    send_response(client_fd, 'A');
    
}

static void server_task(void *params) {
    //silence clang warning
    (void)params;

    int server_fd;
    int client_fd; 
    struct sockaddr_in addr;
    int opt = 1;

    //create socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd < 0){
                    printf("Server: Failed to create socket.\n"); 
                    vTaskDelete(NULL); 
                    return;}

    //ensure socket can be reused
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    //configure bind for IPv4
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);
    
    //bind the socket
    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0){
                    printf("Server: Failed to bind.\n");
                    close(server_fd);
                    vTaskDelete(NULL);
                    return;
    }

    //listen for a single TCP connection
    if (listen(server_fd, 1) < 0){
        printf("Server: Failed to listen.\n");
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }
    printf("Server: Listening on Port %d\n", PORT);

    //accept and handle commands (not real time)
    for(;;) {

        //expected to be blocked by ISR, accept with EINTR retry
        do {
            client_fd = accept(server_fd, NULL, NULL);
        } while (client_fd < 0 && errno == EINTR);
            
        if(client_fd < 0) {
            //some other error not related to the interrupt
            printf("Server: Accept failed with error: %d (%s)\n", errno, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
          
        printf("Server: Client Connected\n");
        
        //main task loop
        for(;;){

            uint8_t cmd;
            ssize_t bytes = read_n_bytes(client_fd, &cmd, 1);

            if (bytes <= 0) {
                printf("Server error. Check client connection.\n");
                break;
            }

            //handle the UI commands
            switch (cmd) {
                case CMD_UPDATE_PID:
                    handle_pid_update(client_fd);
                    break;
                case CMD_UPDATE_PLANT:
                    handle_plant_update(client_fd);
                    break;
                default:
                    printf("Unknown command.\n");
                    send_response(client_fd, 'N');
                    break;
            }

            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        }

        close(client_fd);
    }
}

BaseType_t serverTask_create(void){
    return xTaskCreate(server_task, "Server", SERVER_TASK_STACK_SIZE, NULL, SERVER_TASK_PRIORITY, NULL);
}
