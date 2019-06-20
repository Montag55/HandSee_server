#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace cv;


void sendMessage(int connfd, char * format) {
    char sendBuff[1024];
    memset(sendBuff, '0', sizeof(sendBuff));
    snprintf(sendBuff, sizeof(sendBuff), format);
    write(connfd, sendBuff, strlen(sendBuff));
}

int main(int argc, char **argv) {

    if (argc != 3){
        printf("usage: $./HandSee <server-IP> <server-Port>\n");
        return -1;
    }

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    const char* server_ip = argv[1];
    const long port = std::stol(argv[2]);
    char buffer[15773] = {0};
    char bufferTmp[15773] = {0};

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cout << "socket failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cout << "setsockopt" << std::endl;
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( port );

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cout << "bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        std::cout << "listen" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        std::cout << "accept" << std::endl;
        exit(EXIT_FAILURE);
    }
    //while(valread = read( new_socket , buffer, sizeof(buffer)) > 0 ){
        
        int bytesReceived = 0;
        FILE *image;
        image = fopen("recu.jpg", "wba");
        
        while(bytesReceived < sizeof(buffer))
        {
            std::cout << "Reading image byte array" << std::endl;
            int n = 0;
            if ((n = recv(new_socket, buffer, sizeof(buffer), 0)) < 0){
                perror("recv_size()");
                exit(errno);
            }
            bytesReceived += n;
            std::cout << n << std::endl;
            std::cout << "Converting byte array to image" << std::endl;
            fwrite(buffer, sizeof(char), n, image);
            std::cout << "done" << std::endl;

            printf("%s\n",buffer);
            sendMessage(new_socket, "Message Received!");
        }
        fclose(image);
   // }
    return EXIT_FAILURE;
}