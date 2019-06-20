#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string.h>
#include "server.hpp"

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

    Server server(argv[1], argv[2], 1024);
    server.creatDisplacementMap();
    
    if(server.run() == false){
      std::cout << "failed to run server. exiting." << '\n';
    }

    return 0;

}
