#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string.h>
#include "server.hpp"

int main(int argc, char **argv) {

    if (argc != 3){
        printf("usage: $./HandSee <server-IP> <server-Port>\n");
        return -1;
    }

    Server server(argv[1], argv[2], 1024);

    if(server.run() == false){
      std::cout << "failed to run server. exiting." << '\n';
    }

    return 0;
}
