#ifndef SERVER_HPP
#define SERVER_HPP

#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

class Server{
  public:
    Server(char* server_ip, char* server_port, int buffer_size);
    ~Server();
    bool run();

  private:
    char* m_server_ip;
    long m_server_in_port;
    int m_buffer_size;
    std::vector<char> m_buffer;
};

#endif
