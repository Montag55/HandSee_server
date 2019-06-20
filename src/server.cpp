#include "server.hpp"
#include <string>
#include <iostream>

Server::Server(char* server_ip, char* server_port, int buffer_size):
  m_server_ip {server_ip},
  m_server_in_port {std::stol(server_port)},
  m_buffer_size {buffer_size}
  {
    m_buffer.resize(m_buffer_size);
  }

Server::~Server(){}

bool Server::run(){
  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);

  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      std::cout << "socket failed" << std::endl;
      exit(EXIT_FAILURE);
  }

  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
      std::cout << "setsockopt" << std::endl;
      exit(EXIT_FAILURE);
  }

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(m_server_in_port);

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

  while(valread = read( new_socket , m_buffer.data(), m_buffer.size()) > 0 ){
    for(unsigned int i = 0; i < m_buffer.size(); i++){
      if(&m_buffer[i] != NULL)
        std::cout << "i: " << i << "; " << &m_buffer[i] << std::endl;
    }
  }

  return EXIT_FAILURE;
}
