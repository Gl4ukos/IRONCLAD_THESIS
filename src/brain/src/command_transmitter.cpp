#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"

class CommandTransmitter {
public:
    CommandTransmitter(const std::string& ip, int port) {
        target_ip = ip;
        target_port = port;

        // Create UDP socket
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(target_port);
        inet_pton(AF_INET, target_ip.c_str(), &server_addr.sin_addr);
    }

    ~CommandTransmitter() {
        close(sock);
    }

    // Send velocity and steering as two doubles
    void send_command(double velocity, double steering) {
        double data[2] = {velocity, steering};
        ssize_t sent = sendto(sock, data, sizeof(data), 0,
                              (const struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent != sizeof(data)) {
            perror("sendto failed");
        }
    }


private:
    int sock;
    std::string target_ip;
    int target_port;
    struct sockaddr_in server_addr;
};
