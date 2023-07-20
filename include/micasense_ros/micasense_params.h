#pragma once

#include <assert.h>
#include <arpa/inet.h>

struct MicasenseParams {
    public:
        std::string ip = "192.168.0.83";
        int channel_bit_mask = 63;

    bool validate_ip_address(const std::string &ipAddress) {
        struct sockaddr_in sa;
        int result = inet_pton(AF_INET, ipAddress.c_str(), &(sa.sin_addr));
        return result != 0;
    }

    void assert_valid() {
      assert (channel_bit_mask > 0);
      assert (validate_ip_address(ip));
    }

    const std::string get_ip() {
      return "http://" + ip;
    }
};