#pragma once

#include <assert.h>
#include <arpa/inet.h>

struct MicasenseParams {
    public:
        std::string ip = "192.168.0.83";
        bool use_red = true;
        bool use_blue = true;
        bool use_green = true;
        bool use_red_edge = true;
        bool use_near_infrared = true;
        bool use_panchromatic = true;
        bool use_thermal = true;

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

    int get_channel_bit_mask() {
      return channel_bit_mask;
    }

    void set_channel_bit_mask(int bit_mask) {
      channel_bit_mask = bit_mask;
    }
    
    private:
        int channel_bit_mask = 63;
};