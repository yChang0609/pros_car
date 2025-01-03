// file: src/base64_utils.hpp

#ifndef BASE64_UTILS_HPP
#define BASE64_UTILS_HPP

#include <string>
#include <vector>
#include <stdexcept>

inline std::vector<unsigned char> base64_decode(const std::string &in) {
    std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    
    std::vector<unsigned char> out;
    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++) {
        T[base64_chars[i]] = i;
    }
    
    int val=0, valb=-8;
    for (unsigned char c : in) {
        if (T[c] == -1) break;
        val = (val<<6) + T[c];
        valb += 6;
        if (valb >= 0) {
            out.push_back(char((val>>valb)&0xFF));
            valb -= 8;
        }
    }
    return out;
}

#endif // BASE64_UTILS_HPP
