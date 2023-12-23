#pragma once

#include <string>

namespace msquare {
namespace ddp {

void base64_encode(unsigned char const *, unsigned int len, std::string *ret);

void base64_decode(std::string const &s, std::string *ret);

} // namespace ddp
} // namespace msquare