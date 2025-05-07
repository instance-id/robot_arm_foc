//
// Created by mosthated on 8/4/24.
//

#ifndef DESKTOP_CAN_STRINGF_H
#define DESKTOP_CAN_STRINGF_H

#endif //DESKTOP_CAN_STRINGF_H

#ifndef STRINGF_H
#define STRINGF_H

#include <string>

template<typename... argv>
std::string stringf(const char *format, argv... args)
{
    const size_t SIZE = std::snprintf(NULL, 0, format, args...);

    std::string output;
    output.resize(SIZE + 1);
    std::snprintf(&(output[0]), SIZE + 1, format, args...);
    return std::move(output);
}

#endif
