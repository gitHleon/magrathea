#ifndef STRING_UTILS_H_
#define STRING_UTILS_H_
#include <cstring>
#include <ctype.h>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>

inline std::string downcase(const std::string &s)
{
    std::string str = s;
    transform(str.begin(), str.end(), str.begin(), tolower);
    return s;
}
inline std::string uppercase(const std::string &s)
{
    std::string str = s;
    transform(str.begin(), str.end(), str.begin(), toupper);
    return s;
}

inline bool isws(char c, char const * const wstr=" \t\n\r")
{
    return (strchr(wstr, c) != NULL);
}

inline std::string trim_right(const std::string &s)
{
    std::string b=" \t\n\r";
    std::string str = s;
    return str.erase(str.find_last_not_of(b) +1);
}
inline std::string trim_left(const std::string &s)
{
    std::string b=" \t\n\r";
    std::string str = s;
    return str.erase( 0, str.find_first_not_of(b) );
}
inline std::string trim_str(const std::string &s)
{
    std::string str = s;
    return trim_left(trim_right(str) );
}

/**
 * Splits a string into tokens separated by any of the given
 * separators.
 */
template <typename Container>
void stringtok (Container &container, std::string const &in,
                const char * const delimiters = " \t\n", bool trim=false)
{
    char *token, *dup = strdup(in.c_str());
    while ((token=strsep(&dup, delimiters)))
    {
        std::string stok(token);
        if (trim)
            stok = trim_str(stok);

        container.push_back(stok);
    }
    free(dup);
}


#endif /*UTILS_H_*/
