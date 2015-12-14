#include <zlib.h>

std::string compress_string(const std::string& str,
                            int compressionlevel = Z_BEST_COMPRESSION);
std::string decompress_string(const std::string& str);
std::string compress_string(const unsigned char * data, int length,
                            int compressionlevel = Z_BEST_COMPRESSION);
