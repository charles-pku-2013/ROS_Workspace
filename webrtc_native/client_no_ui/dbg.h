#ifndef _DBG_H
#define _DBG_H

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cstring>
#include <sys/time.h>


//!! __VA_ARGS__ 只是原样展开...
#define DBG(...) do { \
            struct timeval __dbg_timeval;   \
            gettimeofday( &__dbg_timeval, NULL );   \
            fprintf( stderr, "[%llu,%llu] ", (unsigned long long)(__dbg_timeval.tv_sec), \
                            (unsigned long long)(__dbg_timeval.tv_usec) );      \
            fprintf( stderr,  "%s:%d in %s(): ", __FILE__, __LINE__, __func__ ); \
            fprintf( stderr, __VA_ARGS__ ); \
            fprintf( stderr, "\n" ); \
        } while(0)

//!! args可以是包含空格和其他任意符号如<<的字符串
#define DBG_STREAM(args) do { \
            struct timeval __dbg_timeval;   \
            gettimeofday( &__dbg_timeval, NULL );   \
            std::stringstream __dbg_stream_stringstream; \
            __dbg_stream_stringstream << "[" << (unsigned long long)(__dbg_timeval.tv_sec) \
                            << "," << (unsigned long long)(__dbg_timeval.tv_usec) << "] "; \
            __dbg_stream_stringstream << __FILE__ << ":" << __LINE__ << " in " \
                            << __func__ << "(): " << args; \
            fprintf(stderr, "%s\n", __dbg_stream_stringstream.str().c_str()); \
        } while(0)


#define DBG_COND(cond, ...) do { \
            if(cond) DBG(__VA_ARGS__);  \
        } while(0)

#define DBG_STREAM_COND(cond, args) do { \
            if(cond) DBG_STREAM(args); \
        } while(0)


inline
unsigned short checksum(const char *buf)
{
  unsigned int count = strlen(buf);
  unsigned char *addr = (unsigned char*)buf;
  register unsigned int sum = 0;
  
  // Main summing loop
  while(count > 1)
  {
    // sum = sum + *((unsigned short *) addr)++;
    sum = sum + *((unsigned short *) addr);
    addr += 2;
    count -= 2;
  }

  // Add left-over byte, if any
  if (count > 0)
    sum = sum + *((unsigned char *) addr);

  // Fold 32-bit sum to 16 bits
  while (sum >> 16)
    sum = (sum & 0xFFFF) + (sum >> 16);

  return(~sum);
}




#endif
