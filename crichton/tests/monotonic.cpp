#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <ach.h>

int main( int argc, char* argv[] ) {

  #ifdef CLOCK_MONOTONIC
   printf("Clock monotonic defined \n");
  #else
   printf("Not defined \n");
	#endif


  #ifdef ACH_DEFAULT_CLOCK
    printf("Ach default clock defined \n");
    if( ACH_DEFAULT_CLOCK == CLOCK_MONOTONIC ) {
      printf("Monotonic clock identified \n");
    }
	#else
    printf("Ach default clock not defined");
  #endif

  #undef ACH_DEFAULT_CLOCK
  #define ACH_DEFAULT_CLOCK CLOCK_REALTIME

  
  #ifdef ACH_DEFAULT_CLOCK
    printf("2. Ach default clock defined \n");
    if( ACH_DEFAULT_CLOCK == CLOCK_MONOTONIC ) {
      printf("Monotonic clock identified \n");
    } else if( ACH_DEFAULT_CLOCK == CLOCK_REALTIME ) {
      printf("Real time clock identified \n");
    }
	#else
    printf("Ach default clock not defined");
	#endif

}
