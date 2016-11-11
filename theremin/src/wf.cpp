// filename "wf.cpp" (simple wave-form generator)

   #include <iostream>
   #include <cmath>
   #include <stdint.h>

int main()
   {

   const double R=8000; // sample rate (samples per second)
  double C=261.625565; // frequency of middle-C (hertz)
   const double F=R/256; // bytebeat frequency of 1*t due to 8-bit truncation (hertz)
   const double V=127; // a volume constant

   int up = 8000;
   int next = up;
   for ( int t=0; ; t++ )
      {
	if( t > next ) { C = C +  50.0;  next+= up; }
      uint8_t temp = (sin(t*2*M_PI/R*C)+1)*V; // pure middle C sine wave
   // uint8_t temp = t/F*C; // middle C saw wave (bytebeat style)
   // uint8_t temp = (t*5&t>>7)|(t*3&t>>10); // viznut bytebeat composition
      std::cout<<temp;
      }

   }
