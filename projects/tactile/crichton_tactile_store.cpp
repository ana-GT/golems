/**
 * @file crichton_tactile_store.cpp
 */
#include <iostream>
#include <unistd.h>
#include <sns.h>
#include <dsad.h>

ach_channel_t tactile_chan;
FILE* pgnuplot;
bool printHeader = false;


/****/
int main( int argc, char* argv[] ) {

  ach_status r;


  // Open channel
  r = ach_open( &tactile_chan, "tactile-left", NULL ); 
  if( r != ACH_OK ) { printf("Channel tactile-left could NOT be opened \n"); }

  // Start gnuplot
  pgnuplot = popen("gnuplot -persist", "w");
  fprintf( pgnuplot, "set xrange [%f:%f]\n", -0.5,5.5 ); 
  fprintf( pgnuplot, "set yrange [%f:%f]\n", -0.5,13.5 );

  fprintf( pgnuplot, "set terminal x11 noraise \n" );


  fprintf( pgnuplot, "set palette rgbformula -7,2,7\n");
  fprintf( pgnuplot, "set cbrange [0:3000]\n" );
  fprintf( pgnuplot, "set cblabel 'Pad'\n" );
  fprintf( pgnuplot, "set view map \n" );
 

  // Constantly read channel
  size_t frame_size;
  struct sns_msg_sdh_tactile* msg = 0;
  
  int opt_pad = 0;
  while( true ) {
    r = sns_msg_local_get( &tactile_chan, (void**)&msg, &frame_size, NULL, ACH_O_LAST );
    
    // If read, show
    uint16_t* it = 0;
    int max = 0;

    if( r == ACH_OK || r == ACH_MISSED_FRAME ) {

      fprintf(pgnuplot, "splot '-' matrix with image\n");
      it = msg->x;
      for( int y = 0; y < 13; ++y ) {
        for( int x = 0; x < 6; ++x ) {
          if( *it > max ) { max = *it; }

          int p; 
          if( *it != 0 ) { p = 3000; } else { p = 0; }
          fprintf( pgnuplot, "%d ", p );
          it++;
        } fprintf( pgnuplot, "\n" );
      }
      fprintf( pgnuplot, "e\n");
      fprintf( pgnuplot, "e\n");
      fflush( pgnuplot ); 
    
      printf("Max: %d \n", max );
   }

   usleep(0.5*1e6);
  } // end while


  fclose( pgnuplot );

}
