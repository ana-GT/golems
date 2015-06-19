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
int opt_frequency = 30/2;

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

  fprintf( pgnuplot, "unset key\n" );
  fprintf( pgnuplot, "set tic scale 0\n" );
  fprintf( pgnuplot, "set palette rgbformula -7,2,7\n");
  fprintf( pgnuplot, "set cbrange [0:3000]\n" );
  fprintf( pgnuplot, "set cblabel 'Pad'\n" );
  fprintf( pgnuplot, "unset cbtics\n" );

  fprintf( pgnuplot, "set terminal x11 noraise \n" );

  fprintf( pgnuplot, "set view map \n" );
 
  fprintf( pgnuplot, "set size 1,1\n" );
  fprintf( pgnuplot, "set origin 0,0\n" );
//  fprintf( pgnuplot, "set multiplot layout 1,2 columnsfirst upwards title 'Pads'\n" );


  // Constantly read channel
  size_t frame_size;
  struct sns_msg_sdh_tactile* msg = 0;
  int height[6] = {14,13,14,13,14,13};
  int offset[6] = {0,84,162,246,324,408};
  int width[6] = {6,6,6,6,6,6};
  int counter;

  while( true ) {
    r = sns_msg_local_get( &tactile_chan, (void**)&msg, &frame_size, NULL, ACH_O_LAST );
       

    if( r == ACH_OK || r == ACH_MISSED_FRAME ) {

      for( int k = 0; k < 6; k=k+6 ) {
        
        fprintf( pgnuplot, "set title 'Pad %d' \n", k );
        fprintf(pgnuplot, "splot '-' matrix with image \n");
      
        for( int y = height[k]-1; y >= 0; --y ) {
          counter = offset[k] + y*width[k];  
           for( int x = 0; x < width[k]; ++x ) {
            int p; 
            if( msg->x[counter] != 0 ) { p = 3000; } else { p = 0; }
            fprintf( pgnuplot, "%d ", p );
            counter++;
          } 
          fprintf( pgnuplot, "\n" );
        }
        fprintf( pgnuplot, "e\n");
        fprintf( pgnuplot, "e\n");       
   fflush( pgnuplot ); 
      } // for k

   } else {
     printf("Did not get the message for any reason \n");
   }

   usleep( 0.03*1e6);
   aa_mem_region_local_release();
  } // end while


  pclose( pgnuplot );

}
