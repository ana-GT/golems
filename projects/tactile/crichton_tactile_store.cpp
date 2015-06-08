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
  fprintf( pgnuplot, "set xlabel 'Time(s)' \n" );
  fprintf( pgnuplot, "set ylabel 'Q' \n" );
  fprintf( pgnuplot, "set yrange [%f:%f]\n", 0,10 ); 

  // Constantly read channel
  size_t frame_size;
  struct sns_msg_sdh_tactile* msg = 0;
  
  int opt_pad = 0;
  while( true ) {
    r = sns_msg_local_get( &tactile_chan, (void**)&msg, &frame_size, NULL, ACH_O_LAST );
    
    // If read, show
    if( r == ACH_OK || r == ACH_MISSED_FRAME ) {

      if( !printHeader ) {
         printHeader = true;
         fprintf(pgnuplot, "splot '-' matrix with image \n");
      } else {
         fprintf( pgnuplot, "replot\n");
      }     


      for( int y=0; y < dsa_offset[opt_pad].y; y ++ ) {
        for( int x=0; x < dsa_offset[opt_pad].x; x ++ ) {
            fprintf( pgnuplot, "%04.0d ",
                    AA_MATREF( msg->x + dsa_offset[opt_pad].off,
                               y, x, dsa_offset[opt_pad].x ));
        }
        printf("\n");
      }
  

    }

   usleep(0.5*1e6);
  } // end while


  fclose( pgnuplot );

}
