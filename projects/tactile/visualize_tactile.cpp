/**
 * @file visualize_tactile.cpp
 */
#include <iostream>
#include <unistd.h>
#include <sns.h>
#include <dsad.h>

ach_channel_t tactile_chan;
FILE* pgnuplot;
bool printHeader = false;
int opt_frequency = 30;
int opt_side = 0; // Left
char opt_chan_name[50];

/****/
int main( int argc, char* argv[] ) {
/*
  if( argc == 2 ) { opt_side = atoi(argv[1] ); }
  if( opt_side != 0 && opt_side != 1 ) { opt_side = 0; }	
	
  opt_side == 0? sprintf(opt_chan_name,"tactile-left"): sprintf(opt_chan_name, "tactile-right");
*/
  ach_status r;

  // Open channel
  r = ach_open( &tactile_chan, "tactile-left", NULL ); 
  if( r != ACH_OK ) { printf("Channel tactile-left could NOT be opened \n"); }
  ach_flush( &tactile_chan );

  // Start gnuplot
  pgnuplot = popen("gnuplot -persist", "w");
  fprintf( pgnuplot, "set xrange [%f:%f]\n",-0.5,17.5 ); 
  fprintf( pgnuplot, "set yrange [%f:%f]\n", -0.5,26.5 );

  fprintf( pgnuplot, "unset key\n" );
  fprintf( pgnuplot, "set xtics -0.5,6\n" );  
  fprintf( pgnuplot, "set ytics -0.5,14\n" );
  fprintf( pgnuplot, "set grid lt 1 lc rgb '#000000'\n");

  fprintf( pgnuplot, "set palette rgbformulae -7,2,-7\n");
  fprintf( pgnuplot, "set cbrange [0:3000]\n" );

  fprintf( pgnuplot, "set terminal x11 noraise \n" );

  fprintf( pgnuplot, "set view map \n" );
 

  // Constantly read channel
  size_t frame_size;
  struct sns_msg_sdh_tactile* msg = 0;
 
 
  int height[6] = {14,13,14,13,14,13};
  int offset[6] = {0,84,162,246,324,408};
  int counter;
  int p;

  while( true ) {
    r = sns_msg_local_get( &tactile_chan, (void**)&msg, &frame_size, NULL, ACH_O_LAST );

    if( r == ACH_OK || r == ACH_MISSED_FRAME ) {
        fprintf( pgnuplot, "set title 'Pad' \n" );        
        fprintf(pgnuplot, "splot '-' matrix with image \n");

        for( int y = 13 -1; y >= 0; y-- ) {
          for( int finger = 0; finger <= 4; finger = finger + 2 ) {
            counter = offset[finger] + y*6;
            for( int x = 0; x < 6; ++x ) {
              if( msg->x[counter] != 0 ) { p = 3000; } else { p = 0; }
              fprintf( pgnuplot, "%d ", p );
              counter++;              
            }
          }
          fprintf( pgnuplot, "\n" );
        }

 
        for( int y = 14 -1; y >= 0; y-- ) {
          for( int finger = 1; finger <= 5; finger = finger + 2 ) {
            counter = offset[finger] + y*6;
            for( int x = 0; x < 6; ++x ) {
              if( msg->x[counter] != 0 ) { p = 3000; } else { p = 0; }
              fprintf( pgnuplot, "%d ", p );
              counter++;              
            }
          }
          fprintf( pgnuplot, "\n" );
        }
 

        fprintf( pgnuplot, "e\n");
        fprintf( pgnuplot, "e\n");       
        fflush( pgnuplot ); 
   }

   usleep( (1.0/opt_frequency)*1e6);
   aa_mem_region_local_release();
  } // end while


  fclose( pgnuplot );

}
