#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sns.h>
#include <dsad.h>

ach_channel_t tactile_chan;

int main( int argc, char* argv[] ) {

  srand( time(NULL) );

  ach_status r;

  // Open channel
  r = ach_open( &tactile_chan, "tactile-left", NULL );
  if( r != ACH_OK ) { printf("Channel tactile-left could NOT be opened \n"); }

  // While
  while( true ) {

    // Create msg of sdh_tactile type - Randomly
    struct sns_msg_sdh_tactile *msg = (struct sns_msg_sdh_tactile *) alloca( sns_msg_sdh_tactile_size_n(DSA_NCELLS));

    msg->header.n = DSA_NCELLS;
    sns_msg_header_fill( &msg->header );


    for( size_t i = 0; i < DSA_NCELLS; i ++ ) {
       msg->x[i] = rand() % 10;
    }

    for( size_t i = 0; i < 6; ++i ) {
       msg->area[i] = rand() % 10;
       msg->cog_x[i] = rand() % 10;
       msg->cog_y[i] = rand() % 10;
       msg->force[i] = rand() % 10;
    }

    r = ach_put( &tactile_chan, msg, sns_msg_sdh_tactile_size(msg) );

    // Time
    usleep(0.5*1e6);

  } // end while

}
