/**
 * @file crichton_Tactile_save.cpp
 * @brief Save data for left hand in a file when receiving the signal to do so (pressing a key)
 */
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sns.h>

ach_channel_t tactile_chan;
ach_channel_t signal_chan;

/****/
int main( int argc, char* argv[] ) {

  ach_status r;
  char c;

  // Open channel
  r = ach_open( &tactile_chan, "tactile-left", NULL );
  if( r != ACH_OK ) { printf("Channel tactile-left could NOT be opened \n"); }

  // Constantly read channel
  size_t frame_size;
  struct sns_msg_sdh_tactile* msg = 0;
  int counter = 0;

  while( true ) {
    r = sns_msg_local_get( &tactile_chan, (void**)&msg, &frame_size, NULL, ACH_O_LAST );

    // If read, show
    printf("\n\n");
    if( r == ACH_OK || r == ACH_MISSED_FRAME ) {
      for( int i = 0; i < 6; ++i ) {
        printf("[%d] Area: %f Force: %f Cog_x: %f Cog_y: %f \n", i, msg->area[i], msg->force[i], msg->cog_x[i], msg->cog_y[i] );
      }
    

    //-- Store
    short unsigned int* pt = msg->x;
    int width[6] = {6,6,6,6,6,6};
    int height[6] = {14,13,14,13,14,13};

    char name[50];
    sprintf( name, "test_stored_%d.txt", counter );
    std::ofstream ofs( name, std::ofstream::out );
    // Timestamp
    // Sensors
    for( int i = 0; i < 6; ++i ) {
       ofs << msg->area[i] << "  " << msg->force[i] << " " << msg->cog_x[i] << " " << msg->cog_y[i] << std::endl;

       for( int j = 0; j < height[i]; ++j ) {
         for( int k = 0; k < width[i]; ++k ) {
            ofs << *pt << " ";
            pt++;
         }
         ofs << std::endl;
       } 

    } 

    ofs.close();
    counter++;

   }
   usleep(1*1e6);
  } // end while

}
