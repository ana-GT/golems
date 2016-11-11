/*
 *  This extra small demo sends a random samples to your speakers.
 */
#include <alsa/asoundlib.h>
#include <stdio.h>
#include <math.h>

static char *device = "default";                        /* playback device */
snd_output_t *output = NULL;
unsigned char buffer[16800];                          /* some random data */

int main( int argc, char* argv[] ) {

  printf("Sizeof buffer[16*1024] = %d \n", sizeof(buffer) );
  
  int err;
  unsigned int i;
  snd_pcm_t *handle;
  snd_pcm_sframes_t frames;
  
  int vol = 64;
  double sample_rate = 44100; // Hz
  double frequency = 261.625565;
  int channels = 1;
  int allow_resampling = 1;

  for (i = 0; i < sizeof(buffer); i++) {
    //  buffer[i] = random() & 0xff;
    buffer[i] = vol*(1 + sin(i*2*3.1416/sample_rate*frequency) );
  }


  if ((err = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
    printf("Playback open error: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  if ((err = snd_pcm_set_params(handle,
				SND_PCM_FORMAT_U8,
				SND_PCM_ACCESS_RW_INTERLEAVED,
				channels,
				sample_rate,
				allow_resampling,
				500000)) < 0) {   /* 0.5sec */
    printf("Playback open error: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
   
  time_t ts, tf;
  double dt;
  for (i = 0; i < 20; i++) {
    ts = time(NULL);
    printf("Try 16 time? i=%d \n", i);

    frames = snd_pcm_writei(handle, buffer, sizeof(buffer));
    if (frames < 0) {
      frames = snd_pcm_recover(handle, frames, 0);
    } if (frames < 0) {
      printf("snd_pcm_writei failed: %s\n", snd_strerror(frames));
      break;
    }
    if (frames > 0 && frames < (long)sizeof(buffer)) {
      printf("Short write (expected %li, wrote %li)\n", (long)sizeof(buffer), frames);
    } else if( frames == sizeof(buffer) ) {
      printf("The same size! \n");
    }
    
    tf = time(NULL);
    dt = (double)(tf-ts);
    printf("Dt in for [%d] : %f \n", i, dt);
  }

  printf("Closing \n");
  snd_pcm_close(handle);
  return 0;
}
