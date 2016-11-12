/**
 * @file theremin.cpp
 * @brief Client for pickup tasks
 */
#include "theremin.h"
#include "ui_theremin.h"
#include <iostream>
#include <sstream>
#include <string>
#include <stdint.h>
#include <math.h>

#include <errno.h>


/**
 * @brief Constants
 */
double  Theremin::sSharp = 1.05946;
double  Theremin::sEqual_temp_freqs[12] = {16.352, 16.352*Theremin::sSharp, 18.354, 18.354*Theremin::sSharp, 20.602, 21.827, 21.827*Theremin::sSharp, 24.500, 24.500*Theremin::sSharp, 27.500, 27.500*Theremin::sSharp, 30.868};
std::string Theremin::sEqual_temp_labels[12] = { std::string("C*"), std::string("C*#"), std::string("D*"), std::string("D*#"), std::string("E*"), std::string("F*"), std::string("F*#"), std::string("G*"), std::string("G*#"), std::string("A*"), std::string("A*#"), std::string("B*") };
int Theremin::sDiatonic_major_intervals[7] = {0,2,4,5,7,9,11};
int Theremin::sPentatonic_major_intervals[5] = {0,2,4,7,9};
int Theremin::sPentatonic_minor_intervals[5] = {0,3,5,7,10};
int Theremin::blues_intervals[6] = {0,3,5,6,7,10};


/**
 * @function PlayThread
 * @brief Constructor
 */
PlayThread::PlayThread( QObject* parent ) :
  QThread( parent ) {

  this->restart = false;
  this->abort = false;

  this->vol = log10(7);
  this->fs = 44100; // Hz
  this->ft = 261.625565;
  this->channels = 2;
  this->allow_resampling = 1;

  this->freq_min = 20;
  this->freq_max = 2000;

  audio_start();

}

/**
 * @function ~PlayThread
 * @brief Destructor
 */
PlayThread::~PlayThread() {
  this->mutex.lock();
  this->abort = true;
  this->audio_stop();
  //this->condition.wakeOne();
  this->mutex.unlock();

  this->wait();

}

/**
 * @function setVolume
 * @brief Set volume
 */
void PlayThread::setVolume( double _volume ) {
  this->mutex.lock();
  this->vol = _volume;
  this->mutex.unlock();
}



/**
 * @function set_tone
 */
void PlayThread::set_tone( double _freq, 
			   double _vol ) {

  this->mutex.lock();
  this->ft = _freq;
  this->vol = _vol;
  /*
  if( this->mode == 'discrete' ) {
    closest = tone_filter( freq );
  } else {
    closest = freq;
  }

  */
  this->mutex.unlock();
}

/**
 * @function audio_start
 * @brief Start ALSA libraries to produce sound
 */
bool PlayThread::audio_start() {

  int rc;
  // Open device
  /*
  rc = snd_pcm_open( &this->handle, 
		     "default", //"plughw:0,0", 
		     SND_PCM_STREAM_PLAYBACK, 0 );
  // Check error
  if( rc < 0 ) {
    printf("Cannot open default device: %s \n",
	   snd_strerror(rc) );
    return false;
  }

  // Set desired hardware properties
  snd_pcm_set_params( this->handle,
		      SND_PCM_FORMAT_U8,
		      SND_PCM_ACCESS_RW_INTERLEAVED,
		      this->channels,
		      this->fs,
		      this->allow_resampling,
		      250000 ); // 0.25
  */
  unsigned int card = 0; 
  unsigned int device = 0;
  unsigned int flags = PCM_OUT; 
  struct pcm_config config;
  memset( &config, 0, sizeof(config) );//oh so important
  config.channels = this->channels;
  config.rate = this->fs;
  config.format = PCM_FORMAT_S16_LE;
  config.period_size = 1024;
  config.period_count = 4;
  config.start_threshold = 0;
  config.stop_threshold = 0;
  config.silence_threshold = 0;
/*
  this->handle = pcm_open( card, device,
			   flags, &config );

  if( !this->handle || !pcm_is_ready(this->handle) ) {
    printf("ERROR IN BEING READY!!!!!\n");
  }
  if( !this->handle ) { printf("Something wrong becaus device didn't open \n"); }
*/
}

/**
 * @function audio_stop
 * @brief Stop ALSA libraries
 */
void PlayThread::audio_stop() {
  /*
  snd_pcm_drain( this->handle );
  snd_pcm_close( this->handle );
  */
  pcm_close( this->handle );
}

/**
 * @function run
 * @brief Run...well, run
 */
void PlayThread::run() {
  
  long loops;
  int rc;
  int size;
  //snd_pcm_uframes_t frames;
  unsigned int frames;
  unsigned int val;
  int dir;

  // 2 channels, 2 bytes (stereo)
  frames = 1024*4*2*2;
  //unsigned char buffer[frames*1];// 1 bytes (8) * 1 channels
  char buffer[frames*1];

  for( long t = 0; ;++t ) {

  // Fill buffer
  for( int i = 0; i < sizeof(buffer)/2; ++i ) {
    buffer[2*i] = 50*this->vol*(1+sin(i*2*3.1416/this->fs*this->ft) );
    buffer[2*i+1] = buffer[2*i];
  }
  /*
  for( int i = 0; i < frames; i = i + 1000 ) {
    printf("Sample[%d]: %d [%d]: %d \n", 
	   i, buffer[i], 
	   i+1, buffer[i+1] );   
	   }*/



    this->mutex.lock();
    if( this->abort ) { this->mutex.unlock(); return; }
    if( !this->restart ) { this->mutex.unlock(); t = 0; continue; }
    this->mutex.unlock();
    //rc = snd_pcm_writei( this->handle, buffer, frames );
/*    rc = pcm_write( this->handle, buffer, frames );
    printf("Rc: %d \n", rc);
    if( rc == -EPIPE ) {
      printf("XX Underrun occurred \n");
      //snd_pcm_prepare( this->handle );
      pcm_prepare( this->handle );
    } else if( rc < 0 ) {
      //printf("XX Error from writei: %s \n", snd_strerror(rc) );
      printf("XX Error from writei: %s \n", strerror(rc) );
    } else if( rc != (int)frames ) {
      printf("XX Short write, write %d frames \n", rc );
    }
  */
    
  }
  
  return;

}


/**
* @brief Constructor
* @param parent
*/
Theremin::Theremin(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Theremin) {

    ui->setupUi(this);

    // Setting buttons initial state
    ui->stop_pushButton->setEnabled(false);

    // Bes
    QObject::connect( ui->play_pushButton, SIGNAL(clicked()),
                      this, SLOT(play_slot()) );
    QObject::connect( ui->stop_pushButton, SIGNAL(clicked()),
                      this, SLOT(stop_slot()) );
    QObject::connect( ui->scale_buttonGroup, SIGNAL(buttonClicked(int)),
		      this, SLOT(scale_slot(int)) );
    QObject::connect( ui->output_mode_buttonGroup, SIGNAL(buttonClicked(int)),
                      this, SLOT(output_mode_slot(int)) );
    QObject::connect( ui->volume_verticalSlider, SIGNAL(valueChanged(int)),
                      this, SLOT(volume_slot(int)) );
    QObject::connect( ui->frequency_verticalSlider, SIGNAL(valueChanged(int)),
                      this, SLOT(frequency_slot(int)) );

    //this->playThread = new PlayThread( this );

}


/**
 * @brief Destructor
 * @param parent
 */
Theremin::~Theremin() {

}

/**
* @function play_slot
* @brief Start storing music produced
*/
void Theremin::play_slot() {

  this->playThread->start_thread();
  ui->stop_pushButton->setEnabled(true);
}


/**
 * @funcion stop_slot
 * @brief Stop recording music
 */
void Theremin::stop_slot() {
  printf("Should be stopping now! \n");
  this->playThread->stop_thread();
}


/**
 * @function scale_slot
 */
void Theremin::scale_slot( int _id ) {

    
}


/**
 * @function output_mode_slot
 */
void Theremin::output_mode_slot( int _id ){

}

/**
* @function volume_slot
*/
void Theremin::volume_slot( int _in ){
  printf("Change volume %d \n", _in );
  this->playThread->setVolume( log10( (double)_in )*log10(7.2) );
  //elf.set_tone( self.freq, self.vol );
}


/**
* @function volume_slot
*/
void Theremin::frequency_slot( int _in ){
  int vol_entry = ui->volume_verticalSlider->value();
  this->playThread->set_tone( _in, 
			      log10( (double)vol_entry )*log10(7.2) );
}

