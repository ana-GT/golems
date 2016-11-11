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

PlayThread::PlayThread( QObject* parent ) :
  QThread( parent ) {

  this->restart = false;
  this->abort = false;

  this->vol = 64;
  this->sample_rate = 44100; // Hz
  this->frequency = 261.625565;
  this->channels = 1;
  this->allow_resampling = 1;

  audio_start();

}

PlayThread::~PlayThread() {
  this->mutex.lock();
  this->abort = true;
  this->audio_stop();
  //this->condition.wakeOne();
  this->mutex.unlock();

  this->wait();

}


bool PlayThread::audio_start() {

  int rc;
  // Open device
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
		      this->sample_rate,
		      this->allow_resampling,
		      250000 ); // 0.25

}

void PlayThread::audio_stop() {

  snd_pcm_drain( this->handle );
  snd_pcm_close( this->handle );
}


void PlayThread::run() {

  long loops;
  int rc;
  int size;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_uframes_t frames;
  unsigned int val;
  int dir;

  frames = 16800;
  unsigned char buffer[frames*1];// 1 bytes (8) * 1 channels

  // Fill buffer
  for( int i = 0; i < sizeof(buffer); ++i ) {
    buffer[i] = this->vol*(1 + sin(i*2*3.1416/this->sample_rate*this->frequency) );
  }


  for( long t = 0; ;++t ) {

    printf("Still going around \n"); 
    printf("Got there.. abort: %d restart: %d \n", this->abort, this->restart );
    this->mutex.lock();
    if( !this->restart ) { printf("WON'T START AGAIN!!\n"); this->mutex.unlock(); t = 0; continue; }
    if( this->abort ) { printf("SHOULD GET OUT!!\n"); this->mutex.unlock(); return; }
    this->mutex.unlock();

    printf("And got more should be writing...\n");
    snd_pcm_writei( this->handle, buffer, frames );
    if( rc == -EPIPE ) {
      printf("Underrun occurred \n");
      snd_pcm_prepare( this->handle );
    } else if( rc < 0 ) {
      printf("Error from writei: %s \n", snd_strerror(rc) );
    } else if( rc != (int)frames ) {
      printf("Short write, write %d frames \n", rc );
    }
    
    
  } 


  return;

}


/**
* @brief Constructor
* @param parent
*/
Theremin::Theremin(QWidget *parent) :
    QMainWindow(parent),
    mTimer(0),
    mNumTimeouts(0),
    ui(new Ui::Theremin) {

    ui->setupUi(this);

    // Setting buttons initial state
    ui->stop_pushButton->setEnabled(false);

    // Button messages
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


    this->playThread = new PlayThread( this );

    // Timer
    mTimer = new QTimer(this);
    // connect( mTimer, SIGNAL(timeout()), this, SLOT(updateTimeDisplay_slot()) );
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


}

