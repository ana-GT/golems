#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <stdio.h>
//#include <alsa/asoundlib.h>
#include <tinyalsa/asoundlib.h>

namespace Ui {
    class Theremin;
}



/**
 * @class PlayThread
 */
class PlayThread : public QThread {

  Q_OBJECT

 public:
  PlayThread( QObject* parent = 0 );
  ~PlayThread();


  void stop_thread() { this->restart = false; }
  void start_thread() { 
    printf("Start thread \n");
    if( !isRunning() ) {   printf("Start thread \n"); this->start(); }
    printf("Mutexes...\n");
    this->mutex.lock();
    this->abort = false;
    this->restart = true;
    this->mutex.unlock();
  }

  void setVolume( double _volume );
  void set_tone( double _freq, 
		 double _vol );
 protected:
  void run() Q_DECL_OVERRIDE;
  bool audio_start();
  void audio_stop();

 private:
  QMutex mutex;
  bool restart;
  bool abort;

  // Thread stuff
  double vol;
  double fs; // Hz sample rate
  double ft; // Frequency
  //snd_pcm_t* handle; // device
  struct pcm* handle;
  int channels;
  int allow_resampling;

  double freq_min;
  double freq_max;
  

};

/**
 * @class Theremin
 */
class Theremin : public QMainWindow {

    Q_OBJECT

public:
    explicit Theremin(QWidget *parent = 0);
    ~Theremin();


signals:


private slots:
    void play_slot();
    void stop_slot();
    void scale_slot( int _id );
    void output_mode_slot( int _id );
    void volume_slot( int _in );
    void frequency_slot( int _in );

private:
    Ui::Theremin *ui;
    PlayThread* playThread;

 public:
    // Static
    double static sSharp;
    double  static sEqual_temp_freqs[12];
    std::string static sEqual_temp_labels[12];
    int static sDiatonic_major_intervals[7];
    int static sPentatonic_major_intervals[5];
    int static sPentatonic_minor_intervals[5];
    int static blues_intervals[6];

};
