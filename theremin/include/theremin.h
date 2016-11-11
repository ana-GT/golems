#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <alsa/asoundlib.h>


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
  double sample_rate;
  double frequency;
  snd_pcm_t* handle; // device
  int channels;
  int allow_resampling;
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

private:
    Ui::Theremin *ui;

    double mTimes[6];

    int mPx; int mPy;
    QTimer* mTimer;
    int mNumTimeouts;

    PlayThread* playThread;

};
