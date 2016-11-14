#pragma once

#include <QMainWindow>
#include <QAudioOutput>
#include <QThread>
#include <QMutex>
#include <QAccelerometer>
#include <QTimer>
#include <QAltimeter>

namespace Ui {
  class CrichtonTheremin;
}

/**
 * @class UpdateSound
 */
class UpdateSound : public QThread {
  Q_OBJECT

public:
    UpdateSound( QObject* parent = 0 );
    ~UpdateSound();

    void run() Q_DECL_OVERRIDE;
    void setParams( const QAudioFormat &format,
                    qint64 duration_us,
                    int sampleRate );
    void generateData();
    void copyBuffer( QByteArray &_buffer );
    void updateFrequency( int _freq );
    void updateVolume( qreal _vol );

private:
    QMutex mutex;
    QByteArray buffer;

    QAudioFormat format;
    qint64 duration_us;
    int sample_rate;
    qreal volume;

};


class Generator : public QIODevice {

    Q_OBJECT

public:
    Generator( const QAudioFormat &format,
               qint64 duration_us,
               int sampleRate,
               QObject* parent );
    ~Generator();
    void start();
    void stop();
    void updateFrequency( int _freq );
    void updateVolume( qreal _vol );

    qint64 readData( char* data, qint64 maxlen );
    qint64 writeData( const char* data, qint64 len );
    qint64 bytesAvailable() const;

private:
    UpdateSound* mUpdateSoundThread;

    qint64 mPos;
    QByteArray mBuffer;

};

class CrichtonTheremin : public QMainWindow {

  Q_OBJECT
 public:

  explicit CrichtonTheremin( QWidget *parent = 0 );
  ~CrichtonTheremin();


  void initialize_audio();
  void initialize_sensors();
  void create_audio_output();

private slots:
    void start_slot();
    void stop_slot();
    void volume_slot( int _ind );
    void frequency_slot( int _ind );
    void timeout_slot();
    void update_reading_accel_slot();

 private:
  Ui::CrichtonTheremin *ui;
  
  QAudioOutput* mAudioOutput;
  QAudioFormat mFormat;
  QAudioDeviceInfo mDevice;
  QAltimeter* mAltimeter;

  Generator* mGenerator;

  // Sensors
  QAccelerometer* mAccel;
  QTimer mTimer;
  double mAccel_val[3];
  double mRaw_vel[3];
  double mRaw_pos[3];
  double mGravity[3];
  double mWaitTime;
  double mDt;
  quint64 mPrevTimestamp;
};
