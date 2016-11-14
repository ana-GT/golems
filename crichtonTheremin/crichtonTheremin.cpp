#include "ui_crichtonTheremin.h"
#include "crichtonTheremin.h"
#include <qendian.h>
#include <qmath.h>
#include <stdio.h>

const int gDurationSeconds = 1;
const int gToneSampleRateHz = 261;

UpdateSound::UpdateSound( QObject* parent ) {

}

UpdateSound::~UpdateSound() {

}

void UpdateSound::run() {

    for( long t = 0; ; ++t ) {
        // Buffer thing
        this->mutex.lock();
        this->generateData();
        this->mutex.unlock();
    }

}

void UpdateSound::setParams( const QAudioFormat &_format,
                             qint64 _duration_us,
                             int _sample_rate ) {
    //mutex.lock();

    format = _format;
    duration_us = _duration_us;
    sample_rate = _sample_rate;

    //mutex.unlock();
}

void UpdateSound::copyBuffer( QByteArray &_buffer ) {
  // WHY NOT LOCK WORK HERE?
   // mutex.lock();
    _buffer = this->buffer;
   //mutex.unlock();
}

void UpdateSound::updateFrequency( int _freq ) {
    //mutex.lock();
    this->sample_rate = _freq;
    //mutex.unlock();
}

void UpdateSound::updateVolume( qreal _volume ) {
   this->volume = _volume;
}

/**
 *
 */
void UpdateSound::generateData() {

    //this->buffer.clear();

    const int channelBytes = format.sampleSize()/8;
    const int sampleBytes = format.channelCount() * channelBytes;

     qint64 length = (this->format.sampleRate() * this->format.channelCount() * (this->format.sampleSize() / 8))
                         * this->duration_us / 100000;

     Q_ASSERT(length % sampleBytes == 0);
     Q_UNUSED(sampleBytes) // suppress warning in release builds
     this->buffer.resize(length);
     unsigned char *ptr = reinterpret_cast<unsigned char *>(this->buffer.data());
     int sampleIndex = 0;

     while (length) {
         const qreal x = qSin(2 * M_PI * this->sample_rate * qreal(sampleIndex % this->format.sampleRate()) / this->format.sampleRate())*this->volume;
         for (int i=0; i< this->format.channelCount(); ++i) {
             if (this->format.sampleSize() == 8 && this->format.sampleType() == QAudioFormat::UnSignedInt) {
                 const quint8 value = static_cast<quint8>((1.0 + x) / 2 * 255);
                 *reinterpret_cast<quint8*>(ptr) = value;
             } else if (this->format.sampleSize() == 8 && this->format.sampleType() == QAudioFormat::SignedInt) {
                 const qint8 value = static_cast<qint8>(x * 127);
                 *reinterpret_cast<quint8*>(ptr) = value;
             } else if (this->format.sampleSize() == 16 && this->format.sampleType() == QAudioFormat::UnSignedInt) {
                 quint16 value = static_cast<quint16>((1.0 + x) / 2 * 65535);
                 if (this->format.byteOrder() == QAudioFormat::LittleEndian)
                     qToLittleEndian<quint16>(value, ptr);
                 else
                     qToBigEndian<quint16>(value, ptr);
             } else if (this->format.sampleSize() == 16 && this->format.sampleType() == QAudioFormat::SignedInt) {
                 qint16 value = static_cast<qint16>(x * 32767);
                 if (this->format.byteOrder() == QAudioFormat::LittleEndian)
                     qToLittleEndian<qint16>(value, ptr);
                 else
                     qToBigEndian<qint16>(value, ptr);
             }

             ptr += channelBytes;
             length -= channelBytes;
         }
         ++sampleIndex;
     }


}

/**
 * @brief Constructor
 */
Generator::Generator( const QAudioFormat &format,
                      qint64 duration_us,
                      int sampleRate,
                      QObject* parent ) :
    QIODevice(parent),
    mUpdateSoundThread(0),
    mPos(0) {

    this->mUpdateSoundThread = new UpdateSound( this );

    if( format.isValid() ) {
        this->mUpdateSoundThread->setParams( format, duration_us, sampleRate );
        this->mUpdateSoundThread->start();
    }

}


Generator::~Generator() {

}

void Generator::start() {
  open( QIODevice::ReadOnly );
}

void Generator::stop() {
  mPos = 0;
  this->close();
}

void Generator::updateFrequency( int _freq ) {
  this->mUpdateSoundThread->updateFrequency(_freq);
}

void Generator::updateVolume( qreal _vol ) {
    this->mUpdateSoundThread->updateVolume(_vol);
}



qint64 Generator::readData( char* data, qint64 maxlen ) {

    // Copy data from thread (being constantly generated) to mBuffer
    this->mUpdateSoundThread->copyBuffer( mBuffer );

    qint64 total = 0;
    if( !mBuffer.isEmpty() ) {

      while( maxlen - total > 0 ) {
          const qint64 chunk = qMin( (mBuffer.size() - mPos), maxlen - total );
          memcpy( data + total, mBuffer.constData() + mPos, chunk );
          mPos = (mPos + chunk) % mBuffer.size();
          total += chunk;
      }

    }

    return total;
}

qint64 Generator::writeData( const char* data, qint64 len ) {

    Q_UNUSED(data);
    Q_UNUSED(len);
    return 0;
}

qint64 Generator::bytesAvailable() const  {
 return mBuffer.size() + QIODevice::bytesAvailable();
}

/**
 * @function CrichtonTheremin
 * @brief Constructor
 */
CrichtonTheremin::CrichtonTheremin( QWidget *_parent ) :
  QMainWindow(_parent),
  mAudioOutput(0),
  mDevice( QAudioDeviceInfo::defaultOutputDevice() ),
  mGenerator(0),
  mAccel(0),
  ui( new Ui::CrichtonTheremin ) {

  ui->setupUi(this);

  QObject::connect( ui->start_pushButton, SIGNAL(clicked()),
            this, SLOT(start_slot()) );
  QObject::connect( ui->stop_pushButton, SIGNAL(clicked()),
		    this, SLOT(stop_slot()) );
  QObject::connect( ui->frequency_verticalSlider, SIGNAL(valueChanged(int)),
                    this, SLOT(frequency_slot(int)) );
  QObject::connect( ui->volume_verticalSlider, SIGNAL(valueChanged(int)),
                    this, SLOT(volume_slot(int)) );

  this->initialize_audio();
  this->initialize_sensors();

}

/**
 * @function initialize_audio
 */
void CrichtonTheremin::initialize_audio() {

    mFormat.setSampleRate( 44100 ); // gDataSampleRateHz
    mFormat.setChannelCount(1);
    mFormat.setSampleSize(16);
    mFormat.setCodec("audio/pcm");
    mFormat.setByteOrder(QAudioFormat::LittleEndian);
    mFormat.setSampleType(QAudioFormat::SignedInt);

    QAudioDeviceInfo info( QAudioDeviceInfo::defaultOutputDevice());
    if( !info.isFormatSupported(mFormat) ) {
        printf("Default format not supported, trying to use nearest \n");
        mFormat = info.nearestFormat(mFormat);
    }

    mGenerator = new Generator( mFormat, gDurationSeconds*1e6, gToneSampleRateHz, this );

    this->volume_slot( this->ui->volume_verticalSlider->value() );
    this->frequency_slot( this->ui->frequency_verticalSlider->value() );

    create_audio_output();

}


/**
 * @function initialize_sensors
 */
void CrichtonTheremin::initialize_sensors() {

    mPrevTimestamp = 0;
    for( int i = 0; i < 3; ++i ) {
        mRaw_vel[i] = 0;
        mRaw_pos[i] = 0;
    }

    mWaitTime = 5.0;

    this->mAccel = new QAccelerometer( this );
    this->mAccel->setAccelerationMode( QAccelerometer::User );
    QObject::connect( this->mAccel, SIGNAL(readingChanged()), this, SLOT( update_reading_accel_slot() ) );
    this->mAccel->start();

    this->mTimer.setInterval( 20 ); // ms
    QObject::connect( &this->mTimer, SIGNAL(timeout()),
                      this, SLOT(timeout_slot() ) );

}

/**
 * @function update_reading_accel_slot
 */
void CrichtonTheremin::update_reading_accel_slot() {

    if( mPrevTimestamp == 0 ) { mPrevTimestamp = mAccel->reading()->timestamp(); }

    quint64 dt_int;
    dt_int = mAccel->reading()->timestamp() - mPrevTimestamp;
    mDt = dt_int*0.000001;
    mPrevTimestamp = mAccel->reading()->timestamp();

        for( int i = 0; i < 3; ++i ) {
            mGravity[i] = mAccel_val[i];
        }



    // Read
    mAccel_val[0] = this->mAccel->reading()->x();
    mAccel_val[1] = this->mAccel->reading()->y();
    mAccel_val[2] = this->mAccel->reading()->z();

   // In case old one was zero, put it as new one
    for( int i = 0; i < 3; ++i ) {
        if( mGravity[i] == 0 ) { mGravity[i] = mAccel_val[i]; }
    }



    for( int i = 0; i < 3; ++i ) {
      mRaw_pos[i] += mRaw_vel[i]*mDt + 0.5*mAccel_val[i]*mDt*mDt;
      mRaw_vel[i] += mAccel_val[i]*mDt;
    }



    // In this example, alpha is calculated as t / (t + dT),
      // where t is the low-pass filter's time-constant and
      // dT is the event delivery rate.
    double alpha = 0.9;
    // Isolate the force of gravity with the low-pass filter.
    for( int i = 0; i < 3; ++i ) {
      mGravity[i] = alpha * mGravity[i] + (1 - alpha) * mAccel_val[i];
    }

    double temp;
    // Remove the gravity contribution with the high-pass filter.
    for( int i = 0; i < 3; ++i ) {
      temp = mAccel_val[i] - mGravity[i];
      if( fabs(temp) < 0.1 ) { temp = 0; }
      mAccel_val[i] = temp;
    }

}

void CrichtonTheremin::timeout_slot() {

      char msg[240];
//      sprintf( msg,"[%f] Accel: %.2f %.2f %.2f ", mDt, mRaw_pos[0], mRaw_pos[1], mRaw_pos[2] );
      sprintf(msg,"Accel: %f %f %f ", mRaw_vel[0], mRaw_vel[1], mRaw_vel[2] );
//      sprintf(msg,"Accel: %f %f %f ", mAccel_val[0], mAccel_val[1], mAccel_val[2] );

      this->ui->volume_label->setText(msg);

}



CrichtonTheremin::~CrichtonTheremin() {
}

void CrichtonTheremin::volume_slot( int _ind ) {
   float vol; vol = log10(_ind);
  this->mGenerator->updateVolume( vol );
}

void CrichtonTheremin::frequency_slot( int _ind ) {
  this->mGenerator->updateFrequency(_ind);
}

void CrichtonTheremin::create_audio_output() {

    mAudioOutput = 0;
    mAudioOutput = new QAudioOutput( mDevice, mFormat, this );
}

void CrichtonTheremin::start_slot() {

    mGenerator->start();
    mAudioOutput->start(mGenerator);
    this->mTimer.start();

}

void CrichtonTheremin::stop_slot() {

    mAudioOutput->stop();
    this->mTimer.stop();
}
