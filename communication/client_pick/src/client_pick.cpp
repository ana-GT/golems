/**
 * @file client_pick.cpp
 * @brief Client for pickup tasks
 */

#include "client_pick.h"
#include "ui_client_pick.h"
#include "connectDialog.h"
#include <iostream>
#include <sstream>
#include <string>

#include "selectObjectDialog.h"


/**
* @brief Constructor
* @param parent
*/
Client_Pick::Client_Pick(QWidget *parent) :
    QMainWindow(parent),
    mLocalAdapter(0),
    mClient(0),
    mTimer(0),
    mNumTimeouts(0),
    ui(new Ui::Client_Pick) {

    ui->setupUi(this);

    // Setting buttons initial state
    ui->disconnect_pushButton->setEnabled(false);

    // Button messages
    QObject::connect( ui->connect_pushButton, SIGNAL(clicked()),
                      this, SLOT(connect_slot()) );
    QObject::connect( ui->disconnect_pushButton, SIGNAL(clicked()),
                      this, SLOT(clientDisconnected_slot()) );
    QObject::connect( ui->start_pushButton, SIGNAL(clicked()),
                      this, SLOT(sendMsg_slot()) );
    QObject::connect( ui->stop_pushButton, SIGNAL(clicked()),
                      this, SLOT(sendMsg_slot()) );
    QObject::connect( ui->reset_pushButton, SIGNAL(clicked()),
                      this, SLOT(sendMsg_slot()) );
    QObject::connect( ui->goBack_pushButton, SIGNAL(clicked()),
                      this, SLOT(sendMsg_slot()) );
    QObject::connect( ui->cancel_pushButton, SIGNAL(clicked()),
                      this, SLOT(close()) );

    // Get local device
    QList<QBluetoothHostInfo> adapters = QBluetoothLocalDevice::allDevices();
    if ( adapters.count() < 1) {
        ui->connect_pushButton->setEnabled(false);
        this->showPlainMsg( QString("No Bluetooth adapter found") );
        return;
    }
    mLocalAdapter = new QBluetoothLocalDevice( adapters.at(0).address() );
    mLocalAdapter->setHostMode( QBluetoothLocalDevice::HostDiscoverable);

    // Timer
    mTimer = new QTimer(this);
    connect( mTimer, SIGNAL(timeout()), this, SLOT(updateTimeDisplay_slot()) );
}

/**
* @brief Destructor
* @param parent
*/
Client_Pick::~Client_Pick() {
}


/**
 * @function sendMsg_slot
 */
void Client_Pick::sendMsg_slot() {

  QObject* obj = sender();
  bool send = false;
  
  // Get details from combo boxes
  QString command;
  QString side = ui->side_comboBox->currentText();
  
  if( obj == ui->start_pushButton ) {
    command = QString("%1").arg(PICKUP_MSG_CMD_SERVER_START);
    mTimer->start(100);
    send = true;
    
  } else if( obj == ui->stop_pushButton ) {
    //presentReport();
    mTimer->stop();
    
  } else if( obj == ui->reset_pushButton ) {
    command = QString("%1").arg( PICKUP_MSG_CMD_SERVER_RESET);
    this->reset_client();
    send = true;
    
  } else if( obj == ui->goBack_pushButton ) {
    command = QString("%1").arg( PICKUP_MSG_CMD_SERVER_GO_BACK );
    send = true;
  }
  
  if( send == true ) {
      emit sendMsg_signal( command );
      this->showPlainMsg( command );
  }
  
}

/**
 * @function reset_client
 */
void Client_Pick::reset_client() {

    // Stop timer and set it to zero
    mTimer->stop();
    mNumTimeouts = 0;
    ui->time_lcdNumber->display( 0.0 );

    // Clean the message list except for the connection
    ui->msgs_listWidget->clear();

}

/**
* @function connect
*/
void Client_Pick::connect_slot(){

    connectDialog mCd( mLocalAdapter->address(), this );

    if( mCd.exec() == QDialog::Accepted ) {
        QBluetoothServiceInfo service = mCd.service();
        mClient = new client_unit(this);

        QObject::connect( mClient, SIGNAL( rcvMsg_signal(QString,QString)),
                          this, SLOT( rcvMsg_slot( QString, QString)));
        QObject::connect( mClient, SIGNAL( disconnected_signal()),
                          this, SLOT( clientDisconnected_slot()));
        QObject::connect( this, SIGNAL( sendMsg_signal(QString)),
                          mClient, SLOT(sendMsg_slot(QString)));
        QObject::connect( mClient, SIGNAL( dbgMsg_signal(QString)),
                          this, SLOT(dbgMsg_slot(QString)));

        QString msg = QString(" Connection established (Server: %1)").arg( service.serviceName() );
        this->showPlainMsg( msg );

        ui->disconnect_pushButton->setEnabled(true);
        mClient->startClient(service);

    } else {
        this->showPlainMsg( QString("NO server selected") );
    }
}


/**
 * @function showPlainMsg
 * @brief Show messages with additional identification
 */
void Client_Pick::showPlainMsg( const QString &_msg ) {

  new QListWidgetItem( _msg, ui->msgs_listWidget );
}

/**
 * @function rcvMsg
 * @brief Receive message from server through Bluetooth Socket
 */
void Client_Pick::rcvMsg_slot( const QString &_sender,
                                const QString &_message ) {


  //-- DECODE MSG
  int type;
  std::string line;
  std::istringstream iss( _message.toStdString() );
  iss >> type;
  iss >> line;

  // Info msg
  switch( type ) {

  case PICKUP_MSG_CMD_CLIENT_NOTIFY:
    this->showPlainMsg( _message );
    break;  
  
  case PICKUP_MSG_CMD_CLIENT_CHOOSE_IMG:

    char localFile[255];
    sprintf( localFile, "/storage/emulated/0/Bluetooth/%s", line.c_str() );
    showPlainMsg( QString("Got image: %1").arg(localFile) );
    
    SelectObjectDialog so( this);
    so.loadFile( QString(localFile) );
    
    if( so.exec() == QDialog::Accepted ) {
      so.getClicked( mPx, mPy );

      QString ack_msg = QString("%1 %2 %3").arg(PICKUP_MSG_STATUS_CLIENT_CHOOSE_YES).arg( mPx ).arg( mPy );
      emit sendMsg_signal( ack_msg );
      showPlainMsg( QString("Pixels: %1 %2").arg(mPx).arg(mPy) );
    } else {
      QString ack_msg = QString("%1").arg(PICKUP_MSG_STATUS_CLIENT_CHOOSE_NO);
      emit sendMsg_signal( ack_msg );
    }
        
    break;
  }
}

/**
 * @function presentReport
 */
void Client_Pick::presentReport() {

    double totalTime = ui->time_lcdNumber->value();
     // Segmentation time:
    double segmentation_time = mTimes[1] - mTimes[0];
    // Fitting time:
    double fitting_time = mTimes[3] - mTimes[2];
    // Planning time:
    double planning_time = mTimes[5] - mTimes[4];
    // Execution time:
    double execution_time = totalTime - mTimes[5];

    QString reportMsg = QString( "Seg: %1, Fit: %2, Plan: %3, Exec: %4. TOTAL=%5").arg(segmentation_time).arg(fitting_time).arg(planning_time).arg(execution_time).arg(totalTime);
    this->showPlainMsg( reportMsg );

}

/**
* @function clientDisconnected_slot
* @brief Something probably happened on the server side, the connection died.
*/
void Client_Pick::clientDisconnected_slot() {

   if( mClient ) {
        mClient->deleteLater();
   }

   showPlainMsg("Connection to server disconnected");
}


/**
 * @funcion dbgMsg_slot
 * @brief Show messages for debugging purposes. No vital
 */
void Client_Pick::dbgMsg_slot( const QString &_msg ) {
    showPlainMsg( _msg );
}

/**
 * @function updateTimeDisplay_slot
 * @brief Update time showed to client for user to keep track
 */
void Client_Pick::updateTimeDisplay_slot() {

   mNumTimeouts++;
   // Tenths of second show
       double value; value = mNumTimeouts / 10.0;
       ui->time_lcdNumber->display( value );
}

