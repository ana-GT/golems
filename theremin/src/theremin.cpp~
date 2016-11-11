/**
 * @file client_pap.cpp
 * @brief Client for pickup tasks
 */

#include "client_pap.h"
#include "ui_client_pap.h"
#include "connectDialog.h"
#include "msgs/enum_names.h"
#include <iostream>
#include <sstream>
#include <string>

/**
* @brief Constructor
* @param parent
*/
Client_PaP::Client_PaP(QWidget *parent) :
    QMainWindow(parent),
    mLocalAdapter(0),
    mClient(0),
    mTimer(0),
    mNumTimeouts(0),
    ui(new Ui::Client_PaP) {

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
        this->showPlainMsg( QString("No Bluetooth adapter found"), ERROR);
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
Client_PaP::~Client_PaP() {
}


/**
 * @function sendMsg_slot
 */
void Client_PaP::sendMsg_slot() {

    QObject* obj = sender();
    bool send = false;

    // Get details from combo boxes
    QString command;
    QString side = ui->side_comboBox->currentText();
    
    if( obj == ui->start_pushButton ) {
      command = QString("%1 %2").arg(START_SOLUTION_MSG).arg(1);
      mTimer->start(100);
      send = true;

    } else if( obj == ui->stop_pushButton ) {
      //presentReport();
      mTimer->stop();

    } else if( obj == ui->reset_pushButton ) {
      command = QString("%1 %2").arg( RESET_UI_MSG).arg(1);
        this->reset_client();
        send = true;

    } else if( obj == ui->goBack_pushButton ) {
      command = QString("%1 %2").arg( START_SOLUTION_BACK_MSG, 1 );
      send = true;
    }

    if( send == true ) {
      emit sendMsg_signal( command );
            this->showPlainMsg( command, CLIENT_COMMAND );
    }
    
}

/**
 * @function reset_client
 */
void Client_PaP::reset_client() {

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
void Client_PaP::connect_slot(){

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
        this->showPlainMsg( msg, SERVER_NOTIFICATION );

        ui->disconnect_pushButton->setEnabled(true);
        mClient->startClient(service);

    } else {
        this->showPlainMsg( QString("NO server selected"), ERROR );
    }
}


/**
 * @function showPlainMsg
 * @brief Show messages with additional identification
 */
void Client_PaP::showPlainMsg( const QString &_msg, int _option ) {

   QString display_msg;

   switch( _option ) {
       case NOTHING:
        display_msg = QString( _msg );
        break;
   case SERVER_NOTIFICATION:
       display_msg = QString("SERVER NEWS: %1").arg(_msg);
       break;
   case DEBUG:
       display_msg = QString("DEBUG: %1").arg(_msg);
       break;
   case CLIENT_NEWS:
       display_msg = QString("CLIENT NEWS: %1").arg(_msg);
       break;
   case CLIENT_COMMAND:
       display_msg = QString("CLIENT COMMAND: %1").arg(_msg);
       break;
   case SERVER_COMMAND:
       display_msg = QString("SERVER COMMAND: %1").arg(_msg);
       break;
   case ERROR:
       display_msg = QString("ERROR: %1").arg(_msg);
       break;
   case INFO:
       display_msg = QString("INFO: %1").arg(_msg);
       break;
   default:
       display_msg = QString("NO-OPTION: %1").arg(_msg);
       break;
   }

  new QListWidgetItem( display_msg, ui->msgs_listWidget );
}

/**
 * @function rcvMsg
 * @brief Receive message from server through Bluetooth Socket
 */
void Client_PaP::rcvMsg_slot( const QString &_sender,
                                const QString &_message ) {


  //-- DECODE MSG
  std::string msg_type;
  std::string param1;
  std::istringstream iss( _message.toStdString() );
  iss >> msg_type;
  iss >> param1;

  // Info msg
  int type = atoi(msg_type.c_str());
  if( type == INFO_MSG ) {
    this->showPlainMsg( _message, SERVER_NOTIFICATION );
  }
  
}

/**
 * @function presentReport
 */
void Client_PaP::presentReport() {

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
    this->showPlainMsg( reportMsg, CLIENT_NEWS);

}

/**
* @function clientDisconnected_slot
* @brief Something probably happened on the server side, the connection died.
*/
void Client_PaP::clientDisconnected_slot() {

   if( mClient ) {
        mClient->deleteLater();
   }

   showPlainMsg("Connection to server disconnected", ERROR);
}


/**
 * @funcion dbgMsg_slot
 * @brief Show messages for debugging purposes. No vital
 */
void Client_PaP::dbgMsg_slot( const QString &_msg ) {
    showPlainMsg( _msg, DEBUG );
}

/**
 * @function updateTimeDisplay_slot
 * @brief Update time showed to client for user to keep track
 */
void Client_PaP::updateTimeDisplay_slot() {

   mNumTimeouts++;
   // Tenths of second show
       double value; value = mNumTimeouts / 10.0;
       ui->time_lcdNumber->display( value );
}

