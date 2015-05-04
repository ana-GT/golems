/**
 * @file test1_server.cpp
 */

#include "test1_server.h"
#include "ui_test1_server.h"
#include <QListWidget>
#include <QListWidgetItem>
#include <global/crichton_global.h>

#include <sstream>

/**
 * @brief Constructor
 * @param parent
 */
Test1_Server::Test1_Server(QWidget *parent) :
    QMainWindow(parent),
    mLocalAdapter(0),
    mServer(0),
    ui(new Ui::Test1_Server)
{
    ui->setupUi(this);

    // Button messages
    QObject::connect( ui->maydayTest_pushButton, SIGNAL(clicked()),
                      this, SLOT(sendMsg_slot()) );
    QObject::connect( ui->stopComm_pushButton, SIGNAL(clicked()),
                      this, SLOT(stopComm_slot()) );
    QObject::connect( ui->setAch_pushButton, SIGNAL(clicked()),
                      this, SLOT(setAch_slot()) );

    // Get local adapter
    QList<QBluetoothHostInfo> adapters;
    adapters = QBluetoothLocalDevice::allDevices();
    mLocalAdapter = new QBluetoothLocalDevice( adapters.at(0).address() );
    mLocalAdapter->setHostMode( QBluetoothLocalDevice::HostDiscoverable);

    // Create server
    mServer = new server_unit(this);

    this->connect( mServer, SIGNAL(clientConnected_signal(QString)),
                   this, SLOT( clientConnected_slot(QString)) );
    this->connect( mServer, SIGNAL(clientDisconnected_signal(QString)),
                   this, SLOT( clientDisconnected_slot(QString)) );
    this->connect( mServer, SIGNAL(rcvMsg_signal(QString, QString)),
                   this, SLOT( rcvMsg_slot(QString, QString)));
    this->connect( this, SIGNAL(sendMsg_signal(QString)),
                   mServer, SLOT(sendMsg_slot(QString)) );

    this->connect( mServer, SIGNAL(dbgMsg_signal(QString)),
                   this, SLOT(dbgMsg_slot(QString)) );


    // Start server
    mServer->startServer(mLocalAdapter->address() );

    // Get local device name
    mLocalName = QBluetoothLocalDevice().name();

}

/**
 * @function setAch_slot
 */
void Test1_Server::setAch_slot() {

  // Open
  enum ach_status r;
  bool error_opening_chan = false;
  
  r = ach_open( &mServer2See_chan, gServer2See_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { error_opening_chan = true;}
  r = ach_open( &mSee2Server_chan, gSee2Server_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { error_opening_chan = true;}
  r = ach_open( &mServer2Plan_chan, gServer2Plan_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { error_opening_chan = true;}
  r = ach_open( &mPlan2Server_chan, gPlan2Server_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { error_opening_chan = true;}

  if( error_opening_chan ) {
    dbgMsg_slot("[DBG ERROR] Ach channels not opened correctly");
  } else {
    dbgMsg_slot("[DBG] Ach channels set up correctly");
  }
  
}

void Test1_Server::stopComm_slot() {
    // Stop server
    mServer->stopServer();
}

/**
 * @function dbgMsg_slot
 */
void Test1_Server::dbgMsg_slot( const QString &_msg ) {

    new QListWidgetItem( _msg, ui->msgs_listWidget );
}


Test1_Server::~Test1_Server() {
    delete ui;
}

/**
 * @function rcvMsg_slot
 */
void Test1_Server::rcvMsg_slot( const QString &_sender, const QString &_message )  {

  // Process msg
  // Decode the message
  std::string msg = _message.toStdString();
  std::istringstream iss(msg);
  
  std::string command, side, goalConf;
  int command_int, side_int, goalConf_int;
  iss >> command;  iss >> side;  iss >> goalConf;

  int achMsg[3];

  if( command.compare("start") == 0 ) { achMsg[0] = START; }
  else if( command.compare("stop") == 0 ) { achMsg[0] = STOP; }

  if( side.compare("left") == 0 ) { achMsg[1] = LEFT_MSG; }
  else if( side.compare("right") == 0 ) { achMsg[1] = RIGHT_MSG; }

  if( goalConf.compare("resting") == 0 ) { achMsg[2] = RESTING; }
  else if( goalConf.compare("fist") == 0 ) { achMsg[2] = FRONTAL_FIST; }
  else if( goalConf.compare("lookup") == 0 ) { achMsg[2] = LOOKUP_SKY; }
  else if( goalConf.compare("arcup") == 0 ) { achMsg[2] = ARC_UP; }

  // Create message and send
  ach_status_t r;
  r = ach_put( &mServer2Plan_chan, achMsg, sizeof(achMsg) );
  if( r != ACH_OK ) {
    dbgMsg_slot( QString("[BAD] Error sending message") );
  } else {
    dbgMsg_slot( QString("[GOOD] Message sent all right") );
  }
  
  // Show msg
  QString message = QString("%1 : %2").arg( _sender, _message );
  
  new QListWidgetItem( message, ui->msgs_listWidget );
}

/**
 * @brief clientConnected_slot
 */
void Test1_Server::clientConnected_slot( const QString &_name ) {

    QString msg =QString( "%1: Connected to this server").arg(_name);
    new QListWidgetItem( msg, ui->msgs_listWidget );
}

/**
 * @brief clientDisconnected_slot
 */
void Test1_Server::clientDisconnected_slot( const QString &_name ) {

    QString msg =QString( "%1: Disconnected to this server").arg(_name);
    new QListWidgetItem( msg, ui->msgs_listWidget );
}

/**
 * @brief sendMsg_slot
 */
void Test1_Server::sendMsg_slot() {

   QObject* obj = sender();

   if( obj == ui->maydayTest_pushButton ) {
     new QListWidgetItem( tr("Sent: Mayday test"), ui->msgs_listWidget );
     emit sendMsg_signal( "Mayday test" );
   }

}
