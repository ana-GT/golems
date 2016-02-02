/**
 * @file test2_server.cpp
 */

#include "test2_server.h"
#include "ui_test2_server.h"
#include "communication/msgs/server_msgs.h"
#include <sns.h>

#include <QListWidget>
#include <QListWidgetItem>
#include <global/crichton_global.h>

#include <sstream>

/**
 * @brief Constructor
 * @param parent
 */
Test2_Server::Test2_Server(QWidget *parent) :
    QMainWindow(parent),
    mLocalAdapter(0),
    mServer(0),
    mFile_OBB(0),
    mReply_OBB(0),
    mAchComm_ready_flag(false),
    ui(new Ui::Test2_Server)
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

  // FSA
  mMsg_received_flag = false;
  mTimer = new QTimer(this);
  connect( mTimer, SIGNAL(timeout() ), this, SLOT(poll()) );
 
  // Check for ACH channels and update of status
  setAch_slot();

  // Optional: Show state name
  this->showStateName();
}


/**
 * @function ~Test2_Server
 * @brief Destructor
 */
Test2_Server::~Test2_Server() {
    delete ui;
}


/**
 * @function poll
 * @brief Update polling ach channels and fsa 
 */
void Test2_Server::poll() {

  if( mAchComm_ready_flag ) { poll_chan(); }
  poll_fsa();
}

/**
 * @function poll_chan
 * @brief Check the channels for communication from plan and see modules
 */
void Test2_Server::poll_chan() {

  ach_status_t r;
  size_t frame_size;
  sns_msg_process_1 see_msg;
  sns_msg_process_1 plan_msg;

  //--===============================================
  //-- Poll perception channel (See2Server)
  //--===============================================
  r = ach_get( &mSee2Server_chan, &see_msg,
	       sizeof(see_msg), &frame_size, NULL, ACH_O_LAST );
  
  if( r == ACH_OK &&
      !sns_msg_is_expired( &see_msg.header, NULL ) ) {


    //--------------------------------------------//
    // If we received image from See process
    if( see_msg.msg_type == RCV_IMAGE_MSG ) {

      mImage_filename = std::string( see_msg.data_str );
      char fullname[255];
      sprintf( fullname, "%s/%s", gPicturesPath.c_str(), mImage_filename.c_str() );
      mImage_fullname = std::string( fullname );
      

      mMsg_received_flag = true;
      mMsg_source = SEE_MODULE;
      mMsg_type = see_msg.msg_type;
      
      // Notification
      notify_client(END_SEGMENTATION);
    }
    //--------------------------------------------//
    // If we received confirmation of sending object from See process
    else if( see_msg.msg_type == SENT_OBJ_MSG ) {
      
      this->showMsg( QString(" Confirmed object sent to planner"), DEBUG );

      mMsg_received_flag = true;
      mMsg_source = SEE_MODULE;
      mMsg_type = see_msg.msg_type;

      // Notification
      notify_client(END_SQ_FITTING);
    } 
    
  } // end See2Server

  //--===============================================
  //-- Poll plan channel (Plan2Server)
  //--===============================================
  r = ach_get( &mPlan2Server_chan, &plan_msg,
	       sizeof(plan_msg), &frame_size, NULL, ACH_O_LAST );
  
  if( r == ACH_OK && !sns_msg_is_expired( &plan_msg.header, NULL )) {
    
    if( plan_msg.msg_type == PLAN_PREP_DONE_MSG ) {

      this->showMsg( QString("Confirmed plan prep done"), DEBUG );

      mMsg_received_flag = true;
      mMsg_source = PLAN_MODULE;
      mMsg_type = plan_msg.msg_type;
    }

    else if( plan_msg.msg_type == PLAN_DONE_MSG ) {

      this->showMsg( QString("Confirmed plan is done!"), DEBUG );

      mMsg_received_flag = true;
      mMsg_source = PLAN_MODULE;
      mMsg_type = plan_msg.msg_type;
      // Notification
      notify_client( END_PLAN );
    }

    
  } 


  
}

/**
 * @function notify_client
 */
void Test2_Server::notify_client( int _msg ) {
  QString notify_msg = QString( "NOTIFICATION %1" ).arg(_msg);
  showMsg(notify_msg);
  emit sendMsg_signal( notify_msg );
}

/**
 * @function poll_fsa
 * @brief Update our finite state machine if any messages from Bluetooth or ACH came through
 */
void Test2_Server::poll_fsa() {
  
  if( mMsg_received_flag ) {
    
    // Exception messages
    if( mMsg_type == RESET_MSG ) {
      reset_server();
    }
    else if( mMsg_type == GO_BACK_MSG ) {
       printf("Go back was gotten in poll fsa \n");
       goBack();
    }

    else {
      
      int action = mFsa.evaluate( mMsg_source, mMsg_type );
      if( action != NO_ACTION ) {
	this->do_action( action ); 
      }
    }
    
    // Reset msg flag
    mMsg_received_flag = false;

    // Optional: Show state name for easy debugging
    this->showStateName();
  }


}

void Test2_Server::goBack() {

    ach_status_t r;
    sns_msg_server_1 msg;

    sns_msg_set_time( &msg.header, NULL, 0.2*1e9 );
    msg.task_type = TASK_PICK_UP;
    msg.msg_type = MSG_GO_BACK_PLAN_COMMAND;

    r = ach_put( &mServer2Plan_chan, &msg, sizeof(msg) );
}

/**
 * @function reset_server
 * @brief Cleans event list and set state of FSA back to listening
 */
void Test2_Server::reset_server() {

  // Clean list
  ui->msgs_listWidget->clear();
  // Put state back to listening
  mFsa.resetFSA();
}

/**
 * @function showStateName
 * @brief Show current state name of FSA in a label in the server interface
 */
void Test2_Server::showStateName() {
  ui->state_label->setText( QString(mFsa.getCurrentStateName().c_str()) );
}

/**
 * @function do_action
 * @brief It executes an action from a recently changed state
 */
void Test2_Server::do_action( int _action ) {

  if( _action == SEND_IMAGE_REQUEST ) {    

    ach_status_t r;
    sns_msg_server_1 msg;

    sns_msg_set_time( &msg.header, NULL, 0.2*1e9 );
    msg.task_type = TASK_PICK_UP;
    msg.msg_type = MSG_SEE_GRAB_IMG;

    r = ach_put( &mServer2See_chan, &msg, sizeof(msg) );
    
    if( r == ACH_OK ) {
      notify_client( START_SEGMENTATION );
    } else {
      this->showMsg( QString("Failed requesting image"), ERROR );
    }
  } // End send image request

  else if( _action == SEND_INPUT_REQUEST ) {
 
    QBluetoothTransferRequest req( mService_OBB.device().address() );
    mFile_OBB = new QFile( QString(mImage_fullname.c_str()) );
    mReply_OBB = mMgr.put( req, mFile_OBB );

    connect( mReply_OBB, SIGNAL(finished(QBluetoothTransferReply*)),
	     this, SLOT(OB_finishedTransfer_slot(QBluetoothTransferReply*)));
    
    
  } // End sending client input request

  else if( _action == SEND_PREP_REQUEST_PLAN ) {    

    ach_status_t r;
    sns_msg_server_1 msg;

    sns_msg_set_time( &msg.header, NULL, 0.2*1e9 );
    msg.task_type = TASK_PICK_UP;
    msg.msg_type = MSG_PLAN_PREP_REQUEST;

    r = ach_put( &mServer2Plan_chan, &msg, sizeof(msg) );
    
    if( r == ACH_OK ) {
      showMsg( QString("Plan prep request sent correctly"), DEBUG );
    } else {
      showMsg( QString("Requesting plan prep"), ERROR );
    }
  } // End send image request

  else if( _action == SEND_OBJ_COMMAND ) {    
    printf("Sending object command!!!!!! \n");
    ach_status_t r;
    sns_msg_server_1 msg;

    sns_msg_set_time( &msg.header, NULL, 0.2*1e9 );
    msg.task_type = TASK_PICK_UP;
    msg.msg_type = MSG_SEND_OBJ_COMMAND;
    msg.data_int1 = mPx;
    msg.data_int2 = mPy;
    r = ach_put( &mServer2See_chan, &msg, sizeof(msg) );
    
    if( r == ACH_OK ) {
      showMsg( QString("Sent obj command sent correctly"), DEBUG );
      notify_client( START_SQ_FITTING );
    } else {
      showMsg( QString("Sending obj command"), ERROR );
    }
  } // End send obj command


    else if( _action == START_PLANNING_COMMAND ) {    

    ach_status_t r;
    sns_msg_server_1 msg;

    sns_msg_set_time( &msg.header, NULL, 0.2*1e9 );
    msg.task_type = TASK_PICK_UP;
    msg.msg_type = MSG_START_PLAN_COMMAND;

    r = ach_put( &mServer2Plan_chan, &msg, sizeof(msg) );
    
    if( r == ACH_OK ) {
      notify_client( START_PLAN );
    } else {
      showMsg( QString("Starting plan"), ERROR );
    }
  } // End send image request

  
  
}

/**
 *
 */
void Test2_Server::OB_finishedTransfer_slot( QBluetoothTransferReply*) {

  showMsg( QString("Sent image to client"), DEBUG );
  char msg[255];
  sprintf(msg, "RCV_IMG %s", mImage_filename.c_str() );
  
  emit sendMsg_signal( msg );
}



/**
 * @function setAch_slot
 */
void Test2_Server::setAch_slot() {

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
    this->showMsg("Ach channels NOT opened correctly", ERROR);
  } else {
    this->showMsg("Ach channels set up correctly", INFO );
    mAchComm_ready_flag = true;
  }
  
}

void Test2_Server::stopComm_slot() {
    // Stop server
    mServer->stopServer();
}

/**
 * @function showPlainMsg
 * @brief Show messages with additional identification
 */
void Test2_Server::showMsg( const QString &_msg, int _option ) {

   QString display_msg;

   switch( _option ) {
   case NOTHING:
     display_msg = QString( _msg );
     break;
   case DEBUG:
     display_msg = QString("DEBUG: %1").arg(_msg);
     break;
   case CLIENT_NEWS:
     display_msg = QString("CLIENT NEWS: %1").arg(_msg);
     break;
   case SERVER_COMMAND:
     display_msg = QString("SERVER COMMAND: %1").arg(_msg);
     break;
   case SERVER_NOTIFICATION:
     display_msg = QString("SERVER NEWS: %1").arg(_msg);
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
 * @function dbgMsg_slot
 */
void Test2_Server::dbgMsg_slot( const QString &_msg ) {
  showMsg( _msg, DEBUG );
}


/**
 * @function rcvMsg_slot
 */
void Test2_Server::rcvMsg_slot( const QString &_sender, const QString &_message )  {

  //-- SHOW MSG
  QString message = QString("%1 : %2").arg( _sender, _message );
  new QListWidgetItem( message, ui->msgs_listWidget );

  //-- DECODE MSG
  std::string msg_type;
  std::istringstream iss( _message.toStdString() );
  iss >> msg_type;

  //-- CLIENT: START
  if( msg_type.compare( "START_MSG") == 0 ) {

    mMsg_received_flag = true;
    mMsg_source = CLIENT;
    mMsg_type = START_MSG;
    
  } else if( msg_type.compare( "RCV_INPUT_MSG" ) == 0 ) { 

    iss >> mPx; iss >> mPy;
    printf("Received input from client \n");
    mMsg_received_flag = true;
    mMsg_source = CLIENT;
    mMsg_type = RCV_INPUT_MSG;
    
  } else if( msg_type.compare("RESET_MSG") == 0 ) {

    mMsg_received_flag = true;
    mMsg_source = CLIENT;
    mMsg_type = RESET_MSG;
    
  } else if( msg_type.compare("GO_BACK_MSG") == 0 ) {
    dbgMsg_slot(QString("GO BACK WAS RECEIVED IN ELSE IF CORRECTLY" ));
    mMsg_received_flag = true;
    mMsg_source = CLIENT;
    mMsg_type = GO_BACK_MSG;
  }
    
}

/**
 * @function clientConnected_slot
 * @brief Found client, we store its address
 */
void Test2_Server::clientConnected_slot( const QString &_name ) {

    QString msg =QString( "%1: Connected to this server").arg(_name);
    new QListWidgetItem( msg, ui->msgs_listWidget );

    // Start searching for OBB
    mDiscoveryAgent = new QBluetoothServiceDiscoveryAgent( mLocalAdapter );      
    QObject::connect( mDiscoveryAgent, SIGNAL(serviceDiscovered(QBluetoothServiceInfo)),
                      this, SLOT(OB_discovered_slot(QBluetoothServiceInfo)) );
    QObject::connect( mDiscoveryAgent, SIGNAL(finished()),
                      this, SLOT(OB_discoverEnd_slot()) );
    QObject::connect( mDiscoveryAgent, SIGNAL(canceled()),
                      this, SLOT(OB_discoverEnd_slot()) );
    
    mDiscoveryAgent->setUuidFilter( QBluetoothUuid(QBluetoothUuid::ObexObjectPush) );
    mDiscoveryAgent->setRemoteAddress(QBluetoothAddress("A4:9A:58:AA:9D:9D"));
    mDiscoveryAgent->start();
    
    
    /*****************************/
    mMsg_received_flag = true;
    mMsg_source = CLIENT;
    mMsg_type = CONNECTED_MSG;

    mTimer->start(100);
    /*****************************/
}


/**
 * @function OB_discovered_slot
 */
void Test2_Server::OB_discovered_slot( QBluetoothServiceInfo _service ) {
  mService_OBB = _service;
  dbgMsg_slot(QString("OB service discovered"));
  mDiscoveryAgent->stop();
}


/**
 * @brief clientDisconnected_slot
 */
void Test2_Server::clientDisconnected_slot( const QString &_name ) {

    QString msg =QString( "%1: Disconnected to this server").arg(_name);
    new QListWidgetItem( msg, ui->msgs_listWidget );
}

/**
 * @brief sendMsg_slot
 */
void Test2_Server::sendMsg_slot() {

   QObject* obj = sender();

   if( obj == ui->maydayTest_pushButton ) {
     new QListWidgetItem( tr("Sent: Mayday test"), ui->msgs_listWidget );
     emit sendMsg_signal( "Mayday test" );
   }

}
