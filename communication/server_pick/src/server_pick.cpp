/**
 * @file server_pick.cpp
 */

#include "server_pick.h"
#include "ui_server_pick.h"
#include "msgs/server_msgs.h"
#include <sns.h>

#include <QListWidget>
#include <QListWidgetItem>
#include <global/crichton_global.h>

#include <sstream>

/**
 * @brief Constructor
 * @param parent
 */
Server_Pick::Server_Pick(QWidget *parent) :
    QMainWindow(parent),
    mLocalAdapter(0),
    mServer(0),
    mFile_OBB(0),
    mReply_OBB(0),
    mComm_ready(false),
    mClient_got_msg_fsm(false),
    mState_fsm(PICKUP_STATE_NONE),
    ui(new Ui::Server_Pick)
{
  ui->setupUi(this);
  
  // Button messages
  QObject::connect( ui->maydayTest_pushButton, SIGNAL(clicked()),
		    this, SLOT(send_test_msg_slot()) );
  QObject::connect( ui->setComm_pushButton, SIGNAL(clicked()),
		    this, SLOT(set_comm_slot()) );
  QObject::connect( ui->stopComm_pushButton, SIGNAL(clicked()),
		    this, SLOT(stop_server_slot()) );


  // Server
  mServer = new server_unit(this);
    
  this->connect( this, SIGNAL(send_client_msg_signal(QString)),
		 mServer, SLOT(send_client_msg_slot(QString)) );
  this->connect( mServer, SIGNAL(client_connected_signal(QString)),
		 this, SLOT( client_connected_slot(QString)) );
  this->connect( mServer, SIGNAL(client_disconnected_signal(QString)),
		 this, SLOT( client_disconnected_slot(QString)) );
  this->connect( mServer, SIGNAL(rcv_client_msg_signal(QString, QString)),
		 this, SLOT( rcv_client_msg_slot(QString, QString))); 
  this->connect( mServer, SIGNAL(send_server_msg_signal(QString)),
		 this, SLOT(show_server_msg_slot(QString)) );
  
  
  // Get local adapter
  QList<QBluetoothHostInfo> adapters;
  adapters = QBluetoothLocalDevice::allDevices();
  mLocalAdapter = new QBluetoothLocalDevice( adapters.at(0).address() );
  mLocalAdapter->setHostMode( QBluetoothLocalDevice::HostDiscoverable);
  
  
  // Start server
  mServer->start_server(mLocalAdapter->address() );
  
  // Get local device name
  mLocalName = QBluetoothLocalDevice().name();
  
  mTimer = new QTimer(this);
  connect( mTimer, SIGNAL(timeout() ), this, SLOT(listen_fsm()) );
 
  // Check for ACH channels and update of status
  set_comm_slot();

  // Optional: Show state name
  this->show_state_name_fsm();
}


/**
 * @function ~Server_Pick
 * @brief Destructor
 */
Server_Pick::~Server_Pick() {
    delete ui;
}

/**
 * @function show_server_msg_slot
 */
void Server_Pick::show_server_msg_slot( const QString &_msg ) {
  new QListWidgetItem( _msg, ui->msgs_listWidget );
}

/**
 * @brief send_test_msg_slot
 */
void Server_Pick::send_test_msg_slot() {

   QObject* obj = sender();

   new QListWidgetItem( tr("Sent: Test msg 'Mayday'"), ui->msgs_listWidget );
   
   char* text_msg = new char[200];
   sprintf( text_msg, "%d %s", PICKUP_MSG_TEXT, "Mayday test msg");
   emit send_client_msg_signal( QString(text_msg) );  
}

/**
 * @function set_comm_slot
 * @brief Set communication channels for the FSM
 */
void Server_Pick::set_comm_slot() {

  mComm_ready = true;
  if( ach_open( &mServer2Module_chan, gServer2Module_chan_name.c_str(), NULL ) != ACH_OK ) {
    mComm_ready = false;
  }
  if( ach_open( &mModule2Server_chan, gModule2Server_chan_name.c_str(), NULL ) != ACH_OK ) {
    mComm_ready = false;
  }
  
  ach_flush( &mServer2Module_chan );
  ach_flush( &mModule2Server_chan );

  if( mComm_ready ) { show_server_msg_slot( QString("* Comm is OK") ); }
  else { show_server_msg_slot( QString("* Comm is NOT SET") ); }
  
}

/**
 * @function stop_server_slot
 * @brief Stop Bluetooth communication with client using app
 */
void Server_Pick::stop_server_slot() {
    mServer->stop_server();
}

/**
 * @function clientConnected_slot
 * @brief Found client, we store its address
 */
void Server_Pick::client_connected_slot( const QString &_name ) {

    QString msg =QString( "* %1: Connected to this server").arg(_name);
    new QListWidgetItem( msg, ui->msgs_listWidget );

    // Start searching for OBB
    mDiscoveryAgent = new QBluetoothServiceDiscoveryAgent( mLocalAdapter );      
    QObject::connect( mDiscoveryAgent, SIGNAL(serviceDiscovered(QBluetoothServiceInfo)),
                      this, SLOT(OB_discovered_slot(QBluetoothServiceInfo)) );
    mDiscoveryAgent->setUuidFilter( QBluetoothUuid(QBluetoothUuid::ObexObjectPush) );
    mDiscoveryAgent->setRemoteAddress(QBluetoothAddress("A4:9A:58:AA:9D:9D"));
    mDiscoveryAgent->start();
    
    mTimer->start(100); /**< Very important! */

}

/**
 * @brief clientDisconnected_slot
 */
void Server_Pick::client_disconnected_slot( const QString &_name ) {
    QString msg =QString( "* %1: Disconnected to this server").arg(_name);
    new QListWidgetItem( msg, ui->msgs_listWidget );
}


/**
 * @function OB_discovered_slot
 * @brief App and OB functionality is discovered so send message from server to FSM we are ready
 */
void Server_Pick::OB_discovered_slot( QBluetoothServiceInfo _service ) {
  mService_OBB = _service;
  show_server_msg_slot(QString("* OB service discovered!"));
  mDiscoveryAgent->stop();

  // FSM msg indicating we are ready to start
  mMsg_fsm.type = PICKUP_MSG_CMD_SERVER_CONNECTED;
  sprintf( mMsg_fsm.line, "" );
  mClient_got_msg_fsm = true;
}

/**
 * @function rcv_client_msg_slot
 * @brief Receive message from the client using its app through Bluetooth
 */
void Server_Pick::rcv_client_msg_slot( const QString &_sender, const QString &_message )  {

  // Set client msg
  mClient_got_msg_fsm = true;

  std::stringstream ss( _message.toStdString() );
  ss >> mMsg_fsm.type;
  sprintf( mMsg_fsm.line, "%s", _message.toStdString().c_str() );
}


/**
 * @function reset_server
 * @brief Cleans event list and set state of FSA back to listening
 */
void Server_Pick::reset_server() {

  // Clean list
  ui->msgs_listWidget->clear();

  // Put state back to listening
  this->reset_fsm();
}



/**
 * @function OB_finishedTransfer_slot
 */
void Server_Pick::OB_finishedTransfer_slot( QBluetoothTransferReply*) {

  show_server_msg_slot( QString("Sending image to client") );
  char msg[200];
  sprintf(msg, "%d %s", PICKUP_MSG_CMD_CLIENT_CHOOSE_IMG, mImg_filename.c_str() );  
  emit send_client_msg_signal( msg );
}


///////////////////////////////////////////////////////////////////

/**
 * @function showStateName
 * @brief Show current state (from FSA) in a label in the server interface
 */
void Server_Pick::show_state_name_fsm() {

  std::string name;
  
  switch( mState_fsm ) {
  case PICKUP_STATE_LISTENING:
    name = std::string( "Listening" );
    break;
  case PICKUP_STATE_SEEING:
    name = std::string( "Seeing" );
    break;
  case PICKUP_STATE_SELECTING_OBJECT:
    name = std::string("Selecting object");
    break;
  case PICKUP_STATE_SELECTED_PROCESSING:
    name = std::string("Selected processing");
    break;
  case PICKUP_STATE_PLANNING:
    name = std::string("Planning");
    break;
  case PICKUP_STATE_EXECUTING:
    name = std::string("Executing");
    break;
  case PICKUP_STATE_NOTIFYING:
    name = std::string("Notifying");
    break;
  case PICKUP_STATE_NONE:
    name = std::string("None");
    break;
  default:
    name = std::string("WTF?");
    break;
  }
  
  ui->state_label->setText( QString(name.c_str()) );
}

/**
 * @function listen_fsm
 * @brief Constantly checks and updates the states according to msgs received
 */
void Server_Pick::listen_fsm() {

    // Always show state name
    this->show_state_name_fsm();
    if( this->poll_fsm() == true ) {
      this->evaluate_fsm( mMsg_fsm );
    }
  
}

/**
 * @function poll_fsm
 */
bool Server_Pick::poll_fsm() {

  // Check channels
  size_t buf_size;
  size_t frame_size;
  ach_status r;
  
  alpha_msg msg;
  
  // Perception/Plan
  r = ach_get( &mModule2Server_chan, &msg, sizeof(msg), &frame_size, NULL, ACH_O_LAST );
  if( (r == ACH_OK || r == ACH_MISSED_FRAME)  && !sns_msg_is_expired( &msg.header, NULL) ) {
    mMsg_fsm = msg; return true;
  }
  // Client
  if( mClient_got_msg_fsm ) {
    // msg was gotten in the function that set mClient_got_msg
    printf("Got message from client. Type: %d \n", mMsg_fsm.type);
    mClient_got_msg_fsm = false;
    return true;
  }

  return false;
}


/**
 * @function evaluate
 */
void Server_Pick::evaluate_fsm( const alpha_msg &_msg ) {

  // Reset put everything in halt, regardless of the current state
  if( _msg.type == PICKUP_MSG_CMD_SERVER_RESET ) {
      mState_fsm = PICKUP_STATE_LISTENING;
      action_fsm( PICKUP_ACTION_PLAN_RESET );    
  }
  
  // If msg, then next state
  switch( mState_fsm ) {

  case PICKUP_STATE_NONE:
    if( _msg.type == PICKUP_MSG_CMD_SERVER_CONNECTED ) {
      mState_fsm = PICKUP_STATE_LISTENING;
      action_fsm( PICKUP_ACTION_NONE );
    }
    break;

    
  case PICKUP_STATE_LISTENING:
    if( _msg.type == PICKUP_MSG_CMD_SERVER_START ) {
      mState_fsm = PICKUP_STATE_SEEING;
      action_fsm( PICKUP_ACTION_PERCEPTION_GRAB_IMG );
    }
    break;
    
  case PICKUP_STATE_SEEING:
    if( _msg.type == PICKUP_MSG_STATUS_PERCEPTION_IMG_YES ) {
      mState_fsm = PICKUP_STATE_SELECTING_OBJECT;
      std::stringstream ss(_msg.line); int t; ss >> t >> mImg_filename; 
      action_fsm( PICKUP_ACTION_CLIENT_CHOOSE_IMG );
    }
    else if( _msg.type == PICKUP_MSG_STATUS_PERCEPTION_IMG_NO ) {
      mState_fsm = PICKUP_STATE_NOTIFYING;
      mNotify_msg = std::string("(!)_Perception_failed");
      action_fsm( PICKUP_ACTION_CLIENT_NOTIFY );
    }
    break;
    
  case PICKUP_STATE_SELECTING_OBJECT:
    if( _msg.type == PICKUP_MSG_STATUS_CLIENT_CHOOSE_YES ) {
      mState_fsm = PICKUP_STATE_SELECTED_PROCESSING;
      action_fsm( PICKUP_ACTION_PERCEPTION_SEND );
    }
    else if( _msg.type == PICKUP_MSG_STATUS_CLIENT_CHOOSE_NO ) {
      mState_fsm = PICKUP_STATE_NOTIFYING;
      mNotify_msg = std::string("(!)_Client_choosing_failed");
      action_fsm( PICKUP_ACTION_CLIENT_NOTIFY );
    }
    break;
    
  case PICKUP_STATE_SELECTED_PROCESSING:
    if( _msg.type == PICKUP_MSG_STATUS_PERCEPTION_SENT_YES ) {
      mState_fsm = PICKUP_STATE_PLANNING;
      action_fsm( PICKUP_ACTION_PLAN_PLAN );
    }
    else if( _msg.type == PICKUP_MSG_STATUS_PERCEPTION_SENT_NO ) {
      mState_fsm = PICKUP_STATE_NOTIFYING;
      mNotify_msg = std::string("(!)_Perception_picking_failed");
      action_fsm( PICKUP_ACTION_CLIENT_NOTIFY );
    }       
    break;
    
  case PICKUP_STATE_PLANNING:
    if( _msg.type == PICKUP_MSG_STATUS_PLAN_PLANNED_YES ) {
      mState_fsm = PICKUP_STATE_EXECUTING;
      action_fsm( PICKUP_ACTION_PLAN_EXECUTE );
    }
    else if( _msg.type == PICKUP_MSG_STATUS_PLAN_PLANNED_NO ) {
      mState_fsm = PICKUP_STATE_NOTIFYING;
      mNotify_msg = std::string("(!)_Plan_failed");
      action_fsm( PICKUP_ACTION_CLIENT_NOTIFY );
    }       
    break;
    
  case PICKUP_STATE_EXECUTING:
    if( _msg.type == PICKUP_MSG_STATUS_PLAN_EXECUTED_YES ) {
      mState_fsm = PICKUP_STATE_NOTIFYING;
      mNotify_msg = std::string("(!)_Executing_SUCCESS!");
      action_fsm( PICKUP_ACTION_CLIENT_NOTIFY );
    }
    else if( _msg.type == PICKUP_MSG_STATUS_PLAN_EXECUTED_NO ) {
      mState_fsm = PICKUP_STATE_NOTIFYING;
      mNotify_msg = std::string("(!)_Executing_failed");
      action_fsm( PICKUP_ACTION_CLIENT_NOTIFY );
    }              
     break;
     
  case PICKUP_STATE_NOTIFYING:
    if( _msg.type == PICKUP_MSG_STATUS_CLIENT_NOTIFIED ) {
      mState_fsm = PICKUP_STATE_LISTENING;
      action_fsm( PICKUP_ACTION_NONE );
    }       
     break;
  }
  
} 


/**
 * @function action
 */
void Server_Pick::action_fsm( int _action_type ) {

  switch( _action_type ) {
    
  case PICKUP_ACTION_NONE: {
    // Nothing :D
  } break;
    
  case PICKUP_ACTION_PLAN_RESET: {
    alpha_msg msg;
    msg.type = PICKUP_MSG_CMD_PLAN_RESET;
    sns_msg_set_time( &msg.header, NULL, 0.1*1e9);
    ach_put( &mServer2Module_chan, &msg, sizeof(msg) );    
  } break;

    
  case PICKUP_ACTION_PERCEPTION_GRAB_IMG: {
    alpha_msg msg;
    msg.type = PICKUP_MSG_CMD_PERCEPTION_GRAB_IMG;
    sns_msg_set_time( &msg.header, NULL, 0.1*1e9);
    ach_put( &mServer2Module_chan, &msg, sizeof(msg) );
  } break;

  case PICKUP_ACTION_CLIENT_CHOOSE_IMG: {
    char filename_local[200];
    std::stringstream ss( mMsg_fsm.line );
    int t; std::string filename; ss >> t >> filename;
    
    std::cout << "Filename img: "<< filename << std::endl;

    sprintf( filename_local,"%s/%s", gPicturesPath.c_str(), filename.c_str() );
    
    QBluetoothTransferRequest req( mService_OBB.device().address() );
    mFile_OBB = new QFile( QString(filename_local) );
    mReply_OBB = mMgr.put( req, mFile_OBB );

    connect( mReply_OBB, SIGNAL(finished(QBluetoothTransferReply*)),
	     this, SLOT(OB_finishedTransfer_slot(QBluetoothTransferReply*)));
    
  } break;

  case PICKUP_ACTION_PERCEPTION_SEND: {    
    alpha_msg msg;
    // See the coordinates
    int px, py, t;
    std::stringstream ss(mMsg_fsm.line);
    ss >> t >> px >> py;
    msg.type = PICKUP_MSG_CMD_PERCEPTION_SEND;
    sprintf( msg.line, "%d %d %d", msg.type, px, py );
    sns_msg_set_time( &msg.header, NULL, 0.1*1e9);
    ach_put( &mServer2Module_chan, &msg, sizeof(msg) );     
  } break;    

  case PICKUP_ACTION_PLAN_PLAN: {
    alpha_msg msg;
    msg.type = PICKUP_MSG_CMD_PLAN_PLAN;
    sns_msg_set_time( &msg.header, NULL, 0.1*1e9);
    ach_put( &mServer2Module_chan, &msg, sizeof(msg) );     
  } break;        

  case PICKUP_ACTION_PLAN_EXECUTE: {
    alpha_msg msg;
    msg.type = PICKUP_MSG_CMD_PLAN_EXECUTE;
    sns_msg_set_time( &msg.header, NULL, 0.1*1e9);
    ach_put( &mServer2Module_chan, &msg, sizeof(msg) );     
  } break;

  case PICKUP_ACTION_CLIENT_NOTIFY: {
    alpha_msg msg;
    // Put message type and info together
    msg.type = PICKUP_MSG_CMD_CLIENT_NOTIFY;
    sprintf( msg.line, "%d %s", msg.type, mNotify_msg.c_str() );
    sns_msg_set_time( &msg.header, NULL, 0.1*1e9);
    //ach_put( &mServer2Client_chan, &msg, sizeof(msg) );     
  } break;
    
  } // end switch 
  
}

/**
 * @function reset_fsm
 */
void Server_Pick::reset_fsm() {
  mState_fsm = PICKUP_STATE_LISTENING;
}
