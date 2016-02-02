#pragma once

#include <QMainWindow>
#include <QBluetoothHostInfo>
#include <QBluetoothLocalDevice>
#include <QBluetoothServiceDiscoveryAgent>
#include <QBluetoothTransferRequest>
#include <QBluetoothTransferReply>
#include <QFile>

#include <QTimer>

#include <unistd.h>
#include <sns.h>
#include <ach.h>

#include "server_unit.h"
#include <communication/msgs/server_msgs.h>

#include "fsm_header.h"
#include "fsm_msg.h"


namespace Ui {
class Server_Pick;
}

/**
 * @class Server_Pick
 */
class Server_Pick : public QMainWindow
{
    Q_OBJECT

 public:
    explicit Server_Pick(QWidget *parent = 0);
    ~Server_Pick();
    
    void do_action( int _action );    
    void reset_server();

    // FSM functions
    void show_state_name_fsm();
    bool poll_fsm();
    void evaluate_fsm( const alpha_msg &_msg );
    void action_fsm( int _action_type );
    void reset_fsm();

	
 signals:
    void send_client_msg_signal( const QString &_msg );
    
    public slots:
      void show_server_msg_slot( const QString &_msg );

      // FSM slots
      void listen_fsm(); 
      
      private slots:
	void send_test_msg_slot();
	void set_comm_slot();
	void stop_server_slot();
	void client_connected_slot( const QString &_name );
	void client_disconnected_slot( const QString &_name );
	void rcv_client_msg_slot( const QString &_sender, const QString &_msg );
	   
	void OB_discovered_slot( QBluetoothServiceInfo );
	void OB_finishedTransfer_slot( QBluetoothTransferReply*);
	
 private:
	Ui::Server_Pick *ui;
	
	server_unit* mServer;
	QString mLocalName;
	QBluetoothLocalDevice* mLocalAdapter;
	QTimer* mTimer;
	
	// For pushing image
	QBluetoothServiceDiscoveryAgent* mDiscoveryAgent;
	QBluetoothServiceInfo mService_OBB;
	QFile* mFile_OBB;
	QBluetoothTransferReply* mReply_OBB;
	QBluetoothTransferManager mMgr;

	// Variables
	bool mComm_ready;

	//************************************//
	// Finite State Machine
	ach_channel mServer2Module_chan;
	ach_channel mModule2Server_chan;
	bool mClient_got_msg_fsm;
		
	int mState_fsm;
	std::string mNotify_msg;
	std::string mClient_msg;
	alpha_msg mMsg_fsm;

	std::string mImg_filename;
   
};
