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
#include <msgs/server_msgs.h>

#include "test2_fsa.h"

enum OPTIONS_MSG {
  NOTHING=0,
  DEBUG = 1,
  CLIENT_NEWS=2,
  CLIENT_CMD=3,
  SERVER_COMMAND=4,
  SERVER_NOTIFICATION=5,
  ERROR=6,
  INFO = 7
};

namespace Ui {
class Test2_Server;
}

/**
 * @class Test2_Server
 */
class Test2_Server : public QMainWindow
{
    Q_OBJECT

public:
    explicit Test2_Server(QWidget *parent = 0);
    ~Test2_Server();
  
    void showMsg( const QString &_msg, int _option = NOTHING );
    void do_action( int _action );
    void poll_chan();
    void poll_fsa();
    void reset_server();
    void showStateName();
    void notify_client( int _typeNotification );
    void goBack();
signals:
   void sendMsg_signal( const QString &_msg );
   void rcvMsg_signal( const QString &_msg );

 public slots:
   void dbgMsg_slot( const QString &_msg );

private slots:
   void sendMsg_slot();
   void rcvMsg_slot( const QString &_sender, const QString &_msg );
   void clientConnected_slot( const QString &_name );
   void clientDisconnected_slot( const QString &_name );
   void stopComm_slot();
   void setAch_slot();
   void poll();

   void OB_discovered_slot( QBluetoothServiceInfo );
   void OB_finishedTransfer_slot( QBluetoothTransferReply*);
   
private:
   Ui::Test2_Server *ui;

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


   Test2_FSA mFsa;
   bool mMsg_received_flag;
   int mMsg_source; int mMsg_type;
   int mPx, mPy;

   std::string mImage_filename;
   std::string mImage_fullname;

   // Communication between processes
   bool mAchComm_ready_flag;
   ach_channel_t mServer2See_chan;
   ach_channel_t mSee2Server_chan;
   ach_channel_t mServer2Plan_chan;
   ach_channel_t mPlan2Server_chan;
   
};
