#pragma once

#include <QMainWindow>
#include <QBluetoothHostInfo>
#include <QBluetoothLocalDevice>
#include <QTimer>
#include "client_unit.h"

enum OPTIONS_MSG {
  NOTHING=0,
  DEBUG = 1,
  CLIENT_NEWS=2,
  CLIENT_COMMAND=3,
  SERVER_COMMAND=4,
  SERVER_NOTIFICATION=5,
  ERROR=6,
  INFO=7
};

namespace Ui {
    class Client_PaP;
}

/**
 * @class Client_PaP
 */
class Client_PaP : public QMainWindow {

    Q_OBJECT

public:
    explicit Client_PaP(QWidget *parent = 0);
    ~Client_PaP();

    void showPlainMsg( const QString &_msg, int _option = NOTHING );
    void reset_client();
    void presentReport();

signals:
    void sendMsg_signal( const QString &_msg );
    void rcvMsg_signal( const QString &_msg );
    void serverDisconnected();


private slots:
    void connect_slot();
    void sendMsg_slot();
    void dbgMsg_slot(const QString &_msg );
    void rcvMsg_slot( const QString &_sender, const QString &_msg );
    void clientDisconnected_slot();
    void updateTimeDisplay_slot();


private:
    Ui::Client_PaP *ui;
    client_unit* mClient;
    QString mLocalName;
    QBluetoothLocalDevice* mLocalAdapter;

    double mTimes[6];

    int mPx; int mPy;
    QTimer* mTimer;
    int mNumTimeouts;
};
