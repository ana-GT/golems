/**
 * @file client_pick
 */

#pragma once

#include <QMainWindow>
#include <QBluetoothHostInfo>
#include <QBluetoothLocalDevice>
#include <QTimer>
#include "client_unit.h"
#include "fsm_header.h"

namespace Ui {
    class Client_Pick;
}

/**
 * @class Client_Pick
 */
class Client_Pick : public QMainWindow {

    Q_OBJECT

public:
    explicit Client_Pick(QWidget *parent = 0);
    ~Client_Pick();

    void showPlainMsg( const QString &_msg );
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
    Ui::Client_Pick *ui;
    client_unit* mClient;
    QString mLocalName;
    QBluetoothLocalDevice* mLocalAdapter;

    double mTimes[6];

    int mPx; int mPy;
    QTimer* mTimer;
    int mNumTimeouts;
};
