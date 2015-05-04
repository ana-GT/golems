#pragma once

#include <QtBluetooth/QBluetoothServer>
#include <QtBluetooth/QBluetoothSocket>

class server_unit : public QObject {
    Q_OBJECT

public:
    explicit server_unit( QObject* parent  = 0 );
    ~server_unit();

    void startServer( const QBluetoothAddress &_localAdapter = QBluetoothAddress() );
    void stopServer();

public slots:
    void sendMsg_slot( const QString &_message );

signals:
    void rcvMsg_signal( const QString &_sender,
                        const QString &_msg );
    void clientConnected_signal( const QString &_name );
    void clientDisconnected_signal( const QString &_name );
    void dbgMsg_signal(const QString &_msg );

private slots:
    void clientConnected_slot();
    void clientDisconnected_slot();
    void readSocket_slot();

private:
    QBluetoothServer *mRfcommServer;
    QBluetoothServiceInfo mServiceInfo;
    QBluetoothSocket* mClientSocket;
};
