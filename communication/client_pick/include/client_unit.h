#pragma once
#include <QBluetoothServiceInfo>
#include <QtCore/QObject>
#include <QBluetoothSocket>
/**
* @class client_unit
*/
class client_unit : public QObject {

    Q_OBJECT

public:
    explicit client_unit( QObject* _parent = 0 );
    ~client_unit();

    void startClient( const QBluetoothServiceInfo &_remoteService );
    void stopClient();

public slots:
    void sendMsg_slot( const QString &_message );

signals:

    void rcvMsg_signal( const QString &_sender,
    const QString &_message );
    void connected_signal( const QString &_name );
    void disconnected_signal();
    void dbgMsg_signal( const QString &_msg );

private slots:
    void readSocket_slot();
    void connected_slot();

private:
    QBluetoothSocket* mSocket;
};
