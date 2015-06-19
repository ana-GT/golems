#pragma once

#include <QtBluetooth/QBluetoothServer>
#include <QtBluetooth/QBluetoothSocket>

class server_unit : public QObject {
    Q_OBJECT

public:
    explicit server_unit( QObject* parent  = 0 );
    ~server_unit();

    void start_server( const QBluetoothAddress &_localAdapter = QBluetoothAddress() );
    void stop_server();

    public slots:
      void send_client_msg_slot( const QString &_message );

      private slots:
	void client_connected_slot();
	void client_disconnected_slot();
	void read_socket_slot();
	
 signals:
    void rcv_client_msg_signal( const QString &_sender,
				const QString &_msg );
    void client_connected_signal( const QString &_name );
    void client_disconnected_signal( const QString &_name );
    void send_server_msg_signal(const QString &_msg );
      
 private:
      QBluetoothServer *mRfcommServer;
      QBluetoothServiceInfo mServiceInfo;
      QBluetoothSocket* mClientSocket;
};
