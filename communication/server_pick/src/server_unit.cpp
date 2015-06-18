/**
 * @file server_unit.cpp
 */
#include "server_unit.h"

static const QLatin1String serviceUuid("e8e10f95-1a70-4b27-9ccf-02010264e9c8");

/**
 * @brief server_unit::server_unit
 * @param parent
 */
server_unit::server_unit( QObject* parent ) :
 QObject(parent), mRfcommServer(0) {

}

/**
 * @brief server_unit::~server_unit
 * @param
 */
server_unit::~server_unit() {
    stop_server();
}

/**
 * @brief start_server
 * @param Start the bluetooth communication / setting parameters
 */
void server_unit::start_server( const QBluetoothAddress &_localAdapter ) {
    if( mRfcommServer ) {
        return;
    }

   // Create the server
    mRfcommServer = new QBluetoothServer( QBluetoothServiceInfo::RfcommProtocol, this );
    this->connect( mRfcommServer, SIGNAL(newConnection()),
                   this, SLOT(client_connected_slot()) );
    bool result = mRfcommServer->listen( _localAdapter );
    if( !result ) {
        emit send_server_msg_signal("[SU] Cannot bind RFCOMM server to local adapter \n");
    }


    // Set service info
    mServiceInfo.setAttribute(QBluetoothServiceInfo::ServiceName, tr("Crichton Server"));
    mServiceInfo.setAttribute(QBluetoothServiceInfo::ServiceDescription, tr("Control machine service"));
    mServiceInfo.setAttribute(QBluetoothServiceInfo::ServiceProvider, tr("golems.org"));
    QBluetoothServiceInfo::Sequence publicBrowse;
    publicBrowse << QVariant::fromValue(QBluetoothUuid(QBluetoothUuid::PublicBrowseGroup));
    mServiceInfo.setAttribute(QBluetoothServiceInfo::BrowseGroupList, publicBrowse);

    QBluetoothServiceInfo::Sequence classId;

    classId << QVariant::fromValue(QBluetoothUuid(QBluetoothUuid::SerialPort));
    mServiceInfo.setAttribute(QBluetoothServiceInfo::BluetoothProfileDescriptorList,
                             classId);

    classId.prepend(QVariant::fromValue(QBluetoothUuid(serviceUuid)));

    mServiceInfo.setAttribute(QBluetoothServiceInfo::ServiceClassIds, classId);
    mServiceInfo.setAttribute(QBluetoothServiceInfo::BluetoothProfileDescriptorList,classId);

    mServiceInfo.setServiceUuid(QBluetoothUuid(serviceUuid));


    QBluetoothServiceInfo::Sequence protocolDescriptorList;
    QBluetoothServiceInfo::Sequence protocol;
    protocol << QVariant::fromValue(QBluetoothUuid(QBluetoothUuid::L2cap));
    protocolDescriptorList.append(QVariant::fromValue(protocol));
    protocol.clear();
    protocol << QVariant::fromValue(QBluetoothUuid(QBluetoothUuid::Rfcomm))
             << QVariant::fromValue(quint8(mRfcommServer->serverPort()));
    protocolDescriptorList.append(QVariant::fromValue(protocol));
    mServiceInfo.setAttribute(QBluetoothServiceInfo::ProtocolDescriptorList,
                             protocolDescriptorList);

    mServiceInfo.registerService( _localAdapter );

}

/**
 * @function stop_server
 * @brief Stop RFCOMM communication via bluetooth
 */
void server_unit::stop_server() {
    // Unregister service
    mServiceInfo.unregisterService();
    // Close Sockets
    mClientSocket->deleteLater();
    // Close server
    delete mRfcommServer;
    mRfcommServer = 0;
}

/**
 * @brief Send a message through Bluetooth to the app client
 * @param _msg
 */
void server_unit::send_client_msg_slot( const QString &_msg ) {
  
    QByteArray text = _msg.toUtf8() + '\n';
    if( mClientSocket->write( text ) < 0 ) {
      emit send_server_msg_signal("[DBG] ERROR WRITING TO SOCKET");
    }
}

/**
 * @function client_connected_slot
 * @brief Emit a signal when a new client is connected
 */
void server_unit::client_connected_slot() {

    mClientSocket = mRfcommServer->nextPendingConnection();
    if( !mClientSocket ) { return; }

    QObject::connect( mClientSocket, SIGNAL(readyRead()), this, SLOT(read_socket_slot()) );
    QObject::connect( mClientSocket, SIGNAL(disconnected()), this, SLOT(client_disconnected_slot()) );

    emit client_connected_signal( mClientSocket->peerName() );
}

/**
 * @brief server_unit::clientDisconnected
 */
void server_unit::client_disconnected_slot() {

  if( !mClientSocket ) { return; }
  emit send_server_msg_signal("* Client is being disconnected");
  emit client_disconnected_signal( mClientSocket->peerName() );
  mClientSocket->deleteLater();
}

/**
 * @function server_unit::read_socket_slot
 * @brief Read message from client using app
 */
void server_unit::read_socket_slot() {

  while( mClientSocket->canReadLine() ) {
    QByteArray line = mClientSocket->readLine().trimmed();
    emit rcv_client_msg_signal( mClientSocket->peerName(),
				QString::fromUtf8(line.constData(), line.length()));
  }

}
