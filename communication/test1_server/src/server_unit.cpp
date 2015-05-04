
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
    stopServer();
}

/**
 * @brief startServer
 * @param _localAdapter
 */
void server_unit::startServer( const QBluetoothAddress &_localAdapter ) {
    if( mRfcommServer ) {
        return;
    }

   // Create the server
    mRfcommServer = new QBluetoothServer( QBluetoothServiceInfo::RfcommProtocol, this );
    this->connect( mRfcommServer, SIGNAL(newConnection()),
                   this, SLOT(clientConnected_slot()) );
    bool result = mRfcommServer->listen( _localAdapter );
    if( !result ) {
        emit dbgMsg_signal("[SU] Cannot bind RFCOMM server to local adapter \n");
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
 * @function stopServer
 */
void server_unit::stopServer() {
    // Unregister service
    mServiceInfo.unregisterService();
    // Close Sockets
    mClientSocket->deleteLater();
    // Close server
    delete mRfcommServer;
    mRfcommServer = 0;
}

/**
 * @brief server_unit::sendMessage
 * @param _msg
 */
void server_unit::sendMsg_slot( const QString &_msg ) {
    QByteArray text = _msg.toUtf8() + '\n';

    if( mClientSocket->write( text ) < 0 ) {
        emit dbgMsg_signal("[DBG] ERROR WRITING TO SOCKET");
    }
}

/**
 * @function clientConnected
 */
void server_unit::clientConnected_slot() {

    mClientSocket = mRfcommServer->nextPendingConnection();
    if( !mClientSocket ) { return; }

    QObject::connect( mClientSocket, SIGNAL(readyRead()), this, SLOT(readSocket_slot()) );
    QObject::connect( mClientSocket, SIGNAL(disconnected()), this, SLOT(clientDisconnected_slot()) );

    emit clientConnected_signal( mClientSocket->peerName() );
}

/**
 * @brief server_unit::clientDisconnected
 */
void server_unit::clientDisconnected_slot() {

        if( !mClientSocket ) { return; }
        emit dbgMsg_signal("[DBG] Client is being disconnected");
        emit clientDisconnected_signal( mClientSocket->peerName() );
        mClientSocket->deleteLater();

}

/**
 * @brief server_unit::readSocket
 */
void server_unit::readSocket_slot() {

        while( mClientSocket->canReadLine() ) {
            QByteArray line = mClientSocket->readLine().trimmed();
            emit rcvMsg_signal( mClientSocket->peerName(),
                                QString::fromUtf8(line.constData(), line.length()));
        }

}
