

#include "connectDialog.h"
#include "ui_connectDialog.h"
#include <QListWidget>
#include <QLatin1String>

static const QLatin1String serviceUuid("e8e10f95-1a70-4b27-9ccf-02010264e9c8");


/**
* @function connectDialog
* @brief Constructor
*/
connectDialog::connectDialog( const QBluetoothAddress &_localAdapter, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::connectDialog)
{
    ui->setupUi(this);

    QObject::connect( ui->connect_pushButton, SIGNAL(clicked()),
                      this, SLOT(connect_slot()) );

    QObject::connect( ui->cancel_pushButton, SIGNAL(clicked()),
                      this, SLOT(cancel_slot()) );

    mDiscoveryAgent = new QBluetoothServiceDiscoveryAgent(_localAdapter);

    QObject::connect( mDiscoveryAgent, SIGNAL(serviceDiscovered(QBluetoothServiceInfo)),
                      this, SLOT(serviceDiscovered_slot(QBluetoothServiceInfo)) );
    QObject::connect( mDiscoveryAgent, SIGNAL(finished()),
                      this, SLOT(discoveryFinished_slot()) );
    QObject::connect( mDiscoveryAgent, SIGNAL(canceled()),
                      this, SLOT(discoveryFinished_slot()) );

    ui->connect_pushButton->setEnabled(false);
    scan();
}

/**
* @function ~connectDialog
*/
connectDialog::~connectDialog() {
    delete ui;
    delete mDiscoveryAgent;
}

/**
* @function scan
*/
void connectDialog::scan() {

  QBluetoothUuid serviceId(serviceUuid);
  startDiscovery( serviceId );
}

/**
* @function startDiscovery
*/
void connectDialog::startDiscovery( const QBluetoothUuid &_uuid ) {

    // Get the corresponding device
    //QBluetoothAddress address(QString("A4:9A:58:AA:9D:9D")); // phone
    QBluetoothAddress address(QString("00:1A:7D:DA:71:13")); // powell
    //QBluetoothAddress address(QString("00:1A:7D:DA:71:13")); // chicky - FOR SURE


    if( mDiscoveryAgent->setRemoteAddress( address ) == false ) {
        ui->status_label->setText(QString("ERROR: Cannot scan!"));
        return;
    }

    // Access the service to communicate, if available
    mDiscoveryAgent->setUuidFilter( _uuid );
    if( mDiscoveryAgent->isActive() ) { mDiscoveryAgent->stop(); }

    ui->remote_listWidget->clear();
    ui->status_label->setText(QString("Starting to scan..."));
    mDiscoveryAgent->start( QBluetoothServiceDiscoveryAgent::FullDiscovery );

}

/**
* @function stopDiscovery
*/
void connectDialog::stopDiscovery() {
  if( mDiscoveryAgent ) {
    mDiscoveryAgent->stop();
  }

}

/**
* @function cancel_slot
*/
void connectDialog::cancel_slot() {
    if( mDiscoveryAgent ) {
        mDiscoveryAgent->stop();
    }

    this->reject();
}

/**
* @function service
*/
QBluetoothServiceInfo connectDialog::service() const {
  return mService;
}

/**
* @function serviceDiscovered
*/
void connectDialog::serviceDiscovered_slot( const QBluetoothServiceInfo &_serviceInfo ) {

    QString remoteName;
    if( _serviceInfo.device().name().isEmpty() ) {
      remoteName = _serviceInfo.device().address().toString();
    } else {
      remoteName = _serviceInfo.device().name();
    }

    QString msg = QString::fromLatin1("%1 %2").arg(remoteName,
                                                   _serviceInfo.serviceName());
    QListWidgetItem *item = new QListWidgetItem( msg, ui->remote_listWidget );

    mDiscoveredServices.insert(item, _serviceInfo);
    mService = _serviceInfo;
}

/**
* @function discoveryFinished
*/
void connectDialog::discoveryFinished_slot() {
        ui->status_label->setText(QString("Service discovery finished "));
            ui->connect_pushButton->setEnabled(true);
}

/**
* @function connect
*/
void connectDialog::connect_slot() {

  mService = mDiscoveredServices.value( ui->remote_listWidget->currentItem() );

  if( mDiscoveryAgent->isActive() ) {
    mDiscoveryAgent->stop();
  }

  accept();
}

