#pragma once

#include <QtBluetooth/QBluetoothServiceInfo>
#include <QtBluetooth/QBluetoothServiceDiscoveryAgent>
#include <QDialog>
#include <QListWidget>

namespace Ui {
class connectDialog;
}

class connectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit connectDialog( const QBluetoothAddress &_localAdapter, QWidget *parent = 0);
    ~connectDialog();

    void startDiscovery( const QBluetoothUuid &_uuid );
    void stopDiscovery();
    void scan();
    QBluetoothServiceInfo service() const;

private slots:

  void serviceDiscovered_slot( const QBluetoothServiceInfo &_serviceInfo );
  void discoveryFinished_slot();

  void connect_slot();
  void cancel_slot();

private:
    Ui::connectDialog *ui;
    QBluetoothServiceDiscoveryAgent* mDiscoveryAgent;
    QBluetoothServiceInfo mService;
    QMap<QListWidgetItem*, QBluetoothServiceInfo> mDiscoveredServices;

};

