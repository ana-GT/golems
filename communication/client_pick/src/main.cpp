#include "client_pick.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Client_Pick w;
    w.show();

    return a.exec();
}
