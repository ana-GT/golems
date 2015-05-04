#include "test1_server.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Test1_Server w;
    w.show();

    return a.exec();
}
