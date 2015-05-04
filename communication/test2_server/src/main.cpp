#include "test2_server.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Test2_Server w;
    w.show();

    return a.exec();
}
