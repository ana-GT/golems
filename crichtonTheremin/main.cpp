
#include <QApplication>
#include "crichtonTheremin.h"

int main( int argc, char* argv[] ) {
    QApplication app(argc, argv );
    app.setApplicationDisplayName("Crichton Theremin");

    CrichtonTheremin ct;
    ct.show();

    return app.exec();
}
