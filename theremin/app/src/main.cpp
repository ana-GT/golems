#include "theremin.h"
#include <QApplication>

int main( int argc, char* argv[] ) {

  QApplication a( argc, argv );
  Theremin w;
  w.show();

  return a.exec();

}
