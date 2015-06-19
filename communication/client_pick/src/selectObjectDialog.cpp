#include <QMouseEvent>
#include <QMessageBox>
#include <QDir>
#include <QDebug>

#include "selectObjectDialog.h"
#include "ui_selectObjectDialog.h"


ImgLabel::ImgLabel( QWidget* parent, Qt::WindowFlags f ) :
QLabel( parent, f ) {

}

ImgLabel::~ImgLabel() {

}

void ImgLabel::mousePressEvent( QMouseEvent* _event ) {
    mX = _event->pos().x();
    mY = _event->pos().y();

    emit clicked();
}


////////////////////////////////////////////////

/**
 * @function SelectObjectDialog
 */
SelectObjectDialog::SelectObjectDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SelectObjectDialog)
{
    ui->setupUi(this);

    // Create Label properties
    mLabel = new ImgLabel( ui->img_frame );
    mLabel->setBackgroundRole( QPalette::Base );
    mLabel->setScaledContents(true);

    // Buttons
    QObject::connect( mLabel, SIGNAL(clicked()),
                      this, SLOT(imageClicked()) );
    QObject::connect( ui->accept_pushButton, SIGNAL(clicked()),
                      this, SLOT(accept()) );

}



/**
 * @function ~SelectObjectDialog
 */
SelectObjectDialog::~SelectObjectDialog()
{
    delete ui;
}

/**
 * @function getClicked
 */
void SelectObjectDialog::getClicked( int &_x, int &_y ) {
    _x = mPx;
    _y = mPy;
}

/**
 * @function loadFile
 */
bool SelectObjectDialog::loadFile( const QString &_filename ) {

    mFilename = _filename;
    QImage image( _filename );

    if( image.isNull() ) {
       QMessageBox::information(this, QGuiApplication::applicationDisplayName(),
                                tr("Could not load %1.").arg(QDir::toNativeSeparators(_filename)));
       setWindowFilePath(QString());
       mLabel->setPixmap( QPixmap());
       mLabel->adjustSize();
       return false;
    }

    QImage scaledImg;
    mScaleFactor = 0.5;
    scaledImg = image.scaled( QSize(320,240), Qt::KeepAspectRatio, Qt::FastTransformation );

    mLabel->setPixmap( QPixmap::fromImage(scaledImg) );

    mLabel->adjustSize();

    setWindowFilePath( _filename );

    return true;
}

/**
 * @function SelectObjectDialog::imageClicked
 */
void SelectObjectDialog::imageClicked() {

    mPx = (int) ( mLabel->getXclicked() / mScaleFactor );
    mPy = (int) ( mLabel->getYclicked() / mScaleFactor );
    QString sx = QString::number(mPx);
    QString sy = QString::number(mPy);
    QString msg; msg = QString("Pixel picked: %1 %2").arg( sx, sy );
    ui->msg_label->setText( msg );

}




