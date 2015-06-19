#pragma once

#include <QDialog>
#include <QLabel>

namespace Ui {
class SelectObjectDialog;
}

/**
 * @class ImgLabel
 */
class ImgLabel : public QLabel {
    Q_OBJECT
public:
    ImgLabel( QWidget* parent = 0, Qt::WindowFlags f = 0 );
    ~ImgLabel();
    int getXclicked() { return mX; }
    int getYclicked() { return mY; }

signals:
    void clicked();

protected:
    void mousePressEvent( QMouseEvent* _event );
    int mX;
    int mY;
};


/**
 * @class SelectObjectDialog
 */
class SelectObjectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SelectObjectDialog(QWidget *parent = 0);
    ~SelectObjectDialog();
    bool loadFile( const QString &_filename );
    void getClicked( int &_x, int &_y );

public slots:
    void imageClicked();

private:
    double mScaleFactor;
    QString mFilename;
    ImgLabel* mLabel;
    int mX, mY;
    int mPx, mPy;
    Ui::SelectObjectDialog *ui;
};
