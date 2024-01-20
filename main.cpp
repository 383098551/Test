#include "dmcoderecognition.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DMCodeRecognition w;
    w.show();
    return a.exec();
}
