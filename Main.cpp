#include <QApplication>
#include <QWidget>
#include <iostream>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QWidget widget;
    widget.setFixedSize(400, 400);

    QString helloString = "Hello from " + qgetenv("USER") + "!";
    widget.setWindowTitle(helloString);

    widget.show();

    return QApplication::exec();
}
