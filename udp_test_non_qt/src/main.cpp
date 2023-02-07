#include "mainwindow.h"
#include <QApplication>
#include <thread>
#include <omp.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    return a.exec();
}
