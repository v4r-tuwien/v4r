#include <QApplication>
#include "main_window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  MainWindow::Parameter param;
  MainWindow w(argc, argv, param);
  w.show();

  return a.exec();
}
