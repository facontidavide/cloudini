#include "mainwindow.h"

#include <QApplication>
#include <QSettings>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  QCoreApplication::setOrganizationName("Auryn");
  QCoreApplication::setApplicationName("MCAP_Editor");
  QSettings::setDefaultFormat(QSettings::IniFormat);

  MainWindow w;
  w.show();
  return app.exec();
}
