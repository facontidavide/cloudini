#include <QApplication>
#include <QSettings>

#include "mainwindow.h"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  QCoreApplication::setOrganizationName("Auryn");
  QCoreApplication::setApplicationName("MCAP_Editor");
  QSettings::setDefaultFormat(QSettings::IniFormat);

  MainWindow w;
  w.show();
  return app.exec();
}
