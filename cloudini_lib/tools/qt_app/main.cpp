// Copyright (c) 2025 Davide Faconti
//
// This file is part of Cloudini.
//
// Licensed under the FSL-1.1-MIT License.
// You may obtain a copy of the License at
// https://fsl.software/
//
// Two years from the release date of this software, you may use
// this file in accordance with the MIT License, as described in
// the LICENSE file in the root of this repository.

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
