/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
