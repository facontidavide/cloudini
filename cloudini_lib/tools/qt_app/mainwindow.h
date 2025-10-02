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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QByteArray>
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

namespace mcap {
class McapReader;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 private slots:

  void on_toolButtonEncode_clicked();
  void on_toolButtonDecode_clicked();
  void on_toolButtonLoad_clicked();

 private:
  void processMCAP(bool encode);

 private:
  Ui::MainWindow *ui;
  QString filename_;
  std::shared_ptr<mcap::McapReader> reader_;
  QByteArray read_buffer_;
};

#endif  // MAINWINDOW_H
