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
