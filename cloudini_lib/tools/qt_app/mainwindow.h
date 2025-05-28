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
