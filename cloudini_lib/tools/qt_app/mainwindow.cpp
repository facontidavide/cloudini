#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "bytearray_writable.hpp"

#include <QSettings>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>


#include "mcap/reader.hpp"
#include "mcap/writer.hpp"
#include "message_definitions.hpp"

MainWindow::MainWindow(QWidget *parent) :
      QMainWindow(parent),
      ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->progressBar->setHidden(true);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_toolButtonLoad_clicked()
{
  ui->toolButtonEncode->setEnabled(false);
  ui->toolButtonDecode->setEnabled(false);

#ifdef USING_WASM
  auto fileContentReady = [this](const QString &fileName,
                                 const QByteArray &fileContent) {
    filename_ = fileName;
    if (!fileName.isEmpty())
    {
      read_buffer_ = fileContent;

      reader_ = std::make_shared<mcap::McapReader>();
      mcap::BufferReader buffer;
      buffer.reset(reinterpret_cast<const std::byte*>(fileContent.data()),
                   fileContent.size(), fileContent.size());

      auto res = reader_->open(buffer);
      if(!res.ok())
      {
        reader_.reset();
        QMessageBox::warning(this, "Error opening file",
                             QString::fromStdString(res.message));
        return;
      }
      processMCAP();
    }
  };

  QFileDialog::getOpenFileContent("MCAP files (*.mcap)",
                                  fileContentReady);
#else
  QSettings settings;
  QString dir = settings.value("MainWindow.lastDirectoryLoad",
                               QDir::currentPath()).toString();

  filename_ = QFileDialog::getOpenFileName(
      this, "Open a MCAP file", dir, "MCAP files (*.mcap)");

  if(!filename_.isEmpty())
  {
    dir = QFileInfo(filename_).absolutePath();
    settings.setValue("MainWindow.lastDirectoryLoad", dir);

    reader_ = std::make_shared<mcap::McapReader>();
    auto res = reader_->open(filename_.toStdString());
    if(!res.ok())
    {
      reader_.reset();
      QMessageBox::warning(this, "Error opening file",
                           QString::fromStdString(res.message));
      return;
    }
  }
#endif

  auto status = reader_->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  ui->label->setText("file loaded");

  bool enable_encoding = false;
  bool enable_decoding = false;
  for(const auto& [channel_id, channel_ptr]: reader_->channels()) {
    const auto& schema_ptr = reader_->schema(channel_ptr->schemaId);
    qDebug() << "Topic: " << channel_ptr->topic  << " schema: " << schema_ptr->name;
    if(schema_ptr->name == pointcloud_schema_name) {
      enable_encoding = true;
    }
    else if(schema_ptr->name == compressed_schema_name) {
      enable_decoding = true;
    }
  }
  ui->toolButtonEncode->setEnabled(enable_encoding);
  ui->toolButtonDecode->setEnabled(enable_decoding);
}

void MainWindow::on_toolButtonDecode_clicked()
{
  processMCAP(false);
}


void MainWindow::on_toolButtonEncode_clicked()
{
  processMCAP(true);
}


void MainWindow::processMCAP(bool encode) {
  ui->toolButtonLoad->setEnabled(false);
  QSettings settings;
  QString dir = settings.value("MainWindow.lastDirectorySave",
                               QDir::currentPath()).toString();

  QString filename = QFileDialog::getSaveFileName(
      this, "Save MCAP file", dir, "MCAP files (*.mcap)");

  if(filename.isEmpty())
  {
    return;
  }
  if(QFileInfo(filename).suffix() != "mcap")
  {
    filename += ".mcap";
  }
  dir = QFileInfo(filename).absolutePath();
  settings.setValue("MainWindow.lastDirectorySave", dir);

  // Read and write loop

  mcap::McapWriter writer;
  mcap::McapWriterOptions writer_options(reader_->header()->profile);

  auto status = writer.open(filename.toStdString(), writer_options);
  if (!status.ok())
  {
    QMessageBox::warning(this, "Error opening file",
                         "Can't open the file for writing");
    return;
  }
  //--------------------------------
  const auto old_schemas = reader_->schemas();
  std::set< mcap::SchemaId> ordered_schema_id;
  for(const auto& [schema_id, _]: old_schemas) {
    ordered_schema_id.insert(schema_id);
  }

  const auto old_channels = reader_->channels();
  std::set<mcap::ChannelId> ordered_channels_id;
  for(const auto& [channel_id, _]: old_channels) {
    ordered_channels_id.insert(channel_id);
  }

  //--------------------------------
  std::map<mcap::SchemaId, mcap::SchemaId> old_to_new_schema_id;
  std::map<mcap::SchemaId, mcap::ChannelId> old_to_new_channel_id;

  std::set<mcap::SchemaId> schema_to_encode;
  std::set<mcap::SchemaId> schema_to_decode;

  auto copy_string_to_vector = [](const char* str, mcap::ByteArray& array) {
    size_t len = strlen(str);
    const auto* data_ptr = reinterpret_cast<const std::byte*>(str);
    array.resize(len);
    std::memcpy(array.data(), data_ptr, len);
  };

  for(const auto& schema_id: ordered_schema_id)
  {
    const auto& schema_ptr = old_schemas.at(schema_id);
    auto schema_name = schema_ptr->name;
    auto schema_data = schema_ptr->data;
    if(encode && schema_name == pointcloud_schema_name)
    {
      schema_name = compressed_schema_name;
      copy_string_to_vector(compressed_schema_data, schema_data);
      schema_to_encode.insert(schema_id);
    }
    else if(!encode && schema_name == compressed_schema_name)
    {
      schema_name = pointcloud_schema_name;
      copy_string_to_vector(pointcloud_schema_name, schema_data);
      schema_to_decode.insert(schema_id);
    }
    mcap::Schema new_schema(schema_name, schema_ptr->encoding, schema_data);
    writer.addSchema(new_schema);
    old_to_new_schema_id.insert({schema_id, new_schema.id});
  }
  //--------------------------------
  for(const auto& channel_id: ordered_channels_id) {
    const auto channel_ptr = old_channels.at(channel_id);
    auto new_schema_id = old_to_new_schema_id.at(channel_ptr->schemaId);
    mcap::Channel new_channel(channel_ptr->topic, channel_ptr->messageEncoding, new_schema_id);
    writer.addChannel(new_channel);
    old_to_new_channel_id.insert({channel_id, new_channel.id});
  }
  //--------------------------------

  const auto statistics = reader_->statistics();
  size_t total_msgs = 0;
  for(const auto& [channel, msg_count]: statistics->channelMessageCounts) {
    total_msgs += msg_count;
  }
  ui->progressBar->setRange(0, total_msgs);
  ui->progressBar->setHidden(false);
  ui->progressBar->setValue(0);
  QCoreApplication::processEvents();

  mcap::ReadMessageOptions options;
  mcap::ProblemCallback problem = [](const mcap::Status&) {};
  int count = 0;
   ui->label->setText("Saving file");
  for (const auto& msg : reader_->readMessages(problem, options))
  {
    mcap::Message new_msg = msg.message;
    new_msg.channelId = old_to_new_channel_id.at(msg.channel->id);
    auto status = writer.write(new_msg);
    if (!status.ok())
    {
      QMessageBox::warning(this, "Error writing file",
                           "Can't write a message");
      break;
    }
    if(count++ % 10 == 0)
    {
      ui->progressBar->setValue(count);
      QCoreApplication::processEvents();
    }
  }
  writer.close();
  ui->progressBar->setHidden(true);
  ui->label->setText("File saved");
  ui->toolButtonEncode->setEnabled(false);
  ui->toolButtonDecode->setEnabled(false);
  ui->toolButtonLoad->setEnabled(true);
}
