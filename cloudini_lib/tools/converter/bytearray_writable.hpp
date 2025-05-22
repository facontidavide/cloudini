#pragma once

#include <mcap/writer.hpp>
#include <QByteArray>


class ByteArrayInterface: public mcap::IWritable {
public:

    ByteArrayInterface() {};
    virtual ~ByteArrayInterface() = default;

    void end() override { bytes_.end(); };

    uint64_t size() const override { return bytes_.size(); };

    const QByteArray& byteArray() const { return bytes_; }

protected:
    void handleWrite(const std::byte* data, uint64_t size) override
    {
        bytes_.append(reinterpret_cast<const char*>(data), size);
    }

    QByteArray bytes_;
};
