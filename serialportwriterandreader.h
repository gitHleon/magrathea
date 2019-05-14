#ifndef SERIALPORTWRITERANDREADER_H
#define SERIALPORTWRITERANDREADER_H

#include <QObject>
#include <QByteArray>
#include <QSerialPort>
#include <QTextStream>
#include <QTimer>

class SerialPortWriterAndReader : public QObject
{
    Q_OBJECT
public:
    explicit SerialPortWriterAndReader(QSerialPort *serialPort, QObject *parent = nullptr);
    bool write(const QByteArray &writeData);
    void GetRead(QByteArray &Read_data);

public slots:
    bool handleReadyRead();

private slots:
    bool handleBytesWritten(qint64 bytes);
    void handleTimeout_write();
    bool handleTimeout_read();
    bool handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *m_serialPort = nullptr;
    QByteArray m_writeData;
    QByteArray m_readData;
    QTextStream m_standardOutput;
    qint64 m_bytesWritten = 0;
    QTimer m_timer_read;
    QTimer m_timer_write;
};

#endif // SERIALPORTWRITERANDREADER_H
