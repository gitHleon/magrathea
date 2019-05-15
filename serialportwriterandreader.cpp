#include "serialportwriterandreader.h"
#include <QtSerialPort/QSerialPortInfo>

//https://doc.qt.io/qt-5/qtserialport-cwriterasync-serialportwriter-cpp.html

SerialPortWriterAndReader::SerialPortWriterAndReader(QSerialPort *serialPort, QObject *parent) :
    QObject(parent),
    m_serialPort(serialPort),
    m_standardOutput(stdout)
{
    m_timer_write.setSingleShot(true);
    connect(m_serialPort, &QSerialPort::readyRead, this, &SerialPortWriterAndReader::handleReadyRead);
    connect(m_serialPort, &QSerialPort::bytesWritten,
            this, &SerialPortWriterAndReader::handleBytesWritten);
    connect(m_serialPort, &QSerialPort::errorOccurred,
            this, &SerialPortWriterAndReader::handleError);
    connect(&m_timer_write, &QTimer::timeout, this, &SerialPortWriterAndReader::handleTimeout_write);
    connect(&m_timer_read, &QTimer::timeout, this, &SerialPortWriterAndReader::handleTimeout_read);
}

bool SerialPortWriterAndReader::handleBytesWritten(qint64 bytes)
{
    m_bytesWritten += bytes;
    if (m_bytesWritten == m_writeData.size()) {
        m_bytesWritten = 0;
        m_standardOutput << QObject::tr("Data successfully sent to port %1")
                            .arg(m_serialPort->portName()) << endl;
        return false;
    }
    return true;
}

bool SerialPortWriterAndReader::handleReadyRead()
{
    m_readData.clear();
    m_readData.append(m_serialPort->readAll());
    if (!m_timer_read.isActive())
        m_timer_read.start(100);
    return true;
}

void SerialPortWriterAndReader::GetRead(QByteArray &Read_data){
    Read_data.clear();
    Read_data = m_readData;
    m_standardOutput<<"Read from serial : "<<Read_data<<endl;
}

void SerialPortWriterAndReader::handleTimeout_write()
{
    m_standardOutput << QObject::tr("Operation timed out for port %1, error: %2")
                        .arg(m_serialPort->portName())
                        .arg(m_serialPort->errorString())
                     << endl;
}

bool SerialPortWriterAndReader::handleTimeout_read()
{
    if (m_readData.isEmpty()) {
        m_standardOutput << QObject::tr("No data was currently available "
                                        "for reading from port %1")
                            .arg(m_serialPort->portName())
                         << endl;
        return false;
    } else {
        m_standardOutput << QObject::tr("Data successfully received from port %1")
                            .arg(m_serialPort->portName())
                         << endl;
        m_standardOutput << m_readData << endl;
        return true;
    }
}

bool SerialPortWriterAndReader::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::WriteError) {
        m_standardOutput << QObject::tr("An I/O error occurred while writing"
                                        " the data to port %1, error: %2")
                            .arg(m_serialPort->portName())
                            .arg(m_serialPort->errorString())
                         << endl;
        return false;
    }
    return true;
}

bool SerialPortWriterAndReader::write(const QByteArray &writeData)
{

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        m_standardOutput << "Name : " << info.portName().toLocal8Bit().constData()<<endl;
        m_standardOutput << "Description : " << info.description().toLocal8Bit().constData()<<endl;
        m_standardOutput << "Manufacturer: " << info.manufacturer().toLocal8Bit().constData()<<endl;

        // Example use QSerialPort
        QSerialPort serial;
        serial.setPort(info);
        if (serial.open(QIODevice::ReadWrite))
            m_standardOutput<<" >> here 0 "<<endl;
            serial.write(writeData);
            m_standardOutput<<" >> here 00 "<<endl; //this works!!!
            serial.close();
    }

    m_writeData.clear();
    m_writeData = writeData;
    m_standardOutput<<" >> here 1 "<<endl;
    const qint64 bytesWritten = m_serialPort->write(writeData); //this NOT works!!!

    m_standardOutput<<" >> here 1 "<<endl;
    if (bytesWritten == -1) {
        m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2")
                            .arg(m_serialPort->portName())
                            .arg(m_serialPort->errorString())
                         << endl;
        return false;
    } else if (bytesWritten != m_writeData.size()) {
        m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2")
                            .arg(m_serialPort->portName())
                            .arg(m_serialPort->errorString())
                         << endl;
        return false;
    }

    m_timer_write.start(500);
    return true;
}
