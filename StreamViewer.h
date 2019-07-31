/*
 * StreamViewer.h
 *
 *  Created on: Jul 31, 2019
 *      Author: lacasta
 */

#ifndef STREAMVIEWER_H_
#define STREAMVIEWER_H_
#include <deque>
#include <logger.h>
#include <QObject>
#include <QTextEdit>

class StreamViewer : public QObject, public StreamItem
{
        Q_OBJECT
    private:
        struct LineInfo
        {
            QString *msg;
            int tag;
            LineInfo() : msg(0), tag(0)  {}
        };
        QTextEdit *text_view;
        std::deque<LineInfo *> text_fifo;

    public slots:
        void write_text();

    signals:
        void new_message();

    public:
        StreamViewer(QTextEdit *viewer=nullptr);
        virtual ~StreamViewer();

        void set_text_view(QTextEdit *view);

        int flush()
        {
            return 0;
        }
        int write(const char *, size_t count);
        int close()
        {
            return 0;
        }
};

#endif /* STREAMVIEWER_H_ */
