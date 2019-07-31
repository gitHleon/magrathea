/*
 * StreamViewer.cc
 *
 *  Created on: Jul 31, 2019
 *      Author: lacasta
 */

#include <iostream>
#include <string>
#include <QTextEdit>
#include <QColor>
#include <StreamViewer.h>

StreamViewer::StreamViewer(QTextEdit *viewer)
    : text_view(nullptr)
{
    set_text_view(viewer);
    Logger::instance().add_stream(this);

    connect(this, &StreamViewer::new_message, this, &StreamViewer::write_text);
}

StreamViewer::~StreamViewer()
{
    if (text_view)
    {
       // g_object_unref(_text_view);
    }
}

void StreamViewer::set_text_view(QTextEdit *view)
{
    if (text_view)
    {
        // g_object_unref(view); // how do we do hthis in Qt ?
    }
    if (!view)
        return;

    text_view = view;
    text_view->document()->setMaximumBlockCount(500);
    text_view->setReadOnly(true);
}

int StreamViewer::write(const char *ddd, size_t count)
{
    std::string msg(ddd, count);
    LineInfo *info = new LineInfo();
    info->msg = new QString( QString::fromStdString(msg) );
    if (msg.find("DEBUG") != msg.npos)
    {
        info->tag = Log::debug;
    }
    else if (msg.find("FATAL") != msg.npos || msg.find("ERROR") != msg.npos)
    {
        info->tag = Log::error;
    }
    else if (msg.find("WARNING") != msg.npos)
    {
        info->tag = Log::warning;
    }
    else
        info->tag = Log::info;

    text_fifo.push_back(info);
    emit new_message();

    return count;
}

void StreamViewer::write_text()
{
    if (text_view->updatesEnabled())
    {
        do
        {
            LineInfo *txt = text_fifo.front();
            text_fifo.pop_front();
            QColor old_col = text_view->textColor();
            text_view->moveCursor(QTextCursor::End);
            switch (txt->tag) {
                case Log::debug:
                    text_view->setTextColor( QColor("lightsteelblue") );
                    break;
                case Log::error:
                    text_view->setTextColor( QColor("red") );
                    break;

                case Log::warning:
                    text_view->setTextColor( QColor("tomato") );
                    break;

                default:
                    break;
            }
            text_view->insertPlainText(*txt->msg);
            text_view->setTextColor(old_col);
            text_view->moveCursor(QTextCursor::End);
            delete txt->msg,
            delete txt;
        } while (!text_fifo.empty());
    }
}
