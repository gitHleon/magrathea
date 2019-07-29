/*
 * logger.cc
 *
 *  Created on: 23 Dec 2010
 *      Author: lacasta
 */

#include <iostream>
#include <logger.h>

std::pair<Log::LogLevel, std::string> __map_data__[] =
{
     std::make_pair(Log::none, "None"),
     std::make_pair(Log::fatal, "FATAL"),
     std::make_pair(Log::error, "ERROR"),
     std::make_pair(Log::critical, "CRITICAL"),
     std::make_pair(Log::warning, "WARNING"),
     std::make_pair(Log::message, "MESSAGE"),
     std::make_pair(Log::info, "INFO"),
     std::make_pair(Log::debug, "DEBUG")
};

std::map<Log::LogLevel, std::string>
    Logger::loglevel_names(__map_data__,
                           __map_data__ + sizeof(__map_data__)/sizeof(__map_data__[0]));


Logger Logger::_instance;


IOStreamItem::IOStreamItem(FILE *ff, unsigned short m)
   : StreamItem(m), pf(ff) {}

IOStreamItem::IOStreamItem(const IOStreamItem &si)
{
   set_mask(si.get_mask());
    pf = si.pf;
}

int IOStreamItem::write(const char *b, size_t sz)
{
   return fwrite(b, 1, sz, pf);
}
int IOStreamItem::flush()
{
   return fflush(pf);
}
int IOStreamItem::close()
{
   return fclose(pf);
}

StreamItem *IOStreamItem::create(int fd, unsigned int mask)
{
   FILE *pf = fdopen(fd, "w");
   if (!pf) {
	  LoggerStream os;
      os << loglevel(Log::info) << "Could not create stream with file descriptor fd=" << fd << std::endl;
      return 0;
   }   
   return new IOStreamItem(pf, mask);
}

StreamItem *IOStreamItem::create(const char *fname, unsigned int mask)
{
   FILE *pf = fopen(fname, "w");
   if (!pf) {
	  LoggerStream os;
      os << loglevel(Log::info) << "Could not create stream " << fname << std::endl;
      return 0;
   }
   return new IOStreamItem(pf, mask);   
}

StreamList::StreamList()
{

}

// Adds a new stream
void StreamList::add_stream(StreamItem *item)
{
    acquire();
    push_back( item );
    release();
}

StreamList::~StreamList()
{
    StreamList::iterator ip;
    acquire();
    for (ip=begin(); ip!=end(); ++ip)
    {
       (*ip)->close();
//       delete (*ip);
	}
    clear();
    release();
}

StreamBuf::StreamBuf(StreamList *l)
    : log_level(Log::none), slist(l)
{
}


std::string StreamBuf::LineHeader()
{
    char buf[80];
    std::string level = Logger::loglevel_names[log_level];
    int i;

    for (i=level.size(); i<8; i++)
        level.append(" ");

    level.append("] ");
    time_t secondsSinceEpoch = time(NULL);
    tm* brokenTime = localtime(&secondsSinceEpoch);
    strftime(buf, sizeof(buf), "[%d/%m/%y %T - ", brokenTime);

    return std::string(buf) + level;
}
void StreamBuf::set_level(Log::LogLevel s)
{
    log_level=s;
}
StreamBuf::~StreamBuf()
{
    this->sync();
}

typedef std::pair< std::string, std::string> TextLine;
int StreamBuf::sync()
{
    //fprintf(stdout, ">>> StreamBuf::sync\n");
    slist->acquire();
    std::streamsize n = pptr() - pbase();
    if (n)
    {
        /*
         * Split the input in lines. We will add the prefix
         */
        char *p, *q, *e;
        bool first = true;
        std::vector< TextLine > lines;
        p = pbase();
        e = p + n;
        std::string header;
        if ( log_level != Log::none )
            header = LineHeader();

        while ( p < e)
        {
            bool found = false;
            for (q=p; q<e; q++)
            {
                if (*q == '\n') {
                    found = true;
                    break;
                }
            }

            if (found)
                lines.push_back( TextLine(header, std::string(p, q-p+1)));

            else
                lines.push_back( TextLine(header, std::string(p, e-p)));

            if (first)
            {
                header = std::string(header.size(), ' ');
                first = false;
            }
            p = q + 1;
        }

        // Now loop on the lines
        std::vector<TextLine>::iterator iline;
        for (iline=lines.begin(); iline!=lines.end(); ++iline)
        {
            StreamList::iterator ip;
            std::string line = iline->first + iline->second;
            const char *cline = line.c_str();
            size_t sz = line.size();
            for (ip=slist->begin(); ip!=slist->end(); ++ip)
            {
                StreamItem *stream = *ip;
                if ( stream->check_mask(log_level) )
                {
                   stream->write(cline, sz);
                   stream->flush();
                }
            }
        }
        this->pubseekpos(0, std::ios_base::out);
        this->pubseekpos(0, std::ios_base::in);
        //this->str("");
    }
    slist->release();
    //fprintf(stdout, "<<< StreamBuf::sync\n");
    return 0;
}


// TODO: Give a level mask to the streams so that they only produce
//       output for the selected levels

Logger::Logger()
{
   _streams.add_stream( new IOStreamItem(stdout) );
}
Logger::Logger (const Logger &)  {}

Logger::~Logger()
{
}

LoggerStream::LoggerStream()
    : std::ostream( new StreamBuf( Logger::instance().streams() ) )
{
}

LoggerStream::~LoggerStream()
{
    delete rdbuf();
}
void LoggerStream::loglevel( Log::LogLevel lvl )
{
    StreamBuf *bf = dynamic_cast<StreamBuf *>(rdbuf());
    if (bf)
    {
        bf->set_level( lvl );
    }
}

std::ostream &loglevel::operator()(std::ostream &ostr) const
{
    StreamBuf *bf = dynamic_cast<StreamBuf *>(ostr.rdbuf());
    if (bf)
    {
        bf->set_level( _level );
    }
    return ostr;
}
