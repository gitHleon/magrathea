/*
 * logger.h
 *
 *  Created on: 25 Dec 2010
 *      Author: lacasta
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iosfwd>
#include <sstream>
#include <cstdio>
#include <ctime>
#include <functional>
#include <vector>
#include <map>
#include <mutex>

/**

 DAQ++ provides a thread safe logging mechanism. The system is
 managed by Logger. This class is implemented as a
 singleton. Logger::instance() is used to retrieve the
 reference to the only instance.

 Logger allows to register several streams where the
 logging information will be sent. Each stream has assigned a
 level mask to select the accepted logging levels.

 The actual std::ostream where the logging information is
 written is ostream.  Contrary to what happens with
 std::cout, std::cerr, etc. it is not a global ostream and the
 way to use it is by creating an instance at least on each
 thread. Normally, one creates an instance in the calling
 method.

 There are several logging levels:
 - Log::none
 - Log::fatal
 - Log::error
 - Log::critical
 - Log::warning
 - Log::message
 - Log::info
 - Log::debug

 */
namespace Log
{
    enum LogLevel
    {
        none = 1 << 0,      /*!< No header print. The text appears as is */
        fatal = 1 << 1,     /*!< Log level FATAL */
        error = 1 << 2,     /*!< Log level ERROR */
        critical = 1 << 3,  /*!< Log level CRITICAL */
        warning = 1 << 4,   /*!< Log level WARNING */
        message = 1 << 5,   /*!< Log level MESSAGE */
        info = 1 << 6,      /*!< Log level INFO */
        debug = 1 << 7      /*!< Log level DEBUG */
    };
}  // namespace Log

/**
 * This class represents a Log stream.
 *
 * A log stream can be any interface (file, terminal, GUI text
 * box) that receives the data sent to the main stream.
 *
 * It contains a file pointer and the level mask.  Only the levels
 * accepted by the mask will be sent to the stream.
 */
class StreamItem
{
    private:
        unsigned int mask;

    public:
        StreamItem(unsigned short m = 0xff) :
            mask(m) {}

        virtual ~StreamItem() {}

        virtual int write(const char*, size_t sz) = 0;
        virtual int flush() = 0;
        virtual int close() = 0;

        /// Checks if the given level is accepted
        void set_mask(unsigned int m)
        {
            mask = m;
        }
        bool check_mask(unsigned int v)
        {
            return (v & mask);
        }
        unsigned int get_mask() const
        {
            return mask;
        }
};

/**
 *  This is a specialized stream that writes into a file
 */
class IOStreamItem: public StreamItem
{
    private:
        FILE *pf;

    public:
        IOStreamItem(FILE *ff, unsigned short m = 0xff);
        IOStreamItem(const IOStreamItem &si);
        int write(const char*, size_t sz);
        int flush();
        int close();
        FILE* file() { return pf;}

        static StreamItem* create(int fd, unsigned int mask = 0xffff);
        static StreamItem* create(const char *fname, unsigned int mask = 0xffff);
};

/**
 *  This class maintains a collection of streams where the
 *  logging information will be dumped.
 */
class StreamList: private std::vector<StreamItem*>
{
    private:
        /// A Motex to protect the list and the output
        std::mutex _mtx;

    public:
        StreamList();
        ~StreamList();

        using std::vector<StreamItem*>::const_iterator;
        using std::vector<StreamItem*>::iterator;
        using std::vector<StreamItem*>::begin;
        using std::vector<StreamItem*>::end;
        using std::vector<StreamItem*>::size;
        using std::vector<StreamItem*>::empty;

        // Adds a new stream
        void add_stream(StreamItem *item);

        /// acquires the mutex lock
        void acquire()
        {
            _mtx.lock();
        }

        /// Releases the mutex lock
        void release()
        {
            _mtx.unlock();
        }

        bool busy()
        {
            bool rc = _mtx.try_lock();
            if (!rc)
                return false;
            else
            {
                _mtx.unlock();
                return false;
            }
        }
};

/**
 * This is the streambuf that will do the actual writing to the
 * real streams.
 *
 * It is a line based logging system, and the text will be written only
 * after stream flushing. This happens after a std::endl or std::flush
 */
class StreamBuf: public std::basic_stringbuf<char>
{
    private:
        Log::LogLevel log_level;
        StreamList *slist;
        std::string LineHeader();

    public:
        explicit StreamBuf(StreamList *l);
        ~StreamBuf();
        /// Does the actual writing
        int sync();
        /// Set the logging level
        void set_level(Log::LogLevel s);
};

/**
 * This is the main class steering the Logging system.
 * It is implemented as a singleton
 */
class Logger
{
    private:
        /// The list of streams where the text will be sent
        StreamList _streams;

        /// Static pointer to the unique instance of Logger
        static Logger _instance;

    protected:
        /// Constructor.
        Logger();
        /// Copy constructor
        Logger(const Logger&);

    public:
        /// Destructor
        ~Logger();

        /// Returns the only instance
        static Logger& instance()
        {
            return _instance;
        }
        /// Adds a new stream (already opened)
        void add_stream(StreamItem *stream)
        {
            _streams.add_stream(stream);
        }
        /// A pointer to the list of streams
        StreamList* streams()
        {
            return &_streams;
        }

        /// Sets a common message mask
        void set_mask(unsigned int m)
        {
            StreamList::iterator ip, jp = _streams.end();
            for (ip = _streams.begin(); ip != jp; ++ip)
                (*ip)->set_mask(m);
        }

        /// This returns a string representation of the log levels
        static std::map<Log::LogLevel, std::string> loglevel_names;
};

/**
 * This is the actual stream.
 * One should create an instance in each thread to
 * be sure that there are no race conditions. The StreamBuf
 * will take care of writing to the final devices.
 */
class LoggerStream: public std::ostream
{
    public:
        LoggerStream();
        ~LoggerStream();

        /// Sets the current log level
        void loglevel(Log::LogLevel lvl);
};

/**
 * This is an iomanipulator that allows to change the log level
 */
class loglevel
{
    private:
        Log::LogLevel _level;

    public:
        explicit loglevel(Log::LogLevel l)
            : _level(l) {}

        std::ostream& operator()(std::ostream &ostr) const;
};

/// The iomanipulator function
inline std::ostream&
operator<<(std::ostream &ostr, loglevel const &level)
{
    return level(ostr);
}

#endif  // LOGGER_H_
