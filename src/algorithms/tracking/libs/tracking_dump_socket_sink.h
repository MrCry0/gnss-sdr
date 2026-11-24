/*!
 * \file tracking_dump_socket_sink.h
 * \brief Sends the tracking dump records of a channel to a Unix domain stream
 * socket instead of writing them to a file.
 *
 * The sink connects to the socket the first time it has a record and starts
 * the stream with an 8-byte header:
 *
 *   uint8  version, 1
 *   char   system, 'G', 'R', 'E' or 'C'
 *   char   signal[2], "1C", "1G", "1B", "B1"...
 *   uint32 channel
 *
 * followed by records, exactly as dll_pll_veml_tracking::log_data() would
 * write them to the dump file (Tracking_XX.dump=true, dump_mat=false), until
 * the sink is destroyed. Integers and floats are in host byte order, like the
 * dump file.
 *
 * A stream socket has back-pressure: a reader that is slower than the
 * tracking makes send() wait instead of losing records. That suits a file
 * replay, where tracking only runs slower. Records are sent in batches, and
 * the destructor sends what is left.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TRACKING_DUMP_SOCKET_SINK_H
#define GNSS_SDR_TRACKING_DUMP_SOCKET_SINK_H

#include <boost/asio.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */

class Tracking_Dump_Socket_Sink
{
public:
    static constexpr std::size_t dump_record_size = 112;
    static constexpr std::size_t header_size = 8;
    static constexpr std::size_t records_per_write = 16;

    Tracking_Dump_Socket_Sink(std::string path, char system, const char* signal)
        : socket_(io_context_),
          path_(std::move(path)),
          system_(system)
    {
        signal_[0] = signal[0];
        signal_[1] = signal[0] != '\0' ? signal[1] : '\0';
        pending_.reserve(header_size + records_per_write * dump_record_size);
    }

    ~Tracking_Dump_Socket_Sink()
    {
        flush();
    }

    Tracking_Dump_Socket_Sink(const Tracking_Dump_Socket_Sink&) = delete;
    Tracking_Dump_Socket_Sink& operator=(const Tracking_Dump_Socket_Sink&) = delete;

    /*!
     * \brief Adds one record. Returns false once the stream is broken (nobody
     * listening, or the reader went away); records after that are dropped.
     * Waits when the reader is behind.
     */
    bool send(uint32_t channel, const char* record)
    {
        if (failed_)
            {
                return false;
            }
        if (!connected_ && !connect(channel))
            {
                return false;
            }
        pending_.insert(pending_.end(), record, record + dump_record_size);
        if (pending_.size() >= records_per_write * dump_record_size)
            {
                return flush();
            }
        return true;
    }

    /*! \brief Sends the records collected so far. */
    bool flush()
    {
        if (failed_ || !connected_ || pending_.empty())
            {
                return !failed_;
            }
        boost::system::error_code error;
        boost::asio::write(socket_, boost::asio::buffer(pending_), error);
        pending_.clear();
        if (error)
            {
                failed_ = true;
            }
        return !failed_;
    }

    //! The error that ended the stream, empty while it is fine.
    const std::string& error_message() const { return error_message_; }

private:
    bool connect(uint32_t channel)
    {
        boost::system::error_code error;
        socket_.connect(boost::asio::local::stream_protocol::endpoint(path_), error);
        if (error)
            {
                failed_ = true;
                error_message_ = error.message();
                return false;
            }
        connected_ = true;
        std::array<char, header_size> header{};
        header[0] = 1;
        header[1] = system_;
        header[2] = signal_[0];
        header[3] = signal_[1];
        std::memcpy(&header[4], &channel, sizeof(channel));
        pending_.assign(header.begin(), header.end());
        return true;
    }

    boost::asio::io_context io_context_;
    boost::asio::local::stream_protocol::socket socket_;
    std::string path_;
    std::vector<char> pending_;
    std::string error_message_;
    char system_;
    char signal_[2]{};
    bool connected_{false};
    bool failed_{false};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_TRACKING_DUMP_SOCKET_SINK_H
