/*!
 * \file bladerf_source.cc
 * \brief GNU Radio source block for Nuand's bladeRF front-ends (bladeRF x40,
 * x115, and bladeRF 2.0 Micro xA4/xA9), implemented directly against
 * libbladeRF.
 * \author Oleksandr Suvorov, 2026.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "bladerf_source.h"
#include <gnuradio/io_signature.h>
// bladeRF2.h relies on macros defined by libbladeRF.h but does not include
// it itself, so this order must be preserved (protected against
// clang-format's alphabetical include sorting).
// clang-format off
#include <libbladeRF.h>
#include <bladeRF2.h>
// clang-format on
#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
constexpr bladerf_channel kRxChannel = BLADERF_CHANNEL_RX(0);

std::string make_device_identifier(const std::string &bladerf_args, const std::string &device_serial)
{
    if (!bladerf_args.empty())
        {
            return bladerf_args;
        }
    if (!device_serial.empty())
        {
            return "*:serial=" + device_serial;
        }
    return {};
}
}  // namespace


void bladerf_source::bladerf_deleter::operator()(struct bladerf *dev) const
{
    if (dev != nullptr)
        {
            bladerf_close(dev);
        }
}


bladerf_source_sptr bladerf_make_source_sptr(
    const std::string &bladerf_args,
    const std::string &device_serial,
    double sample_rate,
    double freq,
    double bandwidth,
    double gain,
    bool agc_enabled,
    bool rx_bias_tee,
    unsigned int num_buffers,
    unsigned int buffer_size,
    unsigned int num_transfers,
    unsigned int stream_timeout_ms)
{
    return bladerf_source_sptr(new bladerf_source(bladerf_args, device_serial,
        sample_rate, freq, bandwidth, gain, agc_enabled, rx_bias_tee,
        num_buffers, buffer_size, num_transfers, stream_timeout_ms));
}


bladerf_source::bladerf_source(
    const std::string &bladerf_args,
    const std::string &device_serial,
    double sample_rate,
    double freq,
    double bandwidth,
    double gain,
    bool agc_enabled,
    bool rx_bias_tee,
    unsigned int num_buffers,
    unsigned int buffer_size,
    unsigned int num_transfers,
    unsigned int stream_timeout_ms)
    : gr::sync_block("bladerf_source",
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(1, 1, sizeof(gr_complex))),
      buffer_size_(buffer_size),
      stream_timeout_ms_(stream_timeout_ms),
      streaming_(false)
{
    const std::string identifier = make_device_identifier(bladerf_args, device_serial);

    struct bladerf *raw_dev = nullptr;
    int status = bladerf_open(&raw_dev, identifier.empty() ? nullptr : identifier.c_str());
    if (status != 0)
        {
            std::ostringstream oss;
            oss << "bladeRF: unable to open device '" << identifier << "': " << bladerf_strerror(status);
            LOG(ERROR) << oss.str();
            throw std::runtime_error(oss.str());
        }
    dev_.reset(raw_dev);

    std::cout << "bladeRF: opened board '" << bladerf_get_board_name(dev_.get()) << "'\n";
    LOG(INFO) << "bladeRF: opened board '" << bladerf_get_board_name(dev_.get()) << "'";

    bladerf_sample_rate actual_sample_rate = 0;
    status = bladerf_set_sample_rate(dev_.get(), kRxChannel, static_cast<bladerf_sample_rate>(sample_rate), &actual_sample_rate);
    if (status != 0)
        {
            std::ostringstream oss;
            oss << "bladeRF: unable to set sample rate: " << bladerf_strerror(status);
            LOG(ERROR) << oss.str();
            throw std::runtime_error(oss.str());
        }
    std::cout << "bladeRF: actual RX sample rate: " << actual_sample_rate << " [SPS]\n";

    bladerf_bandwidth actual_bandwidth = 0;
    status = bladerf_set_bandwidth(dev_.get(), kRxChannel, static_cast<bladerf_bandwidth>(bandwidth), &actual_bandwidth);
    if (status != 0)
        {
            std::ostringstream oss;
            oss << "bladeRF: unable to set bandwidth: " << bladerf_strerror(status);
            LOG(ERROR) << oss.str();
            throw std::runtime_error(oss.str());
        }
    std::cout << "bladeRF: actual RX bandwidth: " << actual_bandwidth << " [Hz]\n";

    status = bladerf_set_frequency(dev_.get(), kRxChannel, static_cast<bladerf_frequency>(freq));
    if (status != 0)
        {
            std::ostringstream oss;
            oss << "bladeRF: unable to set frequency: " << bladerf_strerror(status);
            LOG(ERROR) << oss.str();
            throw std::runtime_error(oss.str());
        }
    std::cout << "bladeRF: RX frequency set to " << freq << " [Hz]\n";

    status = bladerf_set_gain_mode(dev_.get(), kRxChannel, agc_enabled ? BLADERF_GAIN_DEFAULT : BLADERF_GAIN_MGC);
    if (status != 0)
        {
            LOG(WARNING) << "bladeRF: unable to set gain mode: " << bladerf_strerror(status);
        }

    if (!agc_enabled)
        {
            status = bladerf_set_gain(dev_.get(), kRxChannel, static_cast<bladerf_gain>(gain));
            if (status != 0)
                {
                    LOG(WARNING) << "bladeRF: unable to set gain: " << bladerf_strerror(status);
                }
        }

    if (rx_bias_tee)
        {
            status = bladerf_set_bias_tee(dev_.get(), kRxChannel, true);
            if (status != 0)
                {
                    LOG(WARNING) << "bladeRF: unable to enable RX bias tee (not supported on this board?): " << bladerf_strerror(status);
                }
        }

    status = bladerf_sync_config(dev_.get(), BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11,
        num_buffers, buffer_size_, num_transfers, stream_timeout_ms_);
    if (status != 0)
        {
            std::ostringstream oss;
            oss << "bladeRF: unable to configure synchronous RX stream: " << bladerf_strerror(status);
            LOG(ERROR) << oss.str();
            throw std::runtime_error(oss.str());
        }

    raw_buffer_.resize(static_cast<size_t>(buffer_size_) * 2);
}


bool bladerf_source::start()
{
    int status = bladerf_enable_module(dev_.get(), kRxChannel, true);
    if (status != 0)
        {
            LOG(ERROR) << "bladeRF: unable to enable RX module: " << bladerf_strerror(status);
            return false;
        }
    streaming_ = true;
    return true;
}


bool bladerf_source::stop()
{
    if (streaming_)
        {
            int status = bladerf_enable_module(dev_.get(), kRxChannel, false);
            if (status != 0)
                {
                    LOG(WARNING) << "bladeRF: unable to disable RX module: " << bladerf_strerror(status);
                }
            streaming_ = false;
        }
    return true;
}


int bladerf_source::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    (void)input_items;

    const auto num_samples = std::min(static_cast<unsigned int>(noutput_items), buffer_size_);

    struct bladerf_metadata meta;
    std::memset(&meta, 0, sizeof(meta));

    const int status = bladerf_sync_rx(dev_.get(), raw_buffer_.data(), num_samples, &meta, stream_timeout_ms_);
    if (status == BLADERF_ERR_TIMEOUT)
        {
            return 0;
        }
    if (status != 0)
        {
            LOG(ERROR) << "bladeRF: sync_rx failed: " << bladerf_strerror(status);
            return -1;
        }
    if ((meta.status & BLADERF_META_STATUS_OVERRUN) != 0)
        {
            LOG(WARNING) << "bladeRF: RX overrun detected, samples were dropped";
        }

    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);
    for (unsigned int i = 0; i < num_samples; i++)
        {
            const float i_val = static_cast<float>(raw_buffer_[2 * i]) / 2048.0F;
            const float q_val = static_cast<float>(raw_buffer_[(2 * i) + 1]) / 2048.0F;
            out[i] = gr_complex(i_val, q_val);
        }

    return static_cast<int>(num_samples);
}
