//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "esc_dft.hpp" //implementation
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <curses.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <sstream>


#define FFT_ON_FPGA 1

namespace po = boost::program_options;
using std::chrono::high_resolution_clock;

struct channel_data {
    int16_t channel_pwr[15];
    double lat;
    double lon;
};

struct channel_data data;
std::string client_crt_path = "certs/client_10.147.20.114-0.crt"; 
std::string client_key_path = "certs/client_10.147.20.114-0.key"; 
std::string ca_crt_path = "certs/ca.crt";


void post_power_data(channel_data data, std::string url);

void post_iq_data(std::vector<std::complex<float>> *buff, size_t len, uint8_t channel, std::string url);

void compute_average_on_bins(float *dft, size_t len);

void set_center_frequency(uint32_t freq, uhd::usrp::multi_usrp::sptr usrp, po::variables_map vm);

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, ant, subdev, ref;
    size_t len;
    double rate, freq, gain, bw, frame_rate, step;
    float ref_lvl, dyn_rng;
    bool show_controls;

    //initialize required variables
    rate = 10416667;       //125e6/12
    freq = 3555000000;       //3.555 GHz
    gain = 80;

    data.lat = 38.88095833926984;
    data.lon = -77.11573962785668;
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        // hardware parameters
        ("rate", po::value<double>(&rate), "rate of incoming samples (sps)")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        // display parameters
        ("num-bins", po::value<size_t>(&len)->default_value(512), "the number of bins in the DFT")
        ("frame-rate", po::value<double>(&frame_rate)->default_value(1000), "frame rate of the display (fps)")
        ("ref-lvl", po::value<float>(&ref_lvl)->default_value(0), "reference level for the display (dB)")
        ("dyn-rng", po::value<float>(&dyn_rng)->default_value(60), "dynamic range for the display (dB)")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
        ("step", po::value<double>(&step)->default_value(1e6), "tuning step for rate/bw/freq")
        ("show-controls", po::value<bool>(&show_controls)->default_value(true), "show the keyboard controls")
        ("int-n", "tune USRP with integer-N tuning")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help") or not vm.count("rate")) {
        std::cout << boost::format("UHD RX ASCII Art DFT %s") % desc << std::endl;
        return EXIT_FAILURE;
    }

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args
              << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    // Lock mboard clocks
    if (vm.count("ref")) {
        usrp->set_clock_source(ref);
    }

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev"))
        usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    // set the sample rate
    if (not vm.count("rate")) {
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the center frequency
    if (not vm.count("freq")) {
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq / 1e6) << std::endl;
    uhd::tune_request_t tune_request(freq);
    if (vm.count("int-n"))
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_rx_freq(tune_request);
    std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq() / 1e6)
              << std::endl
              << std::endl;

    // set the rf gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain()
                  << std::endl
                  << std::endl;
    } else {
        gain = usrp->get_rx_gain();
    }

    // set the analog frontend filter bandwidth
    if (vm.count("bw")) {
        std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw / 1e6)
                  << std::endl;
        usrp->set_rx_bandwidth(bw);
        std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                         % (usrp->get_rx_bandwidth() / 1e6)
                  << std::endl
                  << std::endl;
    } else {
        bw = usrp->get_rx_bandwidth();
    }

    // set the antenna
    if (vm.count("ant"))
        usrp->set_rx_antenna(ant);

    std::this_thread::sleep_for(std::chrono::seconds(1)); // allow for some setup time

    // Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_rx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    // create a receive streamer
    uhd::stream_args_t stream_args("fc32"); // complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    std::vector<std::complex<float>> buff(len);
    //------------------------------------------------------------------
    //-- Initialize
    //------------------------------------------------------------------
    //initscr(); // curses init
    rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    auto next_refresh = high_resolution_clock::now();
    auto data_sent_time = high_resolution_clock::now();

    //------------------------------------------------------------------
    //-- Main loop
    //------------------------------------------------------------------
    
    int i = 0;
    bool new_data_available = false;
    while (true) {
        // read a buffer's worth of samples every iteration
        size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md);
        if (num_rx_samps != buff.size())
            continue;

        // // check and update the display refresh condition
        if (high_resolution_clock::now() < next_refresh) {
            continue;
        }
        next_refresh = high_resolution_clock::now()
                       + std::chrono::microseconds(int64_t(1e6 / frame_rate));

        // calculate the dft
        esc_dft::log_pwr_dft_type lpdft(
            esc_dft::log_pwr_dft(&buff.front(), num_rx_samps));

        // re-order the dft so dc in in the center
        const size_t len = lpdft.size() - 1 + lpdft.size() % 2; // make it odd
        esc_dft::log_pwr_dft_type dft(len);
        for (size_t n = 0; n < len; n++) {
            dft[n] = lpdft[(n + len / 2) % len];
        }
        // int64_t average = 0;
        // std::cout << " Ch = " << i+1;
        // for(int i = 0; i < dft.size(); i++){
        //     average = (average + dft[i])/2;
        // }
        // std::cout << " " << average;
        // data.channel_pwr[i] = average;
        // i+=1;
        // if(i > 14){
        //     post_power_data(data, "https://10.147.20.60:1443/sas-api/measurements");
        //     post_iq_data(&buff, len, 14, "https://10.147.20.60:1443/sas-api/samples");
        //     i = 0;
        //     freq = 3555e6;   //Set to channel one center back i.e. 3.555 GHz 
        //     std::cout << "\n";
        // }
        // else{
        //     freq += 10e6;    //Increment frequency by 10 MHz to observe the next channel
        //     set_center_frequency(freq, usrp, vm);
        // }
        compute_average_on_bins(dft.data(), len);
       
        if(high_resolution_clock::now() > data_sent_time){
            data_sent_time  = high_resolution_clock::now()
                       + std::chrono::microseconds(int64_t(500e3));
            post_power_data(data, "https://10.147.20.60:1443/sas-api/measurements");
        }
       
    }

    //------------------------------------------------------------------
    //-- Cleanup
    //------------------------------------------------------------------
    rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    endwin(); // curses done

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

void compute_average_on_bins(float *dft, size_t len){
    // Calculate number of averages we want to take
    int num_averages = 15;

    // Calculate average of first 34 elements
    for (int i = 0; i < num_averages-1; i++) {
        double sum = 0;
        for (int j = i*34 + 1; j < (i+1)*34 + 1; j++) {
            sum = sum + dft[j];
        }
        data.channel_pwr[i] = sum / 34.0;
        std::cout << " Ch = " << i;
        std::cout << " " << data.channel_pwr[i];
    }

    // Calculate average of last 34 elements, excluding the last element
    double sum = 0;
    for (int i = len-35; i < len-1; i++) {
        sum += dft[i];
    }
    data.channel_pwr[num_averages-1] = sum / 34.0;
    std::cout << " Ch = " << 14;
    std::cout << " " << data.channel_pwr[num_averages-1];
    std::cout << "\n";

}

//Function to set the center frequency via the UHD driver
void set_center_frequency(uint32_t freq, uhd::usrp::multi_usrp::sptr usrp, po::variables_map vm){
    uhd::tune_request_t tune_request(freq);
    if (vm.count("int-n"))
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_rx_freq(tune_request);
    //std::cout << boost::format("RX Freq: %f MHz...") % (usrp->get_rx_freq() / 1e6);
}

//Function to send HTTPS post request for all the power values
void post_power_data(channel_data data, std::string url) {
    // Construct the JSON payload
    std::stringstream json_ss;
    json_ss << "{";
    json_ss << "\"sensor_id\": \"CCI-xG-Sensor-01\"," ;
    json_ss << "\"lat\":" << data.lat << ",";
    json_ss << "\"lon\":" << data.lon << ",";
    json_ss << "\"channels\":[";
    for (int i = 0; i < 14; i++) {
        json_ss << "{\"name\":\"channel" << i << "\",\"power\":" << data.channel_pwr[i] << "},";
    }
    json_ss << "{\"name\":\"channel14\",\"power\":" << data.channel_pwr[14] << "}";
    json_ss << "]}";

    std::string json_str = json_ss.str();

    // Construct the curl command
    std::string command = "curl -X POST -H 'Content-Type: application/json' --data '" + json_str + "' --cert " + client_crt_path + " --key " + client_key_path + " --cacert " + ca_crt_path + " " + url;

    // Execute the curl command
    system(command.c_str());
}

//Function to send HTTPS post request for the IQ samples for further processing if a power level is above a threshold.
 void post_iq_data(std::vector<std::complex<float>> *buff, size_t len, uint8_t channel, std::string url){
    std::stringstream json_ss;
    json_ss << "{";
    json_ss << "\"sensor_id\": \"CCI-xG-Sensor-01\"," ;
    json_ss << "\"lat\":" << data.lat << ",";
    json_ss << "\"lon\":" << data.lon << ",";

    json_ss << "\"iq_samples\":[";
    for (int i = 0; i < len - 1; i++) {
        json_ss << "[" << buff->at(i).real() << "," << buff->at(i).imag() << "],";
    }
    json_ss << "[" << buff->at(len - 1).real() << "," << buff->at(len - 1).imag() << "]";
    json_ss << "]}";

    std::string json_str = json_ss.str();

    // Construct the curl command
    std::string command = "curl -X POST -H 'Content-Type: application/json' --data '" + json_str + "' --cert " + client_crt_path + " --key " + client_key_path + " --cacert " + ca_crt_path + " " + url;

    // Execute the curl command
    system(command.c_str());

}