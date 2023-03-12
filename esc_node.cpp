//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

//Modified by Oren Rodney Collaco

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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <mutex>

#define SENSOR_NODE 1

// Paths to the certificates,keys and the url of the OpenSAS server
std::string client_crt_path = "../certs/client_10.147.20.75-0.crt"; 
std::string client_key_path = "../certs/client_10.147.20.75-0.key"; 
std::string ca_crt_path = "../certs/ca.crt";

std::string opensas_url = "https://10.147.20.75:1443/sas-api/";

#define FFT_ON_FPGA 0

// #define DEBUG 1
#define DEBUG 0

#define STATS 1
// #define STATS 0

#define STATS_FFT 0
// #define STATS_FFT 0

// based on the input shape the model will be trained to detect
#define DETECTION_SAMPLE_SIZE 102400

// the threshold for the detection
#define DETECTION_THRESHOLD   -70

#if SENSOR_NODE == 1
#define SENSOR_ID "xG-OpenSense-Node1"
#define SENSOR_LAT 38.88089743634038
#define SENSOR_LON -77.11569278866236

#elif SENSOR_NODE == 2
#define SENSOR_ID "xG-OpenSense-Node2"
#define SENSOR_LAT 38.880863506784166 
#define SENSOR_LON -77.11578465431765

#elif SENSOR_NODE == 3
#define SENSOR_ID "xG-OpenSense-Node3"
#define SENSOR_LAT 38.88099087303452 
#define SENSOR_LON -77.11574442118396

#elif SENSOR_NODE == 4
#define SENSOR_ID "xG-OpenSense-Node4"
#define SENSOR_LAT 38.88091988203791
#define SENSOR_LON 77.11583494573478
#endif


namespace po = boost::program_options;
using std::chrono::high_resolution_clock;

// struct to hold the channel power data and location
struct channel_data {
    float channel_pwr[15];
    double lat;
    double lon;
};

struct channel_data data;

std::mutex curl_mutex;

void post_power_data(channel_data data, std::string url);

void post_iq_data(std::vector<std::complex<float>>& buff, size_t len, uint8_t channel, std::string url);

void post_iq_data_nocurl(std::vector<std::complex<float>>& buff, size_t len, uint8_t channel, std::string url);

void post_json(std::string json_str, std::string url);

int compute_average_on_bins(float *dft, size_t len);

void set_center_frequency(uint32_t freq, uhd::usrp::multi_usrp::sptr usrp, po::variables_map vm);

double get_center_freq(int channel);

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    //initialize channel power data
    data.lat = SENSOR_LAT;
    data.lon = SENSOR_LON;

    //init channel power data
    for (int i = 0; i < 15; i++) {
        data.channel_pwr[i] = -100;
    }
    // variables to be set by po
    std::string args, ant, subdev, ref;
    size_t len;
    double rate, freq, gain, bw, frame_rate, step;
    float ref_lvl, dyn_rng;
    bool show_controls;

    // //initialize required variables
    // rate = 10416667;       //125e6/12
    // freq = 3555000000;       //3.555 GHz
    // gain = 80;

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
    std::vector<std::complex<float>> detect_buff(DETECTION_SAMPLE_SIZE);

    //------------------------------------------------------------------
    //-- Initialize
    //------------------------------------------------------------------
    //initscr(); // curses init

    //Create issue stream command asking for buf samples
    uhd::stream_cmd_t stream_cmd_normal(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE);
    stream_cmd_normal.num_samps = size_t(buff.size());
    stream_cmd_normal.stream_now = true;
    stream_cmd_normal.time_spec  = uhd::time_spec_t();

    //Create issue stream command asking for buf samples
    uhd::stream_cmd_t stream_cmd_detect(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE);
    stream_cmd_detect.num_samps = size_t(detect_buff.size());
    stream_cmd_detect.stream_now = true;
    stream_cmd_detect.time_spec  = uhd::time_spec_t();

    auto next_refresh = high_resolution_clock::now();
    auto data_sent_time = high_resolution_clock::now();
    auto iq_data_sent_time = high_resolution_clock::now();
#if STATS
    auto detection_stats_time = high_resolution_clock::now();
#endif
#if STATS_FFT
    auto fft_stats_time = high_resolution_clock::now();
#endif

    //------------------------------------------------------------------
    //-- Main loop
    //------------------------------------------------------------------
    
    int i = 0;
    bool new_data_available = false;

    //Wait 20 us before starting to stream
    while (high_resolution_clock::now() < next_refresh + std::chrono::microseconds(20)) {
        continue;
    }
    //Verify the waiting time was correct
    auto duration = high_resolution_clock::now() - next_refresh;
    std::cout << "Verifying timing (20us): " << duration.count() / 1000 << " us" << std::endl;
    
    while (true) {
        //Tell USRP to only stream x amount of samples until asked again.
        rx_stream->issue_stream_cmd(stream_cmd_normal);

        // read a buffer's worth of samples every iteration
        size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md);
        if (num_rx_samps != buff.size())
            continue;

        #if DEBUG
        // Print the first 10 IQ samples
        int j = 0;
        while (j < 5) {
            std::cout << "Sample " << j << ": " << buff[j] << std::endl;
            j++;
        }
        #endif

        // // check and update the display refresh condition
        if (high_resolution_clock::now() < next_refresh) {
            continue;
        }
        next_refresh = high_resolution_clock::now()
                       + std::chrono::microseconds(int64_t(1e6 / frame_rate));

        #if STATS_FFT
        fft_stats_time = high_resolution_clock::now();
        #endif
        // calculate the dft
        esc_dft::log_pwr_dft_type lpdft(
            esc_dft::log_pwr_dft(&buff.front(), num_rx_samps));

        // re-order the dft so dc in in the center
        const size_t len = lpdft.size() - 1 + lpdft.size() % 2; // make it odd
        esc_dft::log_pwr_dft_type dft(len);
        for (size_t n = 0; n < len; n++) {
            dft[n] = lpdft[(n + len / 2) % len];
        }
        #if STATS_FFT
        auto fft_stats_duration = (high_resolution_clock::now() - fft_stats_time);
        std::cout << "FFT time: "  << fft_stats_duration.count() / 1000 << " us" << std::endl;
        #endif
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
        // check if any channels are above the threshold

        int detect_channel = compute_average_on_bins(dft.data(), len);

        #if DEBUG
        //print detect channel
        std::cout << "Detect channel: " << detect_channel << std::endl;
        #endif

        if(detect_channel < 0){
            if(high_resolution_clock::now() > data_sent_time){
                data_sent_time  = high_resolution_clock::now()
                        + std::chrono::microseconds(int64_t(250e3));
                #if DEBUG
                //Now send the data to the server
                printf("Sending power meas");
                #endif
                post_power_data(data, opensas_url + "measurements");
            }
        }
        else{
            
            if(high_resolution_clock::now() > iq_data_sent_time){
                size_t num_rx_detect_samps = 0;
                //Change center frequency to the detected channel
                #if STATS
                detection_stats_time = high_resolution_clock::now();
                #endif
                set_center_frequency(get_center_freq(detect_channel), usrp, vm);
                #if STATS
                auto detection_stats_duration = (high_resolution_clock::now() - detection_stats_time);
                std::cout << "Freq shift time: "  << detection_stats_duration.count() / 1000 << " us" << std::endl;
                #endif
                //Change sample rate to 10.24 MHz to capture 1 ms of data for the detected channel
                
                std::cout << boost::format("Setting RX Rate: %f Msps...") % (10.24e6 / 1e6) << std::endl;
                #if STATS
                detection_stats_time = high_resolution_clock::now();
                #endif
                usrp->set_rx_rate(10.24e6);
                #if STATS
                detection_stats_duration = (high_resolution_clock::now() - detection_stats_time);
                std::cout << "Srate change time: "  << detection_stats_duration.count() / 1000 << " us" << std::endl;
                #endif
                std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate() / 1e6)
                        << std::endl
                        << std::endl;
                #if STATS
                detection_stats_time = high_resolution_clock::now();
                #endif
                rx_stream->issue_stream_cmd(stream_cmd_detect);
                while (num_rx_detect_samps < detect_buff.size()) {
                    // Wait for the next buffer of samples
                    num_rx_detect_samps += rx_stream->recv(&detect_buff.front(), detect_buff.size(), md);
                    // Print the number of samples received
                    std::cout << "Received " << num_rx_detect_samps << " samples" << std::endl;
                    if (num_rx_samps != detect_buff.size())
                        continue;
                }
                #if STATS
                detection_stats_duration = (high_resolution_clock::now() - detection_stats_time);
                std::cout << "Samples recv time: "  << detection_stats_duration.count() / 1000 << " us" << std::endl;
                #endif
                
                post_power_data(data, opensas_url + "measurements");
                #if STATS
                detection_stats_time = high_resolution_clock::now();
                #endif
                post_iq_data_nocurl(detect_buff, detect_buff.size(), detect_channel, opensas_url + "samples");
                #if STATS
                detection_stats_duration = (high_resolution_clock::now() - detection_stats_time);
                std::cout << "Https req time: "  << detection_stats_duration.count() / 1000 << " us" << std::endl;
                #endif
                //Change sample rate back to 122.88 MHz
                std::cout << boost::format("Setting RX Rate: %f Msps...") % (122.88e6 / 1e6) << std::endl;
                usrp->set_rx_rate(122.88e6);
                std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate() / 1e6)
                        << std::endl
                        << std::endl;
                iq_data_sent_time  = high_resolution_clock::now()
                        + std::chrono::microseconds(int64_t(50e3));
                #if STATS
                detection_stats_time = high_resolution_clock::now();
                #endif
                set_center_frequency(3650e6, usrp, vm);
                #if STATS
                detection_stats_duration = (high_resolution_clock::now() - detection_stats_time);
                std::cout << "Freq return time: "  << detection_stats_duration.count() / 1000 << " us" << std::endl;
                #endif
            }
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

/*
Computes average on fft points - start_point - end_point_offset bins excluding start_point bins at the beginning and end_point_offset bins at the end, returns -1 if no bins are above threshold,
else returns the index of channel with power above threshold 
*/
int compute_average_on_bins(float *dft, size_t len){
    // Calculate number of averages we want to take
    int num_averages = 15;
    int channel_offset = 5;
    int skip_steps = 1;
    int start_point = 10;
    int end_point_offset = 10;
    int detect_channel = -1;
    float max = -100;
    // Calculate average of first 34 elements
    int step = 41;

    for (int i = channel_offset; i < num_averages-1; i++) {
        double sum = 0;
        for (int j = ((i - channel_offset + skip_steps)  * step) + start_point; j < (i + 1 - channel_offset + skip_steps) * step + start_point; j++) {
            sum = sum + dft[j];
        }
        data.channel_pwr[i] = sum / step;
        #if DEBUG
        std::cout << " Ch = " << i;
        std::cout << " " << data.channel_pwr[i];
        #endif
        // Check if the average is above the threshold
        if(data.channel_pwr[i] > DETECTION_THRESHOLD){
            // Check if the average is greater than the max
            if(data.channel_pwr[i] > max){
                // Update the max and the channel
                max = data.channel_pwr[i];
                detect_channel = i;
            }
        }
    }

    // Calculate average of last 34 elements, excluding the last element
    double sum = 0;
    for (int i = len-(step * (skip_steps+1))-end_point_offset; i < len-(step * (skip_steps)) - end_point_offset; i++) {
        sum += dft[i];
    }
    // Check if the last element is above the threshold
    data.channel_pwr[num_averages-1] = sum / step;
    if(data.channel_pwr[num_averages-1] > -60){
        if(data.channel_pwr[num_averages-1] > max){
            max = data.channel_pwr[num_averages-1];
            detect_channel = num_averages-1;
        }
    }
    #if DEBUG
    std::cout << " Ch = " << 14;
    std::cout << " " << data.channel_pwr[num_averages-1];
    std::cout << "\n";
    #endif
    return detect_channel;
}

/**
 * Calculates the center frequency in MHz for a given channel number.
 * 
 * @param channel The channel number, between 0 and 14 (inclusive).
 * @return The center frequency in MHz.
 */
double get_center_freq(int channel) {
    const double channelWidth = 10.0e6; // The width of each channel in MHz.
    const double baseFreq = 3550.0e6; // The frequency of the first channel in MHz.
    
    // Calculate the center frequency of the specified channel using the formula:
    // centerFreq = baseFreq + (channel * channelWidth) + (channelWidth / 2.0)
    double centerFreq = baseFreq + (channel * channelWidth) + (channelWidth / 2.0);
    
    // Return the center frequency to the caller.
    return centerFreq;
}

//Function to set the center frequency via the UHD driver
void set_center_frequency(uint32_t freq, uhd::usrp::multi_usrp::sptr usrp, po::variables_map vm){
    uhd::tune_request_t tune_request(freq);
    if (vm.count("int-n"))
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_rx_freq(tune_request);
    std::cout << boost::format("RX Freq: %f MHz...\n") % (usrp->get_rx_freq() / 1e6);
}

//Function to send HTTPS post request for all the power values
void post_power_data(channel_data data, std::string url) {
    // Construct the JSON payload
    std::stringstream json_ss;
    json_ss << "{";
    json_ss << "\"sensor_id\":\"" << SENSOR_ID << "\"," ;
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

    post_json(json_str, url);
}

/* 
Function to send HTTPS post request for the IQ samples for further processing if a 
power level is above a threshold.
*/
 void post_iq_data(std::vector<std::complex<float>>& buff, size_t len, uint8_t channel, std::string url){
    std::stringstream json_ss;
    json_ss << "{";
    json_ss << "\"sensor_info\": {";
    json_ss << "\"sensor_id\":\"" << SENSOR_ID << "\"," ;
    json_ss << "\"lat\":" << data.lat << ",";
    json_ss << "\"lon\":" << data.lon << "},";
    json_ss << "\"detected_channel\":" << (int)channel << ",";
    json_ss << "\"iq_samples\":[";
    for (int i = 0; i < len - 1; i++) {
        json_ss << "[" << buff.at(i).real() << "," << buff.at(i).imag() << "],";
    }
    json_ss << "[" << buff.at(len - 1).real() << "," << buff.at(len - 1).imag() << "]";
    json_ss << "]}";

    std::string json_str = json_ss.str();

    //Print the JSON string
    //std::cout << json_str << std::endl;

    //Nofity the user that the data is being sent
    std::cout << "Sending 100k IQ samples to OpenSAS..." << std::endl; 

    // Construct the curl command
    std::string command = "curl -X POST -H 'Content-Type: application/json' --data '" + json_str + "' --cert " + client_crt_path + " --key " + client_key_path + " --cacert " + ca_crt_path + " " + url;

    // Execute the curl command
    system(command.c_str());

    std::cout << command.c_str() << std::endl;

    //Nofity the user that the data has been sent
    std::cout << "Data sent." << std::endl;
}

/* 

 */
void post_iq_data_nocurl(std::vector<std::complex<float>>& buff, size_t len, uint8_t channel, std::string url) {
    std::stringstream json_ss;
    json_ss << "{";
    json_ss << "\"sensor_info\": {";
    json_ss << "\"sensor_id\":\"" << SENSOR_ID << "\"," ;
    json_ss << "\"lat\":" << data.lat << ",";
    json_ss << "\"lon\":" << data.lon << "},";
    json_ss << "\"detected_channel\":" << (int)channel << ",";
    json_ss << "\"iq_samples\":[";
    for (int i = 0; i < len - 1; i++) {
        json_ss << "[" << buff.at(i).real() << "," << buff.at(i).imag() << "],";
    }
    json_ss << "[" << buff.at(len - 1).real() << "," << buff.at(len - 1).imag() << "]";
    json_ss << "]}";

    std::string json_str = json_ss.str();

    #if DEBUG
    //Nofity the user that the data is being sent
    std::cout << "Sending data to server..." << std::endl; 
    #endif

    //Print the JSON string
    // std::cout << json_str << std::endl;

    post_json(json_str, url);
}

void post_json(std::string json_str, std::string url) {

    char url_new[url.length() + 1];
    strcpy(url_new, url.c_str());

    // Find the protocol component
    char *protocol = strtok(url_new, ":/");

    // Find the hostname component
    char *hostname = strtok(NULL, ":/");

    // Find the port component
    char *port = strtok(NULL, "/");

    // Find the path component
    char *path = strtok(NULL, "");
    std::string path_str(path);

    #if DEBUG
    // Print the components
    std::cout << "Protocol: " << protocol << std::endl;
    std::cout << "Hostname: " << hostname << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Path: " << path_str << std::endl;
    #endif
    
    // Initialize OpenSSL
    SSL_library_init();
    SSL_CTX *ssl_ctx = SSL_CTX_new(TLS_client_method());

    // Load the client certificate and key
    if (SSL_CTX_use_certificate_file(ssl_ctx, client_crt_path.c_str(), SSL_FILETYPE_PEM) <= 0) {
        perror("ERROR loading client certificate");
        return;
    }
    if (SSL_CTX_use_PrivateKey_file(ssl_ctx, client_key_path.c_str(), SSL_FILETYPE_PEM) <= 0) {
        perror("ERROR loading client private key");
        return;
    }

    // Load the CA certificate
    if (SSL_CTX_load_verify_locations(ssl_ctx, ca_crt_path.c_str(), nullptr) <= 0) {
        perror("ERROR loading CA certificate");
        return;
    }

    // Create a socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("ERROR opening socket");
        return;
    }

    // Set the server address
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(atoi(port));
    if (inet_pton(AF_INET, hostname, &serv_addr.sin_addr) <= 0) {
        perror("ERROR invalid address");
        return;
    }

    // Connect to the server
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR connecting");
        return;
    }

    // Create an SSL object and attach it to the socket
    SSL *ssl = SSL_new(ssl_ctx);
    SSL_set_fd(ssl, sockfd);

    // Establish the SSL connection
    if (SSL_connect(ssl) != 1) {
        perror("ERROR establishing SSL connection");
        return;
    }

    // Send the HTTP POST request
    std::string post_req = "POST /" + path_str + " HTTP/1.1\r\n";
    post_req += "Host: " + url + "\r\n";
    post_req += "Content-Type: application/json\r\n";
    post_req += "Content-Length: " + std::to_string(json_str.size()) + "\r\n";
    post_req += "\r\n";
    post_req += json_str;
    if (SSL_write(ssl, post_req.c_str(), post_req.size()) < 0) {
        perror("ERROR writing to socket");
        return;
    }

    // Read the response from the server
    char buffer[1024];
    int bytes_read = SSL_read(ssl, buffer, sizeof(buffer));
    if (bytes_read < 0) {
        perror("ERROR reading from socket");
        return;
    }

    // Close the SSL connection and the socket
    SSL_shutdown(ssl);
    SSL_free(ssl);
    SSL_CTX_free(ssl_ctx);
    close(sockfd);
}