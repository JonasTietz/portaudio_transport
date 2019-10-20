#include "ros/ros.h"

#include "portaudio_transport/AudioTransport.h"
#include <sndfile.h>

//! Main function of audiofile_transport_publisher node
/*!
  Initializes a node, which publishes audio data from a file as if it was recorded by a microphone
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "audiofile_transport_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<portaudio_transport::AudioTransport>("/portaudio_transport", 1);
    // Ensure correct size of float data type
    assert(CHAR_BIT * sizeof (float) == 32);

    int frame_rate = 0;
    int frame_size = 0;
    int max_channels = 0;
    std::string file_name;

    // Parse parameters from command line
    // get name of the audio file
    if (nh.getParam("file_name", file_name)) {
        ROS_INFO("Using file: %s", file_name.c_str());
    }
    if (file_name == ""){
        ROS_WARN("Please specify a file to be streamed.");
        return 0;
    }

    // _frame_rate can be specified and must be non-zero
    if (nh.getParam("frame_rate", frame_rate)) {
        ROS_INFO("Using frame rate: %d", frame_rate);
    }
    // If _frame_rate was not parsed, parse the _frame_size, otherwise ignore the frame_size
    if (frame_rate == 0) {
        if (nh.getParam("frame_size", frame_size)) {
            ROS_INFO("Using frame size: %d", frame_size);
        }
    } else {
        ROS_WARN("Frame rate already specified, ignoring input for frame size");
    }

    // _max_channels limits the channels used from the audio file.
    // Input channels with higher index will not be used.
    if (nh.getParam("max_channels", max_channels)) {
        ROS_INFO("Limiting input channels to %d", max_channels);
    }

    // open the audio file
    SF_INFO file_info;
    file_info.format = 0;
    SNDFILE *file = sf_open(file_name.c_str(), SFM_READ, &file_info);
    if (!file) {
        ROS_ERROR("Can not open file %s", file_name.c_str());
        return 0;
    }

    double sample_rate = file_info.samplerate;
    int input_channels = file_info.channels;
    ROS_INFO_STREAM("Using file " << file_name << " with " << input_channels << " channels available.");

    // Calculate frame_rate or frame_size depending on sample frequency and the frame_size or frame_rate accordingly
    if (frame_rate != 0) {
        frame_size = sample_rate / frame_rate;
    } else if (frame_size != 0) {
        frame_rate = sample_rate / frame_size;
    } else {
        ROS_WARN("Neither _frame_size nor _frame_rate specified. Using default frame rate of 100");
        frame_rate = 100;
        frame_size = sample_rate / frame_rate;
    }

    // Limit the input channels if the device input channels exceed the maximum channels
    int used_channels = input_channels;
    if (max_channels != 0) {
        if (input_channels > max_channels) {
            ROS_WARN("Only transporting %d of %d input channels", max_channels, input_channels);
        }
        used_channels = std::min(input_channels, max_channels);
    }

    ROS_INFO("Initialized transport publisher with [%d] channels, frame_size [%d], sample frequency [%.0lfHz] and a resulting frame rate [%dHz]", used_channels, frame_size, sample_rate, frame_rate);

    ros::Rate rate(frame_rate);

    // allocating buffer for reading the audio data
    float *buffer = (float*) malloc(sizeof(float)*frame_size*input_channels);

    // init transport msg
    portaudio_transport::AudioTransport transport;
    transport.channel_count = used_channels;
    transport.frame_size = frame_size;
    transport.sample_frequency = sample_rate;
    transport.channels.resize(transport.channel_count);
    for (int c_channel = 0; c_channel < transport.channel_count; c_channel++) {
        transport.channels[c_channel].frame_data.resize(transport.frame_size);
    }

    while ((nh.ok()) && (ros::ok)) {
        sf_count_t samples_read = sf_readf_float(file, buffer, frame_size);

        for(int channel = 0; channel < used_channels; channel++ ) {
            transport.channels[channel].frame_data.clear();
            for(int i = 0; i < frame_size; i+= input_channels) {
                transport.channels[channel].frame_data.push_back(buffer[i+channel]);
            }
        }

        // right now this publishes a zero padded msg as the last msg
        pub.publish(transport);

        rate.sleep(); // TODO: does this need to be more precise
        if (samples_read < frame_size) {
            ROS_INFO("Finished transmitting file.");
            break;
        }
    }

    sf_close(file);
    return 0;
}
