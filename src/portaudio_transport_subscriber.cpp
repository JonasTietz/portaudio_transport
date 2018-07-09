#include "ros/ros.h"

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <portaudio_transport/playback_subscriber.h>

#include <portaudiocpp/PortAudioCpp.hxx>

#include "portaudio_transport/AudioChannel.h"
#include "portaudio_transport/AudioTransport.h"

PlaybackSubscriber *PlaybackSubscriberPtr;

void PrintOuputDevices(portaudio::System& audioSys) {
    std::stringstream available_devices;
    available_devices << "Available Playback Devices" << std::endl;
    available_devices << "\t*---*-------------------------------------------------------------*---------*------------*" << std::endl;
    available_devices << "\t| # | Name                                                        | Outputs | Samplerate |" << std::endl;
    available_devices << "\t*---*-------------------------------------------------------------*---------*------------*" << std::endl;
    boost::format entry_line = boost::format("|%3i| %|-60|| %|=8|| %|=11|| ");
    for (portaudio::System::DeviceIterator device = audioSys.devicesBegin(); device != audioSys.devicesEnd(); ++device) {
        std::string deviceOptions = "";
        if (device->maxOutputChannels() < 1) { continue; }
        if (device->isSystemDefaultOutputDevice()) { deviceOptions += "!"; }
        if (device->isHostApiDefaultOutputDevice()) { deviceOptions += "*"; }
        available_devices << "\t" << entry_line % device->index() % device->name() % device->maxOutputChannels() % device->defaultSampleRate() << deviceOptions << std::endl;
    }
    available_devices << "\t*---*-------------------------------------------------------------*---------*------------*" << std::endl;
    available_devices << "\t      is default output device: system->!  hostAPI->*";
    ROS_INFO_STREAM(available_devices.str());
}

void AudioCallback(const portaudio_transport::AudioTransport::ConstPtr& msg) {
    PlaybackSubscriberPtr->AddToBuffer(msg->channels);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "portaudio_transport_subscriber");
    ros::NodeHandle nh("~");
    assert(CHAR_BIT * sizeof (float) == 32);

    std::string output_device_name = "";
    int output_device_id = 0;
    int stream_channel_count = 0;
    int stream_frame_size = 0;
    double stream_sample_frequency = 0;
    double device_sample_frequency = 0;
    int device_output_channels = 0;

    ros::Subscriber audio_subscriber;

    if (nh.getParam("output_device", output_device_name)) {
        ROS_INFO("Matching output device name for: %s" , output_device_name.c_str());
        boost::algorithm::to_lower(output_device_name);

    } else if (nh.getParam("output_device", output_device_id)) {
        ROS_INFO("Using output device: %d", output_device_id);
    } else {
        ROS_WARN("_output_device not specified, using system default");
        output_device_name = "default";
    }

    // Initialize audio system
    portaudio::System::initialize();
    ROS_INFO("Initialized %s", portaudio::System::versionText());

    portaudio::System &audioSys = portaudio::System::instance();
    PrintOuputDevices(audioSys);

    // Select output device by ID or name
    if (output_device_id != 0) {
        if (audioSys.deviceByIndex(output_device_id).maxOutputChannels() < 1) {
            ROS_ERROR("Output device is not a recording device. Exiting...");
            return 0;
        }
    } else if (!output_device_name.empty()) {
        for (portaudio::System::DeviceIterator device = audioSys.devicesBegin(); device != audioSys.devicesEnd(); ++device) {
            if (device->maxOutputChannels() < 1) { continue; }
            std::string device_name = device->name();
            boost::algorithm::to_lower(device_name);
            if (device_name.find(output_device_name) != std::string::npos) {
                output_device_id = device->index();
                break;
            }
        }
    }
    if (output_device_id == 0) {
        ROS_ERROR("No matching output device found. Exiting...");
        return 0;
    }

    portaudio_transport::AudioTransport first_audio_message;

    int wait_for_message_timeout = 2;
    while ((stream_channel_count == 0) && (stream_frame_size == 0) && (stream_sample_frequency == 0) && (nh.ok()) && (ros::ok)) {
        portaudio_transport::AudioTransportConstPtr first_audio_message_ptr = ros::topic::waitForMessage<portaudio_transport::AudioTransport>("/portaudio_transport", ros::Duration(std::min(10,(wait_for_message_timeout/2))));
        wait_for_message_timeout++;
        if (first_audio_message_ptr == NULL) {
            ROS_WARN("Waiting for /portaudio_transport message");
        } else {
            stream_channel_count = first_audio_message_ptr->channel_count;
            stream_frame_size = first_audio_message_ptr->frame_size;
            stream_sample_frequency = first_audio_message_ptr->sample_frequency;
            ROS_INFO("Received first /portaudio_transport message, initializing playback device...");
        }
    }
    ROS_INFO_STREAM("Found audio stream with " << stream_channel_count << " channels, " << stream_sample_frequency << "Hz sample frequency and " << stream_frame_size << " samples per frame");

    portaudio::Device& playbackDevice = audioSys.deviceByIndex(output_device_id);
    device_sample_frequency = playbackDevice.defaultSampleRate();
    device_output_channels = playbackDevice.maxOutputChannels();
    ROS_INFO_STREAM("Using device #" << playbackDevice.index() << " (" << playbackDevice.name() << ") with " << playbackDevice.maxOutputChannels() << " output channels available and " << playbackDevice.hostApi().name() << " hostAPI");

    // Create the playback Subscriber
    // required:
    // stream_channel_count, stream_frame_size, stream_sample_frequency
    // device_output_channels, device_sample_frequency
    // optionally: buffer size
//    PlaybackSubscriber objPlaybackSubscriber(stream_channel_count, stream_frame_size, stream_sample_frequency, device_output_channels, device_sample_frequency);
//    PlaybackSubscriberPtr = &objPlaybackSubscriber;


    PlaybackSubscriberPtr = new PlaybackSubscriber(stream_channel_count, stream_frame_size, stream_sample_frequency, device_output_channels, device_sample_frequency);


    portaudio::DirectionSpecificStreamParameters outParamsPlayback(playbackDevice, device_output_channels, portaudio::FLOAT32, false, playbackDevice.defaultLowOutputLatency(), NULL);
    portaudio::StreamParameters paramsPlayback(portaudio::DirectionSpecificStreamParameters::null(), outParamsPlayback, device_sample_frequency, stream_frame_size, paClipOff);
    portaudio::MemFunCallbackStream<PlaybackSubscriber> streamPlayback(paramsPlayback, *PlaybackSubscriberPtr, &PlaybackSubscriber::PlaybackCallback);

    audio_subscriber  = nh.subscribe("/portaudio_transport", 10, AudioCallback);

    ros::Duration(0.5).sleep();
    ROS_INFO("Starting streamPlayback");
    streamPlayback.start();
    ROS_INFO("Started streamPlayback");
    /*while (streamPlayback.isActive())
        sys.sleep(100);
    streamPlayback.stop();*/
    //while ((nh.ok() && ros::ok)) {
        ros::spin();
    //}
}