#!/usr/bin/env bash

# Parse command line arguments
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        # Parse the input port for the audio device
        -i|--input-port-audio)
        input_port_audio="$2"
        shift
        shift
        ;;
        # Parse the launch file for the logger device
        -l|--launch-file)
        launch_file="$2"
        shift
        shift
        ;;
        -h|--help|*)
        # Display help text
        echo "Usage: $0 [OPTIONS]"
        echo "Options:"
        echo "  -i, --input-port-audio    Specify the input port for the audio device (default: /icub/microphone/audio:o)"
        echo "  -l, --launch-file         Specify the launch file for the logger device (default: launch-yarp-robot-logger.xml)"
        echo "  -h, --help                Display this help message"
        exit 0
        ;;
    esac
done

# Set default values if not provided
input_port_audio=${input_port_audio:-/icub/microphone/audio:o}
launch_file=${launch_file:-launch-yarp-robot-logger.xml}

# Function to handle SIGINT signal
function handle_sigint {
    echo "Received SIGINT signal. Forwarding to background programs..."
    # Forward SIGINT to the background program
    kill -SIGINT $bg_pid1 $bg_pid2
}

function handle_exit {

    echo "Loking fro the audio file. It may take a while."
    until [ -f audio_out.wav ]
    do
        sleep 5
    done

    echo "Renaming the audio"

    # get the latest matfile name
    latest_file=$(find . -maxdepth 1 -type f -name "*.mat" -printf "%T@ %f\n" | sort -nr | head -n 1 | awk '{print $2}' | sed 's/\.mat$//')
    mv audio_out.wav $latest_file.wav

    echo "Closing"

    exit
}

# launch the audio device recorder
yarpdev --device AudioPlayerWrapper --name /YarpRobotLogger/audio --subdevice audioToFileDevice --start --file_name audio_out.wav --save_mode overwrite_file &
bg_pid1=$!
yarp wait /YarpRobotLogger/audio/audio:i
yarp connect $input_port_audio /YarpRobotLogger/audio/audio:i fast_tcp

# launch the logger device
yarprobotinterface --config $launch_file &
bg_pid2=$!

# register the signal handlers
trap handle_sigint SIGINT
trap handle_exit EXIT

# wait for closing the application
wait "$bg_pid1"
bg_exit_status1=$?

wait "$bg_pid2"
bg_exit_status2=$?
