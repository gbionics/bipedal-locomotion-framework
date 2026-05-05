/**
 * @file YarpRobotLoggerDeviceCommands.thrift
 * @authors Lorenzo Moretti <lorenzo.moretti@iit.it>
 * @copyright 2024 Fondazione Istituto Italiano di Tecnologia
 *            Released under the terms of the BSD-3-Clause license.
 * @date 2024
 */

service YarpRobotLoggerDeviceCommands
{
    /**
     * Start recording data. Transitions the device from Idle to Recording state.
     * Connects to exogenous signals, starts camera threads, and begins data collection.
     * @return true if the transition was successful, false otherwise.
     */
    bool startRecording();

    /**
     * Save a checkpoint of the recorded data without stopping the recording.
     * The device remains in Recording state and continues collecting data.
     * @param tag optional tag to append to the filename.
     * @return true if the save was successful, false otherwise.
     */
    bool saveRecording(1: string tag = "");

    /**
     * Save all recorded data and stop recording. Transitions from Recording to Idle.
     * Disconnects all exogenous signal ports and clears metadata.
     * @param tag optional tag to append to the filename.
     * @return true if the save and stop was successful, false otherwise.
     */
    bool saveAndStopRecording(1: string tag = "");

    /**
     * Discard all recorded data and stop recording. Transitions from Recording to Idle.
     * Disconnects all exogenous signal ports and clears metadata without saving.
     * @return true if the discard was successful, false otherwise.
     */
    bool discardRecording();

    /**
     * Get the current state of the device.
     * @return a string representing the current state ("Idle", "Recording", or "Saving").
     */
    string getState();
}
