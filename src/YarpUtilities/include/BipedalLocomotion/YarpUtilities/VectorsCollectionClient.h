/**
 * @file VectorsCollectionClient.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT), 2026 Generative Bionics S.R.L.
 * This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_YARP_UTILITIES_VECTORS_COLLECTION_CLIENT_H
#define BIPEDAL_LOCOMOTION_YARP_UTILITIES_VECTORS_COLLECTION_CLIENT_H

// std
#include <vector>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionMetadata.h>

#include <iDynTree/Span.h>

namespace BipedalLocomotion
{

namespace YarpUtilities
{

/**
 * VectorsCollectionClient is a class that implements that allows to receive a VectorsCollection
 * from a VectorsCollectionServer.
 */
class VectorsCollectionClient
{
public:
    /**
     * Constructor.
     */
    VectorsCollectionClient();

    /**
     * Destructor.
     */
    virtual ~VectorsCollectionClient();

    /**
     * Initialize the server.
     * @param handler pointer to the parameters handler.
     * @note The following parameters are required:
     * |   Parameter Name   |   Type   |                                   Description                                   |
     * |:------------------:|:--------:|:-------------------------------------------------------------------------------:|
     * |        local       |  string  |                          Name of the local port.                                |
     * |        remote      |  string  |                          Name of the remote port.                               |
     * |       carrier      |  string  |                           Name of the carrier.                                  |
     * @return true if the server has been initialized successfully, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Connect to the ports.
     * @note The cached metadata are discarded, since the server may be a new instance.
     * @return true if the connection has been established successfully, false otherwise.
     */
    bool connect();

    /**
     * Disconnect from the ports.
     * @return true if the connection has been closed successfully, false otherwise.
     */
    bool disconnect();

    /**
     * Check if the data connection with the server is still alive.
     * @note This function queries the network, so it should not be called in a real-time loop.
     * @return true if the client is connected to the server, false otherwise.
     */
    bool isConnected() const;

    /**
     * Check if new metadata is available.
     * @return true if new metadata is available, false otherwise.
     * @note it requires that the client is connected and that at least one data or metadata has
     * been read from the server.
     */
    bool isNewMetadataAvailable() const;

    /**
     * Get the metadata.
     * @param metadata metadata of the vectors collection.
     * @return true if the metadata has been retrieved successfully, false otherwise.
     */
    bool getMetadata(BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);

    /**
     * Read the data from the port.
     * @param shouldWait if true the function will wait until the data is available.
     * @return a pointer to the VectorsCollection. The ownership of the pointer is controlled by the
     * yarp port.
     */
    BipedalLocomotion::YarpUtilities::VectorsCollection* readData(bool shouldWait = true);

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};
} // namespace YarpUtilities
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_YARP_UTILITIES_VECTORS_COLLECTION_SERVER_H
