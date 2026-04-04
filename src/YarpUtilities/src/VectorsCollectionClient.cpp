/**
 * @file VectorsCollectionClient.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

 #include <BipedalLocomotion/TextLogging/Logger.h>
 #include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>
 #include <BipedalLocomotion/YarpUtilities/VectorsCollectionMetadataService.h>
 #include <BipedalLocomotion/YarpUtilities/VectorsCollectionMetadata.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include <unordered_set>

using namespace BipedalLocomotion::YarpUtilities;

struct VectorsCollectionClient::Impl
{
    yarp::os::BufferedPort<VectorsCollection> port; /**< Buffered port used to communicate with
                                                            the server. */
    yarp::os::Port rpcPort; /**< RPC port used to communicate with the server. */
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadataService rpcInterface;

    std::string localRpcPortName; /**< Name of the local rpc port. */
    std::string localPortName; /**< Name of the local port. */
    std::string remoteRpcPortName; /**< Name of the remote rpc port. */
    std::string remotePortName; /**< Name of the remote port. */
    std::string carrier; /**< Carrier used to connect the port. */

    int cachedVersion{-1}; /**< Cached version of the metadata. */
    bool newMetadataAvailable{false}; /**< True if new metadata is available.*/
    VectorsCollectionMetadata cachedMetadata; /**< Cached metadata. */

    bool isConnected{false}; /**< True if the client is connected. */

    bool updateMetadata(int fromVersion); /**< Update the cached metadata. */
};

VectorsCollectionClient::VectorsCollectionClient()
{
    m_pimpl = std::make_unique<Impl>();
}

VectorsCollectionClient::~VectorsCollectionClient() = default;

bool VectorsCollectionClient::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[VectorsCollectionClient::initialize]";
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is nullptr.", logPrefix);
        return false;
    }

    // open the port
    std::string localPort;
    if (!ptr->getParameter("local", localPort))
    {
        log()->error("{} Unable to retrieve the port prefix.", logPrefix);
        return false;
    }

    std::string remotePort;
    if (!ptr->getParameter("remote", remotePort))
    {
        log()->error("{} Unable to retrieve the port prefix.", logPrefix);
        return false;
    }

    m_pimpl->remotePortName = remotePort + "/measures:o";
    m_pimpl->remoteRpcPortName = remotePort + "/rpc:i";

    m_pimpl->localPortName = localPort + "/measures:i";
    if (!m_pimpl->port.open(m_pimpl->localPortName))
    {
        log()->error("{} Unable to open the port named {}.", //
                     logPrefix,
                     m_pimpl->localPortName);
        return false;
    }

    // open the rpc port
    m_pimpl->localRpcPortName = localPort + "/rpc:o";
    if (!m_pimpl->rpcPort.open(m_pimpl->localRpcPortName))
    {
        log()->error("{} Unable to open the rpc port named {}.",
                     logPrefix,
                     m_pimpl->localRpcPortName);
        return false;
    }

    if (!ptr->getParameter("carrier", m_pimpl->carrier))
    {
        log()->error("{} Unable to retrieve the port prefix.", logPrefix);
        return false;
    }

    return true;
}

bool VectorsCollectionClient::disconnect()
{
    if (!m_pimpl->isConnected)
    {
        return true;
    }

    if (!yarp::os::Network::disconnect(m_pimpl->remotePortName,
                                       m_pimpl->localPortName)
        || !yarp::os::Network::disconnect(m_pimpl->localRpcPortName, //
                                          m_pimpl->remoteRpcPortName))
    {
        return false;
    }

    m_pimpl->isConnected = false;
    return true;
}

bool VectorsCollectionClient::isConnected() const
{
    return m_pimpl->isConnected;
}

bool VectorsCollectionClient::checkConnection()
{
    if (!m_pimpl->isConnected)
    {
        return false;
    }

    // Actively verify that both YARP connections (data and RPC) are still alive
    const bool dataConnected
        = yarp::os::Network::isConnected(m_pimpl->remotePortName, m_pimpl->localPortName);
    const bool rpcConnected
        = yarp::os::Network::isConnected(m_pimpl->localRpcPortName, m_pimpl->remoteRpcPortName);

    if (!dataConnected || !rpcConnected)
    {
        log()->warn("[VectorsCollectionClient::checkConnection] Connection lost. Data port "
                    "connected: {}, RPC port connected: {}.",
                    dataConnected,
                    rpcConnected);
        m_pimpl->isConnected = false;
        return false;
    }

    return true;
}

bool VectorsCollectionClient::connect()
{
    constexpr auto rpcCarrier = "tcp";
    m_pimpl->isConnected = false;

    if (!yarp::os::Network::connect(m_pimpl->remotePortName,
                                    m_pimpl->localPortName,
                                    m_pimpl->carrier)
        || !yarp::os::Network::connect(m_pimpl->localRpcPortName, //
                                       m_pimpl->remoteRpcPortName,
                                       rpcCarrier))
    {
        return false;
    }

    if (!m_pimpl->rpcInterface.yarp().attachAsClient(m_pimpl->rpcPort))
    {
        return false;
    }

    m_pimpl->isConnected = true;

    return true;
}

bool VectorsCollectionClient::isNewMetadataAvailable() const
{
    return m_pimpl->newMetadataAvailable;
}

bool VectorsCollectionClient::getMetadata(
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata)
{
    if (!m_pimpl->isConnected)
    {
        log()->error("[VectorsCollectionClient::getMetadata] Please call connect before asking for "
                     "the metadata");
        return false;
    }

    if (m_pimpl->cachedVersion < 0 || m_pimpl->newMetadataAvailable)
    {
        if (!m_pimpl->updateMetadata(m_pimpl->cachedVersion))
        {
            log()->error("[VectorsCollectionClient::getMetadata] Unable to retrieve the metadata.");
            return false;
        }
        m_pimpl->newMetadataAvailable = false;
    }

    metadata = m_pimpl->cachedMetadata;
    return true;
}

BipedalLocomotion::YarpUtilities::VectorsCollection*
VectorsCollectionClient::readData(bool shouldWait /*= true */)
{
    auto data = m_pimpl->port.read(shouldWait);

    if (data == nullptr)
    {
        return nullptr;
    }

    // Check if the received data has a newer metadata version
    int receivedVersion = data->version;
    if (receivedVersion >= 0 && receivedVersion > m_pimpl->cachedVersion)
    {
        if (!m_pimpl->newMetadataAvailable) // Only log on the first detection
        {
            log()->warn("[VectorsCollectionClient::readData] Received data with new metadata "
                        "version {} (previous {}). New metadata is available.",
                        receivedVersion,
                        m_pimpl->cachedVersion);
        }
        m_pimpl->newMetadataAvailable = true;
    }

    return data;
}

bool VectorsCollectionClient::Impl::updateMetadata(int fromVersion)
{
    constexpr auto logPrefix = "[VectorsCollectionClient::updateMetadata]";

    if (!isConnected)
    {
        log()->error("{} The client is not connected.", logPrefix);
        return false;
    }

    auto delta = rpcInterface.getMetadataIncremental(fromVersion);

    if (delta.version < 0 && delta.vectors.empty())
    {
        log()->warn("{} Metadata not available yet.", logPrefix);
        return false;
    }

    if (delta.version < cachedVersion)
    {
        log()->warn("{} Received older metadata version {} (cached {}).",
                    logPrefix,
                    delta.version,
                    cachedVersion);
        return false;
    }

    for (const auto& [key, value] : delta.vectors)
    {
        cachedMetadata.vectors[key] = value;
    }

    cachedVersion = delta.version;
    cachedMetadata.version = delta.version;

    log()->info("{} Updated metadata to version {}.", logPrefix, cachedVersion);
    return true;
}
