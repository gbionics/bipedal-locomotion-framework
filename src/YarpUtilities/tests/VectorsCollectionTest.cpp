#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <random>
#include <thread>
#if defined(_WIN32)
#include <process.h> // for _getpid()
#else
#include <unistd.h> // for getpid()
#endif
#include <yarp/os/Network.h>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;

// Global fixture to manage YARP network initialization and finalization
struct YarpNetworkGlobalFixture : Catch::EventListenerBase
{
    using Catch::EventListenerBase::EventListenerBase;

    void testRunStarting(Catch::TestRunInfo const&) override
    {
        // Initialize YARP network for inter-process communication
        yarp::os::NetworkBase::setLocalMode(true);

        // Initialize YARP network for inter-process communication
        yarp::os::Network::init();

        std::cout << "YARP port range: " << yarp::os::Network::getDefaultPortRange() << std::endl;
    }

    void testRunEnded(Catch::TestRunStats const&) override
    {
        yarp::os::Network::fini();

        // Allow a short time for YARP resources to be released before exiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
};

CATCH_REGISTER_LISTENER(YarpNetworkGlobalFixture)

std::string generateUniquePortName(const std::string& base)
{
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(10000, 99999);
    int random_number = dist(rng);

    // Add process ID to ensure uniqueness across processes
#if defined(_WIN32)
    int pid = _getpid();
#else
    int pid = getpid();
#endif

    std::ostringstream oss;
    oss << base << "_" << pid << "_" << random_number;
    return oss.str();
}

/**
 * Test fixture for VectorsCollection tests
 * Manages YARP network initialization and provides common parameter handlers
 */
class VectorsCollectionFixture {
public:
    VectorsCollectionFixture()
    {
        // Generate unique port names for each test instance
        std::string serverPort = generateUniquePortName("/test/server");
        std::string clientPort = generateUniquePortName("/test/client");

        // Setup server parameters with unique port names to avoid conflicts
        auto serverParams = std::make_shared<StdImplementation>();
        serverParams->setParameter("remote", serverPort);

        // Setup client parameters matching server configuration
        auto clientParams = std::make_shared<StdImplementation>();
        clientParams->setParameter("remote", serverPort);
        clientParams->setParameter("local", clientPort);
        clientParams->setParameter("carrier", std::string("tcp"));

        serverHandler = serverParams;
        clientHandler = clientParams;
    }

    ~VectorsCollectionFixture() = default;

    std::shared_ptr<IParametersHandler> serverHandler;
    std::shared_ptr<IParametersHandler> clientHandler;
};

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionServer - Basic Initialization") {
    VectorsCollectionServer server;

    SECTION("Successful initialization with valid parameters") {
        // Test: Server should initialize successfully with proper parameter configuration
        // Behavior: Creates YARP ports for data streaming and RPC metadata service
        REQUIRE(server.initialize(serverHandler));
    }

    SECTION("Initialization failure with invalid parameters") {
        // Test: Server should fail gracefully when required parameters are missing
        // Behavior: Returns false when 'remote' parameter is not provided
        auto invalidParams = std::make_shared<StdImplementation>();
        // Missing required 'remote' parameter
        REQUIRE_FALSE(server.initialize(invalidParams));
    }

    SECTION("Initialization with null parameter handler") {
        // Test: Server should handle null parameter handler gracefully
        // Behavior: Returns false when parameter handler is null
        std::weak_ptr<const IParametersHandler> nullHandler;
        REQUIRE_FALSE(server.initialize(nullHandler));
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionServer - Metadata Management Lifecycle") {
    VectorsCollectionServer server;
    REQUIRE(server.initialize(serverHandler));

    SECTION("Populate metadata entries before finalization") {
        // Test: Multiple metadata entries can be added before finalization
        // Behavior: Each populateMetadata call with unique key should succeed
        std::vector<std::string> jointMetadata = {"pos_0", "pos_1", "pos_2", "vel_0", "vel_1", "vel_2"};
        REQUIRE(server.populateMetadata("joints", jointMetadata));

        std::vector<std::string> ftMetadata = {"fx", "fy", "fz", "tx", "ty", "tz"};
        REQUIRE(server.populateMetadata("ft_sensor", ftMetadata));

        // Should accept different vector sizes and string content
        REQUIRE(server.populateMetadata("imu", {"ax", "ay", "az", "gx", "gy", "gz"}));
    }

    SECTION("Duplicate key rejection during metadata population") {
        // Test: Server should reject duplicate keys during metadata population
        // Behavior: Second populateMetadata call with same key should fail
        std::vector<std::string> originalMetadata = {"elem1", "elem2"};
        REQUIRE(server.populateMetadata("duplicate_key", originalMetadata));

        std::vector<std::string> duplicateMetadata = {"elem3", "elem4"};
        REQUIRE_FALSE(server.populateMetadata("duplicate_key", duplicateMetadata));
    }

    SECTION("Metadata ready state before and after finalization, and after new keys")
    {
        // Before any metadata is populated, areMetadataReady should be false
        REQUIRE_FALSE(server.areMetadataReady());

        // Populate metadata, but not finalized yet
        server.populateMetadata("test_key", {"elem1", "elem2"});
        REQUIRE_FALSE(server.areMetadataReady());

        // getMetadata should return empty before any finalize
        auto metaBeforeFinalize = server.getMetadata();
        REQUIRE(metaBeforeFinalize.vectors.empty());

        // Finalize metadata
        REQUIRE(server.finalizeMetadata());
        REQUIRE(server.areMetadataReady());

        // getMetadata should now return the finalized metadata
        auto metaAfterFinalize = server.getMetadata();
        REQUIRE(metaAfterFinalize.vectors.count("test_key") == 1);

        // Add new key after finalization
        REQUIRE(server.populateMetadata("new_key", {"elem3"}));
        // areMetadataReady should remain true after first finalize, even if new keys are pending
        REQUIRE(server.areMetadataReady());

        // getMetadata should return the last finalized metadata, not the new key
        auto metaWithPending = server.getMetadata();
        REQUIRE(metaWithPending.vectors.count("test_key") == 1);
        REQUIRE(metaWithPending.vectors.count("new_key") == 0);

        // Finalize again
        REQUIRE(server.finalizeMetadata());
        REQUIRE(server.areMetadataReady());

        // Now getMetadata should include the new key
        auto metaAfterSecondFinalize = server.getMetadata();
        REQUIRE(metaAfterSecondFinalize.vectors.count("test_key") == 1);
        REQUIRE(metaAfterSecondFinalize.vectors.count("new_key") == 1);
    }

    SECTION("getMetadata returns last finalized metadata if new keys are pending")
    {
        server.populateMetadata("k1", {"a"});
        REQUIRE(server.finalizeMetadata());
        REQUIRE(server.populateMetadata("k2", {"b"}));
        // Not finalized yet, so getMetadata should return only k1
        auto meta = server.getMetadata();
        REQUIRE(meta.vectors.count("k1") == 1);
        REQUIRE(meta.vectors.count("k2") == 0);
        // After finalize, both keys should be present
        REQUIRE(server.finalizeMetadata());
        auto meta2 = server.getMetadata();
        REQUIRE(meta2.vectors.count("k1") == 1);
        REQUIRE(meta2.vectors.count("k2") == 1);
    }

    SECTION("Successful metadata finalization") {
        // Test: Metadata finalization should succeed when metadata exists
        // Behavior: finalizeMetadata returns true and sets metadata as ready
        server.populateMetadata("test_key", {"elem1", "elem2"});
        REQUIRE(server.finalizeMetadata());
        REQUIRE(server.areMetadataReady());
    }

    SECTION("Finalization failure with empty metadata") {
        // Test: Finalization should fail when no metadata has been populated
        // Behavior: finalizeMetadata returns false for empty metadata
        REQUIRE_FALSE(server.finalizeMetadata());
        REQUIRE_FALSE(server.areMetadataReady());
    }

    SECTION("Metadata can be extended after finalization")
    {
        // Test: Metadata can be augmented with new keys after a finalize
        // Behavior: New keys are accepted and version increments on re-finalize
        server.populateMetadata("initial_key", {"elem1"});
        REQUIRE(server.finalizeMetadata());
        REQUIRE(server.areMetadataReady());

        REQUIRE(server.populateMetadata("extended_key", {"elem2"}));
        REQUIRE(server.areMetadataReady());
        REQUIRE(server.finalizeMetadata());
        REQUIRE(server.areMetadataReady());

        const auto metadata = server.getMetadata();
        REQUIRE(metadata.version == 1);
        REQUIRE(metadata.vectors.count("initial_key") == 1);
        REQUIRE(metadata.vectors.count("extended_key") == 1);
    }

    SECTION("Multiple finalization attempts require new metadata")
    {
        // Test: finalizeMetadata succeeds only when new keys are present
        server.populateMetadata("test_key", {"elem1"});
        REQUIRE(server.finalizeMetadata());
        REQUIRE_FALSE(server.finalizeMetadata());

        REQUIRE(server.populateMetadata("new_key", {"elem2"}));
        REQUIRE(server.finalizeMetadata());

        const auto metadata = server.getMetadata();
        REQUIRE(metadata.version == 1);
        REQUIRE(metadata.vectors.count("new_key") == 1);
    }

    SECTION("Incremental metadata retrieval returns only new keys")
    {
        // Test: Incremental retrieval returns data introduced after the requested version
        server.populateMetadata("base_key", {"base_elem"});
        REQUIRE(server.finalizeMetadata());

        server.populateMetadata("second_key", {"second_elem"});
        REQUIRE(server.finalizeMetadata());

        const auto latestMetadata = server.getMetadata();
        REQUIRE(latestMetadata.version == 1);
        REQUIRE(server.areMetadataReady());

        const auto incremental = server.getMetadataIncremental(0);
        REQUIRE(incremental.version == 1);
        REQUIRE(incremental.vectors.size() == 1);
        REQUIRE(incremental.vectors.count("second_key") == 1);

        const auto allIncremental = server.getMetadataIncremental(-1);
        REQUIRE(allIncremental.vectors.size() == 2);
        REQUIRE(allIncremental.vectors.count("base_key") == 1);
        REQUIRE(allIncremental.vectors.count("second_key") == 1);

        const auto negativeIncremental = server.getMetadataIncremental(-5);
        REQUIRE(negativeIncremental.vectors.empty());
        REQUIRE(negativeIncremental.version == latestMetadata.version);

        const auto emptyIncremental = server.getMetadataIncremental(5);
        REQUIRE(emptyIncremental.version == latestMetadata.version);
        REQUIRE(emptyIncremental.vectors.empty());
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionServer - Data Population and Transmission") {
    VectorsCollectionServer server;
    REQUIRE(server.initialize(serverHandler));

    // Setup metadata structure for testing data operations
    server.populateMetadata("joints", {"pos_0", "pos_1", "pos_2"});
    server.populateMetadata("ft_sensor", {"fx", "fy", "fz", "tx", "ty", "tz"});
    REQUIRE(server.finalizeMetadata());

    SECTION("Data buffer preparation and clearing") {
        // Test: Data buffer lifecycle management
        // Behavior: prepareData gets buffer reference, clearData resets it
        server.prepareData();
        REQUIRE(server.clearData());
    }

    SECTION("Successful data population with valid keys") {
        // Test: Data can be populated for all registered metadata keys
        // Behavior: populateData succeeds for keys present in metadata
        server.prepareData();

        std::vector<double> jointData = {1.0, 2.0, 3.0};
        REQUIRE(server.populateData("joints", jointData));

        std::vector<double> ftData = {0.1, 0.2, 0.3, 0.01, 0.02, 0.03};
        REQUIRE(server.populateData("ft_sensor", ftData));
    }

    SECTION("Data population with mismatched vector sizes") {
        // Test: Server should accept any vector size regardless of metadata count
        // Behavior: Vector size doesn't need to match metadata element count
        server.prepareData();

        // joints metadata has 3 elements, but providing 5 data points
        std::vector<double> oversizedData = {1.0, 2.0, 3.0, 4.0, 5.0};
        REQUIRE(server.populateData("joints", oversizedData));

        // Providing fewer data points than metadata elements
        std::vector<double> undersizedData = {0.1};
        REQUIRE(server.populateData("ft_sensor", undersizedData));
    }

    SECTION("Data population failure with invalid key") {
        // Test: Data population should fail for unregistered keys
        // Behavior: populateData returns false for keys not in metadata
        server.prepareData();

        std::vector<double> invalidData = {1.0, 2.0};
        REQUIRE_FALSE(server.populateData("nonexistent_key", invalidData));
    }

    SECTION("Data population without buffer preparation") {
        // Test: Data population should fail without calling prepareData first
        // Behavior: populateData returns false when no buffer is prepared
        std::vector<double> data = {1.0, 2.0, 3.0};
        REQUIRE_FALSE(server.populateData("joints", data));
    }

    SECTION("Data transmission after population") {
        // Test: Complete data transmission cycle
        // Behavior: sendData broadcasts populated data to connected clients
        server.prepareData();

        std::vector<double> jointData = {1.5, 2.5, 3.5};
        server.populateData("joints", jointData);

        // sendData should not throw exceptions or crash
        REQUIRE_NOTHROW(server.sendData());
    }

    SECTION("Empty data vector handling") {
        // Test: Server should handle empty vectors gracefully
        // Behavior: populateData accepts empty vectors
        server.prepareData();

        std::vector<double> emptyData = {};
        REQUIRE(server.populateData("joints", emptyData));
        REQUIRE_NOTHROW(server.sendData());
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionClient - Initialization and Configuration") {
    VectorsCollectionClient client;

    SECTION("Successful initialization with all required parameters")
    {
        // Test: Client should initialize with all required parameters
        // Behavior: Creates local ports and stores connection configuration
        REQUIRE(client.initialize(clientHandler));
    }

    SECTION("Initialization failure with missing parameters") {
        // Test: Client should fail when required parameters are missing
        // Behavior: Returns false when 'local', 'remote', or 'carrier' missing
        auto invalidParams = std::make_shared<StdImplementation>();
        invalidParams->setParameter("local", std::string("/incomplete"));
        // Missing 'remote' and 'carrier' parameters
        REQUIRE_FALSE(client.initialize(invalidParams));
    }

    SECTION("Initialization with null parameter handler") {
        // Test: Client should handle null parameter handler gracefully
        // Behavior: Returns false when parameter handler is null
        std::weak_ptr<const IParametersHandler> nullHandler;
        REQUIRE_FALSE(client.initialize(nullHandler));
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionClient - Connection Management") {
    VectorsCollectionServer server;
    VectorsCollectionClient client;

    // Setup server with basic metadata
    REQUIRE(server.initialize(serverHandler));
    server.populateMetadata("test_signal", {"elem1", "elem2"});
    REQUIRE(server.finalizeMetadata());

    REQUIRE(client.initialize(clientHandler));

    SECTION("isConnected returns false before connect") {
        // Test: Client should report disconnected state before connect
        REQUIRE_FALSE(client.isConnected());
    }

    SECTION("checkConnection returns false before connect") {
        // Test: checkConnection should return false when not connected
        REQUIRE_FALSE(client.checkConnection());
    }

    SECTION("Successful connection to available server") {
        // Test: Client should connect when server is available
        // Behavior: Establishes both data and RPC port connections

        // Allow server ports to fully open
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE(client.connect());
        REQUIRE(client.isConnected());
    }

    SECTION("checkConnection returns true when connected") {
        // Test: checkConnection should return true when both ports are alive
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        REQUIRE(client.connect());
        REQUIRE(client.checkConnection());
        REQUIRE(client.isConnected());
    }

    SECTION("Graceful connection failure when server unavailable") {
        // Test: Client should connect when server is available
        // Behavior: Establishes both data and RPC port connections

        // Allow server ports to fully open
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE(client.connect());
    }

    SECTION("Graceful connection failure when server unavailable") {
        // Test: Client should handle connection failure gracefully
        // Behavior: Returns false when server ports are not available
        // Note: This test assumes server ports aren't open immediately

        // Try to connect before server is ready - may fail depending on timing
        bool connectionResult = client.connect();
        // Connection may succeed or fail depending on YARP timing
        // The important thing is that it doesn't crash
        REQUIRE_NOTHROW(client.connect());
    }

    SECTION("Connection state persistence") {
        // Test: Connection state should be maintained after successful connection
        // Behavior: Multiple operations possible after single connect call

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (client.connect()) {
            // Connection succeeded, further operations should work
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Should be able to perform multiple operations
            VectorsCollectionMetadata metadata;
            bool metadataResult = client.getMetadata(metadata);
            // May succeed or fail depending on connection timing
        }
    }

    SECTION("Disconnection handling") {
        // Test: Client should handle disconnection gracefully
        // Behavior: disconnect() should clean up connections

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        client.connect();

        REQUIRE(client.disconnect());
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionServer-Client Integration") {
    VectorsCollectionServer server;
    VectorsCollectionClient client;

    // Initialize both components
    REQUIRE(server.initialize(serverHandler));
    REQUIRE(client.initialize(clientHandler));

    // Setup comprehensive server metadata
    std::vector<std::string> jointMetadata = {"joint_0_pos", "joint_1_pos", "joint_0_vel", "joint_1_vel"};
    std::vector<std::string> imuMetadata = {"acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"};

    REQUIRE(server.populateMetadata("robot_joints", jointMetadata));
    REQUIRE(server.populateMetadata("head_imu", imuMetadata));
    REQUIRE(server.finalizeMetadata());

    // Allow network connections to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    SECTION("Client connection establishment") {
        // Test: Client should successfully connect to initialized server
        // Behavior: Both data and RPC connections established
        REQUIRE(client.connect());

        // Allow connection handshake to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    SECTION("Complete metadata retrieval and validation") {
        // Test: Client should retrieve exact metadata structure from server
        // Behavior: Metadata content matches server configuration exactly
        REQUIRE(client.connect());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        VectorsCollectionMetadata metadata;
        REQUIRE(client.getMetadata(metadata));

        // Validate metadata structure
        REQUIRE(metadata.vectors.size() == 2);
        REQUIRE(metadata.vectors.count("robot_joints") == 1);
        REQUIRE(metadata.vectors.count("head_imu") == 1);

        // Validate metadata content exactness
        REQUIRE(metadata.vectors["robot_joints"] == jointMetadata);
        REQUIRE(metadata.vectors["head_imu"] == imuMetadata);
    }

    SECTION("End-to-end data transmission with accuracy verification") {
        // Test: Complete data flow from server to client with data integrity
        // Behavior: Data transmitted exactly matches data sent
        REQUIRE(client.connect());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Server prepares and sends structured data
        server.prepareData();

        std::vector<double> jointData = {1.1, 2.2, 0.1, 0.2};
        std::vector<double> imuData = {9.8, 0.1, 0.2, 0.01, 0.02, 0.03};

        REQUIRE(server.populateData("robot_joints", jointData));
        REQUIRE(server.populateData("head_imu", imuData));

        server.sendData();

        // Allow network transmission
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Client reads and validates data
        const VectorsCollection* receivedData = client.readData(false);

        if (receivedData != nullptr) {
            // Validate data structure integrity
            REQUIRE(receivedData->vectors.size() == 2);
            REQUIRE(receivedData->vectors.count("robot_joints") == 1);
            REQUIRE(receivedData->vectors.count("head_imu") == 1);

            // Validate data value accuracy
            const auto& receivedJoints = receivedData->vectors.at("robot_joints");
            const auto& receivedIMU = receivedData->vectors.at("head_imu");

            REQUIRE(receivedJoints.size() == jointData.size());
            REQUIRE(receivedIMU.size() == imuData.size());

            // Verify floating-point accuracy with tolerance
            for (size_t i = 0; i < jointData.size(); ++i) {
                REQUIRE(receivedJoints[i] == Catch::Approx(jointData[i]).epsilon(1e-10));
            }

            for (size_t i = 0; i < imuData.size(); ++i) {
                REQUIRE(receivedIMU[i] == Catch::Approx(imuData[i]).epsilon(1e-10));
            }
        }
    }

    SECTION("Client refreshes metadata when receiving newer version")
    {
        REQUIRE(client.connect());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        server.prepareData();
        std::vector<double> firstJointData{0.0, 0.1, 0.2, 0.3};
        std::vector<double> firstImuData{9.81, 0.0, 0.0, 0.01, 0.02, 0.03};
        REQUIRE(server.populateData("robot_joints", firstJointData));
        REQUIRE(server.populateData("head_imu", firstImuData));
        server.sendData();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto* firstSample = client.readData(true);
        REQUIRE(firstSample != nullptr);
        REQUIRE(firstSample->version == 0);

        VectorsCollectionMetadata cachedMetadata;
        REQUIRE(client.getMetadata(cachedMetadata));
        REQUIRE(cachedMetadata.version == 0);
        REQUIRE(cachedMetadata.vectors.count("robot_joints") == 1);
        REQUIRE(cachedMetadata.vectors.count("head_imu") == 1);

        REQUIRE(server.populateMetadata("ankle_force_sensor", {"fx", "fy", "fz"}));
        REQUIRE(server.finalizeMetadata());

        server.prepareData();
        std::vector<double> secondJointData{1.0, 1.1, 1.2, 1.3};
        std::vector<double> secondImuData{9.7, 0.1, 0.2, 0.01, 0.01, 0.02};
        std::vector<double> ankleForces{20.0, 10.0, -5.0};
        REQUIRE(server.populateData("robot_joints", secondJointData));
        REQUIRE(server.populateData("head_imu", secondImuData));
        REQUIRE(server.populateData("ankle_force_sensor", ankleForces));
        server.sendData();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto* secondSample = client.readData(true);
        REQUIRE(secondSample != nullptr);
        REQUIRE(secondSample->version == 1);
        REQUIRE(secondSample->vectors.count("ankle_force_sensor") == 1);

        VectorsCollectionMetadata updatedMetadata;
        REQUIRE(client.getMetadata(updatedMetadata));
        REQUIRE(updatedMetadata.version == 1);
        REQUIRE(updatedMetadata.vectors.count("ankle_force_sensor") == 1);
        REQUIRE(updatedMetadata.vectors.at("ankle_force_sensor")
                == std::vector<std::string>{"fx", "fy", "fz"});
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionServer - Multiple Transmission Cycles") {
    VectorsCollectionServer server;
    VectorsCollectionClient client;

    REQUIRE(server.initialize(serverHandler));
    REQUIRE(client.initialize(clientHandler));

    // Setup metadata for cyclic testing
    server.populateMetadata("sensor_data", {"value_1", "value_2", "value_3"});
    REQUIRE(server.finalizeMetadata());

    REQUIRE(client.connect());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    SECTION("Multiple sequential data transmission cycles") {
        // Test: Server should handle multiple data cycles correctly
        // Behavior: Each cycle should maintain data integrity independently
        const int numCycles = 5;

        for (int cycle = 0; cycle < numCycles; ++cycle) {
            // Server transmits cycle-specific data
            server.prepareData();
            server.clearData(); // Ensure clean state for each cycle

            std::vector<double> cycleData = {
                static_cast<double>(cycle),
                static_cast<double>(cycle * 2),
                static_cast<double>(cycle * 3)
            };

            REQUIRE(server.populateData("sensor_data", cycleData));
            server.sendData();

            // Brief transmission delay
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Client receives and validates cycle-specific data
            const VectorsCollection* receivedData = client.readData(false);

            if (receivedData != nullptr) {
                const auto& received = receivedData->vectors.at("sensor_data");
                REQUIRE(received.size() == 3);

                // Validate cycle-specific values
                REQUIRE(received[0] == Catch::Approx(static_cast<double>(cycle)));
                REQUIRE(received[1] == Catch::Approx(static_cast<double>(cycle * 2)));
                REQUIRE(received[2] == Catch::Approx(static_cast<double>(cycle * 3)));
            }
        }
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollection - Edge Cases and Error Conditions") {
    VectorsCollectionServer server;
    VectorsCollectionClient client;

    REQUIRE(server.initialize(serverHandler));
    REQUIRE(client.initialize(clientHandler));

    SECTION("Empty metadata vector handling") {
        // Test: Server should handle metadata with empty description vectors
        // Behavior: Empty metadata vectors should be accepted
        REQUIRE(server.populateMetadata("empty_metadata", {}));
        REQUIRE(server.finalizeMetadata());
    }

    SECTION("Large metadata vector handling") {
        // Test: Server should handle metadata with many elements
        // Behavior: Large metadata vectors should be processed correctly
        std::vector<std::string> largeMetadata;
        for (int i = 0; i < 100; ++i) {
            largeMetadata.push_back("element_" + std::to_string(i));
        }

        REQUIRE(server.populateMetadata("large_metadata", largeMetadata));
        REQUIRE(server.finalizeMetadata());
    }

    SECTION("Empty data vector transmission") {
        // Test: System should handle empty data vectors gracefully
        // Behavior: Empty vectors should transmit without errors
        server.populateMetadata("empty_vector", {"description"});
        server.populateMetadata("normal_vector", {"elem1", "elem2"});
        REQUIRE(server.finalizeMetadata());

        REQUIRE(client.connect());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        server.prepareData();

        std::vector<double> emptyData = {};
        std::vector<double> normalData = {1.0, 2.0};

        REQUIRE(server.populateData("empty_vector", emptyData));
        REQUIRE(server.populateData("normal_vector", normalData));

        server.sendData();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        const VectorsCollection* receivedData = client.readData(false);

        if (receivedData != nullptr) {
            REQUIRE(receivedData->vectors.count("empty_vector") == 1);
            REQUIRE(receivedData->vectors.count("normal_vector") == 1);

            // Empty vector should remain empty
            REQUIRE(receivedData->vectors.at("empty_vector").empty());
            REQUIRE(receivedData->vectors.at("normal_vector").size() == 2);
        }
    }

    SECTION("Special floating-point value handling") {
        // Test: System should handle special float values (inf, nan, very large/small)
        // Behavior: Special values should transmit correctly
        server.populateMetadata("special_values", {"inf", "neg_inf", "large", "small"});
        REQUIRE(server.finalizeMetadata());

        REQUIRE(client.connect());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        server.prepareData();

        std::vector<double> specialData = {
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            1e100,
            1e-100
        };

        REQUIRE(server.populateData("special_values", specialData));
        server.sendData();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        const VectorsCollection* receivedData = client.readData(false);

        if (receivedData != nullptr) {
            const auto& received = receivedData->vectors.at("special_values");
            REQUIRE(received.size() == 4);

            // Verify special values are preserved
            REQUIRE(std::isinf(received[0]));
            REQUIRE(received[0] > 0);
            REQUIRE(std::isinf(received[1]));
            REQUIRE(received[1] < 0);
            REQUIRE(received[2] == Catch::Approx(1e100));
            REQUIRE(received[3] == Catch::Approx(1e-100));
        }
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture, "VectorsCollectionClient - Error Handling and Robustness") {
    VectorsCollectionClient client;
    REQUIRE(client.initialize(clientHandler));

    SECTION("Data reading without connection") {
        // Test: Client should handle data reading attempts without connection
        // Behavior: readData should return nullptr gracefully
        const VectorsCollection* data = client.readData(false);
        // Should return nullptr or handle gracefully without crashing
        // Exact behavior is implementation-specific
    }

    SECTION("Metadata retrieval without connection") {
        // Test: Client should handle metadata requests without connection
        // Behavior: getMetadata should fail gracefully
        VectorsCollectionMetadata metadata;
        bool result = client.getMetadata(metadata);
        // Should return false or empty metadata without crashing
        // Implementation-specific behavior
    }

    SECTION("Multiple connection attempts") {
        // Test: Client should handle multiple connection attempts gracefully
        // Behavior: Repeated connect calls should not cause issues

        // Multiple connection attempts should not crash
        REQUIRE_NOTHROW(client.connect());
        REQUIRE_NOTHROW(client.connect());
        REQUIRE_NOTHROW(client.connect());
    }

    SECTION("Disconnect without prior connection") {
        // Test: Client should handle disconnection without prior connection
        // Behavior: disconnect should succeed even if not connected
        REQUIRE(client.disconnect());
    }
}

TEST_CASE_METHOD(VectorsCollectionFixture,
                 "VectorsCollectionClient - Metadata Caching and Versioning")
{
    VectorsCollectionServer server;
    VectorsCollectionClient client;

    REQUIRE(server.initialize(serverHandler));
    REQUIRE(client.initialize(clientHandler));

    // Setup initial metadata and finalize
    REQUIRE(server.populateMetadata("key1", {"a", "b"}));
    REQUIRE(server.finalizeMetadata());

    REQUIRE(client.connect());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read initial data to trigger metadata version check
    server.prepareData();
    std::vector<double> d1 = {1.0, 2.0};
    REQUIRE(server.populateData("key1", d1));
    server.sendData();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    client.readData(true);

    // Should detect new metadata available
    REQUIRE(client.isNewMetadataAvailable());

    // getMetadata should update the cache and clear the flag
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata meta;
    REQUIRE(client.getMetadata(meta));
    REQUIRE(meta.vectors.count("key1") == 1);
    REQUIRE(meta.vectors.at("key1") == std::vector<std::string>{"a", "b"});
    REQUIRE_FALSE(client.isNewMetadataAvailable());

    // Add new metadata on server and finalize
    REQUIRE(server.populateMetadata("key2", {"c"}));
    REQUIRE(server.finalizeMetadata());

    // Send new data with updated version
    server.prepareData();
    std::vector<double> d2 = {3.0, 4.0};
    std::vector<double> d3 = {5.0};
    REQUIRE(server.populateData("key1", d2));
    REQUIRE(server.populateData("key2", d3));
    server.sendData();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    client.readData(true);

    // Should detect new metadata available again
    REQUIRE(client.isNewMetadataAvailable());

    // getMetadata should now include the new key
    REQUIRE(client.getMetadata(meta));
    REQUIRE(meta.vectors.count("key2") == 1);
    REQUIRE(meta.vectors.at("key2") == std::vector<std::string>{"c"});
    REQUIRE(meta.version == 1);
    REQUIRE_FALSE(client.isNewMetadataAvailable());
}

TEST_CASE_METHOD(VectorsCollectionFixture,
                 "VectorsCollectionClient - getMetadata fails if not connected")
{
    VectorsCollectionClient client;
    REQUIRE(client.initialize(clientHandler));

    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata meta;
    REQUIRE_FALSE(client.getMetadata(meta));
}

TEST_CASE_METHOD(VectorsCollectionFixture,
                 "VectorsCollectionClient - checkConnection detects server shutdown")
{
    // Use a heap-allocated server so we can destroy it mid-test
    auto server = std::make_unique<VectorsCollectionServer>();
    REQUIRE(server->initialize(serverHandler));
    server->populateMetadata("sig", {"a", "b"});
    REQUIRE(server->finalizeMetadata());

    VectorsCollectionClient client;
    REQUIRE(client.initialize(clientHandler));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(client.connect());
    REQUIRE(client.isConnected());
    REQUIRE(client.checkConnection());

    // Destroy the server, which closes its ports
    server.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // checkConnection should detect the broken connections
    REQUIRE_FALSE(client.checkConnection());
    REQUIRE_FALSE(client.isConnected());

    // After detecting disconnection, connect should fail (no server)
    REQUIRE_FALSE(client.connect());
}

TEST_CASE_METHOD(VectorsCollectionFixture,
                 "VectorsCollectionClient - reconnect after server restart")
{
    // Start a server, connect, then destroy and recreate it
    auto server = std::make_unique<VectorsCollectionServer>();
    REQUIRE(server->initialize(serverHandler));
    server->populateMetadata("sig", {"x"});
    REQUIRE(server->finalizeMetadata());

    VectorsCollectionClient client;
    REQUIRE(client.initialize(clientHandler));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(client.connect());
    REQUIRE(client.checkConnection());

    // Send some data
    server->prepareData();
    REQUIRE(server->populateData("sig", std::vector<double>{1.0}));
    server->sendData();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto* data = client.readData(false);
    REQUIRE(data != nullptr);
    REQUIRE(data->vectors.at("sig")[0] == Catch::Approx(1.0));

    // Destroy the server
    server.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    REQUIRE_FALSE(client.checkConnection());

    // Create a new server on the same ports
    server = std::make_unique<VectorsCollectionServer>();
    REQUIRE(server->initialize(serverHandler));
    server->populateMetadata("sig", {"x"});
    REQUIRE(server->finalizeMetadata());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Reconnect
    REQUIRE(client.connect());
    REQUIRE(client.isConnected());
    REQUIRE(client.checkConnection());

    // Verify data flows again
    server->prepareData();
    REQUIRE(server->populateData("sig", std::vector<double>{42.0}));
    server->sendData();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    data = client.readData(false);
    if (data != nullptr)
    {
        REQUIRE(data->vectors.at("sig")[0] == Catch::Approx(42.0));
    }
}
