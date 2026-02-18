/**
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_MESSAGE_CONVERSION_UTILITIES_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_MESSAGE_CONVERSION_UTILITIES_H

#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionMetadata.h>

#include <trintrin/msgs/HumanState.h>
#include <trintrin/msgs/WearableTargets.h>
#include <trintrin/msgs/WearableData.h>

#include <Eigen/Core>

#include <string>

namespace BipedalLocomotion
{

static const std::string treeDelim = "::";

/**
 * @brief Extract metadata from a HumanState message and populate a VectorsCollectionMetadata object.
 *
 * @param message The input HumanState message containing the metadata.
 * @param prefix A string prefix to be added to the metadata keys.
 * @param metadata The VectorsCollectionMetadata object to populate with extracted metadata.
 */
void extractMetadata(const trintrin::msgs::HumanState& message,
                     const std::string& prefix,
                     BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);

/**
 * @brief Extract metadata from a WearableTargets message and populate a VectorsCollectionMetadata object.
 *
 * @param message The input WearableTargets message containing the metadata.
 * @param prefix A string prefix to be added to the metadata keys.
 * @param metadata The VectorsCollectionMetadata object to populate with extracted metadata.
 */
void extractMetadata(const trintrin::msgs::WearableTargets& message,
                     const std::string& prefix,
                     BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);

/**
 * @brief Extract metadata from a WearableData message and populate a VectorsCollectionMetadata object.
 *
 * @param message The input WearableData message containing the metadata.
 * @param prefix A string prefix to be added to the metadata keys.
 * @param metadata The VectorsCollectionMetadata object to populate with extracted metadata.
 */
void extractMetadata(const trintrin::msgs::WearableData& message,
                     const std::string& prefix,
                     BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);

/**
 * @brief Convert a HumanState message to a VectorsCollection object.
 *
 * @param message The input HumanState message to be converted.
 * @param prefix A string prefix to be added to the collection keys.
 * @param collection The VectorsCollection object to populate with the converted data.
 */
void convertToVectorsCollection(const trintrin::msgs::HumanState& message,
                                const std::string& prefix,
                                BipedalLocomotion::YarpUtilities::VectorsCollection& collection);

/**
 * @brief Convert a WearableTargets message to a VectorsCollection object.
 *
 * @param message The input WearableTargets message to be converted.
 * @param prefix A string prefix to be added to the collection keys.
 * @param collection The VectorsCollection object to populate with the converted data.
 */
void convertToVectorsCollection(const trintrin::msgs::WearableTargets& message,
                                const std::string& prefix,
                                BipedalLocomotion::YarpUtilities::VectorsCollection& collection);

/**
 * @brief Convert a WearableData message to a VectorsCollection object.
 *
 * @param message The input WearableData message to be converted.
 * @param prefix A string prefix to be added to the collection keys.
 * @param collection The VectorsCollection object to populate with the converted data.
 */
void convertToVectorsCollection(const trintrin::msgs::WearableData& message,
                                const std::string& prefix,
                                BipedalLocomotion::YarpUtilities::VectorsCollection& collection);

/**
 * @brief Convert a trintrin VectorXYZ message to an Eigen::Vector3d.
 *
 * @param vec The input VectorXYZ message to be converted.
 * @return Eigen::Vector3d The converted 3D vector.
 */
Eigen::Vector3d trintrinVectorXYZToVector3(const trintrin::msgs::VectorXYZ& vec);

}

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_MESSAGE_CONVERSION_UTILITIES_H
