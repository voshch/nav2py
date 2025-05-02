/*
 *  License: MIT
 *  Author(s): voshch <dev@voshch.dev>
 */
/*
 * License: MIT
 * Author(s): voshch <dev@voshch.dev>
 */
#ifndef NAV2PY_UTILS_UTILS_HPP_
#define NAV2PY_UTILS_UTILS_HPP_

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

namespace nav2py
{
    namespace utils
    {
        // Forward declarations
        class Layer;
        class Observation;

        /**
         * @brief Base class for classes needing access to parameters via a client and a prefix.
         */
        class ParameterClientUser
        {
        protected:
            /**
             * @brief Shared pointer to the synchronous parameter client.
             */
            std::shared_ptr<rclcpp::SyncParametersClient> param_client_;

            /**
             * @brief Prefix string to prepend to parameter names. Expected to include trailing dot if not empty.
             */
            std::string prefix_;

            /**
             * @brief Method to get a parameter using the stored prefix.
             * @tparam T The expected type of the parameter value.
             * @param name The name of the parameter relative to the prefix (e.g., "topic").
             * @return The value of the parameter.
             */
            template <typename T>
            T get_parameter(const std::string &name) const
            {
                auto parameters = param_client_->get_parameters({prefix_ + name});
                if (parameters.empty())
                    throw std::runtime_error("Parameter not found: " + prefix_ + name);
                return parameters[0].get_value<T>();
            }

            /**
             * @brief Method to get a parameter using the stored prefix.
             * @tparam T The expected type of the parameter value.
             * @param name The name of the parameter relative to the prefix (e.g., "topic").
             * @param default_value Value to return if parameter isn't set.
             * @return The value of the parameter.
             */
            template <typename T>
            T get_parameter(const std::string &name, T default_value) const
            {
                auto parameters = param_client_->get_parameters({prefix_ + name});
                if (parameters.empty())
                    return default_value;
                return parameters[0].get_value<T>();
            }

            /**
             * @brief Constructor for ParameterClientUser.
             * @param param_client Shared pointer to the synchronous parameter client.
             * @param prefix Prefix string for parameters. Should include trailing dot if needed.
             */
            ParameterClientUser(const std::shared_ptr<rclcpp::SyncParametersClient> &param_client, const std::string &prefix)
                : param_client_(param_client), prefix_(prefix)
            {
            }

        public:
            /**
             * @brief Virtual destructor.
             */
            virtual ~ParameterClientUser() = default;
        }; // class ParameterClientUser

        /**
         * @brief Represents a single observation source and provides access to its specific parameters.
         *
         * Inherits from ParameterClientUser to access the parameter client and manage prefix.
         * The prefix is expected to be like "layer_name.source_name." (note the trailing dot).
         * Objects are typically created dynamically via Layer::getObservations().
         */
        class Observation : public ParameterClientUser
        {
        public:
            /**
             * @brief Constructor for Observation.
             * @param param_client Shared pointer to the synchronous parameter client.
             * @param source_prefix The combined prefix including layer and source name, ending with a dot (e.g., "voxel_layer.scan.").
             */
            Observation(
                const std::shared_ptr<rclcpp::SyncParametersClient> &param_client,
                const std::string &source_prefix)
                : ParameterClientUser(param_client, source_prefix)
            {
            }

            /**
             * @brief Gets the ROS 2 topic name associated with this observation source.
             *
             * Retrieves the parameter named "<prefix>topic" from the target node.
             * @return The topic name as a string.
             */
            std::string topic() const
            {
                return get_parameter<std::string>("topic");
            }

            /**
             * @brief Gets the data type associated with this observation source.
             *
             * Retrieves the parameter named "<prefix>data_type" from the target node.
             * @return The data type name as a string.
             */
            std::string data_type() const
            {
                return get_parameter<std::string>("data_type");
            }

            /**
             * @brief Gets the full prefix string used for this observation source (ends with a dot).
             * @return A constant reference to the source prefix string (e.g., "voxel_layer.scan.").
             */
            const std::string &prefix() const
            {
                return prefix_;
            }

        }; // class Observation

        /**
         * @brief Provides an interface to access configuration parameters for a specific layer from a target ROS 2 node.
         *
         * Inherits from ParameterClientUser to access the parameter client and manage prefix.
         * The prefix is stored as "layer_name." (note the trailing dot).
         * Observations are loaded dynamically via getObservations().
         */
        class Layer : public ParameterClientUser
        {
        public:
            /**
             * @brief Constructs a Layer object.
             * @param param_client A shared pointer to the synchronous parameter client.
             * @param layer_name The name of the layer this instance represents. The stored prefix will be "layer_name.".
             */
            Layer(
                const std::shared_ptr<rclcpp::SyncParametersClient> &param_client,
                const std::string &layer_name)
                : ParameterClientUser(param_client, layer_name + ".")
            {
            }

            /**
             * @brief Dynamically retrieves Observation objects for all available observation sources within this layer.
             *
             * Fetches the "<prefix>observation_sources" parameter list from the target node and then creates
             * an Observation object for each source name found. Assumes the parameter is a space-separated string.
             * @return A vector of Observation objects. Returns an empty vector if 'observation_sources' is not declared.
             */
            std::vector<Observation> getObservations() const
            {
                std::string sources_str = get_parameter<std::string>("observation_sources", "");
                std::stringstream ss(sources_str);
                std::vector<Observation> observations;

                std::string source_name;
                while (ss >> source_name)
                    if (!source_name.empty())
                        observations.emplace_back(param_client_, prefix_ + source_name + ".");

                return observations;
            }

            /**
             * @brief Gets the plugin type name associated with this layer.
             *
             * Retrieves the parameter named "<prefix>plugin" from the target node.
             * This typically corresponds to the pluginlib class name (e.g., "nav2_costmap_2d::VoxelLayer").
             * @return The plugin type name as a string.
             */
            std::string plugin() const
            {
                return get_parameter<std::string>("plugin");
            }

        }; // class Layer

        /**
         * @brief Manages access to parameters for multiple layers (plugins) and global settings on a target ROS 2 node.
         *
         * Inherits from ParameterClientUser to hold the parameter client (with an empty prefix for global access).
         * Provides access to dynamically loaded layers (via getLayers) and global parameters.
         */
        class Costmap : public ParameterClientUser
        {
        private:
            rclcpp::Node::SharedPtr node_;

        public:
            /**
             * @brief Constructs a CostmapParams object using an existing node.
             *
             * Initializes a synchronous parameter client targeting the specified node name,
             * creating a node in the given namespace..
             * @param target_node_name The full name of the ROS 2 node from which to retrieve parameters.
             * @param ns Namespace for node.
             */
            Costmap(
                const std::string &target_node_name,
                const std::string &ns = "/")
                : ParameterClientUser(nullptr, "")
            {
                size_t last_slash_pos = target_node_name.find_last_of('/');
                std::string base_name = (last_slash_pos == std::string::npos) ? target_node_name : target_node_name.substr(last_slash_pos + 1);
                node_ = std::make_shared<rclcpp::Node>("_costmap_params_" + base_name, ns);

                param_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, target_node_name);
            }

            /**
             * @brief Dynamically loads and returns the configured layers based on the 'plugins' parameter.
             * Reads the 'plugins' parameter and constructs Layer objects on each call. Assumes 'plugins' is std::vector<std::string>.
             * @return A vector of Layer objects.
             */
            std::vector<Layer> getLayers() const
            {
                std::vector<std::string> plugin_names = get_parameter<std::vector<std::string>>("plugins");
                std::vector<Layer> layers;
                layers.reserve(plugin_names.size());

                for (const auto &layer_name : plugin_names)
                    layers.emplace_back(param_client_, layer_name);

                return layers;
            }

            /**
             * @brief Get the all observations of given type.
             * @param type_ observation type.
             * @return vector<Observation>.
             */
            std::vector<Observation> findallObservationByType(const std::string &type_) const
            {
                std::vector<Observation> observations;
                for (const auto &layer : getLayers())
                    for (const auto &observation : layer.getObservations())
                        if (observation.data_type() == type_)
                            observations.push_back(observation);
                return observations;
            }

            /**
             * @brief Get the first observation of given type or nullopt.
             * @param type_ observation type.
             * @return optional<Observation>.
             */
            std::optional<Observation> findObservationByType(const std::string &type_) const
            {
                for (const auto &layer : getLayers())
                    for (const auto &observation : layer.getObservations())
                        if (observation.data_type() == type_)
                            return observation;
                return std::nullopt;
            }

        }; // class Costmap
    } // namespace utils
} // namespace nav2py

#endif // NAV2PY_UTILS_UTILS_HPP_
