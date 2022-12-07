/**@file   OSRMHelper.cpp
 * @brief  Implementation of a helper interface for OSRM
 * @author André Mazal Krauss
 *
 * This file implements the usage of OSRM to query distance/time information from an OSM database
 *
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#include "OSRMHelper.h"

#include <cmath>

#ifdef OSRM_AVAILABLE

OSRMHelper::OSRMHelper(std::string osmPath)
{

    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore
	config.storage_config = { osmPath };
	config.use_shared_memory = false;

    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    // this must be configured accordingly with how your database was preprocessed!
    // config.algorithm = EngineConfig::Algorithm::CH;
    config.algorithm = EngineConfig::Algorithm::MLD;

}

std::vector<std::vector<double>> OSRMHelper::TableRequest(std::vector<const Vertex*> &vertices)
{

    std::vector<double> longitudes;
    std::vector<double> latitudes;

    longitudes.reserve(vertices.size());
    latitudes.reserve(vertices.size());
    for( int i = 0; i < vertices.size(); i++)
    {
        longitudes.push_back(vertices[i]->position.x);
        latitudes.push_back(vertices[i]->position.y);
    }
    return TableRequest(longitudes, latitudes);
    
}


std::vector<std::vector<double>> OSRMHelper::TableRequest(std::vector<double>& longitudes, std::vector<double>& latitudes)
{
    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm{ config };
    
    std::vector<std::vector<double>> outDistances = std::vector<std::vector<double>>();
    outDistances.reserve(longitudes.size());

    //make a 'table' request
    TableParameters params;

    //params.fallback_speed = 1000000.0;

    for (int i = 0; i < longitudes.size(); i++)
    {
        params.coordinates.push_back({ util::FloatLongitude{longitudes[i]}, util::FloatLatitude{latitudes[i]} });
    }

    // Response is in JSON format
    engine::api::ResultT result = json::Object();

    // Execute request, this does the heavy lifting
    const auto status = osrm.Table(params, result);

    osrm::util::json::Object json_result = result.get<json::Object>();
    if (status == Status::Ok)
    {
        //get array of arrays inside JSON
        auto& durationsMat = json_result.values["durations"].get<json::Array>();

        int i = 0;
        int j = 0;
        for (auto it = durationsMat.values.cbegin(); it != durationsMat.values.cend(); ++it)
        {
            auto& row = it->get<json::Array>();
            std::vector<double> drow = std::vector<double>();
            drow.reserve(latitudes.size());
            j = 0;
            for (auto it = row.values.cbegin(); it != row.values.cend(); ++it)
            {
                it->match(
                    [&drow, &i, &j, &longitudes, &latitudes](json::Number n) {
                        if (n.value == 0.0f && i != j)
                        {
                            if (longitudes[i] != longitudes[j] && latitudes[i] != latitudes[j]) // float comparison, meant as is
                            {
                                std::cerr << "WARN: calculated distance between different cells is 0.0. This is probably due to their centers having snapped to the same point" << std::endl;
                            }
                        }
                        drow.push_back(n.value);
                    },
                    [&drow](json::Null n) {
                        drow.push_back(nan(""));
                    },
                        [](json::Array n) {
                        throw std::runtime_error("unexpected API response");
                    },
                        [](json::False n) {
                        throw std::runtime_error("unexpected API response");
                    },
                        [](json::True n) {
                        throw std::runtime_error("unexpected API response");
                    },
                        [](json::Object n) {
                        throw std::runtime_error("unexpected API response");
                    },
                        [](json::String n) {
                        throw std::runtime_error("unexpected API response");
                    }
                    );
                j++;
            }
            outDistances.push_back(drow);
            i++;
        }

        return outDistances;
    }
    else if (status == Status::Error)
    {
        const auto code = json_result.values["code"].get<json::String>().value;
        const auto message = json_result.values["message"].get<json::String>().value;

        std::cout << "Code: " << code << "\n";
        std::cout << "Message: " << code << "\n";
        return outDistances;
    }
}

double OSRMHelper::GetDistance(double lon1, double lat1, double lon2, double lat2)
{
    const OSRM osrm{ config };

    RouteParameters params;
    params.coordinates.push_back({ util::FloatLongitude{lon1}, util::FloatLatitude{lat1} });
    params.coordinates.push_back({ util::FloatLongitude{lon2}, util::FloatLatitude{lat2} });

    engine::api::ResultT result = json::Object();

    // Execute routing request, this does the heavy lifting
    const auto status = osrm.Route(params, result);

    auto& json_result = result.get<json::Object>();
    if (status == Status::Ok)
    {
        auto& routes = json_result.values["routes"].get<json::Array>();

        // Let's just use the first route
        auto& route = routes.values.at(0).get<json::Object>();
        const auto distance = route.values["distance"].get<json::Number>().value;
        const auto duration = route.values["duration"].get<json::Number>().value;

        // Warn users if extract does not contain the default coordinates from above
        if (distance == 0 || duration == 0)
        {
            std::cout << "WARN: distance or duration is zero. ";
            std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
        }

        return distance;
    }
    else if (status == Status::Error)
    {
        const auto code = json_result.values["code"].get<json::String>().value;
        const auto message = json_result.values["message"].get<json::String>().value;

        std::cout << "Code: " << code << "\n";
        std::cout << "Message: " << message << "\n";
        throw std::runtime_error(message);
    }

}

double OSRMHelper::GetDuration(double lon1, double lat1, double lon2, double lat2)
{
    const OSRM osrm{ config };

    RouteParameters params;
    params.coordinates.push_back({ util::FloatLongitude{lon1}, util::FloatLatitude{lat1} });
    params.coordinates.push_back({ util::FloatLongitude{lon2}, util::FloatLatitude{lat2} });

    engine::api::ResultT result = json::Object();

    // Execute routing request, this does the heavy lifting
    const auto status = osrm.Route(params, result);

    auto& json_result = result.get<json::Object>();
    if (status == Status::Ok)
    {
        auto& routes = json_result.values["routes"].get<json::Array>();

        // Let's just use the first route
        auto& route = routes.values.at(0).get<json::Object>();
        const auto distance = route.values["distance"].get<json::Number>().value;
        const auto duration = route.values["duration"].get<json::Number>().value;

        // Warn users if extract does not contain the default coordinates from above
        if (distance == 0 || duration == 0)
        {
            std::cout << "WARN: distance or duration is zero. ";
            std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
        }

        return duration;
    }
    else if (status == Status::Error)
    {
        const auto code = json_result.values["code"].get<json::String>().value;
        const auto message = json_result.values["message"].get<json::String>().value;

        std::cout << "Code: " << code << "\n";
        std::cout << "Message: " << code << "\n";
        return -1.0;
    }

}

#endif
