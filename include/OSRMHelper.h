/**@file   OSRMHelper.h
 * @brief  Definition of helper interface to use OSM data via OSRM 
 * @author André Mazal Krauss
 *
 * 
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/


#pragma once

#ifdef OSRM_AVAILABLE

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

using namespace osrm;

#endif

#include "ProblemData.h"


/**
    Helper class to facilitate interaction with the OSRM-backend API. Requires access to a local OSM database preprocessed with OSRM with the Multi-Level Dijkstra (MLD) configuration (requires extract+partition+customize).See https://github.com/Project-OSRM/osrm-backend for more information
*/
class OSRMHelper 
{

#ifdef OSRM_AVAILABLE

private:
    EngineConfig config;

public:

    /**
     * Construct an instance of OSRMHelper given a path to a valid osm database preprocessed for MLD
     * 
     * 
    */
    OSRMHelper(
        std::string osmPath /**< path osm database */
        );

    /**
     * Gets travel distance (in meters) between two (longitude, latitude) pairs. 
    */
    double GetDistance(
        double lon1, /**< longitude of pair 1 */
        double lat1, /**< latitude of pair 1 */
        double lon2, /**< longitude of pair 2 */
        double lat2 /**< latitude of pair 2 */
    );

    /**
     * Gets travel duration (in seconds) between two (longitude, latitude) pairs. 
    */
    double GetDuration(
        double lon1, /**< longitude of pair 1 */
        double lat1, /**< latitude of pair 1 */
        double lon2, /**< longitude of pair 2 */
        double lat2 /**< latitude of pair 2 */
    );

    /**
     * Returns the time duration of multiple requests
    */
    std::vector<std::vector<double>> TableRequest(std::vector<const Vertex*> &vertices);
    std::vector<std::vector<double>> TableRequest(std::vector<double>& longitudes, std::vector<double>& latitudes);

    
#else

public:
    OSRMHelper(std::string osmPath){ std::cout << "WARNING: tried using OSRM but lib is not available" << std::endl;  }

    std::vector<std::vector<double>> TableRequest(std::vector<const Vertex*> &vertices) { std::cout << "WARNING: tried using OSRM but lib is not available" << std::endl;  }
    std::vector<std::vector<double>> TableRequest(std::vector<double>& longitudes, std::vector<double>& latitudes) { std::cout << "WARNING: tried using OSRM but lib is not available" << std::endl;  }

    double GetDistance(double lon1, double lat1, double lon2, double lat2) { std::cout << "WARNING: tried using OSRM but lib is not available" << std::endl;  }
    double GetDuration(double lon1, double lat1, double lon2, double lat2) { std::cout << "WARNING: tried using OSRM but lib is not available" << std::endl;  }

#endif

};
