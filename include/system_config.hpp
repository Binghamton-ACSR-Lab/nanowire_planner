//
// Created by acsr on 5/20/21.
//

#ifndef NANOWIREPLANNER_SYSTEMCONFIG_HPP
#define NANOWIREPLANNER_SYSTEMCONFIG_HPP

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "nanowire_utility.hpp"

namespace acsr{
    class SystemConfig
    {

    public:
        SystemConfig()=default;
        virtual ~SystemConfig()=default;

        static double max_distance;
        static int http_port;
        static int tcp_port;


        static void readFile(std::string file_name){
            po::options_description opt_desc("Options");
            opt_desc.add_options()
                    ("max_distance",po::value<double>(&SystemConfig::max_distance)->default_value(60.0),"max distance.")
                    ("http_port",po::value<int>(&SystemConfig::http_port)->default_value(8080),"http server port")
                    ("tcp_port",po::value<int>(&SystemConfig::tcp_port)->default_value(6060),"tcp server port")
                    ;

            po::variables_map varmap;
            std::ifstream ifs( file_name.c_str());
            if( !ifs.is_open() )
                std::cout << "no such file." << std::endl;
            else
            {
                po::store( po::parse_config_file( ifs, opt_desc ), varmap );
                po::notify( varmap );
                std::cout << "read system config file done.\n" << std::endl;
            }

            SystemConfig::max_distance*=1e-6;
        }
    };

    double SystemConfig::max_distance;
    int SystemConfig::http_port;
    int SystemConfig::tcp_port;
}



#endif //NANOWIREPLANNER_SYSTEMCONFIG_HPP
