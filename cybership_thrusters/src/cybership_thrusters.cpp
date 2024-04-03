#include "cybership_thrusters/cybership_thrusters.hpp"

CybershipThruster::CybershipThruster() :
        rclcpp::Node("cybership_thruster",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{


}

void CybershipThruster::initialize()
{
    this->f_initialize();
}

void CybershipThruster::f_initialize()
{

    std::cout << "reading the parameters" << std::endl;
    std::map<std::string, rclcpp::Parameter> parameter_map;

    this->get_parameters("thrusters", parameter_map);
    std::set<std::string> thruster_names;
    for(auto & key_value : parameter_map){
        size_t pos = key_value.first.find('.');
        auto thruster_name = key_value.first.substr(0, pos);
        thruster_names.insert(thruster_name);
    }

    for(auto & thruster_name : thruster_names){
        // TODO: replace it with declare parameter

        std::string thruster_type;
        this->get_parameter_or<std::string>("thrusters." + thruster_name + ".type", thruster_type, std::string());

        std::cout << "thruster name: " << thruster_name << " type: " << thruster_type << std::endl;

        if(thruster_type == "vsp"){
            auto thruster = std::make_shared<VoithSchneider>(
                this->shared_from_this(), thruster_name);
            m_thrusters.push_back(thruster);
        }
        else if(thruster_type == "fixed"){
            auto thruster = std::make_shared<Fixed>(
                this->shared_from_this(), thruster_name);
            m_thrusters.push_back(thruster);
        }
        else if(thruster_type == "azimuth"){
            auto thruster = std::make_shared<Azimuth>(
                this->shared_from_this(), thruster_name);
            m_thrusters.push_back(thruster);
        }
        else{
            std::cerr << "Unknown thruster type: " << thruster_type << std::endl;
        }

    }

}

