#include <memory>
#include <vector>
#include <map>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#define CARRIER_L 1.0
#define CARRIER_W 0.44

#define BOX_H 0.2

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using std::string;
using std::vector;
using std::map;
using std::shared_ptr;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;    
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;        

using BDIManaged::ManagedBelief;

class CarrierMovingBoxesSensor : public Sensor
{
    public:
        CarrierMovingBoxesSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            current_position_sub_ = this->create_subscription<PointStamped>("/"+robot_name_+"_driver/gps", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&CarrierMovingBoxesSensor::carrierGPSCallback, this, _1));
            
            for (auto box : boxes_)
                boxes_positions_subs_.push_back(this->create_subscription<PointStamped>("/"+box+"/gps", 
                    rclcpp::QoS(5).best_effort(),
                    [=](const PointStamped::SharedPtr msg){boxGPSCallback(msg, box);}));

            current_mb_ = Belief();
            current_mb_.name = proto_belief.name;
            current_mb_.pddl_type = proto_belief.pddl_type;
            current_mb_.params = proto_belief.params;
            current_mb_.params[0] = robot_name_;
            current_mb_.value = -1.0f;
        }

        void performSensing()
        {
            float moving_counter = 0.0f;
            for(auto box : boxes_)
            {    
                if(boxes_positions_.count(box) > 0)
                {
                    if(boxWithinXBoundaries(box) && boxWithinYBoundaries(box) && boxWithinZBoundaries(box))
                        moving_counter += 1.0;
                }    
            }

            //UPDATE current value of moving by agent boxes    
            current_mb_.value = moving_counter;
            sense(current_mb_, ADD);
        }

    private:

        /*
            Returns true if the box is within the x boundaries of the carrier
        */
        bool boxWithinXBoundaries(const string& box)
        {
            return boxes_positions_[box].x > current_position_.x - CARRIER_W/2 && boxes_positions_[box].x < current_position_.x + CARRIER_W/2;
        }

        /*
            Returns true if the box is within the y boundaries of the carrier
        */
        bool boxWithinYBoundaries(const string& box)
        {
            return boxes_positions_[box].y > current_position_.y - CARRIER_L/2 && boxes_positions_[box].y < current_position_.y + CARRIER_L/2;
        }

        /*
            Returns true if the box is within the z boundaries of the carrier (i.e. just need to be above)
        */
        bool boxWithinZBoundaries(const string& box)
        {
            return boxes_positions_[box].z > current_position_.z && boxes_positions_[box].z < current_position_.z + BOX_H * 1.5;
        }

        void carrierGPSCallback(const PointStamped::SharedPtr msg)
        {
            current_position_ = msg->point;//current position
        }

        void boxGPSCallback(const PointStamped::SharedPtr msg, const string box)
        {   
            boxes_positions_[box] = msg->point;//update gps value for box a1
        }

        string robot_name_;
        Belief current_mb_;//current value of moving_boxes function for the agent 
        rclcpp::Subscription<PointStamped>::SharedPtr current_position_sub_;
        Point current_position_;
        vector<rclcpp::Subscription<PointStamped>::SharedPtr> boxes_positions_subs_;
        map<string,Point> boxes_positions_;
        vector<string> boxes_ = {"box_a1","box_a2","box_b1","box_b2","box_c1","box_c2"};//Boxes of the environment
        int counter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBFunction("moving_boxes", {""}, 0)).toBelief();
  auto node = std::make_shared<CarrierMovingBoxesSensor>("moving_boxes_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
