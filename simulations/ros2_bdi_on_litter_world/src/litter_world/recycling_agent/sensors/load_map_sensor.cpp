#include <memory>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#include "litter_world_interfaces/msg/grid_status.hpp"

using std::string;
using std::vector;
using std::map;
using std::shared_ptr;
using std::bind;

using ros2_bdi_interfaces::msg::Belief;     
using ros2_bdi_interfaces::msg::BeliefSet;     
using BDIManaged::ManagedParam;
using BDIManaged::ManagedBelief;

using litter_world_interfaces::msg::GridRowStatus;    
using litter_world_interfaces::msg::GridStatus;       

#define PLASTIC_BIN_NAME "plastic_b"
#define PAPER_BIN_NAME "paper_b"

class LoadMapSensor : public Sensor
{
    public:
        LoadMapSensor(const string& sensor_name, const vector<Belief>& proto_beliefs)
        : Sensor(sensor_name, proto_beliefs, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            belief_set_subscriber_ = this->create_subscription<BeliefSet>("/"+robot_name_+"/belief_set", 
                    rclcpp::QoS(1).reliable(), 
                    [&](const BeliefSet::SharedPtr msg){belief_set_ = *msg;});

            litter_world_status_subscriber_ = this->create_subscription<GridStatus>("/litter_world_status", 
                    rclcpp::QoS(1).reliable(),
                    [&](const GridStatus::SharedPtr msg)
                        {
                            if(msg->rows.size() > 0)
                            {
                                BeliefSet bsetAddAll;
                                vector<Belief> nearBeliefs = (loadNearPredicates(msg));
                                for(Belief b : nearBeliefs)
                                    bsetAddAll.value.push_back(b);
                                // std::cout << "sensing " << nearBeliefs.size() << std::flush << std::endl;

                                vector<Belief> freeCellsBeliefs = (markEmptyCells(msg));
                                for(Belief b : freeCellsBeliefs)
                                    bsetAddAll.value.push_back(b);
                                // std::cout << "sensing " << freeCellsBeliefs.size() << std::flush << std::endl;

                                if(robot_name_ == "plastic_agent")
                                {
                                    Belief bPlaBin = getBeliefPrototype("plastic_bin").value();
                                    bPlaBin.name = PLASTIC_BIN_NAME;
                                    bsetAddAll.value.push_back(bPlaBin);
                                    // std::cout << "sensing " << bPlaBin.name << std::flush << std::endl;
                                }

                                if(robot_name_ == "paper_agent")
                                {
                                    Belief bPapBin = getBeliefPrototype("paper_bin").value();
                                    bPapBin.name = PAPER_BIN_NAME;
                                    bsetAddAll.value.push_back(bPapBin);
                                    // std::cout << "sensing " << bPapBin.name << std::flush << std::endl;
                                }

                                vector<Belief> binPoses = (extractBinPoses(msg));
                                for(Belief b : binPoses)
                                    bsetAddAll.value.push_back(b);
                                
                                // std::cout << "sensing " << binPoses.size() << " binPoses" << std::flush << std::endl;
                                if(belief_set_.value.size() < bsetAddAll.value.size())//still need to load all static info
                                    senseAll(bsetAddAll, UpdOperation::ADD);

                                else // loaded all static info -> can quit
                                {
                                    Belief bMapLoaded = getBeliefPrototype("map_loaded").value();
                                    sense(bMapLoaded, UpdOperation::ADD);
                                    for(auto b : belief_set_.value)
                                        if(b.name == "map_loaded")//map loaded and agent knows it->can quit
                                            rclcpp::shutdown();
                                }
                            }
                        });
        }

    private:

        /*
            Load all near predicates for the litter world static map
        */
        vector<Belief> loadNearPredicates(const GridStatus::SharedPtr grid)
        {
            vector<Belief> near_beliefs;
            std::optional<Belief> bproto = getBeliefPrototype("near");
            if(bproto.has_value())
            {
                for(uint16_t i = 0; i<grid->rows.size(); i++)
                    for(uint16_t j = 0; j<grid->rows[i].cells.size(); j++)
                    {   
                        Belief b = bproto.value();
                        string curr_cell = buildCellName(i, j);
                        // std::cout << "Considering " << curr_cell << " which is near to: ";
                        if(i > 0)//near to above cell
                        {
                            b.params = {curr_cell, buildCellName(i-1, j)};
                            // std::cout << buildCellName(i-1, j) << ", ";
                            near_beliefs.push_back(b);
                        }
                        if(i < grid->rows.size() - 1)//near to below cell
                        {
                            b.params = {curr_cell, buildCellName(i+1, j)};
                            // std::cout << buildCellName(i+1, j) << ", ";
                            near_beliefs.push_back(b);
                        }
                        if(j > 0)//near to left cell
                        {
                            b.params = {curr_cell, buildCellName(i, j-1)};
                            // std::cout << buildCellName(i, j-1) << ", ";
                            near_beliefs.push_back(b);
                        }
                        if(j < grid->rows[i].cells.size() - 1)//near to right cell
                        {
                            b.params = {curr_cell, buildCellName(i, j+1)};
                            // std::cout << buildCellName(i, j+1) << ", ";
                            near_beliefs.push_back(b);
                        }
                        // std::cout << std::flush << std::endl;
                    }
            }
            return (near_beliefs);
        }

        /*
           Extract bin poses
        */
       vector<Belief> extractBinPoses(const GridStatus::SharedPtr grid)
       {
            vector<Belief> bin_poses_beliefs;
            std::optional<Belief> bproto = getBeliefPrototype("bin_pose");
            if(bproto.has_value())
            {
                for(uint16_t i = 0; i<grid->rows.size(); i++)
                    for(uint16_t j = 0; j<grid->rows[i].cells.size(); j++)
                    {   
                        if(robot_name_ == "plastic_agent" && grid->rows[i].cells[j] == GridRowStatus().PLASTIC_BIN_CELL)
                        {
                            Belief b = bproto.value();
                            b.params = {PLASTIC_BIN_NAME,buildCellName(i, j)};
                            bin_poses_beliefs.push_back(b);
                            break;
                        }

                        if(robot_name_ == "paper_agent" && grid->rows[i].cells[j] == GridRowStatus().PAPER_BIN_CELL)
                        {
                            Belief b = bproto.value();
                            b.params = {PAPER_BIN_NAME,buildCellName(i, j)};
                            bin_poses_beliefs.push_back(b);
                            break;
                        }
                    }
            }

            return (bin_poses_beliefs);
       }

        /*
            Mark all empty cells for the litter world static map
        */
        vector<Belief> markEmptyCells(const GridStatus::SharedPtr grid)
        {
            vector<Belief> free_cells_beliefs;
            std::optional<Belief> bproto = getBeliefPrototype("free");
            if(bproto.has_value())
            {
                for(uint16_t i = 0; i<grid->rows.size(); i++)
                    for(uint16_t j = 0; j<grid->rows[i].cells.size(); j++)
                    {   
                        if(grid->rows[i].cells[j] != GridRowStatus().OBSTACLE_CELL && grid->rows[i].cells[j] != GridRowStatus().PLASTIC_BIN_CELL && grid->rows[i].cells[j] != GridRowStatus().PAPER_BIN_CELL)
                        {
                            Belief b = bproto.value();
                            b.params = {buildCellName(i, j)};
                            free_cells_beliefs.push_back(b);
                        }
                    }
            }

            return (free_cells_beliefs);
        }

        string buildCellName(const int& i, const int& j)
        {
            return "c_" + std::to_string(i) + "_" + std::to_string(j);
        }
        string robot_name_;
        BeliefSet belief_set_;
        rclcpp::Subscription<BeliefSet>::SharedPtr belief_set_subscriber_;
        rclcpp::Subscription<GridStatus>::SharedPtr litter_world_status_subscriber_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  Belief b_proto_near = (ManagedBelief::buildMBPredicate("near", {ManagedParam{"?c1", "cell"},ManagedParam{"?c2", "cell"}})).toBelief();
  Belief b_proto_free = (ManagedBelief::buildMBPredicate("free", {ManagedParam{"?c", "cell"}})).toBelief();
  Belief b_proto_plastic_bin = (ManagedBelief::buildMBInstance("?pb", "plastic_bin")).toBelief();
  Belief b_proto_paper_bin = (ManagedBelief::buildMBInstance("?pb", "paper_bin")).toBelief();
  Belief b_proto_bin_pose = (ManagedBelief::buildMBPredicate("bin_pose", {ManagedParam{"?b", "bin"}, ManagedParam{"?c", "cell"}})).toBelief();
  Belief b_proto_map_loaded = (ManagedBelief::buildMBPredicate("map_loaded", {})).toBelief();
  vector<Belief> proto_beliefs;
  proto_beliefs.push_back(b_proto_near);
  proto_beliefs.push_back(b_proto_free);
  proto_beliefs.push_back(b_proto_plastic_bin);
  proto_beliefs.push_back(b_proto_paper_bin);
  proto_beliefs.push_back(b_proto_bin_pose);
  proto_beliefs.push_back(b_proto_map_loaded);
  
  auto node = std::make_shared<LoadMapSensor>("load_map_sensor", proto_beliefs);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
