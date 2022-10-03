#include <memory>
#include <random>
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

using litter_world_interfaces::msg::Pose;
using litter_world_interfaces::msg::GridRowStatus;    
using litter_world_interfaces::msg::GridStatus;       

#define MAX_SAME_POSE 8

class AgentAreaSensor : public Sensor
{
    public:
        AgentAreaSensor(const string& sensor_name, const vector<Belief>& proto_beliefs)
        : Sensor(sensor_name, proto_beliefs, false, false)
        {
            srand (time (NULL));
            
            this->declare_parameter("should_patrol", false);

            robot_name_ = this->get_parameter("agent_id").as_string();
            
            last_pose_.x = -1;
            last_pose_.y = -1;
            this->declare_parameter("detection_depth", 1); // plus minus 1 around the agent, thus 3x3 detection by default

            belief_set_subscriber_ = this->create_subscription<BeliefSet>("/"+robot_name_+"/belief_set", 
                    rclcpp::QoS(1).reliable(), 
                    [&](const BeliefSet::SharedPtr msg){
                        vector<Pose> free_poses;

                        for(Belief b : msg->value)
                        {
                            if(b.name == "in" && b.params[0] == robot_name_)
                                last_pose_ = extractPose(b.params[1]);//upd last believed pose
                            else if(b.name == "free")
                                free_poses.push_back(extractPose(b.params[0]));
                        }

                        if(free_poses.size() > 0)
                            rnd_free_pose_ = free_poses[rand() % free_poses.size()];
                    });
        

            litter_world_status_subscriber_ = this->create_subscription<GridStatus>("/litter_world_status", 
                    rclcpp::QoS(5).reliable(), bind(&AgentAreaSensor::areaItemsDetection, this, std::placeholders::_1));
        }

    private:

        /*
            Consider agent current position and update it via sensing
            Consider a square within the simulated detection_depth of the recycling_agent to update:
                - obstacles position (bins, walls, persons)
                - litter positions
        */
        void areaItemsDetection(const GridStatus::SharedPtr grid)
        {
            // STEP 1 agent position
            auto proto_belief_agent_pose = getBeliefPrototype("in");
            Pose curr_agent_pose = extractAgentPose(grid, (robot_name_ == "plastic_agent")? GridRowStatus().PLASTIC_AGENT_CELL : GridRowStatus().PAPER_AGENT_CELL);
            if(curr_agent_pose.x >= 0 and curr_agent_pose.y >= 0)
            {
                if(proto_belief_agent_pose.has_value())
                    senseNewAgentPose(curr_agent_pose, proto_belief_agent_pose.value());

                int detection_depth = this->get_parameter("detection_depth").as_int();
                detection_depth = detection_depth > 0? detection_depth : 1;
                
                auto proto_belief_detection_depth = getBeliefPrototype("detection_depth");
                if(proto_belief_detection_depth.has_value())
                {
                    auto detection_depth_belief = proto_belief_detection_depth.value();
                    detection_depth_belief.params[0] = robot_name_;
                    detection_depth_belief.value = detection_depth;
                    sense(detection_depth_belief, UpdOperation::ADD);
                }

                // STEP2 free cells
                auto proto_belief_free = getBeliefPrototype("free");
                if(proto_belief_free.has_value())
                {
                    senseCellsFreeStatus(grid, curr_agent_pose, proto_belief_free.value(), detection_depth);
                }

                 // STEP3 litter cells
                auto proto_belief_litter_pose = getBeliefPrototype("litter_pose");
                if(proto_belief_litter_pose.has_value())
                {
                    senseCellsLitterStatus(grid, curr_agent_pose, proto_belief_litter_pose.value(), detection_depth);
                }
            }

        }

        /*
            Sense new agent position (if changed)
        */
        void senseNewAgentPose(const Pose& curr_agent_pose, const Belief& proto_belief_agent_pose)
        {
            // old agent position belief 
            auto belief_agent_pose_del = proto_belief_agent_pose;
            belief_agent_pose_del.params[0] = robot_name_;
            belief_agent_pose_del.params[1] = buildCellName(last_pose_.x,last_pose_.y);

            // new agent position belief
            auto belief_agent_pose_add = proto_belief_agent_pose;
            belief_agent_pose_add.params[0] = robot_name_;
            belief_agent_pose_add.params[1] = buildCellName(curr_agent_pose.x,curr_agent_pose.y);

            if(curr_agent_pose.x != last_pose_.x || curr_agent_pose.y != last_pose_.y)
            {
                // agent position has changed
                same_pose_counter_ = 0; // reset same pose counter
                // delete old agent position
                sense(belief_agent_pose_del, UpdOperation::DEL);

                // add new agent position
                sense(belief_agent_pose_add, UpdOperation::ADD);
            }
            else
                same_pose_counter_ ++;
            
            if(same_pose_counter_ > MAX_SAME_POSE)
            {
                // make sure agent knows its current position
                sense(belief_agent_pose_add, UpdOperation::ADD);

                if(this->get_parameter("should_patrol").as_bool())
                {
                    std::cout << "In the same position for too long, triggering patrol mode..." << std::flush << std::endl;
                    senseShouldPatrol();
                    same_pose_counter_ = 0;
                }
            }
        }

        /*Pick believed to know rnd free cell and push the belief that you should sense it; done it when the agent seems to not be doing anything for too long*/
        void senseShouldPatrol()
        {
            auto should_patrol_b_proto = getBeliefPrototype("should_patrol");
            if(should_patrol_b_proto.has_value() && rnd_free_pose_.x >= 0 && rnd_free_pose_.y >= 0)
            {
                Belief should_patrol = should_patrol_b_proto.value();
                should_patrol.params[0] = "c_"+std::to_string(rnd_free_pose_.x)+"_"+std::to_string(rnd_free_pose_.y);
                std::cout << "Agent should patrol " << "c_"+std::to_string(rnd_free_pose_.x)+"_"+std::to_string(rnd_free_pose_.y) << std::flush << std::endl;
                sense(should_patrol, UpdOperation::ADD);
            }
        }

        /*
            Sense cell litter status within the detection depth, considering as centre the agent current position
        */
        void senseCellsLitterStatus(const GridStatus::SharedPtr grid, const Pose& curr_agent_pose, const Belief& proto_belief_litter_pose, const int& detection_depth)
        {
            BeliefSet freeLitterCells;
            BeliefSet litterCells;
            for(int i = curr_agent_pose.x - detection_depth; i <= curr_agent_pose.x + detection_depth; i++)
                for(int j = curr_agent_pose.y - detection_depth; j <= curr_agent_pose.y + detection_depth; j++)
                {
                    if(i >= 0 && i < grid->rows.size() && j >= 0 && j < grid->rows[i].cells.size())
                    {
                        // valid cell in grid

                        auto belief_paper_litter_pose = proto_belief_litter_pose;
                        belief_paper_litter_pose.params[0] = "pap" + std::to_string(i) + std::to_string(j);
                        belief_paper_litter_pose.params[1] = buildCellName(i, j);
                        
                        bool paper_in_ij = false;
                        
                        for(auto paper_pose : grid->litter_paper_poses)
                        {
                            if(paper_pose.x == i && paper_pose.y == j)
                            {
                                paper_in_ij = true;
                                auto paper_instance_b_proto = getBeliefPrototype("paper");
                                if(paper_instance_b_proto.has_value())
                                {
                                    Belief belief_paper_litter = paper_instance_b_proto.value();
                                    belief_paper_litter.name = "pap" + std::to_string(i) + std::to_string(j);
                                    litterCells.value.push_back(belief_paper_litter);
                                    litterCells.value.push_back(belief_paper_litter_pose);
                                    break;
                                }
                                
                                break;
                            }
                        }

                        if(!paper_in_ij)
                            freeLitterCells.value.push_back(belief_paper_litter_pose);

                        auto belief_plastic_litter_pose = proto_belief_litter_pose;
                        belief_plastic_litter_pose.params[0] = "pla" + std::to_string(i) + std::to_string(j);
                        belief_plastic_litter_pose.params[1] = buildCellName(i, j);
                        
                        bool plastic_in_ij = false;
                        
                        for(auto plastic_pose : grid->litter_plastic_poses)
                        {
                            if(plastic_pose.x == i && plastic_pose.y == j)
                            {
                                plastic_in_ij = true;
                                auto plastic_instance_b_proto = getBeliefPrototype("plastic");
                                if(plastic_instance_b_proto.has_value())
                                {
                                    Belief belief_plastic_litter = plastic_instance_b_proto.value();
                                    belief_plastic_litter.name = "pla" + std::to_string(i) + std::to_string(j);
                                    litterCells.value.push_back(belief_plastic_litter);
                                    litterCells.value.push_back(belief_plastic_litter_pose);
                                    break;
                                }
                            }
                        }

                        if(!plastic_in_ij)
                            freeLitterCells.value.push_back(belief_plastic_litter_pose);
                            
                    }
                }
            
            senseAll(freeLitterCells, UpdOperation::DEL);
            senseAll(litterCells, UpdOperation::ADD);
        }

        /*
            Sense cell free status within the detection depth, considering as centre the agent current position
        */
        void senseCellsFreeStatus(const GridStatus::SharedPtr grid, const Pose& curr_agent_pose, const Belief& proto_belief_free, const int& detection_depth)
        {
            BeliefSet freeCells;
            BeliefSet busyCells;
            for(int i = curr_agent_pose.x - detection_depth; i <= curr_agent_pose.x + detection_depth; i++)
                for(int j = curr_agent_pose.y - detection_depth; j <= curr_agent_pose.y + detection_depth; j++)
                {
                    if(i >= 0 && i < grid->rows.size() && j >= 0 && j < grid->rows[i].cells.size())
                    {
                        // cell in grid
                        Belief b = proto_belief_free;
                        UpdOperation updOp = UpdOperation::NOP;

                        if(grid->rows[i].cells[j] == GridRowStatus().OBSTACLE_CELL || grid->rows[i].cells[j] == GridRowStatus().PERSON_CELL) 
                        {    
                            // static (wall) or dynamic (person) obstacle
                            b.params[0] = buildCellName(i,j);
                            updOp = UpdOperation::DEL;
                        }
                        else if(grid->rows[i].cells[j] == GridRowStatus().PLASTIC_BIN_CELL || grid->rows[i].cells[j] == GridRowStatus().PAPER_BIN_CELL)
                        {    
                            // additional static (bins) obstacles
                            b.params[0] = buildCellName(i,j);
                            updOp = UpdOperation::DEL;
                        }
                        else if(robot_name_.find("plastic") != string::npos && grid->rows[i].cells[j] == GridRowStatus().PAPER_AGENT_CELL)
                        {    
                            // additional dynamic (agents) obstacles
                            b.params[0] = buildCellName(i,j);
                            updOp = UpdOperation::DEL;
                        }
                        else if(robot_name_.find("paper") != string::npos && grid->rows[i].cells[j] == GridRowStatus().PLASTIC_AGENT_CELL)
                        {    
                            // additional dynamic (agents) obstacles
                            b.params[0] = buildCellName(i,j);
                            updOp = UpdOperation::DEL;
                        }
                        else 
                        {    
                            // cells with litter or other agent TODO check whether to insert other agent's pose as busy cell
                            b.params[0] = buildCellName(i,j);
                            updOp = UpdOperation::ADD;
                        }
                        
                        if(updOp == UpdOperation::DEL)
                            busyCells.value.push_back(b);
                        else if(updOp == UpdOperation::ADD)
                            freeCells.value.push_back(b);
                    }
                }
            
            senseAll(busyCells, UpdOperation::DEL);
            senseAll(freeCells, UpdOperation::ADD);
        }

        /*
            Extract agent position in the grid
        */
        Pose extractAgentPose(const GridStatus::SharedPtr grid, const int16_t& agent_cell_value)
        {
            Pose result = Pose{};
            result.x = -1;
            result.y = -1;
            for(uint16_t i = 0 ; i < grid->rows.size(); i++)
                for(uint16_t j = 0 ; j < grid->rows[i].cells.size(); j++)
                    if(grid->rows[i].cells[j] == agent_cell_value)
                    {
                        result.x = i;
                        result.y = j;
                        return result;
                    }
            return result;
        }

        string buildCellName(const int& i, const int& j)
        {
            return "c_" + std::to_string(i) + "_" + std::to_string(j);
        }

        /*
            From cell name in format "c_x_y", extract x and y, embedding them into a Pose obj
        */
        Pose extractPose (string cell)
        {
            Pose res = Pose{};
            std::size_t firstU = cell.find_first_of("_");
            std::size_t secondU = cell.find_last_of("_");
            if(firstU != string::npos && secondU != string::npos)
            {
                if(firstU + 1 < cell.length() && secondU + 1 < cell.length())
                {
                    res.x = std::stoi(cell.substr(firstU+1, secondU - firstU - 1));
                    res.y = std::stoi(cell.substr(secondU + 1));
                }
            }

            return res;
        }

        string robot_name_;
        Pose last_pose_;
        Pose rnd_free_pose_;
        int same_pose_counter_ = 0;
        rclcpp::Subscription<BeliefSet>::SharedPtr belief_set_subscriber_;
        rclcpp::Subscription<GridStatus>::SharedPtr litter_world_status_subscriber_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  

  Belief b_proto_detection_depth = (ManagedBelief::buildMBFunction("detection_depth", {ManagedParam{"?a", "recycling_agent"}}, 0.0f)).toBelief();
  Belief b_proto_curr_pose = (ManagedBelief::buildMBPredicate("in", {ManagedParam{"?a", "recycling_agent"}, ManagedParam{"?c", "cell"}})).toBelief();
  Belief b_proto_free = (ManagedBelief::buildMBPredicate("free", {ManagedParam{"?c", "cell"}})).toBelief();
  Belief b_proto_plastic = (ManagedBelief::buildMBInstance("?p", "plastic")).toBelief();
  Belief b_proto_paper = (ManagedBelief::buildMBInstance("?p", "paper")).toBelief();
  Belief b_proto_litter_pose = (ManagedBelief::buildMBPredicate("litter_pose", {ManagedParam{"?l", "litter"}, ManagedParam{"?c", "cell"}})).toBelief();
  Belief b_proto_should_patrol = (ManagedBelief::buildMBPredicate("should_patrol", {ManagedParam{"?c", "cell"}})).toBelief();
  
  vector<Belief> proto_beliefs;
  proto_beliefs.push_back(b_proto_detection_depth);
  proto_beliefs.push_back(b_proto_curr_pose);
  proto_beliefs.push_back(b_proto_free);
  proto_beliefs.push_back(b_proto_plastic);
  proto_beliefs.push_back(b_proto_paper);
  proto_beliefs.push_back(b_proto_litter_pose);
  proto_beliefs.push_back(b_proto_should_patrol);
  auto node = std::make_shared<AgentAreaSensor>("area_detection_sensor", proto_beliefs);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
