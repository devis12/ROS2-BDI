from launch_ros.actions import Node

class AgentAddons:

    def __init__(self, package = '', executable = '', name = '', specific_params = []): # constructor
        self.package = package # action name
        self.executable = executable # executable name
        self.name = name # node name
        self.specific_params =  specific_params # specific parameters (in addition to agent_id, group and co.)
    
    def to_node(self, namespace = '', base_params = []):
        return Node(
            package = self.package,
            executable = self.executable,
            name = self.name,
            namespace = namespace,
            output='screen',
            parameters = base_params + self.specific_params 
        )

class AgentAction(AgentAddons):
    def __init__(self, package = '', executable = '', name = '', specific_params = []):
        super().__init__(package, executable, name, specific_params)
    
class AgentSensor(AgentAddons):
    def __init__(self, package = '', executable = '', name = '', specific_params = []):
        super().__init__(package, executable, name, specific_params)