#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from rcl_interfaces.srv import ListParameters
from rclpy.parameter import Parameter
import time

class ParameterCollector(Node):
    def __init__(self):
        super().__init__('parameter_collector')
        self.all_params = {}

    def get_node_names(self):
        node_names_with_ns = self.get_node_names_and_namespaces()
        return [name for name, _ in node_names_with_ns]

    def get_node_parameters(self, node_name):
        try:
            list_params_client = self.create_client(
                ListParameters,
                f'/{node_name}/list_parameters'
            )
            
            if not list_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Could not reach {node_name}')
                return {}

            request = ListParameters.Request()
            future = list_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                param_names = future.result().result.names
                params = {}
                
                for param_name in param_names:
                    param_client = self.create_client(
                        rcl_interfaces.srv.GetParameters,
                        f'/{node_name}/get_parameters'
                    )
                    
                    if param_client.wait_for_service(timeout_sec=1.0):
                        param_request = rcl_interfaces.srv.GetParameters.Request()
                        param_request.names = [param_name]
                        param_future = param_client.call_async(param_request)
                        rclpy.spin_until_future_complete(self, param_future)
                        
                        if param_future.result() is not None:
                            value = param_future.result().values[0]
                            params[param_name] = Parameter.from_parameter_msg(value).value
                
                return params
            return {}
        except Exception as e:
            self.get_logger().error(f'Error getting parameters for {node_name}: {str(e)}')
            return {}

    def collect_all_parameters(self):
        nodes = self.get_node_names()
        for node_name in nodes:
            if node_name != self.get_name():  # Skip our own node
                self.all_params[node_name] = self.get_node_parameters(node_name)
        
        return self.all_params

def main():
    rclpy.init()
    collector = ParameterCollector()
    
    # Give some time for node discovery
    time.sleep(5.0)
    
    # Collect parameters
    params = collector.collect_all_parameters()
    
    # Save to YAML file
    with open('ros2_parameters.yaml', 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    
    collector.get_logger().info('Parameters saved to ros2_parameters.yaml')
    
    collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()