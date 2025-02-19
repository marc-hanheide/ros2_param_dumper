#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from rcl_interfaces.srv import ListParameters, GetParameters, DescribeParameters
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy_message_converter import message_converter
import time

class ParameterCollector(Node):

    server_call_timeout = 2

    def __init__(self):
        super().__init__('parameter_collector')
        self.all_params = {}

    def get_node_names(self):
        node_names_with_ns = self.get_node_names_and_namespaces()
        return [f"{ns}{name}" for name, ns in node_names_with_ns]

    def get_node_parameters(self, node_name):
        try:
            list_params_client = self.create_client(
                ListParameters,
                f'{node_name}/list_parameters'
            )
            
            if not list_params_client.wait_for_service(timeout_sec=self.server_call_timeout):
                self.get_logger().warn(f'Could not reach {node_name}')
                return {}

            request = ListParameters.Request()
            future = list_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.server_call_timeout)
            
            if future.result() is not None:
                param_names = future.result().result.names
                params = {}
                
                for param_name in param_names:
                    param_client = self.create_client(
                        GetParameters,
                        f'{node_name}/get_parameters'
                    )
                    describe_client = self.create_client(
                        DescribeParameters,
                        f'{node_name}/describe_parameters'
                    )
                    
                    if param_client.wait_for_service(timeout_sec=self.server_call_timeout):
                        
                        self.get_logger().info(f'Getting parameter {param_name} for {node_name}')
                        param_request = GetParameters.Request()
                        param_request.names = [param_name]
                        param_future = param_client.call_async(param_request)
                        rclpy.spin_until_future_complete(self, param_future, timeout_sec=self.server_call_timeout)
                        
                        if param_future.result() is not None:
                            self.get_logger().debug(f'Got parameter {param_name} for {node_name}')
                            value = param_future.result().values[0]
                            self.get_logger().debug(f'ParameterValue message: {param_name}: {value}')
                            parameter = parameter_value_to_python(value)
                            self.get_logger().debug(f'{param_name}: {parameter}')
                            try:
                                params[param_name] = parameter

                                #self.get_logger().debug(f'{param_name}: {params[param_name]}')
                            except Exception as e:
                                self.get_logger().warn(f'Error getting value for {param_name}: {str(e)}')

                    if describe_client.wait_for_service(timeout_sec=self.server_call_timeout):
                        
                        self.get_logger().info(f'Getting description for parameter {param_name} for {node_name}')
                        param_request = DescribeParameters.Request()
                        param_request.names = [param_name]
                        param_future = describe_client.call_async(param_request)
                        rclpy.spin_until_future_complete(self, param_future, timeout_sec=self.server_call_timeout)
                        
                        if param_future.result() is not None:
                            self.get_logger().debug(f'Got parameter {param_name} for {node_name}')
                            value = message_converter.convert_ros_message_to_dictionary(param_future.result().descriptors[0])
                            self.get_logger().debug(f'Description message: {param_name}: {value}')
                            params[param_name+"__DESCR"] = f"{value['description']}"
                            # parameter = parameter_value_to_python(value)
                            # self.get_logger().info(f'{param_name}: {parameter}')
                            # try:
                            #     params[param_name] = parameter

                            #     self.get_logger().info(f'{param_name}: {params[param_name]}')
                            # except Exception as e:
                            #     self.get_logger().warn(f'Error getting value for {param_name}: {str(e)}')


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

def output_markdown(params):
    node_documents = []
    for node_name, node_params in params.items():
        parameters = ""
        for param_name, param_value in node_params.items():
            if param_name.endswith("__DESCR"):
                continue
            parameters += f"| {param_name} | {param_value} | {type(param_value)} | {node_params.get(param_name+'__DESCR', '')} |\n"
        node_documents.append(f"""
## {node_name}

| Parameter | Value | Type | Description |
| --- | --- | --- | --- |
{parameters}

        """)

    output = f"""
# ROS 2 Parameters
*Generated by param_report.py*

{"".join(node_documents)}

    """
    return output

def main():
    rclpy.init()
    collector = ParameterCollector()
    
    # Collect parameters
    params = collector.collect_all_parameters()
    
    print(output_markdown(params))

    # Save to YAML file
    with open('ros2_parameters.yaml', 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    
    collector.get_logger().info('Parameters saved to ros2_parameters.yaml')
    
    collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

