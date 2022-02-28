# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
import fogros2

def generate_launch_description():
    ld = LaunchDescription()
    intermediate_transport = 'raw'
    topic_name = '/camera/image_raw'
    new_cloud_topic_name = topic_name + "/cloud"
    img_publisher_node = Node(
        package="image_transport_benchmarker", executable="image_pub", output="screen")
    image_listener_node = Node(
        package="image_transport_benchmarker", executable="raw_test_cloud", output="screen")
    encoder_node = Node(
        package='image_transport', executable='republish', output='screen',
            name='republish_node2', arguments=[
                'raw',  # Input
                intermediate_transport,  # Output
            ], remappings=[
                ('in', topic_name),
                ('out/' + intermediate_transport, topic_name + "/" + intermediate_transport ),
            ])
    decoder_node = Node(
            package='image_transport', executable='republish', output='screen',
                name='republish_node', arguments=[
                    intermediate_transport,  # Input
                    'raw',  # Output
                ], remappings=[
                    ('in/' + intermediate_transport, topic_name + "/" + intermediate_transport),
                    ('out', new_cloud_topic_name),
                ])

    image_listener_node_robot = Node(
        package="image_transport_benchmarker", executable="raw_test", output="screen")
    ld.add_action(img_publisher_node)
    ld.add_action(image_listener_node)
    ld.add_action(image_listener_node_robot)    
    ld.add_action(encoder_node)
    ld.add_action(decoder_node)
    return ld
