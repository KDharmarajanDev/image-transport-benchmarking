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

from launch import FogROSLaunchDescription
from launch_ros.actions import Node
import fogros2

def generate_launch_description():
    ld = FogROSLaunchDescription()
    machine1 = fogros2.AWS(region="us-west-1", ec2_instance_type="t2.micro", ami_image="ami-09175f2ca3c3dc67c")
    img_publisher_node = Node(
        package="image_transport_benchmarker", executable="image_pub", output="screen")
    image_listener_node = fogros2.CloudNode(
        package="image_transport_benchmarker", executable="png_test_cloud", output="screen",
        machine = machine1, stream_topics=[('/camera/image_raw', 'compressed')],
        parameters=[{
            "/camera/image_raw/compressed/format": "png"
        }])
    image_listener_node_robot = Node(
        package="image_transport_benchmarker", executable="png_test", output="screen")
    ld.add_action(img_publisher_node)
    ld.add_action(image_listener_node)
    ld.add_action(image_listener_node_robot)
    return ld
