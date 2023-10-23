# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys
import threading

from bno055.connectors.i2c import I2C
from bno055.connectors.uart import UART
from bno055.error_handling.exceptions import BusOverRunException
from bno055.params.NodeParameters import NodeParameters
from bno055.sensor.SensorService import SensorService
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

class Bno055Node(Node):
    """
    ROS2 Node for interfacing Bosch Bno055 IMU sensor.

    :param Node: ROS2 Node Class to initialize from
    :type Node: ROS2 Node
    :raises NotImplementedError: Indicates feature/function is not implemented yet.
    """

    sensor = None
    param = None

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('bno055')
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/bno055/imu',  # Change topic if using imu_raw
            self.imu_callback,
            10  # Adjust the queue size as needed
        )
        
    def imu_callback(self, msg):
        # Extract linear acceleration data from the IMU message
        linear_acceleration = msg.linear_acceleration

        # Implement the logic for logging events here
        # Example: Log current velocity and orientation
        # Ensure you have initialized self.linear_velocity as [0.0, 0.0, 0.0] earlier
        dt = self.get_clock().now().to_msg().sec  # Calculate the time difference
        self.linear_velocity[0] += linear_acceleration.x * dt
        self.linear_velocity[1] += linear_acceleration.y * dt
        self.linear_velocity[2] += linear_acceleration.z * dt
        
        orientation = msg.orientation
        if orientation.w < 0.0:
            self.get_logger().info("IMU flipped over!")
        else:
            self.get_logger().info("IMU is upright.")

        max_acceleration = max(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z)
        if max_acceleration > 10.0:
            self.get_logger().warning("Sudden rapid acceleration detected!")



    def setup(self):
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:
        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(self,
                             self.param.uart_baudrate.value,
                             self.param.uart_port.value,
                             self.param.uart_timeout.value)
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            connector = I2C(self,
                            self.param.i2c_bus.value,
                            self.param.i2c_addr.value)
        else:
            raise NotImplementedError('Unsupported connection type: '
                                      + str(self.param.connection_type.value))

        # Connect to BNO055 device:
        connector.connect()

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()


def main(args=None):
    try:
        """Main entry method for this ROS2 node."""
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = Bno055Node()
        node.setup()

        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()

        def read_data():
            """Periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except BusOverRunException:
                # data not available yet, wait for next cycle | see #5
                return
            except ZeroDivisionError:
                # division by zero in get_sensor_data, return
                return
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving sensor data failed with %s:"%s"'
                                       % (type(e).__name__, e))
            finally:
                lock.release()

        def log_calibration_status():
            """Periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving calibration status failed with %s:"%s"'
                                       % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()

        # start regular sensor transmissions:
        # please be aware that frequencies around 30Hz and above might cause performance impacts:
        # https://github.com/ros2/rclpy/issues/520
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
        except UnboundLocalError:
            node.get_logger().info('No timers to shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

