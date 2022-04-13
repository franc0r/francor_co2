import rclpy
import serial

from rclpy.node import Node

from std_msgs.msg import String

class MeasurementData:
    def __init__(self):
        self._temp_C = 0.0
        self._pressure_hPa = 0.0
        self._humidity_rel = 0.0
        self._gas_resistance_MOhm = 0.0

    def convertFromString(self, msg):
        if len(msg) == 34:
            data_lst = msg.split('|')

            self._temp_C = float(data_lst[3]) / 100.0
            self._pressure_hPa = float(data_lst[4]) / 100.0
            self._humidity_rel = float(data_lst[5]) / 1000.0
            self._gas_resistance_MOhm = float(data_lst[7]) / 1000.0
        else:
            self.get_logger().error('Wrong length of message received!')

    def getInfoString(self):
        return "Temp: %.1f °C Pressure: %.2f hPa Humidity: %.3f %% Gas-Resistance: %.3f MOhm" % (self._temp_C, self._pressure_hPa, self._humidity_rel, self._gas_resistance_MOhm)

class CO2Interface(Node):

    def __init__(self):
        super().__init__('co2_interface')
        timer_period = 0.1
        self._timer = self.create_timer(timer_period, self.update)
        self._serial_name = '/dev/ttyFrancorCO2'
        self._timeout = 2
        self._state = 0
        self._data = MeasurementData()
        
        #self._serial_if = serial.Serial('/dev/ttyFrancorCO2')

        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def update(self):
        if self._state == 0:
            try:
                self._serial = serial.Serial(self._serial_name, timeout=self._timeout)
                self._state = 1
            except:
                self.get_logger().error("Failed to intialize serial interface %s" % self._serial_name)
                self._state = 0
                self._serial.close()
        elif self._state == 1:
            try:
                rx_msg = self._serial.readline().decode()
                self._data.convertFromString(rx_msg)
            except:
                self.get_logger().error('Failed to read from serial interface!')
                self._state = 0

            self.get_logger().info("%s" % self._data.getInfoString())

        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    co2_interface = CO2Interface()

    rclpy.spin(co2_interface)

    co2_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()