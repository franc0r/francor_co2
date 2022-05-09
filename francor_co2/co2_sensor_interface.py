import rclpy
import serial

from rclpy.node import Node

from std_msgs.msg import Float32

class MeasurementData:
    def __init__(self):
        self._temp_C = 0.0
        self._pressure_hPa = 0.0
        self._humidity_rel = 0.0
        self._gas_resistance_MOhm = 0.0
        self._avg_humidity = 0.0
        self._co2_level = 0.0

    def convertFromString(self, msg):
        data_lst = msg.split('|')
        if len(data_lst) == 10:  
           self._temp_C = float(data_lst[3]) / 100.0
           self._pressure_hPa = float(data_lst[4]) / 100.0
           self._humidity_rel = float(data_lst[5]) / 1000.0
           self._gas_resistance_MOhm = float(data_lst[7]) / 1000.0
           self._avg_humidity = float(data_lst[7]) / 1000.0
           self._co2_level = float(data_lst[9]) / 100.0
        else:
           raise Exception('Wrong number of data received!')

    def getInfoString(self):
        return "Temp: %.1f °C Pressure: %.2f hPa Humidity: %.3f %% Gas-Resistance: %.3f MOhm" % (self._temp_C, self._pressure_hPa, self._humidity_rel, self._gas_resistance_MOhm)

    def getCO2Level(self):
        return self._co2_level

class CO2Interface(Node):

    def __init__(self):
        super().__init__('co2_interface')

        self._state = 0
        self._data = MeasurementData()

        self.__readParams()
        self.__createPublishers()
        self.__createTimers()

    def update(self):
        if self._state == 0:
            try:
                self._serial = serial.Serial(self._serial_name.value, timeout=self._serial_timeout.value)
                self._state = 1
            except:
                self.get_logger().error("Failed to intialize serial interface %s" % self._serial_name.value)
                self._state = 0
        elif self._state == 1:
            try:
                rx_msg = self._serial.readline().decode()
                self._data.convertFromString(rx_msg)
                self.get_logger().info("%s | CO2-Level: %f" % (self._data.getInfoString(), self._data.getCO2Level()))
            
                msg = Float32()
                msg.data = float(self._data.getCO2Level())
                self._co2_publisher.publish(msg)
            except:
                self.get_logger().error('Failed to read from serial interface!')
                self._state = 0

    def __readParams(self):
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('serial_timeout', 2.0)
        self.declare_parameter('serial_name', '/dev/ttyCO2sensor')

        self._update_rate_hz = rclpy.parameter.Parameter(
            'update_rate_hz',
            rclpy.Parameter.Type.DOUBLE,
            1.0
        )

        self._serial_timeout = rclpy.parameter.Parameter(
            'serial_timeout',
            rclpy.Parameter.Type.DOUBLE,
            2.0
        )

        self._serial_name = rclpy.parameter.Parameter(
            'serial_name',
            rclpy.Parameter.Type.STRING,
            '/dev/ttyCO2Sensor'
        )

    def __createTimers(self):
        self._timer = self.create_timer(1.0 / self._update_rate_hz.value, self.update)

    def __createPublishers(self):
        self._co2_publisher = self.create_publisher(Float32, 'co2_level', 10)

def main(args=None):
    rclpy.init(args=args)

    co2_interface = CO2Interface()

    rclpy.spin(co2_interface)

    co2_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()