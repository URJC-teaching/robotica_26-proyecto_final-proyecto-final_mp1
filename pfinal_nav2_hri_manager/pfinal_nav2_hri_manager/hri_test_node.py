import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient


class HRITestNode(Node):

    def __init__(self):
        super().__init__('hri_test_node')
        self.hri = HRIClient(self)
        self.state = 'WAIT_SERVICES'
        self.text = ''
        self.timer = self.create_timer(0.1, self._loop)

        self.get_logger().info('HRI test: esperando servicios TTS y STT...')

    def _loop(self):
        if self.state == 'WAIT_SERVICES':
            if self.hri.wait_for_services(timeout_sec=1.0):
                self.get_logger().info('Servicios HRI listos')
                self.hri.start_speaking('Hola. Voy a escucharte y repetir lo que digas.')
                self.state = 'SAYING_INTRO'

        elif self.state == 'SAYING_INTRO':
            if self.hri.is_speaking_done():
                self.hri.start_listen()
                self.state = 'LISTENING'

        elif self.state == 'LISTENING':
            if self.hri.is_listen_done():
                self.text = self.hri.get_listened_text()
                self.get_logger().info(f'Escuchado: "{self.text}"')
                if self.text.strip():
                    self.hri.start_speaking(self.text)
                    self.state = 'REPEATING'
                else:
                    self.get_logger().warning('No se escuchó nada')
                    self.state = 'DONE'

        elif self.state == 'REPEATING':
            if self.hri.is_speaking_done():
                self.state = 'DONE'

        elif self.state == 'DONE':
            self.get_logger().info('Prueba HRI finalizada')
            self.timer.cancel()
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = HRITestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
