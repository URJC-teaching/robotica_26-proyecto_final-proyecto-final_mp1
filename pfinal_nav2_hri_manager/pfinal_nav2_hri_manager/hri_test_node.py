# Copyright 2025 - Final Project ROS2 Jazzy
# Nodo de prueba independiente para HRI (TTS + STT)
# Estilo: RepeatNode y HRIExampleClient de los ejemplos base

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from audio_common_msgs.action import TTS
from whisper_msgs.action import STT
from enum import IntEnum


class State(IntEnum):
    INIT        = 0
    WAIT_INTRO  = 1
    LISTEN      = 2
    WAIT_LISTEN = 3
    REPEAT      = 4
    WAIT_REPEAT = 5
    DONE        = 6


class HRITestNode(Node):
    """
    Nodo de prueba standalone para HRI.
    Dice un prompt, escucha y repite lo que ha oído.
    Útil para verificar TTS/STT antes de integrar con Nav2.
    Estilo timer-based (sin blocking spin_until_future_complete).
    """

    def __init__(self):
        super().__init__('hri_test_node')

        self.tts_client = ActionClient(self, TTS, 'say')
        self.stt_client = ActionClient(self, STT, 'whisper/listen')

        self.state = State.INIT
        self.transcribed_text = ''

        self._tts_done = False
        self._stt_done = False

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('HRITest: iniciando prueba de TTS + STT')

    def _control_loop(self):
        if self.state == State.INIT:
            self._say('Hola. Voy a escucharte y repetir lo que digas. Habla cuando quieras.')
            self.state = State.WAIT_INTRO

        elif self.state == State.WAIT_INTRO:
            if self._tts_done:
                self.state = State.LISTEN

        elif self.state == State.LISTEN:
            self._listen()
            self.state = State.WAIT_LISTEN

        elif self.state == State.WAIT_LISTEN:
            if self._stt_done:
                if not self.transcribed_text.strip():
                    self.get_logger().warn('No se escuchó nada')
                    self.state = State.DONE
                else:
                    self.get_logger().info(f'Transcrito: "{self.transcribed_text}"')
                    self.state = State.REPEAT

        elif self.state == State.REPEAT:
            self._say(self.transcribed_text)
            self.state = State.WAIT_REPEAT

        elif self.state == State.WAIT_REPEAT:
            if self._tts_done:
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info('Prueba HRI finalizada')
            self.timer.cancel()

    # ---- TTS ----
    def _say(self, text: str):
        self._tts_done = False
        if not self.tts_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('TTS no disponible')
            self._tts_done = True
            return
        goal = TTS.Goal()
        goal.text = text
        self.get_logger().info(f'TTS: "{text}"')
        self.tts_client.send_goal_async(goal).add_done_callback(self._tts_response_cb)

    def _tts_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._tts_done = True
            return
        handle.get_result_async().add_done_callback(lambda _: setattr(self, '_tts_done', True))

    # ---- STT ----
    def _listen(self):
        self._stt_done = False
        if not self.stt_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('STT no disponible')
            self._stt_done = True
            return
        goal = STT.Goal()
        self.get_logger().info('STT: escuchando...')
        self.stt_client.send_goal_async(goal).add_done_callback(self._stt_response_cb)

    def _stt_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._stt_done = True
            return
        handle.get_result_async().add_done_callback(self._stt_result_cb)

    def _stt_result_cb(self, future):
        result = future.result().result
        text = ''
        if hasattr(result, 'transcription') and hasattr(result.transcription, 'text'):
            text = result.transcription.text
        self.transcribed_text = text
        self._stt_done = True


def main(args=None):
    rclpy.init(args=args)
    node = HRITestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
