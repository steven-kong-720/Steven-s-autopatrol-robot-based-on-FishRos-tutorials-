import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import pyttsx3


class Speaker(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.speech_service = self.create_service(
            SpeechText,
            'speech_text',
            self.speak_text_callback
        )

        self.speaker = pyttsx3.init()

        # 可选：调语速
        # self.speaker.setProperty('rate', 180)

        # 可选：调音量（0.0 ~ 1.0）
        # self.speaker.setProperty('volume', 1.0)

        # 尝试选择中文语音（如果系统里有）
        voices = self.speaker.getProperty('voices')
        for voice in voices:
            if 'zh' in voice.id.lower() or 'chinese' in voice.name.lower():
                self.speaker.setProperty('voice', voice.id)
                self.get_logger().info(f'使用语音: {voice.name}')
                break

    def speak_text_callback(self, request, response):
        self.get_logger().info(f'正在朗读: {request.text}')

        try:
            self.speaker.say(request.text)
            self.speaker.runAndWait()
            response.result = True
        except Exception as e:
            self.get_logger().error(f'朗读失败: {e}')
            response.result = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Speaker('speaker')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()