import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import os
from langdetect import detect
from gtts import gTTS
import tempfile
import subprocess
import threading

class Speaker(Node):
    """
    语音播放服务端 Service, 与Client名字一致
    """
    def __init__(self):
        super().__init__('speaker')
        self.speech_service_ = self.create_service(SpeechText, 'speech_text', self.speech_text_callback)
        self.get_logger().info('语音服务已启动 (gTTS + langdetect)')

    def speech_text_callback(self, request, response):
        text = request.text
        self.get_logger().info(f'收到语音文本: "{text}"')

        try:
            lang = 'en' # detect(text) # 识别文本语言
            self.get_logger().info(f'自动检测语言为: {lang}')

            # 合成语音并保存到临时文件
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as fp:
                tmp_path = fp.name
                tts = gTTS(text=text, lang=lang)
                tts.save(tmp_path)

            os.system(f"mpg123 '{tmp_path}'")
            os.remove(tmp_path)

            # # 非阻塞播放语音并清理临时文件
            # def play_and_delete(path):
            #     subprocess.run(['mpg123', path])
            #     os.remove(path)

            # threading.Thread(target=play_and_delete, args=(tmp_path,), daemon=True).start()

            response.result = True
            self.get_logger().info('语音播放完成')

        except Exception as e:
            self.get_logger().error(f'语音播放失败: {e}')
            response.result = False

        return response
    
def main():
    rclpy.init()
    node = Speaker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
