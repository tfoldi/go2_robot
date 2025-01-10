#!/usr/bin/env python3
# BSD 3-Clause License
#
# Copyright (c) 2024, Intelligent Robotics Lab
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from go2_tts_msgs.msg import TTSRequest
import requests
from pydub import AudioSegment
from pydub.playback import play
import io
from datetime import datetime
import os


class TTSNode(Node):
    def __init__(self):
        super().__init__("go2_tts_node")

        # Initialize parameters
        self.declare_parameter("elevenlabs_api_key", "")
        self.api_key = self.get_parameter("elevenlabs_api_key").value

        if not self.api_key:
            self.get_logger().error("ElevenLabs API key not provided!")
            return

        # Create subscription
        self.subscription = self.create_subscription(
            TTSRequest, "/tts", self.tts_callback, 10
        )

        # Create output directory for wave files
        self.output_dir = "tts_output"
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info("TTS Node initialized")

    def tts_callback(self, msg):
        """Handle incoming TTS requests"""
        try:
            self.get_logger().info(
                f'Received TTS request: "{msg.text}" with voice: {msg.voice_name}'
            )

            # Call ElevenLabs API
            audio_data = self.generate_speech(msg.text, msg.voice_name)

            if audio_data:
                # Save to WAV file
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{self.output_dir}/tts_{timestamp}.wav"
                self.save_wav(audio_data, filename)

                # Play audio
                self.play_audio(audio_data)

                self.get_logger().info(
                    f"Successfully processed TTS request. Saved to {filename}"
                )
            else:
                self.get_logger().error("Failed to generate speech")

        except Exception as e:
            self.get_logger().error(f"Error processing TTS request: {str(e)}")

    def generate_speech(self, text, voice_name):
        """Generate speech using ElevenLabs API"""
        url = f"https:#api.elevenlabs.io/v1/text-to-speech/{voice_name}"

        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.api_key,
        }

        data = {
            "text": text,
            "model_id": "eleven_monolingual_v1",
            "voice_settings": {"stability": 0.5, "similarity_boost": 0.5},
        }

        try:
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            return response.content

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"API request failed: {str(e)}")
            return None

    def save_wav(self, audio_data, filename):
        """Save audio data to WAV file"""
        try:
            # Convert MP3 to WAV
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            audio.export(filename, format="wav")
            self.get_logger().info(f"Saved WAV file: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving WAV file: {str(e)}")

    def play_audio(self, audio_data):
        """Play audio using pydub"""
        try:
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            play(audio)
        except Exception as e:
            self.get_logger().error(f"Error playing audio: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
