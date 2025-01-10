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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from go2_tts_msgs.msg import TTSRequest
from unitree_api.msg import Request
import requests
import json
from pydub import AudioSegment
from pydub.playback import play
import io
from datetime import datetime
import os
import base64
import time

class TTSNode(Node):
    def __init__(self):
        super().__init__('go2_tts_node')
        
        # Initialize parameters
        self.declare_parameter('elevenlabs_api_key', '')
        self.declare_parameter('local_playback', False)  # Default to robot playback
        
        self.api_key = self.get_parameter('elevenlabs_api_key').value
        self.local_playback = self.get_parameter('local_playback').value
        
        if not self.api_key:
            self.get_logger().error('ElevenLabs API key not provided!')
            return

        # Create subscription for TTS requests
        self.subscription = self.create_subscription(
            TTSRequest,
            '/tts',
            self.tts_callback,
            10
        )
        
        # Create publisher for robot audio hub requests
        self.audio_pub = self.create_publisher(
            Request,
            '/api/audiohub/request',
            10
        )
        
        # Create output directory for wave files
        self.output_dir = 'tts_output'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f'TTS Node initialized ({"local" if self.local_playback else "robot"} playback)')

    def tts_callback(self, msg):
        """Handle incoming TTS requests"""
        try:
            self.get_logger().info(f'Received TTS request: "{msg.text}" with voice: {msg.voice_name}')
            
            # Call ElevenLabs API
            audio_data = self.generate_speech(msg.text, msg.voice_name)
            
            if audio_data:
                # Save to WAV file
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{self.output_dir}/tts_{timestamp}.wav"
                wav_data = self.save_wav(audio_data, filename)
                
                if self.local_playback:
                    # Play locally
                    self.play_audio(audio_data)
                else:
                    # Send to robot
                    self.play_on_robot(wav_data)
                
                self.get_logger().info(f'Successfully processed TTS request. Saved to {filename}')
            else:
                self.get_logger().error('Failed to generate speech')
                
        except Exception as e:
            self.get_logger().error(f'Error processing TTS request: {str(e)}')

    def generate_speech(self, text, voice_name):
        """Generate speech using ElevenLabs API"""
        url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_name}"
        
        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.api_key
        }
        
        data = {
            "text": text,
            "model_id": "eleven_monolingual_v1",
            "voice_settings": {
                "stability": 0.5,
                "similarity_boost": 0.5
            }
        }
        
        try:
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            return response.content
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'API request failed: {str(e)}')
            return None

    def save_wav(self, audio_data, filename):
        """Save audio data to WAV file and return the WAV data"""
        try:
            # Convert MP3 to WAV
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            
            # Export to file
            audio.export(filename, format="wav")
            self.get_logger().info(f'Saved WAV file: {filename}')
            
            # Return WAV data
            wav_io = io.BytesIO()
            audio.export(wav_io, format="wav")
            return wav_io.getvalue()
            
        except Exception as e:
            self.get_logger().error(f'Error saving WAV file: {str(e)}')
            return None

    def play_audio(self, audio_data):
        """Play audio locally using pydub"""
        try:
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            play(audio)
        except Exception as e:
            self.get_logger().error(f'Error playing audio: {str(e)}')

    def split_into_chunks(self, data, chunk_size=256*1024):
        """Split data into chunks of specified size"""
        return [data[i:i + chunk_size] for i in range(0, len(data), chunk_size)]

    def play_on_robot(self, wav_data):
        """Send audio to robot's audio hub in chunks"""
        try:
            identity = int(time.time())
            chunks = self.split_into_chunks(wav_data)
            total_chunks = len(chunks)
            
            self.get_logger().info(f'Sending audio in {total_chunks} chunks')
            
            # Start audio
            start_req = Request()
            start_req.header.identity.id = identity
            start_req.header.identity.api_id = 4001
            start_req.header.lease.id = 0
            start_req.header.policy.priority = 0
            start_req.header.policy.noreply = True
            start_req.parameter = ''
            start_req.binary = []
            
            self.audio_pub.publish(start_req)
            
            # Send WAV data in chunks
            for chunk_idx, chunk in enumerate(chunks, 1):
                wav_req = Request()
                wav_req.header.identity.id = identity
                wav_req.header.identity.api_id = 4003
                wav_req.header.lease.id = 0
                wav_req.header.policy.priority = 0
                wav_req.header.policy.noreply = True
                
                audio_block = {
                    "current_block_index": chunk_idx,
                    "total_block_number": total_chunks,
                    "block_content": base64.b64encode(chunk).decode('utf-8')
                }
                wav_req.parameter = json.dumps(audio_block)
                wav_req.binary = []
                
                self.audio_pub.publish(wav_req)
                self.get_logger().info(f'Sent chunk {chunk_idx}/{total_chunks} ({len(chunk)} bytes)')
                
                # Add a small delay between chunks to prevent flooding
                time.sleep(0.01)

            # End audio
            end_req = Request()
            end_req.header.identity.id = identity
            end_req.header.identity.api_id = 4002
            end_req.header.lease.id = 0
            end_req.header.policy.priority = 0
            end_req.header.policy.noreply = True
            end_req.parameter = ''
            end_req.binary = []
            
            self.audio_pub.publish(end_req)
            
            self.get_logger().info('Completed sending audio to robot')
            
        except Exception as e:
            self.get_logger().error(f'Error sending audio to robot: {str(e)}')

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

if __name__ == '__main__':
    main()