#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os
#from ament_index_python.packages import get_package_share_directory
#wav_dir = get_package_share_directory('xf_mic_asr_offline')
#wav_path = os.path.join(wav_dir, 'share/xf_ring_mic_asr_offline/feedback_voice')
wav_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/xf_mic_asr_offline/feedback_voice'

def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    else:    
        return os.path.join(wav_path, 'english', f + '.wav')

def play(voice, volume=80, language='Chinese'):
    try:
        os.system('aplay -q -fS16_LE -r16000 -c1 -N --buffer-size=81920 ' + get_path(voice, language))
    except BaseException as e:
        print('error', e)

if __name__ == '__main__':
    play('ok')
    play('running', language="English")
    play('running')

