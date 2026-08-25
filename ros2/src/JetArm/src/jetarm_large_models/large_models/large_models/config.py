#!/usr/bin/env python3
# encoding: utf-8
import os
import dashscope

# 阶跃星辰key
stepfun_api_key = ''
stepfun_base_url = 'https://api.stepfun.com/v1'
stepfun_llm_model = ''
#'step-1v-8k'/'step-1o-vision-32k'/'step-1.5v-mini'
stepfun_vllm_model = 'step-1o-vision-32k'

# 阿里云key
aliyun_api_key = ''
aliyun_base_url = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
aliyun_llm_model = 'qwen-max-latest'
aliyun_vllm_model = 'qwen-vl-max-latest'
aliyun_tts_model = 'sambert-zhinan-v1'
aliyun_asr_model = 'paraformer-realtime-v2'
dashscope.api_key = aliyun_api_key

# 实际调用的key
api_key = aliyun_api_key
base_url = aliyun_base_url
asr_model = aliyun_asr_model
tts_model = aliyun_tts_model
llm_model = aliyun_llm_model
vllm_model = aliyun_vllm_model

# 获取程序所在路径
code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])
audio_path = os.path.join(code_path, 'resources/audio')

# 录音音频的路径
recording_audio_path = os.path.join(audio_path, 'recording.wav')

# 语音合成音频的路径
tts_audio_path = os.path.join(audio_path, "tts_audio.wav")

# 启动音频的路径
start_audio_path = os.path.join(audio_path, "start_audio.wav")

# 唤醒回答音频的路径
wakeup_audio_path = os.path.join(audio_path, "wakeup.wav")

# 出错音频的路径
error_audio_path = os.path.join(audio_path, "error.wav")

# 没有检测到声音时音频的路径
no_voice_audio_path = os.path.join(audio_path, "no_voice.wav")

# 录音完成时音频的路径
dong_audio_path = os.path.join(audio_path, "dong.wav")

record_finish_audio_path = os.path.join(audio_path, "record_finish.wav")

start_track_audio_path = os.path.join(audio_path, "start_track.wav")

track_fail_audio_path = os.path.join(audio_path, "track_fail.wav")
