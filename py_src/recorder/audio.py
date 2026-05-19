#!/usr/bin/env python3
import sys
import time
import wave
import numpy as np
import pyaudio

# TODO: Replace with actual Unitree microphone client when available
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
# from unitree_sdk2py.go2.audio.audio_client import AudioClient
class AudioClient():
    def GetAudioSample(self):
        return -1, []


sample_rate     = 44100   # Hz
channels        = 2       # 1 = mono, 2 = stereo
sample_format   = pyaudio.paInt16
chunk_size      = 1024    # Frames per read
using_test_mic  = True
duration        = 4       # In seconds or None for endless



def _unitree_audio_client_init() -> AudioClient:
    # channel_id = 0
    # ChannelFactoryInitialize(channel_id)

    client = AudioClient()
    client.SetTimeout(3.0)
    client.Init()
    return client


def _get_chunk_unitree(client: AudioClient) -> bytes | None:
    code, data = client.GetAudioSample()
    if code != 0:
        print(f"[Error] Unitree Audio sampler error! (Code: {code})")
        return None
    return bytes(data)


def _test_mic_init() -> tuple[pyaudio.PyAudio, pyaudio.Stream]:
    pa = pyaudio.PyAudio()
    stream = pa.open(
        format=sample_format,
        channels=channels,
        rate=sample_rate,
        input=True,
        frames_per_buffer=chunk_size,
    )
    return pa, stream


def _get_chunk_test(stream: pyaudio.Stream) -> bytes | None:
    try:
        return stream.read(chunk_size, exception_on_overflow=False)
    except OSError as e:
        print(f"[Error] Could not read audio chunk: {e}")
        return None


def main():
    client = None
    pa     = None
    stream = None

    if not using_test_mic:
        client = _unitree_audio_client_init()
        print("[Recorder] Unitree microphone, initialised!")
    else:
        pa, stream = _test_mic_init()
        print("[Recorder] Local microphone, initialised!")

    output_path  = "output.wav"
    chunk_duration = chunk_size / sample_rate   

    wf = wave.open(output_path, "wb")
    wf.setnchannels(channels)
    wf.setsampwidth(pyaudio.get_sample_size(sample_format))
    wf.setframerate(sample_rate)

    total_chunks = None if duration is None else int((sample_rate / chunk_size) * duration)

    print("[Recorder] Recording loop, started!")
    try:
        chunk_count = 0
        while (total_chunks is None) or (chunk_count < total_chunks):
            start_time = time.perf_counter()

            # Fetch chunk
            chunk = None
            if client is not None:
                chunk = _get_chunk_unitree(client)
            if stream is not None:
                chunk = _get_chunk_test(stream)

            if chunk is None:
                break

            wf.writeframes(chunk)
            chunk_count += 1

            elapsed = time.perf_counter() - start_time
            remaining_sleep = chunk_duration - elapsed
            if remaining_sleep > 0 and client is not None:
                time.sleep(remaining_sleep)

        print("[Recorder] Recording loop, finished!")

    finally:
        wf.close()
        if stream is not None:
            stream.stop_stream()
            stream.close()
        if pa is not None:
            pa.terminate()
        print(f"[Recorder] Resources released. Audio saved to '{output_path}'")


if __name__ == "__main__":
    main()