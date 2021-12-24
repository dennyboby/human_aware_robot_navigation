#!/usr/bin/env python

import rospy
import rospkg
import struct
import wave
from rospkg import RosPack
from gtts import gTTS
from playsound import playsound
from threading import Thread
from picovoice import Picovoice
from pvrecorder import PvRecorder
from freight_navigation.msg import NavIntent

MAPPING = [
    {1 : ['1', 'doctor melissa', 'nurse jack', 'home']},
    {2 : ['2', 'doctor denny', 'nurse anna', 'cardiology lab']},
    {3 : ['3', 'doctor nihal', 'nurse andy', 'orthopedics lab']},
    {4 : ['time']},
    {5 : ['date']},
    {6 : ['who', 'what']},
    {7 : ['joke']},
    {8 : ['do', 'help']}
]
intent_inference = None
package_path = RosPack().get_path('freight_intent_recognition')

class IntentRecognition(Thread):
    def __init__(
            self,
            access_key,
            audio_device_index,
            keyword_path,
            context_path,
            porcupine_library_path=None,
            porcupine_model_path=None,
            porcupine_sensitivity=0.5,
            rhino_library_path=None,
            rhino_model_path=None,
            rhino_sensitivity=0.5,
            require_endpoint=True,
            output_path=None):
        super(IntentRecognition, self).__init__()

        self.intent_pub = rospy.Publisher('/navigation_goal',NavIntent ,queue_size=1)

        self._picovoice = Picovoice(
            access_key=access_key,
            keyword_path=keyword_path,
            wake_word_callback=self._wake_word_callback,
            context_path=context_path,
            inference_callback=self._inference_callback,
            porcupine_library_path=porcupine_library_path,
            porcupine_model_path=porcupine_model_path,
            porcupine_sensitivity=porcupine_sensitivity,
            rhino_library_path=rhino_library_path,
            rhino_model_path=rhino_model_path,
            rhino_sensitivity=rhino_sensitivity,
            require_endpoint=require_endpoint)

        self.audio_device_index = audio_device_index
        self.output_path = output_path

    @staticmethod
    def _wake_word_callback():
        print('[wake word]\n')

    @staticmethod
    def _inference_callback(inference):
        global intent_inference
        if inference.is_understood:
            print('{')
            print("  intent : '%s'" % inference.intent)
            print('  slots : {')
            for slot, value in inference.slots.items():
                print("    %s : '%s'" % (slot, value))
            print('  }')
            print('}\n')
            intent_inference = inference
        else:
            speech_text = "I didn't understand the command. Could you please repeat again?"
            print(speech_text)
            speech_audio = gTTS(text=speech_text, lang='en', slow=False)
            speech_audio.save(package_path + '/data/cmd_failure.mp3')
            Thread(target=playsound, args=(package_path + '/data/cmd_failure.mp3',), daemon=True).start()

    def pub_intent_msg(self, inference):
        msg = NavIntent()
        msg.intent = inference.intent
        for _, value in inference.slots.items():
            for objects in MAPPING:
                if value in list(objects.values())[0]:
                    msg.goals.append(list(objects.keys())[0])
                    break
        self.intent_pub.publish(msg)
        return True

    def run(self):
        global intent_inference
        recorder = None
        wav_file = None

        try:
            recorder = PvRecorder(device_index=self.audio_device_index, frame_length=self._picovoice.frame_length)
            recorder.start()

            if self.output_path is not None:
                wav_file = wave.open(self.output_path, "w")
                wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))

            print(f"Using device: {recorder.selected_device}")
            print('[Listening ...]')

            while not rospy.is_shutdown():
                pcm = recorder.read()

                if wav_file is not None:
                    wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

                self._picovoice.process(pcm)
                    
                if intent_inference is not None:
                    self.pub_intent_msg(intent_inference)
                    intent_inference = None

        except KeyboardInterrupt:
            print('Stopping ...')
        finally:
            if recorder is not None:
                recorder.delete()

            if wav_file is not None:
                wav_file.close()

            self._picovoice.delete()

    @classmethod
    def show_audio_devices(cls):
        devices = PvRecorder.get_audio_devices()

        for i in range(len(devices)):
            print(f'index: {i}, device name: {devices[i]}')

class LoadParams:
    def __init__(self):
        self.this_pack_path = rospkg.RosPack().get_path("freight_intent_recognition")
        self.access_key = None
        self.audio_device_index = None
        self.keyword_path = None
        self.context_path = None
        self.porcupine_library_path = None
        self.porcupine_model_path = None
        self.porcupine_sensitivity = None
        self.rhino_library_path = None
        self.rhino_model_path = None
        self.rhino_sensitivity = None
        self.require_endpoint = None
        self.output_path = None
        self.show_audio_devices = None
        self.load_params()

    def load_params(self):
        self.access_key = rospy.get_param("/access_key")
        self.audio_device_index = int(rospy.get_param("/audio_device_index"))
        self.keyword_path = self.this_pack_path + '/config/' + rospy.get_param("/keyword_file_name")
        self.context_path = self.this_pack_path + '/config/' + rospy.get_param("/context_file_name")
        self.porcupine_library_path = None if rospy.get_param("/wake_word_library_name") == "" \
                                        else self.this_pack_path + '/config/' + rospy.get_param("/wake_word_library_name")
        self.porcupine_model_path = None if rospy.get_param("/wake_word_model_name") == "" \
                                        else self.this_pack_path + '/config/' + rospy.get_param("/wake_word_model_name")
        self.porcupine_sensitivity = float(rospy.get_param("/wake_word_sensitivity"))
        self.rhino_library_path = None if rospy.get_param("/intent_library_name") == "" \
                                        else self.this_pack_path + '/config/' + rospy.get_param("/intent_library_name")
        self.rhino_model_path = None if rospy.get_param("/intent_model_name") == "" \
                                        else self.this_pack_path + '/config/' + rospy.get_param("/intent_model_name")
        self.rhino_sensitivity = float(rospy.get_param("/intent_recognition_sensitivity"))
        self.require_endpoint = bool(rospy.get_param("/require_end_silence"))
        self.output_path = None if rospy.get_param("/output_audio_fle_name") == "" \
                                else self.this_pack_path + '/config/' + rospy.get_param("/output_audio_fle_name")
        self.show_audio_devices = bool(rospy.get_param("/show_audio_devices"))

def main():
    rospy.init_node('intent_recognition_node')
    load_params = LoadParams()
    if load_params.show_audio_devices:
        IntentRecognition.show_audio_devices()
    else:
        if not load_params.keyword_path:
            raise ValueError("Missing path to Porcupine's keyword file.")

        if not load_params.context_path:
            raise ValueError("Missing path to Rhino's context file.")

        IntentRecognition(
            access_key=load_params.access_key,
            audio_device_index=load_params.audio_device_index,
            keyword_path=load_params.keyword_path,
            context_path=load_params.context_path,
            porcupine_library_path=load_params.porcupine_library_path,
            porcupine_model_path=load_params.porcupine_model_path,
            porcupine_sensitivity=load_params.porcupine_sensitivity,
            rhino_library_path=load_params.rhino_library_path,
            rhino_model_path=load_params.rhino_model_path,
            rhino_sensitivity=load_params.rhino_sensitivity,
            require_endpoint=load_params.require_endpoint,
            output_path=load_params.output_path).run()

if __name__ == '__main__':
    main()
