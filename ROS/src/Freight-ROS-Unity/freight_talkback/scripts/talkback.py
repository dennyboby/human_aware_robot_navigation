#!/usr/bin/env python

import rospy
import pyjokes
from gtts import gTTS
from rospkg import RosPack
from playsound import playsound
from time import localtime, strftime
from freight_navigation.msg import NavIntent

class Talkback:
    def __init__(self):
        self.package_path = RosPack().get_path('freight_talkback')
        self.intent = None
        rospy.Subscriber('/navigation_goal', NavIntent, self.callback)

    def get_joke_string(self):
        return pyjokes.get_joke(language='en', category= 'all')

    def get_time_string(self):
        return 'The time is ' + strftime("%I %M %p", localtime())

    def get_date_string(self):
        return 'Today is ' + strftime("%d %B %Y", localtime()) + '. Happy ' + strftime("%A", localtime())

    def process_intent(self):
        if self.intent.intent == 'convo':
            for action in self.intent.goals:
                if action == 4:
                    self.play_sound(self.get_time_string())
                elif action == 5:
                    self.play_sound(self.get_date_string())
                elif action == 6:
                    self.play_sound('I am Jarvis created by Denny and Nihal.')
                elif action == 7:
                    text = self.get_joke_string()
                    print(text)
                    self.play_sound(text)
                else:
                    self.play_sound('I am sorry, I don\'t understand you.')
        elif self.intent.intent == 'help':
            for action in self.intent.goals:
                if action == 1 or action == 2 or action == 3:
                    self.play_sound('The person is in room ' + str(action))
                elif action == 8:
                    self.play_sound('I can help you with the following:\
                                1. Time, 2. Date, 3. Joke, 4. Tell about myself,\
                                5. Transport objects and guide people,\
                                6. Information about the location of staffs')
                else:
                    self.play_sound('I am sorry, I don\'t understand you.')

    def play_sound(self, text):
        speech_audio = gTTS(text=text, lang='en', slow=False)
        speech_audio.save(self.package_path + '/data/audio.mp3')
        playsound(self.package_path + '/data/audio.mp3')

    def callback(self, data):
        if data.intent == 'convo' or data.intent == 'help':
            self.intent = data
            self.process_intent()

if __name__ == '__main__':
    rospy.init_node('talkback_node')
    navigation = Talkback()
    rospy.spin()