import speech_recognition as sr  # convert speech to text
import time
import pyttsx3
from playsound import playsound
import re
import subprocess
import pygame

engine = pyttsx3.init()
engine.setProperty('rate', 150)


def _play_close():
    pygame.mixer.init()
    pygame.mixer.music.load(
    "/home/pi/edo_ws/src/calib/src/media/close.mp3")
    pygame.mixer.music.play()


def _play_listen():
    pygame.mixer.init()
    pygame.mixer.music.load(
            "/home/pi/edo_ws/src/calib/src/media/listening.mp3")
    pygame.mixer.music.play()


def _talk():
    input = sr.Recognizer()
    with sr.Microphone() as source:
        audio = input.listen(source)
        data = ""
        try:
            data = input.recognize_google(audio)
            print("Your question is, " + data)

        except sr.UnknownValueError:
            print("Sorry I did not hear your question, Please repeat again.")

    return data


def _respond(output):
    engine.say(output)
    engine.runAndWait()


def init():
    action_publisher = None
    velocity_publisher = None
    _respond("Hi, I am Bob your personal robot")

    while 1:
        text = _talk().lower()

        if text == 0:
            continue

        if "hi bob" not in text:
            continue

        _play_listen()

        attempt = 0
        while attempt < 3:

            command = _talk().lower()
            numbers = re.findall(r'\d+', command)

            if 'start' in command:
                _respond("Starting robot")
                if action_publisher is not None:
                    action_publisher.kill()

                command_sh = "/opt/ros/melodic/bin/rostopic pub -r 30 /robot_operation std_msgs/String start"
                action_publisher = subprocess.Popen(command_sh.split())
                break
            elif 'stop' in command:
                _respond("Stopping the robot")
                if action_publisher is not None:
                    action_publisher.kill()
                command_sh = "/opt/ros/melodic/bin/rostopic pub -r 30 /robot_operation std_msgs/String stop"
                action_publisher = subprocess.Popen(command_sh.split())
                break
            elif 'velocity' in command and numbers:
                if int(numbers[0]) < 0 or 10 < int(numbers[0]):
                    _respond("Speed can only have a value between 0 and 10")
                    break
                _respond("Setting speed to " + str(numbers))
                if velocity_publisher is not None:
                    velocity_publisher.kill()

                command_sh = "/opt/ros/melodic/bin/rostopic pub -r 1 /robot_velocity std_msgs/String " + numbers[0]
                action_publisher = subprocess.Popen(command_sh.split())
                break
            elif 'joke' in command:
                _respond("What did the man say to his dead robot?")
                time.sleep(2)
                _respond("Rust in peace.")
                time.sleep(0.5)
                _respond("HA HA HA")
                break
            elif 'candle' in command or 'home' in command:
                _respond("Homing robotic arm")
                command_sh = "/opt/ros/melodic/bin/rostopic pub  --once /robot_operation std_msgs/String home"
                subprocess.Popen(command_sh.split())
                break
            else:
                _respond("Operation not available")
                attempt = attempt + 1

        if attempt == 3:
            _play_close()


if __name__ == '__main__':
    init()
