import speech_recognition as sr
import pyttsx3
import pywhatkit
import datetime
import wikipedia
import pyjokes

listener = sr.Recognizer()
engine = pyttsx3.init()
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[1].id)


def talk(text):
    engine.say(text)
    engine.runAndWait()


def take_command():
    command = ""
    try:
        with sr.Microphone() as source:
            listener.adjust_for_ambient_noise(source)  # Adjust for ambient noise
            print('listening...')
            while True:
                audio = listener.listen(source)
                try:
                    command = listener.recognize_google(audio).lower()
                    print("You said: " + command)
                    if 'hey alexa' in command:
                        command = command.replace('hey alexa', '')  # Remove wake word
                        break  # Exit loop when wake word is detected
                except sr.UnknownValueError:
                    print("Sorry, I didn't catch that. Please say 'Hey Alexa' again.")
    except sr.RequestError:
        print("Sorry, there was an issue with the speech recognition service.")
    return command




def run_alexa():
    command = take_command()
    print(command)
    if 'play' in command:
        song = command.replace('play', '')
        talk('playing ' + song)
        pywhatkit.playonyt(song)
    elif 'time' in command:
        time = datetime.datetime.now().strftime('%I:%M %p')
        talk('Current time is ' + time)
    elif 'who is' in command:
        person = command.replace('who is', '')
        info = wikipedia.summary(person, 1)
        print(info)
        talk(info)
    elif 'date' in command:
        talk('sorry, I have a headache')
    elif 'are you single' in command:
        talk('I am in a relationship with wifi')
    elif 'joke' in command:
        talk(pyjokes.get_joke())
    else:
        talk('Please say the command again.')


while True:
    run_alexa()
