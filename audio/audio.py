import pygame
import sys


def play_audio(sound: str):
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    # Keep the script alive until playback finishes
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python play.py <audiofile>")
        sys.exit(1)

    audio_file = sys.argv[1]
    play_audio(audio_file)
