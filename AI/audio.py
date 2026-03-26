import pygame


def play_audio(sound: str):
    pygame.mixer.init()
    pygame.mixer.music.load("AI/audio_fragments/"+ sound +".mp3")
    pygame.mixer.music.play()

    # Keep the script alive until playback finishes
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)