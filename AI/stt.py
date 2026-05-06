from whisper_trt import load_trt_model

model = load_trt_model("tiny.en")

result = model.transcribe("speech.wav")

print(result['text'])