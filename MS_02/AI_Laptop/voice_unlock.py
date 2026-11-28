"""
voice_unlock.py - Neural Calibrator AI twist (runs on the laptop, per the MS3 brief).

The web page shows a riddle. The player says the answer into the arm's MAX9814 mic.
This script:
  1. asks the Pico W to record 3 s of audio           (GET /record)
  2. waits until the recording is ready                 (GET /state -> rec.state == "ready")
  3. downloads it                                       (GET /audio.wav, 16 kHz, 16-bit, mono)
  4. runs offline speech recognition (Vosk, pre-trained English model) restricted to
     the answer word, so it only has to decide "answer word" vs "something else"
  5. decides CORRECT / WRONG / UNCLEAR / NO_SPEECH and (optionally) reports it to the
     Pico, which moves the state machine LOCKED -> WELCOME on CORRECT.

Edge cases handled (MS3 asks for this):
  - silence / nobody spoke          -> NO_SPEECH  (energy check before recognition)
  - mumbling / low model confidence  -> UNCLEAR   (ask the player to repeat)
  - a different word                 -> WRONG
  - Pico not reachable / recording never finishes -> retried, then a clear error

Setup (once):
  pip install -r requirements.txt
  Download "vosk-model-small-en-us-0.15" from https://alphacephei.com/vosk/models
  and unzip it next to this script (folder name: vosk-model-small-en-us-0.15).

Run (laptop connected to the Pico's "NeuralCalibrator" Wi-Fi):
  python voice_unlock.py                      # one attempt, Pico mic, answer "echo"
  python voice_unlock.py --answer clock --loop
  python voice_unlock.py --source laptop      # test with the laptop mic instead
  python voice_unlock.py --wav test.wav       # test on a saved recording
  python voice_unlock.py --report             # also send the result to the Pico
"""

import argparse
import io
import json
import sys
import time
import wave

import numpy as np
import requests

SAMPLE_RATE = 16000          # the Pico records at 16 kHz; Vosk models expect 16 kHz too
RECORD_SECONDS = 3           # matches REC_SECONDS in web_control.c
HTTP_TIMEOUT = 3.0           # seconds per request
HTTP_RETRIES = 3             # tries per request before giving up
READY_TIMEOUT = 8.0          # max wait for the Pico to finish recording

# Decision thresholds (tune on the real mic, print with --verbose)
SILENCE_RMS = 300            # 16-bit RMS below this = nobody spoke
MIN_CONFIDENCE = 0.60        # Vosk word confidence needed to accept the answer

CORRECT, WRONG, UNCLEAR, NO_SPEECH = "correct", "wrong", "unclear", "no_speech"


# ---------------------------------------------------------------- Pico communication
def http_get(url, **kw):
    """GET with retries, so one dropped Wi-Fi packet doesn't fail the whole attempt."""
    last = None
    for attempt in range(1, HTTP_RETRIES + 1):
        try:
            r = requests.get(url, timeout=HTTP_TIMEOUT, **kw)
            r.raise_for_status()
            return r
        except requests.RequestException as e:
            last = e
            time.sleep(0.3 * attempt)
    raise ConnectionError(f"Pico not reachable at {url}: {last}")


def record_on_pico(pico):
    """Ask the Pico to record, wait until it's ready, return the WAV bytes."""
    before = http_get(f"{pico}/state").json()
    if not before.get("mic", {}).get("ok", False):
        raise RuntimeError("Pico reports the microphone is not working (mic.ok = false)")
    old_id = before.get("rec", {}).get("id")

    http_get(f"{pico}/record")
    print(f"Recording {RECORD_SECONDS} s on the arm's mic... speak now")

    deadline = time.time() + READY_TIMEOUT
    while time.time() < deadline:
        rec = http_get(f"{pico}/state").json().get("rec", {})
        if rec.get("state") == "ready" and rec.get("id") != old_id:
            return http_get(f"{pico}/audio.wav", params={"id": rec["id"]}).content
        time.sleep(0.2)
    raise TimeoutError("Pico did not finish the recording in time")


def record_on_laptop():
    """Fallback for testing at home: record from the laptop's own mic."""
    import sounddevice as sd  # only needed for this mode

    print(f"Recording {RECORD_SECONDS} s on the laptop mic... speak now")
    audio = sd.rec(RECORD_SECONDS * SAMPLE_RATE, samplerate=SAMPLE_RATE,
                   channels=1, dtype="int16")
    sd.wait()
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(audio.tobytes())
    return buf.getvalue()


def report_to_pico(pico, result):
    """Tell the Pico the result. Needs a /unlock endpoint in web_control.c (Person 3)."""
    try:
        http_get(f"{pico}/unlock", params={"result": result})
        print(f"Reported '{result}' to the Pico")
    except ConnectionError as e:
        print(f"Could not report to the Pico ({e}). Is /unlock implemented yet?")


# ---------------------------------------------------------------- recognition
def load_wav(wav_bytes):
    """Return int16 samples, checking the format is what Vosk needs."""
    with wave.open(io.BytesIO(wav_bytes), "rb") as w:
        if w.getnchannels() != 1 or w.getsampwidth() != 2:
            raise ValueError("expected 16-bit mono WAV")
        if w.getframerate() != SAMPLE_RATE:
            raise ValueError(f"expected {SAMPLE_RATE} Hz, got {w.getframerate()} Hz")
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)


def load_model(path):
    from vosk import Model, SetLogLevel

    SetLogLevel(-1)  # hide Kaldi's start-up log
    try:
        return Model(path)
    except Exception as e:
        sys.exit(f"Could not load the Vosk model from '{path}': {e}\n"
                 "Download vosk-model-small-en-us-0.15 and unzip it next to this script.")


def recognise(model, samples, answer):
    """Return (heard_text, confidence_of_answer_word or 0)."""
    from vosk import KaldiRecognizer

    # Grammar mode: the recogniser may only output the answer word or [unk].
    # This makes a tiny vocabulary problem: much more reliable than free speech,
    # especially with accents and a cheap mic.
    rec = KaldiRecognizer(model, SAMPLE_RATE, json.dumps([answer, "[unk]"]))
    rec.SetWords(True)
    rec.AcceptWaveform(samples.tobytes())
    out = json.loads(rec.FinalResult())

    best = 0.0
    for w in out.get("result", []):
        if w.get("word") == answer:
            best = max(best, float(w.get("conf", 0.0)))
    return out.get("text", ""), best


def decide(model, wav_bytes, answer, verbose=False):
    samples = load_wav(wav_bytes)
    # Remove any DC offset left in the recording, then measure loudness.
    centred = samples.astype(np.float64) - samples.mean()
    rms = float(np.sqrt(np.mean(centred ** 2))) if len(centred) else 0.0
    if verbose:
        print(f"  audio RMS = {rms:.0f} (silence below {SILENCE_RMS})")
    if rms < SILENCE_RMS:
        return NO_SPEECH, "", 0.0

    text, conf = recognise(model, samples, answer)
    if verbose:
        print(f"  heard = '{text}', confidence for '{answer}' = {conf:.2f}")

    if conf >= MIN_CONFIDENCE:
        return CORRECT, text, conf
    if conf > 0:                     # model thinks it might be the answer, but isn't sure
        return UNCLEAR, text, conf
    return WRONG, text, conf         # loud enough, but only other words ([unk]) were heard


MESSAGES = {
    CORRECT:   "Correct! Interface unlocked.",
    WRONG:     "Wrong answer. Try again.",
    UNCLEAR:   "Didn't catch that clearly. Please say it again.",
    NO_SPEECH: "No speech heard. Speak closer to the microphone.",
}


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Neural Calibrator voice unlock (laptop AI)")
    ap.add_argument("--pico", default="http://192.168.4.1", help="Pico W web address")
    ap.add_argument("--answer", default="echo", help="riddle answer (one word)")
    ap.add_argument("--model", default="vosk-model-small-en-us-0.15", help="Vosk model folder")
    ap.add_argument("--source", choices=["pico", "laptop"], default="pico")
    ap.add_argument("--wav", help="recognise this WAV file instead of recording")
    ap.add_argument("--loop", action="store_true", help="keep trying until correct")
    ap.add_argument("--report", action="store_true", help="send the result to the Pico")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    answer = args.answer.lower().strip()

    model = load_model(args.model)
    print(f"Riddle answer: '{answer}'. Model loaded.")

    while True:
        try:
            if args.wav:
                with open(args.wav, "rb") as f:
                    wav_bytes = f.read()
            elif args.source == "laptop":
                wav_bytes = record_on_laptop()
            else:
                wav_bytes = record_on_pico(args.pico)

            result, text, conf = decide(model, wav_bytes, answer, args.verbose)
        except (ConnectionError, TimeoutError, RuntimeError, ValueError) as e:
            print(f"Error: {e}")
            if not args.loop or args.wav:
                return 2
            time.sleep(2)
            continue

        print(f"[{result.upper()}] {MESSAGES[result]}")
        if args.report and not args.wav:
            report_to_pico(args.pico, result)

        if result == CORRECT or not args.loop or args.wav:
            return 0 if result == CORRECT else 1
        time.sleep(1)


if __name__ == "__main__":
    sys.exit(main())
