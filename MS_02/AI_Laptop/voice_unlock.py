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

import argparse                         # command-line options
import io                               # in-memory files for WAV data
import json                             # grammar for Vosk and its results
import sys                              # exit code
import time                             # sleep and timeouts
import wave                             # read and write WAV files

import numpy as np                      # audio sample maths
import requests                         # HTTP client

SAMPLE_RATE = 16000          # the Pico records at 16 kHz; Vosk models expect 16 kHz too
RECORD_SECONDS = 3           # matches REC_SECONDS in web_control.c
HTTP_TIMEOUT = 3.0           # seconds per request
HTTP_RETRIES = 3             # tries per request before giving up
READY_TIMEOUT = 8.0          # max wait for the Pico to finish recording

# Decision thresholds (tune on the real mic, print with --verbose)
SILENCE_RMS = 300            # 16-bit RMS below this = nobody spoke
MIN_CONFIDENCE = 0.60        # Vosk word confidence needed to accept the answer

CORRECT, WRONG, UNCLEAR, NO_SPEECH = "correct", "wrong", "unclear", "no_speech"  # the four possible results


# ---------------------------------------------------------------- Pico communication
def http_get(url, **kw):                # kw = extra options passed to requests.get
    """GET with retries, so one dropped Wi-Fi packet doesn't fail the whole attempt."""
    last = None                         # last error seen
    for attempt in range(1, HTTP_RETRIES + 1):          # try a few times
        try:                            # the request may fail
            r = requests.get(url, timeout=HTTP_TIMEOUT, **kw)           # send it
            r.raise_for_status()        # treat HTTP errors (404...) as failures
            return r                    # success
        except requests.RequestException as e:          # timeout, no connection, HTTP error...
            last = e                    # remember it for the message
            time.sleep(0.3 * attempt)   # wait a bit longer after each failure
    raise ConnectionError(f"Pico not reachable at {url}: {last}")       # all tries failed


def record_on_pico(pico):               # pico = base address, e.g. http://192.168.4.1
    """Ask the Pico to record, wait until it's ready, return the WAV bytes."""
    before = http_get(f"{pico}/state").json()           # state before recording
    if not before.get("mic", {}).get("ok", False):      # did the mic start?
        raise RuntimeError("Pico reports the microphone is not working (mic.ok = false)")  # no point in recording
    old_id = before.get("rec", {}).get("id")            # id of the previous recording

    http_get(f"{pico}/record")          # start recording
    print(f"Recording {RECORD_SECONDS} s on the arm's mic... speak now")  # tell the player

    deadline = time.time() + READY_TIMEOUT              # when to give up
    while time.time() < deadline:       # poll until then
        rec = http_get(f"{pico}/state").json().get("rec", {})           # current recording status
        if rec.get("state") == "ready" and rec.get("id") != old_id:     # finished, and it's a new one
            return http_get(f"{pico}/audio.wav", params={"id": rec["id"]}).content  # download it
        time.sleep(0.2)                 # wait before asking again
    raise TimeoutError("Pico did not finish the recording in time")     # took too long


def record_on_laptop():                 # used by --source laptop
    """Fallback for testing at home: record from the laptop's own mic."""
    import sounddevice as sd  # only needed for this mode

    print(f"Recording {RECORD_SECONDS} s on the laptop mic... speak now")  # tell the player
    audio = sd.rec(RECORD_SECONDS * SAMPLE_RATE, samplerate=SAMPLE_RATE,  # start recording in the background
                   channels=1, dtype="int16")           # 16-bit mono, like the Pico
    sd.wait()                           # wait until it has finished
    buf = io.BytesIO()                  # WAV file in memory
    with wave.open(buf, "wb") as w:     # write a WAV into it
        w.setnchannels(1)               # mono
        w.setsampwidth(2)               # 2 bytes = 16 bits per sample
        w.setframerate(SAMPLE_RATE)     # 16 kHz
        w.writeframes(audio.tobytes())  # the samples
    return buf.getvalue()               # the whole file as bytes


def report_to_pico(pico, result):       # result = one of the four result strings
    """Tell the Pico the result. Needs a /unlock endpoint in web_control.c (Person 3)."""
    try:                                # reporting is optional: don't crash
        http_get(f"{pico}/unlock", params={"result": result})           # send the result
        print(f"Reported '{result}' to the Pico")       # confirm it
    except ConnectionError as e:        # all retries failed
        print(f"Could not report to the Pico ({e}). Is /unlock implemented yet?")  # warn and carry on


# ---------------------------------------------------------------- recognition
def load_wav(wav_bytes):                # wav_bytes = a whole WAV file
    """Return int16 samples, checking the format is what Vosk needs."""
    with wave.open(io.BytesIO(wav_bytes), "rb") as w:   # open it from memory
        if w.getnchannels() != 1 or w.getsampwidth() != 2:              # must be 16-bit mono
            raise ValueError("expected 16-bit mono WAV")                # reject anything else
        if w.getframerate() != SAMPLE_RATE:             # must be 16 kHz
            raise ValueError(f"expected {SAMPLE_RATE} Hz, got {w.getframerate()} Hz")  # reject anything else
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)  # bytes -> int16 array


def load_model(path):                   # path = folder of the Vosk model
    from vosk import Model, SetLogLevel                 # imported only when recognition is needed

    SetLogLevel(-1)  # hide Kaldi's start-up log
    try:                                # the folder may be missing
        return Model(path)              # load the model
    except Exception as e:              # any loading problem
        sys.exit(f"Could not load the Vosk model from '{path}': {e}\n"  # explain it and quit...
                 "Download vosk-model-small-en-us-0.15 and unzip it next to this script.")  # ...saying how to fix it


def recognise(model, samples, answer):  # answer = the word we hope to hear
    """Return (heard_text, confidence_of_answer_word or 0)."""
    from vosk import KaldiRecognizer    # the recogniser class

    # Grammar mode: the recogniser may only output the answer word or [unk].
    # This makes a tiny vocabulary problem: much more reliable than free speech,
    # especially with accents and a cheap mic.
    rec = KaldiRecognizer(model, SAMPLE_RATE, json.dumps([answer, "[unk]"]))  # only two possible outputs
    rec.SetWords(True)                  # also report a confidence per word
    rec.AcceptWaveform(samples.tobytes())               # feed it all the audio
    out = json.loads(rec.FinalResult())                 # get the final result as a dict

    best = 0.0                          # best confidence so far
    for w in out.get("result", []):     # every word it heard
        if w.get("word") == answer:     # is it the answer?
            best = max(best, float(w.get("conf", 0.0)))                 # keep the highest confidence
    return out.get("text", ""), best    # text heard, and confidence


def decide(model, wav_bytes, answer, verbose=False):    # returns (result, text heard, confidence)
    samples = load_wav(wav_bytes)       # WAV -> samples
    # Remove any DC offset left in the recording, then measure loudness.
    centred = samples.astype(np.float64) - samples.mean()               # centre the samples on 0
    rms = float(np.sqrt(np.mean(centred ** 2))) if len(centred) else 0.0  # loudness (0 if empty)
    if verbose:                         # debug output wanted?
        print(f"  audio RMS = {rms:.0f} (silence below {SILENCE_RMS})")  # show the loudness
    if rms < SILENCE_RMS:               # too quiet
        return NO_SPEECH, "", 0.0       # nobody spoke

    text, conf = recognise(model, samples, answer)      # run the speech recognition
    if verbose:                         # debug output wanted?
        print(f"  heard = '{text}', confidence for '{answer}' = {conf:.2f}")  # show what was heard

    if conf >= MIN_CONFIDENCE:          # sure enough
        return CORRECT, text, conf      # right answer
    if conf > 0:                     # model thinks it might be the answer, but isn't sure
        return UNCLEAR, text, conf      # ask the player to repeat
    return WRONG, text, conf         # loud enough, but only other words ([unk]) were heard


MESSAGES = {                            # text shown for each result
    CORRECT:   "Correct! Interface unlocked.",          # right word, heard clearly
    WRONG:     "Wrong answer. Try again.",              # a different word
    UNCLEAR:   "Didn't catch that clearly. Please say it again.",       # maybe the right word, not sure
    NO_SPEECH: "No speech heard. Speak closer to the microphone.",      # too quiet
}                                       # end of messages


# ---------------------------------------------------------------- main
def main():                             # returns the exit code: 0 correct, 1 not, 2 error
    ap = argparse.ArgumentParser(description="Neural Calibrator voice unlock (laptop AI)")  # command-line parser
    ap.add_argument("--pico", default="http://192.168.4.1", help="Pico W web address")  # where the Pico is
    ap.add_argument("--answer", default="echo", help="riddle answer (one word)")  # the answer word
    ap.add_argument("--model", default="vosk-model-small-en-us-0.15", help="Vosk model folder")  # where the model is
    ap.add_argument("--source", choices=["pico", "laptop"], default="pico")  # which mic to use
    ap.add_argument("--wav", help="recognise this WAV file instead of recording")  # skip recording
    ap.add_argument("--loop", action="store_true", help="keep trying until correct")  # repeat until solved
    ap.add_argument("--report", action="store_true", help="send the result to the Pico")  # tell the Pico
    ap.add_argument("-v", "--verbose", action="store_true")             # debug output
    args = ap.parse_args()              # read the command line
    answer = args.answer.lower().strip()                # lower case, no spaces around it

    model = load_model(args.model)      # load once, use for every attempt
    print(f"Riddle answer: '{answer}'. Model loaded.")  # ready

    while True:                         # one pass = one attempt
        try:                            # recording may fail
            if args.wav:                # a saved file was given
                with open(args.wav, "rb") as f:         # open it
                    wav_bytes = f.read()                # use it as the recording
            elif args.source == "laptop":               # test mode
                wav_bytes = record_on_laptop()          # record from the laptop's mic
            else:                       # normal mode
                wav_bytes = record_on_pico(args.pico)   # record from the arm's mic

            result, text, conf = decide(model, wav_bytes, answer, args.verbose)  # recognise and judge it
        except (ConnectionError, TimeoutError, RuntimeError, ValueError) as e:  # known problems
            print(f"Error: {e}")        # show what went wrong
            if not args.loop or args.wav:               # single attempt: give up
                return 2                # exit code for an error
            time.sleep(2)               # wait before trying again
            continue                    # next attempt

        print(f"[{result.upper()}] {MESSAGES[result]}")                 # show the result
        if args.report and not args.wav:                # only report live attempts
            report_to_pico(args.pico, result)           # send it to the Pico

        if result == CORRECT or not args.loop or args.wav:              # finished?
            return 0 if result == CORRECT else 1        # exit code
        time.sleep(1)                   # short pause before the next attempt


if __name__ == "__main__":              # only when run directly, not when imported
    sys.exit(main())                    # pass the exit code to the shell
