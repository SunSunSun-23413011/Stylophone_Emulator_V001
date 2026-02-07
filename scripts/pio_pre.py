from pathlib import Path

Import("env")


def _patch_esp8266audio_midi_guard():
    lib_src = (
        Path(env.subst("$PROJECT_LIBDEPS_DIR"))
        / env.subst("$PIOENV")
        / "ESP8266Audio"
        / "src"
    )

    files = [
        lib_src / "AudioGeneratorMIDI.h",
        lib_src / "AudioGeneratorMIDI.cpp",
    ]
    old = "#if defined(ESP32) && (__GNUC__ >= 8) && (__XTENSA__)"
    new = "#if defined(ESP32)"

    for path in files:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        if old not in text:
            continue
        path.write_text(text.replace(old, new), encoding="utf-8")
        print("Patched ESP8266Audio guard:", path)


_patch_esp8266audio_midi_guard()
