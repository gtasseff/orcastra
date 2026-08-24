#!/usr/bin/env python3
"""Build voice.bin, the PSRAM sidecar for orcastra_sd, from voice_data.c.

Format (see seed_voice_sample's ORCASTRA_SD_NO_VOICE branch in main.c):
    u32 magic  'VOX1' (0x31584F56 LE)
    u32 rate   sample rate in Hz
    u32 len    sample count
    s8  samples[len]

Stage on-device with:  h\\v\\s voice.bin 700000   (then launch; never reset
between the two - staged data does not survive a display reset).
"""
import re, struct, sys, pathlib

repo = pathlib.Path(__file__).resolve().parents[1]
src = repo / "apps" / "orcastra" / "voice_data.c"
out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else repo / "build" / "voice.bin"

text = src.read_text()
rate = int(re.search(r"voice_data_rate\s*=\s*(\d+)", text).group(1))
length = int(re.search(r"voice_data_len\s*=\s*(\d+)", text).group(1))
body = text[text.index("voice_data[") :]
body = body[body.index("{") + 1 : body.index("};")]
samples = [int(t) for t in re.findall(r"-?\d+", body)]
assert len(samples) == length, f"parsed {len(samples)} samples, header says {length}"
assert all(-128 <= s <= 127 for s in samples), "sample out of int8 range"

out.parent.mkdir(parents=True, exist_ok=True)
with open(out, "wb") as f:
    f.write(struct.pack("<III", 0x31584F56, rate, length))
    f.write(struct.pack(f"<{length}b", *samples))
print(f"{out}: {out.stat().st_size} bytes ({length} samples @ {rate} Hz)")
