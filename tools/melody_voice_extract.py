#!/usr/bin/env python3
"""Extract candidate melodic voices from a polyphonic MIDI arrangement.

The input file used by this project has one piano track and no vocal track.
This tool therefore separates monophonic voices first, instead of assuming
that MIDI channel 0 is the melody.  It uses Partitura's implementation of
the Contig Mapping voice-separation algorithm and writes every detected voice
to one MIDI instrument.  The selected voice is also written as JSON and a
small STM32-friendly C array.

Install the optional dependency with::

    python -m pip install pretty_midi partitura

Example::

    python tools/melody_voice_extract.py tools/song.mid --voice 1

Voice 1 is the first high voice reported by Partitura.  It is a candidate,
not a guarantee of the sung line; use the generated multi-track MIDI to
audition another voice when the source has no explicit vocal track.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

try:
    import partitura.musicanalysis as music_analysis
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit(
        "partitura is required for voice separation. Install it with: "
        "python -m pip install partitura"
    ) from exc

try:
    import pretty_midi
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit(
        "pretty_midi is required. Install it with: "
        "python -m pip install pretty_midi"
    ) from exc


@dataclass(frozen=True)
class SourceNote:
    """A source note with absolute time in seconds."""

    note_id: int
    pitch: int
    start: float
    end: float
    velocity: int


@dataclass
class MelodyNote:
    """A monophonic output note."""

    pitch: int
    start: float
    end: float
    velocity: int

    @property
    def duration(self) -> float:
        return self.end - self.start


def load_notes(path: Path) -> tuple[pretty_midi.PrettyMIDI, list[SourceNote]]:
    """Load all non-drum notes, retaining overlapping same-pitch notes."""

    midi = pretty_midi.PrettyMIDI(str(path))
    notes: list[SourceNote] = []
    for instrument in midi.instruments:
        if instrument.is_drum:
            continue
        for note in instrument.notes:
            if note.end <= note.start:
                continue
            notes.append(
                SourceNote(
                    note_id=len(notes),
                    pitch=int(note.pitch),
                    start=float(note.start),
                    end=float(note.end),
                    velocity=int(note.velocity),
                )
            )
    notes.sort(key=lambda note: (note.start, note.pitch, note.end, note.note_id))
    return midi, notes


def _onset_groups(
    notes: Sequence[SourceNote], tolerance_seconds: float
) -> list[list[SourceNote]]:
    groups: list[list[SourceNote]] = []
    for note in sorted(notes, key=lambda item: (item.start, item.pitch, item.end)):
        if not groups or note.start - groups[-1][0].start > tolerance_seconds:
            groups.append([note])
        else:
            groups[-1].append(note)
    return groups


def prepare_candidates(
    notes: Sequence[SourceNote],
    min_pitch: int,
    max_pitch: int,
    min_duration_ms: float,
    onset_tolerance_ms: float,
    keep_per_onset: int,
) -> list[SourceNote]:
    """Filter malformed notes and cap very dense onset clusters.

    The cap bounds Partitura's graph size while retaining the upper notes and
    the longest articulation in each local chord.  Exact same-pitch notes in
    one onset cluster are collapsed only for this analysis input; the source
    notes used for output remain untouched.
    """

    minimum = min_duration_ms / 1000.0
    filtered = [
        note
        for note in notes
        if min_pitch <= note.pitch <= max_pitch and note.end - note.start >= minimum
    ]
    groups = _onset_groups(filtered, onset_tolerance_ms / 1000.0)
    candidates: list[SourceNote] = []
    for group in groups:
        by_pitch: dict[int, SourceNote] = {}
        for note in group:
            current = by_pitch.get(note.pitch)
            if current is None or (
                note.end - note.start,
                note.velocity,
                note.end,
            ) > (
                current.end - current.start,
                current.velocity,
                current.end,
            ):
                by_pitch[note.pitch] = note
        unique = list(by_pitch.values())
        unique.sort(
            key=lambda note: (
                note.pitch,
                note.end - note.start,
                note.velocity,
            ),
            reverse=True,
        )
        candidates.extend(unique[:keep_per_onset])
    return sorted(candidates, key=lambda note: (note.start, note.pitch, note.note_id))


def make_note_array(
    midi: pretty_midi.PrettyMIDI, notes: Sequence[SourceNote]
) -> np.ndarray:
    """Convert project notes to the structured array expected by Partitura."""

    dtype = np.dtype(
        [
            ("onset_sec", "f4"),
            ("duration_sec", "f4"),
            ("onset_tick", "i4"),
            ("duration_tick", "i4"),
            ("pitch", "i4"),
            ("velocity", "i4"),
            ("track", "i4"),
            ("channel", "i4"),
            ("id", "U32"),
        ]
    )
    result = np.zeros(len(notes), dtype=dtype)
    for index, note in enumerate(notes):
        start_tick = int(round(float(midi.time_to_tick(note.start))))
        end_tick = int(round(float(midi.time_to_tick(note.end))))
        result[index] = (
            note.start,
            note.end - note.start,
            start_tick,
            max(1, end_tick - start_tick),
            note.pitch,
            note.velocity,
            0,
            0,
            f"n{index}",
        )
    return result


def collapse_chords(
    notes: Iterable[SourceNote], onset_tolerance_ms: float
) -> list[MelodyNote]:
    """Make one voice monophonic by keeping the highest note per onset group."""

    groups = _onset_groups(list(notes), onset_tolerance_ms / 1000.0)
    result: list[MelodyNote] = []
    for group in groups:
        selected = max(
            group,
            key=lambda note: (
                note.pitch,
                note.end - note.start,
                note.velocity,
            ),
        )
        result.append(
            MelodyNote(
                pitch=selected.pitch,
                start=selected.start,
                end=selected.end,
                velocity=selected.velocity,
            )
        )
    return result


def merge_repeated_notes(
    notes: Sequence[MelodyNote], merge_gap_ms: float
) -> list[MelodyNote]:
    if not notes:
        return []
    maximum_gap = merge_gap_ms / 1000.0
    merged = [MelodyNote(**asdict(notes[0]))]
    for note in notes[1:]:
        previous = merged[-1]
        if note.pitch == previous.pitch and note.start - previous.end <= maximum_gap:
            previous.end = max(previous.end, note.end)
            previous.velocity = max(previous.velocity, note.velocity)
        else:
            merged.append(MelodyNote(**asdict(note)))
    return merged


def post_process(
    notes: Sequence[SourceNote],
    onset_tolerance_ms: float,
    merge_gap_ms: float,
    min_note_ms: float,
) -> list[MelodyNote]:
    result = collapse_chords(notes, onset_tolerance_ms)
    result = merge_repeated_notes(result, merge_gap_ms)
    minimum = min_note_ms / 1000.0
    result = [note for note in result if note.duration >= minimum]
    for current, following in zip(result, result[1:]):
        current.end = min(current.end, following.start)
    return [note for note in result if note.end > note.start]


def pitch_to_frequency(pitch: int) -> float:
    return 440.0 * 2.0 ** ((pitch - 69) / 12.0)


def build_output_notes(notes: Sequence[MelodyNote]) -> list[dict[str, int | float]]:
    output: list[dict[str, int | float]] = []
    rounded = [
        (round(note.start * 1000.0), max(1, round(note.duration * 1000.0)))
        for note in notes
    ]
    for index, note in enumerate(notes):
        start_ms, duration_ms = rounded[index]
        if index + 1 < len(notes):
            next_start = rounded[index + 1][0]
            gap_ms = max(0, next_start - start_ms - duration_ms)
        else:
            gap_ms = 0
        output.append(
            {
                "pitch": note.pitch,
                "frequency": round(pitch_to_frequency(note.pitch), 3),
                "start_ms": start_ms,
                "duration_ms": duration_ms,
                "gap_ms": gap_ms,
            }
        )
    return output


def sanitize_identifier(name: str) -> str:
    identifier = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not identifier:
        return "melody"
    if identifier[0].isdigit():
        return "_" + identifier
    return identifier


def format_c_array(
    notes: Sequence[dict[str, int | float]], array_name: str
) -> str:
    name = sanitize_identifier(array_name)
    lines = [
        "#include <stdint.h>",
        "",
        "typedef struct",
        "{",
        "    uint16_t freq;",
        "    uint32_t duration;",
        "    uint32_t gap;",
        "} Note;",
        "",
        f"static const Note {name}[] =",
        "{",
    ]
    for note in notes:
        frequency = round(float(note["frequency"]))
        duration = int(note["duration_ms"])
        gap = int(note["gap_ms"])
        if frequency > 65535:
            raise ValueError("a generated frequency does not fit in uint16_t")
        lines.append(f"    {{{frequency}, {duration}, {gap}}},")
    lines.extend(
        [
            "};",
            "",
            f"static const uint32_t {name}_count =",
            f"    sizeof({name}) / sizeof({name}[0]);",
            "",
        ]
    )
    return "\n".join(lines)


def initial_tempo(midi: pretty_midi.PrettyMIDI) -> float:
    _, values = midi.get_tempo_changes()
    return float(values[0]) if len(values) else 120.0


def write_midi(
    path: Path,
    voices: dict[int, Sequence[MelodyNote]],
    source_midi: pretty_midi.PrettyMIDI,
) -> None:
    output = pretty_midi.PrettyMIDI(
        resolution=source_midi.resolution,
        initial_tempo=initial_tempo(source_midi),
    )
    for voice_id in sorted(voices):
        instrument = pretty_midi.Instrument(
            program=0,
            name=f"Separated voice {voice_id}",
        )
        instrument.notes = [
            pretty_midi.Note(
                velocity=max(1, min(127, note.velocity)),
                pitch=note.pitch,
                start=note.start,
                end=note.end,
            )
            for note in voices[voice_id]
        ]
        output.instruments.append(instrument)
    output.write(str(path))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Separate candidate monophonic voices from a polyphonic MIDI"
    )
    parser.add_argument("input", type=Path, help="source MIDI file")
    parser.add_argument("--voice", type=int, default=1, help="voice used for JSON/C output")
    parser.add_argument("--output-midi", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-c", type=Path)
    parser.add_argument("--array-name", default="melody_voice_1")
    parser.add_argument("--min-pitch", type=int, default=70)
    parser.add_argument("--max-pitch", type=int, default=99)
    parser.add_argument("--min-duration-ms", type=float, default=35.0)
    parser.add_argument("--min-note-ms", type=float, default=35.0)
    parser.add_argument("--onset-tolerance-ms", type=float, default=35.0)
    parser.add_argument("--merge-gap-ms", type=float, default=80.0)
    parser.add_argument("--keep-per-onset", type=int, default=8)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    input_path = args.input.resolve()
    if not input_path.is_file():
        raise SystemExit(f"input MIDI does not exist: {input_path}")
    if args.voice < 1:
        raise SystemExit("--voice must be a positive voice number")
    if args.keep_per_onset < 1:
        raise SystemExit("--keep-per-onset must be positive")

    default_stem = input_path.with_name(f"{input_path.stem}_voices")
    output_midi = (args.output_midi or default_stem.with_suffix(".mid")).resolve()
    output_json = (args.output_json or default_stem.with_suffix(".json")).resolve()
    output_c = (
        args.output_c
        or input_path.with_name(f"{input_path.stem}_voice{args.voice}.h")
    ).resolve()
    for path in (output_midi, output_json, output_c):
        path.parent.mkdir(parents=True, exist_ok=True)

    source_midi, source_notes = load_notes(input_path)
    candidates = prepare_candidates(
        source_notes,
        min_pitch=args.min_pitch,
        max_pitch=args.max_pitch,
        min_duration_ms=args.min_duration_ms,
        onset_tolerance_ms=args.onset_tolerance_ms,
        keep_per_onset=args.keep_per_onset,
    )
    if not candidates:
        raise SystemExit("no candidate notes remain; lower --min-pitch or --min-duration-ms")

    note_array = make_note_array(source_midi, candidates)
    voices = music_analysis.estimate_voices(note_array, monophonic_voices=False)
    raw_by_voice: dict[int, list[SourceNote]] = {}
    for note, voice in zip(candidates, voices):
        raw_by_voice.setdefault(int(voice), []).append(note)
    processed = {
        voice_id: post_process(
            notes,
            onset_tolerance_ms=args.onset_tolerance_ms,
            merge_gap_ms=args.merge_gap_ms,
            min_note_ms=args.min_note_ms,
        )
        for voice_id, notes in raw_by_voice.items()
    }
    processed = {voice: notes for voice, notes in processed.items() if notes}
    if args.voice not in processed:
        available = ", ".join(str(voice) for voice in sorted(processed))
        raise SystemExit(f"voice {args.voice} is empty; available voices: {available}")

    write_midi(output_midi, processed, source_midi)
    selected_output = build_output_notes(processed[args.voice])
    summary = {
        "source": input_path.name,
        "algorithm": "partitura-contig-mapping-voice-separation",
        "candidate_count": len(candidates),
        "voice_count": len(processed),
        "selected_voice": args.voice,
        "parameters": {
            "min_pitch": args.min_pitch,
            "max_pitch": args.max_pitch,
            "min_duration_ms": args.min_duration_ms,
            "onset_tolerance_ms": args.onset_tolerance_ms,
            "merge_gap_ms": args.merge_gap_ms,
        },
        "voices": {
            str(voice): {
                "note_count": len(notes),
                "min_pitch": min(note.pitch for note in notes),
                "max_pitch": max(note.pitch for note in notes),
            }
            for voice, notes in sorted(processed.items())
        },
        "notes": selected_output,
    }
    output_json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    output_c.write_text(
        format_c_array(selected_output, args.array_name), encoding="utf-8"
    )

    print(f"Source notes     : {len(source_notes)}")
    print(f"Candidate notes  : {len(candidates)}")
    print(f"Separated voices : {len(processed)}")
    print(
        "Voice counts     : "
        + ", ".join(
            f"{voice}={len(notes)}" for voice, notes in sorted(processed.items())
        )
    )
    print(f"Selected voice   : {args.voice} ({len(selected_output)} notes)")
    print(f"MIDI             : {output_midi}")
    print(f"JSON             : {output_json}")
    print(f"C header         : {output_c}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
