#!/usr/bin/env python3
"""Extract a monophonic melody from a dense piano MIDI file.

Dependency:
    python -m pip install pretty_midi

Example:
    python tools/melody_extractor.py tools/song.mid

The default output files are written next to the input file:
    song_melody.mid, song_melody.json, song_melody.h
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

try:
    import pretty_midi
except ImportError as exc:  # pragma: no cover - depends on the local environment
    raise SystemExit(
        "pretty_midi is required. Install it with: "
        "python -m pip install pretty_midi"
    ) from exc


@dataclass(frozen=True)
class SourceNote:
    """A note read from the source MIDI, with times expressed in seconds."""

    note_id: int
    pitch: int
    start: float
    end: float
    duration: float
    velocity: int


@dataclass
class MelodyNote:
    """A selected monophonic note, with times expressed in seconds."""

    pitch: int
    start: float
    end: float
    velocity: int

    @property
    def duration(self) -> float:
        return self.end - self.start


@dataclass(frozen=True)
class ScoringWeights:
    pitch: float = 4.0
    duration: float = 2.0
    velocity: float = 1.0
    interval: float = 3.0
    overlap: float = 2.0


@dataclass(frozen=True)
class ExtractionConfig:
    window_ms: float = 20.0
    min_pitch: int = 50
    max_pitch: int = 108
    same_onset_ms: float = 50.0
    melody_tolerance_semitones: int = 10
    chord_tolerance_semitones: int = 9
    average_degree_length: float = 8.0
    melody_degree_floor: int | None = 71
    classification_min_note_ms: float = 120.0
    min_score: float = 0.0
    register_band_semitones: int = 19
    pitch_norm_high: int = 96
    duration_norm_ms: float = 800.0
    chord_tolerance_ms: float = 35.0
    continuity_reset_ms: float = 600.0
    min_note_ms: float = 40.0
    merge_gap_ms: float = 60.0
    leap_threshold: int = 10
    return_tolerance: int = 10
    spike_max_ms: float = 600.0
    smooth_passes: int = 6
    octave_fold: bool = True
    octave_fold_max_gap_ms: float = 350.0
    octave_fold_min_improvement: int = 6
    viterbi_step_ms: float = 125.0
    viterbi_min_note_ms: float = 100.0
    viterbi_rest_score: float = 1.2
    viterbi_pitch_weight: float = 0.8
    viterbi_duration_weight: float = 1.0
    viterbi_velocity_weight: float = 2.0
    viterbi_top_weight: float = 0.4
    viterbi_onset_weight: float = 0.8
    viterbi_change_penalty: float = 0.2
    viterbi_rest_transition: float = 0.2
    viterbi_held_switch_penalty: float = 2.0
    viterbi_interval_linear: float = 0.12
    viterbi_large_interval: float = 0.08
    viterbi_large_interval_threshold: int = 7
    viterbi_max_candidates: int = 10

    def validate(self) -> None:
        if self.window_ms <= 0:
            raise ValueError("window_ms must be greater than zero")
        if not 0 <= self.min_pitch <= self.max_pitch <= 127:
            raise ValueError("pitch range must satisfy 0 <= min_pitch <= max_pitch <= 127")
        if self.same_onset_ms < 0:
            raise ValueError("same_onset_ms cannot be negative")
        if self.melody_tolerance_semitones < 0:
            raise ValueError("melody_tolerance_semitones cannot be negative")
        if self.chord_tolerance_semitones <= 0:
            raise ValueError("chord_tolerance_semitones must be greater than zero")
        if self.average_degree_length <= 0:
            raise ValueError("average_degree_length must be greater than zero")
        if self.classification_min_note_ms < 0:
            raise ValueError("classification_min_note_ms cannot be negative")
        if self.melody_degree_floor is not None and not 0 <= self.melody_degree_floor <= 127:
            raise ValueError("melody_degree_floor must be a MIDI pitch or None")
        if self.pitch_norm_high <= self.min_pitch:
            raise ValueError("pitch_norm_high must be greater than min_pitch")
        if self.register_band_semitones <= 0:
            raise ValueError("register_band_semitones must be greater than zero")
        if self.duration_norm_ms <= 0:
            raise ValueError("duration_norm_ms must be greater than zero")
        if self.min_note_ms < 0 or self.merge_gap_ms < 0:
            raise ValueError("post-processing times cannot be negative")
        if self.smooth_passes < 0:
            raise ValueError("smooth_passes cannot be negative")
        if self.octave_fold_max_gap_ms < 0:
            raise ValueError("octave_fold_max_gap_ms cannot be negative")
        if self.octave_fold_min_improvement < 0:
            raise ValueError("octave_fold_min_improvement cannot be negative")
        if self.viterbi_step_ms <= 0 or self.viterbi_min_note_ms < 0:
            raise ValueError("invalid Viterbi timing parameters")
        if self.viterbi_max_candidates <= 0:
            raise ValueError("viterbi_max_candidates must be greater than zero")


@dataclass(frozen=True)
class FrameChoice:
    note_id: int
    pitch: int
    frame_start: float
    frame_end: float
    velocity: int


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def load_midi_notes(path: Path) -> tuple[pretty_midi.PrettyMIDI, list[SourceNote]]:
    """Load all non-drum notes from every pretty_midi instrument."""

    midi = pretty_midi.PrettyMIDI(str(path))
    raw_notes = [
        note
        for instrument in midi.instruments
        if not instrument.is_drum
        for note in instrument.notes
        if note.end > note.start
    ]
    raw_notes.sort(key=lambda note: (note.start, note.pitch, note.end, note.velocity))

    notes = [
        SourceNote(
            note_id=index,
            pitch=int(note.pitch),
            start=float(note.start),
            end=float(note.end),
            duration=float(note.end - note.start),
            velocity=int(note.velocity),
        )
        for index, note in enumerate(raw_notes)
    ]
    return midi, notes


def interval_cost(pitch: int, previous_pitch: int | None) -> float:
    """Return a normalized transition cost with an extra penalty above one octave."""

    if previous_pitch is None:
        return 0.0
    semitones = abs(pitch - previous_pitch)
    cost = min(semitones, 12) / 12.0
    if semitones > 12:
        cost += 1.5 * (semitones - 12) / 12.0
    return min(cost, 3.0)


def overlap_cost(
    note: SourceNote,
    candidates: Sequence[SourceNote],
    frame_start: float,
    previous_pitch: int | None,
    config: ExtractionConfig,
) -> float:
    """Penalize inner chord tones and switching into an already-held chord tone."""

    chord_tolerance = config.chord_tolerance_ms / 1000.0
    same_onset = [
        other
        for other in candidates
        if abs(other.start - note.start) <= chord_tolerance
    ]

    inner_chord_cost = 0.0
    if len(same_onset) > 1:
        chord_top = max(other.pitch for other in same_onset)
        chord_bottom = min(other.pitch for other in same_onset)
        chord_span = max(12, chord_top - chord_bottom)
        inner_chord_cost = clamp((chord_top - note.pitch) / chord_span)

    held_switch_cost = 0.0
    is_held = note.start < frame_start - config.window_ms / 1000.0
    if previous_pitch is not None and note.pitch != previous_pitch and is_held:
        held_switch_cost = 0.5

    return clamp(inner_chord_cost + held_switch_cost)


def score_candidate(
    note: SourceNote,
    candidates: Sequence[SourceNote],
    frame_start: float,
    previous_pitch: int | None,
    config: ExtractionConfig,
    weights: ScoringWeights,
) -> float:
    pitch_norm = clamp(
        (note.pitch - config.min_pitch)
        / (config.pitch_norm_high - config.min_pitch)
    )
    duration_norm = clamp(note.duration * 1000.0 / config.duration_norm_ms)
    velocity_norm = note.velocity / 127.0

    return (
        weights.pitch * pitch_norm
        + weights.duration * duration_norm
        + weights.velocity * velocity_norm
        - weights.interval * interval_cost(note.pitch, previous_pitch)
        - weights.overlap
        * overlap_cost(note, candidates, frame_start, previous_pitch, config)
    )


def one_note_per_pitch(notes: Iterable[SourceNote]) -> list[SourceNote]:
    """Collapse overlapping duplicate pitches while keeping the newest articulation."""

    by_pitch: dict[int, SourceNote] = {}
    for note in notes:
        current = by_pitch.get(note.pitch)
        if current is None or (note.start, note.velocity, note.end) > (
            current.start,
            current.velocity,
            current.end,
        ):
            by_pitch[note.pitch] = note
    return list(by_pitch.values())


def keep_local_melody_register(
    candidates: Sequence[SourceNote], band_semitones: int
) -> list[SourceNote]:
    """Keep the local upper voice without following a single extreme ornament."""

    if len(candidates) < 3:
        return list(candidates)
    pitches = sorted({note.pitch for note in candidates})
    anchor_pitch = pitches[-2] if len(pitches) >= 3 else pitches[-1]
    register_floor = anchor_pitch - band_semitones
    return [note for note in candidates if note.pitch >= register_floor]


def collapse_same_onset_groups(
    notes: Sequence[SourceNote], tolerance_ms: float
) -> list[SourceNote]:
    """Keep only the highest note from each simultaneous-onset group."""

    if not notes:
        return []

    tolerance = tolerance_ms / 1000.0
    collapsed: list[SourceNote] = []
    index = 0
    while index < len(notes):
        group_start = notes[index].start
        group = [notes[index]]
        next_index = index + 1
        while (
            next_index < len(notes)
            and notes[next_index].start - group_start <= tolerance + 1e-9
        ):
            group.append(notes[next_index])
            next_index += 1
        collapsed.append(
            max(group, key=lambda note: (note.pitch, note.duration, note.velocity))
        )
        index = next_index
    return collapsed


def recent_melody_indices(
    melody_intervals: Sequence[float], length_in_whole_notes: float
) -> list[int]:
    """Mirror musicpy.add_to_index(..., start=last, stop=-1, step=-1)."""

    if not melody_intervals:
        return []
    result: list[int] = []
    accumulated = 0.0
    for index in range(len(melody_intervals) - 1, -1, -1):
        accumulated += melody_intervals[index]
        result.append(index)
        if accumulated >= length_in_whole_notes:
            break
    return result


def split_musicpy_style(
    notes: Sequence[SourceNote],
    midi: pretty_midi.PrettyMIDI,
    config: ExtractionConfig,
) -> list[SourceNote]:
    """Adapt musicpy.split_melody to pretty_midi's absolute-time notes."""

    candidates = [
        note
        for note in notes
        if config.min_pitch <= note.pitch <= config.max_pitch
        and note.duration * 1000.0 >= config.classification_min_note_ms
    ]
    candidates = collapse_same_onset_groups(candidates, config.same_onset_ms)
    if len(candidates) < 2:
        return candidates

    whole_note_ticks = midi.resolution * 4.0
    positions = [midi.time_to_tick(note.start) / whole_note_ticks for note in candidates]
    intervals = [
        max(0.0, positions[index + 1] - positions[index])
        for index in range(len(candidates) - 1)
    ] + [0.0]

    chord_tol = config.chord_tolerance_semitones
    melody_tol = config.melody_tolerance_semitones
    start = 1 if candidates[1].pitch - candidates[0].pitch >= chord_tol else 0
    melody = [candidates[start]]
    melody_intervals = [intervals[start]]

    for index in range(start + 1, len(candidates) - 1):
        current = candidates[index]
        following = candidates[index + 1]
        next_degree_difference = following.pitch - current.pitch
        recent_indices = recent_melody_indices(
            melody_intervals, config.average_degree_length
        )
        if not recent_indices:
            continue

        average_pitch = sum(melody[i].pitch for i in recent_indices) / len(
            recent_indices
        )
        average_difference = average_pitch - current.pitch
        above_degree_floor = (
            config.melody_degree_floor is None
            or current.pitch >= config.melody_degree_floor
        )
        recent_continuity = all(
            note.pitch - current.pitch < chord_tol for note in melody[-2:]
        )
        accept = False

        if average_difference <= melody_tol:
            if melody[-1].pitch - current.pitch < chord_tol:
                accept = True
            elif abs(next_degree_difference) < chord_tol and above_degree_floor:
                accept = True
        elif (
            melody[-1].pitch - current.pitch < chord_tol
            and next_degree_difference < chord_tol
            and recent_continuity
        ):
            accept = True
        elif (
            abs(next_degree_difference) < chord_tol
            and above_degree_floor
            and recent_continuity
        ):
            accept = True

        if accept:
            melody.append(current)
            melody_intervals.append(intervals[index])

    return melody


def select_melody_frames(
    notes: Sequence[SourceNote],
    config: ExtractionConfig,
    weights: ScoringWeights,
) -> list[FrameChoice]:
    """Choose the best active note in each fixed-size time window."""

    if not notes:
        return []

    window = config.window_ms / 1000.0
    reset_after = config.continuity_reset_ms / 1000.0
    frame_start = math.floor(notes[0].start / window) * window
    final_time = max(note.end for note in notes)
    active: dict[int, SourceNote] = {}
    start_index = 0
    previous_pitch: int | None = None
    previous_choice_time: float | None = None
    choices: list[FrameChoice] = []

    while frame_start < final_time:
        frame_limit = frame_start + window
        while start_index < len(notes) and notes[start_index].start < frame_limit - 1e-9:
            note = notes[start_index]
            active[note.note_id] = note
            start_index += 1

        expired = [note_id for note_id, note in active.items() if note.end <= frame_start]
        for note_id in expired:
            del active[note_id]

        if previous_choice_time is not None and frame_start - previous_choice_time > reset_after:
            previous_pitch = None

        candidates = one_note_per_pitch(
            note
            for note in active.values()
            if config.min_pitch <= note.pitch <= config.max_pitch
        )
        candidates = keep_local_melody_register(
            candidates, config.register_band_semitones
        )

        if candidates:
            scored_candidates = [
                (
                    score_candidate(
                        note,
                        candidates,
                        frame_start,
                        previous_pitch,
                        config,
                        weights,
                    ),
                    note,
                )
                for note in candidates
            ]
            best_score, best = max(
                scored_candidates,
                key=lambda item: (
                    item[0],
                    item[1].duration,
                    item[1].velocity,
                    item[1].pitch,
                ),
            )
            selected_start = max(frame_start, best.start)
            selected_end = min(frame_limit, best.end)
            if best_score >= config.min_score and selected_end > selected_start:
                choices.append(
                    FrameChoice(
                        note_id=best.note_id,
                        pitch=best.pitch,
                        frame_start=selected_start,
                        frame_end=selected_end,
                        velocity=best.velocity,
                    )
                )
                previous_pitch = best.pitch
                previous_choice_time = frame_start

        frame_start += window

    return choices


def viterbi_emission_score(
    note: SourceNote,
    candidates: Sequence[SourceNote],
    frame_start: float,
    config: ExtractionConfig,
) -> float:
    pitch_norm = clamp(
        (note.pitch - config.min_pitch)
        / (config.pitch_norm_high - config.min_pitch)
    )
    duration_norm = clamp(note.duration * 1000.0 / config.duration_norm_ms)
    velocity_norm = note.velocity / 127.0
    candidate_pitches = [candidate.pitch for candidate in candidates]
    pitch_span = max(1, max(candidate_pitches) - min(candidate_pitches))
    top_norm = (note.pitch - min(candidate_pitches)) / pitch_span
    is_new_onset = note.start >= frame_start - 1e-9
    return (
        config.viterbi_pitch_weight * pitch_norm
        + config.viterbi_duration_weight * duration_norm
        + config.viterbi_velocity_weight * velocity_norm
        + config.viterbi_top_weight * top_norm
        + config.viterbi_onset_weight * float(is_new_onset)
    )


def viterbi_transition_cost(
    previous: SourceNote | None,
    current: SourceNote | None,
    frame_start: float,
    config: ExtractionConfig,
) -> float:
    if previous is None and current is None:
        return 0.0
    if previous is None or current is None:
        return config.viterbi_rest_transition
    if previous.note_id == current.note_id:
        return 0.0

    interval = abs(current.pitch - previous.pitch)
    large_part = max(0, interval - config.viterbi_large_interval_threshold)
    cost = (
        config.viterbi_change_penalty
        + config.viterbi_interval_linear * interval
        + config.viterbi_large_interval * large_part * large_part
    )
    if current.start < frame_start - 1e-9:
        cost += config.viterbi_held_switch_penalty
    return cost


def select_melody_viterbi_frames(
    notes: Sequence[SourceNote], config: ExtractionConfig
) -> list[FrameChoice]:
    """Find a globally smooth melody path over active-note and rest states."""

    if not notes:
        return []

    step = config.viterbi_step_ms / 1000.0
    minimum_duration = config.viterbi_min_note_ms / 1000.0
    first_frame = math.floor(notes[0].start / step) * step
    final_time = max(note.end for note in notes)
    active: dict[int, SourceNote] = {}
    note_by_id = {note.note_id: note for note in notes}
    start_index = 0
    frames: list[tuple[float, list[SourceNote]]] = []
    frame_start = first_frame

    while frame_start < final_time:
        frame_limit = frame_start + step
        while start_index < len(notes) and notes[start_index].start < frame_limit - 1e-9:
            note = notes[start_index]
            active[note.note_id] = note
            start_index += 1
        active = {
            note_id: note
            for note_id, note in active.items()
            if note.end > frame_start
        }
        candidates = one_note_per_pitch(
            note
            for note in active.values()
            if config.min_pitch <= note.pitch <= config.max_pitch
            and note.duration >= minimum_duration
        )
        candidates = keep_local_melody_register(
            candidates, config.register_band_semitones
        )
        candidate_context = tuple(candidates)
        candidates.sort(
            key=lambda note: viterbi_emission_score(
                note, candidate_context, frame_start, config
            ),
            reverse=True,
        )
        frames.append((frame_start, candidates[: config.viterbi_max_candidates]))
        frame_start += step

    if not frames:
        return []

    previous_scores: dict[int, float] = {-1: config.viterbi_rest_score}
    back_pointers: list[dict[int, int]] = [{-1: -1}]
    first_start, first_candidates = frames[0]
    for note in first_candidates:
        previous_scores[note.note_id] = viterbi_emission_score(
            note, first_candidates, first_start, config
        )
        back_pointers[0][note.note_id] = -1

    for frame_start, candidates in frames[1:]:
        state_ids = [-1] + [note.note_id for note in candidates]
        current_scores: dict[int, float] = {}
        current_back: dict[int, int] = {}
        for state_id in state_ids:
            current_note = note_by_id.get(state_id)
            emission = (
                config.viterbi_rest_score
                if current_note is None
                else viterbi_emission_score(
                    current_note, candidates, frame_start, config
                )
            )
            best_previous, best_score = max(
                (
                    (
                        previous_id,
                        previous_score
                        - viterbi_transition_cost(
                            note_by_id.get(previous_id),
                            current_note,
                            frame_start,
                            config,
                        ),
                    )
                    for previous_id, previous_score in previous_scores.items()
                ),
                key=lambda item: item[1],
            )
            current_scores[state_id] = best_score + emission
            current_back[state_id] = best_previous
        previous_scores = current_scores
        back_pointers.append(current_back)

    state_id = max(previous_scores, key=previous_scores.get)
    selected_states = [state_id]
    for frame_index in range(len(frames) - 1, 0, -1):
        state_id = back_pointers[frame_index][state_id]
        selected_states.append(state_id)
    selected_states.reverse()

    choices: list[FrameChoice] = []
    for (frame_start, _), state_id in zip(frames, selected_states):
        note = note_by_id.get(state_id)
        if note is None:
            continue
        selected_start = max(frame_start, note.start)
        selected_end = min(frame_start + step, note.end)
        if selected_end > selected_start:
            choices.append(
                FrameChoice(
                    note_id=note.note_id,
                    pitch=note.pitch,
                    frame_start=selected_start,
                    frame_end=selected_end,
                    velocity=note.velocity,
                )
            )
    return choices


def collapse_frames(frames: Sequence[FrameChoice], window_ms: float) -> list[MelodyNote]:
    """Convert frame decisions into note segments and merge adjacent equal pitches."""

    if not frames:
        return []

    tolerance = window_ms / 1000.0 + 1e-9
    result: list[MelodyNote] = []
    for frame in frames:
        if (
            result
            and result[-1].pitch == frame.pitch
            and frame.frame_start <= result[-1].end + tolerance
        ):
            result[-1].end = max(result[-1].end, frame.frame_end)
            result[-1].velocity = max(result[-1].velocity, frame.velocity)
        else:
            result.append(
                MelodyNote(
                    pitch=frame.pitch,
                    start=frame.frame_start,
                    end=frame.frame_end,
                    velocity=frame.velocity,
                )
            )
    return result


def merge_repeated_notes(
    notes: Sequence[MelodyNote], merge_gap_ms: float
) -> list[MelodyNote]:
    """Merge consecutive equal pitches separated only by a small extraction gap."""

    if not notes:
        return []

    max_gap = merge_gap_ms / 1000.0
    merged = [MelodyNote(**asdict(notes[0]))]
    for note in notes[1:]:
        previous = merged[-1]
        gap = note.start - previous.end
        if note.pitch == previous.pitch and gap <= max_gap:
            previous.end = max(previous.end, note.end)
            previous.velocity = max(previous.velocity, note.velocity)
        else:
            merged.append(MelodyNote(**asdict(note)))
    return merged


def remove_short_notes(
    notes: Sequence[MelodyNote], min_note_ms: float
) -> list[MelodyNote]:
    minimum = min_note_ms / 1000.0
    return [MelodyNote(**asdict(note)) for note in notes if note.duration >= minimum]


def smooth_isolated_leaps(
    notes: Sequence[MelodyNote], config: ExtractionConfig
) -> list[MelodyNote]:
    """Replace a short out-and-back pitch spike with its preceding melody pitch."""

    smoothed = [MelodyNote(**asdict(note)) for note in notes]
    max_spike = config.spike_max_ms / 1000.0

    for _ in range(config.smooth_passes):
        changes = 0
        for index in range(1, len(smoothed) - 1):
            previous = smoothed[index - 1]
            current = smoothed[index]
            following = smoothed[index + 1]
            jumps_out = abs(current.pitch - previous.pitch) >= config.leap_threshold
            jumps_back = abs(current.pitch - following.pitch) >= config.leap_threshold
            neighbors_agree = (
                abs(previous.pitch - following.pitch) <= config.return_tolerance
            )
            if current.duration <= max_spike and jumps_out and jumps_back and neighbors_agree:
                current.pitch = previous.pitch
                current.velocity = max(previous.velocity, current.velocity)
                changes += 1
        if changes == 0:
            break
        smoothed = merge_repeated_notes(smoothed, config.merge_gap_ms)

    return smoothed


def fold_abrupt_octaves(
    notes: Sequence[MelodyNote], config: ExtractionConfig
) -> list[MelodyNote]:
    """Move abrupt nearby notes by octaves while preserving their pitch classes."""

    folded = [MelodyNote(**asdict(note)) for note in notes]
    if not config.octave_fold:
        return folded

    max_gap = config.octave_fold_max_gap_ms / 1000.0
    for previous, current in zip(folded, folded[1:]):
        gap = current.start - previous.end
        original_interval = abs(current.pitch - previous.pitch)
        if gap > max_gap or original_interval < 12:
            continue

        variants = [
            pitch
            for pitch in range(config.min_pitch, config.max_pitch + 1)
            if (pitch - current.pitch) % 12 == 0
        ]
        best_pitch = min(
            variants,
            key=lambda pitch: (
                abs(pitch - previous.pitch),
                abs(pitch - current.pitch),
            ),
        )
        best_interval = abs(best_pitch - previous.pitch)
        if original_interval - best_interval >= config.octave_fold_min_improvement:
            current.pitch = best_pitch

    return folded


def prevent_overlaps(notes: Sequence[MelodyNote]) -> list[MelodyNote]:
    """Trim rounding-level overlaps so every result is strictly monophonic."""

    result = [MelodyNote(**asdict(note)) for note in notes]
    for current, following in zip(result, result[1:]):
        current.end = min(current.end, following.start)
    return [note for note in result if note.end > note.start]


def post_process(
    frames: Sequence[FrameChoice], config: ExtractionConfig
) -> list[MelodyNote]:
    return post_process_notes(collapse_frames(frames, config.window_ms), config)


def post_process_notes(
    notes: Sequence[MelodyNote], config: ExtractionConfig
) -> list[MelodyNote]:
    notes = prevent_overlaps(notes)
    notes = merge_repeated_notes(notes, config.merge_gap_ms)
    notes = remove_short_notes(notes, config.min_note_ms)
    notes = fold_abrupt_octaves(notes, config)
    notes = smooth_isolated_leaps(notes, config)
    notes = merge_repeated_notes(notes, config.merge_gap_ms)
    notes = prevent_overlaps(notes)
    return remove_short_notes(notes, config.min_note_ms)


def pitch_to_frequency(pitch: int) -> float:
    return 440.0 * 2.0 ** ((pitch - 69) / 12.0)


def build_output_notes(notes: Sequence[MelodyNote]) -> list[dict[str, int | float]]:
    output: list[dict[str, int | float]] = []
    rounded_times = [
        (
            round(note.start * 1000.0),
            max(1, round(note.duration * 1000.0)),
        )
        for note in notes
    ]

    for index, note in enumerate(notes):
        start_ms, duration_ms = rounded_times[index]
        if index + 1 < len(notes):
            next_start_ms = rounded_times[index + 1][0]
            gap_ms = max(0, next_start_ms - (start_ms + duration_ms))
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


def sanitize_c_identifier(name: str) -> str:
    identifier = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not identifier:
        return "music"
    if identifier[0].isdigit():
        identifier = "_" + identifier
    return identifier


def format_c_array(output_notes: Sequence[dict[str, int | float]], array_name: str) -> str:
    array_name = sanitize_c_identifier(array_name)
    lines = [
        "#include <stdint.h>",
        "",
        "typedef struct",
        "{",
        "    uint16_t freq;",
        "    uint16_t duration;",
        "    uint16_t gap;",
        "} Note;",
        "",
        f"static const Note {array_name}[] =",
        "{",
    ]

    for note in output_notes:
        frequency = round(float(note["frequency"]))
        duration = int(note["duration_ms"])
        gap = int(note["gap_ms"])
        if max(frequency, duration, gap) > 65535:
            raise ValueError("a generated value does not fit in uint16_t")
        lines.append(f"    {{{frequency}, {duration}, {gap}}},")

    lines.extend(
        [
            "};",
            "",
            f"static const uint32_t {array_name}_count =",
            f"    sizeof({array_name}) / sizeof({array_name}[0]);",
            "",
        ]
    )
    return "\n".join(lines)


def write_json(
    path: Path,
    source: Path,
    notes: Sequence[dict[str, int | float]],
    config: ExtractionConfig,
    weights: ScoringWeights,
    algorithm: str,
) -> None:
    document = {
        "source": source.name,
        "algorithm": algorithm,
        "note_count": len(notes),
        "config": asdict(config),
        "weights": asdict(weights),
        "notes": notes,
    }
    path.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_melody_midi(
    path: Path,
    notes: Sequence[MelodyNote],
    source_midi: pretty_midi.PrettyMIDI,
) -> None:
    _, tempo_values = source_midi.get_tempo_changes()
    initial_tempo = float(tempo_values[0]) if len(tempo_values) else 120.0
    output = pretty_midi.PrettyMIDI(
        resolution=source_midi.resolution,
        initial_tempo=initial_tempo,
    )
    instrument = pretty_midi.Instrument(program=0, name="Extracted Melody")
    instrument.notes = [
        pretty_midi.Note(
            velocity=max(1, min(127, note.velocity)),
            pitch=note.pitch,
            start=note.start,
            end=note.end,
        )
        for note in notes
    ]
    output.instruments.append(instrument)
    output.write(str(path))


def default_output_path(input_path: Path, suffix: str) -> Path:
    return input_path.with_name(f"{input_path.stem}_melody{suffix}")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Extract a scored monophonic melody for an STM32 passive buzzer."
    )
    parser.add_argument("input", type=Path, help="source MIDI file")
    parser.add_argument("--output-midi", type=Path, help="extracted melody MIDI path")
    parser.add_argument("--output-json", type=Path, help="note timing JSON path")
    parser.add_argument("--output-c", type=Path, help="STM32 C header path")
    parser.add_argument("--array-name", default="music", help="C array identifier")
    parser.add_argument(
        "--algorithm",
        choices=("viterbi", "musicpy", "window-score"),
        default="viterbi",
        help="melody classifier (default: viterbi)",
    )

    parser.add_argument("--window-ms", type=float, default=20.0)
    parser.add_argument("--min-pitch", type=int, default=50)
    parser.add_argument("--max-pitch", type=int, default=108)
    parser.add_argument("--same-onset-ms", type=float, default=50.0)
    parser.add_argument("--melody-tolerance-semitones", type=int, default=10)
    parser.add_argument("--chord-tolerance-semitones", type=int, default=9)
    parser.add_argument("--average-degree-length", type=float, default=8.0)
    parser.add_argument("--classification-min-note-ms", type=float, default=120.0)
    parser.add_argument(
        "--melody-degree-floor",
        type=int,
        default=71,
        help="musicpy-style floor pitch; use -1 to disable",
    )
    parser.add_argument("--min-score", type=float, default=0.0)
    parser.add_argument("--register-band-semitones", type=int, default=19)
    parser.add_argument("--pitch-norm-high", type=int, default=96)
    parser.add_argument("--duration-norm-ms", type=float, default=800.0)
    parser.add_argument("--chord-tolerance-ms", type=float, default=35.0)
    parser.add_argument("--continuity-reset-ms", type=float, default=600.0)
    parser.add_argument("--min-note-ms", type=float, default=40.0)
    parser.add_argument("--merge-gap-ms", type=float, default=60.0)
    parser.add_argument("--leap-threshold", type=int, default=10)
    parser.add_argument("--return-tolerance", type=int, default=10)
    parser.add_argument("--spike-max-ms", type=float, default=600.0)
    parser.add_argument("--smooth-passes", type=int, default=6)
    parser.add_argument("--no-octave-fold", action="store_true")
    parser.add_argument("--octave-fold-max-gap-ms", type=float, default=350.0)
    parser.add_argument("--octave-fold-min-improvement", type=int, default=6)
    parser.add_argument("--viterbi-step-ms", type=float, default=125.0)
    parser.add_argument("--viterbi-min-note-ms", type=float, default=100.0)
    parser.add_argument("--viterbi-rest-score", type=float, default=1.2)
    parser.add_argument("--viterbi-pitch-weight", type=float, default=0.8)
    parser.add_argument("--viterbi-duration-weight", type=float, default=1.0)
    parser.add_argument("--viterbi-velocity-weight", type=float, default=2.0)
    parser.add_argument("--viterbi-top-weight", type=float, default=0.4)
    parser.add_argument("--viterbi-onset-weight", type=float, default=0.8)
    parser.add_argument("--viterbi-change-penalty", type=float, default=0.2)
    parser.add_argument("--viterbi-rest-transition", type=float, default=0.2)
    parser.add_argument("--viterbi-held-switch-penalty", type=float, default=2.0)
    parser.add_argument("--viterbi-interval-linear", type=float, default=0.12)
    parser.add_argument("--viterbi-large-interval", type=float, default=0.08)
    parser.add_argument("--viterbi-large-interval-threshold", type=int, default=7)
    parser.add_argument("--viterbi-max-candidates", type=int, default=10)

    parser.add_argument("--pitch-weight", type=float, default=4.0)
    parser.add_argument("--duration-weight", type=float, default=2.0)
    parser.add_argument("--velocity-weight", type=float, default=1.0)
    parser.add_argument("--interval-weight", type=float, default=3.0)
    parser.add_argument("--overlap-weight", type=float, default=2.0)
    return parser


def config_from_args(args: argparse.Namespace) -> ExtractionConfig:
    return ExtractionConfig(
        window_ms=args.window_ms,
        min_pitch=args.min_pitch,
        max_pitch=args.max_pitch,
        same_onset_ms=args.same_onset_ms,
        melody_tolerance_semitones=args.melody_tolerance_semitones,
        chord_tolerance_semitones=args.chord_tolerance_semitones,
        average_degree_length=args.average_degree_length,
        classification_min_note_ms=args.classification_min_note_ms,
        melody_degree_floor=(
            None if args.melody_degree_floor < 0 else args.melody_degree_floor
        ),
        min_score=args.min_score,
        register_band_semitones=args.register_band_semitones,
        pitch_norm_high=args.pitch_norm_high,
        duration_norm_ms=args.duration_norm_ms,
        chord_tolerance_ms=args.chord_tolerance_ms,
        continuity_reset_ms=args.continuity_reset_ms,
        min_note_ms=args.min_note_ms,
        merge_gap_ms=args.merge_gap_ms,
        leap_threshold=args.leap_threshold,
        return_tolerance=args.return_tolerance,
        spike_max_ms=args.spike_max_ms,
        smooth_passes=args.smooth_passes,
        octave_fold=not args.no_octave_fold,
        octave_fold_max_gap_ms=args.octave_fold_max_gap_ms,
        octave_fold_min_improvement=args.octave_fold_min_improvement,
        viterbi_step_ms=args.viterbi_step_ms,
        viterbi_min_note_ms=args.viterbi_min_note_ms,
        viterbi_rest_score=args.viterbi_rest_score,
        viterbi_pitch_weight=args.viterbi_pitch_weight,
        viterbi_duration_weight=args.viterbi_duration_weight,
        viterbi_velocity_weight=args.viterbi_velocity_weight,
        viterbi_top_weight=args.viterbi_top_weight,
        viterbi_onset_weight=args.viterbi_onset_weight,
        viterbi_change_penalty=args.viterbi_change_penalty,
        viterbi_rest_transition=args.viterbi_rest_transition,
        viterbi_held_switch_penalty=args.viterbi_held_switch_penalty,
        viterbi_interval_linear=args.viterbi_interval_linear,
        viterbi_large_interval=args.viterbi_large_interval,
        viterbi_large_interval_threshold=args.viterbi_large_interval_threshold,
        viterbi_max_candidates=args.viterbi_max_candidates,
    )


def weights_from_args(args: argparse.Namespace) -> ScoringWeights:
    return ScoringWeights(
        pitch=args.pitch_weight,
        duration=args.duration_weight,
        velocity=args.velocity_weight,
        interval=args.interval_weight,
        overlap=args.overlap_weight,
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    input_path = args.input.resolve()
    if not input_path.is_file():
        raise SystemExit(f"input MIDI does not exist: {input_path}")

    config = config_from_args(args)
    weights = weights_from_args(args)
    config.validate()

    output_midi = (args.output_midi or default_output_path(input_path, ".mid")).resolve()
    output_json = (args.output_json or default_output_path(input_path, ".json")).resolve()
    output_c = (args.output_c or default_output_path(input_path, ".h")).resolve()
    for path in (output_midi, output_json, output_c):
        path.parent.mkdir(parents=True, exist_ok=True)

    source_midi, source_notes = load_midi_notes(input_path)
    if args.algorithm == "musicpy":
        selected_notes = split_musicpy_style(source_notes, source_midi, config)
        melody = post_process_notes(
            [
                MelodyNote(
                    pitch=note.pitch,
                    start=note.start,
                    end=note.end,
                    velocity=note.velocity,
                )
                for note in selected_notes
            ],
            config,
        )
    elif args.algorithm == "window-score":
        frames = select_melody_frames(source_notes, config, weights)
        melody = post_process(frames, config)
    else:
        frames = select_melody_viterbi_frames(source_notes, config)
        melody = post_process(frames, config)
    if not melody:
        raise SystemExit(
            "no melody notes remained; lower --min-pitch or --min-note-ms"
        )
    output_notes = build_output_notes(melody)

    write_melody_midi(output_midi, melody, source_midi)
    write_json(
        output_json,
        input_path,
        output_notes,
        config,
        weights,
        args.algorithm,
    )
    output_c.write_text(
        format_c_array(output_notes, args.array_name), encoding="utf-8"
    )

    print(f"Source notes : {len(source_notes)}")
    print(f"Algorithm    : {args.algorithm}")
    print(f"Melody notes : {len(melody)}")
    print(f"MIDI         : {output_midi}")
    print(f"JSON         : {output_json}")
    print(f"C header     : {output_c}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
