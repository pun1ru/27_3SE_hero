#!/usr/bin/env python3
"""
MIDI → STM32 蜂鸣器乐谱 转换工具 v2
======================================

两种模式:
  1) 扫描 tools/ 下所有 .mid 文件, 输出曲库.md (无参数运行)
  2) 单文件解析: python tools/midi_parser.py <曲子.mid> [选项]

原理:
  每个音符的时长由 BPM + tick_per_quarter + delta_tick 算出:
    us_per_tick      = tempo / tick_per_quarter
    dur_ms           = delta_tick × us_per_tick / 1000

  tempo 来自 MIDI Meta 事件 FF 51(微秒/四分音符),
  BPM = 60,000,000 / tempo.

MIDI音高 → mardio序号:
  mardio 序号 = MIDI 音高 + 2
  (from_notes_to_pr: note=62 → 261Hz = 中央C,
   MIDI标准: pitch=60 → 261Hz, 差 2)
"""

import struct
import sys
import os
import glob
from collections import namedtuple

MidiEvent = namedtuple('MidiEvent', ['abs_tick', 'type', 'channel', 'note', 'velocity'])
NoteInfo  = namedtuple('NoteInfo', ['pitch', 'channel', 'start_ms', 'dur_ms'])

# ============================================================
#  配置
# ============================================================
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
SONG_MD   = os.path.join(TOOLS_DIR, "曲库.md")

DEFAULT_CHANNEL = 0      # 默认主旋律通道
MIN_PITCH       = 24     # 最低音高
MAX_PITCH       = 96     # 最高音高


# ============================================================
#  MIDI 二进制解析
# ============================================================

def read_var_len(data, offset):
    """读取 MIDI 变长编码 (每字节最高位=1 表示还有后续)"""
    value = 0
    while True:
        b = data[offset]
        offset += 1
        value = (value << 7) | (b & 0x7F)
        if not (b & 0x80):
            break
    return value, offset


def filename_to_varname(filepath):
    """将文件名转为合法的 C 变量名"""
    name = os.path.splitext(os.path.basename(filepath))[0]
    # 只保留字母数字下划线, 首字符不能是数字
    clean = "".join(c if c.isalnum() or c == '_' else '_' for c in name)
    if clean and clean[0].isdigit():
        clean = '_' + clean
    return clean if clean else "unknown"


def filename_to_title(filepath):
    """将文件名转为显示标题 (去掉扩展名, 去掉首尾特殊字符)"""
    name = os.path.splitext(os.path.basename(filepath))[0]
    return name


def parse_midi(filepath):
    """
    解析 MIDI 文件。
    返回: {tick_per_qn, tempo, bpm, notes: [NoteInfo]}
    """
    with open(filepath, 'rb') as f:
        data = f.read()

    offset = 0

    if data[offset:offset+4] != b'MThd':
        return None
    offset += 4
    chunk_len = struct.unpack('>I', data[offset:offset+4])[0]
    offset += 4
    offset += 2  # format_type
    num_tracks = struct.unpack('>H', data[offset:offset+2])[0]
    offset += 2
    tick_per_qn = struct.unpack('>H', data[offset:offset+2])[0]
    offset += 2
    # 跳过文件头剩余数据 (标准MIDI头6字节已读完, 若更大则跳过)
    offset += (chunk_len - 6)

    raw_events = []
    tempo = 500000

    for _ in range(num_tracks):
        if data[offset:offset+4] != b'MTrk':
            break
        offset += 4
        track_len = struct.unpack('>I', data[offset:offset+4])[0]
        offset += 4
        track_end = offset + track_len

        abs_tick = 0
        running_status = 0

        while offset < track_end:
            delta, offset = read_var_len(data, offset)
            abs_tick += delta

            status = data[offset]
            offset += 1

            if status == 0xFF:
                meta_type = data[offset]; offset += 1
                meta_len, offset = read_var_len(data, offset)
                meta_data = data[offset:offset+meta_len]
                offset += meta_len
                if meta_type == 0x51:
                    tempo = struct.unpack('>I', b'\x00' + meta_data)[0]
                continue

            if status in (0xF0, 0xF7):
                sl, offset = read_var_len(data, offset)
                offset += sl
                continue

            if not (status & 0x80):
                note = status
                status = running_status
                vel = data[offset]; offset += 1
                ch = status & 0x0F
                et = status >> 4
                if et == 0x9 and vel > 0:
                    raw_events.append(MidiEvent(abs_tick, 'on', ch, note, vel))
                elif et == 0x8 or (et == 0x9 and vel == 0):
                    raw_events.append(MidiEvent(abs_tick, 'off', ch, note, vel))
                continue

            running_status = status
            ch = status & 0x0F
            et = status >> 4

            if et == 0x8:
                n = data[offset]; offset += 1
                v = data[offset]; offset += 1
                raw_events.append(MidiEvent(abs_tick, 'off', ch, n, v))
            elif et == 0x9:
                n = data[offset]; offset += 1
                v = data[offset]; offset += 1
                if v > 0:
                    raw_events.append(MidiEvent(abs_tick, 'on', ch, n, v))
                else:
                    raw_events.append(MidiEvent(abs_tick, 'off', ch, n, 0))
            elif et in (0xA, 0xB):
                offset += 2
            elif et == 0xC:
                offset += 1
            elif et == 0xD:
                offset += 1
            elif et == 0xE:
                offset += 2

    us_per_tick = tempo / tick_per_qn
    on_map = {}
    notes_raw = []

    for evt in sorted(raw_events,
                      key=lambda e: (e.abs_tick, 0 if e.type == 'on' else 1)):
        k = (evt.channel, evt.note)
        if evt.type == 'on':
            on_map[k] = evt.abs_tick
        else:
            if k in on_map:
                start = on_map.pop(k)
                dur_tick = evt.abs_tick - start
                notes_raw.append(NoteInfo(
                    pitch    = evt.note,
                    channel  = evt.channel,
                    start_ms = start * us_per_tick / 1000.0,
                    dur_ms   = dur_tick * us_per_tick / 1000.0,
                ))

    for k, start in on_map.items():
        notes_raw.append(NoteInfo(
            pitch    = k[1],
            channel  = k[0],
            start_ms = start * us_per_tick / 1000.0,
            dur_ms   = (tick_per_qn // 4) * us_per_tick / 1000.0,
        ))

    notes_raw.sort(key=lambda n: n.start_ms)

    return {
        'tick_per_qn': tick_per_qn,
        'tempo_us': tempo,
        'bpm': 60_000_000 / tempo,
        'notes': notes_raw,
    }


# ============================================================
#  提取策略
# ============================================================

def extract_monophonic(notes, time_tol_ms=8):
    """从多音中提取单旋律: 同时间窗口取最高音, 合并连续同音
从多音中提取单旋律: 同时间窗口取最高音, 合并连续同音"""
    if not notes:
        return []

    notes = sorted(notes, key=lambda n: n.start_ms)

    result = []
    i = 0
    while i < len(notes):
        cluster = [notes[i]]
        j = i + 1
        while (j < len(notes)
               and notes[j].start_ms - notes[i].start_ms < time_tol_ms):
            cluster.append(notes[j])
            j += 1
        result.append(max(cluster, key=lambda n: n.pitch))
        i = j

    merged = []
    for n in result:
        if (merged and merged[-1].pitch == n.pitch
                and abs(merged[-1].start_ms - n.start_ms)
                < merged[-1].dur_ms * 0.7):
            old = merged[-1]
            merged[-1] = NoteInfo(old.pitch, old.channel,
                                  old.start_ms,
                                  n.start_ms + n.dur_ms - old.start_ms)
        else:
            merged.append(n)

    return merged


def filter_melody(notes, min_dur_ms=120):
    """旋律过滤: 短于 min_dur_ms(≈十六分@120BPM)的音合并到前一音,
       保留真正的主旋律线条."""
    if not notes:
        return []
    result = [notes[0]]
    for n in notes[1:]:
        if n.dur_ms < min_dur_ms:
            prev = result[-1]
            result[-1] = NoteInfo(prev.pitch, prev.channel,
                                  prev.start_ms,
                                  max(prev.dur_ms,
                                      n.start_ms + n.dur_ms - prev.start_ms))
        else:
            result.append(n)
    return result


# ============================================================
#  输出格式
# ============================================================

def gap_to_seq(notes):
    """将 NoteInfo 转为线性播放序列 [(mardio_pitch, dur_ms)]

    蜂鸣器机械起振 ~30-40ms; BPM120 十六分=125ms, 三十二分=62ms.
    """
    MIN_DUR_MS  = 60   # 最短音符 (≈三十二分@120BPM)
    MIN_REST_MS = 40   # 最短休止

    seq = []
    for i, n in enumerate(notes):
        pitch_mardio = n.pitch + 2
        dur_midi = round(n.dur_ms)

        if i + 1 < len(notes):
            gap = notes[i + 1].start_ms - n.start_ms

            if gap > dur_midi and gap - dur_midi > MIN_REST_MS:
                # 有自然间隙: 音 + 休止
                out_dur = max(dur_midi, MIN_DUR_MS)
                seq.append((pitch_mardio, out_dur))
                seq.append((100, round(gap - dur_midi)))
            elif dur_midi < MIN_DUR_MS:
                # MIDI时值太短 → 跳过 (装饰音)
                pass
            elif gap > dur_midi:
                # gap略大于音长, 但不够插休止 → 用实际时值, 无休止
                seq.append((pitch_mardio, max(dur_midi, MIN_DUR_MS)))
            else:
                # 重叠(gap ≤ dur_midi): MIDI和弦音叠加, 用实际时值不截断
                seq.append((pitch_mardio, max(dur_midi, MIN_DUR_MS)))
        else:
            seq.append((pitch_mardio, max(dur_midi, MIN_DUR_MS)))

    return seq


def format_struct_array(seq, varname, bpm, tick_per_qn):
    """生成 MusicNote 结构体数组文本 (横向紧凑排列)"""
    lines = [
        f"// BPM: {bpm:.1f}  每四分音符={tick_per_qn} ticks",
        f"// 共 {len(seq)} 个音符/休止",
        f"static const MusicNote {varname}[] = {{",
    ]
    # 每行放 ENTRYS_PER_LINE 个
    ENTRYS_PER_LINE = 8
    for i in range(0, len(seq), ENTRYS_PER_LINE):
        chunk = seq[i:i + ENTRYS_PER_LINE]
        row = "    " + ", ".join(f"{{{note:3d},{dur:4d}}}" for note, dur in chunk) + ","
        lines.append(row)
    lines.append("};")
    return "\n".join(lines)


# ============================================================
#  MD 曲库生成
# ============================================================

SONG_MD_HEADER = """# 曲库

本文件由 `midi_parser.py` 自动生成, 扫描 `tools/` 目录下所有 `.mid` 文件。

## 数据结构

```c
typedef struct
{
    float    note;     // mardio 音高编号 (MIDI音高+2), 100=休止符
    uint16_t dur_ms;   // 持续时间(毫秒)
} MusicNote;
```

## 播放函数

```c
void play_music_notes(const MusicNote* score, int len, TIM_HandleTypeDef htim);
```

---

"""

SONG_ENTRY_TEMPLATE = """
## {title}

`{filename}`

- BPM: {bpm:.1f}
- 音符数: {count}

```c
{code}
```
"""


def generate_song_md(results):
    """生成完整曲库.md"""
    sections = [SONG_MD_HEADER]

    for r in results:
        section = SONG_ENTRY_TEMPLATE.format(
            title    = r['title'],
            filename = r['filename'],
            bpm      = r['bpm'],
            count    = r['count'],
            code     = r['code'],
        )
        sections.append(section)

    return "\n".join(sections)


def process_midi_file(filepath, target_ch=DEFAULT_CHANNEL,
                      min_p=MIN_PITCH, max_p=MAX_PITCH):
    """
    处理单个 MIDI 文件。
    返回: {title, filename, varname, bpm, tick_per_qn, count, code} 或 None
    """
    result = parse_midi(filepath)
    if result is None:
        return None

    notes = result['notes']

    # 筛选通道
    notes = [n for n in notes if n.channel == target_ch]

    # 筛选音高
    notes = [n for n in notes if min_p <= n.pitch <= max_p]

    # 单旋律提取
    notes = extract_monophonic(notes)

    if not notes:
        return None

    notes = filter_melody(notes)
    seq = gap_to_seq(notes)
    varname = filename_to_varname(filepath)
    code = format_struct_array(seq, varname,
                               result['bpm'], result['tick_per_qn'])

    return {
        'title':     filename_to_title(filepath),
        'filename':  os.path.basename(filepath),
        'varname':   varname,
        'bpm':       result['bpm'],
        'tick_per_qn': result['tick_per_qn'],
        'count':     len(seq),
        'code':      code,
    }


def batch_build():
    """扫描 tools/ 下所有 .mid, 生成曲库.md"""
    mid_files = glob.glob(os.path.join(TOOLS_DIR, "*.mid"))
    mid_files = sorted(set(mid_files))  # Windows 大小写不敏感去重

    if not mid_files:
        print("[!] tools/ 目录下没有找到 .mid 文件")
        return

    print(f"找到 {len(mid_files)} 个 MIDI 文件, 开始解析...\n")

    results = []
    for fpath in sorted(mid_files):
        fname = os.path.basename(fpath)
        print(f"  [{fname}] ", end="", flush=True)

        r = process_midi_file(fpath)
        if r is None:
            print("[x] 解析失败或无有效音符")
        else:
            print(f"OK  {r['count']} 个音符  BPM={r['bpm']:.1f}")
            results.append(r)

    if not results:
        print("\n没有成功解析的 MIDI 文件")
        return

    md_content = generate_song_md(results)

    with open(SONG_MD, 'w', encoding='utf-8') as f:
        f.write(md_content)

    print(f"\n[OK] 曲库已生成: {SONG_MD}")
    print(f"   共 {len(results)} 首曲子")


def single_mode(filepath, fmt, target_ch, mono, min_p, max_p):
    """单文件解析 (打印到终端)"""
    print(f"[MIDI] 解析: {os.path.basename(filepath)}")
    result = parse_midi(filepath)
    if result is None:
        print("错误: 不是有效的 MIDI 文件")
        sys.exit(1)

    notes = result['notes']
    print(f"  {result['bpm']:.1f} BPM,  {len(notes)} 个原始音符")

    if target_ch is not None:
        notes = [n for n in notes if n.channel == target_ch]
        print(f"  通道 {target_ch} → {len(notes)} 个音符")
    else:
        # 没指定通道: 自动找音符最多的通道
        from collections import Counter
        ch_counts = Counter(n.channel for n in notes)
        if ch_counts:
            best_ch = ch_counts.most_common(1)[0][0]
            print(f"  自动选择通道 {best_ch} (音符最多)")
            notes = [n for n in notes if n.channel == best_ch]
            print(f"  通道 {best_ch} → {len(notes)} 个音符")

    notes = [n for n in notes if min_p <= n.pitch <= max_p]
    print(f"  音高 [{min_p}..{max_p}] → {len(notes)} 个音符")

    if mono:
        notes = extract_monophonic(notes)
        print(f"  单旋律提取 → {len(notes)} 个音符")

    if not notes:
        print("  [x] 没有符合条件的音符")
        sys.exit(0)

    varname = filename_to_varname(filepath)
    notes = filter_melody(notes)
    seq = gap_to_seq(notes)

    print(f"\n{'='*70}")

    if fmt == 'dual':
        pitches = [s[0] for s in seq]
        durs    = [s[1] for s in seq]
        print(f"static const float {varname}_notes[] = {{")
        for i in range(0, len(pitches), 12):
            chunk = pitches[i:i+12]
            print("    " + ", ".join(f"{p:3d}" for p in chunk) + ",")
        print("};")
        print("")
        print(f"static const uint16_t {varname}_duration_ms[] = {{")
        for i in range(0, len(durs), 12):
            chunk = durs[i:i+12]
            print("    " + ", ".join(f"{d:4d}" for d in chunk) + ",")
        print("};")
    elif fmt == 'song':
        tpq = result['tick_per_qn']
        tempo = result['tempo_us']
        sixteenth_ms = (tpq // 4) * (tempo / tpq) / 1000.0
        notes_sorted = sorted(notes, key=lambda n: n.start_ms)
        print(f"#define LEN {len(notes_sorted)}")
        print(f"uint8_t song[LEN][3] = {{")
        for n in notes_sorted:
            dur_units = max(1, round(n.dur_ms / sixteenth_ms))
            print(f"    {{{n.pitch:3d}, {dur_units:3d}, 255}},")
        print("};")
    else:
        print(format_struct_array(seq, varname,
                                  result['bpm'], result['tick_per_qn']))

    print(f"\n{'='*70}")


# ============================================================
#  主入口
# ============================================================

def main():
    # 无参数 → 批量生成曲库
    if len(sys.argv) < 2:
        batch_build()
        return

    # 有参数 → 单文件模式
    filepath = sys.argv[1]

    fmt = 'struct'
    target_ch = None
    mono = True
    min_p = MIN_PITCH
    max_p = MAX_PITCH

    i = 2
    while i < len(sys.argv):
        a = sys.argv[i]
        if a == '--format' and i + 1 < len(sys.argv):
            fmt = sys.argv[i + 1]; i += 2
        elif a == '--channel' and i + 1 < len(sys.argv):
            target_ch = int(sys.argv[i + 1]); i += 2
        elif a == '--mono':
            mono = True; i += 1
        elif a == '--no-mono':
            mono = False; i += 1
        elif a == '--min-pitch' and i + 1 < len(sys.argv):
            min_p = int(sys.argv[i + 1]); i += 2
        elif a == '--max-pitch' and i + 1 < len(sys.argv):
            max_p = int(sys.argv[i + 1]); i += 2
        else:
            i += 1

    if not os.path.exists(filepath):
        print(f"错误: 文件不存在: {filepath}")
        sys.exit(1)

    single_mode(filepath, fmt, target_ch, mono, min_p, max_p)


if __name__ == '__main__':
    main()
