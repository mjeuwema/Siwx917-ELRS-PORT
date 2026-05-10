#!/usr/bin/env python3
"""Decode LR1121 SPI frames from a Saleae digital transition CSV."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass


COMMANDS = {
    0x0100: "GetStatus",
    0x0101: "GetVersion",
    0x010D: "GetErrors",
    0x010E: "ClearErrors",
    0x010F: "Calibrate",
    0x0110: "SetRegMode",
    0x0111: "CalibrateImage",
    0x0113: "SetDioIrqParams",
    0x0114: "GetIrqStatus",
    0x0115: "ClearIrq",
    0x0117: "SetTcxoMode",
    0x011C: "SetStandby",
    0x0203: "GetRxBufferStatus",
    0x0204: "GetPacketStatus",
    0x0205: "GetRssiInst",
    0x0208: "SetLoRaPublicNetwork",
    0x0209: "SetRx",
    0x020A: "SetTx",
    0x020B: "SetRfFrequency",
    0x020E: "SetPacketType",
    0x020F: "SetModulationParams",
    0x0210: "SetPacketParams",
    0x0213: "SetRxTxFallbackMode",
    0x0215: "SetPaCfg",
    0x022B: "SetLoRaSyncWord",
    0x0700: "ELRS_GET_PACKET",
    0x0701: "ELRS_SET_FREQ_SET_RX",
    0x0702: "ELRS_SET_RX_GET_PACKET",
    0x0703: "ELRS_SET_FREQ_SET_RX_GET_PACKET",
}

PACKET_TYPES = {
    0: "RCDATA/LINKSTATS",
    1: "DATA",
    2: "SYNC",
    3: "RESERVED",
}


@dataclass
class Frame:
    start: float
    end: float
    mosi: list[int]
    miso: list[int]

    @property
    def opcode(self) -> int | None:
        if len(self.mosi) < 2:
            return None
        return (self.mosi[0] << 8) | self.mosi[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--limit", type=int, default=120)
    parser.add_argument("--around", type=float, default=None)
    parser.add_argument("--window", type=float, default=0.05)
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--crc-init", type=lambda value: int(value, 0), default=0xD29C)
    return parser.parse_args()


def bits_to_bytes(bits: list[int]) -> list[int]:
    out: list[int] = []
    for i in range(0, len(bits) - 7, 8):
        val = 0
        for bit in bits[i : i + 8]:
            val = (val << 1) | (bit & 1)
        out.append(val)
    return out


def decode_csv(path: str) -> list[Frame]:
    frames: list[Frame] = []
    state = [0, 0, 0, 1]
    in_frame = False
    start = 0.0
    mosi_bits: list[int] = []
    miso_bits: list[int] = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["Time [s]"])
            new_state = [
                int(row["Channel 0"]),
                int(row["Channel 1"]),
                int(row["Channel 2"]),
                int(row["Channel 3"]),
            ]

            old_sck, _, _, old_cs = state
            sck, mosi, miso, cs = new_state

            if old_cs == 1 and cs == 0:
                in_frame = True
                start = t
                mosi_bits = []
                miso_bits = []

            if in_frame and old_sck == 0 and sck == 1 and cs == 0:
                mosi_bits.append(mosi)
                miso_bits.append(miso)

            if in_frame and old_cs == 0 and cs == 1:
                frames.append(Frame(start, t, bits_to_bytes(mosi_bits), bits_to_bytes(miso_bits)))
                in_frame = False

            state = new_state

    return frames


def fmt_bytes(values: list[int], max_len: int = 24) -> str:
    text = " ".join(f"{value:02X}" for value in values[:max_len])
    if len(values) > max_len:
        text += " ..."
    return text


class Crc2Byte:
    def __init__(self, bits: int, poly: int) -> None:
        self.bits = bits
        self.poly = poly
        self.bitmask = (1 << bits) - 1
        highbit = 1 << (bits - 1)
        self.table: list[int] = []
        for i in range(256):
            crc = i << (bits - 8)
            for _ in range(8):
                crc = (crc << 1) ^ (poly if crc & highbit else 0)
            self.table.append(crc)

    def calc(self, data: list[int], crc: int) -> int:
        for value in data:
            index = ((crc >> (self.bits - 8)) ^ value) & 0xFF
            crc = (crc << 8) ^ self.table[index]
        return crc & self.bitmask


def validate_std_crc(payload: list[int], crc_init: int, nonce: int) -> tuple[bool, int, int]:
    if len(payload) < 8:
        return False, 0, 0
    in_crc = ((payload[0] >> 2) << 8) | payload[7]
    calc_data = [(payload[0] & 0x03)] + payload[1:7]
    calc_crc = Crc2Byte(14, 0x2E57).calc(calc_data, crc_init ^ nonce)
    return in_crc == calc_crc, in_crc, calc_crc


def describe(frame: Frame) -> str:
    opcode = frame.opcode
    if opcode is None:
        return "short"
    name = COMMANDS.get(opcode, "UNKNOWN")
    args = frame.mosi[2:]
    details = ""
    if opcode == 0x020B and len(args) >= 4:
        freq_reg = int.from_bytes(bytes(args[:4]), "big")
        details = f" freqReg=0x{freq_reg:08X}"
    elif opcode == 0x0209 and len(args) >= 3:
        timeout = (args[0] << 16) | (args[1] << 8) | args[2]
        details = f" timeout=0x{timeout:06X}"
    elif opcode == 0x0701 and len(args) >= 7:
        freq_reg = int.from_bytes(bytes(args[:4]), "big")
        timeout = (args[4] << 16) | (args[5] << 8) | args[6]
        details = f" freqReg=0x{freq_reg:08X} timeout=0x{timeout:06X}"
    return f"0x{opcode:04X} {name}{details}"


def get_packet_payload(response: Frame | None) -> list[int]:
    if not response or response.opcode != 0 or len(response.miso) < 14:
        return []
    return response.miso[6:14]


def describe_packet_payload(payload: list[int], crc_init: int = 0xD29C) -> str:
    if len(payload) < 8:
        return "payload=<none>"

    pkt_type = payload[0] & 0x03
    type_name = PACKET_TYPES.get(pkt_type, f"type{pkt_type}")
    nonce_validator = 0 if pkt_type == 2 else -1
    crc_text = ""
    if nonce_validator == 0:
        crc_ok, in_crc, calc_crc = validate_std_crc(payload, crc_init, nonce_validator)
        crc_text = f" crc={'OK' if crc_ok else 'BAD'}({in_crc:04X}/{calc_crc:04X})"
    details = f"type={type_name} raw={fmt_bytes(payload, 8)}{crc_text}"
    if pkt_type == 2:
        details += (
            f" fhss={payload[1]} nonce={payload[2]} rfRate=0x{payload[3]:02X}"
            f" flags=0x{payload[4]:02X} uid={payload[5]:02X}:{payload[6]:02X}"
            f" crcLow=0x{payload[7]:02X}"
        )
    return details


def summarize(frames: list[Frame], crc_init: int) -> None:
    counts: dict[int | None, int] = {}
    for frame in frames:
        counts[frame.opcode] = counts.get(frame.opcode, 0) + 1

    print("Opcode counts:")
    for opcode, count in sorted(counts.items(), key=lambda item: (-item[1], item[0] or -1)):
        if opcode is None:
            print(f"  short: {count}")
        else:
            print(f"  0x{opcode:04X} {COMMANDS.get(opcode, 'UNKNOWN')}: {count}")

    print("\nELRS_GET_PACKET command/response pairs:")
    shown = 0
    packet_type_counts: dict[int, int] = {}
    sync_payloads: list[tuple[float, list[int]]] = []
    packet_times: list[float] = []
    for idx, frame in enumerate(frames):
        if frame.opcode != 0x0700:
            continue
        response = frames[idx + 1] if idx + 1 < len(frames) else None
        response_bytes = response.miso if response and response.opcode == 0 else []
        payload = get_packet_payload(response)
        if payload:
            pkt_type = payload[0] & 0x03
            packet_type_counts[pkt_type] = packet_type_counts.get(pkt_type, 0) + 1
            packet_times.append(frame.start)
            if pkt_type == 2:
                sync_payloads.append((frame.start, payload))
        if shown < 20:
            print(
                f"  t={frame.start:.6f}s status={fmt_bytes(frame.miso, 6)} "
                f"resp={fmt_bytes(response_bytes, 16)} "
                f"{describe_packet_payload(payload, crc_init)}"
            )
            shown += 1

    print("\nDecoded OTA packet types:")
    for pkt_type, count in sorted(packet_type_counts.items()):
        print(f"  {PACKET_TYPES.get(pkt_type, f'type{pkt_type}')}: {count}")

    print("\nSYNC payloads:")
    for start, payload in sync_payloads[:40]:
        print(f"  t={start:.6f}s {describe_packet_payload(payload, crc_init)}")

    if len(packet_times) > 1:
        print("\nUnusual GET_PACKET gaps:")
        unusual_packets = 0
        for prev, current in zip(packet_times, packet_times[1:]):
            dt_ms = (current - prev) * 1000.0
            if dt_ms > 60.0:
                print(f"  gap={dt_ms:.3f}ms at t={current:.6f}s")
                unusual_packets += 1
                if unusual_packets >= 20:
                    break

    retunes = [frame for frame in frames if frame.opcode in (0x020B, 0x0701)]
    print(f"\nRetune commands: {len(retunes)}")
    for frame in retunes[:20]:
        print(f"  t={frame.start:.6f}s {describe(frame)}")

    print("\nUnusual retune gaps:")
    unusual = 0
    for prev, frame in zip(retunes, retunes[1:]):
        dt_ms = (frame.start - prev.start) * 1000.0
        if dt_ms < 1.0 or dt_ms > 60.0:
            print(f"  gap={dt_ms:.3f}ms at t={frame.start:.6f}s")
            unusual += 1
            if unusual >= 20:
                break

    get_irq = [frame for frame in frames if frame.opcode == 0x0114]
    print(f"\nGetIrqStatus commands: {len(get_irq)}")
    for frame in get_irq[:20]:
        print(f"  t={frame.start:.6f}s mosi={fmt_bytes(frame.mosi, 8)} miso={fmt_bytes(frame.miso, 8)}")


def main() -> int:
    args = parse_args()
    frames = decode_csv(args.csv_path)
    selected = frames
    if args.around is not None:
        selected = [
            frame
            for frame in frames
            if args.around - args.window <= frame.start <= args.around + args.window
        ]

    print(f"Decoded frames: {len(frames)}")
    if args.summary:
        summarize(frames, args.crc_init)
        return 0

    print(f"Showing frames: {min(len(selected), args.limit)}")
    for index, frame in enumerate(selected[: args.limit]):
        duration_us = (frame.end - frame.start) * 1_000_000.0
        print(
            f"{index:04d} t={frame.start:10.6f}s dur={duration_us:8.1f}us "
            f"len={len(frame.mosi):02d} {describe(frame)} "
            f"MOSI=[{fmt_bytes(frame.mosi)}] MISO=[{fmt_bytes(frame.miso)}]"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
