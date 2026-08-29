#!/usr/bin/env python3
"""
PID_Levo - kart uzerindeki degiskenleri SWD ile okur.

Hata ayiklayici oturumu ACMADAN, ELF'teki sembol tablosunu kullanarak
RAM'i dogrudan okur ve degerleri cozer. CubeIDE'de Live Expressions
acmaya gerek kalmaz.

Kullanim:
    python tools/read_vars.py                 # varsayilan degisken seti
    python tools/read_vars.py g_heave_mm g_sm_state
    python tools/read_vars.py --elf build/PID_Levo.elf --wait 8

DIKKAT: CubeIDE'de acik bir debug oturumu varsa ST-LINK'i o tutar ve bu
betik "DEV_CONNECT_ERR" alir. Once oturumu kapatin.
"""
import argparse
import re
import struct
import subprocess
import sys
import time

CLI = (r"C:\Program Files\STMicroelectronics\STM32Cube"
       r"\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe")
NM = (r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
      r"\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32"
      r".14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-nm.exe")

# Tur bilgisi ELF'te yok; elle tutulan kucuk bir sozluk yeterli.
# "f" = float, "u" = isaretsiz tamsayi, "fN"/"uN" = N elemanli dizi.
KIND = {
    "g_sm_state": "u", "g_sm_faults": "u", "g_levitate_req": "u",
    "g_vbus_v": "f", "g_power_w": "f", "g_peak_i_a": "f", "g_peak_p_w": "f",
    "g_coil_i_a": "f4", "g_duty": "f4", "g_drv_init_err": "u",
    "g_gap_found": "u", "g_gap_valid_n": "u",
    "g_gap_present": "u4", "g_gap_fault": "u4", "g_gap_raw": "u4",
    "g_gap_ch": "f4", "g_gap_hz": "u4:2", "g_gap_status": "u4",
    "g_est_valid": "u", "g_est_n_used": "u", "g_heave_mm": "f",
    "g_roll_mrad": "f", "g_pitch_mrad": "f", "g_est_resid_mm": "f",
    "g_range_mm": "u", "g_range_status": "u", "g_gap_mm": "f",
    "g_ok_count": "u", "g_sample_hz": "u", "g_i2c_err_count": "u",
    "g_sensor_present": "u", "g_scan_root_n": "u", "g_scan_ch0_n": "u",
    "g_scan_root": "u8", "g_scan_ch0": "u8",
    "g_stat_mean": "f", "g_stat_std": "f", "g_stat_pp": "u", "g_stat_n": "u",
    "uwTick": "u",
}

DEFAULT = ["uwTick", "g_sm_state", "g_sm_faults", "g_vbus_v",
           "g_gap_found", "g_gap_valid_n", "g_gap_present", "g_gap_fault",
           "g_gap_raw", "g_gap_ch", "g_gap_hz", "g_gap_status",
           "g_est_valid", "g_est_n_used", "g_heave_mm", "g_roll_mrad",
           "g_pitch_mrad", "g_est_resid_mm", "g_i2c_err_count"]

SM_NAMES = ["INIT", "SELFTEST", "IDLE", "SOFT_START",
            "LEVITATING", "LANDING", "FAULT", "SAFE_SHUTDOWN"]


def symbols(elf):
    out = subprocess.run([NM, "-S", elf], capture_output=True, text=True).stdout
    tab = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4:
            tab[p[3]] = (int(p[0], 16), int(p[1], 16))
    return tab


def read_range(lo, hi):
    out = subprocess.run(
        [CLI, "-c", "port=SWD", "mode=HOTPLUG", "-r32", hex(lo), str(hi - lo)],
        capture_output=True, text=True).stdout
    if "Error" in out and "0x" not in out:
        sys.exit("SWD okunamadi. CubeIDE'de acik bir debug oturumu var mi?\n" + out)
    mem = {}
    for m in re.finditer(r"^(0x[0-9A-F]+) : (.*)$", out, re.M):
        base = int(m.group(1), 16)
        for i, w in enumerate(m.group(2).split()):
            mem[base + 4 * i] = int(w, 16)
    return mem


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*", default=None)
    ap.add_argument("--elf", default="build/PID_Levo.elf")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="okumadan once beklenecek saniye (acilis icin)")
    args = ap.parse_args()

    names = args.names or DEFAULT
    tab = symbols(args.elf)
    missing = [n for n in names if n not in tab]
    if missing:
        sys.exit("ELF'te bulunamadi: " + ", ".join(missing))

    if args.wait:
        time.sleep(args.wait)

    lo = min(tab[n][0] for n in names) & ~3
    hi = (max(tab[n][0] + tab[n][1] for n in names) + 3) & ~3
    if hi - lo > 0x4000:
        sys.exit("Sembol araligi cok genis; daha az degisken isteyin.")
    mem = read_range(lo, hi)

    def raw(addr, n):
        d = b""
        for k in range(0, n + 3, 4):
            d += struct.pack("<I", mem.get((addr & ~3) + k, 0))
        return d[(addr & 3):(addr & 3) + n]

    for n in names:
        addr, size = tab[n]
        kind = KIND.get(n, "u")
        array_kind = re.fullmatch(r"([fu])(\d+)(?::(\d+))?", kind)
        if array_kind and array_kind.group(1) == "f":
            count = int(array_kind.group(2))
            v = [struct.unpack("<f", raw(addr + 4 * i, 4))[0]
                 for i in range(count)]
            print(f"{n:18s} = [" + ", ".join(f"{x:8.3f}" for x in v) + "]")
        elif array_kind:
            count = int(array_kind.group(2))
            width = int(array_kind.group(3) or 1)
            v = [int.from_bytes(raw(addr + width * i, width), "little")
                 for i in range(count)]
            print(f"{n:18s} = {v}")
        elif kind == "f":
            print(f"{n:18s} = {struct.unpack('<f', raw(addr, 4))[0]:.3f}")
        else:
            val = int.from_bytes(raw(addr, size), "little")
            extra = ""
            if n == "g_sm_state" and val < len(SM_NAMES):
                extra = f"  ({SM_NAMES[val]})"
            if n == "g_sm_faults":
                extra = f"  (0x{val:04X})"
            print(f"{n:18s} = {val}{extra}")


if __name__ == "__main__":
    main()
