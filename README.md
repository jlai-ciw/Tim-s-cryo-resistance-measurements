# Cryo Resistance Logger

Logs **time, temperature, and resistance** using a **Keithley 2400 SourceMeter**
(resistance) and a **Cryo-con 22C** (temperature), with live plots.

## Run it

```
python cryo_resistance_logger.py
```

(Requires `pyvisa` and `matplotlib` — already installed on this PC — plus a
VISA backend such as NI-VISA, which is also already present.)

## How to use

1. **Instruments** — turn both instruments on, click **Rescan bus**, and pick
   each instrument's GPIB address from the dropdowns (or type one manually,
   e.g. `GPIB0::24::INSTR`). Defaults guessed: Keithley = address 24,
   Cryo-con = address 12. Pick the Cryo-con input channel (A/B) and whether
   to use **4-wire** sensing (recommended for low-resistance cryo samples).
   Click **Connect** — the status box shows both instruments' IDs.
2. **Data file** — click **Browse…** to choose the directory and file name.
   Data is saved as CSV: `timestamp, elapsed_s, temperature_K, resistance_ohm`.
   If the file already exists, new data is appended (header written only once).
3. **Logging** — set the interval in seconds and click **Start logging**.
   The interval can be changed *while running*; the new value takes effect on
   the next point. Each point is written and flushed to disk immediately, so
   nothing is lost if the program is killed mid-run.
4. The **Current reading** box shows the latest time / temperature /
   resistance, and the two plots (temperature vs. time, resistance vs. time)
   update live.

## Simulation mode

Check **Simulation mode** before connecting to test the program with fake
data (a simulated cooldown and a wandering ~100 Ω resistance) — no
instruments needed.

## Notes

- The Keithley is put in **auto-ohms mode** (it chooses its own source
  current). If your sample needs a specific excitation current (e.g. to avoid
  self-heating at low temperature), that can be added as a setting.
- If a sensor fault makes the Cryo-con return non-numeric data, the
  temperature is recorded as `nan` and logging continues.
