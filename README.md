# VESC GUI split version

Run:

```bash
python main.py
```

## File layout

- `main.py`: application entry point.
- `config.py`: constants and timing settings.
- `state.py`: shared thread-safe state.
- `diagnostics.py`: diagnostics, logs, and debug counters.
- `vesc_messages.py`: all PyVESC message classes and packet IDs.
- `vesc_comm.py`: all serial/CAN forwarding, custom commands, CAN ping scan, and comm thread.
- `telemetry.py`: value conversion, history, response summaries.
- `widgets.py`: reusable PyQt widgets.
- `gui.py`: GUI layout and UI callbacks.

Communication work should mostly stay in `vesc_comm.py` and `vesc_messages.py`.
