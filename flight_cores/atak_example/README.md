# atak_example core

Simple core that prints each newly received ATAK CoT event summary from the `atak_interface` state stream.

## Run

```bash
export MAIN_CONFIG=flight_cores/atak_example/config.yaml
python main.py
```

## Output

The core logs:

- `last_event` whenever the ATAK interface publishes `ATAK.State.Rx.LastEvent`
