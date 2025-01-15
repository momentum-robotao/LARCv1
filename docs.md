# Documentation

```python
import sys

print(sys.path) # ? para ver path do controller
```

## Debug

### Importance

- Keep after closing Webots and we can save it to analyze even after running other simulations
- Easier to debug a system multiple times (you don't need to create logs, delete / comment and then recreate them the next time)

## Tests

Run via `python -m pytest`

## Run

### Competition

Comment `COPY ./ngrok.txt ./ngrok.txt` from `DockerFile`.
