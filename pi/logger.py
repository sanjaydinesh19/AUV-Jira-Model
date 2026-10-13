"""CSV run logger with the column order fixed in CLAUDE.md Section 6, Step 4."""
import csv

import config


class RunLogger:
    def __init__(self, path):
        self._file = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=config.LOG_COLUMNS)
        self._writer.writeheader()

    def write(self, **row):
        # DictWriter raises on a missing or unknown column, so a log can never
        # silently drift away from the agreed format.
        self._writer.writerow(row)

    def close(self):
        self._file.close()
