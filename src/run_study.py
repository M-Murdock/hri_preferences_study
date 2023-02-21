#!/usr/bin/env python3

import asyncio
import os
import study_runner
from  study_runner.frames.logging import LoggingFrame, RunLogging
import tkinter

class CountConfigFrame(tkinter.Frame):
    def __init__(self, parent, _):
        super().__init__(parent)
        self._entry = tkinter.Entry(self)
        self._entry.grid()

    def get_config(self):
        return {"count": int(self._entry.get())}
    def set_state(self, state):
        self._entry.configure(state=state)

async def count(config, status_cb):
    with RunLogging(config):
        for i in range(config["count"]):
            await asyncio.sleep(1.)
            print(i+1)

class BasicLogger:
    def __init__(self, data_dir, config):
        self._file = open(os.path.join(data_dir, "log.txt"), "w")

    def start(self):
        self._file.write("started")

    def stop(self):
        self._file.write("stopped")
        self._file.close()

def main():
    root = tkinter.Tk()
    runner = study_runner.StudyRunner(root, count)
    runner.add_config_frame(CountConfigFrame, "Count")
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    logging_frame.add_logger("basic", BasicLogger)
    study_runner.runner.main(root)


if __name__ == "__main__":
    main()
