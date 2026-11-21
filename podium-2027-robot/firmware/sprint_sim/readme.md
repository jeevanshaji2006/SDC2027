# Sprint drivetrain simulation
Python model of battery -> PWM timer -> BTS7960 driver -> PG36M555 motor -> encoder.

Setup (Python 3.12):
    python -m venv .venv
    .venv\Scripts\activate.bat
    pip install -r requirements.txt

Run:
    python run_open_loop.py
    python run_encoder_test.py
    python test_reversal.py

All motor, battery and driver numbers are in params.py. Values marked ASSUMED are guesses to replace with measurements.