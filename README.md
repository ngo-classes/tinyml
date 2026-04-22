# TinyML Course Template Repo

This repository provides a single cross-platform layout for a TinyML course using:

- a **conda environment** for Python, Jupyter, plotting, serial tooling, and TensorFlow
- a **downloaded Arduino CLI binary** placed in `tools/arduino-cli/`
- **repo-local Arduino CLI state** so package indexes, downloads, caches, and sketchbook files do not spill into each student's global machine setup

---

## Repository layout

```text
.
├── .arduino-build-cache/
├── .arduino-data/
├── .arduino-downloads/
├── .arduino-user/
├── arduino-cli.yaml
├── data/
├── environment.yml
├── firmware/
├── models/
├── notebooks/
├── scripts/
└── tools/
    └── arduino-cli/
```

- `environment.yml`: conda environment definition
- `arduino-cli.yaml`: Arduino CLI configuration
- `tools/arduino-cli/`: where each student places the correct Arduino CLI binary for their OS
- `data/`: reserved for datasets; contents ignored by git by default
- `models/`: reserved for trained/exported models; contents ignored by git by default
- `.arduino-*`: local Arduino CLI package/cache/user state, kept in the repo but ignored by git

---

## Quick start

### 1. Create and activate the conda environment

```bash
conda env create -f environment.yml
conda activate tinyml
```

### 2. Put the Arduino CLI binary in `tools/arduino-cli/`

Place **one** of the following in `tools/arduino-cli/` depending on your system:

- `arduino-cli.exe` for Windows
- `arduino-cli` for macOS or Linux

That directory is intentionally git-ignored so the binary file is not accidentally committed.

### 3. Activate the repo-local Arduino CLI configuration

#### macOS / Linux / Git Bash

```bash
source scripts/activate.sh
```

#### Windows PowerShell

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\scripts\activate.ps1
```


### 4. Verify setup

#### macOS / Linux / Git Bash

```bash
chmod u+x scripts/check-setup.sh
scripts/check-setup.sh
```

#### Windows PowerShell

```powershell
.\scripts\check-setup.ps1
```

### 5. Setup core libraries

```bash
chmod u+x scripts/bootstrap-arduino.sh
scripts/bootstrap-arduino.sh
```

#### Windows PowerShell

```powershell
.\scripts\bootstrap-arduino.ps1
```


---

## What the activation scripts do

The activation scripts:

1. prepend `tools/arduino-cli/` to your `PATH`
2. point Arduino CLI to this repo's `arduino-cli.yaml`
3. force Arduino CLI's `data`, `downloads`, `user`, and `build_cache` directories to stay local to the repo

This means users can clone the repo, activate the environment, and work without contaminating a machine-wide Arduino setup.


---

## Typical Arduino CLI usage

After activation:

```bash
arduino-cli version
arduino-cli config dump --verbose
arduino-cli core update-index
arduino-cli board list
```

You can then add the board core and libraries you want for the course.

A reasonable starting point for Nano 33 BLE / Nano 33 BLE Sense work is the Arduino Mbed Nano core.

---

## Notes for instructors

- Keep large datasets out of git. Put them under `data/`.
- Keep trained models and exported `.tflite` artifacts under `models/`.
- If you want to pin Arduino cores or libraries for the course, add a bootstrap script in `scripts/`.
- If you later decide to add PlatformIO, you can still keep this structure and add `firmware/platformio.ini` alongside the Arduino CLI flow.

---

## Suggested next additions

- `firmware/` starter sketch templates
- `notebooks/serial_plot.ipynb`
- `models/README.md` describing naming conventions for exported models
