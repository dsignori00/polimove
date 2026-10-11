# polimove

MATLAB project for Polimove perception/offline analysis, with scripts for:
- sensor detection analysis (radar, camera YOLO, PointPillars)
- target tracking analysis
- opponent GPS and ground-truth alignment
- plotting and utility workflows.

## Requirements

- MATLAB (tested on **R2024a**).
- `.mat` datasets/logs exported in the expected structures.
- Optional MATLAB toolboxes/add-ons depending on the script you run.

## Repository layout

- `/Polimove.prj` — MATLAB Project file (recommended entry point).
- `/src/perception/detection` — detection analysis scripts.
- `/src/perception/tracking` — tracking analysis scripts.
- `/src/perception/ground_truth` — ground truth construction scripts.
- `/src/perception/utils` — shared perception utilities.
- `/common` — shared plotting/constants/utilities.
- `/databases` — track databases (`*.mat`) selectable by scripts.
- `/bags` — log files (`*.mat`) loaded by analysis scripts.

## Quick start (recommended: MATLAB Project)

1. Open MATLAB.
2. Open the project file:
   - `Home -> Open -> Project`, then select `Polimove.prj`,
   - or from command window: `openProject('Polimove.prj')`.
3. Keep the project open while running scripts.

Why this matters: several utilities use `currentProject()` and assume the project name is `polimove`, so opening scripts without loading the project can break path/data resolution.

## Data setup

- Put log files in `/bags` (for `uigetfile`-based selection in analysis scripts).
- Keep track databases in `/databases` (already populated with sample track files).
- Ground-truth files used by opponent GPS/tracking workflows are expected under project paths resolved by utility functions.

## Notes and troubleshooting

- If `currentProject()` errors appear, reopen `Polimove.prj`.
- If no files appear in dialogs, verify logs are in `/bags` and have `.mat` extension.
- Some scripts keep variables between runs (`clearvars -except ...`); use a fresh workspace if results look inconsistent.

## Author

**Daniele Signori**
- GitHub: [@dsignori00](https://github.com/dsignori00)
- LinkedIn: [daniele-signori](https://www.linkedin.com/in/daniele-signori/)

## Contributing

Contributions and issues are welcome via GitHub.
