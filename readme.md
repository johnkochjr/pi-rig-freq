# pi-rig-freq

Raspberry Pi / Windows app that reads rig frequency from:
- Hamlib `rigctld`
- Ham Radio Deluxe IP Server (v5 framed protocol)
- USB CAT (e.g., `FA;` / `IF;`)

## Quick start (Windows)
```powershell
py -3 -m venv .venv
.\.venv\Scripts\pip install -r requirements.txt
.\.venv\Scripts\python.exe .\pi-rig-freq.py --gui
