# User interface DDC264EVM_UI for DDC264EVM board

This software was created to record data from board [DDC264EVM](https://www.mouser.com/datasheet/2/405/sbau186-124984.pdf?srsltid=AfmBOorEEgAYgOaRBD1N2l7HP0iqDXy_jllDeBUeok_HDVKO7DB86BW0) automatically. USB communication was established via [DDC264EVM_IO.dll](https://github.com/Padniuk/DDC264EVM_IO) (It is important to mention that it was written for 32bit compilator). As UI tool it uses framework PyQt5.

## Installation and usage
Install 32-bit python on Windows and run the following commands with it:

```powershell
python -m venv env
.\env\Scripts\Activate
```
Install necessary modules:
```powershell
pip install -r requirements.txt 
pip install -r requirements_old.txt # for old python
```
Run app:

```powershell
python main.py
```