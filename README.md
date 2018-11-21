# 4bar-bike-suspension-analysis

Mountain bike suspension analysis for four-bar linkages

## Usage:

* Needs an interactive Python shell (iPython, Spyder)
* Add a bike picture (`.jpg`,`.jpeg`) to the  current directory
* Run `4bar_analysis.py`

## Features:

* Automatic linkage configuration detection (linkage-driven single pivot, Horst link, Split-pivot, VPP, DW/Maestro link)
* Calculate linkage, rear axle and instant center of rotation positions based on main pivot rotation
* Plot rear axle and instant center of rotation paths
* :warning: Absolutely not accurate

![4bar_analysis.png](4bar_analysis.png)

## Python dependencies:

* Python 3 (tested with Python 3.7.0 and Spyder 3.3.1)
* NumPy
* SciPy
* Matplotlib

## ToDo:

* Calculate shock rocker mount path
* Calculate rear wheel/shock travel
* Plot leverage curve (rear wheel travel/shock travel vs rear wheel travel)
