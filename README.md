# 4bar-bike-suspension-analysis

Mountain bike suspension analysis for four-bar linkages: rear wheel and instant center of rotation paths

## Usage:

* Add a bike picture (`.jpg`,`.jpeg`) to the current directory
* Run `python3 4bar_analysis.py`
* Follow the instructions on the image
* The analysis plot `4bar_analysis.png` is saved to the current directory

## Features:

* Automatic linkage configuration detection (linkage-driven single pivot, Horst link, Split-pivot, VPP, DW/Maestro link)
* Calculate linkage, rear axle and instant center of rotation positions based on main pivot rotation
* Plot rear axle and instant center of rotation paths
* :warning: Absolutely not accurate

![4bar_analysis.png](4bar_analysis.png)
(image source: https://www.santacruzbicycles.com/en-US/nomad)

## Python dependencies:

* Python >= 3.7.1
* NumPy >= 1.15.4
* SciPy >= 1.1.0
* Matplotlib >= 2.2.2

## ToDo:

* Calculate shock rocker mount path
* Calculate rear wheel/shock travel
* Plot leverage curve (rear wheel travel/shock travel vs rear wheel travel)
