# Multidrone Robust Planning Toolbox

## Installation
1. Clone the repository or get the toolbox from the MATLAB File Exchange [![View Multidrone Robust Planning on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://it.mathworks.com/matlabcentral/fileexchange/95533-multidrone-robust-planning)
2. Install LKH

```shell
git clone https://github.com/MicheleBolognini/MultidroneRobustPlanning

```

This toolbox relies on LKH to solve TSP instances. It is therefore necessary to download it and compile it,
 so that the resulting LKH script is located in the "LKH" folder.
It is available both from the [official site](http://webhotel4.ruc.dk/~keld/research/LKH-3/) and from the related [git repository](https://github.com/cerebis/LKH3)
Download the code or clone the repository, then rename the folder and compile it

```shell
cd MultidroneRobustPlanning
git clone https://github.com/cerebis/LKH3
tar xvfz LKH-3.0.6.tgz
mv LKH-3.0.6.tgz LKH
cd LKH
make
```

Compiling will make the executable available, the file is called 'LKH'.


This toolbox provides functions for planning paths for a set of drones tasked with traversing a set of 3D points in a known and cluttered environment.
The algorithm is described in [[1]](#1).



## References
<a id="1">[1]</a> 
M. Bolognini, L. Fagiano, M.P. Limongelli (2021). 
An autonomous, robust, multi-agent UAV pathplanner for inspection of the built environment.
???
