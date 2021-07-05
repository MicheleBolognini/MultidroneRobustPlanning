# Multidrone Robust Planning Toolbox

### Requirements
This toolbox relies on LKH to solve TSP instances. It is therefore necessary to download it and compile it,
 so that the resulting LKH script is located in the "LKH" folder.
It is available both from the [official site](http://webhotel4.ruc.dk/~keld/research/LKH-3/) and from the related [git repository](https://github.com/cerebis/LKH3)
Download the code or clone the repository, then rename the folder and compile it

```shell
tar xvfz LKH-3.0.6.tgz
mv LKH-3.0.6.tgz LKH
cd LKH
make
```

Compiling will make the executable available, the file is called 'LKH'.

