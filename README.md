# PSO-COLA

# About

# Installing

## Matlab Installation
PSO-COLA requires the following libraries installed:
- Computer Vision Toolbox
- Image Processing Toolbox


Also, PSO-COLA was tested with Matlab R2018a

Clone the repo to your local directory. To do this. Run the following command:
```shell script
git clone https://github.com/RedMerk/PSO-COLA.git
```

Open the repo folder in Matlab, and compile the file assignmentoptimal.c running the command:
```shell Matlab
mex -largeArrayDims PSO-COLA/assignmentoptimal.c -lut
```
For Windows users we recommend to use Microsoft Visual Studio 2017 with c++ compiler

# Running

A basic usage example is included in Main.m, for the Standford Bunny and Dragon datasets
