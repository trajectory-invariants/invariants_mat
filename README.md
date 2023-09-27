Matlab code for calculating invariant descriptors of trajectories using optimal control. 

A **Python version** of this package is [under active development](https://gitlab.kuleuven.be/robotgenskill/public_code/invariants_py).

## Features

Two types of descriptors are supported:
1. **Vector invariants** (i.e. Frenet-Serret invariants of curvature and torsion) which can be applied to position, orientation, translational velocity, rotational velocity, force and moment trajectories
2. **Screw invariants** (based on the instantaneous screw axis) which can be applied to screw twist and wrench trajectories

In addition to calculating the invariants, this toolbox supports:
- robust calculation of the corresponding moving frames: *Frenet-Serret frames* for the vector invariants and *Instantaneous Screw Axis frames* for the screw invariants
- reconstruction of trajectories from the invariant descriptor


## Installation

Download [**CasADi 3.6.3**](https://github.com/casadi/casadi/releases/tag/3.6.3) according to your version of Matlab and operating system. Unzip the downloaded package and place the resulting folder inside the `invariants-mat/libraries/` directory. 

As an example, if you are in Windows and using Matlab >R2016a, the result should be that CasADi can be found here:
`invariants-mat/libraries/casadi-windows-matlabR2016a-v3.6.3`

## 
Main contributors: Maxim Vochten, Ali Mousavi, Arno Verduyn, Riccardo Burlizzi (KU Leuven)
