Matlab code to support the robust calculation of invariant trajectory descriptors using optimal control. 

## Features

The following type of descriptors can be calculated using optimal control:
1. **Vector invariants** are a generalization of the Frenet-Serret invariants for point curves based on differential-geometric properties such as curvature and torsion. They can be calculated for position trajectories, orientation trajectories, translational velocity trajectories, rotational velocity trajectories, force trajectories, and moment trajectories. 
2. **Screw invariants** are defined based on the concept of the Instantaneous Screw Axis (ISA) for motion and force. This results in them being independent of reference point. The screw invariants can be calculated for pose trajectories, screw twist trajectories, and wrench trajectories. 

In addition to calculating the invariant descriptors, this toolbox supports:
- robust calculation of the corresponding moving frames: *Frenet-Serret frames* for the vector invariants and *Instantaneous Screw Axis frames* for the screw invariants
- reconstruction of trajectories from the invariant descriptor

## Installation instructions

Download [**CasADi 3.5.5**](https://github.com/casadi/casadi/releases/tag/3.5.5) according to your version of Matlab and operating system. Unzip the downloaded package and place the resulting folder inside the `invariants_mat/libraries/` folder. 

As an example, if you are in Windows and using Matlab >R2016a, the result should be that CasADi can be found here: 
`invariants_mat/libraries/casadi-windows-matlabR2016a-v3.5.5/`

## Upcoming improvements
- **More calculation examples** will be added that show how to calculate different invariant descriptors on single trajectories and on special motions. Expected release: end of October 2023.
- Functionality will be added to **enable trajectory generation** starting from an invariant descriptor using the approach described [here](https://doi.org/10.1016/j.robot.2019.103291). Expected release: end of November 2023.
- A **Python version** is currently under active development, referred to as [invariants_py](https://gitlab.kuleuven.be/robotgenskill/public_code/invariants_py). It will mirror all functionality and in addition focus on very fast calculation times by incorporating the [Fatrop solver](https://gitlab.kuleuven.be/robotgenskill/fatrop/fatrop).

### Contributors 

Main contributors: Maxim Vochten, Ali Mousavi, Arno Verduyn, Riccardo Burlizzi (KU Leuven)

For questions and collaborations: maxim(dot)vochten(at)kuleuven(dot)be
