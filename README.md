Matlab code for calculating invariant descriptors of force and moment trajectories using optimal control. Two types of descriptors are supported:
1. **Vector invariants** (i.e. Frenet-Serret invariants of curvature and torsion) which can be applied to position, orientation, translational velocity, rotational velocity, force and moment trajectories
2. **Screw invariants** (based on the instantaneous screw axis) which can be applied to screw twist and wrench trajectories

In addition to calculating the invariants, this toolbox supports:
- robust calculation of the corresponding moving frames: *Frenet-Serret frames* for the vector invariants and *Instantaneous Screw Axis frames* for the screw invariants
- reconstruction of trajectories from the invariant descriptor

Main contributors: Maxim Vochten, Ali Mousavi, Arno Verduyn, Riccardo Burlizzi (KU Leuven)

## Installation instructions

Download the **CasADi 3.5.5** Matlab package from [https://web.casadi.org/get/](https://web.casadi.org/get/) according to your version of Matlab and operating system. Unzip the downloaded package and place the resulting folder inside the `libraries/` folder. 

As an example, if you are in Windows and using Matlab >R2016a, the result should be that CasADi can be found here: 
`invariants-motion-and-force-trajectories/libraries/casadi-windows-matlabR2016a-v3.5.5/`

## Application to a demonstrated 3D contour following task

The framework is applied for a human-demonstrated 3D contour following task where both the motion and contact force of the tool in contact with the contour are recorded. The invariant properties of the descriptors under different calibrations, experimental setups and artificial transformations are verified.

For the analysis with screw invariant descriptors, run `contourfollowing_screw_invariants`.

For the analysis with vector invariant descriptors, run `contourfollowing_vector_invariants`.

## Application to a demonstrated peg-on-hole alignment task



