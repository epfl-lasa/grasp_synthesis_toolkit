# Grasp Synthesis Toolkit
This package is a minimum realization of model-based grasping synthesis problem.
Given the explicit models of robotic hand and target object, the robot grasping task is formulated as a constrained optimization problem.
Symbolic expressions are used to accelerate online optimization.

This package is being actively updated from time to time.

## Get started
- Create a 20-dof human hand model by executing `hand = mySGparadigmatic()`.

- Try intuitive grasping by closing fingers, run `mySGexample` under the directory `mySynGrasp/examples`.

- Try to solve an optimization-based grasping synthesis problem given hand and object model, run `main` under `src`.

## Dependencies

### Hand model and grasp planning
- [SynGrasp 2.3](http://sirslab.dii.unisi.it/syngrasp/)

### Toolbox
- [Analyze N-dimensional Polyhedra in terms of Vertices or (In)Equalities](https://ch.mathworks.com/matlabcentral/fileexchange/30892-analyze-n-dimensional-polyhedra-in-terms-of-vertices-or-in-equalities)

## References
[1] **Chapter 28: Grasping**, Domenico Prattichizzo, Jeffrey C. Trinkle. In *Springer handbook of robotics (2008), B. Siciliano, O. Khatib*

[2] **Hand Posture Subspaces for Dexterous Robotic Grasping**

[3] **SynGrasp: a MATLAB Toolbox for Grasp Analysis of Human and Robotic Hands**

[4] **On the Role of Hand Synergies in the Optimal Choice of Grasping Forces**

## Remarks
- All parameters in *parameter_checklist.md* are defined according to the reference [1], *Chapter 28: Grasping*.

- All variables are defined following column-priority convention.

- Pay attention to Allegro hand DH parameter offset in rotation angles. These offset values only affect joint angle commands sent to the robot, and should be added to joint angle variables before moving fingers.
Function `moveFinger` consideres this offset for both `numerical (num)` evaluation, and `symbolic (sym)` evaluation. Symbolic expressions of link positions are used in optimization problems to accelerate the calculation. Numerical evaluation can be used for double-check. Symbolic expression is obtained in `mySGmakeFinger`, `calcSymbolicLinkTransformation`. Such symbolic expressions are saved as function scripts offline in local directories. Then numerical option appears in `moveFinger`.

## License
This package is created and used for semester project at LASA, EPFL only. YOU ARE NOT PERMITTED TO RELEASE, PUBLISH, OR DISTRIBUTE ANY PART OF THIS PACKAGE.
For more questions, please contact the package author [Kunpeng Yao](kunpeng.yao@epfl.ch).