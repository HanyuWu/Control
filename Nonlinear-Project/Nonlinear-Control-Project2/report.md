# EML 6351 Simulation Project 2<br>

* ## README

For assignment 1, the code for implementing a RISE controller is inside the **_traditional_Rise.m_** file, the file can be implemented directly in MATLAB.<br>
For assignment 2, the code for implementing a composite adaptive controller with a gradient update law is inside the **_Rise.m_** file, this file can be implemented directly in MATLAB too, but it may take longer time because the time step for ODE45 is small, and it takes much longer time to compute and retrieve the input &nbsp; <img src="https://latex.codecogs.com/svg.latex?\tau" title="\tau" /> &nbsp;.<br>

-----------------------
* ## Discussion
* **(a) Dynamic Model**<br>

For this simulation project, the dynamic model is of a two-link rigid revolute root manipulator given by:<br>

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\tau_{1}\\&space;\tau_{2}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;p_{1}&plus;2p_{3}c_{2}&space;&&space;p_{2}&plus;p_{3}c_{2}&space;\\&space;p_{2}&plus;p_{3}c_{2}&space;&&space;p_{2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\ddot{q}_{1}\\&space;\ddot{q}_{2}&space;\end{bmatrix}&plus;&space;\begin{bmatrix}&space;-p_{3}s_{2}\dot{q}_{2}&space;&-p_{3}s_{2}(\dot{q}_{1}&plus;\dot{q}_{2})&space;\\&space;p_{3}s_{2}\dot{q}_{1}&space;&0&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&plus;&space;\begin{bmatrix}&space;f_{d1}&space;&0&space;\\&space;0&space;&f_{d2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&plus;&space;\begin{bmatrix}&space;0.5cos(0.5t)\\&space;sin(t)&space;\end{bmatrix}" title="\begin{bmatrix} \tau_{1}\\ \tau_{2} \end{bmatrix} = \begin{bmatrix} p_{1}+2p_{3}c_{2} & p_{2}+p_{3}c_{2} \\ p_{2}+p_{3}c_{2} & p_{2} \end{bmatrix} \begin{bmatrix} \ddot{q}_{1}\\ \ddot{q}_{2} \end{bmatrix}+ \begin{bmatrix} -p_{3}s_{2}\dot{q}_{2} &-p_{3}s_{2}(\dot{q}_{1}+\dot{q}_{2}) \\ p_{3}s_{2}\dot{q}_{1} &0 \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}+ \begin{bmatrix} f_{d1} &0 \\ 0 &f_{d2} \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}+ \begin{bmatrix} 0.5cos(0.5t)\\ sin(t) \end{bmatrix}" /><br>

where &nbsp; <img src="https://latex.codecogs.com/svg.latex?p_{1}" title="p_{1}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?p_{2}" title="p_{2}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?p_{3}" title="p_{3}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?f_{d1}" title="f_{d1}" />&nbsp;&nbsp;<img src="https://latex.codecogs.com/svg.latex?f_{d2}" title="f_{d2}" />&nbsp; are unknown positive scalar constant,&nbsp;<img src="https://latex.codecogs.com/svg.latex?s_{2}=sin(q_{2})" title="s_{2}=sin(q_{2})" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?c_{2}=cos(q_{2})" title="c_{2}=cos(q_{2})" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\tau_{1}" title="\tau_{1}" />&nbsp; and <img src="https://latex.codecogs.com/svg.latex?\tau_{2}" title="\tau_{2}" />&nbsp;denote the control torque inputs on the first and second joint respectively. For the simulation, only&nbsp;<img src="https://latex.codecogs.com/svg.latex?q(t)" title="q(t)" />&nbsp;and&nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{q}(t)" title="\dot{q}(t)" />&nbsp;are assumed to be measurable.<br>

* **(b) Simulation Section**<br>
    * RISE controller<br>






