# Nonlinear-Control-Project<br>
----------
##### Project 1  of EML6351 Nonlinear Control 2: Adaptive Control<br>
##### Taught by Prof. Warren Dixon<br>
----------
* The objective of this project is to design 3 controllers: traditional adaptive controller(with gradient adaptive update law), and 2 composite
adaptive controllers (with gradient adaptive update law and least squares adaptive update law), to control a two-link rigid revolute robot 
manipulator:<br>  

<img src="https://latex.codecogs.com/svg.latex?\inline&space;\begin{bmatrix}&space;\tau_{1}&space;\\&space;\tau_{2}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;p_{1}&plus;2p_{3}c_{2}&space;&&space;p_{2}&plus;p_{3}c_{2}&space;\\&space;p_{2}&plus;p_{3}c_{2}&space;&&space;p_{2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\ddot{q}_{1}\\&space;\ddot{q}_{2}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;-p_{3}s_{2}\dot{q}_{2}&space;&&space;-p_{3}s_{2}(\dot{q}_{1}&plus;\dot{q}_{2})&space;\\&space;p_{3}s_{2}\dot{q}_{1}&&space;0&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;f_{d1}&space;&&space;0&space;\\0&space;&&space;f_{d2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}" title="\begin{bmatrix} \tau_{1} \\ \tau_{2} \end{bmatrix} = \begin{bmatrix} p_{1}+2p_{3}c_{2} & p_{2}+p_{3}c_{2} \\ p_{2}+p_{3}c_{2} & p_{2} \end{bmatrix} \begin{bmatrix} \ddot{q}_{1}\\ \ddot{q}_{2} \end{bmatrix} + \begin{bmatrix} -p_{3}s_{2}\dot{q}_{2} & -p_{3}s_{2}(\dot{q}_{1}+\dot{q}_{2}) \\ p_{3}s_{2}\dot{q}_{1}& 0 \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix} + \begin{bmatrix} f_{d1} & 0 \\0 & f_{d2} \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}" /><br>
  
* For the project's development, please take a loop into my report.md.

* All the *.m file is independet, thus can be implemented by Matlab separately.
