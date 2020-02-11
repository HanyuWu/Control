# EML 6351 Simulation Project 2<br>

* ## README

All my codes are inside the **src** folder.<br>
For assignment 1, the code for implementing a RISE controller is inside the **_traditional_Rise.m_** file, the file can be implemented directly in MATLAB.<br>
For assignment 2, the code for implementing a composite adaptive controller with a gradient update law is inside the **_Rise.m_** file, this file can be implemented directly in MATLAB too, but it may take longer time because the time step for ODE45 is small, and it takes much longer time to compute and retrieve the input &nbsp; <img src="https://latex.codecogs.com/svg.latex?\tau" title="\tau" /> &nbsp;.<br>

-----------------------
* ## Discussion
* ### **(a) Dynamic Model**<br>

For this simulation project, the dynamic model is of a two-link rigid revolute root manipulator given by:<br>

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\tau_{1}\\&space;\tau_{2}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;p_{1}&plus;2p_{3}c_{2}&space;&&space;p_{2}&plus;p_{3}c_{2}&space;\\&space;p_{2}&plus;p_{3}c_{2}&space;&&space;p_{2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\ddot{q}_{1}\\&space;\ddot{q}_{2}&space;\end{bmatrix}&plus;&space;\begin{bmatrix}&space;-p_{3}s_{2}\dot{q}_{2}&space;&-p_{3}s_{2}(\dot{q}_{1}&plus;\dot{q}_{2})&space;\\&space;p_{3}s_{2}\dot{q}_{1}&space;&0&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&plus;&space;\begin{bmatrix}&space;f_{d1}&space;&0&space;\\&space;0&space;&f_{d2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&plus;&space;\begin{bmatrix}&space;0.5cos(0.5t)\\&space;sin(t)&space;\end{bmatrix}" title="\begin{bmatrix} \tau_{1}\\ \tau_{2} \end{bmatrix} = \begin{bmatrix} p_{1}+2p_{3}c_{2} & p_{2}+p_{3}c_{2} \\ p_{2}+p_{3}c_{2} & p_{2} \end{bmatrix} \begin{bmatrix} \ddot{q}_{1}\\ \ddot{q}_{2} \end{bmatrix}+ \begin{bmatrix} -p_{3}s_{2}\dot{q}_{2} &-p_{3}s_{2}(\dot{q}_{1}+\dot{q}_{2}) \\ p_{3}s_{2}\dot{q}_{1} &0 \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}+ \begin{bmatrix} f_{d1} &0 \\ 0 &f_{d2} \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}+ \begin{bmatrix} 0.5cos(0.5t)\\ sin(t) \end{bmatrix}" /><br>

where &nbsp; <img src="https://latex.codecogs.com/svg.latex?p_{1}" title="p_{1}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?p_{2}" title="p_{2}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?p_{3}" title="p_{3}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?f_{d1}" title="f_{d1}" />&nbsp;&nbsp;<img src="https://latex.codecogs.com/svg.latex?f_{d2}" title="f_{d2}" />&nbsp; are unknown positive scalar constant,&nbsp;<img src="https://latex.codecogs.com/svg.latex?s_{2}=sin(q_{2})" title="s_{2}=sin(q_{2})" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?c_{2}=cos(q_{2})" title="c_{2}=cos(q_{2})" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\tau_{1}" title="\tau_{1}" />&nbsp; and <img src="https://latex.codecogs.com/svg.latex?\tau_{2}" title="\tau_{2}" />&nbsp;denote the control torque inputs on the first and second joint respectively. For the simulation, only&nbsp;<img src="https://latex.codecogs.com/svg.latex?q(t)" title="q(t)" />&nbsp;and&nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{q}(t)" title="\dot{q}(t)" />&nbsp;are assumed to be measurable.<br>

* ### **(b) Simulation Section**<br>

  * **RISE controller**<br>

    **1. Control gains.** <br>

    K = 10, a1 = 1.5, a2 = 1.5 and &nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" />&nbsp; = 10.<br>

    **2. Tracking error plot for each link.**<br>

    <img style="float: center;" src="examples/onlyRISE_E.jpg"><br>

    **3. Control input plot for each link.** <br>

    <img style="float: center;" src="examples/onlyRISE_Tau.jpg"><br>

    **4. Plot of the adaptive estimates.**<br>
    **5. Plot of the parameter estimate errors**<br>
    Because we this time only implement the RISE controller without adaptive component, we don't have adaptive estimates here.<br>

  * **Composite adaptive controler with the additional RISE term**<br>

    **1. Control gains.**<br>

    K1 = 10, K2 = 25, &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha_{1}" title="\alpha_{1}" />&nbsp; = 1,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha_{2}" title="\alpha_{2}" />&nbsp;=3,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" />&nbsp; = 1, and<br>

    <img style = "float: center;" src="https://latex.codecogs.com/svg.latex?\gamma&space;=&space;\begin{bmatrix}&space;10&space;&&space;0&&space;0&&space;0&&space;0\\&space;0&space;&&space;1&&space;0&&space;0&0&space;\\&space;0&space;&&space;0&&space;1&&space;0&0&space;\\&space;0&space;&&space;0&&space;0&&space;10&0&space;\\&space;0&space;&&space;0&&space;0&&space;0&1&space;\end{bmatrix}" title="\gamma = \begin{bmatrix} 3 & 0& 0& 0& 0\\ 0 & 3& 0& 0&0 \\ 0 & 0& 1& 0&0 \\ 0 & 0& 0& 1&0 \\ 0 & 0& 0& 0&1 \end{bmatrix}" />&nbsp;.<br>

    **2. Tracking error plot for each link.**<br>

    <img style="float: center;" src="examples/RISE_E.jpg"><br>

    **3. Control input plot for each link.** <br>

    <img style="float: center;" src="examples/RISE_Tau.jpg"><br>

    **4. Plot of the adaptive estimates.**<br>

    <img style="float: center;" src="examples/RISE_ThetaHat_1.jpg"><br>
    <img style="float: center;" src="examples/RISE_ThetaHat_2.jpg"><br>

    **5. Plot of the parameter estimate errors**<br>

    <img style="float: center;" src="examples/RISE_TildeTheta.jpg"><br>

  * **Standard composite adaptive controller**<br>

    **1. Control gains.**<br>

    K = 10, &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp; = 2,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" />&nbsp; = 1, and<br>

    <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=&space;\begin{bmatrix}&space;30&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&5&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma = \begin{bmatrix} 30 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &5 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" />.<br>

     **2. Tracking error plot for each link.**<br>

     <img style="float: center;" src="examples/Composite_E.jpg"><br>

     **3. Control input plot for each link.** <br>

     <img style="float: center;" src="examples/Composite_Tau.jpg"><br>

     **4. Plot of the adaptive estimates.**<br>

     <img style="float: center;" src="examples/Composite_ThetaHat.jpg"><br>

     **5. Plot of the parameter estimate errors**<br>

     <img style="float: center;" src="examples/Composite_TildeTheta.jpg"><br>

* ### **(c) Discussion section**<br>

  * **Differences in tuning the control gains/adaptations**<br>

    For K1, K2 or K, when incresing those K, tracking error would decrease faser, but somehow the input would increase.<br>
    For &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha_{1}" title="\alpha_{1}" />&nbsp;, &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha_{2}" title="\alpha_{2}" />&nbsp; or &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp;, when increasing those &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp;, tracking error would decrease faster, but it somehow increases the input, and invokes bigger oscillation in the system.<br>
    For &nbsp; <img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" />&nbsp;, when we increse &nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" />&nbsp;, the tracking error would decrease faster, and the parameter estimate errors would dercrease faster as well.<br>
    For &nbsp; <img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, it can somehow impact the adaptive estimation, when we modify the specific components inside &nbsp; <img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, we can speed up some of the parameter estimations.

  * **Performance of the tracking error for each controller**<br>
    
    Below is the first channel of tracking error:<br>

    <img style="float: center;" src="examples/Compare_E1.jpg"><br>

    The second channel of the tracking error:<br>

    <img style="float: center;" src="examples/Compare_E2.jpg"><br>
    
    In my simulation, I set all the intialal error to the same values for the three controllers, and from the comparison of tracking error shown above, the RISE controller without Composite adaptive component has the best performance, and the composite adaptive controler with the additional RISE term is slightly outperform the standard composite adaptive controller.

  * **Performance of the adaptation for each case**<br>

    <div align =center><img src="examples/Compare_Composite_TildeTheta.jpg"><br>
    <center>Composite adaptive controller&nbsp;&nbsp;<img src="https://latex.codecogs.com/svg.latex?\tilde{\theta}" title="\tilde{\theta}" />&nbsp;</center><br>

    <img src="examples/Compare_RISE_TildeTheta.jpg"><br>
    <center>Composite adaptive controller with with the additional RISE &nbsp;&nbsp;<img src="https://latex.codecogs.com/svg.latex?\tilde{\theta}" title="\tilde{\theta}" />&nbsp;</center><br>
    <div align =left>
    The interuption&nbsp;<img src="https://latex.codecogs.com/svg.latex?\tau_{d}" title="\tau_{d}" />&nbsp; is small in this simulation. From the images above, I can conclude that in this situation, the standard composite adaptive controller has better performance of the adaptation.<br>

  * **For the above three discussion topics, be sure to compare and contrast the different results**<br>

    













