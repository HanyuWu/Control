# EML 6351 Simulation Project 1

* ### **Introcuction** 
In this project, we need to design our own traditional adaptive controller (with gradient adaptive update law), and 2 composite adaptive controllers (with gradient adaptive update law and least squares adaptive update law) for a two-link rigid revolute robot manipulator, to make sure the actual angular position of each joints can keep track of the desired angular position.  
The robot manipulator model is as below:

<img src="https://latex.codecogs.com/svg.latex?\inline&space;\begin{bmatrix}&space;\tau_{1}&space;\\&space;\tau_{2}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;p_{1}&plus;2p_{3}c_{2}&space;&&space;p_{2}&plus;p_{3}c_{2}&space;\\&space;p_{2}&plus;p_{3}c_{2}&space;&&space;p_{2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\ddot{q}_{1}\\&space;\ddot{q}_{2}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;-p_{3}s_{2}\dot{q}_{2}&space;&&space;-p_{3}s_{2}(\dot{q}_{1}&plus;\dot{q}_{2})&space;\\&space;p_{3}s_{2}\dot{q}_{1}&&space;0&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;f_{d1}&space;&&space;0&space;\\0&space;&&space;f_{d2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}" title="\begin{bmatrix} \tau_{1} \\ \tau_{2} \end{bmatrix} = \begin{bmatrix} p_{1}+2p_{3}c_{2} & p_{2}+p_{3}c_{2} \\ p_{2}+p_{3}c_{2} & p_{2} \end{bmatrix} \begin{bmatrix} \ddot{q}_{1}\\ \ddot{q}_{2} \end{bmatrix} + \begin{bmatrix} -p_{3}s_{2}\dot{q}_{2} & -p_{3}s_{2}(\dot{q}_{1}+\dot{q}_{2}) \\ p_{3}s_{2}\dot{q}_{1}& 0 \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix} + \begin{bmatrix} f_{d1} & 0 \\0 & f_{d2} \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}" />

where: 

<img src="https://latex.codecogs.com/svg.latex?\inline&space;q_{d1}=cos(0.5t)" title="q_{d1}=cos(0.5t)" /> <br>
<img src="https://latex.codecogs.com/svg.latex?\inline&space;q_{d2}=2cos(t)" title="q_{d2}=2cos(t)" />  

* ### **Derive the filter torque <img src="https://latex.codecogs.com/svg.latex?\tau_{f}(t)" title="\tau_{f}(t)" />, the filtered regression matrix <img src="https://latex.codecogs.com/svg.latex?Y_{f}(t)" title="Y_{f}(t)" /> and the filtered desired regression matrix <img src="https://latex.codecogs.com/svg.latex?Y_{df}(t)" title="Y_{df}(t)" />.**
#### 1. Filtered torque <img src="https://latex.codecogs.com/svg.latex?\tau_{f}(t)" title="\tau_{f}(t)" />.  
   
<img src="https://latex.codecogs.com/svg.latex?\tau" title="\tau" /> is known and designed by us, and <img src="https://latex.codecogs.com/svg.latex?\tau_{f}(t)" title="\tau_{f}(t)" /> is f convolved with <img src="https://latex.codecogs.com/svg.latex?\tau" title="\tau" />.  f designed as <img src="https://latex.codecogs.com/svg.latex?f&space;=&space;\beta&space;e^{-\beta&space;t}" title="f = \beta e^{-\beta t}" />. After Laplacian transform, we will have <img src="https://latex.codecogs.com/svg.latex?\dot{\tau}_{f}=\beta*\tau-\beta*\tau_{f}" title="\dot{\tau}_{f}=\beta*\tau-\beta*\tau_{f}" />.  

#### 2. Filtered regression matrix <img src="https://latex.codecogs.com/svg.latex?Y_{f}(t)" title="Y_{f}(t)" />.  

First, we have &nbsp; <img src="https://latex.codecogs.com/svg.latex?Y_{f}\theta&space;=&space;f&space;\ast&space;m(q)\ddot{q}&space;&plus;&space;f\ast&space;N(q,\dot{q})" title="Y_{f}\theta = f \ast m(q)\ddot{q} + f\ast N(q,\dot{q})" />, and after some mathematics processes, we can derive them into a form as below:  

<img src="https://latex.codecogs.com/svg.latex?Y_{f}\theta&space;=&space;\Omega&space;&plus;&space;\beta&space;m(q(t))\dot{q}(t)-\beta&space;e^{-\beta&space;t}m(q(0))\dot{q}(0)&plus;W" title="Y_{f}\theta = \Omega + \beta m(q(t))\dot{q}(t)-\beta e^{-\beta t}m(q(0))\dot{q}(0)+W" />  

with &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\Omega}&space;=&space;-\beta&space;\Omega-\beta^{2}m(q(t))\dot{q}(t)" title="\dot{\Omega} = -\beta \Omega-\beta^{2}m(q(t))\dot{q}(t)" />, and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{W}(t)&space;=&space;-\beta&space;W(t)&space;&plus;&space;\beta(-\dot{m}(q(t))\dot{q}(t)&plus;N(q(t),\dot{q}(t))" title="\dot{W}(t) = -\beta W(t) + \beta(-\dot{m}(q(t))\dot{q}(t)+N(q(t),\dot{q}(t))" />. After initialize  <img src="https://latex.codecogs.com/svg.latex?\Omega" title="\Omega" /> and <img src="https://latex.codecogs.com/svg.latex?W" title="W" />, we can get <img src="https://latex.codecogs.com/svg.latex?Y_{f}\theta" title="Y_{f}\theta" />, and factor out the theta, we can have <img src="https://latex.codecogs.com/svg.latex?Y_{f}" title="Y_{f}" />.

#### 3. filtered desired regression matrix <img src="https://latex.codecogs.com/svg.latex?Y_{df}(t)" title="Y_{df}(t)" />.

We can get <img src="https://latex.codecogs.com/svg.latex?Y_{df}(t)" title="Y_{df}(t)" /> as follow:  

<img src="https://latex.codecogs.com/svg.latex?Y_{df}&space;=&space;f&space;\ast&space;Y_{d}" title="Y_{df} = f \ast Y_{d}" />  <br>
<img src="https://latex.codecogs.com/svg.latex?\dot{Y_{df}}&space;=&space;\beta*Y_{d}-\beta*Y_{df}" title="\dot{Y_{df}} = \beta*Y_{d}-\beta*Y_{df}" /> <br>
Where <img src="https://latex.codecogs.com/svg.latex?Y_{d}" title="Y_{d}" /> &nbsp;   is derived beforehand:  

<img src="https://latex.codecogs.com/svg.latex?Y_{d11}&space;=&space;\ddot{q_{d1}}" title="Y_{d11} = \ddot{q_{d1}}" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d12}&space;=&space;\ddot{q_{d2}}" title="Y_{d12} = \ddot{q_{d2}}" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d13}&space;=&space;2cos(d_{2})\ddot{q_{d1}}&plus;cos(d_{2})\ddot{q_{d2}}-sin(d_{2})(\dot{q_{d1}}&plus;\dot{q_{d2}})(\dot{q_{d2}})" title="Y_{d13} = 2cos(d_{2})\ddot{q_{d1}}+cos(d_{2})\ddot{q_{d2}}-sin(d_{2})(\dot{q_{d1}}+\dot{q_{d2}})(\dot{q_{d2}})" /> <br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d14}&space;=&space;\dot{q_{d1}}" title="Y_{d14} = \dot{q_{d1}}" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d15}&space;=&space;0" title="Y_{d15} = 0" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d21}&space;=&space;0" title="Y_{d21} = 0" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d22}&space;=&space;\ddot{q_{d1}}&plus;\ddot{q_{d2}}" title="Y_{d22} = \ddot{q_{d1}}+\ddot{q_{d2}}" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d23}&space;=&space;cos(q_{d2})\ddot{q_{d1}}&plus;sin(q_{d2})\dot{q_{d1}}^{2}" title="Y_{d23} = cos(q_{d2})\ddot{q_{d1}}+sin(q_{d2})\dot{q_{d1}}^{2}" /> <br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d24}&space;=&space;0" title="Y_{d24} = 0" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d25}&space;=\dot{q_{d2}}" title="Y_{d25} =\dot{q_{d2}}" /><br>
<img src="https://latex.codecogs.com/svg.latex?Y_{d}&space;=&space;\begin{bmatrix}&space;Y_{d11}&space;&Y_{d12}&space;&Y_{d13}&space;&Y_{d14}&space;&Y_{d15}&space;\\&space;Y_{d21}&space;&Y_{d22}&space;&Y_{d23}&space;&Y_{d24}&space;&Y_{d25}&space;\end{bmatrix}" title="Y_{d} = \begin{bmatrix} Y_{d11} &Y_{d12} &Y_{d13} &Y_{d14} &Y_{d15} \\ Y_{d21} &Y_{d22} &Y_{d23} &Y_{d24} &Y_{d25} \end{bmatrix}" /><br>
<img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" /> &nbsp; is designed by us, so we can intergrate <img src="https://latex.codecogs.com/svg.latex?\dot{Y_{df}}" title="\dot{Y_{df}}" /> &nbsp; with initilization to get <img src="https://latex.codecogs.com/svg.latex?Y_{df}(t)" title="Y_{df}(t)" />.  

* ### **Derive and simulate the controllers**

**(a) Traditional adaptive controller with gredient adaptive update law.**  

The code is included in 'traditional.m', and to implement it, just hit run button or implement the traditional function in command window.  

**(b) Composite adaptive controller with gredient adaptive update law.**

The code is included in 'torquefiltermethod.m', and to implement it, just hit run button or implement the torquefiltermethod function in command window. 

**(c) Composite adaptive controller with least squares adaptive update law.**

The code is included in 'leastsquaremethod.m', and to implement it, just hit run button or implement the leastsquaremethod function in command window. 

**(d) turn in the code**
All the codes are in src file and can be implemented by themselves in Matlab (no dependent needed).

-------------------------------
* ### **Typed report**
  
**(a) Dynamics model.**  

The dynamics model is:

<img src="https://latex.codecogs.com/svg.latex?\inline&space;\begin{bmatrix}&space;\tau_{1}&space;\\&space;\tau_{2}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;p_{1}&plus;2p_{3}c_{2}&space;&&space;p_{2}&plus;p_{3}c_{2}&space;\\&space;p_{2}&plus;p_{3}c_{2}&space;&&space;p_{2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\ddot{q}_{1}\\&space;\ddot{q}_{2}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;-p_{3}s_{2}\dot{q}_{2}&space;&&space;-p_{3}s_{2}(\dot{q}_{1}&plus;\dot{q}_{2})&space;\\&space;p_{3}s_{2}\dot{q}_{1}&&space;0&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;f_{d1}&space;&&space;0&space;\\0&space;&&space;f_{d2}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{q}_{1}\\&space;\dot{q}_{2}&space;\end{bmatrix}" title="\begin{bmatrix} \tau_{1} \\ \tau_{2} \end{bmatrix} = \begin{bmatrix} p_{1}+2p_{3}c_{2} & p_{2}+p_{3}c_{2} \\ p_{2}+p_{3}c_{2} & p_{2} \end{bmatrix} \begin{bmatrix} \ddot{q}_{1}\\ \ddot{q}_{2} \end{bmatrix} + \begin{bmatrix} -p_{3}s_{2}\dot{q}_{2} & -p_{3}s_{2}(\dot{q}_{1}+\dot{q}_{2}) \\ p_{3}s_{2}\dot{q}_{1}& 0 \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix} + \begin{bmatrix} f_{d1} & 0 \\0 & f_{d2} \end{bmatrix} \begin{bmatrix} \dot{q}_{1}\\ \dot{q}_{2} \end{bmatrix}" />

where &nbsp; <img src="https://latex.codecogs.com/svg.latex?p_{1},p_{2},p_{3},f_{d1},f_{d2}" title="p_{1},p_{2},p_{3},f_{d1},f_{d2}" />&nbsp;are unknown positive scalar constans, and &nbsp; <img src="https://latex.codecogs.com/svg.latex?s_{2}=sin(q_{2}),c_{2}=cos(q_{2})" title="s_{2}=sin(q_{2}),c_{2}=cos(q_{2})" />, and:  
<img src="https://latex.codecogs.com/svg.latex?\inline&space;q_{d1}=cos(0.5t)" title="q_{d1}=cos(0.5t)" /> <br>
<img src="https://latex.codecogs.com/svg.latex?\inline&space;q_{d2}=2cos(t)" title="q_{d2}=2cos(t)" />  <br>

**(b) Problem definition and open-loop error system development.**  

The project's objective is to let &nbsp; <img src="https://latex.codecogs.com/svg.latex?q" title="q" />&nbsp; keep track of &nbsp;<img src="https://latex.codecogs.com/svg.latex?q_{d}" title="q_{d}" />.  

**(c) Control design (including adaptive update law) and closed-loop error system development**  

This dynamics model's states are up to sescond order derivative, so we not only design e as &nbsp; <img src="https://latex.codecogs.com/svg.latex?e&space;=&space;q&space;-&space;q_{d}" title="e = q - q_{d}" />, we also should have a second term r as &nbsp; <img src="https://latex.codecogs.com/svg.latex?r=\alpha&space;*e&plus;\dot{e}" title="r=\alpha *e+\dot{e}" />, and when r goes to 0, we can also have e goes to 0. With r, we can have &nbsp; <img src="https://latex.codecogs.com/svg.latex?\ddot{q}" title="\ddot{q}" /> in &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{V}" title="\dot{V}" />, thus put &nbsp; <img src="https://latex.codecogs.com/svg.latex?\tau" title="\tau" /> &nbsp; into lyapunov analysis. For adapative update law, we should have a term &nbsp; <img src="https://latex.codecogs.com/svg.latex?\tilde{\theta}" title="\tilde{\theta}" /> &nbsp; in  &nbsp; <img src="https://latex.codecogs.com/svg.latex?V" title="V" /> &nbsp;, so when can at least bound &nbsp; <img src="https://latex.codecogs.com/svg.latex?\tilde{\theta}" title="\tilde{\theta}" /> &nbsp; after our lyapunov analysis.

**(d) Stability analysis of each controller.**  

For all the 3 controllers, we have the same Lyapunov function:<br>

<img src="https://latex.codecogs.com/svg.latex?V=\frac{1}{2}r^{t}mr&plus;\frac{1}{2}e^{t}e&plus;\frac{1}{2}\tilde{\theta}^{t}\gamma&space;^{-1}\tilde{\theta}" title="V=\frac{1}{2}r^{t}mr+\frac{1}{2}e^{t}e+\frac{1}{2}\tilde{\theta}^{t}\gamma ^{-1}\tilde{\theta}" /><br>

* For the traditional controllers, we design:<br>

<img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}=&space;-&space;\gamma&space;*Y_{d}^{t}*r" title="\dot{\hat{\theta}}= - \gamma *Y_{d}^{t}*r" /><br>
<img src="https://latex.codecogs.com/svg.latex?\tau=-K*r&plus;Y_{d}*\hat{\theta}" title="\tau=-K*r+Y_{d}*\hat{\theta}" /><br>

With doing so, we can finally have:<br>

<img src="https://latex.codecogs.com/svg.latex?\dot{V}=r^{t}N-\alpha&space;e^{2}-kr^{2}" title="\dot{V}=r^{t}N-\alpha e^{2}-kr^{2}" /><br>

where &nbsp; <img src="https://latex.codecogs.com/svg.latex?N&space;=\rho\left&space;\|&space;Z&space;\right&space;\|" title="N =\rho\left \| Z \right \|" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\rho" title="P" /> &nbsp; is a function of &nbsp; <img src="https://latex.codecogs.com/svg.latex?Z" title="Z" />&nbsp; with &nbsp; <img src="https://latex.codecogs.com/svg.latex?Z&space;=&space;[r;e]" title="Z = [r;e]" /> &nbsp;, specifically we can bound the parameters in this function by Mean Value Theorem. With tuning &nbsp; 
<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?k" title="k" /> &nbsp;, we can have:<br> 

<img src="https://latex.codecogs.com/svg.latex?\dot{V}\leqslant&space;-\varphi&space;\left&space;\|&space;Z&space;\right&space;\|^{2}" title="\dot{V}\leqslant -\varphi \left \| Z \right \|^{2}" /><br>

In the end, we can prove that &nbsp; <img src="https://latex.codecogs.com/svg.latex?r" title="r" />r &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?e" title="e" /> &nbsp; go to 0 with Lemma.

* For Composite adaptive controller with gredient adaptive update law, we design:<br>

<img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}&space;=&space;-\gamma&space;*Y_{d}^{t}*r&space;&plus;&space;\gamma&space;*Y_{df}^{t}*\epsilon" title="\dot{\hat{\theta}} = -\gamma *Y_{d}^{t}*r + \gamma *Y_{df}^{t}*\epsilon" /><br>
<img src="https://latex.codecogs.com/svg.latex?\epsilon&space;=\tau_{f}-Y_{df}*\hat{\theta}" title="\epsilon =\tau_{f}-Y_{df}*\hat{\theta}" /><br>

After deriving &nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{V}" title="\dot{V}" />&nbsp;, We can have a similar form like the traditional method, but we will have one more term as follow in the &nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{V}" title="\dot{V}" />&nbsp; and that is &nbsp;<img src="https://latex.codecogs.com/svg.latex?-(1-\frac{\beta*C}{2})\left&space;\|&space;Y_{df}\hat{\theta}&space;\right&space;\|^{2}" title="-(1-\frac{\beta*C}{2})\left \| Y_{df}\theta \right \|^{2}" />&nbsp;, and after tuning &nbsp;<img src="https://latex.codecogs.com/svg.latex?\Beta" title="\Beta" />&nbsp; we have make sure this term is nagative, thus we can have the same result like the tradition method, but we can now also have &nbsp; <img src="https://latex.codecogs.com/svg.latex?Y_{df}\hat{\theta}" title="Y_{df}\hat{\theta}" />&nbsp; is of L2 bound.
 
* For Composite adaptive controller with least squares adaptive update law, we design:<br>

<img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}&=-P*Y_{d}^{t}*r&plus;P*Y_{df}^{t}*\epsilon" title="\dot{\hat{\theta}}&=-P*Y_{d}^{t}*r+P*Y_{df}^{t}*\epsilon" /><br>
<img src="https://latex.codecogs.com/svg.latex?\epsilon&space;=\tau_{f}-Y_{df}*\hat{\theta}" title="\epsilon =\tau_{f}-Y_{df}*\hat{\theta}" /><br>
<img src="https://latex.codecogs.com/svg.latex?\dot{P}=-P*Y_{df}^{t}*Y_{df}*P" title="\dot{P}=-P*Y_{df}^{t}*Y_{df}*P" /><br>
where we initialize &nbsp;<img src="https://latex.codecogs.com/svg.latex?P" title="P" />&nbsp; as a postive definite matrix.  

After deriving &nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{V}" title="\dot{V}" />&nbsp;, we now have the very similar form of the composite adaptive controller with gredient adaptive update law method, and the &nbsp; <img src="https://latex.codecogs.com/svg.latex?1" title="1" /> &nbsp; in &nbsp;<img src="https://latex.codecogs.com/svg.latex?-(1-\frac{\beta*C}{2})\left&space;\|&space;Y_{df}\hat{\theta}&space;\right&space;\|^{2}" title="-(1-\frac{\beta*C}{2})\left \| Y_{df}\theta \right \|^{2}" />&nbsp; term will become &nbsp; <img src="https://latex.codecogs.com/svg.latex?\frac{1}{2}" title="\frac{1}{2}" />&nbsp;. So we will have the same conclusion as the last one.

**(e) Stability analysis of each controller.**  

**For the traditional controllers:**<br>
* Control gain and their value:<br> 
My control gain K = 5, <img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" /> = 1.5, and<br>and<br> <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=&space;\begin{bmatrix}&space;1&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&1&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma = \begin{bmatrix} 1 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &1 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" />.
* Tracking error plot for each link:<br> 
For e:<br>
![Alt text](./examples/trackingerror.jpg "1")<br>
* Control input plot<br>
![Alt text](./examples/tau.jpg "a1")<br>
* Plot of adaptive estimates<br>
![Alt text](./examples/adaptiveestimate.jpg "2")<br>
* Plot of the parameter estimate errors(<img src="https://latex.codecogs.com/svg.latex?\hat{\theta}" title="\hat{\theta}" />)<br>
![Alt text](./examples/tilde.jpg "3")<br>

**For the Composite adaptive controller with gredient adaptive update law:**<br>
* Control gain and their value:<br> 
As for comparison, I use the same control gain as K = 5, <img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" /> = 1.5, <img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" /> = 1, and<br> <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=&space;\begin{bmatrix}&space;1&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&1&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma = \begin{bmatrix} 1 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &1 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" />.
* Tracking error plot for each link:<br> 
For e:<br>
![Alt text](./examples/trackingerror2.jpg "4")<br>
* Control input plot<br>
![Alt text](./examples/tau2.jpg "a1")<br>
* Plot of adaptive estimates<br>
![Alt text](./examples/adaptiveestimate2.jpg "5")<br>
* Plot of the parameter estimate errors(<img src="https://latex.codecogs.com/svg.latex?\hat{\theta}" title="\hat{\theta}" />)<br>
![Alt text](./examples/tilde2.jpg "6")<br>

**For the Composite adaptive controller with least squares adaptive update law:**<br>
* Control gain and their value:<br> 
As for comparison, I use the same control gain as K = 5, <img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" /> = 1.5, <img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" /> = 1, and<br> <img src="https://latex.codecogs.com/svg.latex?P0&space;=&space;\begin{bmatrix}&space;1&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&1&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="P0 = \begin{bmatrix} 1 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &1 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" />.
* Tracking error plot for each link:<br> 
For e:<br>
![Alt text](./examples/trackingerror3.jpg "7")<br>
* Control input plot<br>
![Alt text](./examples/tau3.jpg "a1")<br>
* Plot of adaptive estimates<br>
![Alt text](./examples/adaptiveestimate3.jpg "8")<br>
* Plot of the parameter estimate errors(<img src="https://latex.codecogs.com/svg.latex?\hat{\theta}" title="\hat{\theta}" />)<br>
![Alt text](./examples/tilde3.jpg "9")<br>

**(f) Discution section:**

**1. Differences in tunning the control gains/adaptation gains.**<br>
When increasing control gain K, we can clearly notice that the tracking error would converge faster, but it slow down the convergence of parameter estimate errors, and when increasing adaptation gains &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" /> &nbsp;, tracking error would converge faster, and it may slow down the parameter estimate errors, I can see from my experiment that it suppress the overshoot of parameter estimate errors. When incresing the eigenvalue of the &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, it increase the convergence speed of both the tracking error and the parameter estimate errors, but I think it would increse the input value. 

**2. Performance of the tracking error for each controller.**<br>
I extract the first joint's tracking error from the 3 controller:<br>
![Alt text](./examples/trackingcompare.jpg "10")<br>
Of the same control gain, I can see the traditional controller and the Composite adaptive controller with gredient adaptive update law have the similar performance, and the the Composite adaptive controller with least squares adaptive update law is not as good as the first two. However, overall their performance are not quite different.

**3. Performance of the adaptation for each case.**<br>
![Alt text](./examples/parametererror.jpg "11")<br>
For adaptation, the two composite controller both outperform the traditional one, but I can't see which one of these two can perform better that the other one. I think it's because they have different adaptation parameter as one using <img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />, and the other one has P0 which is initialized by us.

