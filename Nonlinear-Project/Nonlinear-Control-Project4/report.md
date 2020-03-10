<div align =center>  

# EML 6351 Simulation Project 4<br>
## Implement a RISE-based modular controller

<div align =left>  

-------------------

* ## Simulate 4 different adaptive update law<br>
  
  To implemente 4 different adaptive update law, we have to moddify &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}" title="\dot{\hat{\theta}}" /> &nbsp; while &nbsp; <img src="https://latex.codecogs.com/svg.latex?\hat{\theta}" title="\hat{\theta}" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}" title="\dot{\hat{\theta}}" /> &nbsp; satisfy the below conditions:<br>

  <div align =center>  

  <img src="https://latex.codecogs.com/svg.latex?\hat{\theta}(t)&space;=&space;f_{1}(t)&plus;\Phi&space;(q,\dot{q},e_{1},e_{2},t)" title="\hat{\theta}(t) = f_{1}(t)+\Phi (q,\dot{q},e_{1},e_{2},t)" /><br>

  <div align =left>

  where:&nbsp;&nbsp;<img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;f_{1}(t)&space;\right&space;\|\leq&space;\gamma&space;_{1}" title="\left \| f_{1}(t) \right \|\leq \gamma _{1}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;\Phi&space;\right&space;\|\leq&space;\rho&space;_{1}(\left&space;\|&space;\bar{e}&space;\right&space;\|)\left&space;\|&space;\bar{e}&space;\right&space;\|" title="\left \| \Phi \right \|\leq \rho _{1}(\left \| \bar{e} \right \|)\left \| \bar{e} \right \|" />&nbsp;;<br>
  And&nbsp;<img src="https://latex.codecogs.com/svg.latex?\bar{e}=\left&space;[&space;e_{1}^{t},e_{2}^{t}&space;\right&space;]" title="\bar{e}=\left [ e_{1}^{t},e_{2}^{t} \right ]" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;\dot{f_{1}(t)}&space;\right&space;\|\leq&space;\gamma&space;_{2}&plus;\gamma&space;_{3}\left&space;\|&space;e_{1}&space;\right&space;\|&plus;\gamma&space;_{4}\left&space;\|&space;e_{2}&space;\right&space;\|&plus;\gamma&space;_{5}\left&space;\|&space;r&space;\right&space;\|" title="\left \| \dot{f_{1}(t)} \right \|\leq \gamma _{2}+\gamma _{3}\left \| e_{1} \right \|+\gamma _{4}\left \| e_{2} \right \|+\gamma _{5}\left \| r \right \|" />&nbsp;.<br>

  <div align =center>

  <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}(t)=g_{1}(t)&plus;\Omega&space;(q,\dot{q},e_{1},e_{2},r,t)" title="\dot{\hat{\theta}}(t)=g_{1}(t)+\Omega (q,\dot{q},e_{1},e_{2},r,t)" /><br>

  <div align =left>

  where:&nbsp;&nbsp;<img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;g_{1}&space;\right&space;\|\leq&space;\delta&space;_{1}" title="\left \| g_{1} \right \|\leq \delta _{1}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;\Omega&space;\right&space;\|\leq&space;\rho&space;_{2}(\left&space;\|&space;Z&space;\right&space;\|)\left&space;\|&space;Z&space;\right&space;\|" title="\left \| \Omega \right \|\leq \rho _{2}(\left \| Z \right \|)\left \| Z \right \|" />&nbsp;;<br>
  And&nbsp;<img src="https://latex.codecogs.com/svg.latex?Z=[e_{1}^{t},e_{2}^{t},r^{t}]" title="Z=[e_{1}^{t},e_{2}^{t},r^{t}]" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{{\left&space;\|&space;g_{1}(t)&space;\right&space;\|}}\leq&space;\delta&space;_{2}&plus;\delta&space;_{3}\left&space;\|&space;e_{1}&space;\right&space;\|&plus;\delta&space;_{4}\left&space;\|&space;e_{2}&space;\right&space;\|&plus;\delta&space;_{5}\left&space;\|&space;r&space;\right&space;\|" title="\dot{{\left \| g_{1}(t) \right \|}}\leq \delta _{2}+\delta _{3}\left \| e_{1} \right \|+\delta _{4}\left \| e_{2} \right \|+\delta _{5}\left \| r \right \|" />&nbsp;.<br>


  <div align =left>  
  
  All the simulations are implemented with MATLAB, the ***.m** file can be found in the **src** folder. 

-----------------------
* ## Discussion<br>
* ### **(A) Simulation Section**<br>
  * **Adaptive Update Law with &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}=cos(t)" title="\dot{\hat{\theta}}=cos(t)" /> &nbsp;**<br>
  
    **1. Control gains.** <br>  

    In this case, I simply use sinusoidal as &nbsp;<img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}" title="\dot{\hat{\theta}}" />&nbsp;.<br>

    Control gains: ks = 10,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{1}=3" title="\alpha _{1}=3" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{2}=4" title="\alpha _{2}=4" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta=1" title="\beta=1" />&nbsp;.<br>

    **2. Tracking error plot for each link.**<br>  

    <img style="float: center;" src="examples/sinusoidal_tracking.jpg"><br>  

    **3. Control input plot for each link.**<br>  

    <img style="float: center;" src="examples/sinusoidal_input.jpg"><br>

    **4. Plot of the adaptive estimates.** <br>  
    
    <img style="float: center;" src="examples/sinusoidal_estimates.jpg"><br>

    **5. Plot of the parameter estimate errors.** <br>  

    <img style="float: center;" src="examples/sinusoidal_estimate_error.jpg"><br>

  * **Adaptive Update Law with &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}=\gamma\cdot&space;\dot{Y_{d}}\cdot&space;r" title="\dot{\hat{\theta}}=\gamma\cdot \dot{Y_{d}}\cdot r" /> &nbsp;**<br>
  
    **1. Control gains.** <br> 

    In this case, I try to add some regulation terms to &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}" title="\dot{\hat{\theta}}" /> &nbsp; just like the standard adaptive update law.<br>

    Control gains: ks = 10,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{1}=2" title="\alpha _{1}=2" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{2}=3" title="\alpha _{2}=3" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta=1" title="\beta=1" />&nbsp;;<br>

    <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=\begin{bmatrix}&space;10&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&10&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma =\begin{bmatrix} 10 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &10 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" /><br>

    **2. Tracking error plot for each link.**<br>  

    <img style="float: center;" src="examples/YdR_tracking.jpg"/><br>  

    **3. Control input plot for each link.**<br>  

    <img style="float: center;" src="examples/YdR_input.jpg"/><br>

    **4. Plot of the adaptive estimates.** <br>  
    
    <img style="float: center;" src="examples/YdR_estimates.jpg"/><br>

    **5. Plot of the parameter estimate errors.** <br>  

    <img style="float: center;" src="examples/YdR_estimate_error.jpg"/><br>

  * **Adaptive Update Law with &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}=\gamma\cdot&space;\dot{Y_{d}}\cdot&space;(e_{1}&plus;e_{2}&plus;r)" title="\dot{\hat{\theta}}=\gamma\cdot \dot{Y_{d}}\cdot (e_{1}+e_{2}+r)" /> &nbsp;**<br>
  
    **1. Control gains.** <br>

    In this case, I put all the error terms into the adaptive law.<br>

    Control gains: ks = 10,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{1}=2" title="\alpha _{1}=2" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{2}=3" title="\alpha _{2}=3" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta=1" title="\beta=1" />&nbsp;;<br>

    <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=\begin{bmatrix}&space;10&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&10&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma =\begin{bmatrix} 10 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &10 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" /><br>

    **2. Tracking error plot for each link.**<br>  

    <img style="float: center;" src="examples/Yd_e1e2R_tracking.jpg"/><br> 

    **3. Control input plot for each link.**<br>  

    <img style="float: center;" src="examples/Yd_e1e2R_input.jpg"/><br>

    **4. Plot of the adaptive estimates.** <br>  
    
    <img style="float: center;" src="examples/Yd_e1e2R_estimates.jpg"/><br>

    **5. Plot of the parameter estimate errors.** <br>  

    <img style="float: center;" src="examples/Yd_e1e2R_estimate_error.jpg"/><br>

  * **Adaptive Update Law with &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}=\gamma\cdot&space;\dot{Y_{d}}\cdot&space;r&plus;\gamma&space;\cdot&space;\dot{Y_{df}}&space;\cdot&space;r" title="\dot{\hat{\theta}}=\gamma\cdot \dot{Y_{d}}\cdot r+\gamma \cdot \dot{Y_{df}} \cdot r" /> &nbsp;**<br>

    **1. Control gains.** <br>
      
    In this case, I utilize &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{Y_{df}}" title="\dot{Y_{df}}" /> &nbsp;, and see how it work.

    Control gains: ks = 10,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{1}=2" title="\alpha _{1}=2" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha&space;_{2}=3" title="\alpha _{2}=3" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta=1" title="\beta=1" />&nbsp;;<br>

    <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=\begin{bmatrix}&space;10&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&10&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma =\begin{bmatrix} 10 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 \\ 0 &0 &1 &0 &0 \\ 0 &0 &0 &10 &0 \\ 0 &0 &0 &0 &1 \end{bmatrix}" /><br>

    **2. Tracking error plot for each link.**<br>  

    <img style="float: center;" src="examples/Yd_Ydf_tracking.jpg"/><br>

    **3. Control input plot for each link.**<br>  

    <img style="float: center;" src="examples/Yd_Ydf_input.jpg"/><br>

    **4. Plot of the adaptive estimates.** <br>  
    
    <img style="float: center;" src="examples/Yd_Ydf_estimates.jpg"/><br>

    **5. Plot of the parameter estimate errors.** <br>  

    <img style="float: center;" src="examples/Yd_Ydf_estimate_error.jpg"/><br>

* ### **(B) Discussion section**<br>
  
  * **Differences in tuning the control gains/adaptations**<br>  
     
     As we are implementing a RISE-based modular controller, there could be many kinds of parameters can be twiddled in different kind of adaptation law. For my 4 implementation, there are mainly 5 gains/adaptations I can tune: &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha_{1}" title="\alpha_{1}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha_{2}" title="\alpha_{2}" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?k_{s}" title="k_{s}" />&nbsp;and &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;.<br>

     Take the standard adaptive update law for example:<br>

     <img src="https://latex.codecogs.com/svg.latex?k_{s}" title="k_{s}" /> &nbsp; is a term to provide &nbsp;<img src="https://latex.codecogs.com/svg.latex?-k_{s}&space;\cdot&space;r^{2}" title="-k_{s} \cdot r^{2}" /> &nbsp; in the lyapunov analysis, and thus &nbsp;<img src="https://latex.codecogs.com/svg.latex?k_{s}" title="k_{s}" />&nbsp; should be big enough to predominate other &nbsp; <img src="https://latex.codecogs.com/svg.latex?r^{2}" title="r^{2}" /> &nbsp; which may come from Young's inequality and etc. As we increse &nbsp; <img src="https://latex.codecogs.com/svg.latex?k_{s}" title="k_{s}" /> &nbsp; , the tracking error will converge faster, however, the input will increase and estimates error would converge much slower. I think that's a trade-off.<br>

     <img src="https://latex.codecogs.com/svg.latex?\alpha_{1}" title="\alpha_{1}" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha_{2}" title="\alpha_{2}" /> &nbsp; contribute the &nbsp; <img src="https://latex.codecogs.com/svg.latex?-\alpha_{1}e_{1}^{2}-\alpha_{2}e_{2}^{2}" title="-\alpha_{1}e_{1}^{2}-\alpha_{2}e_{2}^{2}" /> &nbsp; in the lyapunov analysis. When incresing &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha_{1}" title="\alpha_{1}" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha_{2}" title="\alpha_{2}" /> &nbsp;, we have to increse &nbsp; <img src="https://latex.codecogs.com/svg.latex?k_{s}" title="k_{s}" /> &nbsp; too, otherwise our simulation could break down. With bigger &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha_{1}" title="\alpha_{1}" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha_{2}" title="\alpha_{2}" /> &nbsp;, we will have  quicker convergence of both estimates error and tracking error. However, the input would increase.<br>

    Twiddling &nbsp; <img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" /> &nbsp; can actually increase the speed of estimate error convergence. In my implementation, I found when &nbsp; <img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" /> &nbsp; is small, then the estimate error would converge faster.<br>

    In my implementation, &nbsp; <img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" /> &nbsp; is a matrix. It's a learning rate hyperparameter. When we modify the specific element in this matrix, we can increse or decrease the corresponding parameter's estimation estimates' changing rates.<br>

  * **Performance of the tracking error for each controller**<br>  
    
    I compared the L2-norm of each controller's tracking error,&nbsp; <img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;e&space;\right&space;\|" title="\left \| e \right \|" />&nbsp;, as shown below:<br>

    <img style="float: center;" src="examples/tracking_compared.jpg"/><br>

    As I expected, all the contoller has good performance on the tracking error as RISE-based modular controller design is designed to achieve asymptotic tracking after all. In fact, the ones with DCAL and only &nbsp; <img src="https://latex.codecogs.com/svg.latex?r" title="r" /> &nbsp; in it has better performance, the one uses random sinusoidal as adaptive update law has slightly worse performance (blue), and the one uses not only &nbsp; <img src="https://latex.codecogs.com/svg.latex?r" title="r" /> &nbsp; but also &nbsp; <img src="https://latex.codecogs.com/svg.latex?e_{1}" title="e_{1}" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?e_{2}" title="e_{2}" /> &nbsp; has the worst performance (yellow).<br>

  * **Performance of the adaptation for each case**<br> 

    I compared the L2-norm of each controller's parameter estimation error,&nbsp; <img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;\tilde{\theta}&space;\right&space;\|" title="\left \| \tilde{\theta} \right \|" /> &nbsp;, as shown below:<br>

    <img style="float: center;" src="examples/estimation_compared.jpg"/><br>

    Among my 4 controllers, the one uses composite adaptive update structure has the best performance. However, I have not used prediction error in this contoller. The one use tradional gradient update law has slightly worse performance, but if we put all the error terms along with the DCAL term, the performance is worst.<br>

    From the result of adaptation and tracking error. I can see that in RISE-based modular controller, the adaptation and tracking error performance have positive correlation. The better adaptation performance we have, the better tracking performance our controller can achieve.<br>

    I think the modular method give us the possibility to twiddle our controller like PID controller, and the way we modify the adaptation law can make a huge difference in performance. The RISE-based method also get us the idea to put &nbsp; <img src="https://latex.codecogs.com/svg.latex?r" title="r" /> &nbsp; in the &nbsp; <img src="https://latex.codecogs.com/svg.latex?\dot{\hat{\theta}}" title="\dot{\hat{\theta}}" /> &nbsp; .<br>

    Things can be improved: I think I can compare my controllers to the composite adaptive controller with RISE-term in **Project 2**, and see if I can design a better controller using the modular method.








    
  