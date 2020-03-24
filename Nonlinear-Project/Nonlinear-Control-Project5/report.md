<div align =center>  

# EML 6351 Simulation Project 5<br>
## Implement a Repetitive Learning Controller (RLC)

<div align =left>

-------------------------

## README 
  
1. All the codes are inside **src** file:<br>
   * The **traditional.m** file is for implementing the standard gradient based adaptive update law;<br> 
    * The **RLC.m** file is for implementing the repetitive learning controller, and the **Sat.m** file is a function for implementing the saturation in the repetitive learning controller.<br> 
  
## Dicussion<br>
* ### **(A) Simulation Section**<br>
    * **Standard Gradient Based Adaptive Update Law**<br>
  
        **1. Control gains.** <br>

        Control gains: &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha&space;=&space;1.5" title="\alpha = 1.5" />&nbsp;, K = 5.<br> 
        Adaptation gains:<br>

        <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=&space;\begin{bmatrix}&space;10&space;&0&space;&0&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&1&space;&0&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&1&space;&0&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&10&space;&0&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&1&space;&0&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&0&space;&1&space;&0&space;\\&space;0&space;&0&space;&0&space;&0&space;&0&space;&0&space;&1&space;\end{bmatrix}" title="\gamma = \begin{bmatrix} 10 &0 &0 &0 &0 &0 &0 \\ 0 &1 &0 &0 &0 &0 &0 \\ 0 &0 &1 &0 &0 &0 &0 \\ 0 &0 &0 &10 &0 &0 &0 \\ 0 &0 &0 &0 &1 &0 &0 \\ 0 &0 &0 &0 &0 &1 &0 \\ 0 &0 &0 &0 &0 &0 &1 \end{bmatrix}" /><br>


        **2. Tracking error plot for each link.**<br>

        <img style="float: center;" src="examples/Tra_E.jpg"><br>

        **3. Control input plot for each link.**<br>

        <img style="float: center;" src="examples/Tra_Tau.jpg"><br>

        **4. Plot of the adaptive estimates.** <br>

        <img style="float: center;" src="examples/Tra_Estimate.jpg"><br>

        **5. Plot of the parameter estimate errors.** <br> 

        <img style="float: center;" src="examples/Tra_Estimate_Error.jpg"><br>

        There are 7 paramters in this case, and I take the root sum square of them &nbsp; <img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;\tilde&space;{\theta}&space;\right&space;\|" title="\left \| \tilde {\theta} \right \|" /> &nbsp;. The result is shown below:<br>

        <img style="float: center;" src="examples/Tra_Estimate_Error_L2norm.jpg"><br>
        <br>
        <br>

    * **Repetitive Learning Controller**<br>
        
        **1. Control gains.** <br>

        Control gains: &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha&space;=&space;1.5" title="\alpha = 1.5" />&nbsp;, K = 5,&nbsp;<img src="https://latex.codecogs.com/svg.latex?K_{n}&space;=&space;2" title="K_{n} = 2" />&nbsp;,&nbsp;<img src="https://latex.codecogs.com/svg.latex?K_{L}&space;=&space;1" title="K_{L} = 1" />&nbsp;.<br>
        Adaptation gains:<br>

        <img src="https://latex.codecogs.com/svg.latex?\gamma&space;=&space;\begin{bmatrix}&space;1&space;&0&space;\\&space;0&space;&1&space;\end{bmatrix}" title="\gamma = \begin{bmatrix} 1 &0 \\ 0 &1 \end{bmatrix}" /><br>

        **2. Tracking error plot for each link.**<br>

        <img style="float: center;" src="examples/RLC_E.jpg"><br>

        **3. Control input plot for each link.**<br>

        <img style="float: center;" src="examples/RLC_Tau.jpg"><br>

        **4. Plot of the adaptive estimates.** <br>

        <img style="float: center;" src="examples/RLC_Estimate.jpg"><br>


        **5. Plot of the parameter estimate errors.** <br> 

        <img style="float: center;" src="examples/RLC_Estimate_Error.jpg"><br>

        The root sum square is as follow:

        <img style="float: center;" src="examples/RLC_Estimate_Error_L2norm.jpg"><br>
        <br>
        <br>

* ### **(B) Discussion section**<br>

  * **Differences in tuning the control gains/adaptations**<br>  
    
    In my simulation, I found that increasing K or &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp; can both make tracking error decrease faster, but increase the input as tradeoff, and in addition, when tracking error decrease faster, the convergence of the parameters estimate error will slow down. To get a better performance, I think we have to make K bigger than &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp;. As, &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp; would bring in oscillation in tracking error, and K can suppress the oscillation.<br>

    For &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, it can control the speed of the parameter estimate error convergence. In general, we can increase them to get faster parameter estimate convergence, however, in my opinion, I think we should just tune the diagonal element &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, because it seems there are no reasons to add correlation in those parameter estimate, and we have to keep &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp; invertible.

    There are 2 more control gains in the repetitive learning controller,&nbsp;<img src="https://latex.codecogs.com/svg.latex?K_{L}" title="K_{L}" />&nbsp; and &nbsp;<img src="https://latex.codecogs.com/svg.latex?K_{n}" title="K_{n}" />&nbsp;. In Lyapunov analysis, we have:<br>

    <div align =center>  

    <img src="https://latex.codecogs.com/svg.latex?\dot{V}&space;\leq&space;(-min(\alpha,K)-\frac{1}{4K_{n}})\left&space;\|&space;Z&space;\right&space;\|^{2}-\frac{1}{2}K_{L}r^{t}r" title="\dot{V} \leq (-min(\alpha,K)-\frac{1}{4K_{n}})\left \| Z \right \|^{2}-\frac{1}{2}K_{L}r^{t}r" />

    <div align =left>  
    Where:&nbsp; <img src="https://latex.codecogs.com/svg.latex?Z&space;=&space;[e^{t},r^{t}]^{t}" title="Z = [e^{t},r^{t}]^{t}" />&nbsp;.<br>

    We know we should keep &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{n}" title="K_{n}" /> &nbsp; big enough and keep &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{L}" title="K_{L}" /> &nbsp; positive. In such way, <img src="https://latex.codecogs.com/svg.latex?\dot{V}" title="\dot{V}" /> will be negative except when Z = 0.

    When I twiddle &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{n}" title="K_{n}" /> &nbsp;, it seems it does not affect the performance. I think the reason would be the control gain K predominate &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{n}" title="K_{n}" /> &nbsp; in my case. I have set &nbsp;  <img src="https://latex.codecogs.com/svg.latex?\rho" title="\rho" /> &nbsp; to a constant in my case, so there will be no difference between K and &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{n}" title="K_{n}" /> &nbsp;. I try twiddling K, &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{n}" title="K_{n}" /> &nbsp;, and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp; all together. The conclusion is, the slower tracking error converge, the faster the parameter estimates can converge.

    <img src="https://latex.codecogs.com/svg.latex?K_{L}" title="K_{L}" /> &nbsp; is a control gain in &nbsp; <img src="https://latex.codecogs.com/svg.latex?\hat{W}" title="\hat{W}" /> &nbsp;. When increasing it, tracking error converge faster and the oscillation is suppressed, but the input becomes bigger. But its affect is limited because of the saturation function we defined. In mu opinion, the bigger &nbsp;  <img src="https://latex.codecogs.com/svg.latex?\beta" title="\beta" /> &nbsp; we set for the saturation the more affect &nbsp; <img src="https://latex.codecogs.com/svg.latex?K_{L}" title="K_{L}" /> &nbsp; can bring out.

  * **Performance of the tracking error for each controller**<br>  
    
    I compare the &nbsp; <img src="https://latex.codecogs.com/svg.latex?\left&space;\|&space;e&space;\right&space;\|" title="\left \| e \right \|" /> &nbsp; between the 2 controllers, the result is as follow:<br>

    <img style="float: center;" src="examples/Compare_E.jpg"><br>

    From the comparison, I can see that their performance of the tracking error is pretty much the same, the controller with standard gradient based adaptive update law has more smooth tracking error convergence. The amplitudes of the high frequency components are smaller for it during the steady state. Other than that, it seems two controllers are pretty similar in their performance of the tracking error.<br>

  * **Performance of the adaptation for each case**<br> 
    
    The parameters those two controllers try to estimate are different, I can only compare the parameters associated with static friction between those two controllers. In fact, both controllers use the same adaptation law for these two parameters, so I can assume that the performances are pretty similar, too. The result is shown below:<br>

    <img style="float: center;" src="examples/Compare_Estimate_Error.jpg"><br>

    From the comparison above, I found that the controller with standard gradient based adaptive update law performs better than the RLC controller, but they both can not converge to 0 as the Lyapunov analysis shows.<br>


  * **Control effort for each case**<br> 

    I compare the root sum square of the input between both cases, the result is as below:<br>

    <img style="float: center;" src="examples/Compare_Tau.jpg"><br>

    From the figure above, I think the control efforts for both case are similar too. Although RLC controller does not have too much information about the dynamics, it can pretty much achieve the same performance as the controller with standard gradient based adaptive update law in the tracking error, and the control effort they need are similar too.