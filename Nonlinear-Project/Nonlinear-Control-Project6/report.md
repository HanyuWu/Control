<div align =center>  

# EML 6351 Simulation Project 6<br>
## Implement Neural Network-based controller

<div align =left>

-------------------------

## README 
  
1. All the codes are inside **src** file:<br>
   * The [**Neural_Networks_Continuous.m**](./src/Model_Composite_2Link_NN_Continuous.slx) file is for implementing with a neural network-based controller with a typical continuous feedback control law for the dynamics, along with the **Model_Composite_2Link_NN_Continuous.slx** file.<br>
   * The [**Neural_Networks.m**](./src/Model_Composite_2Link_NN.slx) file is for implementing with a neural network-based controller with a discontinuous sliding-mode feedback control law for the dynamics, along with the **Model_Composite_2Link_NN.slx** file.<br>
   *  The [**Neural_Networks_RISE.m**](./src/Model_Composite_2Link_NN_RISE.slx) file is for implementing with a neural network-based controller with a continuous RISE feed-back control law for the dynamics, along with the **Model_Composite_2Link_NN_RISE.slx** file.<br>
  
## Dicussion<br>
* ### **(A) Simulation Section**<br>
    * **neural network-based controller with a typical continuous feedback control law for the dynamics**<br>

        **1. Control gains.** <br>

        Control gains: &nbsp; <img src="https://latex.codecogs.com/svg.latex?k&space;=&space;50,&space;k_{n}&space;=&space;10;" title="k = 50, k_{n} = 10" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha&space;=&space;10" title="\alpha = 10" /> &nbsp;.<br>
        Neural Network learning rate: &nbsp; <img src="https://latex.codecogs.com/svg.latex?\gamma_{1}&space;=&space;5,&space;\gamma_{2}&space;=&space;10" title="\gamma_{1} = 5, \gamma_{2} = 10" /> &nbsp;.<br>

        **2. Tracking error plot for each link.**<br>

        <img style="float: center;" src="examples/NN_Continuous/Link_1_Position_Errors_sm.png"><br>
        <img style="float: center;" src="examples/NN_Continuous/Link_2_Position_Errors_sm.png"><br>

        Root Sum Squared of error is as below: 
        <img style="float: center;" src="examples/NN_Continuous/RSS_Error.jpg"><br>

        **3. Control input plot for each link.**<br>

        <img style="float: center;" src="examples/NN_Continuous/Link_1_Control_sm.png"><br>
        <img style="float: center;" src="examples/NN_Continuous/Link_2_Control_sm.png"><br>

        **4. Plot of the adaptive estimates.** <br>

        In this project, we don't explicitly estimate the parameters in dynamics. We use a FCNN architecture to estimate the dynamics in the system as shown below:<br>

        <img style="float: center;" src="examples/fcnn.png"><br>

        We use gaussian activation in the hidden layer, and no activation in the output layer.

        <img style="float: center;" src="examples/NN_Continuous/What.jpg"><br>
        <img style="float: center;" src="examples/NN_Continuous/Vhat.jpg"><br>

        **5. Plot of the parameter estimate errors.** <br>

        In this project, we plot the errors between the FCNN output and the ground truth.<br>

        <img style="float: center;" src="examples/NN_Continuous/Link_1_Parameter_Estimate_Errors_sm.png"><br>
        <img style="float: center;" src="examples/NN_Continuous/Link_2_Parameter_Estimate_Errors_sm.png"><br>
        <br>
        <br>
    
    * **neural network-based controller with a discontinuous sliding-mode feedback control law for the dynamics**<br>

        **1. Control gains.** <br>

        Control gains: &nbsp; <img src="https://latex.codecogs.com/svg.latex?k&space;=&space;50,&space;k_{n}&space;=&space;20,&space;k_{s}&space;=&space;28" title="k = 50, k_{n} = 20, k_{s} = 28" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha&space;=&space;10" title="\alpha = 10" /> &nbsp;.<br>
        Neural Network learning rate: &nbsp; <img src="https://latex.codecogs.com/svg.latex?\gamma_{1}&space;=&space;5,&space;\gamma_{2}&space;=&space;10" title="\gamma_{1} = 5, \gamma_{2} = 10" /> &nbsp;.<br>

        **2. Tracking error plot for each link.**<br>

        <img style="float: center;" src="examples/NN_SLIDING/Link_1_Position_Errors_sm.png"><br>
        <img style="float: center;" src="examples/NN_SLIDING/Link_2_Position_Errors_sm.png"><br>

        Root Sum Squared of error is as below: 
        <img style="float: center;" src="examples/NN_SLIDING/RSS_Error.jpg"><br>

        **3. Control input plot for each link.**<br>

        <img style="float: center;" src="examples/NN_SLIDING/Link_1_Control_sm.png"><br>
        <img style="float: center;" src="examples/NN_SLIDING/Link_2_Control_sm.png"><br>

        **4. Plot of the adaptive estimates.** <br>

        <img style="float: center;" src="examples/NN_SLIDING/What.jpg"><br>
        <img style="float: center;" src="examples/NN_SLIDING/Vhat.jpg"><br>

        **5. Plot of the parameter estimate errors.** <br>

        In this project, we plot the errors between the FCNN output and the ground truth.<br>

        <img style="float: center;" src="examples/NN_SLIDING/Link_1_Parameter_Estimate_Errors_sm.png"><br>
        <img style="float: center;" src="examples/NN_SLIDING/Link_2_Parameter_Estimate_Errors_sm.png"><br>
        <br>
        <br>

    * **neural network-based controller with a continuous RISE feed-back control law for the dynamics**<br>

        **1. Control gains.** <br>

        Control gains: &nbsp; <img src="https://latex.codecogs.com/svg.latex?k_{s}&space;=&space;50" title="k_{s} = 50" /> &nbsp; and &nbsp; <img src="https://latex.codecogs.com/svg.latex?\alpha_{1}&space;=&space;10,&space;\alpha_{2}&space;=&space;20,&space;\beta_{1}&space;=&space;10" title="\alpha_{1} = 10, \alpha_{2} = 20, \beta_{1} = 10" /> &nbsp;.<br>
        Neural Network learning rate: &nbsp; <img src="https://latex.codecogs.com/svg.latex?\gamma_{1}&space;=&space;5,&space;\gamma_{2}&space;=&space;10" title="\gamma_{1} = 5, \gamma_{2} = 10" /> &nbsp;.<br>

        **2. Tracking error plot for each link.**<br>

        <img style="float: center;" src="examples/NN_RISE/Link_1_Position_Errors_sm.png"><br>
        <img style="float: center;" src="examples/NN_RISE/Link_2_Position_Errors_sm.png"><br>

        Root Sum Squared of error is as below: 
        <img style="float: center;" src="examples/NN_RISE/RSS_Error.jpg"><br>

        **3. Control input plot for each link.**<br>

        <img style="float: center;" src="examples/NN_RISE/Link_1_Control_sm.png"><br>
        <img style="float: center;" src="examples/NN_RISE/Link_2_Control_sm.png"><br>

        **4. Plot of the adaptive estimates.** <br>

        <img style="float: center;" src="examples/NN_RISE/What.jpg"><br>
        <img style="float: center;" src="examples/NN_RISE/Vhat.jpg"><br>

        **5. Plot of the parameter estimate errors.** <br>

        In this project, we plot the errors between the FCNN output and the ground truth.<br>

        <img style="float: center;" src="examples/NN_RISE/Link_1_Parameter_Estimate_Errors_sm.png"><br>
        <img style="float: center;" src="examples/NN_RISE/Link_2_Parameter_Estimate_Errors_sm.png"><br>
        <br>
        <br>    

* ### **(B) Discussion section**<br>

  * **Differences in tuning the control gains/adaptations**<br>  

    For &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, it control the derivate of &nbsp; <img src="https://latex.codecogs.com/gif.latex?\hat{W}" title="\hat{W}" /> &nbsp; and <img src="https://latex.codecogs.com/gif.latex?\hat{V}" title="\hat{V}" />. However, after twiddling &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp;, I found that they don't effect the performance of my controller. I think the reason may be that the feedback part in the controller contribute the most effort, and it suppress the feedforward Neural Network part. &nbsp; <img src="https://latex.codecogs.com/gif.latex?\hat{W}" title="\hat{W}" /> &nbsp; in 2 different sets of &nbsp;<img src="https://latex.codecogs.com/svg.latex?\gamma" title="\gamma" />&nbsp; are shown below:<br>

    The first one is when &nbsp; <img src="https://latex.codecogs.com/gif.latex?\gamma_{1}&space;=&space;5,&space;\gamma_{2}&space;=&space;10" title="\gamma_{1} = 5, \gamma_{2} = 10" /> &nbsp;:
    <img style="float: center;" src="examples/W_hat_1.jpg"><br>

    The second one is when &nbsp; <img src="https://latex.codecogs.com/gif.latex?\gamma_{1}&space;=&space;50,&space;\gamma_{2}&space;=&space;100" title="\gamma_{1} = 50, \gamma_{2} = 100" /> &nbsp;:
    <img style="float: center;" src="examples/W_hat_2.jpg"><br>

    The saturation above is because we are using projection method, to prevent them being too large.

    For &nbsp; <img src="https://latex.codecogs.com/gif.latex?k_{s}" title="k_{s}" /> &nbsp; , it is contained both in the Sliding mode term and RISE term, and it is essential in these two implementation. In both cases, only when &nbsp; <img src="https://latex.codecogs.com/gif.latex?k_{s}" title="k_{s}" /> &nbsp; is big enough, we can conclude the asymptomatic tracking in the lyapunov analysis. But, when it become large, the control input would be oscilated and increased. I will show Slding mode controller as examples.<br>

    <img style="float: center;" src="examples/K_s_Error.jpg"><br>
    <img style="float: center;" src="examples/K_s_Input.jpg"><br>

    For &nbsp; <img src="https://latex.codecogs.com/gif.latex?k,&space;k_{n}" title="k, k_{n}" /> &nbsp; , it is contained both in our first continuous controller and the discontinuous controller (with Sliding mode control). We can sum them up and see them as a constant, they contribute a state squared part in the lyapunov analysis, and it's even more essential than the &nbsp; <img src="https://latex.codecogs.com/gif.latex?k_{s}" title="k_{s}" /> &nbsp;. It determine if our tracking error would blow up. When we increse it, the tracking error would decrease faster, but we would increase the input effort as trade-off.

    For &nbsp;<img src="https://latex.codecogs.com/svg.latex?\alpha" title="\alpha" />&nbsp; , it is demanded to be large enough in the Neural Netrok-based controller with RISE term. In the other two controllers, it can improve the convergence of the tracking error, but it also comes with a price as the input effort would be increased.

    For &nbsp; <img src="https://latex.codecogs.com/gif.latex?\beta_{1}" title="\beta_{1}" /> &nbsp; in the Neural Netrok-based controller with RISE term, we knew that it should be big enough to conclude the Semi-GAT result, however, after I twiddling it, it seems it doesn't make a explicit effect on the perforamnce of the controller.

  * **Performance of the tracking error for each controller**<br>  

    <img style="float: center;" src="examples/Error_Compare.jpg"><br>

    In the comparison above, we can know that in term of performance of the tracking error, the first controller with a typical continuous feedback control law behave the worst, and the other two has the similar performance. The neural network-based controller with a discontinuous sliding-mode feedback control law for the dynamics is slightly better than the neural network-based controller with a continuous RISE. But in my implementation, I use a fixed-step solver to deal with the discontinuous sliding-mode feedback, so it tends be inaccurate

  * **Performance of the adaptation for each case**<br> 

    The parameter estimate errors are almost the same in these three cases, it seems that our feedback term has dominated the convergence. In my opinion, the neural network does not have meaningful interpretation given unsupervised learning. Although There is a feedback term &nbsp; <img src="https://latex.codecogs.com/svg.latex?e_{2}" title="e_{2}" /> &nbsp; in the Neural Network update law, we are not using back propagation. In such way, we don't even have to do symmetric breaking, we initilize all the weights in the FCNN as 0. In my implementation the estimate errors very depends on the ground truth &nbsp; <img src="https://latex.codecogs.com/gif.latex?f" title="f" /> &nbsp; .
    
  * **Control effort for each case**<br>    
    
    In terms of control effort, the neural network-based controller with a discontinuous sliding-mode feedback control law for the dynamics behave the worst. So, I think the neural network-based controller with a continuous RISE feed-back control law for the dynamics is the best controller among the three. 









