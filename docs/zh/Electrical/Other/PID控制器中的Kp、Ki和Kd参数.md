## [PID控制器中的Kp、Ki和Kd参数](https://blog.csdn.net/yunddun/article/details/107720644)

PID控制器是一种常见的控制算法，用于调节系统的输出，使其达到设定值。PID控制器由三个参数组成：比例（Kp）、积分（Ki）和微分（Kd）。这些参数分别对应于控制器的比例、积分和微分作用。

### 比例（Kp）

比例参数Kp决定了控制器对误差的响应速度。Kp越大，控制器对误差的反应越快，但也可能导致系统不稳定。比例控制的作用是根据当前误差调整输出，使系统迅速接近设定值。例如：

```
out_speed = Kp * err
```

在这个公式中，*err*是当前误差，*out_speed*是控制器的输出速度。通过调整Kp，可以改变系统的响应速度和稳定性。

### 积分（Ki）  

积分参数Ki用于消除系统的稳态误差。积分控制通过对误差进行累积，使系统在长时间内达到设定值。积分控制的作用是根据误差的累积值调整输出。例如：

```
Integral_bias += err

out_speed = Kp * err + Ki * Integral_bias
```

在这个公式中，*Integral_bias*是误差的累积值。通过调整Ki，可以减小系统的稳态误差，但也可能导致系统超调。

### 微分（Kd）

微分参数Kd用于减小系统的超调量和振荡。微分控制通过对误差的变化率进行调整，使系统更加稳定。微分控制的作用是根据误差的变化率调整输出。例如：

```
out_speed = Kp * err + Kd * (err - last_err)
```

在这个公式中，*last_err*是上一次的误差。通过调整Kd，可以提高系统的稳定性和响应速度。

### 调参经验

在实际应用中，调节PID参数需要一定的经验和技巧。一般来说，可以按照以下步骤进行调参：

将Ki和Kd设为0，逐步增加Kp，直到系统出现振荡，然后将Kp减小到振荡消失。增加Kd，直到系统的超调量和振荡减小。增加Ki，直到系统的稳态误差减小。

通过不断调整Kp、Ki和Kd，可以找到最优的参数组合，使系统达到最佳的控制效果。

效果图如下：

![PID](./Picture/PID.gif)



[1](https://blog.csdn.net/yunddun/article/details/107720644): [PID参数解析+调参经验笔记（经验法）](https://blog.csdn.net/yunddun/article/details/107720644) [2](https://blog.csdn.net/qq_41673920/article/details/84860697): [PID算法原理 一图看懂PID的三个参数](https://blog.csdn.net/qq_41673920/article/details/84860697)