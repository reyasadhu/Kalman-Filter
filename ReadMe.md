This a state estimation with Kalman filter problem, which is part of the homework assignments for the course ECE 275A at UCSD for Fall 2023.

# **Problem:**

Consider a sequence of states that consist of location and velocity, i.e.,

$` \mathbf{x}_{\mathbf{n}} = \left\{ x_{1,n},\ x_{2,n},{\dot{x}}_{1,n},\ {\dot{x}}_{2,n} \right\}^{T},\ \ n \in \left\{ 0,\ldots,N \right\}. `$

The state transition model of the mobile object is given by

$` \mathbf{x}_{\mathbf{n}} = \underset{\mathbf{A}}{\overset{\begin{bmatrix}
\begin{matrix}
1 & 0 \\
0 & 1 \\
\end{matrix} & \begin{matrix}
T & 0 \\
0 & T \\
\end{matrix} \\
\begin{matrix}
0 & 0 \\
0 & 0 \\
\end{matrix} & \begin{matrix}
1 & 0 \\
0 & 1 \\
\end{matrix} \\
\end{bmatrix}}{︸}}\mathbf{x}_{\mathbf{n - 1}} + \ \underset{\mathbf{W}}{\overset{\begin{bmatrix}
0.5T^{2} & 0 \\
\begin{matrix}
0 \\
T \\
0 \\
\end{matrix} & \begin{matrix}
0.5T^{2} \\
0 \\
T \\
\end{matrix} \\
\end{bmatrix}}{︸}}\mathbf{q}_{\mathbf{n}}\mathbf{,\ }with\ \mathbf{q}_{\mathbf{n}}\mathbf{\sim}N\mathbf{(0,}\sigma_{q}^{2}\mathbf{I}_{\mathbf{2}}\mathbf{\ )} `$

where T = 0.1 is the duration of discrete time steps and $`\sigma_{q} = 0.05.\ \ \mathbf{q}_{n} \in R^{2}`$ can be interpreted as a 2-D random acceleration. The prior at time n = 0 is Gaussian, i.e.,
$`\mathbf{x}_{0}\sim N\left( \mathbf{\mu}_{0},\ \mathbf{\Sigma}_{0} \right)\ `$ with given mean $`\mathbf{\mu}_{0}`$ and covariance matrix
$`\mathbf{\Sigma}_{0}`$. For $`n \in \left\{ 1,\ldots,N \right\}.`$, measurements are modeled by an additive-noise measurement model, i.e.,

$` \mathbf{y}_{\mathbf{n}}\mathbf{=}\underset{\mathbf{H}}{\overset{\begin{bmatrix}
\begin{matrix}
1 \\
0 \\
\end{matrix} & \begin{matrix}
\begin{matrix}
0 \\
1 \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\end{matrix} & \begin{matrix}
0 \\
0 \\
\end{matrix} \\
\end{matrix} \\
\end{bmatrix}}{︸}}\mathbf{x}_{\mathbf{n}}\mathbf{+}\mathbf{v}_{\mathbf{n}},\ with\ \mathbf{v}_{\mathbf{n}}\mathbf{\sim}N\mathbf{(0,}\sigma_{v}^{2}\mathbf{I}_{\mathbf{2}}\mathbf{\ )} `$

Where $`\sigma_{v} = 5`$. We need to find the estimated track given the prior and the observations by implementing a Kalman Filter.

For part 2, measurements are generated based on a range-bearing
measurement model, i.e.,

$$y_{1,n} = \left\| \left( x_{1,n}\ x_{2,n} \right)^{T} - \mathbf{p} \right\| + v_{1,n}$$

$$y_{2,n} = atan2\left( x_{1,n} - p_{1},\ x_{2,n} - p_{2} \right) + v_{2,n}\ (in\ radians)$$

With
$` y_{n} = \left( y_{1,n}\ y_{2,n} \right)^{T},\ \left( h_{1}\left( x_{n} \right)\ h_{2}\left( x_{n} \right) \right)^{T},\ v_{n} = \left( v_{1,n}\ v_{2,n} \right)^{T},\ v_{n}\sim N\left( 0,diag\left( \sigma_{v_{1}}^{2},\ \sigma_{v_{2}}^{2} \right) \right) `$
and known sensor location $` p = \left( p_{1}\ p_{2} \right)^{T} `$.
Measurement noise standard deviations are given by
$`\sigma_{v_{1}} = 5\ and\ \sigma_{v_{2}} = 0.01\ .\ `$ For this, we
need to estimate the track using the extended (EKF) and unscented
(UKF) Kalman filter.

# **Code Description:**

The \_common folder holds all the functions that are used by all three
Kalman filters.

#### ***checkAndFixCovarianceMatrix:*** If at any step, the covariance matrix becomes zero, this function adds a very small value to the matrix to make it non-zero.

#### ***getError:*** calculate the root mean square error (RMSE) between the true track and the estimated track.

#### ***getTrueTrack:*** generate the tracks as per the state transition model.

#### ***getModelMatrices:*** compute A, W and H as per definition.

#### ***getObservations:*** Create observations as per observation model. This code will be different for part 1 and 2.

#### ***observationModel:*** helps creating observations for part 2.

#### ***performEstimationKalman:*** implement Kalman filter prediction and update step. There will be **performEstimationEKF** for the extended Kalman filter implementation and **performEstimationUKF** for unscented Kalman filter implementation.

#### ***getSigmapoints:*** generates sigma points given mean and covariance. This is used for unscented Kalman Filter.

#### ***main:*** main function for implementing Kalman filter. Uses different function to get the estimated tracks, plot it and calculate the rmse. There are ***mainEKF*** and ***mainUKF*** for extended and unscented Kalman filter.

# **Solution:**

For the implementation purposes, we assume that we know the driving noise and measurement noise covariances. In real life scenario, this
is not the case. However, an important property of sequential LMMSE estimation and Kalman filtering is that the innovation sequence (estimation error) is zero-mean and uncorrelated (white). This is because the predicted measurement is an LMMSE estimate of the current measurement $`y_{n}`$ based on previous data $`y_{1:n-1}`$. The current innovation (estimation error) is thus orthogonal to all linear functions of previously collected data. The fact that the innovation sequence of a correctly applied Kalman filter is zero-mean and uncorrelated can be used to tune a Kalman filter applied to data from a sequential model with unknown parameters. One can adjust the variance of measurement and driving noise such that a zero-mean and uncorrelated innovation sequence is obtained.

## **Kalman Filter**

The observation model is linear, so these can be written as

$` x_{n} = Ax_{n - 1} + {Wq}_{n}`$, where $`q_{n}\sim N(0,\Sigma_{q})`$

$` y_{n} = Hx_{n} + v_{n}`$ , where $`v_{n}\sim N(0,\Sigma_{v}) `$

So, $` {Wq}_{n}\sim N(0,{W\Sigma}_{q}W^{T}) `$

#### Prediction Step:

$` \mu_{x|y\_} = A\mu_{x\_|y\_} `$, where $`\mu_{x\_|y\_} `$  is the previous estimation. 

$` \Sigma_{xx|y\_} = A\Sigma_{x\_ x\_|y\_}A^{T} + {W\Sigma}_{q}W^{T})`$

#### Update Step:

Kalman gain = K = $` \Sigma_{xx|y\_}H^{T}{({H\Sigma}_{xx|y\_}H^{T} + \Sigma_{v})}^{- 1} `$

Innovation Sequence = $` y - \mu_{y} = y - H\mu_{x|y\_} `$

$` \mu_{x|y} = \mu_{x|y\_} + K(y - H\mu_{x|y\_}) `$

$` \Sigma_{xx|y} = \Sigma_{xx|y\_} - KH\Sigma_{xx|y\_} `$

Error = 1.1348

| Innovation Sequence| True VS Estimated Track|
:-------------------------:|:----------------------:
![](/images/image1.jpg)|![](/images/image2.jpg)


The innovation sequence seems to have zero mean and uncorrelated with each other.

Now, if we introduce model mismatch, i.e we estimate the driving noise or measurement noise parameters different than the actual parameters, the output changes.

If we take a very low value of variance in driving noise, the estimated track becomes a perfect straight line with almost no deviation from the path. Because, in this case the filter trusts its prediction too much, thus does not account for the sudden change in system dynamics. So, in case of a change, it is very slow to adapt the changes. Whereas, for a high value of variance, the filter doesn't trust his prediction and rely on the measurements. Thus the path becomes too noisy. 

| High process noise variance, noisy path|Low Process Noise variance, smooth path, not converging|
:------------------:|:-------------------------:
![](/images/image3.jpg)
![](/images/image4.jpg)

Similarly, if the measurement noise variance is too low, the filter trusts its measurement too much and tends to overfit the measurements. Whereas, in case of high variance, it trusts its prediction more thus slow to adapt changes in the actual system dynamics.

| High Measurement variance, slow to adapt changes|  Low Measurement variance, overfitting measurements |
:------------------:|:-------------------------:
![](/images/image5.jpg)
![](/images/image6.jpg)



## **Extended Kalman Filter**

$`x_{n} = Ax_{n - 1} + {Wq}_{n}\ ,\ where\ q_{n}\sim N(0,\Sigma_{q}) `$

$`y_{n} = h(x_{n}) + v_{n}\ ,\ where\ v_{n}\sim N(0,\Sigma_{v}) `$

So, $`{Wq}_{n}\sim N(0,{W\Sigma}_{q}W^{T})`$

Now,
$` P\left( x_{n - 1} \middle| y_{1:n - 1} \right) = P\left( x_{-} \middle| y_{-} \right)\sim N(\mu_{x\_|y\_},\ \Sigma_{x\_ x\_|y\_})`$

The observation model is no longer linear. We can linearize h(.) about $`\mu_{x|y\_}`$ as,

$`h\left( x_{n} \right) = h\left( \mu_{x|y_{-}} \right) + H_{n}(x_{n} - \mu_{x|y_{-}})`$

Where $` H_{n} = \left. \ \frac{dh\left( x_{n} \right)}{d\left( x_{n} \right)} \right|_{x_{n} = \mu_{x|y_{-}}}`$

$` h\left( x_{n} \right) = \begin{bmatrix}
h_{1}\left( x_{n} \right) \\
h_{2}\left( x_{n} \right) \\
\end{bmatrix}`$

$`H_{n} = \begin{bmatrix}
\left. \ \frac{dh_{1}\left( x_{n} \right)}{d\left( x_{n} \right)} \right|_{x_{n} = \mu_{x|y_{-}}} \\
\left. \ \frac{dh_{2}\left( x_{n} \right)}{d\left( x_{n} \right)} \right|_{x_{n} = \mu_{x|y_{-}}} \\
\end{bmatrix}`$

$`h_{1}\left( x_{n} \right) = \left\| {(x_{1,n}\ x_{1,n})}^{T} - \mathbf{P} \right\| = \sqrt{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}`$

$`h_{2}\left( x_{n} \right) = atan2(x_{1,n} - p_{1},\ x_{2,n} - p_{2})`$

$`\frac{\partial h_{1}\left( x_{n} \right)}{\partial x_{1,n}} = \frac{x_{1,n} - p_{1}}{\sqrt{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}}`$

$`\frac{\partial h_{1}\left( x_{n} \right)}{\partial x_{2,n}} = \frac{x_{2,n} - p_{2}}{\sqrt{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}}`$

$`\frac{\partial h_{2}\left( x_{n} \right)}{\partial x_{1,n}} = \frac{x_{2,n} - p_{2}}{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}`$

$`\frac{\partial h_{2}\left( x_{n} \right)}{\partial x_{2,n}} = - \frac{x_{1,n} - p_{1}}{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}`$

$`\frac{\partial h_{1}\left( x_{n} \right)}{\partial x_{3,n}} = \frac{\partial h_{1}\left( x_{n} \right)}{\partial x_{4,n}} = \frac{\partial h_{2}\left( x_{n} \right)}{\partial x_{3,n}} = \frac{\partial h_{2}\left( x_{n} \right)}{\partial x_{4,n}} = 0`$

$` H_{n} = \left\lbrack \begin{matrix}
\frac{x_{1,n} - p_{1}}{\sqrt{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}} \\
\frac{x_{2,n} - p_{2}}{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}} \\
\end{matrix} `$

$` \begin{matrix}
\frac{x_{2,n} - p_{2}}{\sqrt{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}}} \\
 - \frac{x_{1,n} - p_{1}}{\left( x_{1,n} - p_{1} \right)^{2} + \left( x_{2,n} - p_{2} \right)^{2}} \\
\end{matrix} `$

$`\begin{matrix}
0 & 0 \\
0 & 0 \\
\end{matrix} \right\rbrack_{x_{n} = \mu_{x|y_{-}}} `$

Prediction Step:

$`\mu_{x|y\_} = A\mu_{x\_|y\_}\ ,\ where\ \mu_{x\_|y\_}`$ is the previous estimation. 

$`\Sigma_{xx|y\_} = A\Sigma_{x\_ x\_|y\_}A^{T} + {W\Sigma}_{q}W^{T}`$

Update Step:

Kalman gain = K = $`\Sigma_{xx|y\_}{H_{n}}^{T}{({H_{n}\Sigma}_{xx|y\_}H_{n}^{T} + \Sigma_{v})}^{- 1}`$

$`\mu_{x|y} = \mu_{x|y\_} + K(y - h(\mu_{x|y\_}))`$

$`\Sigma_{xx|y} = \Sigma_{xx|y\_} - KH\Sigma_{xx|y\_}`$

**RMSE= 0.6629**

![](/images/image7.jpg)

## **Unscented Kalman Filter:**

$`x_{n} = Ax_{n - 1} + {Wq}_{n}\ ,\ where\ q_{n}\sim N(0,\Sigma_{q}) `$

$`y_{n} = h(x_{n}) + v_{n}\ ,\ where\ v_{n}\sim N(0,\Sigma_{v}) `$

So, $`{Wq}_{n}\sim N(0,{W\Sigma}_{q}W^{T})`$

Now,
$`P\left( x_{n - 1} \middle| y_{1:n - 1} \right) = P\left( x_{-} \middle| y_{-} \right)\sim N(\mu_{x\_|y\_},\ \Sigma_{x\_ x\_|y\_})`$

For unscented Kalman Filter, instead of linearizing the h function, we capture the propagation of the statistical properties of state estimates through this nonlinear function. The algorithm first generates a set of state values called sigma points. These sigma points capture the mean and covariance of the state estimates. The algorithm uses each of the sigma points as an input to the state transition and measurement functions to get a new set of transformed state points. The mean and covariance of the transformed points is then used to obtain state estimates and state estimation error covariance.

For our problem, we estimate the
$`P\left( x \middle| y_{-} \right)`$ distribution from the sigma points of
the $`P\left( x_{-} \middle| y_{-} \right)`$ distribution.

Now if, $`x_{\_}^{(i)}`$ and $`{weights}_{\_}^{(i)}`$ are $i^{th}$ sigma point and weight respectively from the $`P\left( x\_ \middle| y_{-} \right)`$ distribution,

Then, sigma points for the approximated
$`P\left( x \middle| y_{-} \right)`$ distributions are,

$`x^{(i)} = A*x_{\_}^{(i)}`$

Prediction Step:

$`{\widetilde{\mu}}_{x|y\_} = SP\ approximation\ of\ \ \mu_{x|y\_} = \sum_{i}^{}{{weights}_{\_}^{(i)}*}x^{(i)}`$

$`{\widetilde{\Sigma}}_{xx|y\_} = {SP\ approximation\ of\ \Sigma}_{xx|y\_} = \sum_{i}^{}{{weights}_{\_}^{(i)}*}{(x}^{(i)} - \mu_{x|y\_})*{{(x}^{(i)} - \mu_{x|y\_})}^{T} + {W\Sigma}_{q}W^{T}`$

Update Step:

Sigma points of $`P\left( y \middle| y_{-} \right) `$ are calculated by,

$`y^{(i)} = h(x^{(i)})`$

$`{\widetilde{\mu}}_{y|y\_} = SP\ approximation\ of\ \ \mu_{y|y\_} = \sum_{i}^{}{{weights}_{\_}^{(i)}*}y^{(i)}`$

$`{\widetilde{\Sigma}}_{yy|y\_} = {SP\ approximation\ of\ \Sigma}_{xx|y\_} = \sum_{i}^{}{{weights}_{\_}^{(i)}*}{(y}^{(i)} - \mu_{y|y\_})*{{(y}^{(i)} - \mu_{y|y\_})}^{T} + \Sigma_{v}`$

$`{\widetilde{\Sigma}}_{xy|y\_} = {SP\ approximation\ of\ \Sigma}_{xx|y\_} = \sum_{i}^{}{{weights}_{\_}^{(i)}*}{(x}^{(i)} - \mu_{x|y\_})*{{(y}^{(i)} - \mu_{y|y\_})}^{T}`$

So,

Kalman gain = K = $`{\widetilde{\Sigma}}_{xy|y\_}{\widetilde{\Sigma}}_{yy|y\_}`$

$`{{\widetilde{\mu}}_{x|y} = SP\ approximation\ of\ \mu}_{x|y} = {\widetilde{\mu}}_{x|y\_} + K(y - {\widetilde{\mu}}_{y|y\_})`$

$`{\widetilde{\Sigma}}_{xx|y} = SP\ approximation\ of\ \Sigma_{xx|y} = {\widetilde{\Sigma}}_{xx|y\_} - K{{\widetilde{\Sigma}}_{xy|y\_}}^{T}`$

**RMSE= 0.6622**

![](/images/image8.jpg)
