# CarND-Controls-MPC

## The Model

It is kinematic model and have 6 variables of state:

1. x: x-axis position
2. y: y-axis position
3. psi: angle from the x axis
4. v: speed
5. cte: cross track error
6. epsi: orientation error

And there's two actuators:
1. steering angle
2. throttle. 

We slice MPC controll into time steps, at each time step, use following equations to update state:

1. x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(psi<sub>t</sub>) * dt
2. y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(psi<sub>t</sub>) * dt
3. psi<sub>t+1</sub> = psi<sub>t</sub> - v<sub>t</sub> * delta<sub>t</sub> / Lf * dt
4. v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt
5. cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(epsi<sub>t</sub>) * dt)
6. epsi<sub>t+1</sub> = psi<sub>t</sub> - psides<sub>t</sub> - v<sub>t</sub> * delta<sub>t</sub> / Lf * dt

I choose following cost function:

1. State Error
- difference between the state and reference values, including CTE, Error of PSI and Error of velocity.
- This part keeps the car stay close with reference.
2. Actuator Value
- including delta and accelerator(throttle)
- Adding this part into cost function will make the car have less motion
3. Change rate of actuator input:
- including delta change rate and jerk
- This part makes change of motion slower

Then together with all above and plus constrains, we can use a solver to find a solution that in between the constrains and makes the cost function value the least.


## Polynomial Fitting and MPC Preprocessing

First, we need transform the coordinate system from simulator's global map coordinate system to the car's coordinate system, it is implemented in `main.cpp:119-129`. After transform, the position of the car(x, y) and angle(psi) should be zero.

Then, use a 3-order polyfit to generate curve coefficients according to transformed waypoints, `main.cpp:139`.


## Model Predictive Control with Latency

There is a 100ms or 0.1s latency in this model, we should consider this and update the start state with a prediction. And consider the transform above, the first state, x, y and psi could be considered as `0`, we can use a simplified predictions(`main.cpp:145-151`):

1. x<sub>pred</sub> = 0 + v * latency;
2. y<sub>pred</sub> = 0;
3. psi<sub>pred</sub> = psi - v * steering_angle / Lf * latency;
4. v<sub>pred</sub> = v + throttle * latency;
5. cte<sub>pred</sub> = cte + v * sin(epsi) * latancy;
6. epsi<sub>pred</sub> = epsi + v * -delta / Lf * latancy;


## Parameters Tuning
1. Timestep Length and Elapsed Duration (N & dt)

There are actually 2 other parameters affect the N and dt tuning: reference speed and prediction Time:

T = dt * N

In general, T is the larger the better and proportional to reference speed, but larger T meanse larger N(keeps dt unchanged) or larger dt(keeps N unchanged). The larger N means it will take more time to solve a result or means higher rate to fail. The larger dt means it may have a large change between time steps, but too small dt also introduce concrete error.

First, I set reference speed as 30, then decide T should be >= 1s.

First, I tried N=40, dt=0.05. It seems some time the calculation takes too much time that it fails to give a correct solution.

Then, I tried to reduce N to 20, result seems to be ok.

I also tried to reduce N to 10, I can see the error is larger than N = 20.

So I choose N = 20, dt = 0.05 finally.

2. Cost Weights

All cost function parts are discussed above. I first set all the cost parts' weight as 1, so they take the same importance, but I found the result fails, the car starts to oscillation. So I think it need to minimize the delta and throttle, so I make the delta and throttle weight 10 times larger than other parts, and it works fine.