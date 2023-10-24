# Theories

This document includes the theoretical backgrounds of the package

## Table of Contents
- [Theories](#theories)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Discrete-time MAPH](#discrete-time-maph)
    - [Objective](#objective)
    - [Constraints](#constraints)
      - [Connectivity Constraints](#connectivity-constraints)
      - [Conflict Constraints](#conflict-constraints)
  - [Continuous-time MAPH](#continuous-time-maph)
    - [Objective](#objective-1)
    - [Constraints](#constraints-1)
      - [Connectivity Constraints](#connectivity-constraints-1)
      - [Conflict Constraints](#conflict-constraints-1)
    - [Proposed Algorithm](#proposed-algorithm)


## Introduction

We investigate the general formulation of MAPH problems as binary mixed-integer programming.

In plain language, a Multi-Agent Path Finding (MAPH) problem is that a group of agents wants to travel through a network. Each agent has its own starting point and destination. We need to find a routing schedule that determines the path of all agents where they arrive at their destinations without colliding each other.

Let's first introduce some important notations.

- The network is modeled as a directed graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$
- The agents form a set $\mathcal{A}$ where each agent $i$ has
  - a starting vertex $v^{(s)}_i \in \mathcal{V}$ with a departure time $t_i^{(s)}$, and
  - a destination (goal) vertex $v^{(g)}_i \in \mathcal{V}$ (Note: some literature may introduce a fixed arrival time $t_i^{(g)}$, here let's assume we only want to arrive at the destination as quick as possible without any hard deadline. i.e. $t_i^{(g)} \rightarrow \infty$)

The agents are only allowed to travel from vertex $u$ to another vertex $v$ if such edge $e = (u,v) \in \mathcal{E}$ exists in the graph.

Commonly, the two most common objectives for MAPH problems are as follows. Let $t^{(g)}_i$ be the time where agent $i$ arrives at its destination vertex.

1. `Makespan`: The system time required for the whole fleet of agents to achieve their destinations.

   ```math
   \max_{i \in \mathcal{A}} t^{(g)}_i - \min_{j \in \mathcal{A}} t^{(s)}_j
   ```

2. `Sum of costs`: Sum of all agents' travel time.

   ```math
   \sum_{i \in \mathcal{A}} t^{(g)}_i - t^{(s)}_i
   ```

In this article, we focus the philosophy of `sum of costs`


## Discrete-time MAPH

We formulate the MAPH problem in discrete time. That is, the actions are done within a fixed unit time, and only 1 action allowed at each unit time.

### Objective

```math
\min_{x, y} \sum_{t = 1}^T \sum_{i \in \mathcal{A}} \left( \sum_{e \in \mathcal{E}} x_{i,e,t} c^{(E)}_{i,t} + \sum_{v \in \mathcal{V}} y_{i,v,t} c^{(V)}_{i,t} \right)
```
- `Edge selection variable` $x_{i,e,t} \in \lbrace 0, 1 \rbrace$: $x_{i,e,t} = 1$ when agent $i$ travels through edge $e$ at time $t$
- `Edge cost` $c^{(E)}_{i,t} \in \mathbb{N}$: cost for agent $i$ to travel through edge $e$ at time $t$

- `Vertex selection variable` $y_{i,v,t} \in \lbrace 0, 1 \rbrace$: $y_{i,e,t} = 1$ when agent $i$ stays in vertex $v$ at time $t$
- `Vertex cost` $c^{(V)}_{i,t} \in \mathbb{N}$: cost for agent $i$ to stay in vertex $v$ at time $t$

- `Time limit` $T$: Total execution time. Let $\mathcal{T} = \lbrace 1, 2, \dots, T \rbrace$. Also, let the lifespan of each agent be $\mathcal{T}_i = \lbrace t^{(s)}_i, \dots, \min\left( t_i^{(g)}, T \right) \rbrace$

As the costs $c^{(E)}_{i,t}$ and $c^{(V)}_{i,t}$ are some constants, the only variables are $x_{i,e,t}$ and $y_{i,v,t}$.

In the conventional setting of minimizing time of travel, the costs $c^{(E)}_{i,t}$ is the time of travel and $c^{(V)}_{i,t}$ is the required time of stay in the station. Because we have the discrete time setting, we have $c^{(E)}_{i,t} = c^{(V)}_{i,t} = 1$

### Constraints

#### Connectivity Constraints

No action can be executed before the time of departure. i.e. $\forall i \in \mathcal{A}, v \in \mathcal{V}, e \in \mathcal{E}$
```math
y_{i,v,t} = x_{i,e,t} = 0 \; \forall t \in \mathcal{T} \setminus \mathcal{T}_i
```

Across time starting from the agent's departure, the agent must eventually leave the source vertex. $\forall i \in \mathcal{A}$
```math
\sum_{t \in \mathcal{T}_i} \left( \sum_{e = (v_i^{(s)}, w) \in \mathcal{E}} x_{i,e,t} - \sum_{e' = (u, v_i^{(s)}) \in \mathcal{E}} x_{i,e',t} \right) = 1
```

Similarly, across time starting from the agent's departure, the agent must eventually enter the target vertex and stays there. $\forall i \in \mathcal{A}$
```math
\sum_{t \in \mathcal{T}_i} \left( \sum_{e = (u, v_i^{(g)}) \in \mathcal{E}} x_{i,e,t} + \sum_{e' = (v_i^{(g)}, w) \in \mathcal{E}} x_{i,e',t} \right) = 1
```

An agent can only leave the vertex if it's already there. $\forall i \in \mathcal{A}, t \in \mathcal{T}, v \in \mathcal{V}$
```math
\sum_{e = (u, v) \in \mathcal{E}} x_{i,e,t-1} = \sum_{e' = (v, w) \in \mathcal{E}} x_{i,e',t}
```

Agent $i$ visits vertex $v$ when there's incoming travel via any inbound edge. i.e. $\forall i \in \mathcal{A}, v \in \mathcal{V}, t \in \mathcal{T}_i$
```math
y_{i,v,t} = \sum_{e = (u, v) \in \mathcal{E}} x_{i,e,t-1}
```

At each time frame within agent's lifespan, an agent can only be present at one vertex. $\forall i \in \mathcal{A}, t \in \mathcal{T}_i$
```math
\sum_{v \in \mathcal{V}} y_{i,v,t} = 1
```

#### Conflict Constraints

- **Vertex Conflict**: At any given time slot, no more than 1 agent can occupy a vertex $v$. i.e. $\forall v \in \mathcal{V}, t \in \mathcal{T}$
  ```math
  \sum_{i \in \mathcal{A}} y_{i,v,t} \leq 1
  ```

  A safety window $w_v^{(V)} \in \mathbb{N}$ of occupying a vertex $v$ can be considered as $\forall t \in \mathcal{T}$
  ```math
  \sum_{t' = t}^{t + w_v^{(V)}} \sum_{i \in \mathcal{A}} y_{i,v,t'} \leq 1
  ```
  The conventional vertex conflict is a special case of $w_v^{(V)} = 0$

- **Edge Conflict**: At any given time slot, no more than 1 agent can occupy an edge $e$. i.e. $\forall e \in \mathcal{E}, t \in \mathcal{T}$
  ```math
  \sum_{i \in \mathcal{A}} x_{i,e,t} \leq 1
  ```

  A safety window $w_e^{(E)} \in \mathbb{N}$ of occupying an edge $e$ can be considered as $\forall t \in \mathcal{T}$
  ```math
  \sum_{t' = t}^{t + w_e^{(E)}} \sum_{i \in \mathcal{A}} x_{i,e,t'} \leq 1
  ```
  The conventional vertex conflict is a special case of $w_e^{(E)} = 0$

- **Following Conflict**: Following conflict can be solved by applying a vertex safe window $w_v^{(V)} > 0$
- **Swapping Conflict**: A swapping conflict between agent $i$ going through edge $e = (u,v)$ and agent $j$ going through edge $e' = (v, u)$ can be formulated as follows. $\forall t \in \mathcal{T}$

  ```math
  \sum_{i \in \mathcal{A}} x_{i,e,t} + x_{i,e',t} \leq 1
  ```

  Note that nonzero vertex safe window already prevents swapping.

## Continuous-time MAPH

Consider the time being continuous, and the agents are allowed to have different speed. We can also separate the **cost** function from the travel time, we can have different cost like budget, etc.

### Objective

```math
\min_{x, y} \sum_{i \in \mathcal{A}} \left( \sum_{e \in \mathcal{E}} x_{i,e} c^{(E)}_{i,t} + \sum_{v \in \mathcal{V}} y_{i,v} c^{(V)}_i \right)
```
- `Edge selection variable` $x_{i,e} \in \lbrace 0, 1 \rbrace$: $x_{i,e} = 1$ when agent $i$ travels through edge $e$
- `Edge cost` $c^{(E)}_i \in \mathbb{R}^+$: cost for agent $i$ to travel through edge $e$

- `Vertex selection variable` $y_{i,v} \in \lbrace 0, 1 \rbrace$: $y_{i,e} = 1$ when agent $i$ stays in vertex $v$
- `Vertex cost` $c^{(V)}_i \in \mathbb{R}^+$: cost for agent $i$ to stay in vertex $v$

As the costs $c^{(E)}_i$ and $c^{(V)}_i$ are some constants, the only variables are $x_{i,e}$ and $y_{i,v}$.

In the conventional setting, the costs $c^{(E)}_i$ can be the time of travel and $c^{(V)}_i$ is the required time of stay in the station (agent may be forced to stop for a while.)

Consider the vehicle routing problem where agents are vehicles and the edges are roads. Let $\tau_{i,e}^{(E)} \in \mathbb{R}^+$ be the time required for agent $i$ to travel through edge $e$.

For each agent, the travel time is
```math
t_i^{(g)} - t_i^{(s)} = \sum_{e \in \mathcal{E}} x_{i,e} \tau_{i,e}^{(E)} = \sum_{e \in \mathcal{E}} x_{i,e} \frac{\ell(e)}{\nu_{i,e}^{(max)}} = \sum_{e \in \mathcal{E}} x_{i,e} \frac{\ell(e)}{\min(\nu_i^{(max)}, \nu_e^{(max)})}
```

where $\ell(e)$ is the length of edge $e$; $\nu_{i,e}^{(max)}$ is the maximum speed of agent $i$ on edge $e$. This $\nu_{i,e}^{(max)}$ can be further divided into 2 components, $\nu_{i}^{(max)}$ being the maximum speed of agent $i$ and the $\nu_e^{(max)}$ maximum speed allowed on edge $e$.


### Constraints

#### Connectivity Constraints

To guarantee the liveliness of a program, for each agent $i$, we restrain the departure from the source vertex $v_i^{(s)}$ and the arrival of the destination (goal) vertex $v_i^{(g)}$.

The agent must leave the source vertex
```math
\sum_{e = (v_i^{(s)}, u) \in \mathcal{E}} x_{i,e} - \sum_{e' = (u, v_i^{(s)}) \in \mathcal{E}} x_{i,e'} = 1 \;, \forall i \in \mathcal{A}
```

The agent must go into the target vertex and do not leave
```math
\sum_{e = (u, v_i^{(g)}) \in \mathcal{E}} x_{i,e} + \sum_{e' = (v_i^{(g)}, u)} x_{i,e'} = 1 \;, \forall i \in \mathcal{A}
```

This also applies to vertex selection variable
```math
y_{i, v_i^{(s)}} = y_{i, v_i^{(g)}} = 1 \;, \forall i \in \mathcal{A}
```

The Kirchhoff's flow equation needs to be applied to guarantee the path between an agent's source and destination is connected.

```math
\sum_{e = (u, v) \in \mathcal{E}} x_{i,e} = \sum_{e' = (v, w) \in \mathcal{E}} x_{i,e'} \; ,\forall i \in \mathcal{A}, \forall v \in \mathcal{V} \setminus \lbrace v_i^{(s)}, v_i^{(g)} \rbrace
```

The vertex traversal can be determined by edges, i.e.
```math
y_{i,v} =
\begin{cases}
1 & \sum_{e = (u,v) \in \mathcal{E}} x_{i,e} > 0 \\
0 & \text{otherwise}
\end{cases}
```

This constraint is equivalent to
```math
y_{i,v} = \sum_{e = (u,v) \in \mathcal{E}} x_{i,e}
```

#### Conflict Constraints

Let $\bar{t}_{i,v} \in \mathbb{R}^+$ be the time when agent $i$ arrives at vertex $v$.

Since we allow an agent $i$ to stay at vertex $v$ with time $\tau_{i,v}^{(V)} \in \mathbb{R}^+$ which is separated from the edge traveling time $\tau_{i,e}^{(E)} \in \mathbb{R}^+$, the timing sequence for an agent to travel through a vertex and an edge is as follows
- The time agent $i$ just arrives at vertex $v$ is $\bar{t}_{i,v}$
- The time agent $i$ finishes waiting at vertex $v$ and start to move along an edge $e = (v, v')$ is $\bar{t}_{i,v} + \tau_{i,v}^{(V)}$
- The time agent $i$ finishes traveling through edge $e = (v, v')$ is $\bar{t}_{i,v} + \tau_{i,v}^{(V)} + \tau_{i,e}^{(E)} = \bar{t}_{i,v'}$

The above sequence is true when the agent actually travels through the path, thus the true constraints in our problem is as follows:

```math
\bar{t}_{i,v} \geq \bar{t}_{i,u} + y_{i,v}\tau_{i,u}^{(V)} + x_{i,e} \tau_{i, e}^{(E)} \; \forall e = (u,v) \in \mathcal{E}
```

If the above doesn't work, I suspect it to be
```math
\bar{t}_{i,v} \geq \bar{t}_{i,u} + \tau_{i,u}^{(V)} + \tau_{i,e}^{(E)} \; \forall e = (u,v) \in \mathcal{E}
```

With the base case
```math
\bar{t}_{i,v_i^{(s)}} \geq t_i^{(s)} \; \forall i \in \mathcal{A}
```

- **Vertex Conflict**: at any given time, a vertex can only have no more than 1 agent. i.e. for any pair of agents $i,j \in \mathcal{A}$ and any vertex $v$. The article [Linear Optimization - Putting gaps between scheduled items?](https://math.stackexchange.com/questions/2491500/linear-optimization-putting-gaps-between-scheduled-items?noredirect=1&lq=1) provides detail and reasoning about the formulation. In short, suppose we have the starting and ending time of two events $i,j$ being $S_i, E_i, S_j, E_j$. The formulation of the scheduling the 2 events with at least a time gap $G$ in the middle is as follows

  ```math
  \begin{split}
  S_i & \geq E_j + G - M \delta_{i,j} \\
  S_j & \geq E_i + G - M (1 - \delta_{i,j})
  \end{split}
  ```
  where $M$ is a large enough number as the planning length.

  This formulation is equivalent to
  ```math
  \begin{cases}
  S_i \geq E_j + G & \text{ if } \delta_{i,j} = 1 \\
  S_j \geq E_i + G & \text{ if } \delta_{i,j} = 0
  \end{cases}
  ```
  
  With such formulation in mind, we have the equivalent formulation in our MAPH problem.

  ```math
  \begin{split}
  \bar{t}_{i,v} - y_{i,v} \xi_{i,v}^- & \geq \bar{t}_{j,v} + y_{j,v} \tau_{j,v}^{(V)} + y_{j,v} \xi_{j,v}^+ - M \delta_{i,j,v} \\
  \bar{t}_{j,v} - y_{j,v} \xi_{j,v}^- & \geq \bar{t}_{i,v} + y_{i,v} \tau_{i,v}^{(V)} + y_{i,v} \xi_{i,v}^+ - M (1 - \delta_{i,j,v})
  \end{split} 
  ```
  where 
  - `prior safe time-interval` $\xi_{i,v}^-$ is the reserved time duration as safety margin before agent $i$'s arrival at vertex $v$;
  - `posterior safe time-interval` $\xi_{i,v}^+$ is the reserved time duration as safety margin after agent $i$'s departure at vertex $v$;
  - `if-else variable` $\delta_{i,j,v} \in \{0,1\}$ is the binary variable for the decision-making.
  - `if-else constant` $M$ is a large enough number for the if-else statement in integer programming;

  This means agent $i$ occupies vertex $v$ from time $S_i = \bar{t}_{i,v} - y_{i,v} \xi_{i,v}^-$ until time $E_i = \bar{t}_{i,v} + y_{i,v} \tau_{i,v}^{(V)} + y_{i,v} \xi_{i,v}^+$ without any enforced gap $G=0$. During this time period, no overlapping can happen between any two agents.


- **Edge Conflict**: Similar to the vertex's formulation, we prevent any time overlapping of any pairs of agents to travel through any edge. Now the starting and ending time for agent $i$ to travel on edge $e = (u,v)$ is

  ```math
  \begin{split}
  S_i & = \bar{t}_{i,u} + y_{i,u} \tau_{i,u}^{(V)} - x_{i,e} \xi_{i,e}^- \\
  E_i & = \bar{t}_{i,v} + x_{i,e} \xi_{i,e}^+
  \end{split}
  ```

  Therefore, the non-overlapping constraint becomes

  ```math
  \begin{split}
  \bar{t}_{i,u} + y_{i,u} \tau_{i,u}^{(V)} - x_{i,e} \xi_{i,e}^- & \geq \bar{t}_{j,v} + x_{j,e} \xi_{j,e}^+ - M \delta_{i,j,e} \\
  \bar{t}_{j,u} + y_{j,u} \tau_{j,u}^{(V)} - x_{j,e} \xi_{j,e}^- & \geq \bar{t}_{i,v} + x_{i,e} \xi_{i,e}^+ - M (1 - \delta_{i,j,e})
  \end{split}
  ```

- **Following Conflict**: Following conflict can be solved by applying safe margin of vertex traversal. i.e. having $\xi_{i,v}^- + \xi_{i,v}^+ > 0$

- **Swapping Conflict**: A swapping conflict between $e = (u,v)$ and $e' = (v, u)$ is similar to edge conflict. Without loss of generality, we say agent $i$ tries to use edge while agent $j$ tries to use edge $e'$.
  
  For agent $i$, we have the occupancy time of edge $e$ being
  ```math
  \begin{split}
  S_i & = \bar{t}_{i,u} + y_{i,u} \tau_{i,u}^{(V)} - x_{i,e} \xi_{i,e}^- \\
  E_i & = \bar{t}_{i,v} + x_{i,e} \xi_{i,e}^+
  \end{split}
  ```

  While for agent $j$, the occupancy time of edge $e'$ is
  ```math
  \begin{split}
  S_j & = \bar{t}_{j,v} + y_{j,v} \tau_{j,v}^{(V)} - x_{j,e'} \xi_{j,e'}^- \\
  E_j & = \bar{t}_{j,u} + x_{j,e'} \xi_{j,e'}^+
  \end{split}
  ```

  Therefore, the non-overlapping constraint becomes
  ```math
  \begin{split}
  \bar{t}_{i,u} + y_{i,u} \tau_{i,u}^{(V)} - x_{i,e} \xi_{i,e}^- & \geq \bar{t}_{j,u} + x_{j,e'} \xi_{j,e'}^+ - M \delta^{(sw)}_{i,j,e} \\
  \bar{t}_{j,v} + y_{j,v} \tau_{j,v}^{(V)} - x_{j,e'} \xi_{j,e'}^- & \geq \bar{t}_{i,v} + x_{i,e} \xi_{i,e}^+ - M ( 1 - \delta^{(sw)}_{i,j,e})
  \end{split}
  ```

### Proposed Algorithm

To avoid the potential waste of computational resource by computing $\delta_{i,j,v}$ and $\delta_{i,j,e}$ which are $|V|^2 (|V| + |E|)$ integer constraints, we can use a **branch-and-constrain** technique as what has been done for the **conflict-based search** for the conflict constraints. i.e.

Let $A_{i,v}^{(V)}$ be the arriving time of agent $i$ on vertex $v$ and $D_{i,v}^{(V)}$ be the departure time of agent $j$ on vertex $j$. The same applies to edge arriving time $A_{i,e}^{(E)}$ and $D_{i,e}^{(E)}$.

- Solve the MIP without any vertex nor edge constraints.
- If a vertex conflict is detected: branch out 2 cases with vertex constraints
  - Assume agent $i$ departs before agent $j$ arrives at vertex $v$. i.e. $A_{i,v}^{(V)} \geq D_{j,v}^{(V)}$, or using the above notation

    ```math
    \bar{t}_{i,v} - \xi_{i,v}^- \geq \bar{t}_{j,v} + \tau_{j,v}^{(V)} + \xi_{j,v}^+
    ```
    or
    ```math
    \bar{t}_{i,v} \geq \bar{t}_{j,v} + \tau_{j,v}^{(V)} + \max\left( \xi_{i,v}^-,  \xi_{j,v}^+ \right)
    ```
    with $\max\left( \xi_{i,v}^-,  \xi_{j,v}^+ \right)$ being the safe time interval between $i$'s departure and $j$'s arrival.

  - Assume agent $j$ departs before agent $i$ arrives at vertex $v$. i.e. $A_{j,v}^{(V)} \geq D_{i,v}^{(V)}$, or using the above notation

    ```math
    \bar{t}_{j,v} - \xi_{j,v}^- \geq \bar{t}_{i,v} + \tau_{i,v}^{(V)} + \xi_{i,v}^+
    ```
    or
    ```math
    \bar{t}_{j,v} \geq \bar{t}_{i,v} + \tau_{i,v}^{(V)} + \max\left( \xi_{j,v}^-,  \xi_{i,v}^+ \right)
    ```

- If an edge conflict is detected: branch out 2 cases with edge constraints
  - Assume agent $i$ ends the edge traversal on $e$ before agent $j$ starts the edge traversal on $e$. i.e. $A_{i,e}^{(E)} \geq D_{j,e}^{(E)}$
  - Assume agent $j$ ends the edge traversal on $e$ before agent $i$ starts the edge traversal on $e$. i.e. $A_{j,e}^{(E)} \geq D_{i,e}^{(E)}$

- Iteratively apply the **branch-and-constrain** until no more conflicts are found