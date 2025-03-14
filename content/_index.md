+++
title = "Quantifying Aleatoric and Epistemic Dynamics Uncertainty via Local Conformal Calibration"
[extra]
authors = [
    {name = "Luís Marques", url = "https://luis-marques.github.io/"},
    {name = "Dmitry Berenson", url = "https://berenson.robotics.umich.edu/"}
]
venue = {name = "16th International Workshop on the Algorithmic Foundations of Robotics (WAFR) 2024", url = "https://www.algorithmic-robotics.org/"}
buttons = [
    {name = "Paper", url = "https://arxiv.org/abs/2409.08249"},
    {name = "PDF", url = "https://arxiv.org/pdf/2409.08249"},
    {name = "Slides", url = "https://luis-marques.github.io/slides/wafr24/"},
    {name = "Poster", url = "https://luis-marques.github.io/assets/pdf/posterUMichAISymposium2024.pdf"}
]
katex = true
large_card = true
favicon = false
+++

Whether learned, simulated, or analytical, approximations of a robot's dynamics can be inaccurate when encountering novel environments. Many approaches have been proposed to quantify the aleatoric uncertainty of such methods, i.e. uncertainty resulting from stochasticity, however these estimates alone are not enough to properly estimate the uncertainty of a model in a novel environment, where the actual dynamics can change. Such changes can induce epistemic uncertainty, i.e. uncertainty due to a lack of information/data. Accounting for *both* epistemic and aleatoric dynamics uncertainty in a theoretically-grounded way remains an open problem. We introduce **L**ocal **U**ncertainty **C**onformal **Ca**libration (LUCCa), a conformal prediction-based approach that calibrates the aleatoric uncertainty estimates provided by dynamics models to generate probabilistically-valid prediction regions of the system's state. We account for both epistemic and aleatoric uncertainty non-asymptotically, without strong assumptions about the form of the true dynamics or how it changes. The calibration is performed locally in the state-action space, leading to uncertainty estimates that are useful for planning. We validate our method by constructing probabilistically-safe plans for a double-integrator under significant changes in dynamics.

{% figure(alt=["LUCCA Diagram"] src=["model_final.png"] dark_invert=[true]) %}
**Figure.** Schematic of Local Uncertainty Conformal Calibration (LUCCa). Starting from a calibrated prediction region `$\hat{\mathcal N}_{\tau, cal}$`, we propagate the state uncertainty by composing an approximate dynamics model `$\tilde f$` outputting predictive MVNs of the future state with local conformal calibration. We consider approximate dynamics where the input and output distributions are MVNs of the state, but otherwise do not restrict the structure of `$\tilde f$`.
{% end %}


# Problem Statement

**Problem.** Given an approximation `$\tilde f$` of the system's *unknown* stochastic dynamics `$f$`, a goal region `$\mathcal G \subseteq \mathcal S$`, a safe set `$\mathscr C \subseteq \mathcal S$`, a calibration dataset of state transitions `$D_{cal}$` and an acceptable failure-rate `$\alpha \in (0,1)$`, we aim to recursively solve the following stochastic optimization problem with planning horizon `$H \in \mathbb N$`:
```
$$
\begin{alignat}{3}
&\!\min_{(u_{t},\ldots,u_{t+H-1})}        &\qquad& J(s, u, \mathcal G)\\
&\text{subject to} &      & Y_{\tau} \sim f(\bar{X}_\tau),\quad & \forall\tau\in\{t, \ldots, t+H-1\}\\
&               &  & \mathbb P(Y_{\tau} \in \mathscr C) \ge (1-\alpha),\quad & \forall\tau \in \{t,\ldots, t+H-1\} \\
&               &  & u_{\tau} \in \mathcal U, s_{\tau+1} \in \mathcal S, \quad &\forall\tau\in\{t, \ldots, t+H-1\}
\end{alignat}
$$
```
&emsp; *Dynamics, (2)*: The real system evolves following the unknown stochastic dynamics `$f$`. We only have access to an approximation `$\tilde f$` and the inference transitions in `$D_{cal}$`.<br>
&emsp; *Safety, (3)*: Our trajectory should remain probabilistically safe, which we define as requiring it to lie within a safe set `$\mathscr C$` with at least a user-specified probability. This is difficult to guarantee in general for any `$f$` and `$\tilde f$`, since our approximation can be arbitrarily wrong in deployment conditions. <br>
&emsp;*State and Control Admissibility, (4)*: Both the control inputs and the states must belong to pre-defined sets. <br>
&emsp;*Objective Function, (1)*: Additionally, we aim to achieve optimality relative to an objective that incorporates `$\mathcal G$` along with the state and control sequences, `$s := (s_{t+1},\ldots, s_{t+H})$` and `$u := (u_{t},\ldots,u_{t+H-1})$`. For example, `$J$` might minimize the expectation of a distance metric to `$\mathcal G$`, control effort, or epistemic-uncertainty along the state-control sequences.

# LUCCa

Given a dynamics predictor and a small calibration dataset, LUCCa provides probabilistically valid prediction regions for the robot's future states accounting for both aleatoric and epistemic uncertainty. We prove its validity for any finite set of calibration data, predictors outputting a multivariate normal uncertainty, any unknown true dynamics function, and uncharacterized aleatoric perturbations. LUCCa calibrates the uncertainty locally relative to the system's state-action space, leading to prediction regions that are representative of predictive uncertainty and therefore useful for planning. For the first planning step, LUCCa satisfies the safety condition (3) above. For subsequent planning steps, such a guarantee becomes more complex but we show that if the dynamics approximation is linear (actual dynamics are still unconstrained) and we can satisfy additional assumptions on the controller, then LUCCa can satisfy the safety condition (3) for all planning steps (see Appendix `$A$` for a discussion and the proof).

{{ figure(alt=["LUCCA Algorithm", "Calibrated Rollout"] src=["lucca_algo.png", "calibrated_rollout.png"] dark_invert=[true, true]) }}

# Experiments

We conducted experiments with an MPC controller that uses LUCCa to plan short-horizon trajectories to reach a goal region (without colliding with any obstacles). We compared LUCCa with a baseline (using the same predictor but without the conformal calibration step) on a double-integrator system over four environments shown below. In the white regions the dynamics predictor corresponds to the ground truth dynamics, but in the yellow regions there is a significant mismatch (actual dynamics become lower-friction). In both areas there is aleatoric uncertainty. This dynamical system can build significant momentum, and thus underestimating the predictive uncertainty can lead to entering regions of inevitable collision. Hence, it is crucial to accurately quantify uncertainty multiple time-steps into the future. 

{{ figure(src = ["./Corridor_Uncalibrated Baseline.mp4","./Corridor_LUCCa.mp4"], subcaption = ["**Uncalibrated Baseline** (Corridor Map)","**LUCCa** (Corridor Map)"], dark_invert=[false,false]) }}


{{ figure(src = ["./Passage_Uncalibrated Baseline.mp4","./Passage_LUCCa.mp4"], subcaption = ["**Uncalibrated Baseline** (Passage Map)","**LUCCa** (Passage Map)"], dark_invert=[false,false]) }}

{{ figure(src = ["./U-Turn_Uncalibrated Baseline.mp4","./U-Turn_LUCCa.mp4"], subcaption = ["**Uncalibrated Baseline** (U-Turn Map)","**LUCCa** (U-Turn Map)"], dark_invert=[false,false]) }}

{{ figure(src = ["./L-Turn_Uncalibrated Baseline.mp4","./L-Turn_LUCCa.mp4"], subcaption = ["**Uncalibrated Baseline** (L-Turn Map)","**LUCCa** (L-Turn Map)"], dark_invert=[false,false]) }}

The results suggest that using LUCCa's uncertainty estimate improves the success rate in these scenarios by avoiding collision. However, LUCCa only guarantees that `$(1-\alpha)$` percent of true states will be collision-free, so it does not provide a hard guarantee that the planned actions will result in collision-free states. We also note that the baseline did reach the goal faster when it didn't collide.

{% figure(alt=["LUCCa vs Baseline Performance Table"] src=["perf_table.svg"] dark_invert=[true] style="width:100%") %}
**Table.** Comparison of LUCCa vs baseline in four environment over 30 runs (each).
{% end %}


Details about LUCCa's computational overhead (~0.3 ms per planning step) and its empirical coverage (in agreement with the theoretical bounds) can be found in the full paper (Section `$6$`). We also include visualizations of the local conformal scaling factor in Appendix `$B$`.

# BibTeX <small><small>(cite this!)</small></small>

```
@misc{marques2024quantifyingaleatoricepistemicdynamics,
      title={Quantifying Aleatoric and Epistemic Dynamics Uncertainty via Local Conformal Calibration}, 
      author={Luís Marques and Dmitry Berenson},
      year={2024},
      eprint={2409.08249},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2409.08249}, 
}
```