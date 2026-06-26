# Theory

## Modeling Level

The package uses an averaged (non-switching) dynamic model. This follows the standard state-space averaging perspective used for converter-oriented control and system-level studies.

## Core Equations

### Converter Input/Output Power

The source-side current command is smoothly limited by source availability and converter limits:

```math
I_{src,in} = \operatorname{sat}_{smooth}(I_{cmd}, 0, I_{avail}\land I_{lim})
```

Input power and output power are:

```math
P_{in} = V_{src} I_{src,in}, \quad P_{out} = \eta P_{in}
```

with efficiency modeled from constant + current + voltage-mismatch loss terms and smoothly bounded:

```math
\eta = \operatorname{sat}_{smooth}(\eta_{raw}, \eta_{min}, \eta_{max})
```

### DC Bus Dynamics

The bus capacitor dynamics are:

```math
C_{bus}\,\dot V_{bus} = I_{src,out} + I_{batt} - I_{load} - G_{leak}V_{bus}
```

This directly captures bus droop/rise when supply and demand currents do not balance.

### Battery Model

Battery terminal voltage uses an OCV + resistance form:

```math
V_{batt} = OCV(SOC) - R_{int} I_{batt}
```

SOC evolves by coulomb counting with separate charge/discharge efficiencies:

```math
\dot{SOC} = \frac{\eta_c I_{chg} - I_{dis}/\eta_d}{3600\,Q_{Ah}}
```

Current limits are SOC-dependent and smoothly reduced near SOC bounds.

## Differentiability Notes

Hard clipping and absolute values are replaced with smooth approximations (`smoothmax`, `smoothmin`, smooth clamp), so gradients remain well-defined near operating limits.

## Literature and Open-Source Review

### Literature Anchors

- Middlebrook, R. D., and Cuk, S. (1976), *A General Unified Approach to Modeling Switching-Converter Power Stages*, IEEE PESC. DOI: <https://doi.org/10.1109/PESC.1976.7072895>
- Vorperian, V. (1990), *Simplified Analysis of PWM Converters Using Model of PWM Switch: Parts I & II*, IEEE Transactions on Aerospace and Electronic Systems. DOI: <https://doi.org/10.1109/63.53500>

These are foundational references for averaged converter modeling used in control-oriented simulation.

### Related Open-Source Packages Reviewed

- `PowerSimulationsDynamics.jl` (dynamic power-system simulation in Julia): <https://nrel-sienna.github.io/PowerSimulationsDynamics.jl/v0.7/>
- `PowerDynamics.jl` (dynamic power-grid simulation in Julia): <https://juliaenergy.github.io/PowerDynamics.jl/stable/>
- `PowerModelsACDC.jl` (steady-state optimization/power flow, not dynamic simulation): <https://electa-git.github.io/PowerModelsACDC.jl/dev/>
- Modelica `Electrical.PowerConverters`: <https://build.openmodelica.org/Documentation/Modelica.Electrical.PowerConverters.html>
- Modelica `Electrical.Batteries`: <https://build.openmodelica.org/Documentation/Modelica.Electrical.Batteries.html>
- `PyBaMM` equivalent-circuit battery models: <https://docs.pybamm.org/en/stable/source/examples/notebooks/models/equivalent-circuit-thevenin.html>

### Exact-Match Assessment

No open-source Julia package was found that exactly combines all requested features in one focused API:
- differentiable DC-in/DC-out converter dynamics,
- explicit bus-capacitor voltage dynamics under current mismatch,
- converter loss modeling,
- battery SOC/charge-discharge dynamics and limits,
- compact package-level interface for control/optimization workflows.

That gap is what `PowerConverterDynamics.jl` is designed to fill.

## SIRENOpt Integration Boundary

In SIRENOpt, this package sits between variable renewable/generator sources, the
common DC bus, and battery storage. The ontology consumes source voltage/current,
converter command, output power, bus voltage, battery current, SOC, and losses.
Keeping those quantities explicit makes it possible to replace this averaged model
with a higher-fidelity converter later without changing dispatch or plant-level
constraints.
