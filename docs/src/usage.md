# Usage

## 1. Configure Parameters

```julia
using PowerConverterDynamics

params = ConverterParams(
    reference_bus_voltage = 800.0,
    source_current_limit = 1200.0,
    source_power_limit = 900_000.0,
    converter_loss_constant = 150.0,
    converter_loss_linear = 0.6,
    converter_loss_quadratic = 8e-4,
    battery_capacity_Ah = 320.0,
    battery_current_limit_charge = 500.0,
    battery_current_limit_discharge = 550.0,
)
```

## 2. Set Initial Conditions

```julia
state = SystemState(
    bus_voltage = 790.0,
    soc = 0.55,
    controller_integrator = 0.0,
)
```

## 3. Run One Step (for custom solvers)

```julia
input = SystemInput(
    source_voltage = 680.0,
    source_current_command = 260.0,
    source_current_available = 300.0,
    load_current = 290.0,
    battery_current_bias = 0.0,
)

next_state, flow = PowerConverterDynamics.step(state, params, input, 0.01)
```

## 4. Run a Full Simulation

Profiles can be constants or functions of time.

```julia
result = simulate(
    state,
    params;
    tspan = (0.0, 5.0),
    dt = 0.005,
    source_voltage = t -> 540.0 + 260.0 * abs(sin(0.8 * t)),
    source_current_command = t -> 280.0 + 20.0 * sin(1.5 * t),
    source_current_available = 320.0,
    load_current = t -> 260.0 + 120.0 * max(0.0, sin(0.6 * t)),
    battery_current_bias = 0.0,
)
```

## Sign Convention

- `source_current_* > 0`: power from source to bus.
- `battery_current > 0`: battery discharges to bus.
- `load_current > 0`: load draws from bus.

A negative `bus_imbalance_current` means demand exceeds supply at that instant, so bus voltage drops according to bus capacitance.
