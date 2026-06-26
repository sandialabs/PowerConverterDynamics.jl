# Quickstart

```julia
using PowerConverterDynamics

params = ConverterParams(
    reference_bus_voltage = 800.0,
    bus_controller_kp = 8.0,
    bus_controller_ki = 10.0,
)

state0 = SystemState(
    bus_voltage = 780.0,
    soc = 0.60,
    controller_integrator = 0.0,
)

result = simulate(
    state0,
    params;
    tspan = (0.0, 2.0),
    dt = 0.01,
    source_voltage = t -> 620.0 + 120.0 * sin(2.0 * t),
    source_current_command = 240.0,
    source_current_available = 280.0,
    load_current = t -> 250.0 + 60.0 * sin(3.0 * t),
)

final_state = result.states[end]
final_flow = result.flows[end]

println("Final bus voltage: $(round(final_state.bus_voltage, digits=2)) V")
println("Final SOC: $(round(final_state.soc, digits=4))")
println("Final converter efficiency: $(round(100 * final_flow.converter_efficiency, digits=2)) %")
```

`result` includes full time-series arrays:
- `result.time`
- `result.states`
- `result.flows`
