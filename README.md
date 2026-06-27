# PowerConverterDynamics.jl

Differentiable DC power-conversion dynamics for a variable source, common DC bus, and battery module.

## What This Package Models

- DC source voltage/current into a converter.
- Converter output voltage/current effects on a common DC bus.
- Converter losses across operating conditions.
- Bus voltage transients from current imbalance.
- Battery charge/discharge current, terminal voltage, and SOC dynamics.
- SOC- and hardware-limited current envelopes with smooth clipping.

## Why a New Package

Related open-source tools exist (power-system dynamic simulators, Modelica converter/battery libraries, battery modeling packages), but no open-source Julia package was found that provides this exact differentiable DC-in/DC-out converter + bus + battery dynamics workflow as a focused package API.

See `docs/src/theory.md` for references and links.

## Installation

`PowerConverterDynamics.jl` is distributed as an unregistered Julia package.
Install it from the public repository URL:

```julia
using Pkg
Pkg.add(url = "https://github.com/sandialabs/PowerConverterDynamics.jl")
```

For local development from a checkout:

```julia
using Pkg
Pkg.develop(path="/path/to/PowerConverterDynamics.jl")
```

## Quick Example

```julia
using PowerConverterDynamics

params = ConverterParams(reference_bus_voltage = 800.0)
state0 = SystemState(bus_voltage = 780.0, soc = 0.60, controller_integrator = 0.0)

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

@show result.states[end].bus_voltage
@show result.states[end].soc
@show result.flows[end].converter_efficiency
```

## Key API

- `ConverterParams`
- `SystemState`
- `SystemInput`
- `evaluate_flows`
- `dynamics`
- `PowerConverterDynamics.step`
- `simulate`

## Documentation

Build docs locally:

```julia
julia --project=docs -e 'using Pkg; Pkg.develop(PackageSpec(path=pwd())); Pkg.instantiate(); include("docs/make.jl")'
```

## Testing

```julia
julia --project -e 'using Pkg; Pkg.test()'
```

## License

MIT. See `LICENSE`.
