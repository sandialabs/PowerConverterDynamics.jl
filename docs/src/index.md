# PowerConverterDynamics.jl

`PowerConverterDynamics.jl` is a differentiable, averaged dynamic model of DC power conversion between a variable source, a DC common bus, and a battery subsystem.

The model targets early-stage controls and system studies where you need:
- DC voltage/current in and out of a converter.
- Converter loss modeling across changing operating points.
- Bus voltage transients when supply current cannot meet load current.
- Battery charge/discharge interaction with SOC and current limits.
- AD-compatible equations for gradient-based workflows.

## Scope

This package provides a compact dynamic model, not a switching-level electromagnetic transient simulator.

It is designed to be:
- Stable near edge cases (near-zero voltages, SOC limits, command saturation).
- Type-generic (`Float32`, `Float64`, dual numbers).
- Fast enough for repeated simulation loops in optimization or parameter fitting.

## Open-Source Landscape

A review of available open-source tools found strong related ecosystems (power-system dynamics packages, Modelica converter libraries, and battery libraries), but no open-source Julia package that exactly matched this request: a focused, differentiable DC-in/DC-out converter + bus + battery dynamic package with the API and assumptions used here.

See [Theory](theory.md) for links and references.
