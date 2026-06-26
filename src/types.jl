const _SECONDS_PER_HOUR = 3600

"""
    ConverterParams{T<:Real}

Parameter set for the averaged DC/DC converter + DC bus + battery dynamic model.

The sign convention is:
- Positive source current moves power from the source side to the DC bus.
- Positive battery current discharges the battery into the DC bus.
- Positive load current draws current from the DC bus.
"""
struct ConverterParams{T<:Real}
    reference_bus_voltage::T
    bus_capacitance::T
    bus_leak_conductance::T

    source_current_limit::T
    source_power_limit::T

    converter_loss_constant::T
    converter_loss_linear::T
    converter_loss_quadratic::T
    converter_ratio_loss_gain::T
    converter_power_floor::T
    converter_min_efficiency::T
    converter_max_efficiency::T

    battery_capacity_Ah::T
    battery_soc_min::T
    battery_soc_max::T
    battery_soc_buffer::T
    battery_current_limit_charge::T
    battery_current_limit_discharge::T
    battery_ocv_nominal::T
    battery_ocv_slope::T
    battery_internal_resistance::T
    battery_charge_coulombic_efficiency::T
    battery_discharge_coulombic_efficiency::T

    bus_controller_kp::T
    bus_controller_ki::T

    smooth_eps::T
end

"""
    ConverterParams(; kwargs...)

Create `ConverterParams` with numerically stable defaults.

Defaults target a high-power DC bus and are intended as a starting point, not a tuned design.
"""
function ConverterParams(
    ;
    reference_bus_voltage::Real = 800.0,
    bus_capacitance::Real = 0.05,
    bus_leak_conductance::Real = 2e-3,
    source_current_limit::Real = 1500.0,
    source_power_limit::Real = 1_000_000.0,
    converter_loss_constant::Real = 200.0,
    converter_loss_linear::Real = 0.8,
    converter_loss_quadratic::Real = 1e-3,
    converter_ratio_loss_gain::Real = 0.08,
    converter_power_floor::Real = 500.0,
    converter_min_efficiency::Real = 0.70,
    converter_max_efficiency::Real = 0.99,
    battery_capacity_Ah::Real = 300.0,
    battery_soc_min::Real = 0.05,
    battery_soc_max::Real = 0.95,
    battery_soc_buffer::Real = 0.08,
    battery_current_limit_charge::Real = 600.0,
    battery_current_limit_discharge::Real = 600.0,
    battery_ocv_nominal::Real = 740.0,
    battery_ocv_slope::Real = 180.0,
    battery_internal_resistance::Real = 0.05,
    battery_charge_coulombic_efficiency::Real = 0.985,
    battery_discharge_coulombic_efficiency::Real = 0.995,
    bus_controller_kp::Real = 8.0,
    bus_controller_ki::Real = 15.0,
    smooth_eps::Real = 1e-6,
)
    T = promote_type(
        typeof(reference_bus_voltage),
        typeof(bus_capacitance),
        typeof(bus_leak_conductance),
        typeof(source_current_limit),
        typeof(source_power_limit),
        typeof(converter_loss_constant),
        typeof(converter_loss_linear),
        typeof(converter_loss_quadratic),
        typeof(converter_ratio_loss_gain),
        typeof(converter_power_floor),
        typeof(converter_min_efficiency),
        typeof(converter_max_efficiency),
        typeof(battery_capacity_Ah),
        typeof(battery_soc_min),
        typeof(battery_soc_max),
        typeof(battery_soc_buffer),
        typeof(battery_current_limit_charge),
        typeof(battery_current_limit_discharge),
        typeof(battery_ocv_nominal),
        typeof(battery_ocv_slope),
        typeof(battery_internal_resistance),
        typeof(battery_charge_coulombic_efficiency),
        typeof(battery_discharge_coulombic_efficiency),
        typeof(bus_controller_kp),
        typeof(bus_controller_ki),
        typeof(smooth_eps),
    )

    return ConverterParams{T}(
        T(reference_bus_voltage),
        T(bus_capacitance),
        T(bus_leak_conductance),
        T(source_current_limit),
        T(source_power_limit),
        T(converter_loss_constant),
        T(converter_loss_linear),
        T(converter_loss_quadratic),
        T(converter_ratio_loss_gain),
        T(converter_power_floor),
        T(converter_min_efficiency),
        T(converter_max_efficiency),
        T(battery_capacity_Ah),
        T(battery_soc_min),
        T(battery_soc_max),
        T(battery_soc_buffer),
        T(battery_current_limit_charge),
        T(battery_current_limit_discharge),
        T(battery_ocv_nominal),
        T(battery_ocv_slope),
        T(battery_internal_resistance),
        T(battery_charge_coulombic_efficiency),
        T(battery_discharge_coulombic_efficiency),
        T(bus_controller_kp),
        T(bus_controller_ki),
        T(smooth_eps),
    )
end

"""
    SystemState{T<:Real}

Dynamic states:
- `bus_voltage`: DC bus voltage in volts.
- `soc`: battery state-of-charge fraction in `[0, 1]`.
- `controller_integrator`: integral state used by the battery bus-voltage controller.
"""
struct SystemState{T<:Real}
    bus_voltage::T
    soc::T
    controller_integrator::T
end

"""
    SystemState(; bus_voltage=800.0, soc=0.5, controller_integrator=0.0)

Create a typed state object.
"""
function SystemState(
    ;
    bus_voltage::Real = 800.0,
    soc::Real = 0.5,
    controller_integrator::Real = 0.0,
)
    T = promote_type(typeof(bus_voltage), typeof(soc), typeof(controller_integrator))
    return SystemState{T}(T(bus_voltage), T(soc), T(controller_integrator))
end

"""
    SystemInput{T<:Real}

External inputs at one simulation instant:
- `source_voltage`: source-side DC voltage (V).
- `source_current_command`: commanded source current into converter (A).
- `source_current_available`: available source current limit from upstream module (A).
- `load_current`: bus load current demand (A).
- `battery_current_bias`: dispatch bias for battery current (A); positive discharges to bus.
"""
struct SystemInput{T<:Real}
    source_voltage::T
    source_current_command::T
    source_current_available::T
    load_current::T
    battery_current_bias::T
end

"""
    SystemInput(; kwargs...)

Create a typed input object.
"""
function SystemInput(
    ;
    source_voltage::Real = 800.0,
    source_current_command::Real = 0.0,
    source_current_available::Real = 0.0,
    load_current::Real = 0.0,
    battery_current_bias::Real = 0.0,
)
    T = promote_type(
        typeof(source_voltage),
        typeof(source_current_command),
        typeof(source_current_available),
        typeof(load_current),
        typeof(battery_current_bias),
    )

    return SystemInput{T}(
        T(source_voltage),
        T(source_current_command),
        T(source_current_available),
        T(load_current),
        T(battery_current_bias),
    )
end

"""
    StateDerivative{T<:Real}

Time derivatives for [`SystemState`](@ref).
"""
struct StateDerivative{T<:Real}
    bus_voltage::T
    soc::T
    controller_integrator::T
end

"""
    FlowSnapshot{T<:Real}

Algebraic values evaluated from a state/input pair.
"""
struct FlowSnapshot{T<:Real}
    source_input_current::T
    source_output_current::T
    source_input_power::T
    source_output_power::T
    converter_loss_power::T
    converter_efficiency::T

    battery_current::T
    battery_current_reference::T
    battery_terminal_voltage::T
    battery_terminal_power::T

    load_power::T
    bus_imbalance_current::T
end

"""
    SimulationResult

Outputs from [`simulate`](@ref):
- `time`: simulation timestamps.
- `states`: dynamic state at each timestamp.
- `flows`: algebraic quantities at each timestamp.
"""
struct SimulationResult{T<:Real,TT<:Real}
    time::Vector{TT}
    states::Vector{SystemState{T}}
    flows::Vector{FlowSnapshot{T}}
end

function Base.convert(::Type{ConverterParams{T}}, params::ConverterParams) where {T<:Real}
    return ConverterParams{T}(
        T(params.reference_bus_voltage),
        T(params.bus_capacitance),
        T(params.bus_leak_conductance),
        T(params.source_current_limit),
        T(params.source_power_limit),
        T(params.converter_loss_constant),
        T(params.converter_loss_linear),
        T(params.converter_loss_quadratic),
        T(params.converter_ratio_loss_gain),
        T(params.converter_power_floor),
        T(params.converter_min_efficiency),
        T(params.converter_max_efficiency),
        T(params.battery_capacity_Ah),
        T(params.battery_soc_min),
        T(params.battery_soc_max),
        T(params.battery_soc_buffer),
        T(params.battery_current_limit_charge),
        T(params.battery_current_limit_discharge),
        T(params.battery_ocv_nominal),
        T(params.battery_ocv_slope),
        T(params.battery_internal_resistance),
        T(params.battery_charge_coulombic_efficiency),
        T(params.battery_discharge_coulombic_efficiency),
        T(params.bus_controller_kp),
        T(params.bus_controller_ki),
        T(params.smooth_eps),
    )
end

function Base.convert(::Type{SystemState{T}}, state::SystemState) where {T<:Real}
    return SystemState{T}(T(state.bus_voltage), T(state.soc), T(state.controller_integrator))
end

function Base.convert(::Type{SystemInput{T}}, input::SystemInput) where {T<:Real}
    return SystemInput{T}(
        T(input.source_voltage),
        T(input.source_current_command),
        T(input.source_current_available),
        T(input.load_current),
        T(input.battery_current_bias),
    )
end
