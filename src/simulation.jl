@inline _profile_value(profile::Real, _t::Real) = profile
@inline _profile_value(profile, t::Real) = profile(t)

function _input_at(
    t::Real;
    source_voltage,
    source_current_command,
    source_current_available,
    load_current,
    battery_current_bias,
)
    return SystemInput(
        source_voltage = _profile_value(source_voltage, t),
        source_current_command = _profile_value(source_current_command, t),
        source_current_available = _profile_value(source_current_available, t),
        load_current = _profile_value(load_current, t),
        battery_current_bias = _profile_value(battery_current_bias, t),
    )
end

"""
    step(state, params, input, dt) -> (next_state, flow)

Advance one explicit-Euler step with smooth state limiting.
"""
function step(state::SystemState, params::ConverterParams, input::SystemInput, dt::Real)
    T = promote_type(typeof(state.bus_voltage), typeof(params.reference_bus_voltage), typeof(dt))
    s = convert(SystemState{T}, state)
    p = convert(ConverterParams{T}, params)
    u = convert(SystemInput{T}, input)
    dt_t = T(dt)

    deriv, flow = dynamics(s, p, u)

    next_vbus = _smoothmax(s.bus_voltage + dt_t * deriv.bus_voltage, zero(T), p.smooth_eps)
    next_soc = _smoothclamp(s.soc + dt_t * deriv.soc, p.battery_soc_min, p.battery_soc_max, p.smooth_eps)
    next_ctrl = s.controller_integrator + dt_t * deriv.controller_integrator

    next_state = SystemState{T}(next_vbus, next_soc, next_ctrl)
    return next_state, flow
end

"""
    simulate(state0, params; kwargs...) -> SimulationResult

Run a time-domain simulation using fixed-size steps and user-provided profiles.

Each profile kwarg can be either:
- A constant real value.
- A callable `f(t)` returning the value at time `t`.

Keyword arguments:
- `tspan=(0.0, 1.0)`
- `dt=1e-3`
- `source_voltage=params.reference_bus_voltage`
- `source_current_command=0.0`
- `source_current_available=params.source_current_limit`
- `load_current=0.0`
- `battery_current_bias=0.0`
"""
function simulate(
    state0::SystemState,
    params::ConverterParams;
    tspan::Tuple{<:Real,<:Real} = (0.0, 1.0),
    dt::Real = 1e-3,
    source_voltage = params.reference_bus_voltage,
    source_current_command = 0.0,
    source_current_available = params.source_current_limit,
    load_current = 0.0,
    battery_current_bias = 0.0,
)
    dt > 0 || throw(ArgumentError("dt must be positive."))

    t0 = float(tspan[1])
    tf = float(tspan[2])
    tf >= t0 || throw(ArgumentError("tspan must satisfy tspan[2] >= tspan[1]."))

    T = promote_type(
        typeof(state0.bus_voltage),
        typeof(state0.soc),
        typeof(state0.controller_integrator),
        typeof(params.reference_bus_voltage),
    )

    state = convert(SystemState{T}, state0)
    p = convert(ConverterParams{T}, params)

    time = Float64[t0]
    states = SystemState{T}[state]

    initial_input = _input_at(
        t0;
        source_voltage = source_voltage,
        source_current_command = source_current_command,
        source_current_available = source_current_available,
        load_current = load_current,
        battery_current_bias = battery_current_bias,
    )
    flows = FlowSnapshot{T}[evaluate_flows(state, p, convert(SystemInput{T}, initial_input))]

    t = t0
    dt_f = float(dt)
    tol = max(eps(tf), eps(dt_f))

    while t < tf - tol
        step_dt = min(dt_f, tf - t)

        input = _input_at(
            t;
            source_voltage = source_voltage,
            source_current_command = source_current_command,
            source_current_available = source_current_available,
            load_current = load_current,
            battery_current_bias = battery_current_bias,
        )

        state, _ = step(state, p, convert(SystemInput{T}, input), T(step_dt))

        t += step_dt
        push!(time, t)
        push!(states, state)

        next_input = _input_at(
            t;
            source_voltage = source_voltage,
            source_current_command = source_current_command,
            source_current_available = source_current_available,
            load_current = load_current,
            battery_current_bias = battery_current_bias,
        )
        push!(flows, evaluate_flows(state, p, convert(SystemInput{T}, next_input)))
    end

    return SimulationResult{T,Float64}(time, states, flows)
end
