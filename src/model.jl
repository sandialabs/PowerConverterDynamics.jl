"""
    battery_ocv(soc, params) -> Real

Battery open-circuit voltage model, using a linearized OCV-SOC relation and smooth
SOC limiting for numerical robustness.
"""
function battery_ocv(soc::Real, params::ConverterParams)
    ee = params.smooth_eps
    soc_eff = _smoothclamp(soc, params.battery_soc_min, params.battery_soc_max, ee)
    half = one(soc_eff) / (one(soc_eff) + one(soc_eff))
    return params.battery_ocv_nominal + params.battery_ocv_slope * (soc_eff - half)
end

"""
    battery_current_limits(soc, params) -> (charge_limit, discharge_limit)

Return smooth SOC-dependent current limits in amperes.
"""
function battery_current_limits(soc::Real, params::ConverterParams)
    ee = params.smooth_eps
    zero_soc = zero(soc)
    one_soc = one(soc)

    soc_eff = _smoothclamp(soc, params.battery_soc_min, params.battery_soc_max, ee)
    soc_buffer = _smoothmax(params.battery_soc_buffer, ee, ee)

    discharge_scale = _smoothclamp((soc_eff - params.battery_soc_min) / soc_buffer, zero_soc, one_soc, ee)
    charge_scale = _smoothclamp((params.battery_soc_max - soc_eff) / soc_buffer, zero_soc, one_soc, ee)

    charge_limit = params.battery_current_limit_charge * charge_scale
    discharge_limit = params.battery_current_limit_discharge * discharge_scale

    return charge_limit, discharge_limit
end

"""
    converter_efficiency(source_voltage, source_current, bus_voltage, params) -> Real

Converter efficiency model with constant, linear, quadratic current losses plus
extra loss sensitivity to source-vs-bus voltage mismatch.
"""
function converter_efficiency(
    source_voltage::Real,
    source_current::Real,
    bus_voltage::Real,
    params::ConverterParams,
)
    ee = params.smooth_eps
    zero_v = zero(source_voltage)

    vin = _smoothmax(source_voltage, zero_v, ee)
    iin = _smoothmax(source_current, zero(source_current), ee)
    vbus = _smoothmax(bus_voltage, zero(bus_voltage), ee)

    p_in = vin * iin
    v_mismatch = _smoothabs(vin - vbus, ee)

    p_loss_model =
        params.converter_loss_constant +
        params.converter_loss_linear * _smoothabs(iin, ee) +
        params.converter_loss_quadratic * iin * iin +
        params.converter_ratio_loss_gain * v_mismatch

    p_ref = _smoothmax(p_in, params.converter_power_floor, ee)
    eta_raw = one(p_ref) - p_loss_model / p_ref

    return _smoothclamp(eta_raw, params.converter_min_efficiency, params.converter_max_efficiency, ee)
end

"""
    evaluate_flows(state, params, input) -> FlowSnapshot

Compute algebraic currents, powers, efficiency, and net bus current imbalance for
one operating point.
"""
function evaluate_flows(state::SystemState, params::ConverterParams, input::SystemInput)
    T = promote_type(
        typeof(state.bus_voltage),
        typeof(state.soc),
        typeof(state.controller_integrator),
        typeof(params.reference_bus_voltage),
        typeof(input.source_voltage),
    )

    s = convert(SystemState{T}, state)
    p = convert(ConverterParams{T}, params)
    u = convert(SystemInput{T}, input)

    ee = p.smooth_eps
    zero_t = zero(T)

    v_bus_safe = _smoothmax(s.bus_voltage, ee, ee)

    i_src_avail = _smoothmax(u.source_current_available, zero_t, ee)
    i_src_cmd = _smoothmax(u.source_current_command, zero_t, ee)
    i_src_cap = _smoothmin(i_src_avail, p.source_current_limit, ee)
    i_src_in = _smoothclamp(i_src_cmd, zero_t, i_src_cap, ee)

    eta = converter_efficiency(u.source_voltage, i_src_in, s.bus_voltage, p)
    p_src_in = _smoothmax(u.source_voltage, zero_t, ee) * i_src_in
    p_src_out_uncapped = eta * p_src_in
    p_src_out = _smoothclamp(p_src_out_uncapped, zero_t, p.source_power_limit, ee)
    p_conv_loss = p_src_in - p_src_out
    i_src_out = p_src_out / v_bus_safe

    v_err = p.reference_bus_voltage - s.bus_voltage
    i_batt_ref = p.bus_controller_kp * v_err + p.bus_controller_ki * s.controller_integrator + u.battery_current_bias
    i_charge_limit, i_discharge_limit = battery_current_limits(s.soc, p)
    i_batt = _smoothclamp(i_batt_ref, -i_charge_limit, i_discharge_limit, ee)

    v_batt_terminal = battery_ocv(s.soc, p) - p.battery_internal_resistance * i_batt
    p_batt_terminal = v_batt_terminal * i_batt

    p_load = s.bus_voltage * u.load_current
    i_leak = p.bus_leak_conductance * s.bus_voltage
    i_imbalance = i_src_out + i_batt - u.load_current - i_leak

    return FlowSnapshot(
        i_src_in,
        i_src_out,
        p_src_in,
        p_src_out,
        p_conv_loss,
        eta,
        i_batt,
        i_batt_ref,
        v_batt_terminal,
        p_batt_terminal,
        p_load,
        i_imbalance,
    )
end

"""
    dynamics(state, params, input) -> (StateDerivative, FlowSnapshot)

Compute state derivatives and accompanying algebraic flow values.
"""
function dynamics(state::SystemState, params::ConverterParams, input::SystemInput)
    flow = evaluate_flows(state, params, input)

    T = promote_type(typeof(flow.bus_imbalance_current), typeof(params.bus_capacitance))
    p = convert(ConverterParams{T}, params)
    s = convert(SystemState{T}, state)
    f = FlowSnapshot{T}(
        T(flow.source_input_current),
        T(flow.source_output_current),
        T(flow.source_input_power),
        T(flow.source_output_power),
        T(flow.converter_loss_power),
        T(flow.converter_efficiency),
        T(flow.battery_current),
        T(flow.battery_current_reference),
        T(flow.battery_terminal_voltage),
        T(flow.battery_terminal_power),
        T(flow.load_power),
        T(flow.bus_imbalance_current),
    )

    ee = p.smooth_eps
    cap = _smoothmax(p.bus_capacitance, ee, ee)
    d_vbus = f.bus_imbalance_current / cap

    d_ctrl = p.reference_bus_voltage - s.bus_voltage

    i_discharge = _smoothmax(f.battery_current, zero(T), ee)
    i_charge = _smoothmax(-f.battery_current, zero(T), ee)
    capacity = _smoothmax(p.battery_capacity_Ah, ee, ee)
    eta_discharge = _smoothmax(p.battery_discharge_coulombic_efficiency, ee, ee)

    d_soc =
        (
            p.battery_charge_coulombic_efficiency * i_charge -
            i_discharge / eta_discharge
        ) / (capacity * T(_SECONDS_PER_HOUR))

    return StateDerivative(d_vbus, d_soc, d_ctrl), f
end
