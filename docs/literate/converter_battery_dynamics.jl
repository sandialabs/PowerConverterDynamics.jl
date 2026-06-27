# # Converter + Battery Dynamic Walkthrough
#
# This literate example runs a nontrivial dynamic scenario and generates plots that
# highlight key model capabilities:
#
# - wild source-side voltage feeding a common DC bus,
# - current saturation and conversion losses,
# - bus voltage droop when supply cannot meet load,
# - battery support reducing bus-voltage excursions,
# - SOC and efficiency trends over time.

using PowerConverterDynamics

# Keep GR headless-friendly in local/CI doc builds.
ENV["GKSwstype"] = "100"
using Plots

gr()
default(size = (1200, 800), lw = 2)

# ## Scenario Setup
#
# We compare two cases:
#
# - **Active battery support**: PI bus-voltage control and battery current limits enabled.
# - **No battery support**: battery current limits set to zero.

common = (
    reference_bus_voltage = 780.0,
    bus_capacitance = 0.03,
    bus_leak_conductance = 2.5e-3,
    source_current_limit = 900.0,
    source_power_limit = 650_000.0,
    converter_loss_constant = 180.0,
    converter_loss_linear = 0.7,
    converter_loss_quadratic = 9e-4,
    converter_ratio_loss_gain = 0.07,
    battery_capacity_Ah = 260.0,
    battery_soc_min = 0.10,
    battery_soc_max = 0.90,
)

params_active = ConverterParams(
    ;
    common...,
    bus_controller_kp = 10.0,
    bus_controller_ki = 18.0,
    battery_current_limit_charge = 500.0,
    battery_current_limit_discharge = 500.0,
)

params_no_battery = ConverterParams(
    ;
    common...,
    bus_controller_kp = 0.0,
    bus_controller_ki = 0.0,
    battery_current_limit_charge = 0.0,
    battery_current_limit_discharge = 0.0,
)

state0 = SystemState(bus_voltage = 760.0, soc = 0.62, controller_integrator = 0.0)

source_voltage_profile(t) = 430.0 + 360.0 * abs(sin(0.45 * t)) + 20.0 * sin(3.0 * t)
source_current_command_profile(t) = 310.0 + 15.0 * sin(1.7 * t)
source_current_available_profile(t) = 260.0 + 90.0 * max(0.0, sin(0.3 * t))
load_current_profile(t) = 220.0 + (t > 6.0 ? 160.0 : 0.0) + 80.0 * max(0.0, sin(0.8 * t))

result_active = simulate(
    state0,
    params_active;
    tspan = (0.0, 12.0),
    dt = 0.02,
    source_voltage = source_voltage_profile,
    source_current_command = source_current_command_profile,
    source_current_available = source_current_available_profile,
    load_current = load_current_profile,
)

result_no_battery = simulate(
    state0,
    params_no_battery;
    tspan = (0.0, 12.0),
    dt = 0.02,
    source_voltage = source_voltage_profile,
    source_current_command = source_current_command_profile,
    source_current_available = source_current_available_profile,
    load_current = load_current_profile,
)

# ## Extract Time Series

t = result_active.time
v_bus_active = getfield.(result_active.states, :bus_voltage)
v_bus_no_battery = getfield.(result_no_battery.states, :bus_voltage)
soc_active = getfield.(result_active.states, :soc)

i_src_bus = getfield.(result_active.flows, :source_output_current)
i_batt = getfield.(result_active.flows, :battery_current)
i_load = load_current_profile.(t)

eff = getfield.(result_active.flows, :converter_efficiency)
p_src_bus_kw = getfield.(result_active.flows, :source_output_power) ./ 1000
p_loss_kw = getfield.(result_active.flows, :converter_loss_power) ./ 1000
p_load_kw = getfield.(result_active.flows, :load_power) ./ 1000
p_batt_kw = getfield.(result_active.flows, :battery_terminal_power) ./ 1000

# ## Plots

p1 = plot(
    t,
    v_bus_active;
    label = "Bus voltage (battery support)",
    xlabel = "Time (s)",
    ylabel = "Voltage (V)",
    title = "Bus Voltage Regulation",
)
plot!(p1, t, v_bus_no_battery; label = "Bus voltage (no battery)", ls = :dash)
plot!(p1, t, fill(params_active.reference_bus_voltage, length(t)); label = "Bus reference", ls = :dot)

p2 = plot(
    t,
    i_src_bus;
    label = "Source current to bus",
    xlabel = "Time (s)",
    ylabel = "Current (A)",
    title = "Current Flows",
)
plot!(p2, t, i_batt; label = "Battery current (+ discharge)")
plot!(p2, t, i_load; label = "Load current")

p3 = plot(
    t,
    p_src_bus_kw;
    label = "Source output power",
    xlabel = "Time (s)",
    ylabel = "Power (kW)",
    title = "Power and Losses",
)
plot!(p3, t, p_load_kw; label = "Load power")
plot!(p3, t, p_batt_kw; label = "Battery terminal power")
plot!(p3, t, p_loss_kw; label = "Converter loss power")

p4 = plot(
    t,
    soc_active;
    label = "Battery SOC",
    xlabel = "Time (s)",
    ylabel = "Per-unit",
    title = "SOC and Converter Efficiency",
)
plot!(p4, t, eff; label = "Converter efficiency")

generated_figure_dir = joinpath(@__DIR__, "..", "src", "generated")
mkpath(generated_figure_dir)

overview = plot(p1, p2, p3, p4; layout = (2, 2), legend = :topright)
overview_path = joinpath(generated_figure_dir, "converter_dynamics_overview.svg")
savefig(overview, overview_path)

net_supply_current = i_src_bus .+ i_batt .- i_load
p_deficit = plot(
    t,
    net_supply_current;
    label = "Supply - load current (before bus leak)",
    xlabel = "Time (s)",
    ylabel = "Current (A)",
    title = "Current Deficit Drives Bus Dynamics",
)
hline!(p_deficit, [0.0]; label = "Zero balance", ls = :dot)
deficit_path = joinpath(generated_figure_dir, "bus_current_deficit.svg")
savefig(p_deficit, deficit_path)

# ## Summary Metrics

min_v_active = minimum(v_bus_active)
min_v_no_batt = minimum(v_bus_no_battery)
max_abs_ibatt = maximum(abs.(i_batt))
avg_eta = sum(eff) / length(eff)

println("Minimum bus voltage (battery support): $(round(min_v_active, digits=1)) V")
println("Minimum bus voltage (no battery): $(round(min_v_no_batt, digits=1)) V")
println("Peak |battery current|: $(round(max_abs_ibatt, digits=1)) A")
println("Average converter efficiency: $(round(100 * avg_eta, digits=2)) %")

# ## Generated Figures
#
# ![Converter dynamics overview](converter_dynamics_overview.svg)
#
# ![Bus current deficit](bus_current_deficit.svg)
