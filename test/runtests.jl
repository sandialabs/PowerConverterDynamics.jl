using FiniteDiff
using ForwardDiff
using PowerConverterDynamics
using Test

@testset "PowerConverterDynamics.jl" begin
    @testset "Bus Dynamics Under Current Deficit" begin
        params = ConverterParams(
            bus_capacitance = 0.02,
            bus_controller_kp = 0.0,
            bus_controller_ki = 0.0,
            battery_current_limit_charge = 0.0,
            battery_current_limit_discharge = 0.0,
        )

        state = SystemState(bus_voltage = 800.0, soc = 0.5, controller_integrator = 0.0)
        input = SystemInput(
            source_voltage = 900.0,
            source_current_command = 120.0,
            source_current_available = 120.0,
            load_current = 320.0,
            battery_current_bias = 0.0,
        )

        deriv, flow = dynamics(state, params, input)

        @test flow.bus_imbalance_current < 0
        @test deriv.bus_voltage < 0
    end

    @testset "Battery Supports Bus" begin
        params = ConverterParams(
            reference_bus_voltage = 800.0,
            bus_controller_kp = 7.5,
            bus_controller_ki = 0.0,
            battery_current_limit_discharge = 500.0,
        )

        state = SystemState(bus_voltage = 700.0, soc = 0.60, controller_integrator = 0.0)
        input = SystemInput(
            source_voltage = 600.0,
            source_current_command = 0.0,
            source_current_available = 0.0,
            load_current = 80.0,
            battery_current_bias = 0.0,
        )

        deriv, flow = dynamics(state, params, input)

        @test flow.battery_current > 0
        @test deriv.bus_voltage > 0
    end

    @testset "SOC Limits and Zero-Voltage Robustness" begin
        params = ConverterParams(
            reference_bus_voltage = 760.0,
            bus_controller_kp = 5.0,
            bus_controller_ki = 4.0,
            battery_soc_min = 0.10,
            battery_soc_max = 0.90,
            battery_current_limit_charge = 400.0,
            battery_current_limit_discharge = 400.0,
        )

        state = SystemState(bus_voltage = 0.0, soc = 0.5, controller_integrator = 0.0)
        input = SystemInput(
            source_voltage = 0.0,
            source_current_command = 0.0,
            source_current_available = 0.0,
            load_current = 50.0,
            battery_current_bias = 0.0,
        )

        deriv, flow = dynamics(state, params, input)

        @test isfinite(deriv.bus_voltage)
        @test isfinite(flow.source_output_current)
        @test isfinite(flow.converter_efficiency)

        charge_case = simulate(
            SystemState(bus_voltage = 900.0, soc = 0.89, controller_integrator = 0.0),
            params;
            tspan = (0.0, 10.0),
            dt = 0.05,
            source_voltage = 950.0,
            source_current_command = 450.0,
            source_current_available = 450.0,
            load_current = 20.0,
        )

        discharge_case = simulate(
            SystemState(bus_voltage = 650.0, soc = 0.11, controller_integrator = 0.0),
            params;
            tspan = (0.0, 10.0),
            dt = 0.05,
            source_voltage = 650.0,
            source_current_command = 10.0,
            source_current_available = 10.0,
            load_current = 280.0,
        )

        @test charge_case.states[end].soc <= params.battery_soc_max + 1e-6
        @test discharge_case.states[end].soc >= params.battery_soc_min - 1e-6
    end

    @testset "Type Handling" begin
        p32 = convert(ConverterParams{Float32}, ConverterParams())
        s32 = SystemState(bus_voltage = 800.0f0, soc = 0.5f0, controller_integrator = 0.0f0)
        u32 = SystemInput(
            source_voltage = 900.0f0,
            source_current_command = 200.0f0,
            source_current_available = 240.0f0,
            load_current = 210.0f0,
            battery_current_bias = 0.0f0,
        )

        s_next, flow = PowerConverterDynamics.step(s32, p32, u32, 0.01f0)

        @test s_next.bus_voltage isa Float32
        @test flow.converter_efficiency isa Float32
    end

    @testset "ForwardDiff Differentiability" begin
        function final_bus_voltage(kp)
            params = ConverterParams(
                bus_controller_kp = kp,
                bus_controller_ki = 1.0,
                battery_current_limit_discharge = 500.0,
                battery_current_limit_charge = 500.0,
            )

            result = simulate(
                SystemState(bus_voltage = 730.0, soc = 0.65, controller_integrator = 0.0),
                params;
                tspan = (0.0, 0.25),
                dt = 0.01,
                source_voltage = t -> 650.0 + 120.0 * sin(8.0 * t),
                source_current_command = t -> 210.0 + 15.0 * sin(5.0 * t),
                source_current_available = 260.0,
                load_current = t -> 250.0 + 20.0 * sin(6.0 * t),
            )

            return result.states[end].bus_voltage
        end

        grad_ad = ForwardDiff.derivative(final_bus_voltage, 6.0)
        grad_fd = FiniteDiff.finite_difference_derivative(final_bus_voltage, 6.0, Val(:central))
        @test isfinite(grad_ad)
        @test isfinite(grad_fd)
        @test isapprox(grad_ad, grad_fd; rtol = 1e-6, atol = 1e-8)
    end
end
