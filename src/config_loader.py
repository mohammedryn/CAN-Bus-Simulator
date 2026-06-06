"""
Configuration loader for EV CAN simulation.

Uses Pydantic v2 to validate all YAML configuration files at startup.
Provides a single public function: load_configs(config_dir: Path) -> AppConfig
"""

from pathlib import Path
from typing import Dict

import yaml
from pydantic import BaseModel, Field


# =============================================================================
# Battery Configuration Models
# =============================================================================

class PackConfig(BaseModel):
    """Battery pack physical configuration."""

    cells_series: int = Field(gt=0, description="Number of cells in series")
    cells_parallel: int = Field(gt=0, description="Number of cells in parallel")
    cell_capacity_ah: float = Field(gt=0, description="Cell capacity in Ah")
    nominal_voltage_v: float = Field(gt=0, description="Nominal voltage per cell in V")


class ECM1RCConfig(BaseModel):
    """Equivalent circuit model (1st order RC)."""

    r0_ohm: float = Field(gt=0, description="Series resistance in Ohm")
    r1_ohm: float = Field(gt=0, description="RC branch resistance in Ohm")
    c1_farad: float = Field(gt=0, description="RC branch capacitance in Farad")
    temp_coeff_per_c: float = Field(
        ge=0, description="Temperature coefficient per degree Celsius"
    )


class ProtectionConfig(BaseModel):
    """Battery protection limits."""

    max_cell_voltage_v: float = Field(le=4.30, description="Max cell voltage in V")
    min_cell_voltage_v: float = Field(ge=2.50, description="Min cell voltage in V")
    max_temp_c: float = Field(le=80.0, description="Max temperature in C")
    max_discharge_current_a: float = Field(gt=0, description="Max discharge current in A")
    max_charge_current_a: float = Field(gt=0, description="Max charge current in A")
    precharge_resistor_ohm: float = Field(gt=0, description="Pre-charge resistor in Ohm")
    dc_link_capacitance_f: float = Field(gt=0, description="DC link capacitance in F")
    precharge_timeout_s: float = Field(gt=0, description="Pre-charge timeout in seconds")


class ThermalConfig(BaseModel):
    """Thermal management configuration."""

    thermal_mass_j_per_k: float = Field(gt=0, description="Thermal mass in J/K")
    passive_convection_w_per_k: float = Field(
        ge=0, description="Passive convection in W/K"
    )
    active_cooling_w_per_k: float = Field(
        ge=0, description="Active cooling power in W/K"
    )
    active_cooling_threshold_c: float = Field(
        description="Threshold temperature for active cooling in C"
    )


class BatteryConfig(BaseModel):
    """Complete battery configuration."""

    pack: PackConfig
    ecm_1rc: ECM1RCConfig
    ocv_soc_table: list[list[float]] = Field(
        description="OCV vs SOC lookup table: [[soc_fraction, cell_voltage_v], ...]"
    )
    thermal: ThermalConfig
    protection: ProtectionConfig


# =============================================================================
# Vehicle Configuration Models
# =============================================================================

class ChassisConfig(BaseModel):
    """Vehicle chassis configuration."""

    mass_kg: float = Field(gt=0, description="Vehicle mass in kg")
    wheel_radius_m: float = Field(gt=0, description="Wheel radius in m")
    gear_ratio: float = Field(gt=0, description="Gear ratio (dimensionless)")
    drag_coefficient: float = Field(gt=0, description="Drag coefficient")
    frontal_area_m2: float = Field(gt=0, description="Frontal area in m^2")
    rolling_resistance: float = Field(gt=0, description="Rolling resistance coefficient")
    air_density_kg_m3: float = Field(gt=0, description="Air density in kg/m^3")


class MotorConfig(BaseModel):
    """Electric motor and inverter configuration."""

    peak_torque_nm: float = Field(gt=0, description="Peak torque in Nm")
    peak_power_w: float = Field(gt=0, description="Peak power in W")
    max_rpm: float = Field(gt=0, description="Maximum RPM")
    efficiency: float = Field(gt=0, description="Motor efficiency (0-1)")
    motor_thermal_r_k_per_w: float = Field(
        gt=0, description="Motor thermal resistance in K/W"
    )
    motor_thermal_c_j_per_k: float = Field(
        gt=0, description="Motor thermal capacitance in J/K"
    )
    inverter_thermal_r_k_per_w: float = Field(
        gt=0, description="Inverter thermal resistance in K/W"
    )
    inverter_thermal_c_j_per_k: float = Field(
        gt=0, description="Inverter thermal capacitance in J/K"
    )
    derating_start_c: float = Field(
        description="Start of derating temperature in C"
    )
    derating_end_c: float = Field(description="End of derating temperature in C")


class VCUConfig(BaseModel):
    """Vehicle Control Unit configuration."""

    torque_slew_rate_nm_per_s: float = Field(
        gt=0, description="Torque slew rate in Nm/s"
    )
    throttle_plausibility_threshold_pct: float = Field(
        ge=0, description="Throttle plausibility threshold in %"
    )
    brake_plausibility_threshold_pct: float = Field(
        ge=0, description="Brake plausibility threshold in %"
    )


class VehicleConfig(BaseModel):
    """Complete vehicle configuration."""

    chassis: ChassisConfig
    motor: MotorConfig
    vcu: VCUConfig


# =============================================================================
# CAN Configuration Models
# =============================================================================

class BusConfig(BaseModel):
    """CAN bus configuration."""

    baudrate_bps: int = Field(gt=0, description="CAN bus baudrate in bits/s")


class MessageConfig(BaseModel):
    """CAN message definition."""

    id: int = Field(description="CAN message ID (decimal/hex)")
    cycle_ms: int = Field(description="Message cycle time in milliseconds")
    dlc: int = Field(description="Data Length Code (0-8)")


class WatchdogConfig(BaseModel):
    """CAN watchdog definition."""

    can_id: int = Field(description="CAN message ID to monitor")
    timeout_ms: int = Field(gt=0, description="Watchdog timeout in milliseconds")
    dtc: str = Field(description="Diagnostic Trouble Code if timeout occurs")


class DiagnosticsConfig(BaseModel):
    """Diagnostics addressing configuration."""

    functional_addr: int = Field(description="Functional addressing ID")
    bms_addr: int = Field(description="BMS physical addressing ID")
    vcu_addr: int = Field(description="VCU physical addressing ID")
    mcu_addr: int = Field(description="MCU physical addressing ID")


class PhysicsConfig(BaseModel):
    """Physics simulation configuration."""

    integrator_step_ms: int = Field(gt=0, description="Integrator step size in ms")


class CANConfig(BaseModel):
    """Complete CAN configuration."""

    bus: BusConfig
    messages: Dict[str, MessageConfig] = Field(
        description="Named CAN message definitions"
    )
    watchdogs: Dict[str, WatchdogConfig] = Field(description="Watchdog definitions")
    diagnostics: DiagnosticsConfig
    physics: PhysicsConfig


# =============================================================================
# Top-Level Application Configuration
# =============================================================================

class AppConfig(BaseModel):
    """Complete application configuration."""

    battery: BatteryConfig
    vehicle: VehicleConfig
    can: CANConfig


# =============================================================================
# Configuration Loader
# =============================================================================

def load_configs(config_dir: Path) -> AppConfig:
    """
    Load and validate all configuration files.

    Loads battery.yaml, vehicle.yaml, and can_config.yaml from the specified
    directory, validates them using Pydantic v2, and returns a single AppConfig object.

    Args:
        config_dir: Path to the configuration directory

    Returns:
        AppConfig: Validated application configuration

    Raises:
        FileNotFoundError: If any required YAML file is missing
        ValidationError: If any configuration field is invalid or missing
        yaml.YAMLError: If YAML parsing fails

    """
    config_dir = Path(config_dir)

    # Load battery configuration
    battery_path = config_dir / "battery.yaml"
    with open(battery_path, "r") as f:
        battery_data = yaml.safe_load(f)
    battery_config = BatteryConfig(**battery_data)

    # Load vehicle configuration
    vehicle_path = config_dir / "vehicle.yaml"
    with open(vehicle_path, "r") as f:
        vehicle_data = yaml.safe_load(f)
    vehicle_config = VehicleConfig(**vehicle_data)

    # Load CAN configuration
    can_path = config_dir / "can_config.yaml"
    with open(can_path, "r") as f:
        can_data = yaml.safe_load(f)
    can_config = CANConfig(**can_data)

    # Combine into top-level AppConfig
    return AppConfig(battery=battery_config, vehicle=vehicle_config, can=can_config)
