# USV "surface_constructor" Implementation Summary

## Files Created

### 1. Control Module (`src/modules/usv_control/`)
- `Kconfig` - Module configuration
- `CMakeLists.txt` - Build configuration  
- `module.yaml` - Parameter definitions
- `USVControl.hpp` - Header file
- `USVControl.cpp` - Main control logic

### 2. ROMFS Configuration (`ROMFS/px4fmu_common/init.d/`)
- `rc.usv_defaults` - Default parameters for USV
- `rc.usv_apps` - Startup script

### 3. Airframe Configuration (`ROMFS/px4fmu_common/init.d/airframes/`)
- `60003_surface_constructor` - Airframe parameters

### 4. Updated Files
- `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` - Added airframe registration
- `ROMFS/px4fmu_common/init.d/CMakeLists.txt` - Added USV files registration

## Configuration Details

### Vehicle Type
- **MAV_TYPE**: 14 (Surface Boat)
- **VEHICLE_TYPE**: 3 (Rover)
- **CA_AIRFRAME**: 7 (6DOF Motors - Custom thruster layout)

### Thruster Configuration
- **Count**: 4 thrusters
- **Layout**: Square (0.4m side length)
- **Angle**: 45° inward
- **Bidirectional**: Yes (CA_R_REV = 15)

### Thruster Positions (meters from CG)
```
Thruster 0 (Front-Right):   PX=0.20, PY=0.20, PZ=0.00
Thruster 1 (Front-Left):    PX=0.20, PY=-0.20, PZ=0.00
Thruster 2 (Rear-Right):    PX=-0.20, PY=0.20, PZ=0.00
Thruster 3 (Rear-Left):     PX=-0.20, PY=-0.20, PZ=0.00
```

### Thruster Orientations (unit vectors)
```
Thruster 0: AX=0.707, AY=-0.707, AZ=0.00 (45° inward)
Thruster 1: AX=0.707, AY=0.707,  AZ=0.00 (45° inward)
Thruster 2: AX=0.707, AY=0.707,  AZ=0.00 (45° inward)
Thruster 3: AX=0.707, AY=-0.707, AZ=0.00 (45° inward)
```

### PWM Outputs
- **Channels**: MAIN OUT 1-4
- **Functions**: 101-104 (Motor outputs)
- **Min**: 1100 µs
- **Max**: 1900 µs
- **Disarmed**: 1500 µs

## Build Instructions

```bash
# Build for FMU 6x
make px4_fmu-v6x_default

# Enable USV_CONTROL module (if not default)
# Edit .config or use menuconfig to enable CONFIG_MODULES_USV_CONTROL
```

## Testing

### 1. SITL Testing
```bash
make px4_fmu-v6x_default gazebo-classic_sitl
```

### 2. Load Airframe
In QGroundControl or via MAVLink:
```
param set-default SYS_AUTOSTART 60003
```

### 3. Verify Parameters
```bash
# Check airframe configuration
param show CA_AIRFRAME
param show CA_ROTOR_COUNT
param show MAV_TYPE

# Check thruster geometry
param show CA_ROTOR0_PX
param show CA_ROTOR0_PY
param show CA_ROTOR0_AX
# ... etc for all thrusters
```

### 4. Test Manual Control
- Arm the vehicle
- Test throttle (should control forward thrust)
- Test yaw (should rotate vehicle)

### 5. Actuator Test
```bash
actuator_test
```
This will allow you to test each thruster individually.

## Parameters

### USV Control Parameters
- `USV_THRUST_MAX` - Maximum thrust output (default: 1.0)
- `USV_YAW_RATE_MAX` - Maximum yaw rate in deg/s (default: 90.0)
- `USV_YAW_EXPO` - Yaw stick exponential (default: 0.5)
- `USV_THRUST_EXPO` - Thrust stick exponential (default: 0.5)

## Next Steps

1. **Build and test in SITL** to verify the module loads correctly
2. **Fine-tune parameters** based on your actual vehicle characteristics
3. **Update thruster coefficients** (CT, KM) based on your thruster specifications
4. **Add more control modes** if needed (e.g., stabilized yaw, position control)
5. **Create documentation** for your specific vehicle

## Notes

- The control allocator (CA_AIRFRAME=7) will handle the thruster mixing automatically
- All thrusters are bidirectional for full maneuverability
- The 45° inward angle provides good lateral and rotational control
- MAV_TYPE=14 is the standard MAVLink type for surface boats
