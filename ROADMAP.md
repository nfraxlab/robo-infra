# robo-infra Production Readiness Plan

> From Alpha (0.1.x) -> Beta (0.2.x) -> Production (1.0.0)

## Current Status (v0.2.0-beta)

| Metric | Current | Target (1.0.0) | Gap | Status |
|--------|---------|----------------|-----|--------|
| Version | **0.2.0** | 1.0.0 | Beta release | [OK] Phase 5.22 Complete |
| Tests | 5,861 | 5,500+ | [OK] Exceeded | [OK] Good |
| Coverage | **74%+** | 80%+ | **6% gap** | [!] Hardware exceptions |
| Docs | 10 pages | 20+ pages | 10 pages | Phase 6 |
| Examples | 6 projects | 6 projects | [OK] Met | [OK] |
| Platforms | 7 | 8+ | [OK] Met | [OK] |
| Controllers | 15+ | 15+ | [OK] Met | [OK] |
| Ruff Errors | 0 | 0 | [OK] Clean | [OK] |
| Mypy Errors | 0 | 0 | [OK] Clean | [OK] |
| py.typed | [OK] | [OK] | Done | [OK] Phase 5.12 Complete |
| SECURITY.md | [OK] | [OK] | Done | [OK] |
| Observability | [OK] | [OK] | Done | [OK] Phase 5.14 Complete |
| Benchmarks | [OK] | [OK] | 57 tests | [OK] Phase 5.19 Complete |
| Hardware Tests | [OK] | [OK] | 41 tests | [OK] Phase 5.20 Complete |
| Integration Tests | [OK] | [OK] | 190 tests | [OK] Phase 5.21 Complete |

### Coverage Summary (Phase 5.5-5.11 Complete)

**Overall**: 5,167 tests passed, 218 skipped, 73.86% coverage

| Category | Modules | Status | Notes |
|----------|---------|--------|-------|
| **Safety** | estop 98%, limits 100%, monitor 90%, watchdog 99% | [OK] Exceeds | All targets met |
| **Motion** | fusion 95%, pid 98%, path_planning 92% | [OK] Exceeds | All targets met |
| **Sensors** | camera 90%, imu 90%, distance 84% | [OK] Exceeds | Testable modules pass |
| **Core** | All modules >80% | [OK] Exceeds | Strong foundation |
| **CLI** | 100% | [OK] Complete | 55 tests |
| **Vision** | 15-41% | [!] Hardware | Requires OpenCV |
| **Cameras** | 33-51% | [!] Hardware | Requires SDKs |
| **Platforms** | 47-55% | [!] Hardware | Requires GPIO |
| **Protocols** | 55% | [!] Hardware | Requires CAN bus |

### Production Readiness Gaps (Phase 5.12-5.22)

| Gap | Phase | Priority | Estimate |
|-----|-------|----------|----------|
| Version sync (__init__.py ≠ pyproject.toml) | 5.12 | P0 | 10 min |
| SECURITY.md missing | 5.12 | P0 | 15 min |
| py.typed marker missing | 5.12 | P0 | 5 min |
| Public API stability (__all__ audit) | 5.13 | P0 | 2-3 hrs |
| Observability integration (svc-infra) | 5.14 | P1 | 4-5 hrs |
| Error handling (timeouts, retry, circuit breaker) | 5.15 | P0 | 3-4 hrs |
| Resource management (context managers) | 5.16 | P1 | 2-3 hrs |
| Hardware abstraction hardening | 5.17 | P1 | 3-4 hrs |
| Security hardening (Bandit issues) | 5.18 | P1 | 2-3 hrs |
| ~~Performance benchmarks~~ | ~~5.19~~ | ~~P2~~ | [OK] Complete |
| Hardware validation infrastructure | 5.20 | P2 | 3-4 hrs |
| Integration test enhancement | 5.21 | P1 | 2-3 hrs |
| Release preparation | 5.22 | P0 | 2-3 hrs |
| ~~Real hardware bus implementations~~ | ~~5.23~~ | ~~P0~~ | [OK] Complete |
| ~~Dependency unbundling (optional AI/API)~~ | ~~5.24~~ | ~~P0~~ | [OK] Complete |
| ~~Hardware extras expansion~~ | ~~5.25~~ | ~~P1~~ | [OK] Complete |
| **Total** | 5.12-5.25 | | **~29-39 hrs** |

### What's Done [OK]

| Component | Status | Tests | Coverage |
|-----------|--------|-------|----------|
| Core Abstractions | [OK] Complete | 200+ | 85%+ |
| Actuators | [OK] Complete | 300+ | 90%+ |
| Sensors (testable) | [OK] Complete | 300+ | 84-90% |
| Drivers (testable) | [OK] Complete | 350+ | 80%+ |
| Safety | [OK] Complete | 200+ | 90-100% |
| Controllers | [OK] Complete | 120+ | 80%+ |
| Motion (testable) | [OK] Complete | 220+ | 88-98% |
| CLI | [OK] Complete | 55 | 100% |
| **Examples** | [OK] Complete | 17 | 100% |
| **Phase 5.5-5.11** | [OK] Complete | 5,167 | 73.86% |

### What's Missing [X] (Production Readiness)

| Component | Status | Priority | Phase |
|-----------|--------|----------|-------|
| **Critical Fixes** (version, SECURITY.md, py.typed) | [OK] Complete | **P0 - Critical** | **5.12** |
| **API Stability** (__all__ audit, cleanup) | [OK] Complete | **P0 - Critical** | **5.13** |
| **Observability** (metrics, health, logging) | [OK] Complete | **P1 - High** | **5.14** |
| **Error Handling** (timeouts, retry, circuit breaker) | [OK] Complete | **P0 - Critical** | **5.15** |
| **Resource Management** (context managers, cleanup) | [OK] Complete | **P1 - High** | **5.16** |
| **Hardware Abstraction** (simulation, detection) | [OK] Complete | **P1 - High** | **5.17** |
| **Security Hardening** (Bandit, validation) | [OK] Complete | **P1 - High** | **5.18** |
| **Benchmarks** (performance targets) | [OK] Complete | **P2 - Medium** | **5.19** |
| **Hardware Validation** (real hardware tests) | [OK] Complete | **P2 - Medium** | **5.20** |
| **Integration Testing** (E2E scenarios) | [OK] Complete | **P1 - High** | **5.21** |
| **Release Preparation** (changelog, publish) | [OK] Complete | **P0 - Critical** | **5.22** |
| **Real Hardware Buses** (SMBus2, SpiDev, PySerial, CAN) | [OK] Complete | **P0 - Critical** | **5.23** |
| **Dependency Unbundling** (optional ai-infra/svc-infra) | [OK] Complete | **P1 - High** | **5.24** |
| **Hardware Extras Expansion** (python-can, opencv) | [OK] Complete | **P1 - High** | **5.25** |
| **Platform Support** (detection, factory, GPIO, I2C, SPI) | [OK] Complete | **P2 - Medium** | **8** |
| **Linux Generic Bus Access** (I2C/SPI/UART real hardware) | [OK] Complete | **P0 - Critical** | **8.8.1** |
| **BeagleBone Bus Access** (I2C/SPI/UART real hardware) | [OK] Complete | **P0 - Critical** | **8.8.2** |
| **Remove Deprecated Code** (clean slate for v1.0.0) | [OK] Complete | **P1 - High** | **8.8.3** |
| **v1.0.0 Release Validation** (final checks) | Pending | **P0 - Critical** | **8.9** |
| **Documentation** (20+ pages) | Pending | **P1 - High** | **6** |

### Phase Overview

| Phase | Name | Estimate | Status |
|-------|------|----------|--------|
| 1 | CI/CD Parity | 1 day | [OK] Complete |
| 2 | Controllers (Basic) | 3-4 days | [OK] Complete |
| 3 | Motion/Kinematics (Basic) | 2-3 days | [OK] Complete |
| 4 | Integration Tests | 2 days | [OK] Complete |
| **4.5** | **Platform Implementations** | **5-6 days** | [OK] Complete (433 tests) |
| **4.6** | **Advanced Kinematics** | **4-5 days** | [OK] Complete (Delta, Stewart, SCARA) |
| **4.7** | **Vision/Camera** | **4-5 days** | [OK] Complete (color, markers, processing) |
| **4.8** | **Hardware Drivers** | **4-5 days** | [OK] Complete (15 drivers, 965 tests) |
| **4.9** | **Advanced Controllers** | **3-4 days** | [OK] Complete (Hexapod, Quadcopter, MAVLink) |
| **4.10** | **ROS2 Bridge** | **3 days** | [OK] Complete (1,021 lines, 991 tests) |
| **4.11** | **Integration Refactoring** | **1-2 days** | [OK] Complete (ai_infra, svc_infra, observability) |
| 5 | Examples | 2 days | [OK] Complete |
| **5.5** | **Sensor Test Coverage** | **2-3 days** | [OK] Complete |
| **5.6** | **Safety Module Tests** | **1-2 days** | [OK] Complete |
| **5.7** | **Platform & Integration Coverage** | **2 days** | [OK] Complete |
| **5.8** | **Driver & Protocol Coverage** | **2 days** | [OK] Complete |
| **5.9** | **Motion & Vision Coverage** | **1-2 days** | [OK] Complete |
| **5.10** | **CLI & Linting Fixes** | **0.5 days** | [OK] Complete |
| **5.11** | **Coverage Verification** | **1 day** | [OK] Complete |
| **5.12** | **Critical Fixes** | **1-2 hrs** | [OK] Complete |
| **5.13** | **API Stability** | **2-3 hrs** | [OK] Complete |
| **5.14** | **Observability (svc-infra)** | **4-5 hrs** | [OK] Complete |
| **5.15** | **Error Handling & Resilience** | **3-4 hrs** | [OK] Complete |
| **5.16** | **Resource Management** | **2-3 hrs** | [OK] Complete |
| **5.17** | **Hardware Abstraction** | **3-4 hrs** | [OK] Complete |
| **5.18** | **Security Hardening** | **2-3 hrs** | [OK] Complete |
| **5.19** | **Performance Benchmarks** | **2-3 hrs** | [OK] Complete (57 tests) |
| **5.20** | **Hardware Validation** | **3-4 hrs** | [OK] Complete (41 tests) |
| **5.21** | **Integration Testing** | **2-3 hrs** | [OK] Complete (190 tests) |
| **5.22** | **Release Preparation** | **2-3 hrs** | [OK] Complete |
| **5.23** | **Real Hardware Bus Implementations** | **4-5 hrs** | [OK] Complete |
| **5.24** | **Dependency Unbundling** | **2-3 hrs** | [OK] Complete |
| **5.25** | **Hardware Extras Expansion** | **2-3 hrs** | [OK] Complete |
| 6 | Documentation | 2 days | Pending |
| ~~7~~ | ~~Safety Tests (Legacy)~~ | ~~1 day~~ | [OK] Merged into 5.6 |
| 8 | Platform Support | 2-3 days | [OK] Complete (Phase 8.7) |
| **8.8** | **Production Readiness Gaps** | **1 day** | Pending |
| **8.9** | **v1.0.0 Release Validation** | **2-4 hrs** | Pending |

---

# Beyond v1.0.0 — Vision Phases

> **Purpose**: Features that make robo-infra the most advanced robotics SDK, surpassing ROS2, Viam, and PyRobot.
> These phases transform robo-infra from "great robotics library" to "indispensable robotics platform."

---

## Phase 9: Developer Experience Revolution
**Priority**: P1 - High | **Estimate**: 2-3 weeks | **Target**: v1.1.0

Transform the CLI into the most powerful robotics development tool ever created.

### 9.1 Project Scaffolding

#### 9.1.1 `robo new` Command
- [ ] Create `src/robo_infra/cli/new.py`
- [ ] `robo new <project-name>` - Create new robotics project
- [ ] Interactive prompts:
 - [ ] Robot type (arm, mobile, drone, custom)
 - [ ] Platform (Raspberry Pi, Jetson, Linux, simulation)
 - [ ] Controllers needed (PID, MPC, fuzzy, etc.)
 - [ ] Sensors (camera, lidar, IMU, etc.)
- [ ] Templates:
 - [ ] `robo new mybot --template arm` - 6-DOF arm project
 - [ ] `robo new mybot --template mobile` - Differential drive
 - [ ] `robo new mybot --template drone` - Quadcopter
 - [ ] `robo new mybot --template custom` - Minimal starter
- [ ] Generated project structure:
 ```
 mybot/
 ├── pyproject.toml # Dependencies pre-configured
 ├── config/
 │ └── robot.yaml # Robot configuration
 ├── src/
 │ └── mybot/
 │ ├── __init__.py
 │ ├── robot.py # Main robot class
 │ └── controllers/
 ├── tests/
 └── README.md
 ```

#### 9.1.2 Configuration System
- [ ] Create `src/robo_infra/config/` module
- [ ] YAML-based robot configuration:
 ```yaml
 robot:
 name: my-arm
 type: articulated
 dof: 6
 
 actuators:
 - name: joint_1
 type: servo
 driver: pca9685
 channel: 0
 limits: [-180, 180]
 
 sensors:
 - name: main_camera
 type: camera
 resolution: [640, 480]
 fps: 30
 
 safety:
 estop_pin: 17
 watchdog_timeout: 0.5
 ```
- [ ] `load_robot_config(path) -> RobotConfig`
- [ ] Auto-instantiate drivers, controllers, sensors from config

### 9.2 Hardware Discovery

#### 9.2.1 `robo scan` Command
- [ ] Create `src/robo_infra/cli/scan.py`
- [ ] `robo scan` - Discover all connected hardware
- [ ] Scan types:
 - [ ] `robo scan --i2c` - List I2C devices with chip identification
 - [ ] `robo scan --serial` - List serial ports and baud detection
 - [ ] `robo scan --can` - List CAN interfaces and active nodes
 - [ ] `robo scan --usb` - List USB devices with vendor identification
 - [ ] `robo scan --gpio` - List available GPIO pins
- [ ] Auto-identification:
 - [ ] Known I2C addresses (PCA9685=0x40, MPU6050=0x68, etc.)
 - [ ] USB vendor IDs for common robotics hardware
 - [ ] Auto-detect camera capabilities
- [ ] Output formats:
 - [ ] `--format table` (default, human-readable)
 - [ ] `--format json` (machine-readable)
 - [ ] `--format yaml` (config generation)

#### 9.2.2 `robo doctor` Command
- [ ] Create `src/robo_infra/cli/doctor.py`
- [ ] `robo doctor` - Diagnose robot setup issues
- [ ] Checks:
 - [ ] Python version compatibility
 - [ ] Required packages installed
 - [ ] Hardware permissions (I2C, SPI, GPIO access)
 - [ ] Camera access
 - [ ] CAN interface status
 - [ ] Network connectivity (for remote robots)
 - [ ] Disk space for logs
 - [ ] Memory available
- [ ] Output: Actionable fix recommendations

### 9.3 Interactive Robot Shell

#### 9.3.1 `robo shell` Command
- [ ] Create `src/robo_infra/cli/shell.py`
- [ ] `robo shell` - Interactive Python REPL with robot pre-loaded
- [ ] Features:
 - [ ] Robot instance pre-instantiated from config
 - [ ] Tab completion for robot methods
 - [ ] Syntax highlighting
 - [ ] History persistence
 - [ ] `%record` magic - Record commands to script
 - [ ] `%replay` magic - Replay recorded commands
 - [ ] `%safety on/off` - Toggle safety limits for testing
- [ ] Built-in helpers:
 - [ ] `home()` - Move to home position
 - [ ] `status()` - Print robot status
 - [ ] `sensors()` - Print all sensor readings
 - [ ] `move(joint, angle)` - Move single joint
 - [ ] `goto(x, y, z)` - Move to Cartesian position

#### 9.3.2 Real-Time Monitoring
- [ ] Create `src/robo_infra/cli/monitor.py`
- [ ] `robo monitor` - Real-time TUI dashboard
- [ ] Display panels:
 - [ ] Joint positions (live updating)
 - [ ] Sensor readings (graphs)
 - [ ] Motor currents
 - [ ] Temperature warnings
 - [ ] Error log stream
- [ ] Use `rich` or `textual` for TUI
- [ ] Keyboard shortcuts for common actions

### 9.4 Hot Reload Development

#### 9.4.1 `robo dev` Command
- [ ] Create `src/robo_infra/cli/dev.py`
- [ ] `robo dev` - Development mode with hot reload
- [ ] Watch mode:
 - [ ] Monitor Python files for changes
 - [ ] Reload controllers without robot restart
 - [ ] Keep hardware connections alive
 - [ ] Re-run safety checks on reload
- [ ] Features:
 - [ ] `--watch` - Auto-reload on file changes
 - [ ] `--breakpoint` - Pause on errors for debugging
 - [ ] `--record` - Log all commands for replay
 - [ ] `--slow <factor>` - Slow motion for debugging

### 9.5 Phase 9 Tests

#### 9.5.1 CLI Tests
- [ ] Create `tests/unit/test_cli_new.py` (15 tests)
- [ ] Create `tests/unit/test_cli_scan.py` (12 tests)
- [ ] Create `tests/unit/test_cli_shell.py` (10 tests)
- [ ] Create `tests/unit/test_cli_monitor.py` (8 tests)
- [ ] Create `tests/unit/test_cli_dev.py` (10 tests)
- [ ] **Target: 55 new tests**

### 9.6 Documentation Updates

#### 9.6.1 CLI Documentation
- [ ] Update `docs/cli.md` with new commands:
 - [ ] `robo new` - Project scaffolding guide
 - [ ] `robo scan` - Hardware discovery guide
 - [ ] `robo shell` - Interactive REPL guide
 - [ ] `robo monitor` - Dashboard guide
 - [ ] `robo dev` - Hot reload guide
- [ ] Add examples for each command
- [ ] Add troubleshooting section

#### 9.6.2 Configuration Documentation
- [ ] Create `docs/configuration.md`
- [ ] YAML robot configuration format
- [ ] Environment variables
- [ ] Template customization

### 9.7 Phase 9 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| `robo new` works | Create project from template | |
| `robo scan` works | Detect I2C/Serial/CAN devices | |
| `robo shell` works | Interactive REPL with robot | |
| `robo monitor` works | Real-time TUI dashboard | |
| `robo dev` works | Hot reload development | |
| New tests | 55+ | |
| Docs updated | `docs/cli.md`, `docs/configuration.md` | |

---

## Phase 10: Simulation & Digital Twin
**Priority**: P1 - High | **Estimate**: 4-6 weeks | **Target**: v1.2.0

Enable full simulation before deploying to real hardware.

### 10.1 Physics Simulation Core

#### 10.1.1 PyBullet Integration
- [ ] Create `src/robo_infra/simulation/` module
- [ ] Create `src/robo_infra/simulation/pybullet_backend.py`
- [ ] `class PyBulletSimulator:`
 - [ ] `start(headless=False)` - Start simulation
 - [ ] `stop()` - Clean shutdown
 - [ ] `step(dt)` - Advance physics
 - [ ] `load_urdf(path) -> RobotId` - Load robot model
 - [ ] `set_joint_position(robot_id, joint, angle)`
 - [ ] `get_joint_state(robot_id, joint) -> JointState`
 - [ ] `apply_force(robot_id, link, force, position)`
 - [ ] `get_contacts(robot_id) -> list[Contact]`

#### 10.1.2 URDF Support
- [ ] Create `src/robo_infra/simulation/urdf.py`
- [ ] URDF parsing and validation
- [ ] `load_urdf(path) -> URDFModel`
- [ ] `urdf_to_robot_config(urdf) -> RobotConfig` - Generate config from URDF
- [ ] Built-in URDF models:
 - [ ] `urdf://robo_infra/arm_6dof.urdf`
 - [ ] `urdf://robo_infra/differential_drive.urdf`
 - [ ] `urdf://robo_infra/quadruped.urdf`
 - [ ] `urdf://robo_infra/drone.urdf`

#### 10.1.3 MuJoCo Integration (Optional)
- [ ] Create `src/robo_infra/simulation/mujoco_backend.py`
- [ ] Same interface as PyBullet
- [ ] Higher fidelity for contact-rich tasks
- [ ] MJCF model support

### 10.2 Simulated Hardware

#### 10.2.1 Simulated Drivers
- [ ] Create `src/robo_infra/simulation/drivers/`
- [ ] `SimulatedServo` - Servo with physics response
- [ ] `SimulatedDCMotor` - DC motor with inertia
- [ ] `SimulatedStepper` - Stepper motor simulation
- [ ] `SimulatedEncoder` - Encoder feedback from sim
- [ ] Automatic substitution when in simulation mode

#### 10.2.2 Simulated Sensors
- [ ] `SimulatedCamera` - Rendered camera from sim
- [ ] `SimulatedLidar` - Ray-cast lidar
- [ ] `SimulatedIMU` - Simulated accelerometer/gyro
- [ ] `SimulatedDepthCamera` - Depth from Z-buffer
- [ ] Noise injection for realistic training

### 10.3 Digital Twin

#### 10.3.1 Live Mirroring
- [ ] Create `src/robo_infra/simulation/twin.py`
- [ ] `class DigitalTwin:`
 - [ ] Connect to real robot via network
 - [ ] Mirror all joint positions in real-time
 - [ ] Overlay sensor data on simulation
 - [ ] Predict collisions before they happen
 - [ ] Time-shift replay (see past N seconds)

#### 10.3.2 State Synchronization
- [ ] Real robot -> Sim: Mirror physical state
- [ ] Sim -> Real robot: Preview before executing
- [ ] Bidirectional for hybrid control
- [ ] Latency compensation

### 10.4 Simulation CLI

#### 10.4.1 `robo sim` Command
- [ ] `robo sim` - Launch simulation environment
- [ ] `robo sim --urdf path/to/robot.urdf` - Load specific model
- [ ] `robo sim --twin` - Connect as digital twin
- [ ] `robo sim --headless` - No GUI (for CI)
- [ ] `robo sim --record path/` - Record simulation data
- [ ] `robo sim --playback path/` - Replay recorded session

### 10.5 Phase 10 Tests

#### 10.5.1 Simulation Tests
- [ ] Create `tests/unit/test_simulation_pybullet.py` (20 tests)
- [ ] Create `tests/unit/test_simulation_urdf.py` (15 tests)
- [ ] Create `tests/unit/test_simulation_drivers.py` (20 tests)
- [ ] Create `tests/unit/test_simulation_sensors.py` (15 tests)
- [ ] Create `tests/unit/test_simulation_twin.py` (10 tests)
- [ ] **Target: 80 new tests**

### 10.6 Documentation Updates

#### 10.6.1 Simulation Documentation
- [ ] Create `docs/simulation.md` (expand from Phase 6 basic version)
- [ ] Sections:
 - [ ] PyBullet integration guide
 - [ ] URDF model format and loading
 - [ ] Creating simulated robots
 - [ ] Physics tuning (friction, damping)
 - [ ] Headless mode for CI/CD

#### 10.6.2 Digital Twin Documentation
- [ ] Create `docs/digital-twin.md`
- [ ] Sections:
 - [ ] Digital twin architecture
 - [ ] Real-time mirroring setup
 - [ ] Collision prediction
 - [ ] Recording and playback
 - [ ] Network configuration

### 10.7 Phase 10 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| PyBullet integration | Full physics simulation | |
| URDF loading | Parse and simulate URDF | |
| Simulated drivers | 5+ driver types | |
| Simulated sensors | 4+ sensor types | |
| Digital twin | Real-time mirroring | |
| New tests | 80+ | |
| Docs created | `docs/simulation.md`, `docs/digital-twin.md` | |
| New tests | 80+ | |

---

## Phase 11: Advanced AI Integration
**Priority**: P1 - High | **Estimate**: 4-5 weeks | **Target**: v1.3.0

Leverage ai-infra for intelligent robotics (unique differentiator vs ROS2/Viam).

> **Strategy**: ai-infra already provides STT, TTS, RealtimeVoice, Agent, HITL, Memory.
> We adopt these directly, build robotics-specific wrappers, and contribute
> general-purpose vision/learning modules back to ai-infra.

### 11.1 Adopt ai-infra Voice & NLP (Week 1)

> **Principle**: Use existing ai-infra modules directly. Don't rebuild.

#### 11.1.1 Voice Control Wrapper
- [ ] Create `src/robo_infra/ai/voice.py`
- [ ] `class VoiceController:`
 - [ ] Wraps `ai_infra.STT` for speech-to-text
 - [ ] Wraps `ai_infra.TTS` for robot responses
 - [ ] Wraps `ai_infra.RealtimeVoice` for live conversations
 - [ ] Robotics-specific: Wake word detection ("Hey robot")
 - [ ] Robotics-specific: Command confirmation before execution
- [ ] Dependencies: `from ai_infra import STT, TTS, RealtimeVoice`

#### 11.1.2 NLP Command Parser (Robotics-Specific)
- [ ] Create `src/robo_infra/ai/nlp_control.py`
- [ ] `class NLPController:`
 - [ ] Wraps `ai_infra.Agent` with `controller_to_tools()`
 - [ ] Robotics grammar parser:
 - [ ] "Pick up X" -> `move_to(X) + gripper.close()`
 - [ ] "Move to X" -> `goto(X)`
 - [ ] "Move slowly" -> velocity=0.3
 - [ ] "Stop" -> `emergency_stop()`
 - [ ] Command validation against safety limits
 - [ ] Trajectory generation from intent
- [ ] Example:
 ```python
 nlp = NLPController(controller)
 nlp.execute("Pick up the red cube slowly")
 # -> Parses intent, validates safety, generates trajectory
 ```

#### 11.1.3 Conversational Robot Interface
- [ ] Create `src/robo_infra/ai/chat.py`
- [ ] `class RobotChat:`
 - [ ] Wraps `ai_infra.Agent` with robot context
 - [ ] `ai_infra.MemoryStore` for conversation history
 - [ ] System prompt with robot capabilities
 - [ ] "Why did you stop?" -> Query safety logs
 - [ ] "What sensors are reading?" -> Format sensor data
 - [ ] "Move joint 3 by 10 degrees" -> Execute command
- [ ] CLI: `robo chat` - Interactive chat with robot

### 11.2 AI Safety with HITL (Week 1-2)

> **Principle**: Use ai-infra's Human-in-the-Loop for dangerous commands.

#### 11.2.1 AI Safety Monitor
- [ ] Create `src/robo_infra/ai/safety.py`
- [ ] `class AISafetyMonitor:`
 - [ ] Wraps `ai_infra.HITLConfig` for human approval
 - [ ] Wraps `ai_infra.ApprovalConfig` for tool policies
 - [ ] Robotics-specific policies:
 - [ ] `require_approval=["gripper_close", "move_to_human"]`
 - [ ] `auto_approve=["get_status", "read_sensor"]`
 - [ ] `always_deny=["self_destruct"]` 
 - [ ] Confidence thresholds (reject if LLM confidence < 0.8)
 - [ ] Audit logging of AI decisions
 - [ ] Kill switch for AI control
- [ ] Integration:
 ```python
 from ai_infra import HITLConfig, ApprovalConfig
 
 safety = AISafetyMonitor(
 require_approval=["gripper_close", "high_speed_move"],
 confidence_threshold=0.8,
 )
 agent = Agent(tools=tools, hitl=safety.hitl_config)
 ```

#### 11.2.2 Command Validation
- [ ] Create `src/robo_infra/ai/validation.py`
- [ ] `class CommandValidator:`
 - [ ] Check command against workspace bounds
 - [ ] Check velocity/acceleration limits
 - [ ] Check collision potential
 - [ ] Estimate risk score
 - [ ] Block unsafe commands with explanation

### 11.3 Vision AI (Week 2-3)

> **Strategy**: Build general vision in ai-infra, robotics wrappers in robo-infra.

#### 11.3.1 Contribute to ai-infra: Object Detection
- [ ] Create `ai_infra/vision/detection.py` (in ai-infra repo)
- [ ] `class ObjectDetector:`
 - [ ] YOLOv8 wrapper (ultralytics)
 - [ ] `detect(image) -> list[Detection]`
 - [ ] Pre-trained models: COCO, custom
 - [ ] GPU acceleration with fallback to CPU
- [ ] `class Detection:` (Pydantic model)
 - [ ] `class_name: str`
 - [ ] `confidence: float`
 - [ ] `bbox: tuple[int, int, int, int]`
 - [ ] `mask: np.ndarray | None` (for segmentation)

#### 11.3.2 Contribute to ai-infra: Pose Estimation
- [ ] Create `ai_infra/vision/pose.py` (in ai-infra repo)
- [ ] `class PoseEstimator:`
 - [ ] MediaPipe or YOLO-Pose wrapper
 - [ ] `estimate(image) -> list[Pose]`
 - [ ] Human pose (skeleton)
 - [ ] Hand pose (for gesture recognition)

#### 11.3.3 robo-infra Vision Wrapper
- [ ] Create `src/robo_infra/ai/vision.py`
- [ ] `class RobotVision:`
 - [ ] Wraps `ai_infra.vision.ObjectDetector`
 - [ ] Robotics-specific:
 - [ ] 3D localization from 2D + depth camera
 - [ ] Transform to robot coordinate frame
 - [ ] Grasp point prediction
 - [ ] Object tracking with Kalman filter
 - [ ] Scene graph construction
 - [ ] Object permanence tracking

### 11.4 Learning & Adaptation (Week 3-4)

> **Strategy**: Build general ML patterns in ai-infra, robotics adapters in robo-infra.

#### 11.4.1 Contribute to ai-infra: Imitation Learning
- [ ] Create `ai_infra/learning/imitation.py` (in ai-infra repo)
- [ ] `class ImitationLearner:`
 - [ ] Record state-action pairs from demonstrations
 - [ ] Behavior cloning with neural network
 - [ ] `record(states, actions)`
 - [ ] `train(epochs)`
 - [ ] `predict(state) -> action`
- [ ] Model storage and loading

#### 11.4.2 Contribute to ai-infra: RL Environment
- [ ] Create `ai_infra/learning/rl.py` (in ai-infra repo)
- [ ] `class RLEnvironment:` (Gym-compatible)
 - [ ] `reset() -> observation`
 - [ ] `step(action) -> (obs, reward, done, info)`
 - [ ] Integration with stable-baselines3
- [ ] Training utilities:
 - [ ] `train(env, algorithm, steps)`
 - [ ] `evaluate(env, policy)`
 - [ ] Checkpointing

#### 11.4.3 robo-infra Learning Adapters
- [ ] Create `src/robo_infra/ai/learning.py`
- [ ] `class RobotImitationLearner:`
 - [ ] Wraps `ai_infra.learning.ImitationLearner`
 - [ ] Record from `robo shell` sessions
 - [ ] Learn joint trajectories
 - [ ] Generalize to new start/goal positions
- [ ] `class RobotRLEnv(ai_infra.RLEnvironment):`
 - [ ] State = joint positions + sensor readings
 - [ ] Action = joint velocities
 - [ ] Reward = task-specific (reach goal, avoid collision)
 - [ ] Integration with Phase 10 simulation

#### 11.4.4 Online Adaptation
- [ ] Continuous learning from experience
- [ ] Anomaly detection (unexpected sensor readings)
- [ ] Self-calibration over time

### 11.5 Phase 11 Tests

#### 11.5.1 Voice & NLP Tests
- [ ] Create `tests/unit/test_ai_voice.py` (10 tests)
 - [ ] Test STT/TTS wrapper
 - [ ] Test wake word detection
 - [ ] Test command confirmation
- [ ] Create `tests/unit/test_ai_nlp.py` (15 tests)
 - [ ] Test command parsing ("pick up X")
 - [ ] Test velocity modifiers ("slowly", "quickly")
 - [ ] Test safety validation

#### 11.5.2 Safety Tests
- [ ] Create `tests/unit/test_ai_safety.py` (12 tests)
 - [ ] Test HITL integration
 - [ ] Test approval policies
 - [ ] Test confidence thresholds
 - [ ] Test audit logging

#### 11.5.3 Vision Tests
- [ ] Create `tests/unit/test_ai_vision.py` (15 tests)
 - [ ] Test object detection wrapper
 - [ ] Test 3D localization
 - [ ] Test coordinate transform
 - [ ] Test grasp point prediction

#### 11.5.4 Learning Tests
- [ ] Create `tests/unit/test_ai_learning.py` (15 tests)
 - [ ] Test imitation recording
 - [ ] Test trajectory prediction
 - [ ] Test RL environment step
 - [ ] Test reward calculation

- [ ] **Target: 67 new tests in robo-infra**
- [ ] **Target: 40+ tests in ai-infra (vision + learning contributions)**

### 11.6 ai-infra Contributions Summary

| Module | Location | Description |
|--------|----------|-------------|
| `ai_infra.vision.detection` | ai-infra | YOLO object detection |
| `ai_infra.vision.pose` | ai-infra | Pose estimation |
| `ai_infra.learning.imitation` | ai-infra | Behavior cloning |
| `ai_infra.learning.rl` | ai-infra | Gym-compatible RL |

### 11.7 Documentation Updates

#### 11.7.1 AI Integration Documentation
- [ ] Update `docs/ai-integration.md` with:
 - [ ] Voice control (STT/TTS) usage
 - [ ] NLP command examples
 - [ ] AI safety configuration
 - [ ] HITL approval workflows

#### 11.7.2 Vision AI Documentation
- [ ] Create `docs/ai-vision.md`
- [ ] Sections:
 - [ ] Object detection setup
 - [ ] 3D localization
 - [ ] Grasp point prediction
 - [ ] Camera calibration for AI

#### 11.7.3 Learning Documentation
- [ ] Create `docs/ai-learning.md`
- [ ] Sections:
 - [ ] Imitation learning workflow
 - [ ] Recording demonstrations
 - [ ] Training and deployment
 - [ ] RL environment setup

### 11.8 Phase 11 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Voice control | STT/TTS/Realtime from ai-infra | |
| NLP commands | Parse robotics commands | |
| AI safety | HITL from ai-infra + policies | |
| Object detection | YOLO in ai-infra | |
| Imitation learning | Behavior cloning in ai-infra | |
| RL environment | Gym-compatible in ai-infra | |
| robo-infra tests | 67+ | |
| ai-infra contributions | 4 modules | |
| Docs created | `ai-vision.md`, `ai-learning.md`, updated `ai-integration.md` | |

---

## Phase 12: Fleet Management
**Priority**: P2 - Medium | **Estimate**: 4-5 weeks | **Target**: v1.4.0

Manage multiple robots as a coordinated fleet.

### 12.1 Fleet Architecture

#### 12.1.1 Fleet Controller
- [ ] Create `src/robo_infra/fleet/` module
- [ ] Create `src/robo_infra/fleet/controller.py`
- [ ] `class FleetController:`
 - [ ] `register_robot(robot_id, connection)` - Add robot to fleet
 - [ ] `unregister_robot(robot_id)` - Remove robot
 - [ ] `get_robot_status(robot_id) -> RobotStatus`
 - [ ] `get_all_robots() -> list[RobotStatus]`
 - [ ] `broadcast_command(command)` - Send to all robots
 - [ ] `send_command(robot_id, command)` - Send to specific robot

#### 12.1.2 Fleet Communication
- [ ] Create `src/robo_infra/fleet/comms.py`
- [ ] `class FleetComms:`
 - [ ] WebSocket-based real-time communication
 - [ ] Robot-to-robot messaging
 - [ ] Central server mode
 - [ ] Mesh networking mode (no central server)
 - [ ] Automatic reconnection
 - [ ] Message encryption

### 12.2 Task Allocation

#### 12.2.1 Task Manager
- [ ] Create `src/robo_infra/fleet/tasks.py`
- [ ] `class TaskManager:`
 - [ ] Task queue management
 - [ ] Priority-based scheduling
 - [ ] Load balancing across robots
 - [ ] Task dependencies
 - [ ] Deadline constraints

#### 12.2.2 Allocation Algorithms
- [ ] Create `src/robo_infra/fleet/allocation.py`
- [ ] `class TaskAllocator:`
 - [ ] Round-robin allocation
 - [ ] Nearest-robot allocation
 - [ ] Capability-based allocation
 - [ ] Auction-based allocation
 - [ ] Custom allocation strategies

### 12.3 Multi-Robot Coordination

#### 12.3.1 Collision Avoidance
- [ ] Create `src/robo_infra/fleet/collision.py`
- [ ] `class FleetCollisionAvoidance:`
 - [ ] Share planned trajectories
 - [ ] Detect potential collisions
 - [ ] Negotiate priority
 - [ ] Re-plan conflicting paths
 - [ ] Real-time updates

#### 12.3.2 Formation Control
- [ ] Create `src/robo_infra/fleet/formation.py`
- [ ] `class FormationController:`
 - [ ] Predefined formations (line, square, V-shape)
 - [ ] Leader-follower control
 - [ ] Virtual structure approach
 - [ ] Dynamic formation switching

### 12.4 Swarm Intelligence

#### 12.4.1 Swarm Behaviors
- [ ] Create `src/robo_infra/fleet/swarm.py`
- [ ] `class SwarmController:`
 - [ ] Flocking (separation, alignment, cohesion)
 - [ ] Foraging behavior
 - [ ] Coverage algorithms
 - [ ] Consensus protocols
 - [ ] Emergent behavior simulation

#### 12.4.2 Distributed Sensing
- [ ] Create `src/robo_infra/fleet/distributed_sensing.py`
- [ ] Combine sensor data from multiple robots
- [ ] Distributed SLAM
- [ ] Collaborative mapping
- [ ] Multi-robot localization

### 12.5 Fleet Dashboard

#### 12.5.1 Web Dashboard
- [ ] Create `src/robo_infra/fleet/dashboard/`
- [ ] Real-time fleet visualization
- [ ] Robot status cards
- [ ] Task queue display
- [ ] Live map with robot positions
- [ ] Alert notifications
- [ ] Integration with nfrax-web components

#### 12.5.2 CLI Commands
- [ ] `robo fleet status` - Show fleet status
- [ ] `robo fleet add <robot>` - Add robot to fleet
- [ ] `robo fleet remove <robot>` - Remove robot
- [ ] `robo fleet task <robot> <task>` - Assign task
- [ ] `robo fleet broadcast <command>` - Send to all

### 12.6 Phase 12 Tests

- [ ] Create `tests/unit/test_fleet_controller.py` (15 tests)
- [ ] Create `tests/unit/test_fleet_comms.py` (12 tests)
- [ ] Create `tests/unit/test_fleet_tasks.py` (15 tests)
- [ ] Create `tests/unit/test_fleet_collision.py` (10 tests)
- [ ] Create `tests/unit/test_fleet_formation.py` (10 tests)
- [ ] Create `tests/unit/test_fleet_swarm.py` (12 tests)
- [ ] **Target: 74 new tests**

### 12.7 Documentation Updates

#### 12.7.1 Fleet Management Documentation
- [ ] Create `docs/fleet.md`
- [ ] Sections:
 - [ ] Fleet architecture overview
 - [ ] Setting up a fleet
 - [ ] Task allocation strategies
 - [ ] Formation control
 - [ ] Swarm intelligence patterns

#### 12.7.2 Multi-Robot Coordination Documentation
- [ ] Create `docs/coordination.md`
- [ ] Sections:
 - [ ] Collision avoidance
 - [ ] Distributed sensing
 - [ ] Collaborative SLAM
 - [ ] Robot-to-robot communication

### 12.8 Phase 12 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Fleet controller | Manage 10+ robots | |
| Task allocation | 3+ allocation algorithms | |
| Collision avoidance | Multi-robot path planning | |
| Formation control | 3+ formations | |
| Swarm behaviors | Flocking, foraging | |
| Web dashboard | Real-time visualization | |
| New tests | 74+ | |
| Docs created | `docs/fleet.md`, `docs/coordination.md` | |

---

## Phase 13: Hardware Ecosystem
**Priority**: P2 - Medium | **Estimate**: 3-4 weeks | **Target**: v1.5.0

First-party hardware support and partnerships.

### 13.1 Reference Designs

#### 13.1.1 Reference Robot Arms
- [ ] Create `docs/hardware/reference-arm.md`
- [ ] Bill of Materials (BOM)
- [ ] 3D printable parts (STL files)
- [ ] Wiring diagrams
- [ ] Assembly instructions
- [ ] Tested configurations:
 - [ ] 4-DOF desktop arm ($200 BOM)
 - [ ] 6-DOF industrial arm ($1,000 BOM)
 - [ ] 7-DOF collaborative arm ($2,500 BOM)

#### 13.1.2 Reference Mobile Robots
- [ ] Create `docs/hardware/reference-mobile.md`
- [ ] Differential drive platform
- [ ] Mecanum wheel platform
- [ ] Tracked platform
- [ ] Quadruped reference

#### 13.1.3 Reference Drones
- [ ] Create `docs/hardware/reference-drone.md`
- [ ] Quadcopter design
- [ ] Hexacopter design
- [ ] Flight controller integration

### 13.2 Verified Hardware List

#### 13.2.1 Verified Actuators
- [ ] Create `docs/hardware/verified-actuators.md`
- [ ] Dynamixel servos (all models)
- [ ] ODrive motor controllers
- [ ] ClearPath servos
- [ ] NEMA stepper motors
- [ ] Each with: driver, config, example

#### 13.2.2 Verified Sensors
- [ ] Create `docs/hardware/verified-sensors.md`
- [ ] Intel RealSense cameras
- [ ] RPLIDAR models
- [ ] Velodyne lidars
- [ ] IMU sensors (MPU6050, BNO055, etc.)
- [ ] Each with: driver, calibration, example

#### 13.2.3 Verified Controllers
- [ ] Create `docs/hardware/verified-controllers.md`
- [ ] Raspberry Pi 4/5
- [ ] NVIDIA Jetson (Nano, Xavier, Orin)
- [ ] BeagleBone
- [ ] x86 industrial PCs

### 13.3 Hardware Abstraction Improvements

#### 13.3.1 Universal Hardware Interface
- [ ] Create `src/robo_infra/hardware/universal.py`
- [ ] `class UniversalActuator:`
 - [ ] Common interface for all actuator types
 - [ ] Automatic unit conversion
 - [ ] Profile-based motion
 - [ ] Self-test capabilities

#### 13.3.2 Plug-and-Play Detection
- [ ] Auto-detect connected hardware
- [ ] Generate configuration from detected hardware
- [ ] Suggest compatible drivers
- [ ] One-click setup

### 13.4 Phase 13 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Reference designs | 3+ robot types | |
| Verified actuators | 10+ models | |
| Verified sensors | 10+ models | |
| Hardware docs | 10+ pages in `docs/hardware/` | |
| Plug-and-play | Auto-detection | |
| Docs created | `docs/hardware/*.md` (reference-arm, reference-mobile, reference-drone, verified-actuators, verified-sensors, verified-controllers) | |

---

## Phase 14: Safety Certification Preparation
**Priority**: P2 - Medium | **Estimate**: 4-6 weeks | **Target**: v2.0.0

Prepare for industrial safety certification (ISO 10218, ISO 13849).

### 14.1 Safety Documentation

#### 14.1.1 Safety Manual
- [ ] Create `docs/safety/safety-manual.md`
- [ ] Hazard identification
- [ ] Risk assessment methodology
- [ ] Safety function requirements
- [ ] Safety integrity levels (SIL)

#### 14.1.2 Safety Architecture
- [ ] Create `docs/safety/architecture.md`
- [ ] Safety-related block diagram
- [ ] Fail-safe states
- [ ] Redundancy requirements
- [ ] Diagnostic coverage

### 14.2 Safety Module Enhancements

#### 14.2.1 Safety Integrity
- [ ] Create `src/robo_infra/safety/integrity.py`
- [ ] `class SafetyIntegrityMonitor:`
 - [ ] Continuous self-test
 - [ ] Watchdog with SIL rating
 - [ ] Redundant sensor checking
 - [ ] Fail-safe state machine

#### 14.2.2 Certified Safety Functions
- [ ] Create `src/robo_infra/safety/certified/`
- [ ] Safe torque off (STO)
- [ ] Safe stop 1 (SS1)
- [ ] Safe stop 2 (SS2)
- [ ] Safely-limited speed (SLS)
- [ ] Safe position (SP)
- [ ] Safe direction (SD)

### 14.3 Compliance Tooling

#### 14.3.1 Compliance Checker
- [ ] Create `src/robo_infra/safety/compliance.py`
- [ ] `class ComplianceChecker:`
 - [ ] Check configuration against standards
 - [ ] Generate compliance report
 - [ ] Identify gaps
 - [ ] Suggest remediation

#### 14.3.2 Audit Trail
- [ ] Create `src/robo_infra/safety/audit.py`
- [ ] `class SafetyAuditLog:`
 - [ ] Immutable event logging
 - [ ] Cryptographic signing
 - [ ] Tamper detection
 - [ ] Export for auditors

### 14.4 Documentation Updates

#### 14.4.1 Safety Certification Documentation
- [ ] Update `docs/safety.md` with certification-ready content
- [ ] Create `docs/safety-certification.md`
- [ ] Sections:
 - [ ] ISO 10218 compliance guide
 - [ ] ISO 13849 overview
 - [ ] Safety integrity levels (SIL) explanation
 - [ ] Hazard analysis methodology
 - [ ] Safety function implementation guide
 - [ ] Audit trail usage
 - [ ] Compliance checker usage

### 14.5 Phase 14 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Safety manual | Complete documentation | |
| Safety functions | 6+ certified functions | |
| Compliance checker | Automated checking | |
| Audit trail | Immutable logging | |
| ISO 10218 prep | Gap analysis complete | |
| Docs created | `docs/safety-certification.md` | |

---

## Phase 15: Edge & Embedded Deployment
**Priority**: P2 - Medium | **Estimate**: 3-4 weeks | **Target**: v2.1.0

Optimize for resource-constrained environments.

### 15.1 Lightweight Core

#### 15.1.1 Minimal Installation
- [ ] Create `robo-infra-lite` package
- [ ] Core functionality only (~5MB)
- [ ] No AI dependencies
- [ ] No simulation dependencies
- [ ] Pure Python (no native extensions required)

#### 15.1.2 Memory Optimization
- [ ] Profile memory usage
- [ ] Reduce allocations in control loop
- [ ] Object pooling for high-frequency objects
- [ ] Target: <50MB RAM for basic operation

### 15.2 Embedded Support

#### 15.2.1 MicroPython Port
- [ ] Create `robo-infra-micro` package
- [ ] Core controllers for MicroPython
- [ ] GPIO/I2C/SPI drivers
- [ ] Target: ESP32, RP2040

#### 15.2.2 CircuitPython Port
- [ ] Similar to MicroPython port
- [ ] Adafruit hardware support
- [ ] Educational focus

### 15.3 Edge Deployment

#### 15.3.1 Container Images
- [ ] Create `docker/` directory
- [ ] `robo-infra:base` - Minimal image
- [ ] `robo-infra:full` - All features
- [ ] `robo-infra:jetson` - NVIDIA GPU optimized
- [ ] Multi-arch: arm64, amd64

#### 15.3.2 Deployment CLI
- [ ] `robo deploy <robot>` - Deploy to robot
- [ ] `robo deploy --docker` - Deploy as container
- [ ] `robo deploy --update` - OTA update
- [ ] `robo deploy --rollback` - Rollback on failure

### 15.4 Real-Time Enhancements

#### 15.4.1 Real-Time Scheduling
- [ ] Create `src/robo_infra/realtime/`
- [ ] PREEMPT_RT Linux support
- [ ] Priority configuration
- [ ] CPU affinity for control loops
- [ ] Latency monitoring

#### 15.4.2 Deterministic Control
- [ ] Lock-free data structures
- [ ] Avoid GC in control loop
- [ ] Pre-allocated buffers
- [ ] Target: <1ms jitter at 1kHz

### 15.5 Documentation Updates

#### 15.5.1 Edge Deployment Documentation
- [ ] Create `docs/edge-deployment.md`
- [ ] Sections:
 - [ ] Deployment options overview
 - [ ] Docker container deployment
 - [ ] OTA update process
 - [ ] Rollback procedures
 - [ ] Real-time configuration

#### 15.5.2 Embedded Development Documentation
- [ ] Create `docs/embedded.md`
- [ ] Sections:
 - [ ] MicroPython port setup
 - [ ] CircuitPython port setup
 - [ ] Memory optimization techniques
 - [ ] ESP32/RP2040 examples
 - [ ] Lightweight core usage

### 15.6 Phase 15 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Lite package | <5MB, <50MB RAM | |
| MicroPython port | ESP32 support | |
| Docker images | 3+ variants | |
| Deployment CLI | OTA updates | |
| Real-time | <1ms jitter | |
| Docs created | `docs/edge-deployment.md`, `docs/embedded.md` | |

---

## Phase 16: Cloud Integration
**Priority**: P3 - Low | **Estimate**: 2-3 weeks | **Target**: v2.2.0

Connect robots to cloud services for advanced capabilities.

### 16.1 Cloud Connectivity

#### 16.1.1 Cloud Bridge
- [ ] Create `src/robo_infra/cloud/` module
- [ ] `class CloudBridge:`
 - [ ] Secure connection to nfrax-api
 - [ ] Robot registration and authentication
 - [ ] Bi-directional communication
 - [ ] Offline queuing

#### 16.1.2 Remote Monitoring
- [ ] Real-time telemetry streaming
- [ ] Historical data storage
- [ ] Alert notifications
- [ ] Remote diagnostics

### 16.2 Cloud AI

#### 16.2.1 Cloud Vision
- [ ] Offload heavy vision processing
- [ ] Object detection in cloud
- [ ] Scene understanding
- [ ] Model updates OTA

#### 16.2.2 Cloud Learning
- [ ] Federated learning across fleet
- [ ] Cloud-based training
- [ ] Model distribution
- [ ] A/B testing of policies

### 16.3 Documentation Updates

#### 16.3.1 Cloud Integration Documentation
- [ ] Create `docs/cloud-integration.md`
- [ ] Sections:
 - [ ] Cloud bridge setup
 - [ ] Robot registration
 - [ ] Authentication and security
 - [ ] Telemetry streaming
 - [ ] Remote monitoring setup
 - [ ] Cloud AI offloading

#### 16.3.2 Federated Learning Documentation
- [ ] Create `docs/federated-learning.md`
- [ ] Sections:
 - [ ] Fleet-wide learning overview
 - [ ] Privacy-preserving aggregation
 - [ ] Model distribution
 - [ ] A/B testing policies

### 16.4 Phase 16 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Cloud bridge | Secure connection | |
| Telemetry | Real-time streaming | |
| Cloud AI | Vision offloading | |
| Federated learning | Multi-robot learning | |
| Docs created | `docs/cloud-integration.md`, `docs/federated-learning.md` | |

---

## Phase 17: Marketplace & Ecosystem
**Priority**: P3 - Low | **Estimate**: 4-6 weeks | **Target**: v2.5.0

Build an ecosystem around robo-infra.

### 17.1 Plugin System

#### 17.1.1 Plugin Architecture
- [ ] Create `src/robo_infra/plugins/` module
- [ ] Plugin discovery and loading
- [ ] Plugin sandboxing
- [ ] Dependency resolution
- [ ] Version compatibility

#### 17.1.2 Plugin Types
- [ ] Driver plugins (new hardware)
- [ ] Controller plugins (new algorithms)
- [ ] Sensor plugins (new sensors)
- [ ] Integration plugins (new services)

### 17.2 Marketplace

#### 17.2.1 Plugin Registry
- [ ] Central plugin registry
- [ ] Plugin search and discovery
- [ ] Version management
- [ ] Verified publisher program

#### 17.2.2 Monetization
- [ ] Free plugins
- [ ] Paid plugins (revenue share)
- [ ] Enterprise plugins
- [ ] Support subscriptions

### 17.3 Community

#### 17.3.1 Developer Portal
- [ ] API documentation
- [ ] Plugin development guide
- [ ] Sample plugins
- [ ] Certification program

#### 17.3.2 User Community
- [ ] Forums/Discord
- [ ] Showcase gallery
- [ ] Tutorials and courses
- [ ] Annual conference

### 17.4 Documentation Updates

#### 17.4.1 Plugin Development Documentation
- [ ] Create `docs/plugins.md`
- [ ] Sections:
 - [ ] Plugin architecture overview
 - [ ] Creating driver plugins
 - [ ] Creating controller plugins
 - [ ] Creating sensor plugins
 - [ ] Plugin sandboxing and security
 - [ ] Publishing to marketplace

#### 17.4.2 Marketplace Documentation
- [ ] Create `docs/marketplace.md`
- [ ] Sections:
 - [ ] Browsing and installing plugins
 - [ ] Verified publisher program
 - [ ] Monetization for developers
 - [ ] Enterprise plugins

### 17.5 Phase 17 Completion Criteria

| Metric | Target | Status |
|--------|--------|--------|
| Plugin system | Load external plugins | |
| Plugin registry | Search and install | |
| Developer portal | Full documentation | |
| Community | Active engagement | |
| Docs created | `docs/plugins.md`, `docs/marketplace.md` | |

---

## Timeline Summary

| Phase | Description | Estimate | Priority | Status |
|-------|-------------|----------|----------|--------|
| Phase 1 | CI/CD Parity | 1 day | P0 | [OK] Complete |
| Phase 2 | Controllers | 3-4 days | P0 | [OK] Complete |
| Phase 3 | Motion & Kinematics | 3-4 days | P1 | [OK] Complete |
| Phase 4 | Integrations Coverage | 2 days | P1 | [OK] Complete |
| Phase 4.5 | Platform Implementations | 5-6 days | P1 | [OK] Complete |
| Phase 4.6-4.11 | Advanced Features | 15-20 days | P1-P2 | [OK] Complete |
| Phase 5 | Examples | 2 days | P1 | [OK] Complete |
| **Phase 5.5** | **Sensor Test Coverage** | **2-3 days** | **P0** | [OK] Complete |
| **Phase 5.6** | **Safety Module Tests** | **1-2 days** | **P0** | [OK] Complete |
| **Phase 5.7** | **Platform/Integration Coverage** | **2 days** | **P1** | [OK] Complete |
| **Phase 5.8** | **Driver/Protocol Coverage** | **2 days** | **P1** | [OK] Complete |
| **Phase 5.9** | **Motion/Vision Coverage** | **1-2 days** | **P1** | [OK] Complete |
| **Phase 5.10** | **CLI & Linting Fixes** | **0.5 days** | **P2** | [OK] Complete |
| **Phase 5.11** | **Coverage Verification** | **1 day** | **P0** | [OK] Complete |
| **Phase 5.12** | **Critical Fixes** | **1-2 hrs** | **P0** | [OK] Complete |
| **Phase 5.13** | **API Stability** | **2-3 hrs** | **P0** | [OK] Complete |
| **Phase 5.14** | **Observability (svc-infra)** | **4-5 hrs** | **P1** | [OK] Complete |
| **Phase 5.15** | **Error Handling & Resilience** | **3-4 hrs** | **P0** | [OK] Complete |
| **Phase 5.16** | **Resource Management** | **2-3 hrs** | **P1** | [OK] Complete |
| **Phase 5.17** | **Hardware Abstraction** | **3-4 hrs** | **P1** | [OK] Complete |
| **Phase 5.18** | **Security Hardening** | **2-3 hrs** | **P1** | [OK] Complete |
| **Phase 5.19** | **Benchmarks** | **2-3 hrs** | **P2** | [OK] Complete |
| **Phase 5.20** | **Hardware Validation** | **3-4 hrs** | **P2** | [OK] Complete |
| **Phase 5.21** | **Integration Testing** | **2-3 hrs** | **P1** | [OK] Complete |
| **Phase 5.22** | **Release Preparation** | **2-3 hrs** | **P0** | [OK] Complete |
| **Phase 5.23** | **Real Hardware Buses** | **1.5-2 hrs** | **P0** | [OK] Complete |
| **Phase 5.24** | **Dependency Unbundling** | **1-1.5 hrs** | **P0** | [OK] Complete |
| **Phase 5.25** | **Hardware Extras Expansion** | **1.5-2 hrs** | **P1** | [OK] Complete |
| Phase 6 | Documentation | 2 days | P1 | Pending |
| Phase 7 | Safety Tests | 1-2 days | P1 | [OK] Merged into 5.6 |
| Phase 8 | Platform Support | 2-3 days | P2 | [OK] Complete (Phase 8.7) |
| **Phase 8.8** | **Production Readiness Gaps** | **1 day** | **P0** | Pending |
| **Phase 8.9** | **v1.0.0 Release Validation** | **2-4 hrs** | **P0** | Pending |
| **Total v1.0** | **Remaining: 8.8 + 8.9** | **~1.5 days** | | |

### Beyond v1.0.0 — Vision Phases

| Phase | Description | Estimate | Priority | Target |
|-------|-------------|----------|----------|--------|
| Phase 9 | Developer Experience Revolution | 2-3 weeks | P1 | v1.1.0 |
| Phase 10 | Simulation & Digital Twin | 4-6 weeks | P1 | v1.2.0 |
| Phase 11 | Advanced AI Integration | 4-5 weeks | P1 | v1.3.0 |
| Phase 12 | Fleet Management | 4-5 weeks | P2 | v1.4.0 |
| Phase 13 | Hardware Ecosystem | 3-4 weeks | P2 | v1.5.0 |
| Phase 14 | Safety Certification Preparation | 4-6 weeks | P2 | v2.0.0 |
| Phase 15 | Edge & Embedded Deployment | 3-4 weeks | P2 | v2.1.0 |
| Phase 16 | Cloud Integration | 2-3 weeks | P3 | v2.2.0 |
| Phase 17 | Marketplace & Ecosystem | 4-6 weeks | P3 | v2.5.0 |
| **Total Beyond 1.0** | **Vision Phases** | **~31-43 weeks** | | **~8-10 months** |

### Vision Phase Test Targets

| Phase | New Tests | Focus Area |
|-------|-----------|------------|
| Phase 9 | 55+ | CLI commands (new, scan, shell, monitor, dev) |
| Phase 10 | 80+ | Simulation, URDF, simulated hardware, digital twin |
| Phase 11 | 67+ (robo) + 40+ (ai-infra) | Voice/NLP, AI safety, vision, learning |
| Phase 12 | 74+ | Fleet management, coordination, swarm |
| Phase 13 | 30+ | Hardware ecosystem, reference designs |
| Phase 14 | 40+ | Safety certification, compliance |
| Phase 15 | 50+ | Edge, embedded, real-time |
| Phase 16 | 35+ | Cloud integration, federated learning |
| Phase 17 | 45+ | Plugin system, marketplace |
| **Total New** | **476+ (robo) + 40+ (ai-infra)** | **All vision phases** |

### ai-infra Contributions (Phase 11)

| Module | Description | Tests |
|--------|-------------|-------|
| `ai_infra.vision.detection` | YOLO object detection wrapper | 15+ |
| `ai_infra.vision.pose` | Pose estimation (MediaPipe/YOLO-Pose) | 10+ |
| `ai_infra.learning.imitation` | Behavior cloning | 8+ |
| `ai_infra.learning.rl` | Gym-compatible RL environment | 7+ |
| **Total** | **4 new modules** | **40+** |

### Production Readiness Work Breakdown

| Phase | Tasks | Estimate | Focus Area |
|-------|-------|----------|------------|
| 5.12 | 5 | 1-2 hrs | Version sync, SECURITY.md, py.typed, mypy.ini, classifier |
| 5.13 | 4 | 2-3 hrs | __all__ audit, submodule exports, dead code removal, type annotations |
| 5.14 | 5 | 4-5 hrs | Prometheus metrics, controller instrumentation, health checks, logging |
| 5.15 | 5 | 3-4 hrs | Exception hierarchy, timeouts, graceful degradation, retry, circuit breaker |
| 5.16 | 4 | 2-3 hrs | Context managers, connection pooling, cleanup handlers, memory mgmt |
| 5.17 | 4 | 3-4 hrs | Simulation completeness, detection robustness, platform optimization |
| 5.18 | 4 | 2-3 hrs | Bandit issues, input validation, privilege management, secrets audit |
| 5.19 | 4 | 2-3 hrs | Benchmark suite, performance targets, CI integration, memory profiling |
| 5.20 | 4 | 3-4 hrs | Hardware test framework, CI runner, BOM, wiring diagrams |
| 5.21 | 3 | 2-3 hrs | E2E scenarios, test categories, mock external systems |
| 5.22 | 5 | 2-3 hrs | Changelog, version bump, release notes, validation, PyPI publish |
| 5.23 | 3 | 1.5-2 hrs | Real I2C/SPI/Serial bus implementations (CAN [OK] done) |
| 5.24 | 4 | 1-1.5 hrs | Make ai-infra/svc-infra optional dependencies |
| 5.25 | 5 | 1.5-2 hrs | Hardware extras (python-can, opencv, camera SDKs) |
| **Total** | **58 tasks** | **~32-42 hrs** | **~5-6 days** |

---

## Success Metrics (1.0.0 Release)

| Metric | Current | Target | Gap | Status |
|--------|---------|--------|-----|--------|
| Version | 0.1.12 | 1.0.0 | - | Phase 5.22 |
| Tests | 5,167 | 5,500+ | +333 | [OK] Good |
| Coverage | **73.86%** | 80%+ | **6.14%** | [!] Hardware exceptions |
| Safety Coverage | **90-100%** | 80%+ | [OK] Met | [OK] Exceeds |
| Sensor Coverage (testable) | **84-90%** | 75%+ | [OK] Met | [OK] Exceeds |
| Controllers | 15+ | 15+ | 0 | [OK] Met |
| Examples | 4 | 6 | 2 | [!] Close |
| Docs | 7 | 20+ | 13 | Phase 6 |
| Ruff Errors | 0 | 0 | 0 | [OK] Clean |
| Mypy Errors | 0 | 0 | 0 | [OK] Clean |
| Bandit High/Med | 0 | 0 | 0 | [OK] Secure |
| py.typed | [OK] | [OK] | Done | [OK] Phase 5.12 Complete |
| SECURITY.md | [OK] | [OK] | Done | [OK] Phase 5.12 Complete |
| Observability | [OK] | [OK] | Done | [OK] Phase 5.14 Complete |
| Benchmarks | [OK] | [OK] | 57 tests | [OK] Phase 5.19 Complete |
| Published to PyPI | [!] | [OK] | Ready | Final publish pending |

---

## Version Milestones

### v0.2.0 (Beta) - [OK] ACHIEVED
- [x] Phase 1 complete (CI/CD parity)
- [x] Phase 2 complete (Controllers)
- [x] Phase 4 complete (Integration tests)
- [x] Phase 5 complete (Examples)
- [x] Phase 5.5-5.11 complete (Coverage verification)
- [x] 5,167 tests
- [x] Coverage >=73% (hardware exceptions acceptable)
- [x] 0 ruff/mypy errors

### v0.3.0 (Beta+) - Target: +1 week
- [x] Phase 5.12 complete (Critical fixes)
- [x] Phase 5.13 complete (API stability)
- [x] Phase 5.14 complete (Observability)
- [x] Phase 5.15 complete (Error handling)
- [x] SECURITY.md, py.typed, version sync
- [x] All __all__ exports sorted

### v0.4.0 (RC) - [OK] ACHIEVED
- [x] Phase 5.14 complete (Observability)
- [x] Phase 5.16-5.18 complete (Resource mgmt, hardware, security)
- [x] Phase 5.21 complete (Integration testing)
- [x] Metrics, health checks, structured logging
- [x] All Bandit issues resolved or documented

### v0.5.0 (RC2) - [OK] ACHIEVED
- [x] Phase 5.19-5.20 complete (Benchmarks, hardware validation)
- [x] Performance targets documented
- [x] Hardware test infrastructure ready
- [x] Real hardware validation complete

### v1.0.0 (Production) - Target: Documentation Only
- [x] Phase 5.22 complete (Release preparation)
- [x] Phase 4.6-4.11 complete (Advanced features)
- [x] Phase 5.23-5.25 complete (Hardware buses, dependency unbundling)
- [ ] Phase 6 complete (Full documentation)
- [x] 5,861 tests (exceeds 5,500+ target)
- [x] All examples working
- [ ] Full documentation (20+ pages)
- [x] No known security issues
- [x] Production classifier in pyproject.toml
- [ ] Published to PyPI

---

## Beyond v1.0.0 — Vision Milestones

### v1.1.0 (Developer Experience) - Target: +2-3 weeks after 1.0
- [ ] Phase 9 complete (Developer Experience Revolution)
- [ ] `robo new` - Project scaffolding from templates
- [ ] `robo scan` - Hardware discovery
- [ ] `robo shell` - Interactive robot REPL
- [ ] `robo monitor` - Real-time TUI dashboard
- [ ] `robo dev` - Hot reload development
- [ ] 55+ new tests

### v1.2.0 (Simulation) - Target: +4-6 weeks after 1.1
- [ ] Phase 10 complete (Simulation & Digital Twin)
- [ ] PyBullet physics simulation
- [ ] URDF robot model support
- [ ] Simulated drivers and sensors
- [ ] Digital twin with live mirroring
- [ ] `robo sim` command
- [ ] 80+ new tests

### v1.3.0 (AI-Native) - Target: +4-5 weeks after 1.2
- [ ] Phase 11 complete (Advanced AI Integration)
- [ ] Voice control via ai-infra STT/TTS/RealtimeVoice
- [ ] NLP commands with robotics grammar parser
- [ ] AI safety via ai-infra HITL + custom policies
- [ ] Vision AI via ai-infra ObjectDetector (new module)
- [ ] Imitation learning via ai-infra ImitationLearner (new module)
- [ ] RL environment via ai-infra RLEnvironment (new module)
- [ ] 67+ new tests in robo-infra
- [ ] 40+ new tests in ai-infra (4 contributed modules)

### v1.4.0 (Fleet) - Target: +4-5 weeks after 1.3
- [ ] Phase 12 complete (Fleet Management)
- [ ] Multi-robot fleet controller
- [ ] Task allocation algorithms
- [ ] Collision avoidance
- [ ] Formation control
- [ ] Swarm intelligence
- [ ] Fleet web dashboard
- [ ] 74+ new tests

### v1.5.0 (Hardware Ecosystem) - Target: +3-4 weeks after 1.4
- [ ] Phase 13 complete (Hardware Ecosystem)
- [ ] Reference robot designs with BOM
- [ ] Verified hardware list (actuators, sensors, controllers)
- [ ] Plug-and-play hardware detection
- [ ] 30+ new tests

### v2.0.0 (Safety Certified) - Target: +4-6 weeks after 1.5
- [ ] Phase 14 complete (Safety Certification Preparation)
- [ ] Safety manual and architecture docs
- [ ] Certified safety functions (STO, SS1, SS2, SLS, SP, SD)
- [ ] Compliance checker and audit trail
- [ ] ISO 10218 gap analysis complete
- [ ] 40+ new tests

### v2.1.0 (Edge) - Target: +3-4 weeks after 2.0
- [ ] Phase 15 complete (Edge & Embedded Deployment)
- [ ] `robo-infra-lite` package (<5MB, <50MB RAM)
- [ ] MicroPython/CircuitPython ports
- [ ] Docker container images
- [ ] OTA deployment CLI
- [ ] Real-time control (<1ms jitter)
- [ ] 50+ new tests

### v2.2.0 (Cloud) - Target: +2-3 weeks after 2.1
- [ ] Phase 16 complete (Cloud Integration)
- [ ] Cloud bridge to nfrax-api
- [ ] Remote monitoring and telemetry
- [ ] Cloud AI offloading
- [ ] Federated learning across fleet
- [ ] 35+ new tests

### v2.5.0 (Ecosystem) - Target: +4-6 weeks after 2.2
- [ ] Phase 17 complete (Marketplace & Ecosystem)
- [ ] Plugin system architecture
- [ ] Plugin registry and marketplace
- [ ] Developer portal
- [ ] Community engagement
- [ ] 45+ new tests

---

## Production Readiness Checklist

### Code Quality
- [x] Overall coverage >=73% (80% target limited by hardware-only modules)
- [x] Safety module coverage >=80% (actual: 90-100%)
- [x] Sensor module coverage >=75% (testable: 84-90%)
- [x] 0 ruff errors
- [x] 0 mypy errors
- [ ] 0 high/medium bandit issues (54 low remain)

### Infrastructure
- [x] py.typed marker exists
- [x] SECURITY.md exists
- [x] Version synced (__init__.py = pyproject.toml)
- [x] All __all__ exports sorted
- [x] Observability integrated (svc-infra)
- [x] Benchmarks documented

### Testing
- [x] 5,167 tests pass
- [x] Example tests pass
- [x] Integration tests pass
- [x] E2E scenarios documented
- [x] Hardware validation infrastructure

### Error Handling
- [x] All blocking ops have timeouts
- [x] Retry utilities available
- [x] Circuit breaker available
- [x] Graceful degradation implemented

### Resource Management
- [x] All drivers are context managers
- [x] Connection pooling for buses
- [x] Cleanup handlers registered
- [x] Memory stable in long runs

### Documentation
- [ ] 20+ documentation pages
- [x] All public APIs documented
- [x] Examples for each controller type
- [x] Hardware setup guide

### Security
- [x] No hardcoded secrets
- [x] All secrets validated in production
- [x] No eval/exec on untrusted input
- [x] Bandit scan documented
