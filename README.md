# Segway Balance Controller — ECE 551 (UW–Madison)

A SystemVerilog RTL design, verification, and synthesis project for a self-balancing
**Segway** controller, completed for ECE 551 (Digital System Design) at the University
of Wisconsin–Madison.

The design reads a rider's lean angle from an inertial sensor, reads load-cell and
battery data through an A/D converter, runs a PID balance-control loop, and drives the
left/right motors via PWM — while handling rider authentication (BLE/UART), steering
enable, over-current shutdown, and a piezo audio driver.

## Top-level interface

`Design/Segway.sv` is the top module. It ties together the SPI interfaces to the
inertial sensor and A/D converter, the balance/steering control path, and the motor and
piezo drivers:

- **Inputs:** `clk`, `RST_n`, SPI returns (`INERT_MISO`, `A2D_MISO`), `INERT_INT`,
  motor over-current (`OVR_I_lft/rght`), `RX` (UART from BLE module)
- **Outputs:** SPI controls to the sensor & A/D, motor PWM (`PWM1/2_lft`, `PWM1/2_rght`),
  piezo drive (`piezo`, `piezo_n`), over-current shutdown

## Repository layout

```
Design/                     RTL source (SystemVerilog)
  Segway.sv                 Top-level module
  balance_cntrl.sv, PID.sv, SegwayMath.sv   Balance / PID control path
  inert_intf.sv, inertial_integrator.sv     Inertial sensor interface + integration
  A2D_Intf.sv, SPI_mnrch.sv                 A/D + SPI master
  mtr_drv.sv, PWM11.sv                       Motor drivers
  steer_en.sv, steer_en_SM.sv               Steering-enable FSM
  Auth_blk.sv, UART_rx.sv, UART_tx.sv        Rider authentication (BLE/UART)
  piezo_drv.sv, PB_release.sv, rst_synch.sv  Piezo, push-button, reset sync
  ProvidedFiles/                             Course-provided models (sensor/A2D/Segway plant)

Testbench/                  Functional verification
  Segway_tb.sv, tb_tasks.sv                  Top testbench + reusable tasks
  *_test.sv                                  Directed tests (auth, battery, over-current,
                                             piezo, rider-lean, steering, too-fast, etc.)
  coverage_report/                           Coverage results (.txt report + .png snapshots)

Synthesis/                  Synopsys Design Compiler flow
  segway.dc                                  Synthesis script / constraints
  Segway.vg                                  Synthesized gate-level netlist
  area.txt, timing_report.txt                Area & timing reports

Post_Synthesis_testbench/   Gate-level (post-synthesis) verification
  Segway.vg                                  Netlist under test
  rider_lean_test.sv                         Post-synthesis directed test
  Screenshot_PostSynthesis_Simulation.png    Simulation result
```

## How to run

**RTL simulation (ModelSim/Questa):**

```sh
cd Testbench
vlog ../Design/*.sv ../Design/ProvidedFiles/*.sv *.sv
vsim -c Segway_tb -do "run -all; quit"
```

**Synthesis (Synopsys Design Compiler):**

```sh
cd Synthesis
dc_shell -f segway.dc
```

**Post-synthesis simulation:** compile the gate-level `Segway.vg` netlist against the
post-synthesis testbench in `Post_Synthesis_testbench/`.

> Tool-generated artifacts (`work/`, `vsim.wlf`, `transcript`, DC intermediates, coverage
> databases) are intentionally excluded via `.gitignore` — only source, scripts, and
> result reports are tracked.

## Notes & attribution

Team project. Contributions included PID & Segway math, the piezo driver, and the A/D
interface tuning (battery defaults to a non-low value in `A2D_Intf.sv` for testing).
