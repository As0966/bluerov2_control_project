#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════╗
║          BlueROV2 MISSION CONTROL — Interactive Launcher     ║
╚══════════════════════════════════════════════════════════════╝
Professional terminal UI for launching BlueROV2 simulations.
"""
import os, sys, time, signal, subprocess, threading
import numpy as np

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.prompt import Prompt, Confirm
from rich.layout import Layout
from rich.live import Live
from rich.text import Text
from rich.align import Align
from rich.columns import Columns
from rich import box
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TimeElapsedColumn
from rich.rule import Rule
from rich.style import Style

console = Console()

# ── Banner ────────────────────────────────────────────────────────
BANNER = r"""
   ____  _            ____   _____  __   ___
  | __ )| |_   _  ___|  _ \ / _ \ \/ /  |__ \
  |  _ \| | | | |/ _ \ |_) | | | \  /     ) |
  | |_) | | |_| |  __/  _ <| |_| /  \    / /
  |____/|_|\__,_|\___|_| \_\\___/_/\_\  |___|
       M I S S I O N   C O N T R O L
"""

CONTROLLERS = {
    "1": ("backstepping",            "Robust Backstepping",         "Expert teacher — Lyapunov-based robustness",     "simple"),
    "2": ("observer_linear",         "Linear Observer",             "Linearized Luenberger state estimation",         "observer"),
    "3": ("hasten_state_feedback",   "HASTEN State Feedback",       "Pure linear state feedback (PDF Section 3)",      "hasten"),
    "4": ("hasten_fl_input",         "HASTEN FL (Input-State)",     "Velocity-level linearization (PDF Section 5.1)", "hasten"),
    "5": ("hasten_fl_io",            "HASTEN FL (Input-Output)",    "Trajectory-level linearization (PDF Section 5.2)","hasten"),
    "6": ("hasten_mrac",             "HASTEN MRAC",                 "Standard adaptive control (PDF Section 6)",      "hasten"),
    "7": ("hasten_adaptive_backstepping", "HASTEN Adaptive BS",      "Pure adaptive backstepping with regressor",      "hasten"),
    "8": ("hasten_inn",              "HASTEN Adaptive INN",         "Backprop-based online learning (PDF Section 8)", "hasten"),
}


def show_banner():
    console.print(Panel(
        Align.center(Text(BANNER, style="bold cyan")),
        border_style="bright_blue",
        box=box.DOUBLE_EDGE,
        subtitle="[dim]ROS 2 Humble • Gazebo Harmonic • 6-DOF Underwater Vehicle[/dim]",
    ))


def show_controller_table():
    table = Table(
        title="[bold bright_cyan]Available Controllers[/]",
        box=box.ROUNDED, border_style="bright_blue",
        show_header=True, header_style="bold magenta",
        pad_edge=True, expand=True,
    )
    table.add_column("#", style="bold yellow", justify="center", width=3)
    table.add_column("Controller", style="bold white", min_width=22)
    table.add_column("Description", style="dim white")
    table.add_column("Type", justify="center", style="cyan")

    type_emoji = {"simple": "⚡", "observer": "👁", "adaptive": "🧠", "neural": "🤖", "hasten": "📜"}
    type_color = {"simple": "green", "observer": "yellow", "adaptive": "magenta", "neural": "bright_red", "hasten": "bright_cyan"}

    for key, (_, name, desc, ctype) in CONTROLLERS.items():
        emoji = type_emoji[ctype]
        color = type_color[ctype]
        table.add_row(key, name, desc, f"[{color}]{emoji} {ctype}[/]")

    console.print(table)


def animated_loading(msg, duration=1.5):
    with Progress(
        SpinnerColumn("dots12", style="bright_cyan"),
        TextColumn(f"[bold bright_white]{msg}[/]"),
        BarColumn(bar_width=30, style="bright_blue", complete_style="bright_cyan"),
        TimeElapsedColumn(),
        console=console, transient=True,
    ) as progress:
        task = progress.add_task("", total=100)
        for i in range(100):
            time.sleep(duration / 100)
            progress.update(task, advance=1)


def show_mission_summary(use_gazebo, ctrl_name, ctrl_display, mode, duration=80.0):
    mode_style = "green" if mode == "nominal" else "red"
    mode_icon = "✅" if mode == "nominal" else "🌊"
    gz_icon = "🌐" if use_gazebo else "📊"

    summary = Table(box=box.SIMPLE_HEAVY, border_style="bright_blue", expand=True, show_header=False)
    summary.add_column("Key", style="bold bright_cyan", width=18)
    summary.add_column("Value", style="bold white")
    summary.add_row("Simulation",  f"{gz_icon}  {'Gazebo Harmonic 3D' if use_gazebo else 'Standalone (scipy + plots)'}")
    summary.add_row("Controller",  f"🎮  {ctrl_display}")
    summary.add_row("Scenario",    f"{mode_icon}  [{mode_style}]{mode.upper()}[/]")
    summary.add_row("Duration",    f"⏱️   {duration:.0f} seconds")
    summary.add_row("Thrusters",   "🔧  6× T200 vectored configuration")
    summary.add_row("Control Rate", "🔄  50 Hz")

    console.print(Panel(summary, title="[bold bright_green]━━ MISSION BRIEFING ━━[/]",
                        border_style="bright_green", box=box.DOUBLE_EDGE))


def launch_gazebo():
    console.print(Rule("[bold bright_cyan]Launching Gazebo Harmonic[/]", style="bright_blue"))
    animated_loading("Initializing physics engine", 2.0)

    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    cmd = "source /opt/ros/humble/setup.bash && " \
          "source ~/bluerov2_control_project/ros2_ws/install/setup.bash && " \
          "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && " \
          "ros2 launch bluerov2_gazebo start_simulation.launch.py"
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env=env, preexec_fn=os.setsid)
    console.print("[bold green]✓[/] Gazebo simulation launched (PID: {})".format(proc.pid))
    time.sleep(8)  # Give Gazebo time to fully load the world
    return proc


def launch_controller_gazebo(ctrl_name, duration, disturbance, trajectory):
    console.print(Rule(f"[bold bright_cyan]Starting {ctrl_name} Controller[/]", style="bright_blue"))
    animated_loading("Connecting to Gazebo bridge", 1.5)

    dist_str = "true" if disturbance else "false"
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    cmd = "source /opt/ros/humble/setup.bash && " \
          "source ~/bluerov2_control_project/ros2_ws/install/setup.bash && " \
          "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && " \
          f"ros2 launch bluerov2_control run_controller.launch.py controller:={ctrl_name} duration:={duration} disturbance:={dist_str} trajectory:={trajectory}"
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env=env, preexec_fn=os.setsid)
    console.print(f"[bold green]✓[/] Controller node started (PID: {proc.pid})")
    return proc


def run_standalone(ctrl_name, disturbance_enabled, duration=60.0, trajectory='spline'):
    """Run the standalone scipy simulation with plots."""
    console.print(Rule("[bold bright_cyan]Running Standalone Simulation[/]", style="bright_blue"))
    animated_loading("Integrating 4-DOF dynamics (RK45)", 2.0)


    # Map to existing simulate scripts (all 8 controllers)
    script_map = {
        "backstepping":           "src.simulate_backstepping",
        "state_feedback":         "src.simulate_state_feedback",
        "feedback_linearization": "src.simulate_feedback_linearization",
        "observer":               "src.simulate_observer",
        "observer_linear":        "src.simulate_observer_linear",
        "mrac":                   "src.simulate_mrac",
        "inn":                    "src.simulate_inn",
    }

    if ctrl_name in script_map:
        mod = __import__(script_map[ctrl_name], fromlist=["simulate", "plot_results", "save_results_csv"])
        with Progress(
            SpinnerColumn("dots12", style="bright_cyan"),
            TextColumn("[bold]Simulating...[/]"),
            BarColumn(bar_width=40, style="bright_blue", complete_style="bright_green"),
            TimeElapsedColumn(), console=console,
        ) as progress:
            task = progress.add_task("", total=1)

            result = mod.simulate(disturbance_enabled=disturbance_enabled, T_final=duration, trajectory_type=trajectory)
            progress.update(task, advance=1)

        # Show results table
        res_table = Table(title="[bold bright_green]Simulation Results[/]",
                          box=box.ROUNDED, border_style="bright_green")
        res_table.add_column("Metric", style="bold cyan")
        res_table.add_column("Value", style="bold white", justify="right")
        res_table.add_row("RMS Error",       f"{result['e_rms']:.4f} m")
        res_table.add_row("Max Error",        f"{result['e_max']:.4f} m")
        res_table.add_row("Control Effort",   f"{result['control_effort']:.2f}")
        res_table.add_row("Duration",         f"{duration:.0f} s")
        console.print(res_table)

        # Generate plots
        animated_loading("Generating publication-quality plots", 1.0)
        os.makedirs("figures", exist_ok=True)
        os.makedirs("results", exist_ok=True)

        mode_name = "Disturbed" if disturbance_enabled else "Nominal"
        
        # We handle plot_results or plot_combined_results depending on what was imported, 
        # but the patch script renamed everything to plot_results
        try:
            mod.plot_results(result, mode_name)
        except AttributeError:
            console.print("[red]Error: Please ensure the patch script updated the plot_results function.[/]")

        mod.save_results_csv(result, f"results/{ctrl_name}_{mode_name.lower()}.csv")

        console.print("[bold green]✓[/] Figures saved in [cyan]figures/[/]")
        console.print("[bold green]✓[/] CSV data saved in [cyan]results/[/]")
    else:
        console.print("[yellow]⚠ No standalone simulate script for this controller[/]")


def main():
    os.system("clear")
    show_banner()

    # ── Step 1: Gazebo? ───────────────────────────────────────────
    console.print()
    console.print(Panel(
        "[bold]Choose your simulation environment:[/]\n\n"
        "  [bold cyan]1[/]  🌐  [bold]Gazebo Harmonic[/] — Full 3D underwater simulation\n"
        "              Real physics, thrusters, cameras, IMU\n\n"
        "  [bold cyan]2[/]  📊  [bold]Standalone[/] — Fast scipy integration + matplotlib plots\n"
        "              No Gazebo needed, instant results with publication plots\n",
        title="[bold bright_cyan]━━ SIMULATION MODE ━━[/]",
        border_style="bright_blue", box=box.ROUNDED,
    ))
    sim_choice = Prompt.ask(
        "[bold bright_yellow]Select mode[/]",
        choices=["1", "2"], default="2")
    use_gazebo = sim_choice == "1"

    # ── Step 2: Controller ────────────────────────────────────────
    console.print()
    show_controller_table()
    ctrl_choice = Prompt.ask(
        "\n[bold bright_yellow]Select controller[/]",
        choices=list(CONTROLLERS.keys()), default="1")
    ctrl_name, ctrl_display, _, _ = CONTROLLERS[ctrl_choice]

    # ── Step 3: Trajectory ────────────────────────────────────────
    console.print()
    console.print(Panel(
        "  [bold cyan]1[/]  〰️  [bold]3D Spline[/] — Complex smooth 3D path (default)\n\n"
        "  [bold cyan]2[/]  ⭕  [bold]Circle[/]    — Constant radius orbit at fixed depth\n\n"
        "  [bold cyan]3[/]  ♾️  [bold]Figure-8[/]  — Path at fixed depth",
        title="[bold bright_cyan]━━ TRAJECTORY SHAPE ━━[/]",
        border_style="bright_blue", box=box.ROUNDED,
    ))
    traj_choice = Prompt.ask(
        "[bold bright_yellow]Select trajectory[/]",
        choices=["1", "2", "3"], default="1")
    traj_map = {"1": "spline", "2": "circle", "3": "figure8"}
    trajectory = traj_map[traj_choice]

    # ── Step 4: Scenario ──────────────────────────────────────────
    console.print()
    console.print(Panel(
        "  [bold cyan]1[/]  ✅  [bold green]Nominal[/]    — Clean conditions, no external forces\n\n"
        "  [bold cyan]2[/]  🌊  [bold red]Disturbed[/]  — Ocean currents + model mismatch (1.2M, 1.4D)",
        title="[bold bright_cyan]━━ SCENARIO ━━[/]",
        border_style="bright_blue", box=box.ROUNDED,
    ))
    scenario = Prompt.ask(
        "[bold bright_yellow]Select scenario[/]",
        choices=["1", "2"], default="1")
    mode = "nominal" if scenario == "1" else "disturbed"
    disturbance = scenario == "2"

    # ── Step 5: Duration ──────────────────────────────────────────
    console.print()
    console.print(Panel(
        "  [bold cyan]1[/]  ⏱️   [bold]30 seconds[/]   — Quick test run\n\n"
        "  [bold cyan]2[/]  ⏱️   [bold]60 seconds[/]   — Standard simulation\n\n"
        "  [bold cyan]3[/]  ⏱️   [bold]80 seconds[/]   — Full trajectory cycle\n\n"
        "  [bold cyan]4[/]  ⏱️   [bold]120 seconds[/]  — Extended evaluation",
        title="[bold bright_cyan]━━ SIMULATION DURATION ━━[/]",
        border_style="bright_blue", box=box.ROUNDED,
    ))
    dur_choice = Prompt.ask(
        "[bold bright_yellow]Select duration[/]",
        choices=["1", "2", "3", "4"], default="2")
    duration_map = {"1": 30.0, "2": 60.0, "3": 80.0, "4": 120.0}
    sim_duration = duration_map[dur_choice]

    # ── Mission Summary ──────────────────────────────────────────
    console.print()
    show_mission_summary(use_gazebo, ctrl_name, ctrl_display, mode, sim_duration)
    console.print()

    if not Confirm.ask("[bold bright_yellow]🚀 Launch mission?[/]", default=True):
        console.print("[dim]Mission aborted.[/]")
        return

    console.print()
    console.print(Rule("[bold bright_green]━━ MISSION START ━━[/]", style="bright_green"))
    console.print()

    # ── Execute ──────────────────────────────────────────────────
    gazebo_proc = None
    ctrl_proc = None

    try:
        if use_gazebo:
            gazebo_proc = launch_gazebo()
            console.print("[dim]Waiting for Gazebo to initialize...[/]")
            time.sleep(3)
            ctrl_proc = launch_controller_gazebo(ctrl_name, sim_duration, disturbance, trajectory)

            console.print()
            console.print(Panel(
                "[bold green]🟢 SIMULATION RUNNING[/]\n\n"
                "[dim]The BlueROV2 is tracking the reference trajectory in Gazebo.\n"
                "The live plotter should appear automatically.\n"
                f"The simulation will automatically stop after {sim_duration}s.\n\n"
                "Press [bold red]Ctrl+C[/] to stop the mission early.[/]",
                border_style="bright_green", box=box.DOUBLE_EDGE,
            ))

            # Keep alive
            while True:
                time.sleep(1)
                
                # Check if Gazebo crashed
                if gazebo_proc.poll() is not None:
                    console.print("[yellow]⚠ Gazebo process ended unexpectedly[/]")
                    break
                    
                # Check if controller finished normally (reached duration)
                if ctrl_proc.poll() is not None:
                    console.print("[bold bright_green]✅ Gazebo Mission Complete (Duration Reached)[/]")
                    break

        else:
            run_standalone(ctrl_name, disturbance, sim_duration, trajectory)
            console.print()
            console.print(Panel(
                "[bold bright_green]✅ MISSION COMPLETE[/]\n\n"
                f"[dim]Controller: {ctrl_display}\n"
                f"Scenario: {mode}\n"
                "Results and figures have been saved.[/]",
                border_style="bright_green", box=box.DOUBLE_EDGE,
            ))

    except KeyboardInterrupt:
        console.print("\n")
        console.print(Rule("[bold red]━━ MISSION ABORT ━━[/]", style="red"))
        animated_loading("Shutting down systems", 1.0)

    finally:
        if ctrl_proc and ctrl_proc.poll() is None:
            os.killpg(os.getpgid(ctrl_proc.pid), signal.SIGTERM)
            console.print("[yellow]⏹ Controller stopped[/]")
        if gazebo_proc and gazebo_proc.poll() is None:
            os.killpg(os.getpgid(gazebo_proc.pid), signal.SIGTERM)
            console.print("[yellow]⏹ Gazebo stopped[/]")

        console.print()
        console.print(Align.center(Text("BlueROV2 Mission Control — Session Ended", style="dim")))
        console.print()


if __name__ == "__main__":
    main()
