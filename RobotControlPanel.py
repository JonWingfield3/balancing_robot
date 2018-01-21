#!/usr/local/bin/python3
# PYTHON_ARGCOMPLETE_OK

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import tkinter as tk
from tkinter import ttk
import numpy as np
from RobotMessenger import RobotMessenger
import argparse
import argcomplete
import time
import random

style.use("seaborn-dark")

class RobotControlPanelApp(tk.Tk):
    def __init__(self, cmd_line_args, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        tk.Tk.wm_title(self, "Robot Control Panel") 
        
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frame = ControlPanel(container, self, cmd_line_args)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.tkraise()

class ControlPanel(tk.Frame):
    def __init__(self, parent, controller, cmd_line_args):
        tk.Frame.__init__(self, parent)

        self.is_init = False
        self.robot_messenger = RobotMessenger(cmd_line_args.verbosity,
                                              cmd_line_args.stub)

        self.fig = Figure(figsize=(20,20), dpi=100)
        self.verbosity = cmd_line_args.verbosity
        self.NUM_PLOT_SAMPLES = cmd_line_args.plot_samples

        self.pid_pitch_set_pt_plt = self.fig.add_subplot(211)
        self.pid_state_plt = self.fig.add_subplot(212)

        self.sample_number = 0
        self.pid_target_lst = [0]*self.NUM_PLOT_SAMPLES
        self.pid_input_lst = [0]*self.NUM_PLOT_SAMPLES
        self.p_error_lst = [0]*self.NUM_PLOT_SAMPLES
        self.i_error_lst = [0]*self.NUM_PLOT_SAMPLES
        self.d_error_lst = [0]*self.NUM_PLOT_SAMPLES

        APP_FONT = ("Verdana", 10)

        # SAMPLING FREQUENCY SLIDERS AND STRINGS
        samp_freq_str = "SAMPLING FREQ -- {} HZ    ".format(
            cmd_line_args.sampling_freq)
        self.samp_freq_str = tk.StringVar()
        self.samp_freq_str.set(samp_freq_str)

        samp_freq_label = tk.Label(self, textvariable=self.samp_freq_str,
                                   font=APP_FONT)

        self.samp_freq_scale = ttk.Scale(self, orient="vertical", length=500, from_=10,
                                   to=100, command=self.update_samp_freq_scale)
        self.samp_freq_scale.set(cmd_line_args.sampling_freq)

        samp_freq_label.pack(side="right")
        self.samp_freq_scale.pack(side="right")

        # BETA SLIDERS AND STRINGS
        beta_gain_str = "BETA GAIN -- {}    ".format(cmd_line_args.beta)
        self.beta_str = tk.StringVar()
        self.beta_str.set(beta_gain_str)

        beta_label = tk.Label(self, textvariable=self.beta_str, font=APP_FONT)

        self.beta_scale = ttk.Scale(self, orient="vertical", length=500, from_=0.0,
                                    to=1.0, command=self.update_beta_scale)
        self.beta_scale.set(cmd_line_args.beta)

        beta_label.pack(side="right")
        self.beta_scale.pack(side="right")

        # PID SLIDERS AND STRINGS
        p_gain_str = "    P GAIN -- {}".format(cmd_line_args.pid_gains[0])
        i_gain_str = "    I GAIN -- {}".format(cmd_line_args.pid_gains[1])
        d_gain_str = "    D GAIN -- {}".format(cmd_line_args.pid_gains[2])

        self.p_gain_str = tk.StringVar()
        self.i_gain_str = tk.StringVar()
        self.d_gain_str = tk.StringVar()

        self.p_gain_str.set(p_gain_str)
        self.i_gain_str.set(i_gain_str)
        self.d_gain_str.set(d_gain_str)

        p_label = tk.Label(self, textvariable=self.p_gain_str, font=APP_FONT)
        i_label = tk.Label(self, textvariable=self.i_gain_str, font=APP_FONT)
        d_label = tk.Label(self, textvariable=self.d_gain_str, font=APP_FONT)

        self.p_scale = ttk.Scale(self, orient="vertical", length=500, 
                                 from_=0, to=1000, command=self.update_scale)
        self.i_scale = ttk.Scale(self, orient="vertical", length=500, 
                                 from_=0, to=10000, command=self.update_scale)
        self.d_scale = ttk.Scale(self, orient="vertical", length=500, 
                                 from_=0, to=50, command=self.update_scale)

        self.p_scale.set(cmd_line_args.pid_gains[0])
        self.i_scale.set(cmd_line_args.pid_gains[1])
        self.d_scale.set(cmd_line_args.pid_gains[2])

        p_label.pack(side="left")
        self.p_scale.pack(side="left")
        i_label.pack(side="left")
        self.i_scale.pack(side="left")
        d_label.pack(side="left")
        self.d_scale.pack(side="left")

# NOT SURE THIS IS WORTH ADDING ATM
#
#        # COMMAND TIMEOUT TEXT ENTRIES AND BUTTON
#        self.steer_cmd_timeout_str = tk.StringVar()
#        self.drive_cmd_timeout_str = tk.StringVar()
#
#        self.steer_cmd_timeout_str.set(str(cmd_line_args.motor_command_timeout[0]))
#        self.drive_cmd_timeout_str.set(str(cmd_line_args.motor_command_timeout[1]))
#
#        steer_cmd_entry = ttk.Entry(self, width=24,
#                                         textvariable=self.steer_cmd_timeout_str)
#        drive_cmd_entry = ttk.Entry(self, width=24,
#                                         textvariable=self.drive_cmd_timeout_str)
#
#        steer_cmd_entry.delete(0, tk.END)
#        steer_cmd_entry.insert(0, self.steer_cmd_timeout_str)
#        drive_cmd_entry.delete(0, tk.END)
#        drive_cmd_entry.insert(0, self.drive_cmd_timeout_str)
#
#        update_cmd_timeouts_btn = ttk.Button(self, text="Update command timeouts",
#                                             width=24, command=self.update_cmd_timeouts)
#
#        steer_cmd_entry.pack(side="top")
#        drive_cmd_entry.pack(side="top") 
#        update_cmd_timeouts_btn.pack(side="top")       
#
        # RESET BUTTON
        reset_btn = ttk.Button(self, command=self.robot_init, text="Reset",
                               width=24).pack(side="top")

        # CALIBRATION BUTTON
        calibrate_btn = ttk.Button(self, command=self.calib, width=24, 
                                   text="Calibrate IMU").pack(side="top")

        # ENABLE MOTORS CHECKBUTTON
        self.enable_mtrs_btn = tk.IntVar()
        self.enable_motors_button = ttk.Checkbutton(self, text="Enable motors", 
                                               variable=self.enable_mtrs_btn,
                                               command=self.set_motor_enables,
                                               width=25, onvalue=1, offvalue=0)
        self.enable_motors_button.pack(side="top")

        # LOGGING WINDOW LABEL AND CHECKBUTTON
        self.log_button_state = tk.StringVar() 
        log_button = ttk.Checkbutton(self, text="Display log", width=25,
                                     variable=self.log_button_state,
                                     onvalue="on", offvalue="off").pack(side="top")

        self.log = tk.StringVar()
        log = tk.Label(self, textvariable=self.log, anchor="e", justify="left",\
                       font=APP_FONT, relief="sunken").pack(side="top")

        # MOTOR CTL KEY BINDINGS, LABELS, STRINGS, AND STATE 
        self.frame = tk.Frame(self, width=100, height=25,
                              borderwidth=5, relief="ridge")
        self.frame.bind("<Key>", self.key_press_handler)
        self.frame.bind("<Button-1>", self.button_callback)
        self.frame.pack(side="bottom")

        self.drive_str_var = tk.StringVar()
        self.drive_str_var.set(
            "Click in box below to enable keyboard control for robot")
        drive_text = ttk.Label(self, textvariable=self.drive_str_var)
        drive_text.pack(side="bottom")

        self.arrow_str = tk.StringVar()
        shape_label = tk.Label(self, textvariable=self.arrow_str,
                               font=APP_FONT)
        shape_label.pack(side="bottom")
        self.key_state = {"a": False, "w": False, "s": False, "d": False}
        self.key_press_time = 0

        # DRIVE PITCH AND STEER DC SLIDERS AND LABELS/STRINGS
        self.steer_dc = cmd_line_args.steer_dc

        drive_pitch_fwd = cmd_line_args.drive_pitch[0]
        drive_pitch_bkwd = cmd_line_args.drive_pitch[1]

        self.drive_pitch_fwd = tk.DoubleVar()
        self.drive_pitch_bkwd = tk.DoubleVar()

        self.drive_pitch_fwd.set(drive_pitch_fwd)
        self.drive_pitch_fwd.set(drive_pitch_bkwd)

        drive_pitch_fwd_str = "FWD DRIVE PITCH -- {0:.4f} [RADIANS]".format(
            drive_pitch_fwd)
        self.drive_pitch_fwd_str = tk.StringVar()
        self.drive_pitch_fwd_str.set(drive_pitch_fwd_str)

        drive_pitch_bkwd_str = "BKWD DRIVE PITCH -- {0:.4f} [RADIANS]".format(
            drive_pitch_bkwd)
        self.drive_pitch_bkwd_str = tk.StringVar()
        self.drive_pitch_bkwd_str.set(drive_pitch_bkwd_str)

        drive_pitch_fwd_label = tk.Label(self, font=APP_FONT,
            textvariable=self.drive_pitch_fwd_str)
        drive_pitch_bkwd_label = tk.Label(self, font=APP_FONT,
            textvariable=self.drive_pitch_bkwd_str)

        drive_pitch_fwd_scale = ttk.Scale(self, orient="horizontal", 
                                          length=100, from_=0.0, to=0.25,
                                          variable=self.drive_pitch_fwd)
        drive_pitch_bkwd_scale = ttk.Scale(self, orient="horizontal",
                                           length=100, from_=0.0, to=-0.25,
                                           variable=self.drive_pitch_bkwd)

        drive_pitch_fwd_scale.set(drive_pitch_fwd)
        drive_pitch_bkwd_scale.set(drive_pitch_bkwd)

        drive_pitch_bkwd_scale.pack(side="bottom")
        drive_pitch_bkwd_label.pack(side="bottom")
        drive_pitch_fwd_scale.pack(side="bottom")
        drive_pitch_fwd_label.pack(side="bottom")

        self.log_filters = cmd_line_args.log_filter
        self.show_arrows = cmd_line_args.show_direction_arrows
        self.mtr_cmd_timeout = cmd_line_args.motor_command_timeout
        self.key_timeout_ms = cmd_line_args.key_timeout_ms

        canvas = FigureCanvasTkAgg(self.fig, self)
        canvas.show()
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.is_init = True
        init_at_startup = not cmd_line_args.no_init_at_startup
        if init_at_startup:
            reset_at_startup = not cmd_line_args.no_reset_at_startup
            self.robot_init(reset_at_startup)
   
    def msg(self, msg_str, msg_data=None):
        if not self.is_init:
            return None

        rsp = self.robot_messenger.message(msg_str, msg_data)
        log_str = self.robot_messenger.dump_log()
        if self.log_button_state.get() == "on":
            if msg_str not in self.log_filters:
                self.log.set(log_str)
        else:
            self.log.set("")

        return rsp

    def robot_init(self, reset=True):
        if reset: 
            self.msg("reset")
            self.enable_mtrs_btn.set(0)

        self.msg("set_pid", (self.p_scale.get(), self.i_scale.get(),
                 self.d_scale.get()))
        self.msg("samp_freq", int(self.samp_freq_scale.get()))
        self.msg("beta", self.beta_scale.get())
        self.msg("cmd_timeout", tuple(self.mtr_cmd_timeout))
        status_rsp, motors_enabled = self.msg("status") 
        if motors_enabled:
            self.enable_mtrs_btn.set(1)

    def update_cmd_timeouts(self):
        if self.is_init:
            self.msg("cmd_timeout", (int(self.steer_cmd_timeout_str.get()),\
                                     int(self.drive_cmd_timeout_str.get())))

    def calib(self):
        if self.is_init: self.msg("calib")

    def command_motion(self):
        if not self.is_init:
            return

        time_now = int(time.time()*1000)
        if time_now > self.key_press_time + self.key_timeout_ms:
            for key in self.key_state.keys():
                self.key_state[key] = False
            self.arrow_str.set("")
            return

        key_pressed = ""
        for key in self.key_state.keys():
            if self.key_state[key]:
                key_pressed = key
                break
        else:
            return

        key_cmd_map = {\
        "a": ("steer_cmd", self.steer_dc),
        "w": ("drive_cmd", self.drive_pitch_fwd.get()),
        "s": ("drive_cmd", self.drive_pitch_bkwd.get()),
        "d": ("steer_cmd", -self.steer_dc)
        }

        dir_strs = {
        "d": "   |\ \n===  \ \n===  /\n   |/",
        "a":  "/|\n   /  ===\n   \  ===\n\|",
        "w":   "    /\ \n    /    \ \n   ||\n   ||",
        "s":  "     ||\n     ||\n     \   /\n     \/"
        }
        
        mtr_cmd, cmd_arg = key_cmd_map[key_pressed]
        status, motors_enabled = self.msg(mtr_cmd, cmd_arg)
        self.enable_mtrs_btn.set(1 if motors_enabled else 0)
        if self.show_arrows == True:
            self.arrow_str.set(dir_strs[key_pressed])

    def update_drive_pitch_strs(self):
        drive_pitch_fwd = "DRIVE PITCH FWD -- {0:.4f} [RADIANS]".format(
            self.drive_pitch_fwd.get())
        self.drive_pitch_fwd_str.set(drive_pitch_fwd)
        drive_pitch_bkwd = "DRIVE PITCH BKWD -- {0:.4f} [RADIANS]".format(
            self.drive_pitch_bkwd.get())
        self.drive_pitch_bkwd_str.set(drive_pitch_bkwd)

    def key_press_handler(self, event):
        key_pressed = str(event.char)
        if self.is_init and key_pressed in self.key_state.keys():
            for key in self.key_state.keys():
                self.key_state[key] = False
            self.key_state[key_pressed] = True
            self.key_press_time = int(time.time()*1000)

    def button_callback(self, event):
        self.frame.focus_set()
        if self.enable_mtrs_btn.get():
            self.drive_str_var.set("Key control enabled")

    def set_motor_enables(self):
        if not self.is_init:
            return
        motors_en = self.enable_mtrs_btn.get()
        self.msg("mtr_config", "en" if motors_en else "dis")
        if not motors_en: self.drive_str_var.set("")

    def update_samp_freq_scale(self, s):
        if not self.is_init:
            return
        new_sample_freq = int(self.samp_freq_scale.get())
        self.msg("samp_freq", new_sample_freq)
        samp_freq_str =\
            "SAMPLING FREQUENCY -- {} HZ    ".format(new_sample_freq)
        self.samp_freq_str.set(samp_freq_str)
 
    def update_beta_scale(self, s):
        if not self.is_init:
            return
        beta_gain = float(self.beta_scale.get())
        self.msg("beta", beta_gain)
        beta_str = "BETA GAIN -- {0:.2f}    ".format(beta_gain)
        self.beta_str.set(beta_str)

    def update_scale(self, s):
        if not self.is_init:
            return
        p_gain_str = "    P GAIN -- {}".format(int(self.p_scale.get()))
        self.p_gain_str.set(p_gain_str)
        i_gain_str = "    I GAIN -- {}".format(int(self.i_scale.get()))
        self.i_gain_str.set(i_gain_str)
        d_gain_str = "    D GAIN -- {}".format(int(self.d_scale.get()))
        self.d_gain_str.set(d_gain_str)
        self.msg("set_pid", (self.p_scale.get(), self.i_scale.get(),
                 self.d_scale.get()))
    
    def updater(self, s):
        if not self.is_init:
            return
        ret = self.msg("pid_state", "weighted")
        if ret:
            pid_target, pid_input, p_error, i_error, d_error, pid_output = ret

            self.pid_target_lst.pop(0)
            self.pid_target_lst.append(pid_target)
            self.pid_input_lst.pop(0)
            self.pid_input_lst.append(pid_input)
            self.p_error_lst.pop(0)
            self.p_error_lst.append(p_error)
            self.i_error_lst.pop(0)
            self.i_error_lst.append(i_error)
            self.d_error_lst.pop(0)
            self.d_error_lst.append(d_error)

            sample_lst = range(self.sample_number - self.NUM_PLOT_SAMPLES,
                               self.sample_number)
            self.sample_number += 1

            self.pid_pitch_set_pt_plt.clear()
            self.pid_state_plt.clear()

            self.pid_pitch_set_pt_plt.plot(sample_lst, self.pid_input_lst, ".-r",
                                     label="pitch [radians]")
            self.pid_pitch_set_pt_plt.plot(sample_lst, self.pid_target_lst, ".-g",
                                     label="set point [radians]")
            self.pid_pitch_set_pt_plt.legend(bbox_to_anchor=(0, 1.02, 1., 0.102),
                                             loc=3, ncol=3, mode="expand", 
                                             borderaxespad=0.)
            self.pid_pitch_set_pt_plt.get_xaxis().set_visible(False)
            self.pid_pitch_set_pt_plt.grid(True, which="both")

            self.pid_state_plt.plot(sample_lst, self.p_error_lst, ".-r",
                                        label="weighted p error")
            self.pid_state_plt.plot(sample_lst, self.i_error_lst, ".-g",
                                        label="weighted i error")
            self.pid_state_plt.plot(sample_lst, self.d_error_lst, ".-b",
                                        label="weighted d error")
            self.pid_state_plt.legend(bbox_to_anchor=(0, 1.02, 1., 0.102),
                                          loc=3, ncol=3, mode="expand",
                                          borderaxespad=0.)
            self.pid_state_plt.set_xlabel("Sample Number")
            self.pid_state_plt.grid(True, which="both")

            self.command_motion() 
            self.update_drive_pitch_strs()

if __name__ == '__main__':
    DEFAULT_VERBOSITY = 0
    DEFAULT_APP_UPDATE_FREQ = 50
    DEFAULT_PLOT_SAMPLES = 50
    DEFAULT_LOG_FILTER = []
    DEFAULT_KEY_TIMEOUT = 200
    DEFAULT_PID_GAINS = [700, 3000, 8]
    DEFAULT_SAMP_FREQ = 70
    DEFAULT_BETA = 0.1
    DEFAULT_DRIVE_PITCHES = [0.4, -0.4]
    DEFAULT_STEER_DC = 25
    DEFAULT_MTR_CMD_TIMEOUTS = [200, 500]

    parser = argparse.ArgumentParser(description="Robot Control Panel Application. "\
                                     "Sliders, switches, and data visualization for "\
                                     "the self balancing robot")
    # APP OPTIONS
    parser.add_argument("--verbosity", "-v",
                        type=int,
                        metavar="v",
                        default=DEFAULT_VERBOSITY, 
                        help="set log verbosity, default is: {}".format(DEFAULT_VERBOSITY))
    parser.add_argument("--stub",
                        type=str,
                        default="none",
                        choices=["none", "silent", "mock", "mock-err"],
                        help="stub out robot messenger")

    parser.add_argument("--app-update-freq",
                        type=int,
                        metavar="f",
                        default=DEFAULT_APP_UPDATE_FREQ,
                        help="set app's update freq, default is: {}".format(DEFAULT_APP_UPDATE_FREQ))
    parser.add_argument("--plot-samples",
                        type=int,
                        metavar="N",
                        default=DEFAULT_PLOT_SAMPLES,
                        help="set plot sample size, default is: {}".format(DEFAULT_PLOT_SAMPLES))
    parser.add_argument("--show-direction-arrows",
                        action="store_true",
                        default="False",
                        help="show arrows for keyboard commands")
    parser.add_argument("--log-filter",
                        type=str,
                        default=DEFAULT_LOG_FILTER,
                        nargs='?',
                        choices=[
                          "mtr_config","steer_cmd","drive_cmd","set_pid",\
                          "pid_state","calib", "samp_freq", "reset"\
                        ],
                        help="set msgs to remove from log")
    parser.add_argument("--no-init-at-startup", "-s",
                        action="store_true",
                        default=False,
                        help="when enabled doesn't init robot at startup")
    parser.add_argument("--no-reset-at-startup", "-r",
                        action="store_true",
                        default=False,
                        help="initializes robot without resetting it")
    parser.add_argument("--key-timeout-ms",
                        type=int,
                        default=DEFAULT_KEY_TIMEOUT,
                        metavar="MS",
                        help="timeout for key commands in GUI, "\
                        "default is {}".format(DEFAULT_KEY_TIMEOUT))
    # ROBOT OPTIONS
    parser.add_argument("--pid-gains",
                        nargs=3,
                        type=int,
                        default=DEFAULT_PID_GAINS,
                        metavar=("P", "I", "D"),
                        help="initial pid gains, default pid gains are {}".format(DEFAULT_PID_GAINS))

    parser.add_argument("--sampling-freq",
                        type=int,
                        default=DEFAULT_SAMP_FREQ,
                        metavar="fs",
                        choices=range(10, 100, 5),
                        help="initial sampling freq for IMU, "\
                        "default sampling freq is {}".format(DEFAULT_SAMP_FREQ))

    parser.add_argument("--beta",
                        type=float,
                        default=DEFAULT_BETA,
                        help="initial beta gain used by madgwick filter, "\
                        "default is: {}".format(DEFAULT_BETA))

    parser.add_argument("--drive-pitch",
                        nargs=2,
                        type=float,
                        default=DEFAULT_DRIVE_PITCHES,
                        metavar=("FWD_DRIVE_PITCH", "BKWD_DRIVE_PITCH"),
                        help="inital pitches used for keyboard control, "\
                        "defaults are: {}".format(DEFAULT_DRIVE_PITCHES))

    parser.add_argument("--steer-dc",
                        type=int,
                        default=DEFAULT_STEER_DC,
                        help="duty cycle offsets to use for keyboard control, "\
                        "default is {}".format(DEFAULT_STEER_DC))

    parser.add_argument("--motor-command-timeout",
                        nargs=2,
                        type=int,
                        default=DEFAULT_MTR_CMD_TIMEOUTS,
                        metavar=("STEER_CMD_TIMEOUT", "DRIVE_CMD_TIMEOUT"),
                        help="number of milliseconds after which command expires for robot, "\
                        "defaults are {} for steer and drive commands".format(DEFAULT_MTR_CMD_TIMEOUTS))

    argcomplete.autocomplete(parser)
    cmd_line_args = parser.parse_args()

    app = RobotControlPanelApp(cmd_line_args=cmd_line_args)
    ani = animation.FuncAnimation(app.frame.fig, app.frame.updater,
                                  interval=1000.0/cmd_line_args.app_update_freq)
    app.mainloop()
