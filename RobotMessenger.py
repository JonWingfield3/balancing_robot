#!/usr/local/bin/python3
# PYTHON_ARGCOMPLETE_OK

import time
import sys
import struct
import os
import re
import datetime
import collections
import math
import serial
import argparse
import argcomplete
import random

class RobotMessenger:
    #/////////////////////////////////////////////////////////////////////////
    ACK_BYTE = 0x22
    ERR_BYTE = 0x33
    SYNC_BYTE = 0x55
    PORT = "/dev/tty.FSHACK-DevB"
    BAUD = 115200 
    CRITICAL=0
    WARNING=1
    INFO=2
    DEBUG=3 

    #/////////////////////////////////////////////////////////////////////////
    @staticmethod
    def get_time_str():
        now = datetime.datetime.now()
        return "{:02d}:{:02d}:{:02d}.{:06d}".format(
            now.hour, now.minute, now.second, now.microsecond)

    #/////////////////////////////////////////////////////////////////////////
    @staticmethod
    def get_millis():
        now = datetime.datetime.now()
        return (now.microsecond / 1000.0)

    #/////////////////////////////////////////////////////////////////////////
    def log(self, log_msg, verbosity):
        log_time = self.get_time_str()
        if self.verbosity >= verbosity:
            v_strs = ["CRITICAL", "WARNING", "INFO", "DEBUG"]
            self.log_str += "[{}] {}: {}\n".format(
                log_time, v_strs[min(verbosity, 3)], log_msg)

    #/////////////////////////////////////////////////////////////////////////
    def dump_log(self):
      log_str = self.log_str
      self.log_str = ""
      return log_str

    #/////////////////////////////////////////////////////////////////////////
    @staticmethod
    def pack_float(actual_val, min_val, max_val, num_bytes):
        clamped_val = min(max(min_val, actual_val), max_val)
        return int((2**(8 * num_bytes) - 1) *
                   ((clamped_val - min_val) / (max_val - min_val)))

    #/////////////////////////////////////////////////////////////////////////
    @staticmethod
    def unpack_float(packed_val, min_val, max_val, num_bytes):
        return (max_val - min_val) * (packed_val / (2.0**(8*num_bytes) - 1)) + min_val

    #/////////////////////////////////////////////////////////////////////////
    def __init__(self, verbosity=0, stub="none"):
        self.verbosity = verbosity
        self.stub = stub
        self.log_str = ""
        self.send_time = list()
        if self.stub != "none":
            self.read = lambda x: 0
            self.write = lambda x: 0
            self.flush = lambda x: 0
        else:
            try:
                self.ser = serial.Serial(self.PORT, timeout=5.0)
                self.log("opened port {} at {} baud with timeout set to {} seconds".format(
                    self.PORT, self.BAUD, 0), self.INFO)
                self.read = self.ser.read
                self.write = self.ser.write
                self.flush = self.ser.flush
            except serial.SerialException:
                self.log("unable to open serial port: {}".format(self.PORT), self.CRITICAL)
                print("can't open serial")
                self.terminate()

    #/////////////////////////////////////////////////////////////////////////
    def terminate(self):
        print(self.dump_log())
        quit()

    #/////////////////////////////////////////////////////////////////////////
    @classmethod
    def build_header(cls, msg_id, pld_len=0):
        return bytearray([cls.SYNC_BYTE, (((pld_len & 0xf) << 4) | (msg_id & 0xf))])

    #/////////////////////////////////////////////////////////////////////////
    @staticmethod
    def get_crc(msg):
        crc = 0
        for byte in msg:
            crc ^= byte
        return crc

    #/////////////////////////////////////////////////////////////////////////
    def create_msg(self):
        msg = self.build_header(self.msg_id, len(self.pld))
        for pld_byte in self.pld:
            msg.append(pld_byte)
        msg.append(self.get_crc(msg))
        return msg

    #/////////////////////////////////////////////////////////////////////////
    def create_simple_msg(self):
        self.log("creating simple request... (no payload)", self.INFO)
        msg = self.build_header(self.msg_id)
        msg.append(self.get_crc(msg))
        return msg

    #/////////////////////////////////////////////////////////////////////////
    def create_set_dc_msg(self):
        self.pld = struct.pack("<bb", self.pld_args[0], self.pld_args[1])
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_mtr_config_msg(self):
        enable_mtr_ctl = (self.pld_args == "en")
        self.pld = bytearray([enable_mtr_ctl])
        return self.create_msg() 

    #/////////////////////////////////////////////////////////////////////////
    def create_mtr_cmd_msg(self):
        cmd_map = {"fwd":0, "bkwd":1, "left":2, "right":3}
        cmd_val = cmd_map[self.pld_args[0]]
        speed_map = {"slow":0, "med":1, "fast":2}
        speed_val = speed_map[self.pld_args[1]]
        self.log("sending {} command at {} speed".format(self.pld_args[0], self.pld_args[1]), self.INFO)
        self.pld = bytearray([ ((speed_val & 0xf) << 4) | (cmd_val & 0xf)])
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_steer_cmd_msg(self):
        steer_dc = self.pld_args
        self.log("setting steer dc to {}".format(steer_dc), self.INFO)
        self.pld = struct.pack("<b", steer_dc)
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_drive_cmd_msg(self):
        drive_pitch = self.pld_args
        self.log("setting drive pitch to {}".format(drive_pitch), self.INFO)
        DRIVE_PITCH_MIN, DRIVE_PITCH_MAX, DRIVE_PITCH_BYTES = -0.25, 0.25, 1
        packed_drive_pitch = self.pack_float(drive_pitch, DRIVE_PITCH_MIN,
                                             DRIVE_PITCH_MAX, DRIVE_PITCH_BYTES)
        self.pld = struct.pack("<B", packed_drive_pitch)
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_cmd_timeout_msg(self):
        steer_cmd_timeout, drive_cmd_timeout = self.pld_args
        self.log("setting steer command timeout to {}, "\
                 "drive command timeout to {}".format(
                 steer_cmd_timeout, drive_cmd_timeout), self.INFO)
        self.pld = struct.pack("<HH", steer_cmd_timeout, drive_cmd_timeout)
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_set_pid_msg(self):
        p, i, d = self.pld_args
        self.log("setting p = {}, i = {}, d = {}".format(p,i,d), self.INFO)
        self.pld = struct.pack("<HHH", int(p), int(i), int(d))
        return self.create_msg() 

    #/////////////////////////////////////////////////////////////////////////
    def create_pid_state_msg(self):
        weighted = self.pld_args == "weighted"
        self.pld = bytearray([0x01 if weighted else 0x00])
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_samp_freq_msg(self):
        samp_freq = max(min(int(self.pld_args), 100), 10)
        self.pld = struct.pack("<H", samp_freq)
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def create_beta_gain_msg(self):
        beta = max(min(float(self.pld_args), 1.0), 0.0)
        BETA_GAIN_MIN = 0.0
        BETA_GAIN_MAX = 1.0
        BETA_GAIN_BYTES = 2
        packed_beta = self.pack_float(beta, BETA_GAIN_MIN, BETA_GAIN_MAX, BETA_GAIN_BYTES)
        self.pld = struct.pack("<H", packed_beta)
        return self.create_msg()

    #/////////////////////////////////////////////////////////////////////////
    def send_message(self, msg, pld_args=0):
        msg_desc = self.get_msg_desc(msg)
        if msg_desc[0] == -1:
            self.log("invalid msg: {}".format(msg), self.CRITICAL)
            return -1
        else:
            self.pld_args = pld_args
            self.msg_id, self.create_function, self.pld_parser = msg_desc
            self.log("sending message id {}".format(self.msg_id), self.INFO)
            msg = self.create_function()
            self.log("data to be sent: {}".format([format(byte, "#04x") for byte in msg]), self.DEBUG)
            self.write(msg)
            self.send_time = self.get_millis()
            if self.stub == "none": self.flush()
            return 0

    #/////////////////////////////////////////////////////////////////////////
    def parse_message(self):
        self.parse_function(self)

    #/////////////////////////////////////////////////////////////////////////
    def get_response(self):
        recvd_ack = self.read()
        if not recvd_ack:
            self.log("no ack received", self.CRITICAL)
            print("no ack received")
            self.terminate()
        if ord(recvd_ack) != self.ACK_BYTE:
            if ord(recvd_ack) != self.ERR_BYTE:
              self.log("didn't receive expected ack byte ({} != {})".format(
                       ord(recvd_ack), self.ACK_BYTE), self.CRITICAL)
            else:
              self.log("received error response!", self.CRITICAL)
            print("not ack")
            self.terminate()

        self.log("received ack (0x22)", self.INFO)
        recvd_sync = self.read()
        if ord(recvd_sync) != self.SYNC_BYTE:
            self.log("didn't receive expected sync byte ({} != {})".format(
                ord(recvd_sync), self.SYNC_BYTE), self.WARNING)
            print("didnt receive expected sync")
            self.terminate()

        header = ord(self.read())
        self.recvd_pld_len = (header >> 4) & 0xf
        self.recvd_msg_id = (header & 0xf)

        self.recvd_pld = bytearray([ord(self.read()) for i in range(self.recvd_pld_len)])

        recvd_crc = ord(self.read())
        self.recv_time = self.get_millis()
        
        msg = [byte for byte in self.recvd_pld]
        msg.append(header)
        msg.append(self.SYNC_BYTE)
        calc_crc = self.get_crc(msg)
        if calc_crc != recvd_crc:
            self.log("received crc doesn't match expected crc ({} != {})".format(
                    recvd_crc, calc_crc), self.WARNING)
        else:
            self.log("received crc matches expected crc of {}".format(recvd_crc), self.INFO)
        
        self.log("received message w/ msg id = {}, length = {}, "
            "payload =  {}".format(self.recvd_msg_id, self.recvd_pld_len, [
                    format(byte, "#04x") for byte in self.recvd_pld]), self.DEBUG)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()


    #/////////////////////////////////////////////////////////////////////////
    def parse_response(self):
        if self.stub != "none":
            self.log("stubbed, no actual responses expected", self.INFO)
            if self.stub == "silent": return
        else:
            self.get_response()

            if self.recvd_msg_id != self.msg_id:
                self.log("received msg id doesn't match expected msg id ({} != {})" .format(
                            self.recvd_msg_id, self.msg_id), self.WARNING)
            rtt = self.recv_time - self.send_time
            if rtt < 0:
                rtt += 1000
            self.log("rtt: {:03f} ms".format(rtt), self.INFO)

        return self.pld_parser()

    #/////////////////////////////////////////////////////////////////////////
    def message(self, msg, pld_args=None):
        self.dump_log()
        self.send_message(msg, pld_args)
        return self.parse_response()

    #/////////////////////////////////////////////////////////////////////////
    def parse_status_rsp(self):
        if self.stub != "none":
            mock_status_rsps = {"mock": bytearray([0x80]),\
                                "mock-err": bytearray([0x02])}
            self.recvd_pld = mock_status_rsps[self.stub]
            self.log("{} type response, setting recvd pld to = {}".format(
                self.stub, self.recvd_pld), self.INFO)

        status_strs = ["No err", "Bad payload length", "Panic Mode"]
        status_rsp = self.recvd_pld[0] & 0x7f
        motors_enabled = ((self.recvd_pld[0] & 0x80) == 0x80)
        if status_rsp not in (0,1,2):
            self.log("Invalid status response received! {}".format(
                status_rsp), self.CRITICAL) 
        self.log("Received Response! Status = {}, motors enabled = {}".format(
            status_strs[status_rsp], motors_enabled), self.CRITICAL)
        return status_rsp, motors_enabled

    #/////////////////////////////////////////////////////////////////////////
    def parse_dc_rsp(self):
        if self.stub != "none":
            mock_dc_rsps = {\
            "mock": struct.pack("<BB", random.randint(0, 100), random.randint(0, 100)),\
            "mock-err": bytearray([0x00])\
            }
            self.recvd_pld = mock_dc_rsps[self.stub]
            self.log("{} type response, setting recvd pld to = {}".format(
                self.stub, self.recvd_pld), self.INFO)

        if len(self.recvd_pld) != 2:
            self.log("received invalid dc payload! Length = {}, data = {}".format(
                     len(self.recvd_pld), self.recvd_pld), self.CRITICAL)
            return 0, 0

        left_dc, right_dc = struct.unpack("<BB", self.recvd_pld)
        self.log("left duty cycle = {}, right duty cycle = {}".format(
            left_dc, right_dc), self.CRITICAL)
        return left_dc, right_dc

    #/////////////////////////////////////////////////////////////////////////
    def parse_pid_rsp(self):
       if self.pld_args == "weighted":
            return self.parse_weighted_pid_rsp()
       else:
            return self.parse_unweighted_pid_rsp()

    #/////////////////////////////////////////////////////////////////////////
    def parse_unweighted_pid_rsp(self):
        if self.stub != "none":
            mock_pid_rsps = {\
            "mock": struct.pack("<HHBBBb",\
                                random.randint(0, 65535), random.randint(0, 65535),\
                                random.randint(0, 255), random.randint(0, 255),\
                                random.randint(0, 255), random.randint(-128, 127)),\
            "mock-err": bytearray([0x00])\
            }
            self.recvd_pld = mock_pid_rsps[self.stub]
            self.log("{} type response, setting recvd pld to = {}".format(
                self.stub, self.recvd_pld), self.INFO)

        if len(self.recvd_pld) != 8:
            self.log("received invalid pid payload! Length = {}, data = {}".format(
                     len(self.recvd_pld), self.recvd_pld), self.CRITICAL)
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        
        PID_TARGET_MIN = -0.8
        PID_TARGET_MAX = 0.8
        PID_TARGET_BYTES = 2

        P_ERROR_MIN = -2.0
        P_ERROR_MAX = 2.0
        P_ERROR_BYTES = 1

        I_ERROR_MIN = -5.0
        I_ERROR_MAX = 5.0
        I_ERROR_BYTES = 1

        D_ERROR_MIN = -10.0
        D_ERROR_MAX = 10.0
        D_ERROR_BYTES = 1
 
        PID_INPUT_MIN = -2.0
        PID_INPUT_MAX = 2.0
        PID_INPUT_BYTES = 2

        packed_pid_target, packed_pid_input, packed_p_error, packed_i_error,\
            packed_d_error, pid_output = struct.unpack("<HHBBBb", self.recvd_pld)

        pid_target = self.unpack_float(packed_pid_target,
                                       PID_TARGET_MIN,
                                       PID_TARGET_MAX,
                                       PID_TARGET_BYTES)
        pid_input = self.unpack_float(packed_pid_input,
                                      PID_INPUT_MIN,
                                      PID_INPUT_MAX,
                                      PID_INPUT_BYTES)
        p_error = self.unpack_float(packed_p_error,
                                    P_ERROR_MIN,
                                    P_ERROR_MAX,
                                    P_ERROR_BYTES)
        i_error = self.unpack_float(packed_i_error,
                                    I_ERROR_MIN,
                                    I_ERROR_MAX,
                                    I_ERROR_BYTES)
        d_error = self.unpack_float(packed_d_error,
                                    D_ERROR_MIN,
                                    D_ERROR_MAX,
                                    D_ERROR_BYTES)

        self.log("Received pid state payload!\npid_target = {0:.2f}, pid_input = {0:.2f}\n"\
                 "p_error = {0:.2f}, i_error = {0:.2f}, d_error = {0:.2f}\n"\
                 "pid_output = {0:.2f}".format(
                 pid_target, pid_input, p_error, i_error, d_error, pid_output),
                 self.CRITICAL)
        return pid_target, pid_input, p_error, i_error, d_error, pid_output

    #/////////////////////////////////////////////////////////////////////////
    def parse_weighted_pid_rsp(self):
        if self.stub != "none":
            mock_pid_rsps = {\
            "mock": struct.pack("<HHHHHb",\
                                random.randint(0, 65535), random.randint(0, 65535),\
                                random.randint(0, 65535), random.randint(0, 65535),\
                                random.randint(0, 65535), random.randint(-128, 127)),\
            "mock-err": bytearray([0x00])\
            }
            self.recvd_pld = mock_pid_rsps[self.stub]
            self.log("{} type response, setting recvd pld to = {}".format(
                self.stub, self.recvd_pld), self.INFO)

        if len(self.recvd_pld) != 11:
            self.log("received invalid wtd pid payload! Length = {}".format(
                len(self.recvd_pld)), self.CRITICAL)
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
 
        PID_TARGET_MIN = -0.8
        PID_TARGET_MAX = 0.8
        PID_TARGET_BYTES = 2

        PID_ERROR_MIN = -100.0
        PID_ERROR_MAX = 100.0
        PID_ERROR_BYTES = 2

        PID_INPUT_MIN = -2.0
        PID_INPUT_MAX = 2.0
        PID_INPUT_BYTES = 2

        packed_pid_target, packed_pid_input, packed_p_error, packed_i_error,\
            packed_d_error, pid_output = struct.unpack("<HHHHHb", self.recvd_pld)

        pid_target = self.unpack_float(packed_pid_target,
                                       PID_TARGET_MIN,
                                       PID_TARGET_MAX,
                                       PID_TARGET_BYTES)
        pid_input = self.unpack_float(packed_pid_input,
                                      PID_INPUT_MIN,
                                      PID_INPUT_MAX,
                                      PID_INPUT_BYTES)
        p_error = self.unpack_float(packed_p_error,
                                    PID_ERROR_MIN,
                                    PID_ERROR_MAX,
                                    PID_ERROR_BYTES)
        i_error = self.unpack_float(packed_i_error,
                                    PID_ERROR_MIN,
                                    PID_ERROR_MAX,
                                    PID_ERROR_BYTES)
        d_error = self.unpack_float(packed_d_error,
                                    PID_ERROR_MIN,
                                    PID_ERROR_MAX,
                                    PID_ERROR_BYTES)
     
        self.log("Received weighted pid state payload!\n"
                 "pid_target = {0:.2f}, pid_input = {0:.2f}\n"\
                 "weighted p_error = {2:.2f}, weighted i_error = {2:.2f}, "\
                 "weighted d_error = {2:.2f}\npid_output = {2:.2f}".format(
                 pid_target, pid_input, p_error, i_error, d_error, pid_output),
                 self.CRITICAL)

        return pid_target, pid_input, p_error, i_error, d_error, pid_output

    #/////////////////////////////////////////////////////////////////////////
    def parse_mpu_rsp(self):
        if self.stub != "none":
            mock_pid_rsps = {\
            "mock": struct.pack("<HHHHHHB",\
                                random.randint(0, 65535), random.randint(0, 65535),\
                                random.randint(0, 65535), random.randint(0, 65535),\
                                random.randint(0, 65535), random.randint(0, 65535),\
                                random.randint(0, 255)),\
            "mock-err": bytearray([0x00])\
            }
            self.recvd_pld = mock_pid_rsps[self.stub]
            self.log("{} type response, setting recvd pld to = {}".format(
                self.stub, self.recvd_pld), self.INFO)

        if len(self.recvd_pld) != 13:
            self.log("received invalid mpu data payload! Length = {}, data = {}".format(
                     len(self.recvd_pld), self.recvd_pld), self.CRITICAL)
            return 0,0,0,0,0,0,0

        packed_gyro_x, packed_gyro_y, packed_gyro_z, packed_accel_x, packed_accel_y,\
            packed_accel_z, packed_temp = struct.unpack("<HHHHHHB", self.recvd_pld)
 
        GYRO_MIN = -15.0
        GYRO_MAX = 15.0
        GYRO_BYTES = 2

        gyro_x = self.unpack_float(packed_gyro_x, GYRO_MIN, GYRO_MAX, GYRO_BYTES)
        gyro_y = self.unpack_float(packed_gyro_y, GYRO_MIN, GYRO_MAX, GYRO_BYTES)
        gyro_z = self.unpack_float(packed_gyro_z, GYRO_MIN, GYRO_MAX, GYRO_BYTES)

        ACCEL_MIN = -4.0
        ACCEL_MAX = 4.0
        ACCEL_BYTES = 2
 
        accel_x = self.unpack_float(packed_accel_x, ACCEL_MIN, ACCEL_MAX, ACCEL_BYTES)
        accel_y = self.unpack_float(packed_accel_y, ACCEL_MIN, ACCEL_MAX, ACCEL_BYTES)
        accel_z = self.unpack_float(packed_accel_z, ACCEL_MIN, ACCEL_MAX, ACCEL_BYTES)
        
        TEMP_MIN = 0.0
        TEMP_MAX = 100.0
        TEMP_BYTES = 1

        temp = self.unpack_float(packed_temp, TEMP_MIN, TEMP_MAX, TEMP_BYTES)

        self.log("Received MPU data payload!\ngyro x = {0:.2f}, gyro y = {0:.2f}, gyro z = {0:.2f}\n"
                 "accel x = {0:.2f}, accel y = {0:.2f}, accel z = {0:.2f}, temp = {0:.2f}".format(
                 gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp), self.CRITICAL)

        return gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp

    #/////////////////////////////////////////////////////////////////////////
    def get_msg_desc(self, msg_str):
        msg_desc = {
            "get_dc":      (0,  self.create_simple_msg, self.parse_dc_rsp),
            "mtr_config":  (1,  self.create_mtr_config_msg, self.parse_status_rsp),
            "steer_cmd":   (2,  self.create_steer_cmd_msg, self.parse_status_rsp),
            "drive_cmd":   (3,  self.create_drive_cmd_msg, self.parse_status_rsp),
            "cmd_timeout": (4,  self.create_cmd_timeout_msg, self.parse_status_rsp),
            "set_pid":     (5,  self.create_set_pid_msg, self.parse_status_rsp),
            "pid_state":   (6,  self.create_pid_state_msg, self.parse_pid_rsp),
            "mpu":         (7,  self.create_simple_msg, self.parse_mpu_rsp),
            "calib":       (8,  self.create_simple_msg, self.parse_status_rsp),
            "samp_freq":   (9,  self.create_samp_freq_msg, self.parse_status_rsp),
            "beta":        (10, self.create_beta_gain_msg, self.parse_status_rsp),
            "reset":       (11, self.create_simple_msg, self.parse_status_rsp),
            "status":      (12, self.create_simple_msg, self.parse_status_rsp)
        }
        return msg_desc.get(msg_str, (-1,-1,-1))
 
#/////////////////////////////////////////////////////////////////////////
if __name__ == "__main__":
    pld_arg_list=[]

    parser = argparse.ArgumentParser(description="Python messenger interface for self balancing robot")

    parser.add_argument("--verbosity", "-v", type=int, metavar="v",
        help="verbosity level. 0) print only parsed responses or "
        "critical errors (default). 1) print non-critical "
        "errors. 2) print all received data. 3) print all "
        "sent data. 4) print everything", default=0)

    parser.add_argument("--rtt", help="display round trip time", default=False,
                        action="store_true")

    parser.add_argument("--timeout", type=float, metavar="t",
                        help="set timeout for serial port (default = 0.100 s)",
                        default=0.100)

    parser.add_argument("--repeat", type=int, metavar="n",
                        help="send message n times", default=1)
    parser.add_argument("--delay-ms", type=int, default=100)

    parser.add_argument("--stub", help="stub out recipient", type=str, default="none", 
                        choices=["none", "silent", "mock", "mock-err"])

    subparsers = parser.add_subparsers()

    get_dc_parser = subparsers.add_parser("get_dc")
    get_dc_parser.set_defaults(func=lambda args: ("get_dc", (0)))
   
    mtr_config_parser = subparsers.add_parser("mtr_config")
    mtr_config_parser.add_argument("enable_disable", type=str, choices=["en", "dis"])
    mtr_config_parser.set_defaults(func=lambda args: ("mtr_config", args.enable_disable))

    steer_cmd_parser = subparsers.add_parser("steer_cmd")
    steer_cmd_parser.add_argument("steer_dc", type=int)
    steer_cmd_parser.set_defaults(func=lambda args: ("steer_cmd", args.steer_dc))

    drive_cmd_parser = subparsers.add_parser("drive_cmd")
    drive_cmd_parser.add_argument("drive_pitch", type=float)
    drive_cmd_parser.set_defaults(func=lambda args: ("drive_cmd", args.drive_pitch))

    cmd_timeout_parser = subparsers.add_parser("cmd_timeout")
    cmd_timeout_parser.add_argument("steer_cmd_timeout", type=int)
    cmd_timeout_parser.add_argument("drive_cmd_timeout", type=int)
    cmd_timeout_parser.set_defaults(func=lambda args: ("cmd_timeout", args.steer_cmd_timeout,\
                                                                      args.drive_cmd_timeout))

    set_pid_parser = subparsers.add_parser("set_pid")
    set_pid_parser.add_argument("p", type=int, default=0)
    set_pid_parser.add_argument("i", type=int, default=0)
    set_pid_parser.add_argument("d", type=int, default=0)
    set_pid_parser.set_defaults(func=lambda args: ("set_pid", (args.p, args.i, args.d)))

    pid_state_parser = subparsers.add_parser("pid_state")
    pid_state_parser.add_argument("weighted", type=str, help="weight pid errors by pid gains",
                                  choices=["weighted", "unweighted"])
    pid_state_parser.set_defaults(func=lambda args: ("pid_state", args.weighted))

    mpu_parser = subparsers.add_parser("mpu")
    mpu_parser.set_defaults(func=lambda args: ("mpu", 0))

    calib_parser = subparsers.add_parser("calib")
    calib_parser.set_defaults(func=lambda args: ("calib", 0))

    samp_freq_parser = subparsers.add_parser("samp_freq")
    samp_freq_parser.add_argument("sampling_freq", type=int, default=1000)
    samp_freq_parser.set_defaults(func=lambda args: ("samp_freq", args.sampling_freq))

    beta_gain_parser = subparsers.add_parser("beta")
    beta_gain_parser.add_argument("beta_gain", type=float, default=0.1)
    beta_gain_parser.set_defaults(func=lambda args: ("beta", args.beta_gain))

    system_reset_parser = subparsers.add_parser("reset")
    system_reset_parser.set_defaults(func=lambda args: ("reset", 0))

    system_status_parser = subparsers.add_parser("status")
    system_status_parser.set_defaults(func=lambda args: ("status", 0))

    argcomplete.autocomplete(parser)
    cmd_line_args = parser.parse_args()
    msg, pld_args = cmd_line_args.func(cmd_line_args)

    messenger = RobotMessenger(cmd_line_args.verbosity, cmd_line_args.stub)

    for send_num in range(cmd_line_args.repeat):
        rsp = messenger.message(msg, pld_args)
        if cmd_line_args.repeat > 1:
            print("MSG NO {}".format(send_num))
        print(messenger.dump_log())
        time.sleep(cmd_line_args.delay_ms/1000.0)
