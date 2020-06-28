# A utility class to test resonances of the printer
#
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import chelper
import logging, math, multiprocessing

MIN_TEST_FREQ = 10.

def float_range(s, e, i):
    while s <= e + 1e-9:
        yield s
        s += i

class RawValues:
    def __init__(self, raw_values):
        self.n = n = raw_values.n
        self.t = [raw_values.t[i] for i in range(0, n)]
        self.ax = [raw_values.ax[i] for i in range(0, n)]
        self.ay = [raw_values.ay[i] for i in range(0, n)]
        self.az = [raw_values.az[i] for i in range(0, n)]

def measure_accel(connection, time):
    ffi_main, ffi_lib = chelper.get_ffi()
    raw_values = None
    adxl345 = ffi_lib.adxl345_init()
    if adxl345 != ffi_main.NULL:
        values_wrapper = ffi_lib.adxl345_measure(adxl345, time)
        if values_wrapper != ffi_main.NULL:
            raw_values = RawValues(values_wrapper)
            ffi_lib.accel_values_free(values_wrapper)
        ffi_lib.adxl345_free(adxl345)
    connection.send(raw_values)
    connection.close()

class ResonanceTester:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.seg_sec = config.getfloat('seg_sec', 0.0005, above=0.0)
        self.move_speed = config.getfloat('move_speed', 25.0, above=0.0)
        self.accel_per_hz = config.getfloat('accel_per_hz', 100.0, above=0.0)
        self.min_accel = config.getfloat('min_accel', 1000.0, above=0.0)
        self.probe_time = config.getfloat('probe_time', 2.0, above=0.0)
        self.meas_offset = config.getfloat('meas_offset', 0.4
                , above=0.0, below=self.probe_time)
        self.meas_time = config.getfloat('meas_time', 1.5
                , above=0.0, below=self.probe_time-self.meas_offset)
        self.flush_time = config.getfloat('flush_time', 0.5, minval=0.0)
        self.pause_between_probes = config.getfloat('pause_between_probes', 2.0, above=0.1)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("MEASURE_AXES_NOISE", self.cmd_MEASURE_AXES_NOISE)
        self.gcode.register_command("MEASURE_ACCEL", self.cmd_MEASURE_ACCEL)
        self.gcode.register_command("TEST_FREQ", self.cmd_TEST_FREQ)
        self.gcode.register_command("TEST_VIBRATIONS", self.cmd_TEST_VIBRATIONS)
        self.gcode.register_command("TEST_RESONANCES", self.cmd_TEST_RESONANCES)

    def cmd_TEST_VIBRATIONS(self, gcmd):
        gcodestatus = self.gcode.get_status(None)
        if not gcodestatus['absolute_coordinates']:
            raise self.gcode.error(
                    "TEST_VIBRATIONS does not support relative move mode")
        X = gcmd.get_float("X")
        Y = gcmd.get_float("Y")
        Z = gcmd.get_float("Z")

        # Parse parameters
        axis = gcmd.get("AXIS", None)
        if axis is not None:
            if axis.upper() == "X":
                vX, vY = 1., 0.
            elif axis.upper() == "Y":
                vX, vY = 0., 1.
            else:
                raise gcmd.error("AXIS should only be X or Y, got '%s'", axis)
        else:
            vX = gcmd.get_float("VIB_X")
            vY = gcmd.get_float("VIB_Y")

        freq_start = gcmd.get_float("FREQ_START", minval=MIN_TEST_FREQ)
        freq_end = gcmd.get_float("FREQ_END", minval=freq_start)
        freq_step = gcmd.get_float("FREQ_STEP")
        vel = gcmd.get_float("MOVE_SPEED", self.move_speed)
        min_accel = gcmd.get_float("MIN_ACCEL", self.min_accel)
        probe_time = gcmd.get_float("PROBE_TIME", self.probe_time)
        meas_time = gcmd.get_float("MEAS_TIME", self.meas_time)
        meas_offset = gcmd.get_float("MEAS_OFFSET", self.meas_offset)

        output = gcmd.get("OUTPUT", None)
        raw_output_fmt = gcmd.get("RAW_OUTPUT_FMT", None)

        csvfile = None
        if output is not None:
            try:
                csvfile = open(output, "wb")
                csvfile.write("freq,x,y,z\n")
            except IOError as e:
                raise gcmd.error("Error writing to file '%s': %s", output,
                                 str(e))
        gcmd.respond_info("Axes vibrations: freq, x, y, z")

        toolhead = self.printer.lookup_object('toolhead')
        reactor = self.printer.get_reactor()
        idle_timeout = self.printer.lookup_object('idle_timeout')

        vib_dir = self._normalize([vX, vY])
        sX = X - vib_dir[1] * vel * probe_time * .5
        sY = Y - vib_dir[0] * vel * probe_time * .5
        g1_gcmd = self.gcode.create_gcode_command("G1", "G1"
                , {'X': sX, 'Y': sY, 'Z': Z, 'F': self.move_speed * 60.})
        self.gcode.cmd_G1(g1_gcmd)
        toolhead.get_last_move_time()

        freqs = float_range(freq_start, freq_end, freq_step)
        def test_single_freq(eventtime):
            if idle_timeout.state == "Printing":
                return eventtime + .5
            try:
                freq = next(freqs)
            except StopIteration:
                if csvfile is not None:
                    csvfile.close()
                return reactor.NEVER
            accel = min(max(min_accel, self.accel_per_hz * freq),
                        toolhead.max_accel_to_decel)
            raw_output = (None if raw_output_fmt is None
                          else raw_output_fmt % (freq,))
            vx, vy, vz = self._run_test(toolhead, [sX, sY], vib_dir, vel, accel
                    , freq, probe_time, meas_time, meas_offset, raw_output)
            gcmd.respond_info("%.3f, %.6f, %.6f, %.6f" % (freq, vx, vy, vz))
            if csvfile is not None:
                csvfile.write("%.3f,%.6f,%.6f,%.6f\n" % (freq, vx, vy, vz))
                csvfile.flush()
            return reactor.monotonic() + self.pause_between_probes
        reactor.register_timer(test_single_freq, reactor.NOW)

    def cmd_TEST_FREQ(self, gcmd):
        gcodestatus = self.gcode.get_status(None)
        if not gcodestatus['absolute_coordinates']:
            raise self.gcode.error(
                    "TEST_FREQ does not support relative move mode")
        currentPos = gcodestatus['gcode_position']
        X = currentPos[0]
        Y = currentPos[1]

        # Parse parameters
        axis = gcmd.get("AXIS", None)
        if axis is not None:
            if axis.upper() == "X":
                vX, vY = 1., 0.
            elif axis.upper() == "Y":
                vX, vY = 0., 1.
            else:
                raise gcmd.error("AXIS should only be X or Y, got '%s'", axis)
        else:
            vX = gcmd.get_float("VIB_X")
            vY = gcmd.get_float("VIB_Y")

        toolhead = self.printer.lookup_object('toolhead')

        freq = gcmd.get_float("FREQ", minval=MIN_TEST_FREQ)
        accel = gcmd.get_float("ACCEL", min(self.accel_per_hz * freq,
                                            toolhead.max_accel_to_decel))
        vel = gcmd.get_float("MOVE_SPEED", self.move_speed)
        probe_time = gcmd.get_float("PROBE_TIME", self.probe_time)
        meas_time = gcmd.get_float("MEAS_TIME", self.meas_time)
        meas_offset = gcmd.get_float("MEAS_OFFSET", self.meas_offset)
        raw_output = gcmd.get("RAW_OUTPUT", None)

        vx, vy, vz = self._run_test(toolhead, [X, Y], [vX, vY], vel, accel, freq
                , probe_time, meas_time, meas_offset, raw_output)
        gcmd.respond_info("Axes vibrations: %.6f (x), %.6f (y), %.6f (z)"
                          % (vx, vy, vz))

    def cmd_TEST_RESONANCES(self, gcmd):
        gcodestatus = self.gcode.get_status(None)
        if not gcodestatus['absolute_coordinates']:
            raise self.gcode.error(
                    "TEST_FREQ does not support relative move mode")
        currentPos = gcodestatus['gcode_position']
        X = currentPos[0]
        Y = currentPos[1]

        # Parse parameters
        vX = gcmd.get_float("VIB_X", None)
        vY = gcmd.get_float("VIB_Y", None)
        if vX is None and vY is None:
            axis = gcmd.get("AXIS")
            if axis.upper() == "X":
                vX, vY = 1., 0.
            elif axis.upper() == "Y":
                vX, vY = 0., 1.
            else:
                raise gcmd.error("AXIS should only be X or Y, got '%s'", axis)
        else:
            vX = vX or 0.
            vY = vY or 0.

        toolhead = self.printer.lookup_object('toolhead')

        freq_start = gcmd.get_float("FREQ_START", 10.)
        freq_end = gcmd.get_float("FREQ_END", 100.)
        freq_step = gcmd.get_float("FREQ_STEP", 1.)
        min_accel = gcmd.get_float("MIN_ACCEL", self.min_accel)
        raw_output = gcmd.get("RAW_OUTPUT")

        vib_dir = self._normalize([vX, vY])

        # Approximate (over-)estimate of the total test time
        test_time = sum([2.5 / freq for freq in
                         float_range(freq_start, freq_end, freq_step)])
        # Generate moves
        def exec_test():
            sign = 1.
            flush_time = 0.
            for freq in float_range(freq_start, freq_end, freq_step):
                t = 1. / freq
                accel = min(max(min_accel, self.accel_per_hz * freq),
                            toolhead.requested_accel_to_decel)
                V = accel * t
                toolhead.cmd_M204(self.gcode.create_gcode_command(
                    "M204", "M204", {"S": accel}))
                L = .5 * accel * t**2
                nX = X + sign * vib_dir[0] * L
                nY = Y + sign * vib_dir[1] * L
                F = V * 60.
                self.gcode.cmd_G1(self.gcode.create_gcode_command(
                    "G1", "G1", {'X': nX, 'Y': nY, 'F': F}))
                flush_time += t
                if flush_time >= toolhead.buffer_time_high:
                    toolhead.get_last_move_time()
                    flush_time = 0.
                self.gcode.cmd_G1(self.gcode.create_gcode_command(
                    "G1", "G1", {'X': X, 'Y': Y, 'F': F}))
                sign = -sign
                flush_time += t
            toolhead.get_last_move_time()
        raw_values = self._measure_accel(test_time + self.flush_time, exec_test)
        if raw_output is not None:
            self._save_raw_values(raw_values, raw_output)

    def cmd_MEASURE_ACCEL(self, gcmd):
        output = gcmd.get("RAW_OUTPUT")
        T = gcmd.get_float("T", self.meas_time)
        values = self._measure_accel(T)
        self._save_raw_values(values, output)

    def cmd_MEASURE_AXES_NOISE(self, gcmd):
        meas_time = gcmd.get_float("MEAS_TIME", self.meas_time)

        toolhead = self.printer.lookup_object('toolhead')
        vx, vy, vz = self._measure_zero_vibrations(self.min_accel, meas_time)
        gcmd.respond_info("Axes noise: %.6f (x), %.6f (y), %.6f (z)"
                          % (vx, vy, vz))

    def _run_test(self, toolhead, start, vib_dir, vel, accel, freq, probe_time,
                  meas_time, meas_offset, raw_output=None):
        # Build list of linear coordinates to move to
        moves = self._plan_test(start, vib_dir, vel, accel, freq, probe_time)

        # Convert into G1 commands
        for move in moves:
            g1_gcmd = self.gcode.create_gcode_command("G1", "G1", move)
            self.gcode.cmd_G1(g1_gcmd)

        raw_values = self._measure_accel(probe_time + self.flush_time
                , toolhead.get_last_move_time)

        g1_gcmd = self.gcode.create_gcode_command("G1", "G1"
                , {'X': start[0], 'Y': start[1], 'F': self.move_speed * 60.})
        self.gcode.cmd_G1(g1_gcmd)
        toolhead.get_last_move_time()

        if raw_output is not None:
            self._save_raw_values(raw_values, raw_output)
        return self._calc_vibrations(raw_values, accel, freq,
                                     meas_time, meas_offset)

    def _normalize(self, vec):
        inv_D = 1. / math.sqrt(sum([x*x for x in vec]))
        return [x * inv_D for x in vec]

    def _plan_test(self, start, vib_dir, vel, accel, freq, T):
        t_seg = self.seg_sec
        omega = 2 * math.pi * freq
        A = accel / omega**2 if omega > 1e-9 else 0.
        # Normalize vibration vector
        vib_dir = self._normalize(vib_dir)
        # Orthogonal to vib_dir
        m_dir = (vib_dir[1], vib_dir[0])

        # Generate moves
        moves = []
        i = 0
        pX, pY = start[0], start[1]
        while True:
            i += 1
            t = t_seg * i
            if t >= T:
                break
            m_pos = vel * t
            vib_pos = A * math.sin(omega * t)
            X = start[0] + m_dir[0] * m_pos + vib_dir[0] * vib_pos
            Y = start[1] + m_dir[1] * m_pos + vib_dir[1] * vib_pos
            V = math.sqrt((X-pX)**2 + (Y-pY)**2) / t_seg
            F = V * 60.
            moves.append({'X': X, 'Y': Y, 'F': F})
            pX, pY = X, Y
        return moves

    def _measure_zero_vibrations(self, min_accel, meas_time):
        raw_values = self._measure_accel(meas_time)
        ax, ay, az = self._integrate_squared(
                raw_values, self._integrate(raw_values, 0, raw_values.n-1),
                0, raw_values.n-1)
        a2 = min_accel * min_accel
        return 2. * ax / a2, 2. * ay / a2, 2. * az / a2

    def _measure_accel(self, T, cb_run_during_measuring=None):
        parent_conn, child_conn = multiprocessing.Pipe()
        p = multiprocessing.Process(target=measure_accel
                , args=(child_conn, T))
        p.start()

        if cb_run_during_measuring is not None:
            cb_run_during_measuring()

        raw_values = parent_conn.recv()
        p.join()
        if raw_values is None:
            raise self.gcode.error(
                    "Error reading acceleration values from ADXL345")
        return raw_values

    def _calc_vibrations(self, raw_values, accel, freq, time, offset):
        # Take the largest number of full periods
        time = math.floor(time * freq) / freq
        # Find start of acceleration
        a2 = accel * accel
        SLICE = 20
        # Beginning of raw_values should have non-moving toolhead
        avg_slice0 = self._integrate(raw_values, 0, SLICE)
        for i in range(SLICE, raw_values.n):
            ax, ay, az = self._integrate_squared(raw_values, avg_slice0,
                                                 i-SLICE, i)
            # A heuristic to find the beginning of the move
            if ax + ay + az > .125 * a2:
                break
        if i >= raw_values.n-1:
            raise self.gcode.error("No vibrations measured, wrong timing?")
        # Skip 'offset' time from the beginning of the toolhead move
        for j in range(i+1, raw_values.n):
            if raw_values.t[j] - raw_values.t[i] >= offset:
                break
        i = j
        # Find 'time' interval
        for j in range(i+1, raw_values.n):
            if raw_values.t[j] - raw_values.t[i] >= time:
                break
        logging.info("Processing vibrations within [%.6f, %.6f]",
                     raw_values.t[i], raw_values.t[j])
        ax, ay, az = self._integrate_squared(raw_values,
                                             self._integrate(raw_values, i, j),
                                             i, j)
        return 2. * ax / a2, 2. * ay / a2, 2. * az / a2

    # Integrate amplitudes from start to end, inclusive
    def _integrate(self, raw_values, start, end):
        T = raw_values.t[end] - raw_values.t[start]
        def integrate(a):
            res = 0.
            for i in range(start, end):
                dt = raw_values.t[i+1] - raw_values.t[i]
                res += .5 * (a[i] + a[i+1]) * dt
            return res / T
        avg_ax = integrate(raw_values.ax)
        avg_ay = integrate(raw_values.ay)
        avg_az = integrate(raw_values.az)
        return avg_ax, avg_ay, avg_az

    # Integrate squared amplitudes from start to end, inclusive
    def _integrate_squared(self, raw_values, avg_values, start, end):
        T = raw_values.t[end] - raw_values.t[start]
        def integrate_squared(a, avg):
            res = 0.
            for i in range(start, end):
                dt = raw_values.t[i+1] - raw_values.t[i]
                res += .5 * ((a[i]-avg)**2 + (a[i+1]-avg)**2) * dt
            return res / T
        sqr_ax = integrate_squared(raw_values.ax, avg_values[0])
        sqr_ay = integrate_squared(raw_values.ay, avg_values[1])
        sqr_az = integrate_squared(raw_values.az, avg_values[2])
        return sqr_ax, sqr_ay, sqr_az

    def _save_raw_values(self, values, output):
        try:
            with open(output, "wb") as csvfile:
                csvfile.write("t,ax,ay,az\n")
                for i in range(values.n):
                    csvfile.write("%.6f,%.1f,%.1f,%.1f\n" % (values.t[i]
                        , values.ax[i], values.ay[i], values.az[i]))
        except IOError as e:
            raise self.gcode.error("Error writing to file '%s': %s", output,
                                   str(e))


def load_config(config):
    return ResonanceTester(config)
