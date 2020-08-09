# A utility class to test resonances of the printer
#
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

MIN_TEST_FREQ = 10.

def float_range(s, e, i):
    while s <= e + 1e-9:
        yield s
        s += i

class ResonanceTester:
    def __init__(self, config, accel_chip):
        self.printer = config.get_printer()
        self.accel_chip = accel_chip
        self.seg_sec = config.getfloat('seg_sec', 0.0005, above=0.0)
        self.move_speed = config.getfloat('move_speed', 25.0, above=0.0)
        self.accel_per_hz = config.getfloat('accel_per_hz', 100.0, above=0.0)
        self.min_accel = config.getfloat('min_accel', 1000.0, above=0.0)
        self.probe_time = config.getfloat('probe_time', 2.0, above=0.0)
        self.meas_offset = config.getfloat('meas_offset', 0.4
                , above=0.0, below=self.probe_time)
        self.meas_time = config.getfloat('meas_time', 1.5
                , above=0.0, below=self.probe_time-self.meas_offset)
        self.pause_between_probes = config.getfloat('pause_between_probes', 2.0,
                                                    above=0.1)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("MEASURE_AXES_NOISE",
                                    self.cmd_MEASURE_AXES_NOISE)
        self.gcode.register_command("MEASURE_ACCEL", self.cmd_MEASURE_ACCEL)
        self.gcode.register_command("TEST_FREQ", self.cmd_TEST_FREQ)
        self.gcode.register_command("TEST_VIBRATIONS", self.cmd_TEST_VIBRATIONS)
        self.gcode.register_command("TEST_RESONANCES", self.cmd_TEST_RESONANCES)

    def cmd_TEST_VIBRATIONS(self, gcmd):
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
        toolhead.manual_move([sX, sY, Z], self.move_speed)

        freqs = float_range(freq_start, freq_end, freq_step)
        for freq in freqs:
            toolhead.dwell(0.500)
            accel = min(max(min_accel, self.accel_per_hz * freq),
                        toolhead.max_accel_to_decel)
            raw_output = (None if raw_output_fmt is None
                          else raw_output_fmt % (freq,))
            try:
                vx, vy, vz = self._run_test(
                        toolhead, [sX, sY], vib_dir, vel, accel, freq,
                        probe_time, meas_time, meas_offset, raw_output)
            except self.gcode.error as e:
                gcmd.respond_info("Error measuring vibrations: %s" % (str(e),))
                if csvfile is not None:
                    csvfile.close()
                return
            gcmd.respond_info("%.3f, %.6f, %.6f, %.6f" % (freq, vx, vy, vz))
            if csvfile is not None:
                csvfile.write("%.3f,%.6f,%.6f,%.6f\n" % (freq, vx, vy, vz))
                csvfile.flush()
        if csvfile is not None:
            csvfile.close()

    def cmd_TEST_FREQ(self, gcmd):
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
        currentPos = toolhead.get_position()
        X = currentPos[0]
        Y = currentPos[1]

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
        currentPos = toolhead.get_position()
        X = currentPos[0]
        Y = currentPos[1]

        freq_start = gcmd.get_float("FREQ_START", 10.)
        freq_end = gcmd.get_float("FREQ_END", 100.)
        freq_step = gcmd.get_float("FREQ_STEP", 1.)
        min_accel = gcmd.get_float("MIN_ACCEL", self.min_accel)
        raw_output = gcmd.get("RAW_OUTPUT")

        vib_dir = self._normalize([vX, vY])

        # Generate moves
        self.accel_chip.start_measurements()
        if 1:
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
                toolhead.manual_move([nX, nY], V)
                flush_time += t
                # XXX - should not peak at toolhead member variables
                if flush_time >= toolhead.buffer_time_high:
                    toolhead.get_last_move_time()
                    flush_time = 0.
                toolhead.manual_move([X, Y], V)
                sign = -sign
                flush_time += t
        raw_values = self.accel_chip.finish_measurements()
        if raw_output is not None:
            self._save_raw_values(raw_values, raw_output)

    def cmd_MEASURE_ACCEL(self, gcmd):
        output = gcmd.get("RAW_OUTPUT")
        T = gcmd.get_float("T", self.meas_time)
        self.accel_chip.start_measurements()
        self.printer.lookup_object('toolhead').dwell(T)
        raw_values = self.accel_chip.finish_measurements()
        self._save_raw_values(raw_values, output)

    def cmd_MEASURE_AXES_NOISE(self, gcmd):
        meas_time = gcmd.get_float("MEAS_TIME", self.meas_time)
        self.accel_chip.start_measurements()
        self.printer.lookup_object('toolhead').dwell(meas_time)
        raw_values = self.accel_chip.finish_measurements()

        vx, vy, vz = self._measure_zero_vibrations(raw_values, self.min_accel)
        gcmd.respond_info("Axes noise: %.6f (x), %.6f (y), %.6f (z)"
                          % (vx, vy, vz))

    def _run_test(self, toolhead, start, vib_dir, vel, accel, freq, probe_time,
                  meas_time, meas_offset, raw_output=None):
        # Build list of linear coordinates to move to
        moves = self._plan_test(start, vib_dir, vel, accel, freq, probe_time)

        # Convert into G1 commands
        self.accel_chip.start_measurements()
        for move, speed in moves:
            toolhead.move(move, speed)
        raw_values = self.accel_chip.finish_measurements()

        toolhead.manual_move([start[0], start[1]], self.move_speed)

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
        Z, E = self.printer.lookup_object('toolhead').get_position()[2:]
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
            moves.append(([X, Y, Z, E], V))
            pX, pY = X, Y
        return moves

    def _measure_zero_vibrations(self, raw_values, min_accel):
        count = len(raw_values.get_samples())
        ax, ay, az = self._integrate_squared(
                raw_values, self._integrate(raw_values, 0, count-1),
                0, count-1)
        a2 = min_accel * min_accel
        return 2. * ax / a2, 2. * ay / a2, 2. * az / a2

    def _calc_vibrations(self, raw_values, accel, freq, time, offset):
        samples = raw_values.get_samples()
        # Take the largest number of full periods
        time = math.floor(time * freq) / freq
        # Find start of acceleration
        a2 = accel * accel
        SLICE = 20
        # Beginning of raw_values should have non-moving toolhead
        avg_slice0 = self._integrate(raw_values, 0, SLICE)
        for i in range(SLICE, len(samples)):
            ax, ay, az = self._integrate_squared(raw_values, avg_slice0,
                                                 i-SLICE, i)
            # A heuristic to find the beginning of the move
            if ax + ay + az > .125 * a2:
                break
        if i >= len(samples)-1:
            raise self.gcode.error("No vibrations measured, wrong timing?")
        # Skip 'offset' time from the beginning of the toolhead move
        for j in range(i+1, len(samples)):
            if samples[j].time - samples[i].time >= offset:
                break
        i = j
        # Find 'time' interval
        for j in range(i+1, len(samples)):
            if samples[j].time - samples[i].time >= time:
                break
        logging.info("Processing vibrations within [%.6f, %.6f]",
                     samples[i].time, samples[j].time)
        ax, ay, az = self._integrate_squared(raw_values,
                                             self._integrate(raw_values, i, j),
                                             i, j)
        return 2. * ax / a2, 2. * ay / a2, 2. * az / a2

    # Integrate amplitudes from start to end, inclusive
    def _integrate(self, raw_values, start, end):
        samples = raw_values.get_samples()
        T = samples[end].time - samples[start].time
        def integrate(spos):
            res = 0.
            for i in range(start, end):
                dt = samples[i+1].time - samples[i].time
                res += .5 * (samples[i][spos] + samples[i+1][spos]) * dt
            return res / T
        avg_ax = integrate(1)
        avg_ay = integrate(2)
        avg_az = integrate(3)
        return avg_ax, avg_ay, avg_az

    # Integrate squared amplitudes from start to end, inclusive
    def _integrate_squared(self, raw_values, avg_values, start, end):
        samples = raw_values.get_samples()
        T = samples[end].time - samples[start].time
        def integrate_squared(spos, avg):
            res = 0.
            for i in range(start, end):
                dt = samples[i+1].time - samples[i].time
                res += .5 * ((samples[i][spos]-avg)**2
                             + (samples[i+1][spos]-avg)**2) * dt
            return res / T
        sqr_ax = integrate_squared(1, avg_values[0])
        sqr_ay = integrate_squared(2, avg_values[1])
        sqr_az = integrate_squared(3, avg_values[2])
        return sqr_ax, sqr_ay, sqr_az

    def _save_raw_values(self, raw_values, output):
        samples = raw_values.get_samples()
        try:
            with open(output, "wb") as csvfile:
                csvfile.write("t,ax,ay,az\n")
                for i in range(len(samples)):
                    csvfile.write("%.6f,%.1f,%.1f,%.1f\n"
                                  % (samples[i].time, samples[i].accel_x,
                                     samples[i].accel_y, samples[i].accel_z))
        except IOError as e:
            raise self.gcode.error("Error writing to file '%s': %s", output,
                                   str(e))
