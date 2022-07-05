' Robot kinematic module parameter data file
' Controller: Default Configuration
' System software: GCU
' File creation date: 

Begin: 1.0

DI: 2000, 0, 0, 0, "Number of axes" = 4
DI: 2001, 0, 0, 0, "Standard split X-axis" = 0
DI: 2002, 0, 0, 0, "Robot name" = "4-Axis Motion Device"
DI: 2003, 0, 0, 0, "Axis mask" = 15
DI: 2004, 0, 0, 0, "Number of extra, non-servoed, axes" = 0
DI: 2005, 0, 0, 0, "Motor linearity compensation" = 0
DI: 2006, 0, 0, 0, "Robot type special option flags" = 0
DI: 2009, 0, 0, 0, "GCU Info" = "", "", "", ""
DI: 2010, 0, 0, 0, "Motor map nodes" = 1, 1, 1, 1
DI: 2012, 0, 0, 0, "Motor name" = "","","",""

DI: 2101, 0, 0, 0, "RapidDecel deceleration in %" = 100
DI: 2102, 0, 0, 0, "Auto execute master/slave gear ratio" = 0
DI: 2103, 0, 0, 0, "Auto execute master/slave ramp time in sec" = 0
DI: 2104, 0, 0, 0, "Enable robot operation" = 1
DI: 2105, 0, 0, 0, "Simulate servo interface" = 0
DI: 2107, 0, 0, 0, "Enable servo command trace" = 0

DI: 2200, 0, 0, 0, "Gripper open DOUT" = 13
DI: 2201, 0, 0, 0, "Gripper close DOUT" = 14
DI: 2202, 0, 0, 0, "Velocity control AIN" = 0

DI: 2300, 0, 0, 0, "Joint to motor scale factors" = 22.222222, 22.222222, 22.222222, 22.222222
DI: 2302, 0, 0, 0, "Joint roll over value in deg" = 0, 0, 0, 0
DI: 2303, 0, 0, 0, "Unidirectional roll over" = 0, 0, 0, 0
DI: 2304, 0, 0, 0, "Vel Ctl inrange tolerance in deg/sec" = 0, 0, 0, 0
DI: 2305, 0, 0, 0, "Pos Ctl inrange tolerance in ecnts" = 6, 6, 6, 6

DI: 2600, 0, 0, 0, "Nulling time out in sec" = 3
DI: 2601, 0, 0, 0, "Vel Ctl inrange time in sec" = 0.05

DI: 2700, 0, 0, 0, "100% joint speeds in (deg or mm)/sec" = 9000, 9000, 9000, 9000
DI: 2701, 0, 0, 0, "100% Cartesian speeds in (deg or mm)/sec" = 9000,23040
DI: 2702, 0, 0, 0, "100% joint accels in (deg or mm)/sec^2" = 24000, 24000, 24000, 24000
DI: 2703, 0, 0, 0, "100% Cartesian accels in (deg or mm)/sec^2" = 24000, 256000

DI: 2704, 0, 0, 0, "Max %speed allowed" = 200
DI: 2705, 0, 0, 0, "Max %accel allowed" = 100
DI: 2706, 0, 0, 0, "Max %decel allowed" = 100
DI: 2707, 0, 0, 0, "Couple %accel/%decel to %speed" = 200
DI: 2708, 0, 0, 0, "Default %speed" = 100
DI: 2709, 0, 0, 0, "Default Cartesian rotation %speed" = 0
DI: 2710, 0, 0, 0, "Default %accel" = 100
DI: 2711, 0, 0, 0, "Default %decel" = 100
DI: 2712, 0, 0, 0, "Default max accel ramp time in sec" = 0.1
DI: 2713, 0, 0, 0, "Default max decel ramp time in sec" = 0.1

DI: 2801, 0, 0, 0, "Robot homed DOUT" = 0
DI: 2802, 0, 0, 0, "Homing sequence" = 1
DI: 2803, 0, 0, 0, "Homing method" = 35, 35, 35, 35
DI: 2804, 0, 0, 0, "Homing speed 1, ecnts/sec" = 100, 100, 100, 100
DI: 2805, 0, 0, 0, "Homing speed 2, ecnts/sec" = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
DI: 2807, 0, 0, 0, "Motor homing accel, ecnts/sec^2" = 392, 392, 392, 392
DI: 2809, 0, 0, 0, "Commutate during homing" = 0
DI: 2810, 0, 0, 0, "Homing order" = 0, 0, 0, 0
DI: 2820, 0, 0, 0, "Max joint jog speed as %" = 5
DI: 2821, 0, 0, 0, "Max world/tool jog speed as %" = 5
DI: 2822, 0, 0, 0, "Rate of change of jog speed in %/sec" = 400
DI: 2823, 0, 0, 0, "Free mode inhibited axes mask" = 0
DI: 2824, 0, 0, 0, "Free mode gravity comp motor mask" = 0
DI: 2825, 0, 0, 0, "Jog tick joint step as %" = 0.001
DI: 2826, 0, 0, 0, "Jog tick world/tool step as %" = 0.001

DI: 3524, 0, 0, 0, "In-range DOUT" = 0

DI: 10008, 0, 0, 0, "Servo board configuration code" = -1
DI: 10012, 0, 0, 0, "Ignore amplifier type from hardware" = 0
DI: 10020, 0, 0, 0, "Axis capability bitmask" = 15, 15, 15, 15
DI: 10021, 0, 0, 0, "Position loop update rate, second" = 0.0005, 0.0005, 0.0005, 0.0005
DI: 10025, 0, 0, 0, "Minor axis number (dual-loop)" = 0, 0, 0, 0
DI: 10026, 0, 0, 0, "Encoder hardware channel number" = 1, 2, 3, 4
DI: 10027, 0, 0, 0, "Encoder type" = 0, 0, 0, 0
DI: 10028, 0, 0, 0, "Amplifier hardware channel number" = 1, 2, 3, 4
DI: 10029, 0, 0, 0, "Amplifier type" = 2, 2, 2, 2
DI: 10030, 0, 0, 0, "Motor commutation method" = 1, 1, 1, 1
DI: 10049, 0, 0, 0, "Robot initialized" = 1

DI: 10104, 0, 0, 0, "Axis configuration word" = 0, 0, 0, 0
DI: 10105, 0, 0, 0, "Home switch DIN signal" = 0, 0, 0, 0
DI: 10106, 0, 0, 0, "Positive overtravel DIN signal" = 0, 0, 0, 0
DI: 10107, 0, 0, 0, "Negative overtravel DIN signal" = 0, 0, 0, 0
DI: 10108, 0, 0, 0, "Dedicated DIN selection" = 0, 0, 0, 0
DI: 10122, 0, 0, 0, "Hardstop envelope limit, mcnt" = 500, 500, 500, 500
DI: 10123, 0, 0, 0, "Hardstop pos steady tolerance, mcnt" = 0, 0, 0, 0
DI: 10124, 0, 0, 0, "Hardstop back out distance, mcnt" = 1000, 1000, 1000, 1000

DI: 10200, 0, 0, 0, "Encoder software configuration" = 0, 0, 0, 0
DI: 10202, 0, 0, 0, "Encoder sign, +/- 1" = 1, 1, 1, 1
DI: 10203, 0, 0, 0, "Encoder counts for resolution calc, ecnt" = 8000, 8000, 8000, 8000
DI: 10204, 0, 0, 0, "Encoder revs for resolution calc, rev" = 1, 1, 1, 1
DI: 10206, 0, 0, 0, "Encoder to motor scale factor, mcnt/ecnt" = 1, 1, 1, 1
DI: 10207, 0, 0, 0, "Motor velocity SPR filter pole, Hz" = 100, 100, 100, 100
DI: 10208, 0, 0, 0, "Run-time speed limit, mcnt/sec" = 800000, 800000, 800000, 800000
DI: 10209, 0, 0, 0, "Manual mode speed limit, mcnt/sec" = 133333, 133333, 133333, 133333
DI: 10210, 0, 0, 0, "Power sequence speed limit, mcnt/sec" = 20000, 20000, 20000, 20000
DI: 10211, 0, 0, 0, "Enable dynamic slippage adjustment" = 0, 0, 0, 0
DI: 10212, 0, 0, 0, "Dual loop position slippage limit, mcnt" = 500, 500, 500, 500
DI: 10213, 0, 0, 0, "Minor to major encoder scale factor" = 1, 1, 1, 1
DI: 10252, 0, 0, 0, "Min. accel time to 5000 RPM, msec" = 2, 2, 2, 2

DI: 10300, 0, 0, 0, "Feedback compensator configuration" = 48, 48, 48, 48
DI: 10302, 0, 0, 0, "Soft envelope error limit, mcnt" = 1000, 1000, 1000, 1000
DI: 10303, 0, 0, 0, "Hard envelope error limit, mcnt" = 5000, 5000, 5000, 5000
DI: 10304, 0, 0, 0, "Envelope error check duration, sec" = 0.004, 0.004, 0.004, 0.004
DI: 10305, 0, 0, 0, "Velocity feedback blending factor" = 1, 1, 1, 1
DI: 10320, 0, 0, 0, "Enable clearing of integrator state flag" = 1, 1, 1, 1
DI: 10321, 0, 0, 0, "Feedforward type selection" = 0, 0, 0, 0
DI: 10322, 0, 0, 0, "Velocity error type selection" = 0, 0, 0, 0
DI: 10324, 0, 0, 0, "Position in-tolerance duration, sec" = 0.008, 0.008, 0.008, 0.008
DI: 10325, 0, 0, 0, "Position in-tolerance prediction window, sec" = 0.004, 0.004, 0.004, 0.004
DI: 10326, 0, 0, 0, "Velocity error SPR filter pole, Hz" = 100, 100, 100, 100
DI: 10327, 0, 0, 0, "Accel error DPR filter pole, Hz" = 0, 0, 0, 0
DI: 10328, 0, 0, 0, "Feedforward SPR filter pole, Hz" = 100, 100, 100, 100
DI: 10329, 0, 0, 0, "Torque output filter pole, Hz" = 0, 0, 0, 0
DI: 10330, 0, 0, 0, "Torque output filter damping ratio" = 0, 0, 0, 0
DI: 10331, 0, 0, 0, "Proportional gain, Kp" = 25.32, 25.32, 25.32, 25.32
DI: 10332, 0, 0, 0, "Integral gain, Ki" = 0.1266, 0.1266, 0.1266, 0.1266
DI: 10333, 0, 0, 0, "Derivative gain, Kd" = 253.2, 253.2, 253.2, 253.2
DI: 10334, 0, 0, 0, "Integrator limit" = 2000, 2000, 2000, 2000
DI: 10335, 0, 0, 0, "Integrator rate limit" = 20, 20, 20, 20
DI: 10336, 0, 0, 0, "Acceleration feedforward gain" = 0, 0, 0, 0
DI: 10337, 0, 0, 0, "Velocity feedforward gain" = 0, 0, 0, 0
DI: 10338, 0, 0, 0, "Accel error gain, Ka" = 0, 0, 0, 0

DI: 10420, 0, 0, 0, "Trajectory interpolation type selection" = 0, 0, 0, 0
DI: 10423, 0, 0, 0, "Servo tick delay after a trajectory setpoint" = 0, 0, 0, 0
DI: 10440, 0, 0, 0, "ADC input torque command enable" = 0, 0, 0, 0

DI: 10601, 0, 0, 0, "Motor type per CANopen spec" = 10, 10, 10, 10
DI: 10606, 0, 0, 0, "Error block time configuration" = 4, 4, 4, 4
DI: 10607, 0, 0, 0, "Brake pulse time configuration" = 0, 0, 0, 0
DI: 10609, 0, 0, 0, "Torque sign, +/- 1" = 1, 1, 1, 1
DI: 10611, 0, 0, 0, "Rated motor current, amp" = 1, 1, 1, 1
DI: 10613, 0, 0, 0, "AUTO mode max motor current % (per rated current)" = 100, 100, 100, 100, 100
DI: 10615, 0, 0, 0, "MANUAL mode max motor current % (per rated current)" = 50, 50, 50, 50
DI: 10617, 0, 0, 0, "Motor stalled check duration, sec" = 2, 2, 2, 2
DI: 10621, 0, 0, 0, "Duty cycle SPR filter pole, Hz" = 0.1, 0.1, 0.1, 0.1
DI: 10622, 0, 0, 0, "Duty cycle exceeded duration, sec" = 2, 2, 2, 2
DI: 10623, 0, 0, 0, "Duty cycle limit in terms of rated torque, %" = 200, 200, 200, 200
DI: 10625, 0, 0, 0, "Auxiliary brake release DOUT signal" = 0, 0, 0, 0
DI: 10650, 0, 0, 0, "Commutation sign, +/- 1" = 1, 1, 1, 1
DI: 10651, 0, 0, 0, "# of pole-pairs per motor revolution" = 2, 2, 2, 2
DI: 10652, 0, 0, 0, "Commutation counts per electrical cycle, mcnt" = 4000, 4000, 4000, 4000
DI: 10656, 0, 0, 0, "Current loop proportional gain, Kp" = 0.004882813, 0.004882813, 0.004882813, 0.004882813
DI: 10657, 0, 0, 0, "Current loop damping gain, Kv" = 0.004882812, 0.004882812, 0.004882812, 0.004882812
DI: 10658, 0, 0, 0, "Motor back-EMF constant" = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
DI: 10682, 0, 0, 0, "Current feedback sensor gain" = 1, 1, 1, 1
DI: 10683, 0, 0, 0, "Current offset of phase 1, ccnt" = 0, 0, 0, 0
DI: 10684, 0, 0, 0, "Current offset of phase 2, ccnt" = 0, 0, 0, 0

DI: 10700, 0, 0, 0, "Commutation reference setup config" = 5, 5, 5, 5
DI: 10704, 0, 0, 0, "PAF angle adjustment amount, degree" = 5, 5,5, 5
DI: 10705, 0, 0, 0, "PAF angle fine adjustment amount, degree" = 1.5, 1.5, 1.5, 1.5
DI: 10706, 0, 0, 0, "PAF duration time for twang ON, sec" = 0.05, 0.05, 0.05, 0.05
DI: 10707, 0, 0, 0, "PAF duration time for twang OFF, sec" = 0.02, 0.02, 0.02, 0.02
DI: 10708, 0, 0, 0, "PAF duration time for brake release, sec" = 1, 1, 1, 1
DI: 10709, 0, 0, 0, "PAF duration time for brake engage, sec" = 0.25, 0.25, 0.25, 0.25
DI: 10710, 0, 0, 0, "PAF twang range convergence factor, (0,1)" = 0.5, 0.5, 0.5, 0.5
DI: 10727, 0, 0, 0, "Twang-ref duration for twang ON, sec" = 1, 1, 1, 1
DI: 10728, 0, 0, 0, "Twang-ref duration for twang OFF, sec" = 1, 1, 1, 1
DI: 10729, 0, 0, 0, "Twang-ref delay time to release brake, sec" = 1, 1, 1, 1
DI: 10730, 0, 0, 0, "Twang-ref delay time to engage brake, sec" = 1, 1, 1, 1
DI: 10731, 0, 0, 0, "# of ramp steps to a twang" = 5, 5, 5, 5
DI: 10732, 0, 0, 0, "# of twang-ref checks" = 4, 4, 4, 4
DI: 10733, 0, 0, 0, "Twang-ref check duration time, sec" = 0.5, 0.5, 0.5, 0.5
DI: 10734, 0, 0, 0, "Twang-ref check position tolerance limit, %" = 8, 8, 8, 8
DI: 10735, 0, 0, 0, "Twang-ref torque %" = 100, 100, 100, 100
DI: 10750, 0, 0, 0, "Hall sensor step 1 nominal angle, degree" = -1, -1, -1, -1
DI: 10751, 0, 0, 0, "Hall sensor step 2 nominal angle, degree" = -1, -1, -1, -1
DI: 10752, 0, 0, 0, "Hall sensor step 3 nominal angle, degree" = -1, -1, -1, -1
DI: 10753, 0, 0, 0, "Hall sensor step 4 nominal angle, degree" = -1, -1, -1, -1
DI: 10754, 0, 0, 0, "Hall sensor step 5 nominal angle, degree" = -1, -1, -1, -1
DI: 10755, 0, 0, 0, "Hall sensor step 6 nominal angle, degree" = -1, -1, -1, -1
DI: 10760, 0, 0, 0, "Search ref torque limit %" = 100, 100, 100, 100
DI: 10761, 0, 0, 0, "Search ref torque step %" = 0.2, 0.2, 0.2, 0.2
DI: 10762, 0, 0, 0, "Min. angle move in binary search, degree" = 2.8, 2.8, 2.8, 2.8
DI: 10763, 0, 0, 0, "Axis load direction" = 0, 0, 0, 0
DI: 10764, 0, 0, 0, "Max. drop allowed for loaded axis, mcnt" = 1000, 1000, 1000, 1000
DI: 10765, 0, 0, 0, "Search ref enable refinement" = 0, 0, 0, 0
DI: 10766, 0, 0, 0, "Search ref step gain" = 0.5, 0.5, 0.5, 0.5
DI: 10767, 0, 0, 0, "Search ref damping gain" = 35, 35, 35, 35
DI: 10768, 0, 0, 0, "Search ref max angle step, degree" = 5, 5, 5, 5
DI: 10769, 0, 0, 0, "Search ref pre-load torque %" = 0, 0, 0, 0
DI: 10771, 0, 0, 0, "Search ref delay after current ON/OFF, sec" = 1, 1, 1, 1
DI: 10772, 0, 0, 0, "Search ref delay to release brake, sec" = 1, 1, 1, 1
DI: 10773, 0, 0, 0, "Search ref delay to engage brake, sec" = 1, 1, 1, 1
DI: 10774, 0, 0, 0, "Search ref check duration, sec" = 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
DI: 10775, 0, 0, 0, "Commutation offset, mcnt" =  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
DI: 10776, 0, 0, 0, "Search ref check angle, degree" = 14, 14, 14, 14
DI: 10777, 0, 0, 0, "Search ref check tolerance %" = 3.6, 3.6, 3.6, 3.6
DI: 10778, 0, 0, 0, "Search ref operation effort" = 35, 35, 35, 35
DI: 10779, 0, 0, 0, "Search ref auto adjust pre-load torque" = 1, 1, 1, 1
End:

' End of file
