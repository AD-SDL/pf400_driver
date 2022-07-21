' Robot kinematic module parameter data file
' Controller: Default Configuration
' System software: GCU
' File creation date: 

Begin: 1.0

DI: 2000, 0, 0, 0, "Number of axes" = 1
DI: 2001, 0, 0, 0, "Standard split X-axis" = 0
DI: 2002, 0, 0, 0, "Robot name" = "Encoder Only Module"
DI: 2003, 0, 0, 0, "Axis mask" = 0
DI: 2004, 0, 0, 0, "Number of extra, non-servoed, axes" = 0
DI: 2010, 0, 0, 0, "Motor map nodes" = 1

DI: 2104, 0, 0, 0, "Enable robot operation" = 1
DI: 2105, 0, 0, 0, "Simulate servo interface" = 0
DI: 2107, 0, 0, 0, "Enable servo command trace" = 0

DI: 2300, 0, 0, 0, "Joint to motor scale factors" = 5.6888889, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
DI: 2302, 0, 0, 0, "Joint roll over value in deg" = 0
DI: 2303, 0, 0, 0, "Unidirectional roll over" = 0

DI: 10008, 0, 0, 0, "Servo board configuration code" = -1
DI: 10020, 0, 0, 0, "Axis capability bitmask" = 1, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0
DI: 10021, 0, 0, 0, "Position loop update rate, second" = 0.001,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
DI: 10025, 0, 0, 0, "Minor axis number (dual-loop)" = 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0
DI: 10026, 0, 0, 0, "Encoder hardware channel number" = 10, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0
DI: 10027, 0, 0, 0, "Encoder type" = 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0
DI: 10028, 0, 0, 0, "Amplifier hardware channel number" = 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0
DI: 10029, 0, 0, 0, "Amplifier type" = 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0
DI: 10030, 0, 0, 0, "Motor commutation method" = -1, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0
DI: 10049, 0, 0, 0, "Robot initialized" = 1

DI: 10104, 0, 0, 0, "Axis configuration word" = 0
DI: 10108, 0, 0, 0, "Dedicated DIN selection" = 0

DI: 10200, 0, 0, 0, "Encoder software configuration" = 0
DI: 10202, 0, 0, 0, "Encoder sign, +/- 1" = 1
DI: 10203, 0, 0, 0, "Encoder counts for resolution calc, ecnt" = 2048
DI: 10204, 0, 0, 0, "Encoder revs for resolution calc, rev" = 1
DI: 10206, 0, 0, 0, "Encoder to motor scale factor, mcnt/ecnt" = 1
DI: 10207, 0, 0, 0, "Motor velocity SPR filter pole, Hz" = 100
DI: 10208, 0, 0, 0, "Run-time speed limit, mcnt/sec" = 800000
DI: 10209, 0, 0, 0, "Manual mode speed limit, mcnt/sec" = 800000
DI: 10210, 0, 0, 0, "Power sequence speed limit, mcnt/sec" = 800000
DI: 10220, 0, 0, 0, "Incremental encoder hardware configuration" = 149
DI: 10221, 0, 0, 0, "Index skew count limit, mcnt" = 15
DI: 10222, 0, 0, 0, "Index noise spikes limit" = 4

DI: 10300, 0, 0, 0, "Feedback compensator configuration" = 0
DI: 10322, 0, 0, 0, "Velocity error type selection" = 0
DI: 10326, 0, 0, 0, "Velocity error SPR filter pole, Hz" = 100

DI: 10601, 0, 0, 0, "Motor type per CANopen spec" = 10
DI: 10606, 0, 0, 0, "Error block time configuration" = 7
DI: 10607, 0, 0, 0, "Brake pulse time configuration" = 0

End:

' End of file
