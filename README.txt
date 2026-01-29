#
MIL (Model-in-the-Loop) PID CRUISE CONTROLLER 
README
(c) 2026 Joe Bosco

SYSTEM REQUIREMENTS: 
	UNIX, POSIX required
	FreeRTOS 202406.04 LTS

HOW TO RUN
	Place the project folder within the Demo folder in your FreeRTOS install, like so:
	
		/FreeRTOS/Demo/PID_Controller

	Additional configuration may be required. 
	
INTERFACE MANUAL
	PID Parameters:
		Defaults: Kp = 10.0, Ki = 1.0, Kd = 0.1, Setpoint = 20.0
		Suggested ranges: Kp = [5.0, 100.0], Ki = [0.5, 10.0], Kd = [0.01, 1.0]

	Vehicle Parameters:
		Defaults: Mass = 1000.0, Drag = 0.5, Engine Force = 2000.0, Slope = 0.0
		Suggested ranges: Mass = [500.0, 10000.0], Drag = [0.0, 1.0], Engine Force = [1000.0, 10000.0], Slope = [-10.0, 10.0]

	Stress Testing:
		Recommended Test Rate: 10Hz
		Breaks around 100Hz due to small timeouts included in the interface for stability.
		Duration doesn't matter since the control task always preempts the server. 

TASK STATISTICS
	WCET: 288us (2.88% usage)
	Jitter: +- 1 OS tick (on older Intel Unix machine, =1ns. On an M1 Silicon machine, may vary, but in a similar range, which is negligible for this application.)

NOTES:
Why use atomic shared memory rather than FreeRTOS queues?
Since only the most recent update matters, and updates are infrequent compared to the control loop frequency, it's faster to use atomic memory swap. Queues may be better for multiple updates coming in at once since they can preserve message order, but they require context switching from the scheduler which uses more overhead than DMA. 