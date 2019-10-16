int32 shutdownEvent
int32 result
int32 delay

#below constant define for coding

int32 ShutdownEvent = 1      #shutdownEvent same to /shutdown_cmd 
int32 RebootEvent = 2        
int32 SleepEvent = 3         # 1 2 3 from rcu to ccu
int32 PowerButtonPressedEvent = 5  # physical button to shutdown, from ccu to rcu

int32 AllScaLaunchEvent = 10
int32 AllScaHealthCheckEvent = 11
int32 AllSensorHealthCheckEvent = 12
int32 BrakeReleaseEvent = 13

int32 SitDownGesture = 20
int32 BrakeLockEvent = 21
int32 AllScaCloseEvent = 22
int32 SensorsSleepEvent = 23
int32 DelayAndShutdownEvent = 30

#result 0 is success, 1 is failed
int32 SUCCESS_RST = 0
int32 FAIL_RST = 1

