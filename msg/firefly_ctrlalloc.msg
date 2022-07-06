uint64 timestamp				# time since system start (microseconds)
uint32 status					# operating flags for control alloc
uint32 noutputs					# valid outputs
float32[8] controls				# output data, in natural output units
float32[8] output				# output data, in natural output units
float32[8] pwm_limited				# output data, in natural output units
float32[8] delta				# difference in throttle between output
