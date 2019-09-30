
# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 current_capacity # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 init_capacity    # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 dump_energy      # Charge percentage on 0 to 1 range
uint8   cell_type        # The charging status as reported. Values defined above
uint8   id               # The battery health metric. Values defined above
uint32  status           # The battery chemistry. Values defined above

