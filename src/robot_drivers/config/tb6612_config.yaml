# tb6612_config.yaml
# GPIO pin assignments and physical parameters for the TB6612FNG motor drivers.
# All GPIO numbers are BCM (Broadcom) numbering.

tb6612_motor_driver_node:
  ros__parameters:

    # Physical robot dimensions
    wheel_base_width: 0.29      # metres — lateral distance between left and right wheels
    wheel_radius:     0.045     # metres
    max_speed:        0.5       # m/s

    # ── Front TB6612FNG (drives FL + FR motors) ──────────────────────────
    front_pwm_a: 12   # PWM  → Front-Left  motor speed
    front_ain1:  23   # AIN1 → Front-Left  direction
    front_ain2:  24   # AIN2 → Front-Left  direction
    front_pwm_b: 13   # PWM  → Front-Right motor speed
    front_bin1:  20   # BIN1 → Front-Right direction
    front_bin2:  21   # BIN2 → Front-Right direction
    front_stby:  25   # STBY → enable chip (HIGH = enabled)

    # ── Rear TB6612FNG (drives RL + RR motors) ───────────────────────────
    rear_pwm_a:  18   # PWM  → Rear-Left  motor speed
    rear_ain1:    5   # AIN1 → Rear-Left  direction
    rear_ain2:    6   # AIN2 → Rear-Left  direction
    rear_pwm_b:  19   # PWM  → Rear-Right motor speed
    rear_bin1:   16   # BIN1 → Rear-Right direction
    rear_bin2:   26   # BIN2 → Rear-Right direction
    rear_stby:   22   # STBY → enable chip (HIGH = enabled)
