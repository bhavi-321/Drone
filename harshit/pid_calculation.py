#!/usr/bin/env python3
"""
auto_pid_physical.py
Calculates PID gains based on drone physical properties and desired response.
"""

import math

m = float(input("Enter drone mass (kg): "))
F_max = float(input("Enter max thrust per axis (N): "))
t_r = float(input("Enter desired rise time (s): "))
zeta = float(input("Enter damping ratio (0.5-1 recommended): "))
t_i = float(input("Enter integral time constant (s, e.g., 3-10 * t_r): "))

# Natural frequency for desired rise time
omega_n = math.pi / (t_r * math.sqrt(1 - zeta**2))

# PID gains
Kp = m * omega_n**2
Kd = 2 * m * zeta * omega_n
Ki = Kp / t_i

print("\n PID Gains based on Drone Physical Properties")
print(f"Mass (m) = {m} kg")
print(f"Max thrust (F_max) = {F_max} N")
print(f"Desired rise time (t_r) = {t_r} s")
print(f"Damping ratio (zeta) = {zeta}")
print(f"Integral time constant (t_i) = {t_i} s\n")

print(f"Calculated PID gains:")
print(f"Kp = {Kp:.3f}")
print(f"Ki = {Ki:.3f}")
print(f"Kd = {Kd:.3f}")
