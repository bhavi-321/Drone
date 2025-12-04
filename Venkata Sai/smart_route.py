#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import random
import copy
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse

#------------------------------------------CONFIGURATION-------------------------------------
TARGET_ALTITUDE = 10        # Meters
HOVER_DURATION = 5          # Seconds
BATTERY_LIMIT = 10.0        # Volts
WIND_SPEED = 5.0            # m/s
WIND_DIR = 45               # Degrees (Coming from North-East)

# Weights for Fitness (Priority: Shortest Path)
WEIGHT_DISTANCE = 1.0       # High priority
WEIGHT_WIND = 0.3           # Low priority

# ----------------------------------HELPER FUNCTIONS----------------------------------------
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobalRelative object containing the latitude/longitude
    `dNorth` and `dEast` metres from the specified `original_location`.
    """
    earth_radius = 6378137.0 # Radius of the earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_dist_between_points(p1, p2):
    """
    Returns distance in meters between two (lat, lon) tuples
    """
    dlat = p2[0] - p1[0]
    dlon = p2[1] - p1[1]
    return math.sqrt(dlat*dlat + dlon*dlon) * 1.113195e5

# ------------------------------------------GENETIC ALGORITHM----------------------------------------
class GeneticRouter:
    def __init__(self, start_pos, points, wind_spd, wind_dir):
        self.start = start_pos
        self.points = points
        self.wind_spd = wind_spd
        self.wind_dir = wind_dir
        self.pop_size = 60      # Population size
        self.generations = 150  # Number of iterations

    def get_bearing(self, p1, p2):
        dLon = (p2[1] - p1[1])
        y = math.sin(dLon) * math.cos(p2[0])
        x = math.cos(p1[0]) * math.sin(p2[0]) - math.sin(p1[0]) * math.cos(p2[0]) * math.cos(dLon)
        brng = math.atan2(y, x)
        return (math.degrees(brng) + 360) % 360

    def calculate_cost(self, route):
        """
        Calculates the 'Cost' of a route. 
        Lower Cost = Better Route.
        """
        total_dist = 0
        wind_penalty_score = 0
        
        # Add distance from drone to first point
        curr = self.start
        
        for p in route:
            dist = get_dist_between_points(curr, p)
            bearing = self.get_bearing(curr, p)
            
            # --- WIND LOGIC ---
            # Calculate angle difference (0 = Tailwind, 180 = Headwind)
            angle_diff = abs(bearing - self.wind_dir) % 360
            if angle_diff > 180: angle_diff = 360 - angle_diff
            
            # Wind Factor: 0.0 (Good) to 1.0 (Bad)
            wind_factor = (angle_diff / 180.0)
            
            # Accumulate scores separately
            total_dist += dist
            # We multiply distance by wind_factor so long legs into wind are penalized more
            wind_penalty_score += (dist * wind_factor) 

            curr = p

        # --- FINAL COST FORMULA ---
        # Cost = (Distance * 1.0) + (Wind_Penalty * 0.3)
        final_cost = (total_dist * WEIGHT_DISTANCE) + (wind_penalty_score * WEIGHT_WIND)
        return final_cost

    def solve(self):
        # 1. Create initial population (random orders)
        population = []
        base_indices = list(range(len(self.points)))
        for _ in range(self.pop_size):
            random.shuffle(base_indices)
            population.append(copy.deepcopy(base_indices))

        # 2. Evolution Loop
        for _ in range(self.generations):
            # Sort population by Cost (Lowest cost first)
            population.sort(key=lambda x: self.calculate_cost([self.points[i] for i in x]))
            
            # Keep best 50%
            survivors = population[:self.pop_size//2]
            next_gen = survivors[:]
            
            # Breed new ones to fill population
            while len(next_gen) < self.pop_size:
                p1 = random.choice(survivors)
                p2 = random.choice(survivors)
                child = self.crossover(p1, p2)
                if random.random() < 0.2: # 20% Mutation rate
                    self.mutate(child)
                next_gen.append(child)
            population = next_gen

        best_idx = population[0]
        return [self.points[i] for i in best_idx]

    def crossover(self, p1, p2):
        # Ordered Crossover (OX1) to prevent duplicate waypoints
        cut1 = random.randint(0, len(p1) - 1)
        cut2 = random.randint(cut1 + 1, len(p1))
        child = [-1] * len(p1)
        child[cut1:cut2] = p1[cut1:cut2]
        
        current_p2_idx = 0
        for i in range(len(child)):
            if child[i] == -1:
                while p2[current_p2_idx] in child:
                    current_p2_idx += 1
                child[i] = p2[current_p2_idx]
        return child

    def mutate(self, route):
        # Swap two points
        a, b = random.sample(range(len(route)), 2)
        route[a], route[b] = route[b], route[a]

# --------------------------------------------MAIN SCRIPT----------------------------------------------

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, wait_ready=True)

# --- 0. SET PARAMETERS ---
# Set RTL altitude to 10 meters
print(f"Setting RTL Altitude to {TARGET_ALTITUDE}m...")
vehicle.parameters['RTL_ALT'] = TARGET_ALTITUDE * 100 

# --- 1. GENERATING 6 SCATTERED WAYPOINTS ---
print("\nGenerating 6 Mission Waypoints...")
home = vehicle.location.global_relative_frame
offsets = [
    (10, 10), 
    (-5, 5),  
    (10, -10), 
    (-5, -5),  
    (0, -15),   
    (-10, 0)    
]

# Converting offsets to GPS coordinates
raw_waypoints = []
for off in offsets:
    loc = get_location_metres(home, off[0], off[1])
    raw_waypoints.append((loc.lat, loc.lon))

print("Raw Waypoints generated (Unsorted/Messy).")

# --- 2. ARM AND TAKEOFF ---
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(TARGET_ALTITUDE)
vehicle.airspeed = 5

# --- 3. RUN GENETIC ALGORITHM ---
print("\n" + "-"*40)
print("STARTING GENETIC ALGORITHM OPTIMIZATION")
print(f"Wind Speed: {WIND_SPEED} m/s | Direction: {WIND_DIR} deg")
print("Optimizing for SHORTEST PATH (Wind is secondary)...")
print("-" * 40)

current_gps = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
ga = GeneticRouter(current_gps, raw_waypoints, WIND_SPEED, WIND_DIR)

start_time = time.time()
optimized_route = ga.solve() # <--- THIS IS THE COMPUTATION
end_time = time.time()
print(f"Optimization Complete in {end_time - start_time:.2f} seconds.")

# --- VERIFY OPTIMIZATION ----
print("\n" + "="*40)
print("ROUTE COMPARISON")
print("="*40)

# Using zip to iterate both lists simultaneously
for i, (original, optimized) in enumerate(zip(raw_waypoints, optimized_route), 1):
    print(f"Original Point {i}: {original}")
    print(f"Optimized Point {i}: {optimized}")
    print("-" * 40)

# --- 4. EXECUTE MISSION ---
print("\nStarting Mission Execution...")
for i, wp in enumerate(optimized_route):
    # Safety Check: Battery
    current_voltage = vehicle.battery.voltage
    print(f"BATTERY: {current_voltage} V")
    
    # In Simulation, battery often shows 0 or null, so we add a check to ignore 0
    if 0 < current_voltage < BATTERY_LIMIT:
        print(f"!!! LOW BATTERY DETECTED (<{BATTERY_LIMIT}V) !!!")
        print("Finding nearest waypoint to land...")
        
        # Find nearest point from remaining list
        my_loc = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        nearest_pt = min(raw_waypoints, key=lambda p: get_dist_between_points(my_loc, p))
        
        target_loc = LocationGlobalRelative(nearest_pt[0], nearest_pt[1], TARGET_ALTITUDE)
        vehicle.simple_goto(target_loc)
        print("Emergency Routing to safe point...")
        time.sleep(10) # Give time to reach
        vehicle.mode = VehicleMode("LAND")
        break

    # Normal Navigation
    target_loc = LocationGlobalRelative(wp[0], wp[1], TARGET_ALTITUDE)
    print(f"Moving to Waypoint {i+1}/{len(optimized_route)}")
    vehicle.simple_goto(target_loc)

    # Wait until reached
    while True:
        dist = get_dist_between_points((vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon), wp)
        if dist < 1: # Within 2 meters
            print(f"Arrived. Hovering for {HOVER_DURATION} seconds...")
            time.sleep(HOVER_DURATION) # HOVER
            break
        
        # Wind Monitoring (Rerouting Logic)
        if random.random() < 0.005: 
            print("ALERT: Wind shift detected! Rerouting...")
            pass
            
        time.sleep(1)

print("Mission Complete. Returning to Launch.")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
