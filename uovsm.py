import time
import os
import math
import msvcrt

# ==============================
# VEHICLE PARAMETERS
# ==============================

MASS = 1850  
G = 9.81
C_RR = 0.015 
AIR_DENSITY = 1.225
CD = 0.38
FRONTAL_AREA = 2.8
DRIVETRAIN_EFF = 0.90 
ROAD_ANGLE_DEG = 0

# ==============================
# BRAKE & TIRES
# ==============================
BRAKE_FORCE_MAX = 8000 
TIRE_RADIUS = 0.38     
FINAL_DRIVE = 3.909

# ==============================
# ENGINE PHYSICS
# ==============================

RPM_MAX = 4600         
RPM_IDLE = 750
RPM_STALL = 400        
PEAK_TORQUE = 343      
ENGINE_INERTIA = 0.25  

# Turbo Logic
BOOST_MAX = 1.2        
BOOST_LAG = 0.85       
BOOST_THRESHOLD = 1350 

# ==============================
# DRIVETRAIN
# ==============================

GEAR_RATIOS = {
    0: 0.0,
    1: 4.313,
    2: 2.330,
    3: 1.436,
    4: 1.000,
    5: 0.838
}

DT = 0.03 
FAC_LIMIT = 500

# ==============================
# PHYSICS FUNCTIONS
# ==============================

def get_engine_friction(rpm):
    if rpm < 50: return 0
    return 15 + (rpm / 1000) * 8 + (rpm / 3500)**2 * 15

def get_base_torque(rpm):
    if rpm < 400: return 0 
    if rpm < 1400: 
        return 80 + (rpm - 400) * 0.15 
    if rpm < 2000: 
        return 170 + (rpm - 1400) * 0.25
    if rpm < 2800: 
        return PEAK_TORQUE
    if rpm < 4000: 
        return PEAK_TORQUE - (rpm - 2800) * 0.15
    return max(0, PEAK_TORQUE - (rpm - 4000) * 0.5)

def get_boost_target(rpm, throttle_pct):
    if rpm < BOOST_THRESHOLD: return 0.0
    rpm_factor = min(1.0, (rpm - BOOST_THRESHOLD) / 1000)
    load_factor = (throttle_pct / 100) ** 1.5
    return BOOST_MAX * rpm_factor * load_factor

def calculate_stress(load, rpm, gear, boost):
    lugging_index = 0
    if rpm < 1500 and gear > 2 and load > 50:
        lugging_index = (load/100) * ((1500 - rpm)/500) * (gear/2)
        
    rpm_stress = (rpm / RPM_MAX) ** 3
    load_stress = (load / 100) * (1 + boost)
    
    s_inst = (rpm_stress * 0.3) + (load_stress * 0.5) + lugging_index
    return s_inst, lugging_index

# ==============================
# MAIN SIMULATION
# ==============================

def run():
    gear = 0
    speed = 0.0 
    rpm = RPM_IDLE
    throttle = 0.0
    brake = 0.0
    boost = 0.0
    fac = 0.0
    
    clutch_temp = 20.0 
    clutch_state = "DISENGAGED"
    clutch_engagement = 0.0 # NEW: Tracks the physical position of the pedal (0.0 to 1.0)
    engine_running = True
    load_pct = 0.0
    
    print("REALISTIC DIESEL PHYSICS MODEL")
    print("Controls: W/S (Throttle) | Space/B (Brake) | 0-5 (Gears) | Q (Quit)")
    time.sleep(1)

    while True:
        # --- INPUT HANDLING ---
        if msvcrt.kbhit():
            key = msvcrt.getch().decode().lower()
            if key == 'w': throttle = min(throttle + 10, 100)
            elif key == 's': throttle = max(throttle - 10, 0)
            elif key == 'b' or key == ' ': brake = min(brake + 20, 100)
            elif key == 'n': brake = 0 
            elif key in ['0','1','2','3','4','5']: gear = int(key)
            elif key == 'q': break
        
        if not msvcrt.kbhit():
            brake = max(0, brake - 5)

        # --- PHYSICS CALCULATIONS ---
        
        # 1. External Resistive Forces
        F_aero = 0.5 * AIR_DENSITY * CD * FRONTAL_AREA * speed**2
        F_roll = MASS * G * C_RR if speed > 0.1 else 0
        F_grade = MASS * G * math.sin(math.radians(ROAD_ANGLE_DEG))
        F_brake_phys = (brake / 100) * BRAKE_FORCE_MAX
        
        F_resistance = F_aero + F_roll + F_grade + F_brake_phys
        
        # 2. Engine State & ECU Governor
        # NEW: ECU Anti-stall logic. Adds artificial throttle if RPM drops below idle
        ecu_throttle = throttle
        if rpm < RPM_IDLE and engine_running:
            idle_error = RPM_IDLE - rpm
            # ECU can add up to 40% throttle to save the engine from stalling
            ecu_compensation = min(40.0, idle_error * 0.5) 
            ecu_throttle = max(throttle, ecu_compensation)

        target_boost = get_boost_target(rpm, ecu_throttle)
        if target_boost > boost:
            boost += (target_boost - boost) * (0.05 * (rpm/2000))
        else:
            boost += (target_boost - boost) * 0.2
            
        combustion_torque_base = get_base_torque(rpm)
        air_density_ratio = 1 + (boost * 0.8) 
        
        # Power is generated based on ECU throttle, not just user throttle
        generated_torque = combustion_torque_base * air_density_ratio * (ecu_throttle / 100)
        
        friction_torque = get_engine_friction(rpm)
        net_engine_torque = generated_torque - friction_torque
        
        # 3. Transmission & Coupling Logic
        if not engine_running:
            net_engine_torque = 0
            friction_torque = 20
            
        wheel_rpm = (speed / (2 * math.pi * TIRE_RADIUS)) * 60
        
        if gear == 0:
            # NEUTRAL
            clutch_state = "DISENGAGED"
            clutch_engagement = max(0.0, clutch_engagement - 2.0 * DT) # Pedal goes down
            
            alpha_engine = net_engine_torque / ENGINE_INERTIA
            rpm += alpha_engine * DT * (30/math.pi)
            
            accel_car = -(F_resistance / MASS)
            speed = max(0, speed + accel_car * DT)
            load_pct = 0
            
        else:
            # IN GEAR: Smooth Bite Point Logic
            ratio = GEAR_RATIOS[gear] * FINAL_DRIVE
            target_engine_rpm = wheel_rpm * ratio
            rpm_diff = rpm - target_engine_rpm
            
            clutch_locked = False
            
            if abs(rpm_diff) < 50 and speed > 1.0:
                clutch_locked = True
                clutch_state = "LOCKED"
                clutch_engagement = 1.0
            else:
                clutch_state = "SLIPPING"
                
                # NEW: The virtual driver tries to find the friction zone
                # Starts grabbing at 600 RPM, fully released by 900 RPM
                target_engagement = max(0.0, min(1.0, (rpm - 600) / 300))
                
                # Move the pedal smoothly towards the target
                if clutch_engagement < target_engagement:
                    clutch_engagement += 0.6 * DT # Slowly let clutch out
                else:
                    clutch_engagement -= 1.5 * DT # Push clutch in faster if RPM drops
                    
                clutch_engagement = max(0.0, min(1.0, clutch_engagement))
            
            if not clutch_locked:
                # SLIPPING PHYSICS
                # Max torque the clutch plates can hold right now
                max_clutch_capacity = 600.0 * clutch_engagement 
                rubbing_friction = abs(rpm_diff) * 1.5
                transmitted_torque = min(max_clutch_capacity, rubbing_friction)
                
                if rpm_diff > 0: 
                    # Engine pulling car
                    alpha_engine = (net_engine_torque - transmitted_torque) / ENGINE_INERTIA
                    rpm += alpha_engine * DT * (30/math.pi)
                    
                    drive_force = (transmitted_torque * ratio * DRIVETRAIN_EFF) / TIRE_RADIUS
                    accel_car = (drive_force - F_resistance) / MASS
                    speed = max(0, speed + accel_car * DT)
                else: 
                    # Car pulling engine (Engine braking)
                    alpha_engine = (net_engine_torque + transmitted_torque) / ENGINE_INERTIA
                    rpm += alpha_engine * DT * (30/math.pi)
                    
                    drag_force = (transmitted_torque * ratio * DRIVETRAIN_EFF) / TIRE_RADIUS
                    accel_car = (-drag_force - F_resistance) / MASS
                    speed = max(0, speed + accel_car * DT)

                clutch_temp += (transmitted_torque * abs(rpm_diff)) * 0.00002
                load_pct = min(100, (transmitted_torque / max(1, combustion_torque_base)) * 100)

            else:
                # LOCKED PHYSICS
                reflected_inertia = (MASS * TIRE_RADIUS**2) / (ratio**2 * DRIVETRAIN_EFF)
                total_inertia = ENGINE_INERTIA + reflected_inertia
                
                road_torque_load = (F_resistance * TIRE_RADIUS) / ratio
                total_net_torque = net_engine_torque - road_torque_load
                
                alpha_system = total_net_torque / total_inertia
                rpm += alpha_system * DT * (30/math.pi)
                
                speed_new = (rpm / ratio) * (2 * math.pi * TIRE_RADIUS) / 60
                speed = max(0, speed_new)
                
                load_pct = min(100, (net_engine_torque / (get_base_torque(rpm)*air_density_ratio + 1)) * 100)
                clutch_temp = max(20, clutch_temp - 0.5)

        # 4. Limits and Stalling
        if rpm > RPM_MAX:
            rpm = RPM_MAX 
            throttle = 0 
            
        if rpm < RPM_STALL and gear != 0 and speed < 1:
            engine_running = False
            rpm = 0

        # Note: The old artificial neutral idle logic was removed here, 
        # because the new ECU physics natively handle maintaining the idle perfectly!

        # 5. Stress Accumulation
        s_inst, eli = calculate_stress(load_pct if gear!=0 else 0, rpm, gear, boost)
        fac += s_inst * DT

        # --- DISPLAY ---
        os.system('cls' if os.name == 'nt' else 'clear')
        
        status = "RUNNING" if engine_running else "STALLED (Press 0, then W)"
        if not engine_running and gear == 0 and throttle > 0:
             engine_running = True 
        
        print(f"==============================================")
        print(f"2KD-FTV DYNAMICS | STATUS: {status}")
        print(f"==============================================")
        print(f"Speed:  {speed*3.6:5.1f} km/h  | Gear:   {gear if gear!=0 else 'N'}")
        print(f"RPM:    {int(rpm):5}      | Brake:  {int(brake)}%")
        print(f"Boost:  {boost:5.2f} bar   | Usr Thr:{int(throttle)}%")
        print(f"----------------------------------------------")
        
        rpm_visual = max(0, int(rpm / 150))
        t_bar = '#' * int(throttle/5)
        e_bar = '+' * int(ecu_throttle/5) if ecu_throttle > throttle else t_bar
        r_bar = '=' * rpm_visual
        
        print(f"RPM: [{r_bar:<30}]")
        print(f"USR: [{t_bar:<20}] (Your Foot)")
        print(f"ECU: [{e_bar:<20}] (Actual Fuel)")
        
        print(f"----------------------------------------------")
        print(f"Clutch:    {clutch_state} (Bite: {clutch_engagement*100:.0f}%)")
        print(f"Torque:    {int(generated_torque)} Nm (Fric: {int(friction_torque)} Nm)")
        print(f"Clutch T:  {clutch_temp:.1f} C")
        print(f"==============================================")
        
        time.sleep(DT)

if __name__ == "__main__":
    run()