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

BRAKE_FORCE_MAX = 8000 

# TUNED: Switched to Innova/Hiace standard specs for realistic RPM-to-Speed mapping
TIRE_RADIUS = 0.33     # Standard 205/65R15 tires
FINAL_DRIVE = 3.909    # Standard shorter differential 

# ==============================
# ENGINE PHYSICS (2KD-FTV)
# ==============================

RPM_MAX = 4600         
RPM_IDLE = 750
RPM_STALL = 400        
PEAK_TORQUE = 343      
ENGINE_INERTIA = 0.45  

BOOST_MAX = 1.2        

GEAR_RATIOS = {0: 0.0, 1: 4.313, 2: 2.330, 3: 1.436, 4: 1.000, 5: 0.838}
DT = 0.03 

# ==============================
# PHYSICS FUNCTIONS
# ==============================

def get_engine_friction(rpm):
    if rpm < 50: return 0
    return 20 + (rpm / 1000) * 10 + (rpm / 3000)**2 * 20

def get_max_torque(rpm):
    if rpm < 400: return 0 
    if rpm < 1200: return 200 + (rpm - 400) * 0.1 
    if rpm < 2000: return 280 + (rpm - 1200) * 0.078
    if rpm < 2800: return PEAK_TORQUE
    if rpm < 4000: return PEAK_TORQUE - (rpm - 2800) * 0.1
    return max(0, 223 - (rpm - 4000) * 0.4)

# ==============================
# MAIN SIMULATION
# ==============================

def run():
    gear = 0
    target_gear = 0
    shift_timer = 0.0 
    
    speed = 0.0 
    rpm = RPM_IDLE
    
    user_pedal = 0.0     
    actual_pedal = 0.0   
    
    brake = 0.0
    boost = 0.0
    
    clutch_engagement = 0.0 
    engine_running = True
    
    print("REALISTIC DIESEL PHYSICS MODEL (Fixed RPM/Speed Ratios)")
    print("Controls: W/S (Throttle) | Space/B (Brake) | 0-5 (Gears) | Q (Quit)")
    time.sleep(2)

    while True:
        # --- INPUT HANDLING ---
        if msvcrt.kbhit():
            key = msvcrt.getch().decode().lower()
            if key == 'w': user_pedal = min(user_pedal + 10, 100)
            elif key == 's': user_pedal = max(user_pedal - 10, 0)
            elif key == 'b' or key == ' ': brake = min(brake + 20, 100)
            elif key == 'n': brake = 0 
            elif key in ['0','1','2','3','4','5']: 
                new_gear = int(key)
                if new_gear != target_gear:
                    target_gear = new_gear
                    shift_timer = 0.35 
            elif key == 'q': break
        
        if not msvcrt.kbhit():
            brake = max(0, brake - 5)

        # --- PHYSICS CALCULATIONS ---
        
        # Shift Logic
        if shift_timer > 0:
            shift_timer -= DT
            gear = 0 
            clutch_engagement = 0.0 
        else:
            gear = target_gear
        
        # External Resistive Forces
        F_aero = 0.5 * AIR_DENSITY * CD * FRONTAL_AREA * speed**2
        F_roll = MASS * G * C_RR if speed > 0.1 else 0
        F_brake_phys = (brake / 100) * BRAKE_FORCE_MAX
        F_resistance = F_aero + F_roll + F_brake_phys
        
        # 1. Drive-by-Wire & ECU Idle Governor
        actual_pedal += (user_pedal - actual_pedal) * 8.0 * DT
        
        user_fuel_request = (actual_pedal / 100.0) ** 0.75 
        ecu_idle_fuel = max(0.0, (765 - rpm) / 100.0) if engine_running else 0.0
        target_fuel = max(user_fuel_request, ecu_idle_fuel)
        
        # 2. ECU Smoke Limiter 
        max_clean_fuel = 0.75 + (boost / BOOST_MAX) * 0.25
        injected_fuel = min(target_fuel, max_clean_fuel)
        
        # 3. VNT Turbocharger Physics
        exhaust_energy = (rpm / 2500.0) * injected_fuel 
        target_boost = max(0.0, (exhaust_energy - 0.10) * 3.0) 
        target_boost = min(BOOST_MAX, target_boost)
        
        if target_boost > boost:
            spool_factor = 1.2 + (rpm / 3000.0) * 4.0 
            boost += (target_boost - boost) * spool_factor * DT
        else:
            boost += (target_boost - boost) * 5.0 * DT 
            
        # 4. Torque Generation
        generated_torque = get_max_torque(rpm) * injected_fuel
        friction_torque = get_engine_friction(rpm)
        
        if rpm > RPM_MAX - 100: 
            generated_torque *= max(0.0, (RPM_MAX - rpm) / 100)

        net_engine_torque = generated_torque - friction_torque
        
        # --- TRANSMISSION LOGIC ---
        if not engine_running:
            net_engine_torque = 0
            friction_torque = 20
            
        wheel_rpm = (speed / (2 * math.pi * TIRE_RADIUS)) * 60
        clutch_state = "DISENGAGED"
        
        if gear == 0:
            if shift_timer <= 0: 
                clutch_engagement = max(0.0, clutch_engagement - 4.0 * DT) 
            
            alpha_engine = net_engine_torque / ENGINE_INERTIA
            rpm += alpha_engine * DT * (30/math.pi)
            
            accel_car = -(F_resistance / MASS)
            speed = max(0, speed + accel_car * DT)
        else:
            ratio = GEAR_RATIOS[gear] * FINAL_DRIVE
            target_engine_rpm = wheel_rpm * ratio
            rpm_diff = rpm - target_engine_rpm
            
            if abs(rpm_diff) < 80 and speed > 1.0:
                clutch_state = "LOCKED"
                clutch_engagement = 1.0
                
                reflected_inertia = (MASS * TIRE_RADIUS**2) / (ratio**2 * DRIVETRAIN_EFF)
                total_inertia = ENGINE_INERTIA + reflected_inertia
                road_torque_load = (F_resistance * TIRE_RADIUS) / ratio
                
                alpha_system = (net_engine_torque - road_torque_load) / total_inertia
                rpm += alpha_system * DT * (30/math.pi)
                speed = max(0, (rpm / ratio) * (2 * math.pi * TIRE_RADIUS) / 60)
            else:
                clutch_state = "SLIPPING"
                
                if speed > 1.5:
                    target_engagement = 1.0 
                    engage_rate = 5.0 * DT 
                else:
                    target_engagement = 0.0 if (target_fuel < 0.05 and speed < 1.0) else max(0.0, min(1.0, (rpm - 550) / 400))
                    engage_rate = 1.5 * DT 
                
                if clutch_engagement < target_engagement: 
                    clutch_engagement += engage_rate 
                else: 
                    clutch_engagement -= 3.0 * DT 
                    
                clutch_engagement = max(0.0, min(1.0, clutch_engagement))
                
                transmitted_torque = min(1200.0 * clutch_engagement, abs(rpm_diff) * 5.0)
                
                if rpm_diff > 0: 
                    rpm += ((net_engine_torque - transmitted_torque) / ENGINE_INERTIA) * DT * (30/math.pi)
                    speed = max(0, speed + ((((transmitted_torque * ratio * DRIVETRAIN_EFF) / TIRE_RADIUS) - F_resistance) / MASS) * DT)
                else: 
                    rpm += ((net_engine_torque + transmitted_torque) / ENGINE_INERTIA) * DT * (30/math.pi)
                    speed = max(0, speed + ((((-transmitted_torque * ratio * DRIVETRAIN_EFF) / TIRE_RADIUS) - F_resistance) / MASS) * DT)

        if rpm < RPM_STALL and gear != 0 and speed < 1:
            engine_running = False
            rpm = 0

        # --- DISPLAY ---
        os.system('cls' if os.name == 'nt' else 'clear')
        
        if not engine_running and gear == 0 and user_pedal > 0:
             engine_running = True 
             rpm = RPM_IDLE 
             
        gear_display = f"[SHIFTING]" if shift_timer > 0 else (target_gear if target_gear != 0 else 'N')
        
        print(f"==================================================")
        print(f"2KD-FTV DYNAMICS | STATUS: {'RUNNING' if engine_running else 'STALLED (0 -> W)'}")
        print(f"==================================================")
        print(f"Speed:  {speed*3.6:5.1f} km/h  | Gear:   {gear_display}")
        print(f"RPM:    {int(rpm):5}      | Brake:  {int(brake)}%")
        print(f"Boost:  {boost:5.2f} bar   | Pedal:  {int(user_pedal)}%")
        print(f"--------------------------------------------------")
        
        print(f"RPM:  [{'=' * max(0, int(rpm / 150)):<30}]")
        print(f"Req : [{'#' * int(target_fuel * 20):<20}] (ECU Fuel Request)")
        print(f"Inj : [{'+' * int(injected_fuel * 20):<20}] (Actual Injected)")
        if target_fuel > injected_fuel + 0.05:
            print(f"      *** FUEL LIMITED BY MAP SENSOR (TURBO LAG) ***")
        
        print(f"--------------------------------------------------")
        print(f"Clutch: {clutch_state} (Bite: {clutch_engagement*100:.0f}%)")
        print(f"Torque: {int(generated_torque)} Nm (Net: {int(net_engine_torque)} Nm)")
        print(f"==================================================")
        
        time.sleep(DT)

if __name__ == "__main__":
    run()