import math
import pygame
import sys
import csv
from datetime import datetime

# ==============================
# VEHICLE PARAMETERS
# ==============================
MASS = 1750            
G = 9.81
C_RR = 0.015           
AIR_DENSITY = 1.225
CD = 0.35              
FRONTAL_AREA = 2.6     
DRIVETRAIN_EFF = 0.90  
BRAKE_FORCE_MAX = 8000 
TIRE_RADIUS = 0.33     
FINAL_DRIVE = 3.909 

# ==============================
# ENGINE PHYSICS (2.5L Turbo Diesel)
# ==============================
RPM_MAX = 4200         
RPM_IDLE = 750
RPM_STALL = 400        
PEAK_TORQUE = 280      
ENGINE_INERTIA = 0.45  
BOOST_MAX = 1.0        
GEAR_RATIOS = {0: 0.0, 1: 4.313, 2: 2.330, 3: 1.436, 4: 1.000, 5: 0.838}
DT = 0.03 

# ==============================
# PHYSICS FUNCTIONS
# ==============================
def get_engine_friction(rpm):
    if rpm < 50: return 0
    return 20 + (rpm / 1000) * 12 + (rpm / 3000)**2 * 25

def get_max_torque(rpm):
    if rpm < 400: return 0 
    if rpm < 1400: return 160 + (rpm - 400) * 0.1
    if rpm < 2000: return 220 + (rpm - 1400) * 0.1
    if rpm < 3400: return PEAK_TORQUE               
    if rpm <= 4200: return PEAK_TORQUE - (rpm - 3400) * 0.1
    return 0 

# ==============================
# UI HELPER FUNCTIONS
# ==============================
def draw_bar(surface, x, y, width, height, progress, color, bg_color=(40, 40, 40), max_val=1.0, text="", font=None, text_color=(255, 255, 255)):
    pygame.draw.rect(surface, bg_color, (x, y, width, height), border_radius=4)
    fill_width = max(0, min(width, int((progress / max_val) * width)))
    if fill_width > 0:
        pygame.draw.rect(surface, color, (x, y, fill_width, height), border_radius=4)
    pygame.draw.rect(surface, (100, 100, 100), (x, y, width, height), 1, border_radius=4)
    if font and text:
        text_surface = font.render(text, True, text_color)
        surface.blit(text_surface, (x + 10, y + max(0, (height - text_surface.get_height()) // 2)))

def get_stress_color(value):
    if value < 0.4: return (40, 200, 80)   
    if value < 0.75: return (240, 180, 40) 
    return (220, 50, 50)                   

def draw_dyno_graph(surface, x, y, w, h, points, current_rpm, current_hp, font_small):
    pygame.draw.rect(surface, (30, 32, 38), (x, y, w, h), border_radius=5)
    pygame.draw.rect(surface, (80, 80, 80), (x, y, w, h), 1, border_radius=5)
    MAX_RPM_GRAPH, MAX_HP_GRAPH = 5000, 140

    for i in range(4):
        hp_val = i * 40
        gy = y + h - (hp_val / MAX_HP_GRAPH) * h
        pygame.draw.line(surface, (50, 50, 50), (x, gy), (x + w, gy), 1)
        surface.blit(font_small.render(f"{hp_val}", True, (120, 120, 120)), (x - 25, gy - 8))
    
    for i in range(6):
        rpm_val = i * 1000
        gx = x + (rpm_val / MAX_RPM_GRAPH) * w
        pygame.draw.line(surface, (50, 50, 50), (gx, y), (gx, y + h), 1)
        if i > 0:
            surface.blit(font_small.render(f"{i}k", True, (120, 120, 120)), (gx - 10, y + h + 5))

    if len(points) > 1:
        scaled_points = [(x + (r / MAX_RPM_GRAPH) * w, y + h - (hp / MAX_HP_GRAPH) * h) for r, hp in points]
        pygame.draw.lines(surface, (0, 200, 255), False, scaled_points, 3)

    cx = x + (current_rpm / MAX_RPM_GRAPH) * w
    cy = y + h - (current_hp / MAX_HP_GRAPH) * h
    pygame.draw.circle(surface, (255, 255, 255), (cx, cy), 5)

# ==============================
# MAIN SIMULATION
# ==============================
def run():
    pygame.init()
    WIDTH, HEIGHT = 1050, 800 
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2.5L Turbo Diesel + HUMS OBD Telemetry Logger")
    clock = pygame.time.Clock()

    font_large = pygame.font.SysFont("segoeui", 55, bold=True)
    font_medium = pygame.font.SysFont("segoeui", 26, bold=True)
    font_small = pygame.font.SysFont("segoeui", 16)
    
    BG_COLOR = (15, 18, 22)
    ACCENT_BLUE = (0, 150, 255)
    GREEN = (40, 200, 80)
    YELLOW = (240, 180, 40)
    RED = (220, 50, 50)
    WHITE = (240, 240, 240)
    GRAY = (150, 150, 150)

    gear, target_gear, shift_timer = 0, 0, 0.0
    speed, rpm = 0.0, RPM_IDLE
    user_pedal, actual_pedal, brake = 0.0, 0.0, 0.0
    boost, clutch_engagement = 0.0, 0.0
    engine_running = True
    
    target_fuel, injected_fuel = 0.0, 0.0
    generated_torque, net_engine_torque = 0.0, 0.0
    F_resistance = 0.0
    clutch_state = "DISENGAGED"
    dyno_points = []
    current_hp = 0.0

    previous_pedal = 0.0
    previous_torque = 0.0
    total_stress = 0.0
    engine_stress = 0.0
    clutch_stress = 0.0
    drivetrain_stress = 0.0
    aggression = 0.0

    # DATA LOGGER SETUP
    session_time = 0.0
    telemetry_dataset = []
    dataset_headers = [
        "Time_s", "Speed_kmh", "RPM", "Gear", "Throttle_pct", "Brake_pct", 
        "Boost_bar", "Req_Fuel_pct", "Inj_Fuel_pct", "Torque_Nm", "Power_HP", 
        "Engine_Stress_pct", "Drivetrain_Stress_pct", "Clutch_Stress_pct", 
        "Driver_Aggression_pct", "Total_Vehicle_Stress_pct"
    ]

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q: running = False
                elif event.key in [pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5]:
                    target_gear = int(event.unicode)
                    shift_timer = 0.35 

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]: user_pedal = min(user_pedal + 45 * DT, 100)
        else: user_pedal = max(user_pedal - 45 * DT, 0)
        
        if keys[pygame.K_SPACE] or keys[pygame.K_b]: brake = min(brake + 90 * DT, 100)
        else: brake = max(0, brake - 70 * DT)

        if not engine_running and gear == 0 and user_pedal > 5:
            engine_running = True
            rpm = RPM_IDLE
            dyno_points.clear() 

        # --- PHYSICS CALCULATIONS ---
        if shift_timer > 0:
            shift_timer -= DT
            gear = 0 
            clutch_engagement = 0.0 
        else:
            gear = target_gear
        
        F_aero = 0.5 * AIR_DENSITY * CD * FRONTAL_AREA * speed**2
        F_roll = MASS * G * C_RR if speed > 0.1 else 0
        F_brake_phys = (brake / 100) * BRAKE_FORCE_MAX
        F_resistance = F_aero + F_roll + F_brake_phys
        
        actual_pedal += (user_pedal - actual_pedal) * 8.0 * DT
        user_fuel_request = (actual_pedal / 100.0) ** 0.8 
        ecu_idle_fuel = max(0.0, (765 - rpm) / 100.0) if engine_running else 0.0
        target_fuel = max(user_fuel_request, ecu_idle_fuel)
        
        max_clean_fuel = 0.60 + (boost / BOOST_MAX) * 0.40
        injected_fuel = min(target_fuel, max_clean_fuel)
        
        if rpm >= RPM_MAX: injected_fuel = 0.0

        exhaust_energy = ((max(1, rpm) / 2800.0)**1.5) * injected_fuel 
        target_boost = max(0.0, (exhaust_energy - 0.15) * 2.5) 
        target_boost = min(BOOST_MAX, target_boost)
        
        if target_boost > boost: 
            spool_factor = 0.5 + (rpm / 3500.0) * 3.0 
            boost += (target_boost - boost) * spool_factor * DT
        else: 
            boost += (target_boost - boost) * 3.0 * DT 
            
        generated_torque = get_max_torque(rpm) * injected_fuel
        friction_torque = get_engine_friction(rpm)
        net_engine_torque = generated_torque - friction_torque
        
        current_hp = (max(0, net_engine_torque) * rpm) / 9549

        if injected_fuel > 0.8 and engine_running and gear != 0 and rpm < (RPM_MAX - 15):
            if not dyno_points or abs(rpm - dyno_points[-1][0]) > 20:
                 dyno_points.append((rpm, current_hp))
        elif gear == 0 and len(dyno_points) > 0:
             dyno_points.clear()
        
        if not engine_running: net_engine_torque, friction_torque = 0, 20
            
        wheel_rpm = (speed / (2 * math.pi * TIRE_RADIUS)) * 60
        clutch_state = "DISENGAGED"
        current_rpm_diff = 0 
        
        if gear == 0:
            if shift_timer <= 0: clutch_engagement = max(0.0, clutch_engagement - 4.0 * DT) 
            alpha_engine = net_engine_torque / ENGINE_INERTIA
            rpm += alpha_engine * DT * (30/math.pi)
            if rpm > RPM_MAX: rpm = RPM_MAX
            accel_car = -(F_resistance / MASS)
            speed = max(0, speed + accel_car * DT)
        else:
            ratio = GEAR_RATIOS[gear] * FINAL_DRIVE
            target_engine_rpm = wheel_rpm * ratio
            rpm_diff = rpm - target_engine_rpm
            current_rpm_diff = abs(rpm_diff)
            
            if abs(rpm_diff) < 80 and speed > 1.0:
                clutch_state = "LOCKED"
                clutch_engagement = 1.0
                reflected_inertia = (MASS * TIRE_RADIUS**2) / (ratio**2 * DRIVETRAIN_EFF)
                total_inertia = ENGINE_INERTIA + reflected_inertia
                road_torque_load = (F_resistance * TIRE_RADIUS) / ratio
                
                alpha_system = (net_engine_torque - road_torque_load) / total_inertia
                rpm += alpha_system * DT * (30/math.pi)
                if rpm > RPM_MAX: rpm = RPM_MAX 
                speed = max(0, (rpm / ratio) * (2 * math.pi * TIRE_RADIUS) / 60)
            else:
                clutch_state = "SLIPPING"
                if speed > 1.5: target_engagement, engage_rate = 1.0, 5.0 * DT
                else:
                    target_engagement = 0.0 if (target_fuel < 0.05 and speed < 1.0) else max(0.0, min(1.0, (rpm - 550) / 400))
                    engage_rate = 1.5 * DT
                
                if clutch_engagement < target_engagement: clutch_engagement += engage_rate 
                else: clutch_engagement -= 3.0 * DT 
                clutch_engagement = max(0.0, min(1.0, clutch_engagement))
                
                transmitted_torque = min(1200.0 * clutch_engagement, abs(rpm_diff) * 5.0)
                
                if rpm_diff > 0: 
                    rpm += ((net_engine_torque - transmitted_torque) / ENGINE_INERTIA) * DT * (30/math.pi)
                    speed = max(0, speed + ((((transmitted_torque * ratio * DRIVETRAIN_EFF) / TIRE_RADIUS) - F_resistance) / MASS) * DT)
                else: 
                    rpm += ((net_engine_torque + transmitted_torque) / ENGINE_INERTIA) * DT * (30/math.pi)
                    speed = max(0, speed + ((((-transmitted_torque * ratio * DRIVETRAIN_EFF) / TIRE_RADIUS) - F_resistance) / MASS) * DT)
                if rpm > RPM_MAX: rpm = RPM_MAX

        if rpm < RPM_STALL and gear != 0 and speed < 1:
            engine_running = False
            rpm = 0

        # ==============================
        # STRESS TELEMETRY CALCULATIONS
        # ==============================
        rpm_ratio = min(1.0, rpm / RPM_MAX)
        boost_ratio = min(1.0, boost / BOOST_MAX)
        torque_ratio = min(1.0, generated_torque / PEAK_TORQUE)
        
        engine_stress = min(1.0, 0.4 * (rpm_ratio ** 2) + 0.3 * boost_ratio + 0.3 * torque_ratio)
        clutch_stress = min(1.0, (current_rpm_diff / 1000.0) * (1.0 - clutch_engagement))

        if gear != 0:
            wheel_torque = generated_torque * GEAR_RATIOS.get(gear, 0) * FINAL_DRIVE
            drivetrain_stress = min(1.0, abs(wheel_torque) / 4000.0)
        else:
            drivetrain_stress = 0

        pedal_delta = abs(user_pedal - previous_pedal)
        torque_spike = abs(generated_torque - previous_torque)
        aggression = min(1.0, 0.5 * (pedal_delta / 100.0) + 0.3 * (brake / 100.0) + 0.2 * min(1.0, torque_spike / 200.0))

        previous_pedal = user_pedal
        previous_torque = generated_torque

        total_stress = min(1.0, (0.4 * engine_stress) + (0.25 * clutch_stress) + (0.2 * drivetrain_stress) + (0.15 * aggression))

        # ==============================
        # DATA LOGGER APPEND
        # ==============================
        session_time += DT
        # Log the current frame's data
        current_data_row = [
            round(session_time, 2),
            round(speed * 3.6, 2),
            int(rpm),
            gear,
            int(user_pedal),
            int(brake),
            round(boost, 2),
            round(target_fuel * 100, 1),
            round(injected_fuel * 100, 1),
            int(net_engine_torque),
            int(current_hp),
            int(engine_stress * 100),
            int(drivetrain_stress * 100),
            int(clutch_stress * 100),
            int(aggression * 100),
            int(total_stress * 100)
        ]
        telemetry_dataset.append(current_data_row)

        # --- DRAWING THE UI ---
        screen.fill(BG_COLOR)
        
        screen.blit(font_medium.render("2.5L Turbo Diesel Dynamics Simulator", True, WHITE), (40, 20))
        status_color = GREEN if engine_running else RED
        status_text = "ENGINE RUNNING" if engine_running else "STALLED"
        screen.blit(font_small.render(status_text, True, status_color), (WIDTH - 150, 30))

        speed_kmh = speed * 3.6
        screen.blit(font_large.render(f"{int(speed_kmh):03d}", True, WHITE), (40, 60))
        screen.blit(font_medium.render("km/h", True, GRAY), (145, 85))

        gear_display = "S" if shift_timer > 0 else (str(target_gear) if target_gear != 0 else "N")
        gear_color = YELLOW if shift_timer > 0 else ACCENT_BLUE
        pygame.draw.circle(screen, (30, 30, 30), (320, 90), 45)
        pygame.draw.circle(screen, gear_color, (320, 90), 45, 2)
        gear_text = font_large.render(gear_display, True, gear_color)
        screen.blit(gear_text, (320 - gear_text.get_width()//2, 90 - gear_text.get_height()//2))

        bar_x = 40
        bar_w = 420
        
        rpm_color = GREEN if rpm < 3500 else (YELLOW if rpm < 4000 else RED)
        draw_bar(screen, bar_x, 170, bar_w, 35, rpm, rpm_color, max_val=RPM_MAX, text=f"RPM: {int(rpm)}", font=font_small)
        draw_bar(screen, bar_x, 220, bar_w, 25, boost, ACCENT_BLUE, max_val=1.5, text=f"Boost Pressure: {boost:.2f} bar", font=font_small)
        draw_bar(screen, bar_x, 260, bar_w, 25, target_fuel * 100, (80, 80, 80), max_val=100, text="Requested Fuel %", font=font_small)
        draw_bar(screen, bar_x, 295, bar_w, 25, injected_fuel * 100, YELLOW, max_val=100, text="Injected Fuel %", font=font_small)

        draw_bar(screen, bar_x, 350, bar_w//2 - 10, 25, user_pedal, GREEN, max_val=100, text=f"Throttle: {int(user_pedal)}%", font=font_small)
        draw_bar(screen, bar_x + bar_w//2 + 10, 350, bar_w//2 - 10, 25, brake, RED, max_val=100, text=f"Brake: {int(brake)}%", font=font_small)

        y_offset = 400
        stats = [
            f"Net Engine Torque: {int(net_engine_torque)} Nm",
            f"Current Power Output: {int(current_hp)} HP",
            f"Clutch Status: {clutch_state} ({clutch_engagement*100:.0f}%)",
            f"Aerodynamic & Rolling Drag: {int(F_resistance)} N"
        ]
        for i, stat in enumerate(stats):
            screen.blit(font_small.render(stat, True, GRAY), (bar_x, y_offset + (i * 26)))

        graph_x, graph_y, graph_w, graph_h = 520, 170, 480, 335
        draw_dyno_graph(screen, graph_x, graph_y, graph_w, graph_h, dyno_points, rpm, current_hp, font_small)
        screen.blit(font_small.render("Live Power Curve (HP)", True, WHITE), (graph_x, graph_y - 25))

        dash_y = 540
        pygame.draw.rect(screen, (22, 25, 30), (30, dash_y, 990, 200), border_radius=8)
        pygame.draw.rect(screen, (60, 60, 65), (30, dash_y, 990, 200), 1, border_radius=8)
        screen.blit(font_medium.render("HUMS Stress & Driver Aggression Telemetry", True, WHITE), (50, dash_y + 15))

        draw_bar(screen, 50, dash_y + 60, 460, 25, engine_stress, get_stress_color(engine_stress), text=f"Engine Stress: {int(engine_stress*100)}%", font=font_small)
        draw_bar(screen, 540, dash_y + 60, 460, 25, drivetrain_stress, get_stress_color(drivetrain_stress), text=f"Drivetrain Stress: {int(drivetrain_stress*100)}%", font=font_small)
        
        draw_bar(screen, 50, dash_y + 100, 460, 25, clutch_stress, get_stress_color(clutch_stress), text=f"Clutch Slip Stress: {int(clutch_stress*100)}%", font=font_small)
        draw_bar(screen, 540, dash_y + 100, 460, 25, aggression, get_stress_color(aggression), text=f"Driver Aggression (Jerk/Spike): {int(aggression*100)}%", font=font_small)

        draw_bar(screen, 50, dash_y + 145, 950, 35, total_stress, get_stress_color(total_stress), max_val=1.0, text=f"OVERALL VEHICLE STRESS SCORE: {int(total_stress*100)}%", font=font_medium, text_color=(0,0,0) if total_stress > 0.4 else WHITE)

        controls = font_small.render("Controls: Hold W (Accelerate) | Space (Brake) | 0-5 (Shift Gears) | Q (Quit to Save Data)", True, (100, 100, 100))
        screen.blit(controls, (WIDTH//2 - controls.get_width()//2, HEIGHT - 35))

        pygame.display.flip()
        clock.tick(int(1 / DT))

    # ==============================
    # EXPORT CSV WHEN APP CLOSES
    # ==============================
    pygame.quit()
    
    if len(telemetry_dataset) > 0:
        filename = f"obd_telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(dataset_headers)
                writer.writerows(telemetry_dataset)
            print(f"\n✅ SUCCESS: Driving session saved to {filename}")
            print(f"📊 Total data points logged: {len(telemetry_dataset)}")
        except Exception as e:
            print(f"\n❌ ERROR saving CSV: {e}")
    else:
        print("\n⚠️ No data was recorded during the session.")

    sys.exit()

if __name__ == "__main__":
    run()