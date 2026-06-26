from gpiozero import OutputDevice
import time

# Define the GPIO pin connected to your Motor Driver
# (We use OutputDevice because we only need it to spin one direction)
motor_pin = OutputDevice(17) 

# Adjust this time slightly based on your exact motor speed
# 0.6 seconds is the theoretical time for 1 rotation at 100 RPM
TIME_PER_SHOT = 0.6  

def fire_single_shot():
    print("Firing one bullet...")
    motor_pin.on()              # Turn motor ON
    time.sleep(TIME_PER_SHOT)   # Wait for the cam to make exactly one rotation
    motor_pin.off()             # Turn motor OFF
    print("Reloaded and ready.")

def fire_full_auto(shots):
    print(f"Firing {shots} bullets in full auto!")
    motor_pin.on()
    # Multiply the time per shot by the number of bullets
    time.sleep(TIME_PER_SHOT * shots) 
    motor_pin.off()
    print("Cease fire.")

# --- Main Program ---
try:
    print("Launcher Online.")
    time.sleep(1)
    
    # Test firing 1 shot
    fire_single_shot()
    
    time.sleep(2) # Wait 2 seconds
    
    # Test firing a 3-round burst
    fire_full_auto(3)

except KeyboardInterrupt:
    print("Shutting down launcher.")
    motor_pin.off()