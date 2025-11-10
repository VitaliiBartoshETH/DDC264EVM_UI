from rotational_control import RotationalController
import time
import json

# Create controller - make sure COM6 is the correct port
rot_controller = RotationalController(port="COM6")

try:
    # Test enable/disable first
   
    
    rot_controller.arduino.write(json.dumps({"command": "enable_motor"}).encode() + b'\n')
    time.sleep(0.5)
    
 
    
    for i in range(6):
        rot_controller.rotate(60)
        print("Rotating...")
        time.sleep(2)

    rot_controller.arduino.write(json.dumps({"command": "disable_motor"}).encode() + b'\n')
    time.sleep(0.5)



except Exception as e:
    print(f"Error: {e}")
finally:
    rot_controller.arduino.close()  # Always close the serial port