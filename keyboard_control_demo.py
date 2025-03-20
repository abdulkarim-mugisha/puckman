import serial
import time
from pynput import keyboard

class PicoMotorController:
    def __init__(self, port, baudrate=115200):
        """Initialize connection to the Pi Pico"""
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        print(f"Connected to {port}")
    
    def send_command(self, command):
        """Send a command to the Pi Pico"""
        self.ser.write(f"{command}\n".encode())
        time.sleep(0.1)  # Small delay to ensure command is processed
    
    def move(self, direction):
        """Move motors in specified direction"""
        valid_directions = ["left", "l", "right", "r", "up", "u", "down", "d"]
        if direction.lower() in valid_directions:
            self.send_command(direction.lower())
            print(f"Moving {direction}")
        else:
            print(f"Invalid direction: {direction}")
    
    def stop(self):
        """Stop motor movement"""
        self.send_command("stop")
        print("Motors stopped")
    
    def close(self):
        """Close the serial connection"""
        self.ser.close()
        print("Connection closed")

if __name__ == "__main__":
    port = "ACTUAL_PORT"  # Change this to your actual port
    
    try:
        controller = PicoMotorController(port)
        
        print("Keyboard-Controlled Stepper Motor Controller")
        print("Controls:")
        print("Arrow keys or WASD for movement")
        print("Space to stop")
        print("ESC to quit")

        # Create a shared dictionary to store state
        state = {"current_direction": None}
        
        def on_press(key):
            try:
                if key == keyboard.Key.esc:
                    # Stop and quit
                    controller.stop()
                    return False
                elif key == keyboard.Key.space:
                    controller.stop()
                    state["current_direction"] = None
                elif key == keyboard.Key.left or (hasattr(key, 'char') and key.char == 'a'):
                    controller.move('left')
                    state["current_direction"] = 'left'
                elif key == keyboard.Key.right or (hasattr(key, 'char') and key.char == 'd'):
                    controller.move('right')
                    state["current_direction"] = 'right'
                elif key == keyboard.Key.up or (hasattr(key, 'char') and key.char == 'w'):
                    controller.move('up')
                    state["current_direction"] = 'up'
                elif key == keyboard.Key.down or (hasattr(key, 'char') and key.char == 's'):
                    controller.move('down')
                    state["current_direction"] = 'down'
            except AttributeError:
                pass

        def on_release(key):
            try:
                # Only stop if this is the key that started the current movement
                if ((key == keyboard.Key.left and state["current_direction"] == 'left') or
                    (key == keyboard.Key.right and state["current_direction"] == 'right') or
                    (key == keyboard.Key.up and state["current_direction"] == 'up') or
                    (key == keyboard.Key.down and state["current_direction"] == 'down') or
                    (hasattr(key, 'char') and key.char == 'a' and state["current_direction"] == 'left') or
                    (hasattr(key, 'char') and key.char == 'd' and state["current_direction"] == 'right') or
                    (hasattr(key, 'char') and key.char == 'w' and state["current_direction"] == 'up') or
                    (hasattr(key, 'char') and key.char == 's' and state["current_direction"] == 'down')):
                    controller.stop()
                    state["current_direction"] = None
            except AttributeError:
                pass

        # Start keyboard listener
        with keyboard.Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            listener.join()
        
        controller.close()
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"Make sure the Pi Pico is connected and the port '{port}' is correct.")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        try:
            controller.stop()
            controller.close()
        except:
            pass