import socket
import struct
import time


HOST = 'localhost'
PORT = 8888

#easier to deal with commands as single bytes (easily extendible)
CMD_UPDATE_PID = 0x01
CMD_UPDATE_PLANT = 0x02
CMD_STRESS_TEST = 0x03

class InterfaceUtility:
    def __init__(self):
        self.s = None #py NULL, apparently

    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.settimeout(1.0)
            self.s.connect((HOST, PORT))
            print("Connected to server\n")
            return True
        except Exception as e:
            print(f"Connection failed with error: {e}\n")
            self.s = None
            return False

    def disconnect(self):
        if self.s:
            self.s.close()
            self.s = None
    
    def update_pid_params(self, kp, ki, kd, setpoint):
        try:
            self.s.sendall(bytes([CMD_UPDATE_PID])) # one byte command signal

            data = struct.pack('ffffff', kp, ki, kd, setpoint, 0.0, 0.0) # six x 32bit floats = 24 bytes data signal
            self.s.sendall(data)

            resp = self.s.recv(1)

            if resp == b'A':
                print(f"Successfully updated PID parameters:\n Kp = {kp}, Ki = {ki}, Kd = {kd}, setpoint = {setpoint} m/s\n")
                return True
            else:
                print("Parameter Update Failed")
                return False
            
        except Exception as e:
            print(f"Update failed with Error: {e}")
            return False
        
    def update_plant_params(self, mass, drag_coeff, engine_force, slope):
        try:
            self.s.sendall(bytes([CMD_UPDATE_PLANT]))

            data = struct.pack('fffffff', 0.0, 0.0, mass, drag_coeff, slope, 0.0, engine_force)
            self.s.sendall(data)

            resp = self.s.recv(1)

            if resp == b'A':
                print(f"Successfully updated Plant parameters:\n mass = {mass}kg, drag = {drag_coeff}, engine force = {engine_force}, slope = {slope} m/s\n")
                return True
            else:
                print("Parameter Update Failed")
                return False
            
        except Exception as e:
            print(f"Update failed with Error: {e}")
            return False
        
    def stress_test(self, duration, rate):

        period = 1.0 / rate

        if not self.connect():
            print("Not connected")
            return
        
        success = 0
        failure = 0

        start = time.time()
        next_time = start
        count = 0

        try:
            while (time.time() - start) < duration:
                if time.time() >= next_time:
                    if self.update_pid_params(10, 1, 0.1, 20):
                        success += 1
                    else:
                        failure += 1

                    count += 1
                    next_time += period

                time.sleep(0.001) #pause for a moment to prevent buffer errors

        except KeyboardInterrupt:
            print("\nTest Stopped")
        
        print(f"Stress Test Results: \n{count} Messages Sent over {duration} seconds,\n{success} Successes, {failure} Failures.")

        
#generate the CLI and handle commands
def interface():

    client = InterfaceUtility()

    #connect to server
    if not client.connect():
        print("Connection Failed. Is the server running?")
        return
    
    print("=" * 60)
    print("Cruise Control Tuning Interface")
    print("=" * 60)

    try:
        while True:
            print("\nOptions:\n")
            print("1. Tune PID Controller")
            print("2. Change Vehicle Parameters")
            print("3. Stress Test")
            print("0. Exit")

            selection = input("\n>>").strip() #strip trailing or leading whitespaces

            match selection:
                case "0":
                    print("Exiting interface...")
                    break
                case "1":
                    print("\nSelected: PID Tuning ")
                    try:
                        kp = float(input("Kp: "))
                        ki = float(input("Ki: "))
                        kd = float(input("Kd: "))
                        setpoint = float(input("setpoint (m/s): "))

                        client.update_pid_params(kp, ki, kd, setpoint)

                    except ValueError:
                        print("Invalid input - enter only numerical data")
                case "2":
                    print("\nSelected: Vehicle Parameters ")
                    try:
                        mass = float(input("\nMass (kg): "))
                        drag_coeff = float(input("Drag coefficient: "))
                        engine_force = float(input("Engine force (N): "))
                        slope = float(input("Road slope (deg): "))

                        client.update_plant_params(mass, drag_coeff, engine_force, slope)
                    except ValueError:
                        print("Invalid input - enter only numerical data")
                case "3":
                    print("Stress test: send messages as fast as possible to the server.")
                    duration = int(input("Duration in seconds: "))
                    rate = int(input("Rate in Hz: "))
                    client.stress_test(duration, rate)
                case _:
                    print("Invalid Selection. Please select a number from the list.")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally: client.disconnect()

if __name__ == "__main__":
    interface()