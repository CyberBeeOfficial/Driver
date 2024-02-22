import serial
import re
import time
from enum import Enum
import threading

# Global variables
division_rate = 0
baud_rate_index = 4  # Default baud rate index for 115200
testing_baud = False
baud_rates = [9600, 19200, 38400, 57600, 115200]  # Baud rates list
listen_only = False


# Enum definitions
class UserCommand(Enum):
    Test = 0x20
    SetDivisionRate = 0x22
    Confirm = 0x23
    ChangeBaudRate = 0x30
    # Add more commands as needed


class SentData(Enum):
    ACK = 0x00
    ODOMETRY = 0x01
    POSITION = 0x02
    # Add more message types here


class PoseMessage:
    def __init__(self, message_type, timestamp, position, quaternion, velocity):
        self.message_type = message_type
        self.timestamp = timestamp
        self.position = position
        self.quaternion = quaternion
        self.velocity = velocity

    def __str__(self):
        return (
            f"Type: {self.message_type}, Time: {self.timestamp}, "
            f"Position: {self.position}, Quaternion: {self.quaternion}, "
            f"Velocity: {self.velocity}"
        )


def initialize_serial_connection(port="/dev/ttyAMA0", baud_rate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        print(f"Serial port {port} initialized with baud rate {ser.baudrate}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None


def parse_and_print_message(message):
    print(f"Raw Message: {message}")
    try:
        # Splitting the message into its components
        parts = message.split(";")
        if len(parts) != 8:  # Including the checksum part
            raise ValueError("Incomplete message format")

        (
            _,
            message_type,
            timestamp,
            position_data,
            quaternion_data,
            velocity_data,
            received_chksum,
            _,
        ) = parts

        # Recompute the checksum for validation
        computed_chksum = calculate_checksum(
            ";".join(parts[:-2])
        )  # Excluding the received checksum and <EN> part

        # Check if the received checksum matches the computed checksum
        if int(received_chksum) != computed_chksum:
            raise ValueError("Checksum mismatch")

        # Extracting position, quaternion, and velocity
        position = tuple(map(float, position_data.split(",")))
        quaternion = tuple(map(float, quaternion_data.split(",")))
        velocity = tuple(map(float, velocity_data.split(",")))

        # Constructing and printing the message
        pose_message = PoseMessage(
            message_type, timestamp, position, quaternion, velocity
        )
        print(f"Validated Message: {pose_message}")

    except ValueError as e:
        print(f"Error parsing message: {e}")


def calculate_checksum(command_str):
    return sum(bytearray(command_str.encode("utf-8"))) % 256


# Updated function with added print statement
def send_command(ser, command, data):
    if ser and ser.isOpen():
        command_str = f"<ST>;{command.value};{data}"
        chksum = calculate_checksum(command_str)
        command_str_with_checksum = f"{command_str};{chksum};<EN>"
        ser.write(command_str_with_checksum.encode())
        print(f"Sent Command: {command.name}, Data: {data}, Checksum: {chksum}")


def update_baud_rate(ser, rate_index, previous_rate_index):
    global baud_rate_index
    baud_rate_index = rate_index
    new_ser = initialize_serial_connection(baud_rate=baud_rates[baud_rate_index])

    if new_ser:  # Replace with actual test command
        if ser:
            ser.close()
        print(f"Baud Rate successfully changed to {baud_rates[baud_rate_index]}")
        test_baud_rate(new_ser)
        return new_ser
    else:
        print("Failed to change Baud Rate, reverting to previous setting.")
        return initialize_serial_connection(baud_rate=baud_rates[previous_rate_index])


def test_baud_rate(ser):
    global testing_baud
    testing_baud = True
    send_command(ser, UserCommand.Test, 0)  # Sending TEST command


def receive_data(ser):
    global testing_baud, listen_only

    accumulated_data = ""
    while True:
        if ser and ser.isOpen():
            try:
                line = ser.readline().decode("utf-8").strip()

                accumulated_data += line

                if "<EN>" in accumulated_data:
                    messages = re.findall(r"<ST>(.*?)<EN>", accumulated_data, re.DOTALL)
                    accumulated_data = ""  # Clear the buffer after processing

                    for message in messages:
                        if listen_only:  # Check if line is not empty
                            print(
                                f"Received Raw Data: {message}"
                            )  # Print every line received, regardless of content
                        if testing_baud and str(SentData.ACK.value) in message:
                            send_command(ser, UserCommand.Confirm, 0)
                            print("ACK received, Confirm sent")
                            testing_baud = False
                        else:
                            parse_and_print_message(message)

            except serial.SerialException as e:
                print(f"Serial Exception: {e}")
                ser.close()
                ser = None
                time.sleep(1)  # Wait before reconnecting
                ser = initialize_serial_connection()
            except Exception as e:
                print(f"Unexpected error: {e}")
        else:
            # Attempt to reconnect
            ser = initialize_serial_connection()
            time.sleep(1)  # Wait before retrying


# Initial connection
accumulated_data = ""  # Buffer for accumulating data

# Initial connection
ser = initialize_serial_connection()

# Create and start the thread for receiving data
receiving_thread = threading.Thread(target=receive_data, args=(ser,))
receiving_thread.start()
while True:
    if ser and not listen_only:
        try:
            command_input = input("Enter command: ")

            if command_input == "t":
                division_rate = (division_rate + 1) % 6  # Cycle through 0 to 5
                send_command(ser, UserCommand.SetDivisionRate, division_rate)
                print(f"Set Division Rate to {division_rate}")
            elif command_input == "y":
                previous_baud_rate_index = baud_rate_index
                baud_rate_index = (baud_rate_index + 1) % len(baud_rates)
                send_command(ser, UserCommand.ChangeBaudRate, baud_rate_index)
                time.sleep(2)
                ser = update_baud_rate(ser, baud_rate_index, previous_baud_rate_index)
            elif (
                command_input == "z"
            ):  # Assuming 'z' is the trigger for the TEST command
                test_baud_rate(ser)
                print("Baud rate start test ")

            elif command_input == "l":
                listen_only = True  # Toggle listen-only mode
                print(
                    "Listen-only mode activated. No commands will be sent. Listening for incoming data..."
                )

                # No else clause needed; the program will continue to listen and print data in the background

        except serial.SerialException as e:
            print(f"Serial Exception: {e}")
            ser.close()
            ser = None
            time.sleep(1)  # Wait before reconnecting
            ser = initialize_serial_connection()
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")

    elif ser and listen_only:
        # If in listen-only mode, continuously read and print data from the serial port
        try:
            while True:
                if ser and ser.isOpen():
                    print("in while loop and ser open")
                    line = ser.readline().decode("utf-8").strip()
                    if line:  # If there's data, print it
                        print(line)
                        print("sdgsdg")

        except KeyboardInterrupt:
            print("Exiting listen-only mode...")
            break
        except Exception as e:
            print(f"Error while listening: {e}")
    else:
        # Attempt to reconnect
        ser = initialize_serial_connection()
        time.sleep(1)  # Wait before retrying

receiving_thread.join()

if ser:
    ser.close()
