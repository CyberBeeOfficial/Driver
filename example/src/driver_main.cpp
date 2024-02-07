#include <csignal>
#include <iostream>
#include <memory>

#include "../include/serial_comm.hpp"  // Adjust the include path according to your project structure
#include "serial_comm.cpp"
std::unique_ptr<SerialPort> globalSerialPort;

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // Assuming globalSerialPort is your SerialPort object
    // No explicit call to close the serial port is needed here
    // because it will be handled by the SerialPort destructor.

    // Terminate program
    exit(signum);
}

int main()
{
    // Register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    std::string portName = "/dev/ttyAMA0";  // ttyAMAO is the port on Pi26
    unsigned int baudRate = 115200;         // current baud rate for pi26

    // Create an instance of SerialPort
    globalSerialPort = std::make_unique<SerialPort>(portName, baudRate);

    // Start communication in a separate thread
    globalSerialPort->StartCommunication();

    // Create an instance of DataReceiver
    // DataReceiver data_receiver(globalSerialPort.get());
    // DataSender data_sender(globalSerialPort.get());

    // Your application logic here
    // For example, keep the program running or perform other tasks
    // ...

    // Use a method from DataReceiver
    // data_receiver.readData(serialPort);

    // Use the calculateChecksum function with SenderTag
    // std::string message = "example message";
    // int senderChecksum = calculateCheckSum<DataReceiver>(message,
    // SenderTag{});

    // Use the calculateChecksum function with ReceiverTag
    // int receiverChecksum = calculateCheckSum<DataReceiver>(message,
    // ReceiverTag{});

    // Use other methods or functions as needed

    // Wait for a signal (e.g., SIGINT) to terminate the program
    // This loop is just a placeholder; adjust according to your program's needs
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Cleanup is handled automatically by the unique_ptr and SerialPort
    // destructor

    return 0;
}