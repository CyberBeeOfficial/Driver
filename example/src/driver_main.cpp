/*
 * Copyright (c) 2024 CyberBee Ltd.
 * All rights reserved.
 *
 * This file is part of the CyberBee IMX8 C++ Driver.
 * 
 * Description:
 * This code is part of the CyberBee  IMX8 C++ Driver layer, responsible for
 * managing communication between imx8 and other devices sucha s PI-4B.
 * This code meant to be compiled on the client device i.e. PI-4B,or other ubuntu system.
 *
 */

#include <csignal>
#include <iostream>
#include <memory>
#include <thread>  // Include the thread header

#include "../include/data_receiver.hpp"
#include "../include/data_sender.hpp"
#include "../include/serial_comm.hpp"  // Adjust the include path according to your project structure
std::unique_ptr<SerialPort> globalSerialPort;
std::unique_ptr<DataReceiver> data_receiver;
std::unique_ptr<DataSender> data_sender;

enum UserCommand : uint8_t {
    TestBaudRate = 30,
    SetDivisionRate = 32,
    Confirm = 34,
    ChangeBaudRate = 36,
    GpsMsg = 37,
    FtReset = 39
};

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // Signal DataReceiver to stop
    if (data_receiver)
    {
        data_receiver->stop();
    }
    // Terminate program
    exit(signum);
}

void readDataThread(DataReceiver* data_receiver)
{
    while (true)
    {
        data_receiver->readData();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Global variables
uint16_t division_rate = 0;
uint8_t baud_rate_index = 4;  // Default baud rate index for 115200
bool testing_baud = false;
int baud_rates[5] = { 9600, 19200, 38400, 57600, 115200 };  // Baud rates list
bool listen_only = false;
std::stringstream data;

int main()
{
    // Register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    std::string portName = "/dev/ttyUSB0";  // ttyAMAO is the port on Pi26
    unsigned int baudRate = 115200;         // current baud rate for pi26
    char command_input;
    bool loop = 0;
    // Create an instance of SerialPort
    globalSerialPort = std::make_unique<SerialPort>(portName, baudRate);

    // Start communication in a separate thread
    globalSerialPort->OpenSerialConnection();

    data_receiver = std::make_unique<DataReceiver>(globalSerialPort.get());
    data_sender = std::make_unique<DataSender>(globalSerialPort.get());

    // Create a thread for reading data and another for extracting it from buff
    std::thread readThread(&DataReceiver::readData, data_receiver.get());
    std::thread processThread(&DataReceiver::processData, data_receiver.get());

    // Wait for a signal (e.g., SIGINT) to terminate the program
    // This loop is just a placeholder; adjust according to your program's needs
    while (true)
    {
        if (globalSerialPort->IsSerialConnectionOpen())
        {
            try
            {
                std::cout << "Enter Command:" << std::endl;
                std::cin >> command_input;
                switch (command_input)
                {
                    case 'a':
                        /* TODO: SET DEVISION RATE AND SEND IT */
                        division_rate = (division_rate + 1) % 6;  // cycle through 0 to 5
                        data_sender->SendCommand(SetDivisionRate, std::to_string(division_rate), loop);
                        std::cout << "Set Devision Rate to " << division_rate << std::endl;
                        break;

                    case 'b':
                        /* TODO: SET NEW BAUD RATE AND SEND IT */
                        baud_rate_index = (baud_rate_index + 1) % (sizeof(baud_rates) / sizeof(baud_rates[0]));
                        data_sender->SendCommand(ChangeBaudRate, std::to_string(baud_rate_index), loop);
                        std::cout << "Baud Rate Changed to: " << baud_rates[baud_rate_index] << std::endl;
                        break;

                    case 'c':
                        /* TODO: START BAUD RATE TEST - requires seperate
                         * function to test message send */
                        data_sender->SendCommand(TestBaudRate, std::to_string(0), loop);
                        std::cout << "Baud Rate Test start " << std::endl;
                        break;

                    case 'd':
                        /* TODO: START Confirm - start getting info */
                        data_sender->SendCommand(Confirm, std::to_string(0), loop);
                        std::cout << "Confirm start " << std::endl;
                        break;

                    case 'e':
                        loop = 1;
                        /* TODO: Gps msg - send gps msg */
                        //get and build the gps message
                        data = data_sender->createGpsMsg();
                        //send the gps message
                        data_sender->SendCommand(GpsMsg, data.str(), loop);
                        std::cout << "Gps msg sent " << std::endl;
                        break;
                    
                    case 'r':
                        data_sender->SendCommand(FtReset, "1", loop);
                        std::cout << "Reset msg sent " << std::endl;
                        command_input = 'e';
                        break;

                    default:
                        std::cout << "Toggle listen-only mode " << std::endl;
                        break;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Cleanup is handled automatically by the unique_ptr and SerialPort
    // destructor

    // data_receiver.stop();

    // Wait for the threads to finish
    readThread.join();
    processThread.join();

    return 0;
}