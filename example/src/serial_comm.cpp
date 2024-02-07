#include "../include/serial_comm.hpp"
#include <boost/asio.hpp>
#include <mutex>
#include <string>


//constructor for the object of type SerialPort
SerialPort::SerialPort(const std::string& port, unsigned int baud_rate)
    : io_service(), serial(io_service, port), mtx(std::make_mutex()) {
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}


// open serial port 
void SerialPort::OpenSerialConnection(std::string port, boost::system::error_code& error)
{   
    std::thread connectThread([this, port, &error]() {

    // Specify the port name and error code variable
    std::string port_name = "/dev/ttyAMA0"; // Replace with your actual port name
    boost::system::error_code error;

    // Open the port with desired options
    serial.open(port_name, error);

    if (error) {
    // Handle error opening the port (e.g., print error message, exit)
    std::cerr << "Error opening serial port: " << error.message() << std::endl;
    }
    });
    connectThread.join();
}