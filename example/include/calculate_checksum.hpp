// calculate_checksum.hpp

#ifndef CALCULATE_CHECKSUM_HPP
#define CALCULATE_CHECKSUM_HPP

#include <string>

// Define the ReceiverTag struct
struct ReceiverTag {};

// Define the SenderTag struct
struct SenderTag {};


template <class T>
int calculateCheckSum(std::string& message, SenderTag) {
    // ... do something specific for Sender
}









template <class T>
int calculateCheckSum(std::string& message, ReceiverTag) {
    // ... do something specific for Receiver
}

/*

template <class T>
int calculateCheckSum(std::string& message)
{

    // Function implementation using the type T
    T object;
    

    else{

        //do something else this time 
    }
    
    return 0; // Placeholder return value
}
*/
#endif