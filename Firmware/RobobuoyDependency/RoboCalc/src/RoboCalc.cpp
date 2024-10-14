#include "RoboCalc.h"

// Subroutine to add CRC to a string (similar to NMEA format)
String addCRCToString(String input)
{
    // Find where the checksum starts (between '├' and '┤')
    int start = input.indexOf('├');
    int end = input.indexOf('┤');

    // If there's no '├' or '┤' in the string, return as is
    if (start == -1 || end == -1 || end <= start)
    {
        return input; // Invalid format, return original string
    }

    // Calculate the checksum (XOR of all characters between '├' and '┤')
    byte crc = 0;
    for (int i = start + 1; i < end; i++)
    {
        crc ^= input[i]; // XOR operation for each character
    }

    // Convert checksum to hexadecimal format
    char crcHex[3];               // Buffer to hold two hex digits + null terminator
    sprintf(crcHex, "%02X", crc); // Convert byte to uppercase hex string

    // Append the checksum after the asterisk in the string
    input += crcHex;

    return input;
}

// Subroutine to check if the checksum in the string is valid
bool verifyCRC(String input)
{
    // Find where the checksum starts (between '├' and '┤')
    int start = input.indexOf('├');
    int end = input.indexOf('┤');

    // If the string doesn't contain '├' or '┤', it's invalid
    if (start == -1 || end == -1 || end <= start || end + 2 >= input.length())
    {
        return false; // Invalid format
    }

    // Calculate the checksum (XOR of all characters between '├' and '┤')
    byte calculatedCRC = 0;
    for (int i = start + 1; i < end; i++)
    {
        calculatedCRC ^= input[i]; // XOR operation for each character
    }

    // Extract the given checksum from the string (the part after the '┤')
    String givenCRC = input.substring(end + 1, end + 3);

    // Convert calculated CRC to a hexadecimal string
    char calculatedCRCHex[3];
    sprintf(calculatedCRCHex, "%02X", calculatedCRC); // Convert byte to hex

    // Compare the calculated checksum with the provided checksum
    return givenCRC.equalsIgnoreCase(calculatedCRCHex);
}
