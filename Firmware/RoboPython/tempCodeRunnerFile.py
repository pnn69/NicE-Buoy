def validate_and_extract_data(input_string):
    # Step 1: Check if the input contains '*' for separating data and checksum
    if '*' not in input_string:
        return -1

    # Step 2: Split the string into data and checksum
    data, checksum_str = input_string.split('*')
    
    # Remove the initial '$' from data if present
    if data.startswith('$'):
        data = data[1:]

    # Step 3: Convert the provided checksum to an integer
    try:
        checksum = int(checksum_str)
    except ValueError:
        return -1  # Invalid checksum format

    # Step 4: Calculate the checksum by XOR-ing ASCII values of each character in the data
    calculated_checksum = 0
    for char in data:
        calculated_checksum ^= ord(char)
    print(hex(calculated_checksum))
    print(hex(checksum))
    # Step 5: Compare the calculated checksum with the provided checksum
    if calculated_checksum == checksum:
        return data  # Checksum matches, return data
    else:
        return -1  # Checksum does not match
