    # def send_distance_direction(self):
    #     try:
    #         distance = float(self.distance_input.text())
    #         direction = float(self.direction_input.text())
    #     except ValueError:
    #         print("Invalid distance or direction input.")
    #         return

    #     # Construct and send the message
    #     base_message = f"{LORAINF},{DIRDIST},{IDLE},{direction},{distance}"
    #     full_message = generate_nmea_message(base_message)
    #     send_message(full_message, self.serial_port)
    #     print(f"Distance & Direction sent: {direction},{distance}")
