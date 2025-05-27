
    value_strs = [float_to_str(v) for v in ordered_values]
    base_message = f"1,2,6,{id},0," + ",".join(value_strs)+",0,0,0"
    crc_hex = compute_nmea_crc(base_message)