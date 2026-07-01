import websocket
import threading
import time
import sys

# Target ESP32 WebSocket server
WS_URL = "ws://192.168.1.166:81"

buoy_id = None
received_packets = []

def calculate_crc(content):
    crc = 0
    for char in content:
        crc ^= ord(char)
    return crc

def make_command(target_id, cmd_id):
    payload = f"{target_id},99,3,{cmd_id},{cmd_id}"
    crc = calculate_crc(payload)
    return f"${payload}*{crc:02X}\r\n"

def on_message(ws, message):
    global buoy_id, received_packets
    msg = message.strip()
    print(f"[RECV] {msg}")
    
    if msg.startswith("$") and "*" in msg:
        content = msg[1:].split("*")[0]
        fields = content.split(",")
        if len(fields) >= 5:
            target = fields[0].lower()
            sender = fields[1].lower()
            cmd = int(fields[3])
            status = int(fields[4])
            
            if sender != "99" and buoy_id is None:
                buoy_id = sender
                print(f"\n🌟 DISCOVERED ACTIVE BUOY ID: {buoy_id}")

def on_error(ws, error):
    print(f"[ERROR] {error}")

def on_close(ws, close_status_code, close_msg):
    print("[CLOSED] Connection closed.")

def on_open(ws):
    print("[OPEN] Connected successfully to LoraController WebSocket!")
    
    # Run a test thread to execute scenarios
    def test_scenarios():
        global buoy_id
        print("⏳ Waiting for a buoy telemetry packet to discover ID...")
        start = time.time()
        while buoy_id is None:
            if time.time() - start > 10:
                print("❌ Timeout waiting for buoy telemetry. Is the buoy turned on?")
                ws.close()
                return
            time.sleep(0.5)
            
        print(f"\n--- STARTING SCENARIO TESTS ON BUOY {buoy_id} ---")
        
        # Scenario 1: Send LOCKING command
        print("\n🔒 Sending LOCK command with Target ID '1'...")
        lock_cmd = make_command("1", 12) # MsgType.LOCKING = 12
        print(f"[SEND] {lock_cmd.strip()}")
        ws.send(lock_cmd)
        
        print("⏳ Listening for 5 seconds to observe buoy response/OLED screen updates...")
        time.sleep(5)
        
        # Scenario 2: Send IDLE command to restore
        print("\n💤 Sending IDLE command with Target ID '1'...")
        idle_cmd = make_command("1", 8) # MsgType.IDELING = 8
        print(f"[SEND] {idle_cmd.strip()}")
        ws.send(idle_cmd)
        
        print("⏳ Listening for 3 seconds...")
        time.sleep(3)
        
        print("\n--- TEST SCENARIO COMPLETION REPORT ---")
        print("✅ Tested LOCK.")
        print("✅ Tested IDLE.")
        ws.close()
        
    threading.Thread(target=test_scenarios, daemon=True).start()

if __name__ == "__main__":
    print(f"Connecting to {WS_URL}...")
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(WS_URL,
                              on_open=on_open,
                              on_message=on_message,
                              on_error=on_error,
                              on_close=on_close)
    ws.run_forever()
