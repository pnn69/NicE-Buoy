import json
import math

def check_rotation():
    try:
        with open('diag_p1_horizontal.txt', 'r') as f:
            lines = f.readlines()
        data = [json.loads(l) for l in lines if l.strip()]
        
        # Check Mag Y and Mag Z during level rotation
        my = [d['m'][1] for d in data]
        mz = [d['m'][2] for d in data]
        
        # Calculate cross-correlation or just look at the phase
        # If clockwise rotation:
        # If Forward=Z, Right=Y, then MagZ is cos, MagY is -sin
        print("First 10 points (Mag Y, Mag Z):")
        for i in range(10):
            print(f"{my[i]:6.1f}, {mz[i]:6.1f}")
            
    except Exception as e:
        print(f"Error: {e}")

check_rotation()
