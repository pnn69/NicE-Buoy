import json
import math

def check_circle(filename):
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        data = [json.loads(line) for line in lines if line.strip()]
        if not data: return
        
        # We assume Y and Z are the horizontal axes based on previous analysis
        ys = [d['m'][1] for d in data]
        zs = [d['m'][2] for d in data]
        
        # Simple center estimate
        cy = (min(ys) + max(ys)) / 2
        cz = (min(zs) + max(zs)) / 2
        
        print(f"Center estimate: Y={cy:.2f}, Z={cz:.2f}")
        
        # Calculate radius for each point
        radii = [math.sqrt((y - cy)**2 + (z - cz)**2) for y, z in zip(ys, zs)]
        
        avg_r = sum(radii) / len(radii)
        min_r = min(radii)
        max_r = max(radii)
        std_r = math.sqrt(sum((r - avg_r)**2 for r in radii) / len(radii))
        
        print(f"Radius: Avg={avg_r:.2f}, Min={min_r:.2f}, Max={max_r:.2f}, Std={std_r:.2f}")
        print(f"Non-linearity (Std/Avg): {std_r/avg_r:.1%}")

    except Exception as e:
        print(f"Error: {e}")

print("--- Phase 1 (Horizontal Rotation) ---")
check_circle('diag_p1_horizontal.txt')
