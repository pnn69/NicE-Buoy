import json
import math

def check_linearity(filename):
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        data = [json.loads(line) for line in lines if line.strip()]
        if not data: return
        
        # We assume Y and Z are the horizontal axes
        # and atan2(mz, my) is the heading
        headings = []
        for d in data:
            my = d['m'][1]
            mz = d['m'][2]
            # Simple hard-iron center from previous run: Y=-5.30, Z=6.05
            h = math.atan2(mz - 6.05, my - (-5.30)) * 180.0 / math.pi
            if h < 0: h += 360
            headings.append(h)
        
        # Check for jumps or non-linear regions
        # Since we don't know the "true" angle, we look at the distribution
        # If it was rotating at a constant speed, the histogram should be flat.
        
        # Let's just look at the headings
        print(f"Heading range: {min(headings):.1f} to {max(headings):.1f}")
        
        # Sort headings to see coverage
        sorted_h = sorted(headings)
        # Check gaps
        max_gap = 0
        for i in range(len(sorted_h)-1):
            max_gap = max(max_gap, sorted_h[i+1] - sorted_h[i])
        print(f"Max gap: {max_gap:.2f} degrees")

    except Exception as e:
        print(f"Error: {e}")

check_linearity('diag_p1_horizontal.txt')
