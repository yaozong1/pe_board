"""
Add IMEI/ICCID display logic to GUI
"""

# Read the file
with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find the line with "self.lbl_eg915.set_state(res.eg915_ok)"
target_idx = None
for i, line in enumerate(lines):
    if 'self.lbl_eg915.set_state(res.eg915_ok)' in line:
        target_idx = i
        break

if target_idx is None:
    print("ERROR: Could not find self.lbl_eg915.set_state line")
    import sys
    sys.exit(1)

# Insert IMEI/ICCID update logic after the EG915 status update
insert_lines = [
    '        \n',
    '        # Update IMEI and ICCID labels\n',
    '        if res.eg915_imei:\n',
    '            self.lbl_imei.setText(f"IMEI: {res.eg915_imei}")\n',
    '        else:\n',
    '            self.lbl_imei.setText("IMEI: -")\n',
    '        \n',
    '        if res.eg915_iccid:\n',
    '            self.lbl_iccid.setText(f"ICCID: {res.eg915_iccid}")\n',
    '        else:\n',
    '            self.lbl_iccid.setText("ICCID: -")\n',
]

# Insert after the target line
lines = lines[:target_idx+1] + insert_lines + lines[target_idx+1:]

# Write back
with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print(f"Inserted IMEI/ICCID display logic after line {target_idx+1}")
print("GUI update complete!")
