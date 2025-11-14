"""
Update GUI to parse and display IMEI and ICCID
"""

import re

# Read the file
with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Step 1: Update JSON parsing to extract IMEI and ICCID
old_parse = '''        res.eg915_ok = bool(data.get("eg915_ok", False))
        m = data.get("motion", {}) or {}'''

new_parse = '''        res.eg915_ok = bool(data.get("eg915_ok", False))
        res.eg915_imei = str(data.get("eg915_imei", ""))
        res.eg915_iccid = str(data.get("eg915_iccid", ""))
        m = data.get("motion", {}) or {}'''

if old_parse in content:
    content = content.replace(old_parse, new_parse)
    print("Step 1: Updated JSON parsing for IMEI/ICCID")
else:
    print("ERROR: Could not find JSON parsing code")
    import sys
    sys.exit(1)

# Save the file
with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("\nGUI parsing updated successfully!")
print("\nNext steps:")
print("1. Add UI labels to display IMEI and ICCID")
print("2. Update the display logic to show these values")
