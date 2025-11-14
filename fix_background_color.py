"""
修改背景色恢复逻辑：使用默认背景色而不是白色
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    content = f.read()

# 替换白色背景为默认背景
old_line = 'self.volt_table.item(i, 2).setBackground(QColor(255, 255, 255))  # 白色背景'
new_line = 'self.volt_table.item(i, 2).setBackground(QColor())  # 恢复默认背景色'

if old_line in content:
    content = content.replace(old_line, new_line)
    print("✓ Replaced background color setting")
else:
    print("✗ Cannot find old line")
    import sys
    sys.exit(1)

with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("\n" + "="*80)
print("修改完成！")
print("="*80)
print("现在清空表格时会恢复为GUI启动时的原始背景色（系统默认）")
print("而不是强制设置为白色")
print("="*80)
