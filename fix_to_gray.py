"""
修改背景色为浅灰色（和N/A状态一样）
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    content = f.read()

# 替换为浅灰色
old_line = 'self.volt_table.item(i, 2).setBackground(QColor())  # 恢复默认背景色'
new_line = 'self.volt_table.item(i, 2).setBackground(QColor(211, 211, 211))  # 浅灰色（和N/A一样）'

if old_line in content:
    content = content.replace(old_line, new_line)
    print("✓ Changed to light gray color (211, 211, 211)")
else:
    print("✗ Cannot find line to replace")
    import sys
    sys.exit(1)

with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("\n" + "="*80)
print("修改完成！")
print("="*80)
print("现在清空表格时 Status 列的背景色为:")
print("  浅灰色 RGB(211, 211, 211) - 和 N/A 状态显示的颜色一样")
print("  与左边 Voltage 列的默认灰色背景一致")
print("="*80)
