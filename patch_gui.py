"""
重新创建 Status 列的表格项，恢复默认背景色
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到清空表格的代码块并替换
modified = False
for i, line in enumerate(lines):
    if '# 清空 8-CH Voltage 表格' in line:
        # 找到这个 for 循环的范围
        # 替换接下来的几行
        if i+4 < len(lines) and 'setBackground(QColor(211, 211, 211))' in lines[i+4]:
            # 重新构建这个代码块
            new_block = [
                '        # 清空 8-CH Voltage 表格\n',
                '        for i in range(8):\n',
                '            self.volt_table.item(i, 1).setText("-")  # Voltage列\n',
                '            # 重新创建 Status 列的项，恢复默认背景色（和 Voltage 列一样的深灰色）\n',
                '            self.volt_table.setItem(i, 2, QTableWidgetItem("-"))\n',
                '        \n'
            ]
            
            # 替换从注释行到 setBackground 这一行（共5行）
            lines[i:i+6] = new_block
            modified = True
            print(f"✓ Replaced lines {i+1} to {i+6}")
            break

if not modified:
    print("✗ Failed to find code to replace")
    import sys
    sys.exit(1)

with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("\n" + "="*80)
print("修改完成！")
print("="*80)
print("现在清空表格时会:")
print("1. Voltage 列：setText('-') - 保持原有的深灰色背景")
print("2. Status 列：重新创建 QTableWidgetItem('-') - 自动使用默认深灰色背景")
print("3. 两列的背景色完全一致（都是系统默认的深灰色）")
print("="*80)
