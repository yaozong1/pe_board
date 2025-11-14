"""
添加独立的停止逻辑：收到治具回复后停止发送 !GUI_REQUEST_DATA
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到 _parse_voltage_adc 方法中 if im_tested 的位置
modified = False
for i, line in enumerate(lines):
    # 找到 IM测试结果解析后的位置
    if 'if im_tested:' in line and i > 800 and i < 900:
        # 检查下一行是否是 self.lbl_im.set_state
        if i+1 < len(lines) and 'self.lbl_im.set_state' in lines[i+1]:
            # 在这之后添加停止逻辑
            # 找到这个if块后的空行
            insert_idx = i + 2  # self.lbl_im.set_state(im_pass) 的下一行
            
            # 插入新代码
            new_code = '''            
            # 收到治具的完整payload后，停止发送 !GUI_REQUEST_DATA
            if self._requesting_jig_data:
                self._requesting_jig_data = False
                print("[JIG][DBG] Received payload, stopped sending !GUI_REQUEST_DATA")
                self._append_log("[JIG] Payload received, stopped requesting\\n")
'''
            lines.insert(insert_idx, new_code)
            modified = True
            print(f"✓ Inserted stop logic after line {insert_idx}")
            print(f"  Context:")
            for j in range(max(0, i-1), min(len(lines), insert_idx+2)):
                print(f"    {j+1}: {lines[j].rstrip()}")
            break

if not modified:
    print("✗ Failed to find insertion point")
    import sys
    sys.exit(1)

# 写回文件
with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("\n" + "="*80)
print("修改完成！")
print("="*80)
print("\n现在 !GUI_REQUEST_DATA 会:")
print("1. 在 flash 成功后开始每 500ms 发送一次")
print("2. 收到治具的 VOLTAGE_ADC payload 后自动停止")
print("3. 独立于 !GUI_SELFTEST_ACK 的停止逻辑")
print("="*80)
