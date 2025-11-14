"""
在 Flash & Wait 时清空 Selftest Results 和 8-CH Voltage 显示
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到 _on_flash_and_wait 函数并修改
modified = False
in_function = False
function_start = -1

for i, line in enumerate(lines):
    if 'def _on_flash_and_wait(self):' in line:
        in_function = True
        function_start = i
        continue
    
    # 找到第一次出现的 self._append_log("[FLASH] === New test round started ===
    if in_function and '_append_log("[FLASH] === New test round started ===' in line:
        # 删除从函数开始到这一行的所有重复代码
        # 重新构建这个函数的开头部分
        
        # 保存函数定义和文档字符串
        new_function = []
        new_function.append(lines[function_start])  # def _on_flash_and_wait(self):
        new_function.append(lines[function_start + 1])  # 文档字符串
        
        # 添加清空逻辑
        new_function.append('''        # 清空上一轮测试的标志和状态（新一轮测试开始）
        self._awaiting_result = False
        self._requesting_jig_data = False
        if hasattr(self, '_last_ack_ms'):
            delattr(self, '_last_ack_ms')
        if hasattr(self, '_last_jig_req_ms'):
            delattr(self, '_last_jig_req_ms')
        if hasattr(self, '_jig_nosend_counter'):
            delattr(self, '_jig_nosend_counter')
        
        # 清空界面上的 Selftest Results 显示
        self.lbl_overall.setText("-")
        self.lbl_overall.setStyleSheet("")
        self.lbl_eg915.setText("-")
        self.lbl_eg915.setStyleSheet("")
        self.lbl_motion.setText("-")
        self.lbl_motion.setStyleSheet("")
        self.lbl_motion_mag.setText("mag=0.000")
        self.lbl_rs485.setText("-")
        self.lbl_rs485.setStyleSheet("")
        self.lbl_rs485_info.setText("-")
        self.lbl_can.setText("-")
        self.lbl_can.setStyleSheet("")
        self.lbl_can_info.setText("-")
        self.lbl_gnss.setText("-")
        self.lbl_gnss.setStyleSheet("")
        self.lbl_gnss_info.setText("-")
        self.lbl_bat.setText("-")
        self.lbl_bat.setStyleSheet("")
        self.lbl_bat_v.setText("V=0.00V")
        self.lbl_ign.setText("-")
        self.lbl_ign.setStyleSheet("")
        self.lbl_im.setText("-")
        self.lbl_im.setStyleSheet("")
        
        # 清空 8-CH Voltage 表格
        for i in range(8):
            self.volt_table.item(i, 1).setText("-")  # Voltage列
            self.volt_table.item(i, 2).setText("-")  # Status列
            self.volt_table.item(i, 2).setBackground(QColor(255, 255, 255))  # 白色背景
        
        print("[FLASH][DBG] Cleared previous test results and flags for new round")
        self._append_log("[FLASH] === New test round started ===\\n")
        
''')
        
        # 找到这个重复块的结束位置（下一个有效代码行）
        skip_until = i + 1
        while skip_until < len(lines):
            stripped = lines[skip_until].strip()
            # 跳过空行和重复的清空代码
            if not stripped or 'self._awaiting_result = False' in stripped or \
               'self._requesting_jig_data = False' in stripped or \
               'delattr(self' in stripped or \
               'print("[FLASH][DBG] Cleared' in stripped or \
               '_append_log("[FLASH] === New test round' in stripped:
                skip_until += 1
            else:
                break
        
        # 构建新的文件内容
        result = lines[:function_start] + new_function + lines[skip_until:]
        
        modified = True
        print(f"✓ Replaced function from line {function_start+1} to {skip_until+1}")
        print(f"  Removed {skip_until - function_start - len(new_function)} duplicate lines")
        break

if not modified:
    print("✗ Failed to find function")
    import sys
    sys.exit(1)

# 写回文件
with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.writelines(result)

print("\n" + "="*80)
print("修改完成！")
print("="*80)
print("\n现在每次点击 'Flash & Wait' 时会清空:")
print("1. Selftest Results 所有测试项的显示（恢复为 '-'）")
print("2. 8-CH Voltage 表格的电压值和状态")
print("3. 上一轮的测试标志和计时器")
print("\n界面将完全重置，准备新一轮测试！")
print("="*80)
