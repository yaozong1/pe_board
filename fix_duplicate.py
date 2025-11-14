"""
彻底清理重复代码
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到 _on_flash_and_wait 并彻底重写
for i, line in enumerate(lines):
    if 'def _on_flash_and_wait(self):' in line:
        # 找到下一个函数定义的位置
        next_func = i + 1
        while next_func < len(lines) and not (lines[next_func].strip().startswith('def ') and not lines[next_func].strip().startswith('def _on_flash_and_wait')):
            next_func += 1
        
        # 找到 flash_port = self.flash_combo.currentText() 这一行
        flash_port_line = -1
        for j in range(i, next_func):
            if 'flash_port = self.flash_combo.currentText()' in lines[j]:
                flash_port_line = j
                break
        
        if flash_port_line == -1:
            print("ERROR: Cannot find flash_port line")
            import sys
            sys.exit(1)
        
        # 重新构建函数
        new_lines = lines[:i]  # 前面的内容
        
        # 添加新的函数开头
        new_lines.append('    def _on_flash_and_wait(self):\n')
        new_lines.append('        """极简版：COM24保持打开，只断开/重连COM6"""\n')
        new_lines.append('        # 清空上一轮测试的标志和状态（新一轮测试开始）\n')
        new_lines.append('        self._awaiting_result = False\n')
        new_lines.append('        self._requesting_jig_data = False\n')
        new_lines.append('        if hasattr(self, \'_last_ack_ms\'):\n')
        new_lines.append('            delattr(self, \'_last_ack_ms\')\n')
        new_lines.append('        if hasattr(self, \'_last_jig_req_ms\'):\n')
        new_lines.append('            delattr(self, \'_last_jig_req_ms\')\n')
        new_lines.append('        if hasattr(self, \'_jig_nosend_counter\'):\n')
        new_lines.append('            delattr(self, \'_jig_nosend_counter\')\n')
        new_lines.append('        \n')
        new_lines.append('        # 清空界面上的 Selftest Results 显示\n')
        new_lines.append('        self.lbl_overall.setText("-")\n')
        new_lines.append('        self.lbl_overall.setStyleSheet("")\n')
        new_lines.append('        self.lbl_eg915.setText("-")\n')
        new_lines.append('        self.lbl_eg915.setStyleSheet("")\n')
        new_lines.append('        self.lbl_motion.setText("-")\n')
        new_lines.append('        self.lbl_motion.setStyleSheet("")\n')
        new_lines.append('        self.lbl_motion_mag.setText("mag=0.000")\n')
        new_lines.append('        self.lbl_rs485.setText("-")\n')
        new_lines.append('        self.lbl_rs485.setStyleSheet("")\n')
        new_lines.append('        self.lbl_rs485_info.setText("-")\n')
        new_lines.append('        self.lbl_can.setText("-")\n')
        new_lines.append('        self.lbl_can.setStyleSheet("")\n')
        new_lines.append('        self.lbl_can_info.setText("-")\n')
        new_lines.append('        self.lbl_gnss.setText("-")\n')
        new_lines.append('        self.lbl_gnss.setStyleSheet("")\n')
        new_lines.append('        self.lbl_gnss_info.setText("-")\n')
        new_lines.append('        self.lbl_bat.setText("-")\n')
        new_lines.append('        self.lbl_bat.setStyleSheet("")\n')
        new_lines.append('        self.lbl_bat_v.setText("V=0.00V")\n')
        new_lines.append('        self.lbl_ign.setText("-")\n')
        new_lines.append('        self.lbl_ign.setStyleSheet("")\n')
        new_lines.append('        self.lbl_im.setText("-")\n')
        new_lines.append('        self.lbl_im.setStyleSheet("")\n')
        new_lines.append('        \n')
        new_lines.append('        # 清空 8-CH Voltage 表格\n')
        new_lines.append('        for i in range(8):\n')
        new_lines.append('            self.volt_table.item(i, 1).setText("-")  # Voltage列\n')
        new_lines.append('            self.volt_table.item(i, 2).setText("-")  # Status列\n')
        new_lines.append('            self.volt_table.item(i, 2).setBackground(QColor(255, 255, 255))  # 白色背景\n')
        new_lines.append('        \n')
        new_lines.append('        print("[FLASH][DBG] Cleared previous test results and flags for new round")\n')
        new_lines.append('        self._append_log("[FLASH] === New test round started ===\\n")\n')
        new_lines.append('        \n')
        
        # 添加从 flash_port 开始的剩余代码
        new_lines.extend(lines[flash_port_line:])
        
        # 写回
        with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
            f.writelines(new_lines)
        
        print("✓ Successfully rewrote _on_flash_and_wait function")
        print(f"  Removed duplicate code")
        print(f"  Total lines: {len(lines)} -> {len(new_lines)}")
        break
else:
    print("ERROR: Cannot find function")
    import sys
    sys.exit(1)

print("\n" + "="*80)
print("完成！现在函数干净整洁，包含:")
print("1. 清空测试标志")
print("2. 清空 Selftest Results 显示")
print("3. 清空 8-CH Voltage 表格")
print("="*80)
