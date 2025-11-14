"""
清理并添加治具请求功能 - 使用和 ACK 完全相同的逻辑
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 步骤1: 找到并替换 on_tick 中的治具请求代码块
# 找到 "# flash等待结束后只发送一次 !GUI_REQUEST_DATA" 这一行
found_old_block = False
new_lines = []
skip_until_blank = False

for i, line in enumerate(lines):
    # 跳过旧的治具请求代码块
    if "# flash等待结束后只发送一次 !GUI_REQUEST_DATA" in line:
        found_old_block = True
        skip_until_blank = True
        # 插入新的代码块（和ACK逻辑完全一样）
        new_lines.append("        # 如果需要向治具请求测试数据,每 500ms 发送请求（和ACK逻辑完全一样）\n")
        new_lines.append("        if self._requesting_jig_data and self.ctrl_serial.ser and self.ctrl_serial.ser.is_open:\n")
        new_lines.append("            if not hasattr(self, \"_last_jig_req_ms\"):\n")
        new_lines.append("                self._last_jig_req_ms = 0\n")
        new_lines.append("            now_ms = int(time.time() * 1000)\n")
        new_lines.append("            if now_ms - self._last_jig_req_ms >= 500:\n")
        new_lines.append("                try:\n")
        new_lines.append("                    self.ctrl_serial.ser.write(b\"!GUI_REQUEST_DATA\\n\")\n")
        new_lines.append("                    self.ctrl_serial.ser.flush()\n")
        new_lines.append("                    self._append_log(\"[JIG] Sent: !GUI_REQUEST_DATA\\n\")\n")
        new_lines.append("                    self._last_jig_req_ms = now_ms\n")
        new_lines.append("                except Exception as e:\n")
        new_lines.append("                    self._append_log(f\"[JIG] Request failed: {e}\\n\")\n")
        new_lines.append("\n")
        continue
    
    # 跳过旧代码块直到遇到空行后的有效代码
    if skip_until_blank:
        # 检查是否到达代码块结束（两个连续的空行或者新的代码段）
        if line.strip() == "" and i + 1 < len(lines) and lines[i + 1].strip().startswith("chunk ="):
            skip_until_blank = False
            # 不添加这个空行，让新代码块自己的空行生效
            continue
        else:
            continue
    
    new_lines.append(line)

if not found_old_block:
    print("WARNING: Old block not found, code may not have the expected structure")

# 写回文件
with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print(f"✓ Successfully updated on_tick method")
print(f"  Lines before: {len(lines)}")
print(f"  Lines after: {len(new_lines)}")
print(f"  Old block found: {found_old_block}")
