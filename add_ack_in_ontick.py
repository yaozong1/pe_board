file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到 on_tick 方法中读取flash_serial之前的位置
insert_pos = None
for i, line in enumerate(lines):
    if 'flash_serial.read_lines()' in line and 'chunk = self.flash_serial.read_lines()' in line:
        # 在这一行之前插入ACK发送逻辑
        insert_pos = i
        break

if insert_pos:
    indent = '        '
    new_code = [
        '\n',
        indent + '# 如果在等待自测结果,每 500ms 发送 ACK\n',
        indent + 'if self._awaiting_result and self.flash_serial.ser and self.flash_serial.ser.is_open:\n',
        indent + '    if not hasattr(self, "_last_ack_ms"):\n',
        indent + '        self._last_ack_ms = 0\n',
        indent + '    now_ms = int(time.time() * 1000)\n',
        indent + '    if now_ms - self._last_ack_ms >= 500:\n',
        indent + '        try:\n',
        indent + '            self.flash_serial.ser.write(b"!GUI_SELFTEST_ACK\\n")\n',
        indent + '            self.flash_serial.ser.flush()\n',
        indent + '            self._append_log("[ACK] Sent: !GUI_SELFTEST_ACK\\n")\n',
        indent + '            self._last_ack_ms = now_ms\n',
        indent + '        except Exception as e:\n',
        indent + '            self._append_log(f"[ACK] Send failed: {e}\\n")\n',
        '\n',
    ]
    lines[insert_pos:insert_pos] = new_code
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    print(f'ACK logic added to on_tick at line {insert_pos}')
else:
    print('Error: Could not find insertion point')
