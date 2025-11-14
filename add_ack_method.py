file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 在 _append_log_async 之前添加 _send_ack 方法
insert_pos = content.find('    def _append_log_async(self, text: str):')
if insert_pos == -1:
    print('Error: Could not find _append_log_async method')
    exit(1)

new_method = """    def _send_ack(self):
        \"\"\"发送ACK到DUT\"\"\"
        if not self.flash_serial.ser or not self.flash_serial.ser.is_open:
            self._append_log('[ACK] Flash port closed, stopping ACK timer\\n')
            self.ack_timer.stop()
            self._ack_running = False
            return
        
        try:
            self.flash_serial.ser.write(b'!GUI_SELFTEST_ACK\\n')
            self.flash_serial.ser.flush()
            self._append_log('[ACK] Sent: !GUI_SELFTEST_ACK\\n')
        except Exception as e:
            self._append_log(f'[ACK] Send failed: {e}\\n')
            self.ack_timer.stop()
            self._ack_running = False

"""

new_content = content[:insert_pos] + new_method + content[insert_pos:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(new_content)

print('_send_ack method added successfully')
