file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 在 _send_ack 方法之前添加 _start_ack_timer 和 _stop_ack_timer 方法
insert_pos = content.find('    def _send_ack(self):')
if insert_pos == -1:
    print('Error: Could not find _send_ack method')
    exit(1)

new_methods = """    def _start_ack_timer(self):
        \"\"\"启动ACK定时器\"\"\"
        if not self._ack_running:
            self._ack_running = True
            self.ack_timer.start()
            self._append_log('[ACK] ACK timer started (500ms interval)\\n')
            print('[ACK][DBG] ACK timer started')

    def _stop_ack_timer(self):
        \"\"\"停止ACK定时器\"\"\"
        if self._ack_running:
            self.ack_timer.stop()
            self._ack_running = False
            self._append_log('[ACK] ACK timer stopped\\n')
            print('[ACK][DBG] ACK timer stopped')

"""

new_content = content[:insert_pos] + new_methods + content[insert_pos:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(new_content)

print('ACK timer control methods added successfully')
