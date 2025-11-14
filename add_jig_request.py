lines = open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8').readlines()file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'

with open(file_path, 'r', encoding='utf-8') as f:

# 在第1178行（break之前）插入3行代码    lines = f.readlines()

insert_lines = [

    '                            # 同时向治具发送请求,让治具发送测试 payload\n',# 找到向DUT发送ACK的代码块,在其后添加向治具发送请求的代码

    '                            self._requesting_jig_data = True\n',for i, line in enumerate(lines):

    '                            print("[FLASH][DBG] set _requesting_jig_data=True, will request jig data")\n',    if '[ACK] Send failed: {e}' in line and 'self._append_log' in line:

]        # 在这个except块后添加治具请求逻辑

        # 找到这个except块的结束(下一个非缩进或同级别的代码)

# 插入位置：第1177行之后（0-indexed为1176）        indent = '        '

lines[1177:1177] = insert_lines        new_lines = [

            '\n',

open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8').writelines(lines)            indent + '# 如果需要向治具请求测试数据,每 500ms 发送一次请求\n',

print('Successfully inserted _requesting_jig_data=True before break statement')            indent + 'if self._requesting_jig_data and self.ctrl_serial.ser and self.ctrl_serial.ser.is_open:\n',

            indent + '    if not hasattr(self, "_last_jig_req_ms"):\n',
            indent + '        self._last_jig_req_ms = 0\n',
            indent + '    now_ms = int(time.time() * 1000)\n',
            indent + '    if now_ms - self._last_jig_req_ms >= 500:\n',
            indent + '        try:\n',
            indent + '            self.ctrl_serial.ser.write(b"!GUI_REQUEST_DATA\\n")\n',
            indent + '            self.ctrl_serial.ser.flush()\n',
            indent + '            self._append_log("[JIG] Sent: !GUI_REQUEST_DATA\\n")\n',
            indent + '            self._last_jig_req_ms = now_ms\n',
            indent + '        except Exception as e:\n',
            indent + '            self._append_log(f"[JIG] Request failed: {e}\\n")\n',
        ]
        lines[i+1:i+1] = new_lines
        break

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print('Added jig data request logic in on_tick')
