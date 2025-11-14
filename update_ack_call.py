file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到并替换启动ACK timer的代码
for i, line in enumerate(lines):
    if 'QTimer.singleShot(0, self._start_ack_timer)' in line:
        # 替换这一行及其注释
        indent = '                            '
        # 删除当前行及其上一行注释
        new_lines = [
            indent + '# 启动 ACK 定时器,每 500ms 发送 ACK (必须在主线程中调用)\n',
            indent + 'print("[FLASH][DBG] scheduling _start_ack_timer on main thread")\n',
            indent + 'def start_ack():\n',
            indent + '    print("[FLASH][DBG] executing _start_ack_timer on main thread")\n',
            indent + '    self._start_ack_timer()\n',
            indent + 'QTimer.singleShot(0, start_ack)\n',
        ]
        # 检查上一行是否是注释
        if i > 0 and '启动 ACK' in lines[i-1]:
            lines[i-1:i+1] = new_lines
        else:
            lines[i:i+1] = new_lines
        break

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print('Updated _start_ack_timer call with debug info')
