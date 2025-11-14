file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到设置_awaiting_result的位置,在后面添加治具请求标志
for i, line in enumerate(lines):
    if 'set _awaiting_result=True, ACK will be sent every 500ms' in line:
        # 在这一行后添加治具请求标志
        indent = '                            '
        new_lines = [
            indent + '# 同时向治具发送请求,让治具发送测试 payload\n',
            indent + 'self._requesting_jig_data = True\n',
            indent + 'print("[FLASH][DBG] set _requesting_jig_data=True, will request jig data every 500ms")\n',
        ]
        lines[i+1:i+1] = new_lines
        break

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print('Added jig request flag')
