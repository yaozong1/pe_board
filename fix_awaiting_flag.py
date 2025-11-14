file_path = r'c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui\factory_gui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 替换flash成功后的代码,添加_awaiting_result标志设置
old_code = '''                            print(f"[FLASH][DBG] flash port reopened successfully on attempt {i+1}")
                            # 启动 ACK 定时器,每 500ms 发送 ACK (必须在主线程中调用)
                            print("[FLASH][DBG] scheduling _start_ack_timer on main thread")
                            def start_ack():
                                print("[FLASH][DBG] executing _start_ack_timer on main thread")
                                self._start_ack_timer()
                            QTimer.singleShot(0, start_ack)'''

new_code = '''                            print(f"[FLASH][DBG] flash port reopened successfully on attempt {i+1}")
                            # 设置等待标志,触发 on_tick 中的 ACK 发送
                            self._awaiting_result = True
                            self._await_deadline_ms = int(time.time() * 1000) + 40_000
                            print("[FLASH][DBG] set _awaiting_result=True, ACK will be sent every 500ms")'''

if old_code in content:
    content = content.replace(old_code, new_code)
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print('Successfully set _awaiting_result flag after flash port reopen')
else:
    print('Error: Could not find the code to replace')
    print('Trying alternative approach...')
    
    # 尝试只替换关键部分
    if 'scheduling _start_ack_timer on main thread' in content:
        # 找到这行并替换整个区块
        lines = content.split('\n')
        new_lines = []
        i = 0
        while i < len(lines):
            if 'scheduling _start_ack_timer on main thread' in lines[i]:
                # 跳过旧的代码块(到QTimer.singleShot那一行)
                new_lines.append('                            # 设置等待标志,触发 on_tick 中的 ACK 发送')
                new_lines.append('                            self._awaiting_result = True')
                new_lines.append('                            self._await_deadline_ms = int(time.time() * 1000) + 40_000')
                new_lines.append('                            print("[FLASH][DBG] set _awaiting_result=True, ACK will be sent every 500ms")')
                # 跳过旧的6行(注释+print+def+print+调用+QTimer)
                while i < len(lines) and 'QTimer.singleShot' not in lines[i]:
                    i += 1
                i += 1  # 跳过QTimer.singleShot那一行
            else:
                new_lines.append(lines[i])
                i += 1
        
        content = '\n'.join(new_lines)
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print('Successfully updated via alternative approach')
