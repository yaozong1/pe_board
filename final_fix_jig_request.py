"""
最终修复脚本：添加治具请求功能
彻底清理并重新添加正确的代码
"""
import sys
import os

filepath = 'tools/factory_gui/factory_gui.py'

print(f"Reading {filepath}...")
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

# 备份
backup_path = filepath + '.backup_before_final_fix'
with open(backup_path, 'w', encoding='utf-8') as f:
    f.write(content)
print(f"Backup saved to {backup_path}")

# 1. 确保初始化中有 _requesting_jig_data
if '_requesting_jig_data = False' not in content:
    print("ERROR: Cannot find _requesting_jig_data initialization!")
    sys.exit(1)
else:
    print("✓ Found _requesting_jig_data initialization")

# 2. 找到并检查 flash 成功后的代码块
search_marker = 'print("[FLASH][DBG] set _awaiting_result=True, ACK will be sent every 500ms")'
if search_marker not in content:
    print(f"ERROR: Cannot find marker: {search_marker}")
    sys.exit(1)

# 检查是否已经有 _requesting_jig_data = True
if 'self._requesting_jig_data = True' in content:
    # 检查是否在正确的位置（在 _awaiting_result 之后）
    idx_ack = content.find(search_marker)
    idx_jig = content.find('self._requesting_jig_data = True')
    if idx_jig > idx_ack and idx_jig - idx_ack < 500:
        print("✓ _requesting_jig_data = True already exists in correct position")
    else:
        print("WARNING: _requesting_jig_data = True exists but in wrong position")
else:
    print("ERROR: _requesting_jig_data = True NOT FOUND!")
    # 添加它
    old_block = '''                            print("[FLASH][DBG] set _awaiting_result=True, ACK will be sent every 500ms")
                            break'''
    new_block = '''                            print("[FLASH][DBG] set _awaiting_result=True, ACK will be sent every 500ms")
                            # 同时向治具发送请求,让治具发送测试 payload
                            self._requesting_jig_data = True
                            print("[FLASH][DBG] set _requesting_jig_data=True, will request jig data every 500ms")
                            break'''
    
    if old_block in content:
        content = content.replace(old_block, new_block)
        print("✓ Added _requesting_jig_data = True")
    else:
        print("ERROR: Cannot find insertion point!")
        sys.exit(1)

# 3. 找到 on_tick 中的 ACK 发送逻辑，在其后添加 JIG 请求逻辑
ack_block_marker = '# 如果在等待自测结果,每 500ms 发送 ACK'
if ack_block_marker not in content:
    print(f"ERROR: Cannot find ACK block marker")
    sys.exit(1)
else:
    print("✓ Found ACK block")

# 查找 ACK 块后面是否已经有 JIG 请求块
jig_block_marker = '# 如果需要向治具请求测试数据'
if jig_block_marker in content:
    print("✓ JIG request block already exists")
else:
    print("Adding JIG request block...")
    # 找到 ACK 块的结束位置（下一个空行后）
    # 在 flash_serial 的 ACK 发送后添加 ctrl_serial 的请求发送
    
    # 查找插入点：ACK 块后的位置
    lines = content.split('\n')
    insert_line_idx = -1
    
    for i, line in enumerate(lines):
        if '# 如果在等待自测结果,每 500ms 发送 ACK' in line:
            # 找到这个块后面的第一个空行
            for j in range(i+1, min(i+20, len(lines))):
                if lines[j].strip() == '' and j+1 < len(lines):
                    # 检查下一行是否是有效代码开始
                    next_line = lines[j+1].strip()
                    if next_line and not next_line.startswith('#'):
                        insert_line_idx = j + 1
                        break
            break
    
    if insert_line_idx == -1:
        print("ERROR: Cannot find insertion point for JIG block")
        sys.exit(1)
    
    # 插入 JIG 请求块
    jig_request_code = '''        # 如果需要向治具请求测试数据，每 500ms 发送请求（和ACK逻辑完全一样）
        if self._requesting_jig_data and self.ctrl_serial.ser and self.ctrl_serial.ser.is_open:
            if not hasattr(self, "_last_jig_req_ms"):
                self._last_jig_req_ms = 0
            now_ms = int(time.time() * 1000)
            if now_ms - self._last_jig_req_ms >= 500:
                try:
                    self.ctrl_serial.ser.write(b"!GUI_REQUEST_DATA\\n")
                    self.ctrl_serial.ser.flush()
                    self._append_log("[JIG] Sent: !GUI_REQUEST_DATA\\n")
                    print("[JIG][DBG] Sent !GUI_REQUEST_DATA to ctrl_serial")
                    self._last_jig_req_ms = now_ms
                except Exception as e:
                    self._append_log(f"[JIG] Request failed: {e}\\n")
                    print(f"[JIG][DBG] Send failed: {e}")
        elif self._requesting_jig_data:
            # 调试：为什么没有发送
            if not hasattr(self, '_jig_nosend_counter'):
                self._jig_nosend_counter = 0
            self._jig_nosend_counter += 1
            if self._jig_nosend_counter == 1:
                ctrl_open = self.ctrl_serial.ser is not None and self.ctrl_serial.ser.is_open if self.ctrl_serial.ser else False
                print(f"[JIG][DBG] Not sending: ctrl_serial.ser={self.ctrl_serial.ser is not None}, is_open={ctrl_open}")

'''
    
    lines.insert(insert_line_idx, jig_request_code)
    content = '\n'.join(lines)
    print("✓ Inserted JIG request block")

# 4. 写回文件
print(f"\nWriting modified content back to {filepath}...")
with open(filepath, 'w', encoding='utf-8') as f:
    f.write(content)

print("\n" + "="*80)
print("FINAL FIX COMPLETED!")
print("="*80)
print("\nChanges made:")
print("1. ✓ Ensured _requesting_jig_data = True is set after flash success")
print("2. ✓ Added JIG request sending in on_tick (every 500ms, same as ACK)")
print("\nNext steps:")
print("1. Close any running factory_gui.py processes")
print("2. Run: python tools\\factory_gui\\factory_gui.py")
print("3. Look for these messages in console:")
print("   - [FLASH][DBG] set _requesting_jig_data=True, will request jig data every 500ms")
print("   - [JIG][DBG] Sent !GUI_REQUEST_DATA to ctrl_serial")
print("="*80)
