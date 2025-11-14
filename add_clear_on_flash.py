"""
在 Flash & Wait 按钮点击时清空上一轮测试的标志
"""

with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# 找到 _on_flash_and_wait 函数
modified = False
for i, line in enumerate(lines):
    if 'def _on_flash_and_wait(self):' in line:
        # 在函数的第一行（注释后）插入清空逻辑
        # 找到注释行
        insert_idx = i + 2  # def 行 + 注释行 + 插入位置
        
        clear_code = '''        # 清空上一轮测试的标志和状态（新一轮测试开始）
        self._awaiting_result = False
        self._requesting_jig_data = False
        if hasattr(self, '_last_ack_ms'):
            delattr(self, '_last_ack_ms')
        if hasattr(self, '_last_jig_req_ms'):
            delattr(self, '_last_jig_req_ms')
        if hasattr(self, '_jig_nosend_counter'):
            delattr(self, '_jig_nosend_counter')
        print("[FLASH][DBG] Cleared previous test flags for new round")
        self._append_log("[FLASH] === New test round started ===\\n")
        
'''
        
        lines.insert(insert_idx, clear_code)
        modified = True
        print(f"✓ Inserted clear logic at line {insert_idx}")
        print(f"  Context:")
        for j in range(i, min(len(lines), insert_idx+3)):
            print(f"    {j+1}: {lines[j].rstrip()}")
        break

if not modified:
    print("✗ Failed to find _on_flash_and_wait function")
    import sys
    sys.exit(1)

# 写回文件
with open('tools/factory_gui/factory_gui.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("\n" + "="*80)
print("修改完成！")
print("="*80)
print("\n现在每次点击 'Flash & Wait' 按钮时会:")
print("1. 停止上一轮的 ACK 发送 (_awaiting_result = False)")
print("2. 停止上一轮的治具请求 (_requesting_jig_data = False)")
print("3. 清空所有计时器和计数器")
print("4. 在日志中显示 '=== New test round started ==='")
print("\n这样每次刷机都是全新的测试周期！")
print("="*80)
