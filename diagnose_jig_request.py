"""
诊断脚本：检查为什么 !GUI_REQUEST_DATA 没有发送
"""

# 读取文件
with open('tools/factory_gui/factory_gui.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

print("="*80)
print("诊断报告：检查治具请求功能")
print("="*80)

# 1. 检查初始化
print("\n1. 检查 __init__ 中的初始化:")
for i, line in enumerate(lines, 1):
    if '_requesting_jig_data = False' in line and 190 < i < 200:
        print(f"   ✓ Line {i}: {line.strip()}")
        break
else:
    print("   ✗ 未找到初始化")

# 2. 检查flash成功后是否设置标志
print("\n2. 检查 flash 成功后是否设置 _requesting_jig_data = True:")
found = False
for i, line in enumerate(lines, 1):
    if 'self._requesting_jig_data = True' in line and 1200 < i < 1220:
        print(f"   ✓ Line {i}: {line.strip()}")
        # 检查前后3行
        print(f"   上下文:")
        for j in range(max(0, i-3), min(len(lines), i+2)):
            prefix = "   >>>" if j == i-1 else "      "
            print(f"{prefix} {j+1}: {lines[j].rstrip()}")
        found = True
        break

if not found:
    print("   ✗ 未找到设置语句")

# 3. 检查 on_tick 中的发送逻辑
print("\n3. 检查 on_tick 中的治具请求发送逻辑:")
found = False
for i, line in enumerate(lines, 1):
    if '如果需要向治具请求测试数据' in line:
        print(f"   ✓ Line {i}: 找到注释: {line.strip()}")
        # 显示接下来的15行
        print(f"   代码块:")
        for j in range(i-1, min(len(lines), i+14)):
            print(f"      {j+1}: {lines[j].rstrip()}")
        found = True
        break

if not found:
    print("   ✗ 未找到发送逻辑")

# 4. 检查是否有ctrl_serial的打开逻辑
print("\n4. 检查 ctrl_serial 的打开逻辑:")
ctrl_open_lines = []
for i, line in enumerate(lines, 1):
    if 'ctrl_serial.open' in line:
        ctrl_open_lines.append((i, line.strip()))

if ctrl_open_lines:
    print(f"   ✓ 找到 {len(ctrl_open_lines)} 处打开 ctrl_serial:")
    for line_num, content in ctrl_open_lines[:3]:
        print(f"      Line {line_num}: {content}")
else:
    print("   ✗ 未找到打开逻辑")

print("\n" + "="*80)
print("诊断完成")
print("="*80)

# 5. 生成测试建议
print("\n【测试建议】")
print("运行GUI后，在控制台应该看到:")
print("1. [FLASH][DBG] set _requesting_jig_data=True, will request jig data every 500ms")
print("2. [JIG][DBG] Sent !GUI_REQUEST_DATA to ctrl_serial  (每500ms一次)")
print("\n如果看不到第1条，说明flash成功后的代码块没有执行")
print("如果看到第1条但看不到第2条，说明:")
print("   - ctrl_serial 没有打开")
print("   - 或者条件判断失败")
print("\n请粘贴完整的控制台输出，包括所有[DBG]标记的行")
print("="*80)
