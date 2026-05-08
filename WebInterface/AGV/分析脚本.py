#!/usr/bin/env python3
"""
检查JSON中特定位置的问题
"""

import json

def check_problem_position():
    """检查3754位置附近的问题"""
    with open('server_response_parsed.json', 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # 重新生成JSON字符串
    response_str = json.dumps(data, ensure_ascii=False)
    print(f"🔧 JSON总长度: {len(response_str)} 字符")
    
    # 检查3754位置附近
    pos = 3754
    start = max(0, pos - 20)
    end = min(len(response_str), pos + 20)
    
    print(f"\n🔧 检查位置 {pos} 附近:")
    print(f"位置 {pos}: 字符 '{response_str[pos] if pos < len(response_str) else '超出范围'}'")
    print(f"附近内容: ...{response_str[start:end]}...")
    
    # 显示上下文
    print(f"\n🔧 完整上下文 (位置 {max(0, pos-50)} 到 {min(len(response_str), pos+50)}):")
    context_start = max(0, pos - 50)
    context_end = min(len(response_str), pos + 50)
    print(response_str[context_start:context_end])
    
    # 检查这个位置是什么字段
    print(f"\n🔧 分析这个位置在哪个字段中...")
    
    # 统计到pos位置为止的字符
    partial = response_str[:pos+1]
    # 计算大括号和引号的数量
    open_braces = partial.count('{')
    close_braces = partial.count('}')
    open_brackets = partial.count('[')
    close_brackets = partial.count(']')
    quotes = partial.count('"')
    
    print(f"到位置 {pos} 为止:")
    print(f"  - 开大括号 {{: {open_braces}")
    print(f"  - 闭大括号 }}: {close_braces}")
    print(f"  - 开方括号 [: {open_brackets}")
    print(f"  - 闭方括号 ]: {close_brackets}")
    print(f"  - 双引号 \": {quotes} (应该是偶数)")
    
    if quotes % 2 != 0:
        print(f"⚠️  引号数量为奇数！可能在字符串中间有未转义的双引号")
    
    # 保存带有位置标记的版本
    with open('server_response_with_positions.txt', 'w', encoding='utf-8') as f:
        for i in range(0, len(response_str), 100):
            chunk = response_str[i:i+100]
            f.write(f"{i:4d}: {chunk}\n")
    
    print(f"\n📁 已保存带位置标记的响应到: server_response_with_positions.txt")

def find_problem_in_data():
    """在数据结构中查找问题"""
    with open('server_response_parsed.json', 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    print(f"\n🔧 检查数据结构中的潜在问题...")
    
    # 检查所有字符串字段
    def check_strings(obj, path=""):
        if isinstance(obj, dict):
            for key, value in obj.items():
                check_strings(value, f"{path}.{key}" if path else key)
        elif isinstance(obj, list):
            for i, item in enumerate(obj):
                check_strings(item, f"{path}[{i}]")
        elif isinstance(obj, str):
            # 检查字符串中的问题
            if '"' in obj and not obj.startswith('"') and not obj.endswith('"'):
                print(f"⚠️  字符串中包含未转义的双引号: {path} = {obj[:50]}...")
            if '\\' in obj:
                print(f"⚠️  字符串中包含反斜杠: {path} = {obj[:50]}...")
            if '\n' in obj or '\r' in obj or '\t' in obj:
                print(f"⚠️  字符串中包含控制字符: {path} = {obj[:50]}...")
    
    check_strings(data)
    
    # 特别检查agvs和tasks
    print(f"\n🔧 检查AGV数据:")
    for i, agv in enumerate(data.get('data', {}).get('agvs', [])):
        for key, value in agv.items():
            if isinstance(value, str) and ('"' in value or '\\' in value):
                print(f"  AGV-{agv.get('id', i)}.{key}: {value[:50]}...")
    
    print(f"\n🔧 检查任务数据:")
    for i, task in enumerate(data.get('data', {}).get('tasks', [])):
        for key, value in task.items():
            if isinstance(value, str) and ('"' in value or '\\' in value):
                print(f"  {task.get('id', f'task-{i}')}.{key}: {value[:50]}...")

if __name__ == "__main__":
    print("🔧 检查JSON响应中的问题")
    print("=" * 60)
    
    check_problem_position()
    find_problem_in_data()