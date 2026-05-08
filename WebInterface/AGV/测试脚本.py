#!/usr/bin/env python3
"""
检查服务端返回的完整数据
"""

import socket
import json
import sys

def get_complete_response():
    """获取完整的服务端响应"""
    print("🔧 开始获取服务端完整响应...")
    
    request = {
        "action": "GET_USER_DATA",
        "email": "demo@example.com"
    }
    
    try:
        # 连接TCP服务器
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        sock.connect(("127.0.0.1", 8888))
        
        # 发送请求
        request_str = json.dumps(request, ensure_ascii=False)
        print(f"🔧 发送请求: {request_str}")
        sock.send(request_str.encode('utf-8'))
        
        # 接收完整响应
        print("🔧 接收响应...")
        all_data = b""
        
        # 设置非阻塞模式
        sock.setblocking(False)
        
        import time
        start_time = time.time()
        timeout = 5  # 5秒超时
        
        while True:
            if time.time() - start_time > timeout:
                print("⚠️  接收超时")
                break
                
            try:
                chunk = sock.recv(65536)  # 64KB
                if chunk:
                    all_data += chunk
                    print(f"🔧 收到 {len(chunk)} 字节, 总计 {len(all_data)} 字节")
                    start_time = time.time()  # 重置超时计时
                else:
                    # 没有数据了
                    time.sleep(0.1)
            except BlockingIOError:
                # 没有数据可读
                time.sleep(0.1)
                continue
            except Exception as e:
                print(f"❌ 接收错误: {e}")
                break
        
        sock.close()
        
        if all_data:
            response_str = all_data.decode('utf-8', errors='ignore')
            print(f"\n✅ 收到完整响应")
            print(f"🔧 总长度: {len(response_str)} 字符")
            print(f"🔧 文件大小: {len(all_data)} 字节")
            
            # 保存到文件以便分析
            with open('server_response_raw.txt', 'w', encoding='utf-8') as f:
                f.write(response_str)
            print(f"📁 原始响应已保存到: server_response_raw.txt")
            
            # 分析响应结构
            print(f"\n🔧 分析响应...")
            print(f"🔧 前100字符: {response_str[:100]}")
            print(f"🔧 最后100字符: {response_str[-100:]}")
            
            # 检查常见问题
            if '\x00' in response_str:
                print("⚠️  响应中包含NULL字符")
            
            if len(response_str) > 10000:
                print(f"⚠️  响应很大 ({len(response_str)} 字符)，可能包含大量数据")
            
            # 尝试解析JSON
            print(f"\n🔧 尝试解析JSON...")
            try:
                response = json.loads(response_str)
                print(f"✅ JSON解析成功!")
                print(f"🔧 响应键: {list(response.keys())}")
                
                # 保存解析后的JSON
                with open('server_response_parsed.json', 'w', encoding='utf-8', errors='ignore') as f:
                    json.dump(response, f, ensure_ascii=False, indent=2)
                print(f"📁 解析后的JSON已保存到: server_response_parsed.json")
                
                # 检查数据大小
                if 'data' in response and isinstance(response['data'], dict):
                    data = response['data']
                    print(f"\n🔧 数据详情:")
                    print(f"🔧 数据键: {list(data.keys())}")
                    
                    for key in data:
                        if isinstance(data[key], list):
                            print(f"  - {key}: {len(data[key])} 项")
                        elif isinstance(data[key], dict):
                            print(f"  - {key}: 字典, {len(data[key])} 个键")
                        else:
                            value = str(data[key])[:50] + "..." if len(str(data[key])) > 50 else str(data[key])
                            print(f"  - {key}: {value}")
                
                return response_str
                
            except json.JSONDecodeError as e:
                print(f"❌ JSON解析失败: {e}")
                print(f"🔧 错误位置: 第{e.lineno}行, 第{e.colno}列")
                print(f"🔧 错误附近的内容:")
                
                # 显示错误位置附近的内容
                pos = e.pos
                start = max(0, pos - 50)
                end = min(len(response_str), pos + 50)
                print(f"位置 {pos} 附近: ...{response_str[start:end]}...")
                
                # 查找可能的截断位置
                print(f"\n🔧 查找可能的截断点...")
                for i in range(max(0, pos-10), min(len(response_str), pos+10)):
                    print(f"字符 {i}: '{response_str[i]}' (ASCII: {ord(response_str[i]) if i < len(response_str) else 'N/A'})")
                
                return response_str
                
        else:
            print("❌ 没有收到响应数据")
            return None
            
    except ConnectionRefusedError:
        print("❌ 连接被拒绝，请确保服务端已启动")
        return None
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return None

def analyze_response_file(filename):
    """分析保存的响应文件"""
    print(f"\n🔧 分析文件: {filename}")
    
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
        
        print(f"🔧 文件大小: {len(content)} 字符")
        
        # 检查文件编码
        import chardet
        with open(filename, 'rb') as f:
            raw_data = f.read()
        encoding_result = chardet.detect(raw_data)
        print(f"🔧 检测到的编码: {encoding_result}")
        
        # 检查特殊字符
        print(f"\n🔧 检查特殊字符...")
        for i, char in enumerate(content[:1000]):
            if ord(char) > 127 or ord(char) < 32:
                print(f"字符 {i}: '{char}' (ASCII: {ord(char)})")
        
        return content
    except Exception as e:
        print(f"❌ 读取文件失败: {e}")
        return None

if __name__ == "__main__":
    print("=" * 60)
    print("🔧 AGV服务端响应检查工具")
    print("=" * 60)
    
    # 首先确保服务端在运行
    print("🔧 确保服务端在运行...")
    
    # 获取响应
    response = get_complete_response()
    
    if response:
        print(f"\n✅ 检查完成")
        print(f"🔧 建议:")
        print(f"  1. 查看 server_response_raw.txt 文件")
        print(f"  2. 查看 server_response_parsed.json 文件 (如果解析成功)")
        print(f"  3. 检查响应中是否有特殊字符")
    else:
        print(f"\n❌ 未能获取响应")
    
    print("\n🔧 可选: 分析现有文件")
    if input("分析现有文件? (y/n): ").lower() == 'y':
        filename = input("输入文件名: ")
        analyze_response_file(filename)